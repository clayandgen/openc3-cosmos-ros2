#!/usr/bin/env python3
"""
ROS2 to OpenC3 COSMOS Command/Telemetry Definition Generator

Enumerates ROS2 topics, services, actions, and parameters via the `ros2` CLI,
then emits cmd.txt + tlm.txt definitions using JsonAccessor.

Usage:
  ros2_to_cosmos.py [--out-dir DIR] [--manifest-only PATH]

Requires a sourced ROS2 environment (`source /opt/ros/<distro>/setup.bash`)
so the `ros2` CLI is on PATH and can see your DDS domain.
"""
from __future__ import annotations

import argparse
import json
import re
import shutil
import subprocess
import sys
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Optional


# ROS2 primitive type -> COSMOS (cosmos_type, bit_size)
TYPE_MAP = {
    "bool":    ("UINT", 8),
    "byte":    ("UINT", 8),
    "char":    ("UINT", 8),
    "int8":    ("INT", 8),
    "uint8":   ("UINT", 8),
    "int16":   ("INT", 16),
    "uint16":  ("UINT", 16),
    "int32":   ("INT", 32),
    "uint32":  ("UINT", 32),
    "int64":   ("INT", 64),
    "uint64":  ("UINT", 64),
    "float32": ("FLOAT", 32),
    "float64": ("FLOAT", 64),
    "string":  ("STRING", 0),
    "wstring": ("STRING", 0),
}

# Topics never useful as TLM
SKIP_TOPICS = {"/parameter_events", "/rosout"}

# Services to skip (per-node parameter plumbing)
SKIP_SERVICE_SUFFIXES = (
    "/describe_parameters",
    "/get_parameter_types",
    "/get_parameters",
    "/list_parameters",
    "/set_parameters",
    "/set_parameters_atomically",
    "/get_type_description",
)


def sanitize(name: str) -> str:
    """COSMOS-safe identifier: alnum + underscore, uppercase, no leading digit."""
    s = re.sub(r"[^A-Za-z0-9_]", "_", name).strip("_").upper()
    if s and s[0].isdigit():
        s = "T_" + s
    return s or "ITEM"


@dataclass
class Field:
    name: str           # original ROS name (snake_case)
    ros_type: str       # e.g. float32, std_msgs/Header
    is_array: bool = False
    array_size: Optional[int] = None
    is_complex: bool = False  # non-primitive (nested msg)


@dataclass
class Interface:
    """Parsed representation of a .msg / .srv / .action"""
    type_name: str
    fields: list[Field] = field(default_factory=list)
    # for srv/action additional sections
    response_fields: list[Field] = field(default_factory=list)
    feedback_fields: list[Field] = field(default_factory=list)


def run(cmd: list[str]) -> str:
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode != 0:
        raise RuntimeError(f"{' '.join(cmd)} -> {res.stderr.strip()}")
    return res.stdout


# ---------------------------------------------------------------------------
# Interface parser
# ---------------------------------------------------------------------------

FIELD_RE = re.compile(
    r"""^
    \s*                              # leading indent (will be filtered)
    (?P<type>[A-Za-z_][\w/]*)        # type name (e.g. float32, std_msgs/Header)
    (?:<=\d+)?                       # bounded string upper bound (string<=10)
    (?P<arr>\[(?:<=)?\d*\])?         # array suffix [], [3], [<=5]
    \s+
    (?P<name>[A-Za-z_]\w*)           # field name
    (?:\s+(?P<value>.+))?$           # optional default / constant value
    """,
    re.VERBOSE,
)


def parse_interface_show(text: str, sections: int = 1) -> Interface:
    """Parse `ros2 interface show TYPE` output.

    sections=1 for msg, 2 for srv (req---resp), 3 for action (goal---result---feedback).
    Nested msgs are flattened: we only keep top-level fields per section. Nested
    indented lines (4 spaces or tab) are skipped — their data still rides as a
    JSON sub-object under the parent field.
    """
    iface = Interface(type_name="")
    buckets: list[list[Field]] = [[] for _ in range(sections)]
    idx = 0
    for raw in text.splitlines():
        line = raw.rstrip()
        if not line.strip():
            continue
        if line.strip() == "---":
            idx = min(idx + 1, sections - 1)
            continue
        # Skip nested (indented) lines — keep only top-level fields
        if line.startswith((" ", "\t")):
            continue
        # Strip inline comments
        stripped = line.split("#", 1)[0].strip()
        if not stripped:
            continue
        m = FIELD_RE.match(stripped)
        if not m:
            continue
        type_name = m.group("type")
        name = m.group("name")
        value = m.group("value")
        arr = m.group("arr")
        # Constants (UPPER_CASE with value) — skip; not over-the-wire
        if value is not None and name.isupper():
            continue
        is_array = arr is not None
        array_size = None
        if is_array:
            inner = arr[1:-1]
            if inner.isdigit():
                array_size = int(inner)
        is_complex = "/" in type_name
        buckets[idx].append(Field(
            name=name,
            ros_type=type_name,
            is_array=is_array,
            array_size=array_size,
            is_complex=is_complex,
        ))
    iface.fields = buckets[0]
    if sections >= 2:
        iface.response_fields = buckets[1]
    if sections >= 3:
        iface.feedback_fields = buckets[2]
    return iface


def get_interface(type_name: str, sections: int) -> Optional[Interface]:
    try:
        text = run(["ros2", "interface", "show", type_name])
    except RuntimeError as e:
        print(f"  warn: cannot show {type_name}: {e}", file=sys.stderr)
        return None
    iface = parse_interface_show(text, sections=sections)
    iface.type_name = type_name
    return iface


# ---------------------------------------------------------------------------
# Discovery
# ---------------------------------------------------------------------------

def list_topics() -> list[tuple[str, str]]:
    """Return list of (topic_name, msg_type)."""
    out = run(["ros2", "topic", "list", "-t"])
    result = []
    for line in out.splitlines():
        line = line.strip()
        if not line:
            continue
        # Format: /topic_name [pkg/msg/Type]
        m = re.match(r"(\S+)\s+\[([^\]]+)\]", line)
        if m:
            topic, mtype = m.group(1), m.group(2)
            if topic in SKIP_TOPICS:
                continue
            result.append((topic, mtype))
    return result


def list_services() -> list[tuple[str, str]]:
    out = run(["ros2", "service", "list", "-t"])
    result = []
    for line in out.splitlines():
        line = line.strip()
        if not line:
            continue
        m = re.match(r"(\S+)\s+\[([^\]]+)\]", line)
        if m:
            svc, stype = m.group(1), m.group(2)
            if any(svc.endswith(sfx) for sfx in SKIP_SERVICE_SUFFIXES):
                continue
            result.append((svc, stype))
    return result


def list_actions() -> list[tuple[str, str]]:
    try:
        out = run(["ros2", "action", "list", "-t"])
    except RuntimeError:
        return []
    result = []
    for line in out.splitlines():
        line = line.strip()
        if not line:
            continue
        m = re.match(r"(\S+)\s+\[([^\]]+)\]", line)
        if m:
            result.append((m.group(1), m.group(2)))
    return result


def list_params() -> list[tuple[str, str, str]]:
    """Return (node, param_name, type) for every parameter."""
    try:
        nodes_out = run(["ros2", "node", "list"])
    except RuntimeError:
        return []
    params: list[tuple[str, str, str]] = []
    for node in nodes_out.splitlines():
        node = node.strip()
        if not node:
            continue
        try:
            plist = run(["ros2", "param", "list", node])
        except RuntimeError:
            continue
        for p in plist.splitlines():
            p = p.strip()
            if not p or p.endswith(":"):
                continue
            ptype = "string"
            try:
                desc = run(["ros2", "param", "describe", node, p])
                m = re.search(r"Type:\s*(\S+)", desc)
                if m:
                    ptype = m.group(1).lower()
            except RuntimeError:
                pass
            params.append((node, p, ptype))
    return params


# ---------------------------------------------------------------------------
# COSMOS generation
# ---------------------------------------------------------------------------

def cosmos_type_for(f: Field) -> tuple[str, int]:
    """Return (cosmos_type, bit_size). For complex/array types use JsonAccessor with size 0."""
    if f.is_array or f.is_complex:
        # Arrays and nested messages ride through JsonAccessor as sub-trees;
        # bit size 0 means "let the accessor figure it out".
        return ("STRING", 0)
    return TYPE_MAP.get(f.ros_type, ("STRING", 0))


def default_value(f: Field):
    if f.is_array:
        return []
    if f.is_complex:
        return {}
    t = f.ros_type
    if t == "bool":
        return False
    if t in ("string", "wstring"):
        return ""
    if t.startswith("float"):
        return 0.0
    return 0


def emit_tlm_packet(out, target: str, topic: str, iface: Interface) -> None:
    """One TELEMETRY packet per topic. Identified by $.topic so the protocol can
    route inbound rosbridge `publish` ops to the right packet purely by ID."""
    pkt_name = sanitize(topic)
    out.write(f"# {'=' * 76}\n")
    out.write(f"# {topic}  [{iface.type_name}]\n")
    out.write(f"# {'=' * 76}\n")
    out.write(
        f'TELEMETRY {target} {pkt_name} BIG_ENDIAN "ROS2 topic {topic}"\n'
    )
    out.write("  ACCESSOR JsonAccessor\n")

    template = {"op": "publish", "topic": topic, "msg": {f.name: default_value(f) for f in iface.fields}}
    out.write(f"  TEMPLATE '{json.dumps(template)}'\n")

    # Routing identifier: rosbridge publishes carry "topic"
    out.write(f'  APPEND_ID_ITEM TOPIC 0 STRING "{topic}" "Source topic"\n')
    out.write("    KEY $.topic\n")
    out.write(f'  APPEND_ITEM OP 0 STRING "rosbridge op"\n')
    out.write("    KEY $.op\n")

    for f in iface.fields:
        ctype, bits = cosmos_type_for(f)
        out.write(
            f'  APPEND_ITEM {sanitize(f.name)} {bits} {ctype} "{f.ros_type}{"[]" if f.is_array else ""}"\n'
        )
        out.write(f"    KEY $.msg.{f.name}\n")
    out.write("\n")


def emit_cmd_for_publish(out, target: str, topic: str, iface: Interface) -> None:
    """Optional: every topic also exposed as publishable command (lets COSMOS push)."""
    pkt_name = sanitize(topic) + "_PUB"
    out.write(f"# {'=' * 76}\n")
    out.write(f"# publish {topic}  [{iface.type_name}]\n")
    out.write(f"# {'=' * 76}\n")
    out.write(
        f'COMMAND {target} {pkt_name} BIG_ENDIAN "Publish to {topic}"\n'
    )
    out.write("  ACCESSOR JsonAccessor\n")
    template = {"op": "publish", "topic": topic, "msg": {f.name: default_value(f) for f in iface.fields}}
    out.write(f"  TEMPLATE '{json.dumps(template)}'\n")

    out.write(f'  APPEND_PARAMETER _OP 0 STRING "publish" "rosbridge op"\n')
    out.write("    KEY $.op\n")
    out.write(f'  APPEND_PARAMETER _TOPIC 0 STRING "{topic}" "Target topic"\n')
    out.write("    KEY $.topic\n")
    for f in iface.fields:
        ctype, bits = cosmos_type_for(f)
        write_param_line(out, f, ctype, bits, key_prefix="$.msg.")
    out.write("\n")


def emit_cmd_for_service(out, target: str, service: str, iface: Interface) -> None:
    pkt_name = sanitize(service) + "_SRV"
    out.write(f"# {'=' * 76}\n")
    out.write(f"# call_service {service}  [{iface.type_name}]\n")
    out.write(f"# {'=' * 76}\n")
    out.write(
        f'COMMAND {target} {pkt_name} BIG_ENDIAN "Call service {service}"\n'
    )
    out.write("  ACCESSOR JsonAccessor\n")
    template = {
        "op": "call_service",
        "service": service,
        "args": {f.name: default_value(f) for f in iface.fields},
    }
    out.write(f"  TEMPLATE '{json.dumps(template)}'\n")

    out.write(f'  APPEND_PARAMETER _OP 0 STRING "call_service" "rosbridge op"\n')
    out.write("    KEY $.op\n")
    out.write(f'  APPEND_PARAMETER _SERVICE 0 STRING "{service}" "Service name"\n')
    out.write("    KEY $.service\n")
    for f in iface.fields:
        ctype, bits = cosmos_type_for(f)
        write_param_line(out, f, ctype, bits, key_prefix="$.args.")
    out.write("\n")


def emit_cmd_for_action(out, target: str, action: str, iface: Interface) -> None:
    pkt_name = sanitize(action) + "_ACT"
    out.write(f"# {'=' * 76}\n")
    out.write(f"# send_action_goal {action}  [{iface.type_name}]\n")
    out.write(f"# {'=' * 76}\n")
    out.write(
        f'COMMAND {target} {pkt_name} BIG_ENDIAN "Send action goal {action}"\n'
    )
    out.write("  ACCESSOR JsonAccessor\n")
    template = {
        "op": "send_action_goal",
        "action": action,
        "action_type": iface.type_name,
        "args": {f.name: default_value(f) for f in iface.fields},
    }
    out.write(f"  TEMPLATE '{json.dumps(template)}'\n")

    out.write(f'  APPEND_PARAMETER _OP 0 STRING "send_action_goal" "rosbridge op"\n')
    out.write("    KEY $.op\n")
    out.write(f'  APPEND_PARAMETER _ACTION 0 STRING "{action}" "Action name"\n')
    out.write("    KEY $.action\n")
    out.write(f'  APPEND_PARAMETER _ACTION_TYPE 0 STRING "{iface.type_name}" "Action type"\n')
    out.write("    KEY $.action_type\n")
    for f in iface.fields:
        ctype, bits = cosmos_type_for(f)
        write_param_line(out, f, ctype, bits, key_prefix="$.args.")
    out.write("\n")


def emit_cmd_for_param_get(out, target: str, node: str, pname: str) -> None:
    pkt = sanitize(f"{node}{pname}") + "_GET"
    out.write(
        f'COMMAND {target} {pkt} BIG_ENDIAN "Get param {node} {pname}"\n'
    )
    out.write("  ACCESSOR JsonAccessor\n")
    svc = f"{node}/get_parameters"
    template = {"op": "call_service", "service": svc, "args": {"names": [pname]}}
    out.write(f"  TEMPLATE '{json.dumps(template)}'\n")
    out.write(f'  APPEND_PARAMETER _OP 0 STRING "call_service" "op"\n')
    out.write("    KEY $.op\n")
    out.write(f'  APPEND_PARAMETER _SERVICE 0 STRING "{svc}" "service"\n')
    out.write("    KEY $.service\n")
    out.write(f'  APPEND_PARAMETER _NAMES 0 STRING "" "param names JSON array (set via TEMPLATE)"\n')
    out.write("    KEY $.args.names\n")
    out.write("\n")


def emit_cmd_for_param_set(out, target: str, node: str, pname: str, ptype: str) -> None:
    pkt = sanitize(f"{node}{pname}") + "_SET"
    out.write(
        f'COMMAND {target} {pkt} BIG_ENDIAN "Set param {node} {pname}"\n'
    )
    out.write("  ACCESSOR JsonAccessor\n")
    svc = f"{node}/set_parameters"
    template = {
        "op": "call_service",
        "service": svc,
        "args": {"parameters": [{"name": pname, "value": {"type": 0}}]},
    }
    out.write(f"  TEMPLATE '{json.dumps(template)}'\n")
    out.write(f'  APPEND_PARAMETER _OP 0 STRING "call_service" "op"\n')
    out.write("    KEY $.op\n")
    out.write(f'  APPEND_PARAMETER _SERVICE 0 STRING "{svc}" "service"\n')
    out.write("    KEY $.service\n")
    out.write(f'  APPEND_PARAMETER VALUE 0 STRING "" "Param value (auto-typed: {ptype})"\n')
    out.write("    KEY $.args.parameters[0].value.string_value\n")
    out.write("\n")


def write_param_line(out, f: Field, ctype: str, bits: int, key_prefix: str) -> None:
    name = sanitize(f.name)
    desc = f"{f.ros_type}{'[]' if f.is_array else ''}"
    if ctype == "STRING":
        # Arrays / nested msgs ride as sub-JSON; literal default stays empty,
        # the real default lives in the TEMPLATE.
        out.write(f'  APPEND_PARAMETER {name} 0 STRING "" "{desc}"\n')
    elif ctype == "FLOAT":
        out.write(f'  APPEND_PARAMETER {name} {bits} FLOAT MIN MAX 0.0 "{desc}"\n')
    elif ctype == "INT":
        out.write(f'  APPEND_PARAMETER {name} {bits} INT MIN MAX 0 "{desc}"\n')
    else:
        maxv = (1 << bits) - 1
        out.write(f'  APPEND_PARAMETER {name} {bits} UINT 0 {maxv} 0 "{desc}"\n')
    out.write(f"    KEY {key_prefix}{f.name}\n")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def discover() -> dict:
    print("Discovering ROS2 graph...", file=sys.stderr)
    topics = list_topics()
    services = list_services()
    actions = list_actions()
    params = list_params()
    print(f"  topics: {len(topics)}, services: {len(services)}, "
          f"actions: {len(actions)}, params: {len(params)}", file=sys.stderr)

    manifest = {"topics": [], "services": [], "actions": [], "params": []}
    for topic, mtype in topics:
        iface = get_interface(mtype, sections=1)
        if not iface:
            continue
        manifest["topics"].append({"name": topic, "type": mtype, "iface": asdict(iface)})
    for svc, stype in services:
        iface = get_interface(stype, sections=2)
        if not iface:
            continue
        manifest["services"].append({"name": svc, "type": stype, "iface": asdict(iface)})
    for act, atype in actions:
        iface = get_interface(atype, sections=3)
        if not iface:
            continue
        manifest["actions"].append({"name": act, "type": atype, "iface": asdict(iface)})
    for node, pname, ptype in params:
        manifest["params"].append({"node": node, "name": pname, "type": ptype})
    return manifest


def _to_iface(d: dict) -> Interface:
    return Interface(
        type_name=d["type_name"],
        fields=[Field(**f) for f in d["fields"]],
        response_fields=[Field(**f) for f in d.get("response_fields", [])],
        feedback_fields=[Field(**f) for f in d.get("feedback_fields", [])],
    )


def generate(manifest: dict, target: str, out_dir: Path, topics_file: Optional[Path] = None) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    tlm_path = out_dir / "tlm.txt"
    cmd_path = out_dir / "cmd.txt"

    if topics_file is not None:
        topics_file.parent.mkdir(parents=True, exist_ok=True)
        with topics_file.open("w") as tf:
            tf.write("# Auto-generated subscription list. One topic per line; type after whitespace.\n")
            for t in manifest["topics"]:
                tf.write(f"{t['name']} {t['type']}\n")
        print(f"Wrote {topics_file}", file=sys.stderr)

    with tlm_path.open("w") as tlm:
        tlm.write("# ROS2 Telemetry Definitions — auto-generated\n")
        tlm.write("# Each topic = one TELEMETRY packet, routed by $.topic ID item.\n\n")
        for t in manifest["topics"]:
            emit_tlm_packet(tlm, target, t["name"], _to_iface(t["iface"]))

    with cmd_path.open("w") as cmd:
        cmd.write("# ROS2 Command Definitions — auto-generated\n")
        cmd.write("# Topics -> publish, services -> call_service, actions -> send_action_goal.\n\n")
        for t in manifest["topics"]:
            emit_cmd_for_publish(cmd, target, t["name"], _to_iface(t["iface"]))
        for s in manifest["services"]:
            emit_cmd_for_service(cmd, target, s["name"], _to_iface(s["iface"]))
        for a in manifest["actions"]:
            emit_cmd_for_action(cmd, target, a["name"], _to_iface(a["iface"]))
        for p in manifest["params"]:
            emit_cmd_for_param_get(cmd, target, p["node"], p["name"])
            emit_cmd_for_param_set(cmd, target, p["node"], p["name"], p["type"])

    print(f"Wrote {tlm_path}", file=sys.stderr)
    print(f"Wrote {cmd_path}", file=sys.stderr)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--target", default="ROS2", help="COSMOS target name (default: ROS2)")
    ap.add_argument(
        "--out-dir",
        type=Path,
        required=True,
        help="Output directory for cmd.txt/tlm.txt (typically <plugin>/targets/<TARGET>/cmd_tlm)",
    )
    ap.add_argument(
        "--manifest",
        type=Path,
        help="Read manifest JSON from this file instead of querying ros2 CLI",
    )
    ap.add_argument(
        "--dump-manifest",
        type=Path,
        help="Also write discovered manifest JSON to this path",
    )
    ap.add_argument(
        "--topics-file",
        type=Path,
        help="Also write subscription list here (consumed by rosbridge_interface.py)",
    )
    args = ap.parse_args()

    if args.manifest:
        manifest = json.loads(args.manifest.read_text())
    else:
        if not shutil.which("ros2"):
            print("error: `ros2` CLI not found on PATH. Source your ROS2 setup first.", file=sys.stderr)
            return 2
        manifest = discover()

    if args.dump_manifest:
        args.dump_manifest.write_text(json.dumps(manifest, indent=2))
        print(f"Wrote manifest {args.dump_manifest}", file=sys.stderr)

    generate(manifest, args.target, args.out_dir, topics_file=args.topics_file)
    return 0


if __name__ == "__main__":
    sys.exit(main())
