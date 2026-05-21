# openc3-cosmos-ros2

**Scaffolding generator** for OpenC3 COSMOS plugins that bridge to ROS2 via
[`rosbridge_suite`](https://github.com/RobotWebTools/rosbridge_suite).

This repo is **not itself a COSMOS plugin**. It contains the tools you use to
*generate* a plugin tailored to a specific ROS2 robot. The output is a new
COSMOS plugin directory (e.g. `openc3-cosmos-ros2-turtlebot`) that you build
and install with the standard `rake build VERSION=X.Y.Z` flow.

## What it does

1. **Discovers** your live ROS2 graph: topics, services, actions, parameters.
2. **Generates** OpenC3 COSMOS definitions:
   - `targets/<TARGET>/cmd_tlm/tlm.txt` — one TELEMETRY packet per topic
   - `targets/<TARGET>/cmd_tlm/cmd.txt` — COMMAND per topic publish / service / action / param
   - `targets/<TARGET>/lib/topics.txt` — subscription list consumed by the interface
3. **Vendors** a small subscribe-on-connect protocol (`rosbridge_subscribe_protocol.py`)
   into the new plugin. The plugin uses OpenC3's built-in `tcpip_client_interface`
   plus `terminated_protocol` for `\0` framing — no custom socket code, no extra
   Python deps.

All telemetry uses `JsonAccessor`. Inbound `rosbridge` `publish` frames are
routed to the right packet via an `APPEND_ID_ITEM TOPIC` keyed on `$.topic` —
no per-message decoders needed. Outbound commands are pre-shaped as rosbridge
op envelopes by the generator, so the interface just forwards them.

## Architecture

```
┌──────────── ROS2 host (laptop / robot) ────────────┐    ┌──── COSMOS docker ─────────┐
│ ROS2 nodes (DDS)                                   │    │                            │
│   ▼                                                │    │ <TARGET> plugin            │
│ rosbridge_tcp     tcp://0.0.0.0:9090  ─────────────┼──▶ │  tcpip_client_interface    │
│ (ros2 launch rosbridge_server                      │TCP │  + terminated_protocol(\0) │
│  rosbridge_tcp.launch.xml)                         │    │  + rosbridge_subscribe…    │
└────────────────────────────────────────────────────┘    │  cmd_tlm/{cmd,tlm}.txt     │
                                                          │  lib/topics.txt            │
                                                          └────────────────────────────┘
```

DDS doesn't cross the Docker bridge cleanly on macOS/Windows. `rosbridge_tcp`
gives you a stable JSON-over-TCP boundary that COSMOS consumes with its
built-in interface plus the standard `terminated_protocol` for null-byte
framing — no DDS inside the container, no extra Python deps.

## Prerequisites

On the ROS2 host:

```bash
sudo apt install ros-${ROS_DISTRO}-rosbridge-suite
source /opt/ros/${ROS_DISTRO}/setup.bash
# Start the bridge (default port 9090):
ros2 launch rosbridge_server rosbridge_tcp.launch.xml
# Bring up your robot stack so `ros2 topic list` shows what you expect.
```

On the machine running the generator (typically the same laptop):

- Python 3.8+
- ROS2 environment sourced (`ros2` CLI on `$PATH`)

## Bootstrap a new plugin

```bash
git clone https://github.com/clayandgen/openc3-cosmos-ros2.git
cd openc3-cosmos-ros2

# Scaffold an empty plugin directory:
./bin/scaffold-ros2-plugin \
  --target TURTLEBOT \
  --out ../openc3-cosmos-ros2-turtlebot

# Or scaffold AND generate cmd/tlm against a live graph in one shot:
./bin/scaffold-ros2-plugin \
  --target TURTLEBOT \
  --out ../openc3-cosmos-ros2-turtlebot \
  --discover
```

`--target` is the COSMOS target name (uppercase by convention). It's used as
both the gem suffix (`openc3-cosmos-ros2-<lowercased>`) and the directory
name under `targets/`.

## Regenerate cmd/tlm later

```bash
./helpers/discover_ros2.sh \
  --target TURTLEBOT \
  --out-dir ../openc3-cosmos-ros2-turtlebot/targets/TURTLEBOT/cmd_tlm \
  --topics-file ../openc3-cosmos-ros2-turtlebot/targets/TURTLEBOT/lib/topics.txt
```

Pass `--manifest path.json` to skip the live ROS2 query and regenerate from a
saved manifest, or `--dump-manifest path.json` to capture one.

## Layout

```
bin/scaffold-ros2-plugin       # bootstrap a new plugin from the template
helpers/discover_ros2.sh       # wrapper that runs ros2 CLI + generator
helpers/ros2_to_cosmos.py      # parses ros2 graph, emits cmd/tlm/topics
templates/
  plugin.txt.tmpl                       # plugin.txt scaffold (substitutes __TARGET__)
  lib/rosbridge_subscribe_protocol.py   # COSMOS Protocol, copied into each plugin
  targets/__TARGET__/                   # cmd.txt/tlm.txt/topics.txt placeholders
```

## License

MIT. See [LICENSE.txt](LICENSE.txt).
