# openc3-cosmos-ros2

**Scaffolding generator** for OpenC3 COSMOS plugins that bridge to ROS2 via
[`rosbridge_suite`](https://github.com/RobotWebTools/rosbridge_suite) (WebSocket transport).

This repo is **not itself a COSMOS plugin**. It generates a plugin tailored to
a specific ROS2 robot (e.g. `openc3-cosmos-ros2-turtlesim`).

## What it does

1. **Discovers** your live ROS2 graph: topics, services, actions, parameters.
2. **Generates** OpenC3 COSMOS definitions:
   - `cmd_tlm/tlm.txt` — one TELEMETRY packet per topic
   - `cmd_tlm/cmd.txt` — COMMAND per topic publish / service / action / param
   - `lib/topics.txt` — subscription list consumed by the interface
3. **Vendors** a WebSocket interface (`rosbridge_websocket_interface.py`) and
   subscribe-on-connect protocol (`rosbridge_subscribe_protocol.py`) into the
   new plugin.

All telemetry uses `JsonAccessor`. Inbound rosbridge `publish` frames are
routed via `APPEND_ID_ITEM TOPIC` keyed on `$.topic`. Outbound commands are
pre-shaped as rosbridge op envelopes. Units are emitted when available from
interface comments or well-known field names.

## Architecture

```
┌──────── ROS2 host (laptop / robot) ────────┐    ┌──── COSMOS docker ──────────┐
│ ROS2 nodes (DDS)                           │    │                             │
│   ▼                                        │    │ <TARGET> plugin             │
│ rosbridge_websocket  ws://0.0.0.0:9090 ────┼──▶ │  rosbridge_websocket_iface  │
│ (ros2 launch rosbridge_server              │ WS │  + rosbridge_subscribe…     │
│  rosbridge_websocket_launch.xml)           │    │  cmd_tlm/{cmd,tlm}.txt      │
└────────────────────────────────────────────┘    │  lib/topics.txt             │
                                                   └─────────────────────────────┘
```

## Prerequisites

On the ROS2 host:

```bash
sudo apt install ros-${ROS_DISTRO}-rosbridge-suite
source /opt/ros/${ROS_DISTRO}/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

On the machine running the generator:

- Python 3.8+
- ROS2 environment sourced (`ros2` CLI on `$PATH`)

## Bootstrap a new plugin

```bash
git clone https://github.com/clayandgen/openc3-cosmos-ros2.git
cd openc3-cosmos-ros2

# Scaffold AND generate cmd/tlm against a live graph:
./bin/scaffold-ros2-plugin \
  --target TURTLESIM \
  --out ../openc3-cosmos-ros2-turtlesim \
  --discover
```

`--force` regenerates target files but never touches Rakefile, gemspec, or README.md.

## Regenerate cmd/tlm later

```bash
./helpers/discover_ros2.sh \
  --target TURTLESIM \
  --out-dir ../openc3-cosmos-ros2-turtlesim/targets/TURTLESIM/cmd_tlm \
  --topics-file ../openc3-cosmos-ros2-turtlesim/targets/TURTLESIM/lib/topics.txt
```

Pass `--dump-manifest path.json` to save, or `--manifest path.json` to regenerate from a saved manifest.

## Layout

```
bin/scaffold-ros2-plugin           # bootstrap a new plugin
helpers/discover_ros2.sh           # wrapper: ros2 CLI + generator
helpers/ros2_to_cosmos.py          # parses ros2 graph, emits cmd/tlm/topics
templates/
  plugin.txt.tmpl                  # plugin.txt scaffold (__TARGET__ substitution)
  lib/rosbridge_websocket_interface.py   # COSMOS Interface (vendored into each plugin)
  lib/rosbridge_subscribe_protocol.py    # COSMOS Protocol (vendored into each plugin)
```

## License

MIT. See [LICENSE.txt](LICENSE.txt).
