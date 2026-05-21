# encoding: utf-8
"""
rosbridge_subscribe_protocol — OpenC3 COSMOS Protocol that issues
`{"op":"subscribe", ...}` envelopes on connect for every topic listed in
`topics_file`.

All read/write data passes through unchanged — this protocol exists purely
so a fresh connection automatically subscribes to the topic set without
anyone having to push subscribe ops by hand from a startup script.

Plugin.txt usage:

    INTERFACE TURTLEBOT_INT lib/rosbridge_websocket_interface.py \\
        host.docker.internal 9090 10.0
      MAP_TARGET TURTLEBOT
      PROTOCOL READ_WRITE lib/rosbridge_subscribe_protocol.py lib/topics.txt 100

Start the rosbridge WebSocket server on the ROS2 host:

    ros2 launch rosbridge_server rosbridge_websocket_launch.xml
"""
from __future__ import annotations

import json
import os
from typing import Optional

from openc3.interfaces.protocols.protocol import Protocol
from openc3.utilities.logger import Logger


class RosbridgeSubscribeProtocol(Protocol):
    """Send rosbridge `subscribe` ops on connect; pass data through otherwise."""

    def __init__(self, topics_file: str, throttle_rate: str = "0",
                 allow_empty_data: Optional[str] = None):
        super().__init__(allow_empty_data)
        self.topics_file = topics_file
        self.throttle_rate = int(throttle_rate)  # ms between messages per topic

    # ------------------------------------------------------------------
    # Lifecycle hooks
    # ------------------------------------------------------------------

    def connect_reset(self) -> None:
        """Called by the Interface whenever it (re)connects."""
        super().connect_reset()
        try:
            self._send_subscriptions()
        except Exception as e:
            Logger.error(f"rosbridge_subscribe: failed to send subscriptions: {e}")

    # ------------------------------------------------------------------
    # Data passthrough
    # ------------------------------------------------------------------

    def read_data(self, data, extra=None):
        if not data or len(data) == 0:
            return "STOP", extra
        # Only pass through topic publish messages; drop service responses,
        # status pings, and other rosbridge ops that have no matching TLM packet.
        try:
            text = data.decode("utf-8") if isinstance(data, (bytes, bytearray)) else data
            msg = json.loads(text)
            if msg.get("op") != "publish":
                return "STOP", extra
        except (json.JSONDecodeError, UnicodeDecodeError):
            return "STOP", extra
        return data, extra

    def write_data(self, data, extra=None):
        return data, extra

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    def _send_subscriptions(self) -> None:
        if not self.interface:
            return
        path = self._resolve_topics_file(self.topics_file)
        if not path:
            Logger.warn(
                f"rosbridge_subscribe: topics file {self.topics_file!r} not found; "
                "no subscriptions issued"
            )
            return

        count = 0
        with open(path, "r") as f:
            for raw in f:
                line = raw.strip()
                if not line or line.startswith("#"):
                    continue
                parts = line.split()
                op = {"op": "subscribe", "topic": parts[0]}
                if len(parts) > 1:
                    op["type"] = parts[1]
                if self.throttle_rate > 0:
                    op["throttle_rate"] = self.throttle_rate
                payload = json.dumps(op).encode("utf-8")
                self.interface.write_interface(payload)
                count += 1
        Logger.info(f"rosbridge_subscribe: subscribed to {count} topic(s) from {path}")

    @staticmethod
    def _resolve_topics_file(configured: str) -> Optional[str]:
        if os.path.isabs(configured) and os.path.isfile(configured):
            return configured
        file_dir = os.path.dirname(__file__)
        parent_dir = os.path.dirname(file_dir)  # target root (parent of lib/)
        for base in (os.getcwd(), file_dir, parent_dir):
            cand = os.path.join(base, configured)
            if os.path.isfile(cand):
                return cand
        return None
