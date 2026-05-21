# encoding: utf-8
"""
rosbridge_subscribe_protocol — OpenC3 COSMOS Protocol that issues
`{"op":"subscribe", ...}` envelopes on connect for every topic listed in
`topics_file`.

Layered above `terminated_protocol` (which handles `\\0` framing for the
`rosbridge_tcp` transport). All read/write data passes through unchanged —
this protocol exists purely so a fresh TCP connection automatically
subscribes to the topic set without anyone having to push subscribe ops by
hand from a startup script.

Plugin.txt usage:

    INTERFACE TURTLEBOT_INT openc3/interfaces/tcpip_client_interface.py \\
        host.docker.internal 9090 9090 10.0 nil BURST
      MAP_TARGET TURTLEBOT
      PROTOCOL READ_WRITE openc3/interfaces/protocols/terminated_protocol.py \\
        0x00 0x00 True 0 nil False
      PROTOCOL READ_WRITE lib/rosbridge_subscribe_protocol.py lib/topics.txt

Start the laptop bridge with the TCP launch (NOT the websocket one):

    ros2 launch rosbridge_server rosbridge_tcp.launch.xml
"""
from __future__ import annotations

import json
import os
from typing import Optional

from openc3.interfaces.protocols.protocol import Protocol
from openc3.utilities.logger import Logger


class RosbridgeSubscribeProtocol(Protocol):
    """Send rosbridge `subscribe` ops on connect; pass data through otherwise."""

    def __init__(self, topics_file: str, allow_empty_data: Optional[str] = None):
        super().__init__(allow_empty_data)
        self.topics_file = topics_file

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
        # Terminated protocol below already stripped the \0; nothing to do.
        return data, extra

    def write_data(self, data, extra=None):
        # Commands generated from cmd.txt are already valid rosbridge JSON.
        # Terminated protocol below will append the \0 framing byte.
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
                payload = json.dumps(op).encode("utf-8")
                # Re-enter the write chain from the top. The terminated protocol
                # below us will append \0; this protocol's own write_data is a
                # passthrough so nothing else mutates the payload.
                self.interface.write_interface(payload)
                count += 1
        Logger.info(f"rosbridge_subscribe: subscribed to {count} topic(s) from {path}")

    @staticmethod
    def _resolve_topics_file(configured: str) -> Optional[str]:
        if os.path.isabs(configured) and os.path.isfile(configured):
            return configured
        for base in (os.getcwd(), os.path.dirname(__file__)):
            cand = os.path.join(base, configured)
            if os.path.isfile(cand):
                return cand
        return None
