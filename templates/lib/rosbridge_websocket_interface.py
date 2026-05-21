# encoding: utf-8
"""
rosbridge_websocket_interface — OpenC3 COSMOS Interface that connects to
rosbridge_suite's WebSocket transport.

WebSocket handles message framing natively (no NUL-byte terminator needed),
so this replaces the tcpip_client_interface + terminated_protocol stack that
was used with the old rosbridge_tcp transport.

Plugin.txt usage:

    INTERFACE TURTLEBOT_INT lib/rosbridge_websocket_interface.py \
        host.docker.internal 9090 10.0
      MAP_TARGET TURTLEBOT
      PROTOCOL READ_WRITE lib/rosbridge_subscribe_protocol.py lib/topics.txt

Start the rosbridge WebSocket server on the ROS2 host:

    ros2 launch rosbridge_server rosbridge_websocket_launch.xml
"""
from __future__ import annotations

from openc3.config.config_parser import ConfigParser
from openc3.interfaces.interface import Interface
from openc3.interfaces.stream_interface import StreamInterface


class RosbridgeWebsocketStream:
    """Thin WebSocket stream for rosbridge (no subprotocol, text frames)."""

    def __init__(self, url: str, write_timeout: float | None, read_timeout: float | None, connect_timeout: float = 5.0):
        self.url = url
        self.write_timeout = write_timeout
        self.read_timeout = read_timeout
        self.connect_timeout = connect_timeout
        self.connection = None

    def connect(self):
        from websockets.sync.client import connect
        self.connection = connect(
            self.url,
            open_timeout=self.connect_timeout,
            ping_interval=None,  # rosbridge doesn't reply to pings
        )

    def connected(self):
        return self.connection is not None

    def read(self):
        from websockets.exceptions import ConnectionClosedOK, ConnectionClosedError
        try:
            msg = self.connection.recv(self.read_timeout)
        except TimeoutError:
            return None
        except (ConnectionClosedOK, ConnectionClosedError):
            return None
        if isinstance(msg, str):
            return msg.encode("utf-8")
        return msg

    def write(self, data):
        if isinstance(data, (bytes, bytearray)):
            data = data.decode("utf-8")
        self.connection.send(data)

    def disconnect(self):
        if self.connection:
            try:
                self.connection.close()
            except Exception:
                pass
            self.connection = None


class RosbridgeWebsocketInterface(StreamInterface):
    """
    OpenC3 Interface that speaks to rosbridge via WebSocket.

    Parameters (from plugin.txt INTERFACE line):
        hostname     - Host running rosbridge_websocket (default: localhost)
        port         - WebSocket port (default: 9090)
        write_timeout - Seconds before aborting writes (default: 10.0)
        read_timeout  - Seconds before aborting reads (default: nil = block)
        protocol_type - Optional protocol type
        *protocol_args - Optional protocol args
    """

    def __init__(self, hostname="localhost", port="9090", write_timeout="10.0",
                 read_timeout="nil", protocol_type=None, *protocol_args):
        super().__init__(protocol_type, list(protocol_args))
        self.hostname = hostname
        self.port = int(port)
        self.write_timeout = ConfigParser.handle_none(write_timeout)
        if self.write_timeout is not None:
            self.write_timeout = float(self.write_timeout)
        self.read_timeout = ConfigParser.handle_none(read_timeout)
        if self.read_timeout is not None:
            self.read_timeout = float(self.read_timeout)

    def connection_string(self):
        return f"ws://{self.hostname}:{self.port} (R/W)"

    def connect(self):
        url = f"ws://{self.hostname}:{self.port}"
        self.stream = RosbridgeWebsocketStream(
            url, self.write_timeout, self.read_timeout
        )
        # Open the stream first, then run connect_reset on protocols
        # (which sends subscribe ops). We call Interface.connect() directly
        # instead of StreamInterface.connect() to avoid a second
        # self.stream.connect() call that would replace our connection.
        self.stream.connect()
        Interface.connect(self)

    def details(self):
        result = super().details()
        result["hostname"] = self.hostname
        result["port"] = self.port
        result["write_timeout"] = self.write_timeout
        result["read_timeout"] = self.read_timeout
        return result
