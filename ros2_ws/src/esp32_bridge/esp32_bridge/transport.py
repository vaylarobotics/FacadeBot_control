import json
import socket


class Esp32TransportError(Exception):
    """Raised for any failure talking to the ESP32 (not connected, timeout,
    connection dropped, malformed reply). Callers only need to catch this
    one type instead of every possible socket/JSON exception."""


class Esp32Transport:
    """Owns the TCP connection to the ESP32 and speaks its newline-delimited
    JSON protocol (see esp32_firmware/main.py). No ROS2 imports here on
    purpose - business logic must never touch a socket directly, so this
    class is the single place that does."""

    def __init__(self, host: str, port: int, timeout_sec: float) -> None:
        self._host = host
        self._port = port
        self._timeout_sec = timeout_sec
        self._sock: socket.socket | None = None
        self._sock_file = None

    def connect(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(self._timeout_sec)  # must be set before connect()
        try:
            sock.connect((self._host, self._port))
            sock_file = sock.makefile("rb")
            # ESP32 sends {"status": "ready"} unsolicited on connect
            # (esp32_firmware/main.py:132) - consume it here so it can't be
            # mistaken for the reply to the first real command.
            greeting = sock_file.readline()
            if not greeting:
                raise Esp32TransportError("ESP32 closed the connection during handshake")
            json.loads(greeting.strip())
        except (OSError, ValueError) as exc:
            sock.close()
            raise Esp32TransportError(f"failed to connect to ESP32: {exc}") from exc

        self._sock = sock
        self._sock_file = sock_file

    def disconnect(self) -> None:
        if self._sock is not None:
            self._sock.close()
        self._sock = None
        self._sock_file = None

    def send_command(self, command: dict) -> None:
        if self._sock is None:
            raise Esp32TransportError("send_command called while not connected")
        try:
            self._sock.sendall((json.dumps(command) + "\n").encode())
        except OSError as exc:
            raise Esp32TransportError(f"failed to send command to ESP32: {exc}") from exc

    def read_response(self) -> dict:
        if self._sock_file is None:
            raise Esp32TransportError("read_response called while not connected")
        try:
            raw = self._sock_file.readline()
        except OSError as exc:
            raise Esp32TransportError(f"failed to read response from ESP32: {exc}") from exc

        if not raw:
            raise Esp32TransportError("ESP32 closed the connection")

        try:
            return json.loads(raw.strip())
        except ValueError as exc:
            raise Esp32TransportError(f"invalid JSON from ESP32: {exc}") from exc
