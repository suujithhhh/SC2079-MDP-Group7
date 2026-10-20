"""Android <-> RPi Bluetooth link: SPP/RFCOMM server, one line per message.

Uses the stdlib `socket.AF_BLUETOOTH` (no PyBluez). Listens on one RFCOMM
channel, accepts one tablet at a time and goes back to listening after a
disconnect, so the tablet can reconnect without restarting the program.
Inbound lines go to `inbox`; `send()` drops messages while disconnected.
"""

from __future__ import annotations

import queue
import socket
import threading
from typing import Callable


class LineFramer:
    """Split a byte stream into lines; accepts both \\n and \\r\\n."""

    def __init__(self, max_line: int = 64 * 1024) -> None:
        self._buffer = b""
        self._max_line = max_line

    def feed(self, data: bytes) -> list[str]:
        self._buffer += data
        lines = []
        while b"\n" in self._buffer:
            raw, self._buffer = self._buffer.split(b"\n", 1)
            text = raw.decode("utf-8", errors="replace").strip()
            if text:
                lines.append(text)
        if len(self._buffer) > self._max_line:
            self._buffer = b""
        return lines


class AndroidLink:
    def __init__(self, channel: int = 1, log: Callable[[str], None] = print) -> None:
        self.channel = channel
        self.inbox: queue.Queue[str] = queue.Queue()
        self.connected = threading.Event()
        self._log = log
        self._client: socket.socket | None = None
        self._send_lock = threading.Lock()
        self._stopped = threading.Event()

    def start(self) -> None:
        threading.Thread(target=self._serve, name="android-link", daemon=True).start()

    def stop(self) -> None:
        self._stopped.set()
        self._close_client()

    def send(self, message: str) -> bool:
        with self._send_lock:
            client = self._client
            if client is None:
                self._log(f"Android (offline, dropped) <- {message}")
                return False
            try:
                client.sendall(f"{message}\n".encode("utf-8"))
            except OSError as exc:
                self._log(f"Android send failed: {exc}")
                return False
        self._log(f"Android <- {message}")
        return True

    def _close_client(self) -> None:
        with self._send_lock:
            client, self._client = self._client, None
        self.connected.clear()
        if client is not None:
            try:
                client.close()
            except OSError:
                pass

    def _serve(self) -> None:
        server = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((socket.BDADDR_ANY, self.channel))
        server.listen(1)
        self._log(f"Bluetooth: listening on RFCOMM channel {self.channel}")
        while not self._stopped.is_set():
            client, address = server.accept()
            with self._send_lock:
                self._client = client
            self.connected.set()
            self._log(f"Bluetooth: tablet connected from {address[0]}")
            self.inbox.put("__CONNECTED__")
            self._read_until_closed(client)
            self._close_client()
            self._log("Bluetooth: tablet disconnected; listening again")
            self.inbox.put("__DISCONNECTED__")
        server.close()

    def _read_until_closed(self, client: socket.socket) -> None:
        framer = LineFramer()
        while not self._stopped.is_set():
            try:
                data = client.recv(1024)
            except OSError:
                return
            if not data:
                return
            for line in framer.feed(data):
                self._log(f"Android -> {line}")
                self.inbox.put(line)
