"""websocket.py - a minimal RFC 6455 server, standard library only.

Not in the PRD's file list.  It exists because the alternative was a pip
dependency in the one place the PRD is emphatic about friction: the phone
client must work with "no app, no install", and a laptop-side server that
needs a working package index before the rover can move is the same problem
one step back.  Server-side WebSocket framing is about 150 lines; this is it.

Scope is deliberately small - exactly what W-3 and W-4 need:

  - the HTTP upgrade handshake
  - text and binary frames, including continuation frames
  - ping/pong (the browser sends pings; we answer, and we send our own)
  - close, with the status code echoed back

Not implemented, because nothing here uses them: extensions (permessage-deflate
would fight the JPEG payloads), and client-role masking.
"""

from __future__ import annotations

import base64
import hashlib
import os
import socket
import struct

GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"

OP_CONT = 0x0
OP_TEXT = 0x1
OP_BINARY = 0x2
OP_CLOSE = 0x8
OP_PING = 0x9
OP_PONG = 0xA

# A 480px JPEG is 20-60 KB; a megabyte is a generous ceiling that still stops a
# malformed length field from asking for a gigabyte of memory.
MAX_PAYLOAD = 1 << 20


class WebSocketError(Exception):
    pass


class WebSocketClosed(WebSocketError):
    pass


def accept_key(client_key: str) -> str:
    """The handshake's one piece of cryptography, such as it is."""
    digest = hashlib.sha1((client_key + GUID).encode("ascii")).digest()
    return base64.b64encode(digest).decode("ascii")


def handshake_response(headers: dict) -> bytes | None:
    """Build the 101 response, or None if this is not a WebSocket upgrade."""
    upgrade = headers.get("upgrade", "").lower()
    connection = headers.get("connection", "").lower()
    key = headers.get("sec-websocket-key")
    if "websocket" not in upgrade or "upgrade" not in connection or not key:
        return None
    return (
        "HTTP/1.1 101 Switching Protocols\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        f"Sec-WebSocket-Accept: {accept_key(key)}\r\n"
        "\r\n"
    ).encode("ascii")


class WebSocket:
    """One connection.  Not thread-safe for reads; writes are serialised by the
    caller holding ``send_lock``."""

    def __init__(self, sock: socket.socket):
        self.sock = sock
        self.closed = False
        self._buf = b""

    # -- low level ------------------------------------------------------
    def _recv_exact(self, n: int) -> bytes:
        while len(self._buf) < n:
            try:
                chunk = self.sock.recv(65536)
            except (TimeoutError, socket.timeout):
                raise
            except OSError as exc:
                raise WebSocketClosed(str(exc)) from exc
            if not chunk:
                raise WebSocketClosed("peer closed")
            self._buf += chunk
        out, self._buf = self._buf[:n], self._buf[n:]
        return out

    def _read_frame(self):
        head = self._recv_exact(2)
        fin = bool(head[0] & 0x80)
        opcode = head[0] & 0x0F
        masked = bool(head[1] & 0x80)
        length = head[1] & 0x7F

        if length == 126:
            (length,) = struct.unpack(">H", self._recv_exact(2))
        elif length == 127:
            (length,) = struct.unpack(">Q", self._recv_exact(8))

        if length > MAX_PAYLOAD:
            raise WebSocketError(f"frame of {length} bytes exceeds the {MAX_PAYLOAD} limit")

        mask = self._recv_exact(4) if masked else None
        payload = self._recv_exact(length) if length else b""

        if mask:
            payload = bytes(b ^ mask[i & 3] for i, b in enumerate(payload))
        return fin, opcode, payload

    # -- public ---------------------------------------------------------
    def receive(self):
        """Return ``(opcode, payload)`` for the next data message.

        Control frames are handled here and never surface to the caller;
        fragmented messages are reassembled.
        """
        frames = []
        first_opcode = None

        while True:
            fin, opcode, payload = self._read_frame()

            if opcode == OP_CLOSE:
                code = struct.unpack(">H", payload[:2])[0] if len(payload) >= 2 else 1000
                try:
                    self.send_frame(OP_CLOSE, struct.pack(">H", code))
                except OSError:
                    pass
                self.closed = True
                raise WebSocketClosed(f"close {code}")

            if opcode == OP_PING:
                self.send_frame(OP_PONG, payload)
                continue
            if opcode == OP_PONG:
                continue

            if opcode in (OP_TEXT, OP_BINARY):
                first_opcode = opcode
                frames = [payload]
            elif opcode == OP_CONT:
                frames.append(payload)
            else:
                raise WebSocketError(f"unknown opcode 0x{opcode:X}")

            if fin:
                data = b"".join(frames)
                if first_opcode == OP_TEXT:
                    return (OP_TEXT, data.decode("utf-8", "replace"))
                return (OP_BINARY, data)

    def send_frame(self, opcode: int, payload: bytes):
        if self.closed:
            raise WebSocketClosed("already closed")
        length = len(payload)
        header = bytearray([0x80 | opcode])
        if length < 126:
            header.append(length)
        elif length < (1 << 16):
            header.append(126)
            header += struct.pack(">H", length)
        else:
            header.append(127)
            header += struct.pack(">Q", length)
        # Server-to-client frames are never masked.
        self.sock.sendall(bytes(header) + payload)

    def send_text(self, text: str):
        self.send_frame(OP_TEXT, text.encode("utf-8"))

    def send_binary(self, data: bytes):
        self.send_frame(OP_BINARY, data)

    def ping(self, payload: bytes = b""):
        self.send_frame(OP_PING, payload or os.urandom(4))

    def close(self, code: int = 1000):
        if self.closed:
            return
        try:
            self.send_frame(OP_CLOSE, struct.pack(">H", code))
        except OSError:
            pass
        self.closed = True
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
