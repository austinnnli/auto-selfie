"""server.py - HTTPS + WebSocket server for the phone client (D2's other half).

C-6 IS THE RISK TO RETIRE FIRST, and this file is where it lives.  The page
must be a secure origin for camera access, and a secure origin cannot open a
plain WebSocket, so the laptop serves the page over HTTPS with a self-signed
certificate that Safari will warn about once, and the WebSocket runs as wss://
on the same origin.  Test this in the first hour of the build:

    python -m app.server --selftest     # no phone needed
    python -m app.server                # then open the printed URL on the phone

If Safari refuses camera access after accepting the warning, fall back to an
off-the-shelf RTSP streaming app and accept losing motion data and speech.

W-3 phone -> laptop, two message kinds on one socket:
    Frame   10-15 Hz  binary: 8-byte phone timestamp, then JPEG bytes
    Motion  50 Hz     JSON:   {t, rate_z, pitch, roll, ax, ay, az}

W-4 laptop -> phone, JSON:
    {"say": "a bit higher"}  {"capture": true}  {"quality": 0.6}  {"stop": true}
"""

from __future__ import annotations

import argparse
import json
import logging
import pathlib
import socket
import ssl
import struct
import threading
import time

from app import websocket as ws
from app.streams import Streams

log = logging.getLogger("server")

ROOT = pathlib.Path(__file__).resolve().parents[1]
CLIENT_DIR = ROOT / "client"


def lan_ip() -> str:
    """Best guess at the address the phone should connect to.

    No packet is actually sent - connect() on a UDP socket just picks the
    route, which is exactly the question being asked.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect(("10.255.255.255", 1))
        return sock.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        sock.close()


class PhoneServer:
    """Serves the client page and owns the phone's WebSocket.

    One phone at a time.  A second connection displaces the first, which is
    what you want when a page was reloaded and the old socket has not timed
    out yet.
    """

    def __init__(self, streams: Streams, config: dict, host: str = "0.0.0.0",
                 port: int = 8443, certfile=None, keyfile=None):
        self.streams = streams
        self.config = config
        self.host = host
        self.port = port
        self.certfile = pathlib.Path(certfile or ROOT / config.get("net", {}).get("cert", "certs/server.crt"))
        self.keyfile = pathlib.Path(keyfile or ROOT / config.get("net", {}).get("key", "certs/server.key"))

        self._sock: socket.socket | None = None
        self._ctx: ssl.SSLContext | None = None
        self._stop = threading.Event()
        self._client: ws.WebSocket | None = None
        self._send_lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self.connected_since: float | None = None
        self.captures: list[bytes] = []
        self.on_capture = None

    # -- lifecycle ------------------------------------------------------
    def start(self):
        if not self.certfile.exists() or not self.keyfile.exists():
            raise FileNotFoundError(
                f"no certificate at {self.certfile}. Run: python tools/gen_certs.py")

        self._ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        self._ctx.load_cert_chain(self.certfile, self.keyfile)

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self.host, self.port))
        self._sock.listen(8)
        self._sock.settimeout(0.5)

        self._thread = threading.Thread(target=self._accept_loop, name="https", daemon=True)
        self._thread.start()

        log.info("open this on the phone:  https://%s:%d/", lan_ip(), self.port)
        log.info("Safari will warn about the self-signed certificate once - "
                 "accept it, or the camera will never start (C-6)")
        return self

    def stop(self):
        self._stop.set()
        if self._client:
            try:
                self._client.close()
            except Exception:
                pass
        if self._sock:
            self._sock.close()
        if self._thread:
            self._thread.join(timeout=2.0)

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.stop()

    # -- accept ---------------------------------------------------------
    def _accept_loop(self):
        while not self._stop.is_set():
            try:
                raw, addr = self._sock.accept()
            except (socket.timeout, TimeoutError):
                continue
            except OSError:
                break
            threading.Thread(target=self._handle, args=(raw, addr),
                             name=f"conn-{addr[1]}", daemon=True).start()

    def _handle(self, raw: socket.socket, addr):
        try:
            conn = self._ctx.wrap_socket(raw, server_side=True)
        except (ssl.SSLError, OSError) as exc:
            # Entirely normal: Safari probes, then connects for real.
            log.debug("tls handshake from %s failed: %s", addr[0], exc)
            raw.close()
            return

        try:
            conn.settimeout(10.0)
            request = self._read_request(conn)
            if request is None:
                return
            method, path, headers, body = request

            upgrade = ws.handshake_response(headers)
            if upgrade is not None:
                conn.sendall(upgrade)
                conn.settimeout(30.0)
                self._serve_socket(ws.WebSocket(conn), addr)
                return

            if method == "POST" and path.startswith("/capture"):
                self._handle_capture(conn, headers, body)
                return

            self._serve_file(conn, path)
        except (ws.WebSocketClosed, OSError):
            pass
        except Exception:
            log.exception("connection from %s failed", addr[0])
        finally:
            try:
                conn.close()
            except OSError:
                pass

    @staticmethod
    def _read_request(conn):
        buf = b""
        while b"\r\n\r\n" not in buf:
            try:
                chunk = conn.recv(8192)
            except (socket.timeout, TimeoutError, OSError):
                return None
            if not chunk:
                return None
            buf += chunk
            if len(buf) > 64 * 1024:
                return None

        head, _, rest = buf.partition(b"\r\n\r\n")
        lines = head.decode("latin-1").split("\r\n")
        method, path, _version = (lines[0].split() + ["", "", ""])[:3]
        headers = {}
        for line in lines[1:]:
            if ":" in line:
                key, _, value = line.partition(":")
                headers[key.strip().lower()] = value.strip()

        body = rest
        length = int(headers.get("content-length", 0) or 0)
        while len(body) < length:
            chunk = conn.recv(65536)
            if not chunk:
                break
            body += chunk
        return (method, path, headers, body)

    # -- static files ---------------------------------------------------
    def _serve_file(self, conn, path: str):
        name = path.split("?", 1)[0].lstrip("/") or "index.html"
        target = (CLIENT_DIR / name).resolve()
        if not str(target).startswith(str(CLIENT_DIR.resolve())) or not target.is_file():
            conn.sendall(b"HTTP/1.1 404 Not Found\r\nContent-Length: 9\r\n"
                         b"Connection: close\r\n\r\nnot found")
            return

        data = target.read_bytes()
        ctype = {".html": "text/html; charset=utf-8", ".js": "text/javascript",
                 ".css": "text/css", ".json": "application/json",
                 ".ico": "image/x-icon"}.get(target.suffix, "application/octet-stream")
        conn.sendall(
            f"HTTP/1.1 200 OK\r\nContent-Type: {ctype}\r\n"
            f"Content-Length: {len(data)}\r\n"
            # The page must not be cached: during tuning it changes constantly
            # and a stale copy on the phone is a baffling half-hour.
            "Cache-Control: no-store\r\n"
            "Connection: close\r\n\r\n".encode("ascii") + data)

    def _handle_capture(self, conn, headers, body):
        """C-5: the phone POSTs the full-resolution grab back here."""
        self.captures.append(body)
        log.info("capture received: %.1f kB", len(body) / 1024)
        if self.on_capture:
            try:
                self.on_capture(body)
            except Exception:
                log.exception("capture handler failed")
        conn.sendall(b"HTTP/1.1 200 OK\r\nContent-Length: 2\r\n"
                     b"Connection: close\r\n\r\nok")

    # -- the phone socket -----------------------------------------------
    def _serve_socket(self, sock: ws.WebSocket, addr):
        if self._client is not None:
            # A reloaded page: drop the stale socket rather than serving two.
            try:
                self._client.close()
            except Exception:
                pass
        self._client = sock
        self.connected_since = time.monotonic()
        log.info("phone connected from %s", addr[0])

        try:
            self._ping_pong_rounds(sock)
            while not self._stop.is_set():
                opcode, payload = sock.receive()
                if opcode == ws.OP_BINARY:
                    self._on_binary(payload)
                else:
                    self._on_text(payload)
        except (ws.WebSocketClosed, OSError, ValueError) as exc:
            log.info("phone disconnected: %s", exc)
        finally:
            if self._client is sock:
                self._client = None
                self.connected_since = None

    def _on_binary(self, payload: bytes):
        """W-3 frame: 8-byte phone timestamp, then JPEG bytes."""
        if len(payload) < 9:
            return
        (t_ms,) = struct.unpack("<d", payload[:8])
        self.streams.push_frame(payload[8:], t_ms / 1000.0)

    def _on_text(self, text: str):
        try:
            msg = json.loads(text)
        except ValueError:
            return
        kind = msg.get("type")

        if kind == "motion":
            # W-3 motion, at 50 Hz.  Same clock as the frames.
            self.streams.push_motion({
                "t": float(msg.get("t", 0)) / 1000.0,
                "rate_z": float(msg.get("rate_z", 0.0)),
                "pitch": float(msg.get("pitch", 0.0)),
                "roll": float(msg.get("roll", 0.0)),
                "ax": float(msg.get("ax", 0.0)),
                "ay": float(msg.get("ay", 0.0)),
                "az": float(msg.get("az", 0.0)),
            })
        elif kind == "hello":
            # C-1.4: the negotiated resolution, so the laptop can compute
            # normalised coordinates correctly.  Constraints fall back silently,
            # so what the phone ASKED for is not what it got.
            self.streams.hello = msg
            log.info("phone: %sx%s video, working width %s, %s",
                     msg.get("videoWidth"), msg.get("videoHeight"),
                     msg.get("workWidth"), msg.get("ua", "")[:40])
        elif kind == "pong":
            self.streams.clock.add_round(
                float(msg.get("t_send", 0)) / 1000.0,
                float(msg.get("t_phone", 0)) / 1000.0,
                time.monotonic())
        elif kind == "transcript":
            text_value = msg.get("text", "")
            log.info("heard: %r", text_value)
            self.streams.push_transcript(text_value)
        elif kind == "tap":
            self.streams.push_transcript("__tap__")

    def _ping_pong_rounds(self, sock, rounds: int = 9):
        """P-2's clock offset, measured at startup.

        Nine rounds, median kept.  It takes about a second and it removes the
        single most confusing class of bug in the whole build.
        """
        for _ in range(rounds):
            sock.send_text(json.dumps({"type": "ping", "t_send": time.monotonic() * 1000.0}))
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                opcode, payload = sock.receive()
                if opcode == ws.OP_TEXT:
                    self._on_text(payload)
                    if json.loads(payload).get("type") == "pong":
                        break
                else:
                    self._on_binary(payload)
        if self.streams.clock.locked:
            log.info("clock offset %.3f s, rtt %.0f ms",
                     self.streams.clock.offset, self.streams.clock.rtt * 1000)

    # -- W-4 laptop -> phone --------------------------------------------
    def send(self, message: dict) -> bool:
        sock = self._client
        if sock is None:
            return False
        try:
            with self._send_lock:
                sock.send_text(json.dumps(message))
            return True
        except (ws.WebSocketClosed, OSError):
            self._client = None
            return False

    def say(self, text: str) -> bool:
        """C-4: the phone is 3-4 m from the subject and facing them, so it is
        the right speaker.  The laptop chooses the words."""
        return self.send({"say": text})

    def capture(self) -> bool:
        return self.send({"capture": True})

    def set_quality(self, quality: float) -> bool:
        return self.send({"quality": round(float(quality), 2)})

    def stop_client(self) -> bool:
        return self.send({"stop": True})

    @property
    def connected(self) -> bool:
        return self._client is not None


# ---------------------------------------------------------------------------
# C-6 self-test
# ---------------------------------------------------------------------------

def selftest(port: int = 8443) -> int:
    """Prove the HTTPS + wss path works before a phone is anywhere near it.

    Connects to the server over TLS, does the WebSocket handshake by hand,
    sends a motion message and a frame, and checks they arrive.  If this
    passes and Safari still will not open the camera, the problem is the
    certificate's trust prompt, not the transport.
    """
    import base64 as _b64
    import os as _os

    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
    cfg = json.loads((ROOT / "config.json").read_text())
    streams = Streams()
    server = PhoneServer(streams, cfg, host="127.0.0.1", port=port)
    server.start()
    time.sleep(0.3)

    ctx = ssl.create_default_context()
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE

    # 1. the page itself
    with ctx.wrap_socket(socket.create_connection(("127.0.0.1", port)),
                         server_hostname="localhost") as conn:
        conn.sendall(b"GET / HTTP/1.1\r\nHost: localhost\r\nConnection: close\r\n\r\n")
        page = b""
        while True:
            chunk = conn.recv(65536)
            if not chunk:
                break
            page += chunk
    assert b"200 OK" in page, "the client page did not load over HTTPS"
    print(f"ok   client page served over HTTPS ({len(page)} bytes)")

    # 2. the socket
    key = _b64.b64encode(_os.urandom(16)).decode()
    conn = ctx.wrap_socket(socket.create_connection(("127.0.0.1", port)),
                           server_hostname="localhost")
    conn.sendall(
        f"GET /ws HTTP/1.1\r\nHost: localhost\r\nUpgrade: websocket\r\n"
        f"Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\n"
        f"Sec-WebSocket-Version: 13\r\n\r\n".encode())
    resp = conn.recv(4096)
    assert b"101" in resp, f"upgrade refused: {resp[:120]!r}"
    assert ws.accept_key(key).encode() in resp, "Sec-WebSocket-Accept is wrong"
    print("ok   websocket upgrade accepted over TLS (wss)")

    client = _ClientSocket(conn)

    # The server opens with clock ping-pong; answer it.
    for _ in range(9):
        opcode, payload = client.receive()
        msg = json.loads(payload)
        assert msg.get("type") == "ping"
        client.send_text(json.dumps({"type": "pong", "t_send": msg["t_send"],
                                     "t_phone": time.monotonic() * 1000.0}))
    time.sleep(0.2)
    assert streams.clock.locked, "clock offset never locked"
    print(f"ok   clock offset measured over 9 rounds "
          f"(rtt {streams.clock.rtt * 1000:.1f} ms)")

    client.send_text(json.dumps({"type": "hello", "videoWidth": 1920,
                                 "videoHeight": 1080, "workWidth": 480}))
    client.send_text(json.dumps({"type": "motion", "t": time.monotonic() * 1000.0,
                                 "rate_z": 1.5, "pitch": -3.0, "roll": 0.2}))
    jpeg = b"\xff\xd8\xff" + b"x" * 4000
    client.send_binary(struct.pack("<d", time.monotonic() * 1000.0) + jpeg)
    time.sleep(0.4)

    assert streams.hello.get("videoWidth") == 1920, "hello did not arrive"
    assert streams.motion.count >= 1, "motion did not arrive"
    frame = streams.frames.peek()
    assert frame is not None and frame.jpeg == jpeg, "frame did not arrive intact"
    print(f"ok   hello, motion and a {len(jpeg)}-byte frame all arrived intact")

    assert server.say("a bit higher"), "could not send a say message"
    opcode, payload = client.receive()
    assert json.loads(payload)["say"] == "a bit higher"
    print("ok   laptop -> phone speech message delivered (W-4)")

    client.close()
    server.stop()
    print("\nC-6 transport verified end to end: HTTPS page + wss on one origin.")
    return 0


class _ClientSocket(ws.WebSocket):
    """Client role: outgoing frames must be masked (RFC 6455 5.3).  Only used
    by the self-test, which is the only place this process is a client."""

    def send_frame(self, opcode: int, payload: bytes):
        import os as _os
        length = len(payload)
        header = bytearray([0x80 | opcode])
        if length < 126:
            header.append(0x80 | length)
        elif length < (1 << 16):
            header.append(0x80 | 126)
            header += struct.pack(">H", length)
        else:
            header.append(0x80 | 127)
            header += struct.pack(">Q", length)
        mask = _os.urandom(4)
        header += mask
        masked = bytes(b ^ mask[i & 3] for i, b in enumerate(payload))
        self.sock.sendall(bytes(header) + masked)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--port", type=int, default=8443)
    parser.add_argument("--selftest", action="store_true",
                        help="prove the HTTPS + wss path works without a phone (C-6)")
    args = parser.parse_args()

    if args.selftest:
        raise SystemExit(selftest(args.port))

    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
    cfg = json.loads((ROOT / "config.json").read_text())
    streams = Streams()
    with PhoneServer(streams, cfg, port=args.port):
        try:
            while True:
                time.sleep(5)
                log.info("streams: %s", streams.health())
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
