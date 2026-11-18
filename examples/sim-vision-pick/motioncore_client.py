"""Minimal client for ratavyr's public API: REST commands plus the WebSocket
telemetry stream. Standard library only (including a small RFC 6455
WebSocket client), so it runs on any PC with Python 3.10+.

The client is just another operator: every command goes through ratavyr and
the engine, which validate it (role, robot state, soft limits, collisions).
"""

from __future__ import annotations

import base64
import hashlib
import http.client
import json
import os
import random
import socket
import ssl
import struct
import threading
import time
import urllib.parse


class ApiError(Exception):
    def __init__(self, status: int, code: str, message: str, retry_after: float | None = None):
        super().__init__(f"{status} {code}: {message}")
        self.status, self.code, self.message, self.retry_after = status, code, message, retry_after


class Api:
    """REST calls. `base_url` like http://127.0.0.1:8080; `token` may be None
    when the server has authentication off (loopback only)."""

    def __init__(self, base_url: str, token: str | None, timeout: float = 10.0):
        self.base = base_url.rstrip("/")
        self.token = token or None
        self.timeout = timeout

    def request(self, method: str, path: str, body: dict | None = None, retries: int = 3):
        # http.client rather than urllib: urllib always sends
        # "Connection: close", and a server that rejects a POST before
        # reading its body may then reset the connection instead of
        # delivering the JSON error.
        data = None if body is None else json.dumps(body).encode()
        headers = {"Accept": "application/json"}
        if data is not None:
            headers["Content-Type"] = "application/json"
        if self.token:
            headers["Authorization"] = f"Bearer {self.token}"
        u = urllib.parse.urlsplit(self.base)
        conn_cls = http.client.HTTPSConnection if u.scheme == "https" else http.client.HTTPConnection
        for attempt in range(retries + 1):
            conn = conn_cls(u.netloc, timeout=self.timeout)
            try:
                conn.request(method, u.path.rstrip("/") + path, body=data, headers=headers)
                r = conn.getresponse()
                raw = r.read()
            except OSError as e:
                raise ApiError(0, "unreachable", f"{self.base}: {e}") from None
            finally:
                conn.close()
            if 200 <= r.status < 300:
                return json.loads(raw) if raw else None
            err = _api_error(r.status, r.reason, r.getheader("Retry-After"), raw)
            if err.code == "rate_limited" and attempt < retries:
                time.sleep(err.retry_after or 1.0)
                continue
            raise err

    def get(self, path: str):
        return self.request("GET", path)

    def command(self, name: str, params: dict | None = None):
        return self.request("POST", f"/api/commands/{name}", params or {})

    def ws_url(self) -> str:
        u = urllib.parse.urlsplit(self.base)
        scheme = "wss" if u.scheme == "https" else "ws"
        return f"{scheme}://{u.netloc}/api/ws"


def _api_error(status: int, reason: str, retry: str | None, raw: bytes) -> ApiError:
    try:
        retry_s = float(retry) if retry else None
    except ValueError:
        retry_s = None
    try:
        body = json.loads(raw or b"{}")["error"]
        return ApiError(status, body.get("code", "unknown"), body.get("message", ""), retry_s)
    except (ValueError, KeyError, TypeError):
        return ApiError(status, "http_error", reason or "", retry_s)


# ----------------------------------------------------------------- WebSocket

class _WebSocket:
    """Client side of RFC 6455: text frames in, close/ping handling."""

    def __init__(self, url: str, timeout: float = 5.0):
        u = urllib.parse.urlsplit(url)
        port = u.port or (443 if u.scheme == "wss" else 80)
        sock = socket.create_connection((u.hostname, port), timeout=timeout)
        if u.scheme == "wss":
            sock = ssl.create_default_context().wrap_socket(sock, server_hostname=u.hostname)
        self.sock = sock
        key = base64.b64encode(os.urandom(16)).decode()
        path = u.path + (f"?{u.query}" if u.query else "")
        sock.sendall((f"GET {path} HTTP/1.1\r\nHost: {u.netloc}\r\nUpgrade: websocket\r\n"
                      f"Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n").encode())
        head = b""
        while b"\r\n\r\n" not in head:
            chunk = sock.recv(1)
            if not chunk:
                raise ConnectionError("connection closed during WebSocket handshake")
            head += chunk
        lines = head.decode("latin-1").split("\r\n")
        if " 101 " not in lines[0] + " ":
            raise ConnectionError(f"WebSocket upgrade refused: {lines[0]}")
        accept = base64.b64encode(hashlib.sha1((key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode()).digest()).decode()
        hdrs = {k.strip().lower(): v.strip() for k, _, v in (l.partition(":") for l in lines[1:] if l)}
        if hdrs.get("sec-websocket-accept") != accept:
            raise ConnectionError("bad Sec-WebSocket-Accept")
        self.close_code: int | None = None

    def _read(self, n: int) -> bytes:
        buf = b""
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("WebSocket closed")
            buf += chunk
        return buf

    def _send(self, opcode: int, payload: bytes = b"") -> None:
        mask = os.urandom(4)
        n = len(payload)
        if n < 126:
            hdr = struct.pack("!BB", 0x80 | opcode, 0x80 | n)
        elif n < 65536:
            hdr = struct.pack("!BBH", 0x80 | opcode, 0x80 | 126, n)
        else:
            hdr = struct.pack("!BBQ", 0x80 | opcode, 0x80 | 127, n)
        self.sock.sendall(hdr + mask + bytes(b ^ mask[i % 4] for i, b in enumerate(payload)))

    def recv_text(self) -> str:
        parts: list[bytes] = []
        while True:
            b0, b1 = self._read(2)
            opcode, n = b0 & 0x0F, b1 & 0x7F
            if n == 126:
                n = struct.unpack("!H", self._read(2))[0]
            elif n == 127:
                n = struct.unpack("!Q", self._read(8))[0]
            mask = self._read(4) if b1 & 0x80 else None
            data = self._read(n)
            if mask:
                data = bytes(b ^ mask[i % 4] for i, b in enumerate(data))
            if opcode == 0x8:  # close
                self.close_code = struct.unpack("!H", data[:2])[0] if len(data) >= 2 else 1005
                try:
                    self._send(0x8, data[:2])
                except OSError:
                    pass
                raise ConnectionError(f"WebSocket closed by server (code {self.close_code})")
            if opcode == 0x9:  # ping
                self._send(0xA, data)
                continue
            if opcode == 0xA:  # pong
                continue
            parts.append(data)
            if b0 & 0x80:  # FIN
                return b"".join(parts).decode()

    def close(self) -> None:
        try:
            self._send(0x8, struct.pack("!H", 1000))
        except OSError:
            pass
        self.sock.close()


class Telemetry:
    """Keeps the latest telemetry from GET /api/ws in a background thread,
    reconnecting with a fresh ticket and backoff (0.5 s -> 10 s, +-20 %)."""

    STALE_AFTER_S = 1.5  # same threshold as sindrava

    def __init__(self, api: Api, auth_required: bool, log=print):
        self.api, self.auth_required, self.log = api, auth_required, log
        self._lock = threading.Condition()
        self._latest: dict | None = None
        self._at = 0.0
        self.engine_connected = False
        self._stop = threading.Event()
        self._ws: _WebSocket | None = None
        self._thread = threading.Thread(target=self._run, name="telemetry", daemon=True)
        self._thread.start()

    def _connect(self) -> _WebSocket:
        url = self.api.ws_url()
        if self.auth_required:
            ticket = self.api.request("POST", "/api/ws-tickets", {})["ticket"]
            url += "?ticket=" + urllib.parse.quote(ticket)
        return _WebSocket(url)

    def _run(self) -> None:
        backoff = 0.5
        while not self._stop.is_set():
            try:
                self._ws = self._connect()
                self._ws.sock.settimeout(5.0)
                backoff = 0.5
                while not self._stop.is_set():
                    msg = json.loads(self._ws.recv_text())
                    if msg.get("type") == "telemetry":
                        with self._lock:
                            self._latest, self._at = msg["data"], time.monotonic()
                            self._lock.notify_all()
                    elif msg.get("type") == "engine":
                        self.engine_connected = bool(msg["data"].get("connected"))
                        if not self.engine_connected:
                            self.log("telemetry: engine disconnected from ratavyr")
            except (OSError, ConnectionError, ApiError, ValueError) as e:
                if self._stop.is_set():
                    break
                self.log(f"telemetry: {e}; reconnecting in {backoff:.1f} s")
            finally:
                if self._ws:
                    self._ws.close()
                    self._ws = None
            self._stop.wait(backoff * random.uniform(0.8, 1.2))
            backoff = min(backoff * 2, 10.0)

    def latest(self) -> dict | None:
        """Latest telemetry, or None when there is none or it is stale."""
        with self._lock:
            if self._latest is None or time.monotonic() - self._at > self.STALE_AFTER_S:
                return None
            return self._latest

    def wait(self, predicate, timeout: float):
        """Block until predicate(telemetry) is true; returns that telemetry.
        Raises TimeoutError, or StaleTelemetry when data stops arriving."""
        deadline = time.monotonic() + timeout
        with self._lock:
            while True:
                now = time.monotonic()
                if self._latest is not None and now - self._at <= self.STALE_AFTER_S and predicate(self._latest):
                    return self._latest
                if self._latest is not None and now - self._at > self.STALE_AFTER_S:
                    raise StaleTelemetry(f"no telemetry for {now - self._at:.1f} s")
                if now >= deadline:
                    raise TimeoutError("timed out waiting for the robot")
                self._lock.wait(min(0.2, deadline - now))

    def close(self) -> None:
        self._stop.set()
        if self._ws:
            try:
                self._ws.sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass


class StaleTelemetry(Exception):
    pass
