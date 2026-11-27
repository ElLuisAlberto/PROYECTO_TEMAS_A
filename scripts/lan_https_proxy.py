#!/usr/bin/env python3
"""Serve the LAN UI over TLS and forward WebSocket upgrades to localhost.

The gateway itself is deliberately bound to loopback.  This proxy replaces,
never appends to, the forwarded client IP header.  The host and its local
processes remain part of the trusted boundary.
"""

from __future__ import annotations

import argparse
import json
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import select
import socket
import ssl
import threading


class BoundedHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address, handler, *, directory, backend_port, max_clients, session_mode="simulation"):
        super().__init__(address, handler)
        self.directory = directory
        self.backend_port = backend_port
        self.session_mode = session_mode
        self.slots = threading.BoundedSemaphore(max_clients)

    def process_request(self, request, client_address):
        if not self.slots.acquire(blocking=False):
            self.shutdown_request(request)
            return
        try:
            super().process_request(request, client_address)
        except BaseException:
            self.slots.release()
            raise

    def process_request_thread(self, request, client_address):
        try:
            # Do TLS handshake in a bounded worker; an idle client must not
            # block accept() and deny service to all other clients.
            request.settimeout(5)
            secured = self.tls_context.wrap_socket(request, server_side=True)
            super().process_request_thread(secured, client_address)
        except (OSError, ssl.SSLError):
            request.close()
        finally:
            self.slots.release()


class Handler(SimpleHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=kwargs.pop("directory", None) or args[2].directory, **kwargs)

    def log_message(self, format, *args):
        # Avoid leaking paths/query strings and credentials into logs.
        pass

    def do_GET(self):
        if self.path == "/ws":
            self._websocket()
        elif self.headers.get("Upgrade"):
            self.send_error(404)
        elif self.path == "/session-mode":
            body = json.dumps({"mode": self.server.session_mode}).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        else:
            super().do_GET()

    def _websocket(self):
        if self.headers.get("Upgrade", "").lower() != "websocket":
            self.send_error(400, "WebSocket upgrade required")
            return
        if "upgrade" not in self.headers.get("Connection", "").lower():
            self.send_error(400, "Connection upgrade required")
            return
        # Only forward WebSocket handshake headers, not untrusted forwarding
        # headers or arbitrary headers that could influence the backend.
        names = ("Origin", "Sec-WebSocket-Key", "Sec-WebSocket-Version",
                 "Sec-WebSocket-Protocol", "Sec-WebSocket-Extensions")
        lines = ["GET /ws HTTP/1.1", f"Host: 127.0.0.1:{self.server.backend_port}",
                 "Upgrade: websocket", "Connection: Upgrade",
                 f"X-Forwarded-For: {self.client_address[0]}"]
        for name in names:
            for value in self.headers.get_all(name, []):
                if "\r" in value or "\n" in value:
                    self.send_error(400)
                    return
                lines.append(f"{name}: {value}")
        try:
            with socket.create_connection(("127.0.0.1", self.server.backend_port), timeout=5) as upstream:
                upstream.sendall(("\r\n".join(lines) + "\r\n\r\n").encode("ascii"))
                response = bytearray()
                while b"\r\n\r\n" not in response and len(response) < 16384:
                    chunk = upstream.recv(4096)
                    if not chunk:
                        return
                    response.extend(chunk)
                if b"\r\n\r\n" not in response:
                    return
                self.connection.sendall(response)
                if not response.startswith(b"HTTP/1.1 101 "):
                    return
                self.close_connection = True
                self._relay(upstream)
        except (OSError, UnicodeError, ssl.SSLError):
            self.close_connection = True

    def _relay(self, upstream):
        for sock in (upstream, self.connection):
            sock.settimeout(5)
            try:
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            except OSError:
                pass
        peers = (self.connection, upstream)
        while True:
            # SSLSocket may already have decrypted bytes buffered internally.
            pending = self.connection.pending()
            readable, _, _ = select.select(peers, (), (), 0 if pending else 1)
            if pending and self.connection not in readable:
                readable.append(self.connection)
            for source in readable:
                data = source.recv(65536)
                if not data:
                    return
                (upstream if source is self.connection else self.connection).sendall(data)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", required=True)
    parser.add_argument("--port", type=int, default=8443)
    parser.add_argument("--backend-port", type=int, default=8765)
    parser.add_argument("--directory", required=True)
    parser.add_argument("--cert", required=True)
    parser.add_argument("--key", required=True)
    parser.add_argument("--max-clients", type=int, default=32)
    parser.add_argument("--session-mode", choices=("simulation", "physical", "read-only"), default="simulation")
    args = parser.parse_args()
    if args.max_clients < 1:
        parser.error("max-clients must be positive")
    server = BoundedHTTPServer((args.host, args.port), Handler, directory=args.directory,
                               backend_port=args.backend_port, max_clients=args.max_clients, session_mode=args.session_mode)
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    context.minimum_version = ssl.TLSVersion.TLSv1_2
    context.load_cert_chain(args.cert, args.key)
    server.tls_context = context
    server.serve_forever()


if __name__ == "__main__":
    main()
