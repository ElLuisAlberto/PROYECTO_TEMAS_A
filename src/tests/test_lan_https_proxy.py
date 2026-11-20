"""End-to-end TLS and trusted proxy smoke test using only loopback."""
from __future__ import annotations

import json
import os
from pathlib import Path
import socket
import ssl
import subprocess
import tempfile
import threading
import unittest


ROOT = Path(__file__).resolve().parents[2]


class HttpsProxyTest(unittest.TestCase):
    def test_partial_ca_is_not_overwritten(self):
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp)
            (base / "certs").mkdir(mode=0o700)
            (base / "certs/ca").mkdir(mode=0o700)
            (base / "run").mkdir(mode=0o700)
            marker = base / "certs/ca/ca.crt"
            marker.write_text("incomplete", encoding="ascii")
            result = subprocess.run(["python3", str(ROOT / "scripts/prepare_lan_security.py"),
                                     "--ip", "127.0.0.1", "--run-dir", str(base / "run"),
                                     "--cert-dir", str(base / "certs")],
                                    stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual(marker.read_text(), "incomplete")
            self.assertFalse((base / "run/credentials.json").exists())

    def test_tls_static_websocket_and_forwarded_ip(self):
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp)
            (base / "certs").mkdir(mode=0o700)
            (base / "run").mkdir(mode=0o700)
            (base / "www").mkdir()
            (base / "www" / "index.html").write_text("PROXY-OK", encoding="utf-8")
            subprocess.run(["python3", str(ROOT / "scripts/prepare_lan_security.py"),
                            "--ip", "127.0.0.1", "--run-dir", str(base / "run"),
                            "--cert-dir", str(base / "certs")], check=True,
                           stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
            credentials = json.loads((base / "run/credentials.json").read_text())
            self.assertEqual(set(credentials), {"operators", "observers", "admins"})
            self.assertEqual(os.stat(base / "run/credentials.json").st_mode & 0o777, 0o600)
            old_ca = (base / "certs/ca/ca.crt").read_bytes()
            old_cert = (base / "certs/server-127.0.0.1/server.crt").read_bytes()
            (base / "run2").mkdir(mode=0o700)
            subprocess.run(["python3", str(ROOT / "scripts/prepare_lan_security.py"),
                            "--ip", "127.0.0.1", "--run-dir", str(base / "run2"),
                            "--cert-dir", str(base / "certs")], check=True,
                           stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
            self.assertEqual(old_ca, (base / "certs/ca/ca.crt").read_bytes())
            self.assertEqual(old_cert, (base / "certs/server-127.0.0.1/server.crt").read_bytes())
            self.assertNotEqual(credentials, json.loads((base / "run2/credentials.json").read_text()))

            backend = socket.socket()
            backend.bind(("127.0.0.1", 0))
            backend.listen(1)
            backend.settimeout(10)
            headers = []
            def serve_backend():
                with backend.accept()[0] as peer:
                    peer.settimeout(10)
                    request = b""
                    while b"\r\n\r\n" not in request:
                        request += peer.recv(4096)
                    headers.append(request)
                    peer.sendall(b"HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n\r\n")
                    peer.sendall(peer.recv(64))
            thread = threading.Thread(target=serve_backend, daemon=True)
            thread.start()
            with socket.socket() as reservation:
                reservation.bind(("127.0.0.1", 0))
                proxy_port = reservation.getsockname()[1]
            process = subprocess.Popen(["python3", str(ROOT / "scripts/lan_https_proxy.py"),
                "--host", "127.0.0.1", "--port", str(proxy_port),
                "--backend-port", str(backend.getsockname()[1]), "--directory", str(base / "www"),
                "--cert", str(base / "certs/server-127.0.0.1/server.crt"),
                "--key", str(base / "certs/server-127.0.0.1/server.key")],
                stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
            try:
                import time
                context = ssl.create_default_context(cafile=str(base / "certs/ca/ca.crt"))
                deadline = time.monotonic() + 10
                while True:
                    try:
                        raw = socket.create_connection(("127.0.0.1", proxy_port), timeout=1)
                        break
                    except OSError:
                        if process.poll() is not None or time.monotonic() > deadline:
                            self.fail("Proxy TLS did not start")
                        time.sleep(0.05)
                with context.wrap_socket(raw, server_hostname="127.0.0.1") as client:
                    client.sendall(b"GET / HTTP/1.1\r\nHost: 127.0.0.1\r\nConnection: close\r\n\r\n")
                    response = b""
                    while part := client.recv(4096):
                        response += part
                    self.assertIn(b"PROXY-OK", response)
                with self.assertRaises(ssl.SSLCertVerificationError):
                    with ssl.create_default_context().wrap_socket(
                        socket.create_connection(("127.0.0.1", proxy_port)), server_hostname="127.0.0.1"
                    ):
                        pass
                with self.assertRaises(ssl.SSLCertVerificationError):
                    with context.wrap_socket(socket.create_connection(("127.0.0.1", proxy_port)),
                                             server_hostname="wrong.invalid"):
                        pass
                with context.wrap_socket(socket.create_connection(("127.0.0.1", proxy_port)),
                                         server_hostname="127.0.0.1") as client:
                    client.settimeout(5)
                    client.sendall(b"GET /ws HTTP/1.1\r\nHost: 127.0.0.1\r\nUpgrade: websocket\r\n"
                                   b"Connection: Upgrade\r\nX-Forwarded-For: 8.8.8.8\r\n\r\n")
                    self.assertIn(b"101 Switching Protocols", client.recv(4096))
                    client.sendall(b"hello")
                    self.assertEqual(client.recv(5), b"hello")
                thread.join(timeout=3)
                self.assertEqual(headers[0].count(b"X-Forwarded-For:"), 1)
                self.assertIn(b"X-Forwarded-For: 127.0.0.1", headers[0])
                self.assertNotIn(b"8.8.8.8", headers[0])
            finally:
                process.terminate()
                process.communicate(timeout=5)
                backend.close()


if __name__ == "__main__":
    unittest.main()
