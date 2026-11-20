#!/usr/bin/env python3
"""Create a local lab CA, IP-bound TLS certificate, and ephemeral role tokens."""

from __future__ import annotations

import argparse
import ipaddress
import json
import os
from pathlib import Path
import secrets
import ssl
import subprocess
import tempfile


def openssl(*args):
    subprocess.run(("openssl", *map(str, args)), check=True,
                   stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ip", required=True)
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--cert-dir", required=True)
    args = parser.parse_args()
    ip = str(ipaddress.IPv4Address(args.ip))
    run_dir, cert_dir = Path(args.run_dir), Path(args.cert_dir)
    if not run_dir.is_dir() or not cert_dir.is_dir():
        parser.error("run-dir and cert-dir must already exist")
    if run_dir.stat().st_mode & 0o077 or cert_dir.stat().st_mode & 0o077:
        parser.error("run-dir and cert-dir must be private (mode 0700)")

    ca = cert_dir / "ca"
    if not ca.exists():
        with tempfile.TemporaryDirectory(prefix="ca-", dir=cert_dir) as tmp:
            temp = Path(tmp)
            openssl("req", "-x509", "-newkey", "rsa:3072", "-nodes", "-sha256",
                    "-days", "3650", "-subj", "/CN=Teleop Jaco2 laboratorio CA",
                    "-addext", "basicConstraints=critical,CA:TRUE",
                    "-addext", "keyUsage=critical,keyCertSign,cRLSign",
                    "-keyout", temp / "ca.key", "-out", temp / "ca.crt")
            os.chmod(temp / "ca.key", 0o600)
            os.rename(temp, ca)
    if not (ca / "ca.key").is_file() or not (ca / "ca.crt").is_file():
        raise RuntimeError("La CA existente está incompleta; consérvela y revise manualmente")
    if (ca / "ca.key").stat().st_mode & 0o077:
        raise RuntimeError("Permisos inseguros en la clave privada de la CA")
    openssl("x509", "-checkend", "0", "-noout", "-in", ca / "ca.crt")

    leaf = cert_dir / ("server-" + ip)
    if not leaf.exists():
        with tempfile.TemporaryDirectory(prefix="server-", dir=cert_dir) as tmp:
            temp = Path(tmp)
            openssl("req", "-new", "-newkey", "rsa:2048", "-nodes", "-sha256",
                    "-subj", "/CN=Teleop Jaco2 LAN", "-keyout", temp / "server.key",
                    "-out", temp / "server.csr")
            ext = temp / "extensions.cnf"
            ext.write_text("basicConstraints=critical,CA:FALSE\nkeyUsage=critical,digitalSignature,keyEncipherment\nextendedKeyUsage=serverAuth\nsubjectAltName=IP:" + ip + "\n", encoding="ascii")
            openssl("x509", "-req", "-in", temp / "server.csr", "-CA", ca / "ca.crt",
                    "-CAkey", ca / "ca.key", "-set_serial", str(secrets.randbits(128)),
                    "-days", "365", "-sha256", "-extfile", ext, "-out", temp / "server.crt")
            os.chmod(temp / "server.key", 0o600)
            os.rename(temp, leaf)
    openssl("verify", "-CAfile", ca / "ca.crt", "-verify_ip", ip, leaf / "server.crt")
    if (leaf / "server.key").stat().st_mode & 0o077:
        raise RuntimeError("Permisos inseguros en la clave privada del servidor")
    ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER).load_cert_chain(leaf / "server.crt", leaf / "server.key")
    # x mode prevents clobbering a previous run's credentials.
    credentials = {role: [secrets.token_urlsafe(32) for _ in range(count)]
                   for role, count in (("operators", 2), ("observers", 3), ("admins", 1))}
    fd = os.open(run_dir / "credentials.json", os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, "w", encoding="utf-8") as stream:
        json.dump(credentials, stream)
        stream.write("\n")
    print("CA:", ca / "ca.crt")
    print("Certificado servidor:", leaf / "server.crt")
    print("Credenciales privadas de esta sesión:", run_dir / "credentials.json")


if __name__ == "__main__":
    main()
