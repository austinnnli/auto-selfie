#!/usr/bin/env python3
"""Generate the self-signed certificate the phone client needs.

C-6: the page must be a secure origin for camera access, and a secure origin
cannot open a plain WebSocket.  So: HTTPS with a self-signed certificate that
Safari will warn about once.

The certificate carries the laptop's current LAN IP as a subjectAltName.  That
matters: Safari rejects a certificate whose SAN does not cover the address in
the URL bar, and the phone will be connecting to an IP, not a hostname.  If the
laptop moves to another network, re-run this.

    python tools/gen_certs.py
    python tools/gen_certs.py --ip 192.168.1.50   # override the guess
"""

from __future__ import annotations

import argparse
import datetime
import ipaddress
import pathlib
import socket
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
CERT_DIR = ROOT / "certs"


def lan_ip() -> str:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect(("10.255.255.255", 1))
        return sock.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        sock.close()


def with_cryptography(ip: str, days: int) -> bool:
    try:
        from cryptography import x509
        from cryptography.hazmat.primitives import hashes, serialization
        from cryptography.hazmat.primitives.asymmetric import rsa
        from cryptography.x509.oid import NameOID
    except ImportError:
        return False

    key = rsa.generate_private_key(public_exponent=65537, key_size=2048)
    name = x509.Name([
        x509.NameAttribute(NameOID.COMMON_NAME, ip),
        x509.NameAttribute(NameOID.ORGANIZATION_NAME, "framing rover"),
    ])
    alt = [x509.DNSName("localhost"), x509.IPAddress(ipaddress.ip_address("127.0.0.1"))]
    try:
        alt.append(x509.IPAddress(ipaddress.ip_address(ip)))
    except ValueError:
        alt.append(x509.DNSName(ip))

    now = datetime.datetime.now(datetime.timezone.utc)
    cert = (
        x509.CertificateBuilder()
        .subject_name(name)
        .issuer_name(name)
        .public_key(key.public_key())
        .serial_number(x509.random_serial_number())
        .not_valid_before(now - datetime.timedelta(minutes=5))
        .not_valid_after(now + datetime.timedelta(days=days))
        .add_extension(x509.SubjectAlternativeName(alt), critical=False)
        .add_extension(x509.BasicConstraints(ca=True, path_length=None), critical=True)
        .sign(key, hashes.SHA256())
    )

    CERT_DIR.mkdir(exist_ok=True)
    (CERT_DIR / "server.key").write_bytes(key.private_bytes(
        encoding=serialization.Encoding.PEM,
        format=serialization.PrivateFormat.TraditionalOpenSSL,
        encryption_algorithm=serialization.NoEncryption()))
    (CERT_DIR / "server.crt").write_bytes(cert.public_bytes(serialization.Encoding.PEM))
    return True


def with_openssl(ip: str, days: int) -> bool:
    CERT_DIR.mkdir(exist_ok=True)
    cmd = [
        "openssl", "req", "-x509", "-newkey", "rsa:2048", "-nodes",
        "-keyout", str(CERT_DIR / "server.key"),
        "-out", str(CERT_DIR / "server.crt"),
        "-days", str(days), "-subj", f"/CN={ip}",
        "-addext", f"subjectAltName=IP:{ip},IP:127.0.0.1,DNS:localhost",
    ]
    try:
        subprocess.run(cmd, check=True, capture_output=True)
        return True
    except (OSError, subprocess.CalledProcessError):
        return False


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--ip", default=None, help="address the phone will use")
    parser.add_argument("--days", type=int, default=825)
    args = parser.parse_args()

    ip = args.ip or lan_ip()
    if not (with_cryptography(ip, args.days) or with_openssl(ip, args.days)):
        print("need either the `cryptography` package or the openssl binary",
              file=sys.stderr)
        return 1

    print(f"wrote certs/server.crt and certs/server.key for {ip}")
    print(f"open  https://{ip}:8443/  on the phone and accept the warning once")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
