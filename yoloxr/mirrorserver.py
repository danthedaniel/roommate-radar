"""Serve the headset view (SteamVR's /dev/video99 compositor mirror) as H.264 in MPEG-TS over UDP.

Clients send any datagram to the port at least every few seconds (see ../udpmirror.py) and get the stream back
from it. ffmpeg only runs while someone is watching. Started by run.sh. Usage: mirrorserver.py [port]
"""
import os
import selectors
import signal
import socket
import subprocess
import sys
import time

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 5600
TIMEOUT = 3  # s without a keepalive before a client is dropped
PKT = 7 * 188  # 7 TS packets per datagram, the usual MPEG-TS-over-UDP size
# libx264: the hardware encoder (v4l2h264enc / h264_v4l2m2m) fails on this input
FFMPEG = ["ffmpeg", "-hide_banner", "-loglevel", "error", "-f", "v4l2", "-i", "/dev/video99", "-pix_fmt", "yuv420p",
          "-c:v", "libx264", "-preset", "ultrafast", "-tune", "zerolatency", "-b:v", "8M", "-g", "30",
          "-f", "mpegts", "-"]

signal.signal(signal.SIGTERM, lambda *_: sys.exit())
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4 << 20)
sock.bind(("", PORT))
sel = selectors.DefaultSelector()
sel.register(sock, selectors.EVENT_READ)
clients = {}  # addr -> last keepalive
enc, buf = None, b""
print(f"mirrorserver: udp {PORT}", flush=True)

try:
    while True:
        for key, _ in sel.select(timeout=1):
            if key.fileobj is sock:
                _, addr = sock.recvfrom(64)
                if addr not in clients:
                    print(f"mirrorserver: + {addr[0]}:{addr[1]}", flush=True)
                clients[addr] = time.monotonic()
            else:
                data = os.read(enc.stdout.fileno(), 1 << 16)
                if not data:  # ffmpeg died; restarted below while clients remain
                    sel.unregister(enc.stdout)
                    enc.wait()
                    enc = None
                    continue
                buf += data
                n = len(buf) // PKT * PKT
                for i in range(0, n, PKT):
                    for addr in clients:
                        try:
                            sock.sendto(buf[i:i + PKT], addr)
                        except OSError:  # e.g. ENOBUFS on a burst: drop the packet, the decoder recovers
                            pass
                buf = buf[n:]
        now = time.monotonic()
        for addr in [a for a, t in clients.items() if now - t > TIMEOUT]:
            print(f"mirrorserver: - {addr[0]}:{addr[1]}", flush=True)
            del clients[addr]
        if clients and not enc:
            enc, buf = subprocess.Popen(FFMPEG, stdout=subprocess.PIPE, stdin=subprocess.DEVNULL), b""
            sel.register(enc.stdout, selectors.EVENT_READ)
        elif not clients and enc:
            sel.unregister(enc.stdout)
            enc.terminate()
            enc.wait()
            enc = None
finally:
    if enc:
        enc.kill()
