#!/usr/bin/env python3
"""Watch the headset view streamed by yoloxr/mirrorserver.py (runs with yoloxr) over UDP.
Usage: ./udpmirror.py [host] [port]             -> plays in ffplay
       ./udpmirror.py [host] [port] > out.ts    -> saves the H.264 MPEG-TS stream
"""
import os
import socket
import subprocess
import sys
import threading
import time

host = sys.argv[1] if len(sys.argv) > 1 else os.environ.get("FRAME_HOST", "frame.local")
port = int(sys.argv[2]) if len(sys.argv) > 2 else 5600
addr = (socket.gethostbyname(host), port)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 << 20)  # an 8 Mbit/s keyframe arrives as one burst


def keepalive():
    while True:
        sock.sendto(b"hi", addr)
        time.sleep(1)


threading.Thread(target=keepalive, daemon=True).start()
if sys.stdout.isatty():
    # Relay to ffplay over localhost UDP, not a pipe: ffplay blocks in a pipe read() uninterruptibly whenever the
    # stream stalls (yoloxr not running, headset idle), so closing its window would hang until Ctrl+C.
    local = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    local.bind(("127.0.0.1", 0))
    lport = local.getsockname()[1]
    local.close()
    player = subprocess.Popen(["ffplay", "-loglevel", "error", "-fflags", "nobuffer", "-flags", "low_delay",
                               "-framedrop", "-f", "mpegts", "-window_title", "Steam Frame",
                               f"udp://127.0.0.1:{lport}?fifo_size=100000&overrun_nonfatal=1"])
    write = lambda data: sock.sendto(data, ("127.0.0.1", lport))
else:
    player, out = None, open(1, "wb", buffering=0)
    write = out.write
sock.settimeout(1)  # so closing ffplay ends this even when nothing arrives
try:
    while player is None or player.poll() is None:
        try:
            write(sock.recv(65536))
        except (TimeoutError, ConnectionRefusedError):  # refused: ffplay not listening yet, or just quit
            pass
except (BrokenPipeError, KeyboardInterrupt):
    pass
finally:
    if player:
        player.terminate()
