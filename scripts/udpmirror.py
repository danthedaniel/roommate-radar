#!/usr/bin/env python3
"""Watch the headset view streamed by mirrorserver.py (a user service on the Frame, see ansible/) over UDP.
Usage: scripts/udpmirror.py [host] [port]             -> plays in ffplay
       scripts/udpmirror.py [host] [port] > out.ts    -> saves the H.264 MPEG-TS stream
       scripts/udpmirror.py --record out.mp4          -> plays and also saves the stream (no re-encode) to an MP4
"""
import argparse
import os
import socket
import subprocess
import sys
import threading
import time

ap = argparse.ArgumentParser()
ap.add_argument("host", nargs="?", default=os.environ.get("FRAME_HOST", "frame.local"))
ap.add_argument("port", nargs="?", type=int, default=5600)
ap.add_argument("--record", metavar="MP4", help="also copy the H.264 stream into this MP4 file")
args = ap.parse_args()
host, port = args.host, args.port
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 << 20)  # an 8 Mbit/s keyframe arrives as one burst


def keepalive():
    # Re-resolve every time and never die: a Wi-Fi blip, sleep/wake or a new DHCP address must not stop the stream.
    while True:
        try:
            sock.sendto(b"hi", (socket.gethostbyname(host), port))
        except OSError:
            pass
        time.sleep(1)


threading.Thread(target=keepalive, daemon=True).start()
if sys.stdout.isatty():
    # Relay to ffplay over localhost UDP, not a pipe: ffplay blocks in a pipe read() uninterruptibly whenever the
    # stream stalls (headset idle), so closing its window would hang until Ctrl+C.
    local = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    local.bind(("127.0.0.1", 0))
    lport = local.getsockname()[1]
    local.close()
    player = subprocess.Popen(["ffplay", "-loglevel", "error", "-fflags", "nobuffer", "-flags", "low_delay",
                               "-framedrop", "-probesize", "32", "-analyzeduration", "0",  # first frame in ~1 s, not ~6 s
                               "-f", "mpegts", "-window_title", "Steam Frame",
                               f"udp://127.0.0.1:{lport}?fifo_size=100000&overrun_nonfatal=1"])
    write = lambda data: sock.sendto(data, ("127.0.0.1", lport))
else:
    player, out = None, open(1, "wb", buffering=0)
    write = out.write
recorder = None
if args.record:
    # Fragmented MP4 stays playable if this is killed before ffmpeg can finish the file.
    recorder = subprocess.Popen(["ffmpeg", "-loglevel", "error", "-y", "-f", "mpegts", "-i", "pipe:0", "-c", "copy",
                                 "-movflags", "+frag_keyframe+empty_moov", args.record], stdin=subprocess.PIPE)
    play = write

    def write(data):
        recorder.stdin.write(data)
        play(data)
sock.settimeout(1)  # so closing ffplay ends this even when nothing arrives
try:
    while player is None or player.poll() is None:
        try:
            write(sock.recv(65536))
        except BrokenPipeError:  # stdout reader went away
            break
        except OSError:  # timeout; refused (ffplay not listening yet, server restarting); network down
            pass
except KeyboardInterrupt:
    pass
finally:
    if player:
        player.terminate()
    if recorder:
        recorder.stdin.close()
        recorder.wait()
