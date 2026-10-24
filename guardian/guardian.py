#!/usr/bin/python3
"""Started by SteamVR (guardian.vrmanifest, see guardian --register): start camserver (root) unless it's already
running, then guardian. Extra args go to guardian. Output: /tmp/guardian.log
Python 3.12 (the Frame's /usr/bin/python3), stdlib only."""

import os
import signal
import subprocess
import sys

os.chdir(os.path.dirname(os.path.abspath(__file__)))
log = os.open("/tmp/guardian.log", os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o644)
os.dup2(log, 1)
os.dup2(log, 2)
# Low GPU priority: the game preempts the detector (patched ncnn, the third_party/ncnn submodule)
os.environ.setdefault("NCNN_QUEUE_GLOBAL_PRIORITY", "low")
if not os.path.exists("calib.txt"):
    subprocess.run([sys.executable, "calib.py", "calib.txt"], check=False)
own_cam = subprocess.run(["pgrep", "-x", "camserver"], stdout=subprocess.DEVNULL, check=False).returncode != 0
if own_cam:
    with open("/tmp/camserver.log", "w") as f:
        subprocess.Popen(["sudo", "-n", "build/camserver"], stdout=f, stderr=f)
app = subprocess.Popen(["build/guardian", *sys.argv[1:]])


def stop(*_: object) -> None:
    app.send_signal(signal.SIGINT)


for s in (signal.SIGINT, signal.SIGTERM):
    signal.signal(s, stop)
try:
    code = app.wait()
finally:
    if own_cam:
        subprocess.run(["sudo", "-n", "pkill", "-x", "camserver"], check=False)
sys.exit(code if code >= 0 else 128 - code)
