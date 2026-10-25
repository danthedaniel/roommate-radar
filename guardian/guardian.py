#!/usr/bin/python3
"""The app's entry point. Steam runs it in the Steam Linux Runtime 4 container, from the Library or from SteamVR's
auto-launch (through Steam). Steam has no install scripts on Linux, so it does the app's setup on every start:
- registers driver/ (driver_guardian, the camera frames) with SteamVR; it loads on SteamVR's next start
- writes guardian.vrmanifest (the overlay, launched through Steam) for guardian --register to add, where Steam
  hasn't, with auto-launch
- calib.txt from the driver's copy of the factory calibration
then runs bin/guardian. Extra args go to guardian. Output: /tmp/guardian.log
Python 3.13 (the runtime's /usr/bin/python3), stdlib only."""

import json
import os
import signal
import subprocess
import sys

here = os.path.dirname(os.path.abspath(__file__))
os.chdir(here)
log = os.open("/tmp/guardian.log", os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o644)
os.dup2(log, 1)
os.dup2(log, 2)
# Low GPU priority: the game preempts the detector (patched ncnn, the third_party/ncnn submodule)
os.environ.setdefault("NCNN_QUEUE_GLOBAL_PRIORITY", "low")
CALIB_DIR = "/dev/shm/steamframe-calib"  # shm::kCalibDir, written by the driver


def register_driver() -> None:
    with open(os.path.expanduser("~/.config/openvr/openvrpaths.vrpath")) as f:
        paths = json.load(f)
    driver = os.path.join(here, "driver")
    if driver not in (paths.get("external_drivers") or []):
        vrpathreg = os.path.join(paths["runtime"][0], "bin", "vrpathreg.sh")
        subprocess.run([vrpathreg, "adddriver", driver], check=False)
        print(f"registered {driver} with SteamVR; it loads when SteamVR next starts", flush=True)


def write_manifest(appid: str, gameid: str) -> None:
    # The entry Steam generates for a "Launch SteamVR Overlay" app; vrclient names an overlay steam.overlay.<SteamAppId>.
    manifest = {
        "source": "user",
        "applications": [
            {
                "app_key": f"steam.overlay.{appid}",
                "launch_type": "url",
                "url": f"steam://rungameid/{gameid}",
                "is_dashboard_overlay": True,
                "strings": {
                    "en_us": {"name": "Guardian", "description": "Outlines people, cats and dogs over any VR app"}
                },
            }
        ],
    }
    with open("guardian.vrmanifest", "w") as f:
        json.dump(manifest, f, indent="\t")


register_driver()
if os.environ.get("SteamAppId") and os.environ.get("SteamGameId"):
    write_manifest(os.environ["SteamAppId"], os.environ["SteamGameId"])
if os.path.exists(f"{CALIB_DIR}/xrservice.json"):
    subprocess.run([sys.executable, "calib.py", "calib.txt", CALIB_DIR], check=False)
subprocess.run(["bin/guardian", "--register"], check=False)  # before the overlay connects, see guardian.cpp
app = subprocess.Popen(["bin/guardian", *sys.argv[1:]])


def stop(*_: object) -> None:
    app.send_signal(signal.SIGINT)


for s in (signal.SIGINT, signal.SIGTERM):
    signal.signal(s, stop)
code = app.wait()
sys.exit(code if code >= 0 else 128 - code)
