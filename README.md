# Roommate Radar for Steam Frame

https://github.com/user-attachments/assets/209e909d-0cea-47e2-8dc6-f2a63bc855a5

<img src="packages/roommate-radar/assets/icon.svg" width="96" align="right" alt="">

See the people, cats and dogs around you while you play in VR, without turning on passthrough. Roommate Radar is a
SteamVR overlay for the Steam Frame: it watches the headset's tracking cameras, runs a stereo YOLO detector on the GPU
alongside your game, and outlines whoever is within range, with what they are and how sure it is, on top of whatever
you're playing.

- People, cats and dogs within range (1.5–8 m) get an outline and a label, world-locked and turned to face you. Your own
  arms and hands are ignored.
- Works over any VR app. A **Roommate Radar** tab in the SteamVR dashboard turns it on and off, picks what to detect,
  the range, whether to detect within the play area, the detection resolution and rate, and whether to run only while a game is running.
- Starts with SteamVR. No root and no system changes: it installs like a Steam app (in the Steam Linux Runtime 4
  container), plus a SteamVR driver that reads the camera frames.
- Light on the GPU: YOLO11n at 320 px and 4 Hz (default, adjustable up to 640px and 16Hz), on a low-priority queue the
  game preempts.

## Requirements

- A Steam Frame with **Settings → System → Developer Mode** on, reachable as `frame.local` (set in `ansible/hosts`).
- To build: an Apple silicon Mac with Docker Desktop (give it a few GB of memory), [uv](https://docs.astral.sh/uv/)
  and Ansible. The build runs in Valve's Steam Runtime 4 arm64 SDK image, so an arm64 Linux machine with Docker should
  work too (untested).

## Build and install

```sh
git clone --recursive git@github.com:danthedaniel/roommate-radar.git
cd roommate-radar
make build                   # models, ncnn and the app, built in the Steam Runtime SDK into dist/roommate-radar
make deploy                  # pairs your ssh key with the Frame (approve on the headset), installs it as a Steam app
```

Then start **Devkit Game: roommate_radar** from the Steam Library once and restart SteamVR: the first run registers
the driver, which SteamVR loads when it starts. From then on Roommate Radar starts with SteamVR. `make logs` follows its
log.

## How it works

A SteamVR driver, running inside vrserver (which may read XRService's camera buffers), publishes the newest frames
from the two SLAM cameras to shared memory. The app rectifies them into a stereo pair aligned with your head, runs
YOLO11n (ncnn on Vulkan) on both views at once, matches the boxes across views for depth, and tracks each person or
pet in 3D (Kalman filter + Hungarian matching) to draw smooth, world-locked outlines as OpenVR overlays. The details,
the measurements behind the defaults and what didn't work are in
[packages/roommate-radar/README.md](packages/roommate-radar/README.md).

## Repository

- [`packages/roommate-radar/`](packages/roommate-radar/README.md): the app and its SteamVR driver (Rust), its build
  (`build.sh`) and install (`deploy.yml`).
- [`packages/mirror/`](packages/mirror/README.md): the headset view streamed to your computer over UDP, for watching
  and recording what's on screen.
- [`docs/steam-frame.md`](docs/steam-frame.md): notes on the Frame itself: access, devkit key pairing, the cameras,
  dead ends.
- `scripts/`: raw camera tools (live mosaic of all four tracking cameras).
- `third_party/ncnn`: submodule, [danthedaniel/ncnn](https://github.com/danthedaniel/ncnn) (upstream ncnn plus a GPU
  queue priority option and a few C API calls).
- `ansible/` (inventory, key pairing, the playbook that runs each package's `deploy.yml`) and the `Makefile`:
  `make build | deploy | deploy-all | deploy-mirror | logs | stream | cams | ssh`. `make deploy-all` sets up a
  fresh Frame and redeploys an existing one; every first-time step is skipped once its output exists.
- Lint/format: ruff and basedpyright for the Python (3.12, the Frame's) in `pyproject.toml`; `cargo fmt` and
  `cargo clippy` for the Rust (`packages/roommate-radar/rustfmt.toml`). CI (`.github/workflows/ci.yml`) type-checks
  the Python and builds and tests Roommate Radar (`build.sh --check`, in the SDK image on an arm64 runner).

## License

[AGPL-3.0](LICENSE). It uses ncnn (BSD-3-Clause), the OpenVR SDK (BSD-3-Clause), Noto Sans (SIL OFL 1.1) and
Ultralytics' YOLO11n weights (AGPL-3.0), which the build downloads.
