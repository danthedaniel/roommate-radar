# Steam Frame

Notes and code for poking at my Steam Frame over the LAN.

## Device

| | |
|-|-|
| Hostname | `frame` (`frame.local` via mDNS) |
| IP | frame.local (DHCP, may change) |
| User | `steamos` |
| OS | SteamOS, aarch64, kernel 6.18 |

## Connect

```sh
ssh steamos@frame.local
```

Key auth uses `~/.ssh/id_rsa`. Needs **Settings → System → Developer Mode** on.

## Pairing a key (devkit host pairing)

The Frame runs a devkit service on port 32000 (the thing Valve's SteamOS Devkit
Client talks to). Pairing is one POST: the public key + a `devkit-client:` comment
+ the client's magic phrase. Approve on the headset if it prompts.

```sh
key=$(cut -d' ' -f1-2 ~/.ssh/id_rsa.pub)
curl -X POST -H 'Content-Type: text/plain' --data-binary \
  "$key devkit-client:$(whoami)@$(hostname -s) 900b919520e4cf601998a71eec318fec
" http://frame.local:32000/register
# -> Registered
```

Source: `client/devkit_client/__init__.py` `register()` in
https://gitlab.steamos.cloud/devkit/steamos-devkit

Other endpoints: `GET /login-name` → `steamos`, `GET /properties.json`.

SSH password auth is off (`/etc/ssh/sshd_config.d/10-no-password.conf`: `PasswordAuthentication no`), so
pairing is the only way to add a key. Developer settings → **Set User Password** still works for local/sudo use.

Root: `steamos` has passwordless sudo via `/etc/sudoers.d/zz-steamos-nopasswd`.

## Cameras

| Node | Sensor | Name (XRService) | Native | Notes |
|-|-|-|-|-|
| `/dev/video0` | og01a1b `4-0036` | `slam_right` | 1056×1024 Y8, ~115 fps | outward fisheye |
| `/dev/video3` | og01a1b `4-0060` | `slam_left` | 1056×1024 Y8 | |
| `/dev/video9` | og0ve10 `5-003e` | upper (L or R) | 640×480 Y8 | outward fisheye |
| `/dev/video13` | og0ve10 `5-0060` | upper (L or R) | 640×480 Y8 | |
| — | arcimx616 ×2 (`0-0010`, `0-001a`) | ? ("passthrough cameras") | UYVY 1920×1080 | not streaming |
| `/dev/video99` | v4l2loopback | — | 1920×1080 RGB | compositor mirror (`steamvr-v4l2cam`) |

XRService alternates exposures: ~8.4 ms (tracking/passthrough) and ~0.037 ms (controller LEDs).

### Live raw frames without stopping SteamVR

XRService holds the capture queues exclusively, but as root `rawcam.py` duplicates its fds
(`pidfd_getfd`), asks each queue for its newest completed buffer (`VIDIOC_QUERYBUF`,
read-only), and mmaps that dmabuf. Buffers are NV12 with Y stride 1152 (SLAM) / 640 (upper).

```sh
scripts/stream.sh                            # live 2x2 mosaic in ffplay
scripts/stream.sh --exposure controller      # the LED-tracking exposure instead
ssh steamos@frame.local 'sudo python3 - --frames 1' < scripts/rawcam.py \
  | ffmpeg -f rawvideo -pix_fmt gray -s 2112x1504 -i - snap.png   # one raw snapshot
scripts/udpmirror.py                         # headset view (/dev/video99) from the mirrorserver service; black/~1 fps when not worn
scripts/udpmirror.py > out.ts                # same, saved
scripts/udpmirror.py --record out.mp4        # watch and save to MP4 (stream copied, not re-encoded)
```

`mirrorserver.py` runs on the Frame as a systemd user service (`mirrorserver`, UDP 5600, H.264 MPEG-TS, libx264
only while a client sends keepalives), independent of guardian. Logs: `journalctl --user -u mirrorserver -f`.

## Ansible

`ansible/deploy.yml` sets up a fresh Frame and redeploys an existing one (`make deploy`, or
`cd ansible && ansible-playbook deploy.yml [--tags mirror|guardian]`). Only Developer Mode has to be switched on by hand.
Every first-time step is skipped once its output exists.

- Always: pairs `~/.ssh/id_rsa.pub` over the devkit service if ssh with it fails (approve on the headset if asked).
- `mirror`: ships `mirrorserver.py` and enables the `mirrorserver` user service.
- `guardian`: on the Mac, checks out the ncnn submodule, downloads `yolo11n-grayscale.pt` and exports + preps the
  320 model into `models/` (`uvx`, needs uv); on the Frame, rsyncs ncnn and builds it, ships the models, and
  rsyncs `guardian/` to `~/steamframe/guardian` (keeps the Frame's `build/` and `calib.txt`) and builds it (camserver,
  guardian).

## guardian: people, cats and dogs over any VR app

OpenVR overlay app (`guardian/guardian.cpp`, stereo YOLO detector in `detector.h`) for playing without
passthrough: outlines people, cats and dogs within range, with class and confidence, on top of whatever game is
running. Each one is an upright sprite (rounded outline + label) facing you, world-locked in the standing universe.
Detections are matched into tracks by a ByteTrack-style tracker (`tracker.h`: per-track constant-velocity Kalman
filter for prediction, Hungarian matching on 3D distance, low-confidence detections only extend tracks). Box centers
and sizes follow a Catmull-Rom spline through each track's last 4 detections (sizes and distance
alpha-smoothed first, `--size-alpha 0.2`, `--dist-alpha 0.2`), played back one detection period behind (~300-350 ms
total; drawing at the Kalman prediction for "now" instead looked worse). Boxes with a controller,
or the forearm leading to it, inside them (same depth) are your arms and are ignored.

- Starts with SteamVR: `build/guardian --register` (in `~/steamframe/guardian`) adds `guardian.vrmanifest` with
  auto-launch and starts it. `guardian.py` starts camserver if it isn't running. Log: `/tmp/guardian.log`.
- "Guardian" tab in the SteamVR dashboard: on/off, only in games (runs from a game's launch,
  intro included, until it quits; passthrough itself isn't reported to apps), people, cats and dogs, ignore my hands,
  range, status. Saved in `~/.config/guardian.conf`.
- GPU sharing with the game: YOLO at 320 and 4 Hz, on a low-priority queue (`NCNN_QUEUE_GLOBAL_PRIORITY=low`, set
  by `guardian.py`) so the game preempts it (msm has preemption on: 4 rings). Every 5 s it logs the game's
  presented/dropped/reprojected frames and GPU time; `--no-detector` gives the baseline.

```
camserver (root)  ──/dev/shm/steamframe-cams──▶  detector thread (detector.h)  ──▶  tracker + OpenVR overlays
newest normal-exposure SLAM L/R frames           rectify both views to           (guardian.cpp)
+ capture time (CLOCK_MONOTONIC)                 head-aligned pinholes, YOLO
                                                 (ncnn Vulkan) on the stacked
                                                 pair, stereo-match, depth=fB/d
```

- Calibration: `/persist/xrservice.json` (factory KB4 intrinsics + extrinsics) and
  `/persist/device_config.json` (`head` and XRService's `cv/cad_from_cal`). `calib.py` chains
  them into `calib.txt` (head_from_cam per SLAM camera). Cam0 = `slam_right`, Cam1 = `slam_left`.
- Camera timestamps are `TIMESTAMP_MONOTONIC`-flagged but run ~210–230 ms ahead of
  `CLOCK_MONOTONIC`; camserver tracks the offset from arrival times. `--latency-ms` tunes the rest.
- The rectified views share the head's orientation, so they're a rectified stereo pair with a
  105.6 mm horizontal baseline.

Build & run on the Frame (ncnn built in `~/steamframe/third_party`, see below). The `Makefile` wraps the common ones:
`make deploy | build | logs | stream | cams | ssh | sysroot`. `make sysroot` copies the Frame's headers into
`.sysroot/` so clangd on the Mac (`guardian/compile_flags.txt`, aarch64 target) resolves every include.
Style: `.clang-format` for the C++, ruff and basedpyright (Python 3.12, the Frame's) in `pyproject.toml`.

```sh
make build   # from the Mac: ship + build (ansible, see above)
ssh steamos@frame.local 'cd ~/steamframe/guardian && build/guardian --register'   # once: auto-launch with SteamVR
```

What ansible does for ncnn and the models, by hand. ncnn is the submodule `third_party/ncnn`: branch `ncnn` of this repo,
a history-less snapshot of upstream c15904b plus our commits (`NCNN_QUEUE_GLOBAL_PRIORITY`, see guardian). Its URL is `./`,
so check it out with `git -c protocol.file.allow=always submodule update --init`. Copy it to
`~/steamframe/third_party/ncnn`, then `cmake -G Ninja -DNCNN_VULKAN=ON -DNCNN_SIMPLEVK=OFF -DNCNN_SYSTEM_GLSLANG=ON
-DGLSLANG_TARGET_DIR=/usr/lib/cmake/glslang -DNCNN_SHARED_LIB=OFF -DNCNN_BUILD_TOOLS=OFF
-DNCNN_BUILD_EXAMPLES=OFF -DCMAKE_INSTALL_PREFIX=$HOME/steamframe/third_party/ncnn-install`.
(`SIMPLEVK=OFF` because its stub Vulkan types clash with `vulkan.h`.)
Models: Ultralytics' `yolo11n-grayscale.pt` (YOLO11n trained on grayscale COCO, 1-channel input;
https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo11n-grayscale.pt).
`yolo export model=yolo11n-grayscale.pt format=ncnn imgsz=<2S>,<S>` (ultralytics) → rename `yolo11n-grayscale_ncnn_model/model.ncnn.*`
to `~/steamframe/models/yolo11n-grayscale_<S>.ncnn.{param,bin}`, then `python3 guardian/prep_model.py models/yolo11n-grayscale_<S>.ncnn`
→ `yolo11n-grayscale_<S>_gray.ncnn.*`, which guardian loads (attention MatMuls → Vulkan SDPA, since ncnn's MatMul is CPU-only;
1/255 folded into the first conv). prep_model.py also takes the RGB `yolo11n.pt` export (GRAY2RGB folded in too); `--model` picks one.
The export bakes in the input size, so each `--size S` needs its own (guardian uses S=320).
Grayscale vs RGB weights on 500 random COCO val2017 images (seed 0), gray, letterboxed and stacked two per 576x1152 input, ncnn
Vulkan on the Frame, pycocotools: mAP50-95 **35.8 vs 32.3** (mAP50 49.3 vs 45.4, AP small/medium/large 15.4/40.6/56.6 vs
13.6/37.9/53.0); speed identical (~26 ms median each, nothing else on the GPU).
YOLO11n on the Adreno 750 (Turnip) for the stacked S x 2S input, SteamVR running: 23 ms @480, 31.4 ms @576, 38.5 ms @640
(37 ms @576 before prep_model.py and the conv options).

### Dead ends (so we don't retry them)

- OpenXR `/interaction_profiles/valve/frame_controller`: SteamVR rejects suggested bindings for it; suggest
  `valve/index_controller` + `khr/simple_controller` aim poses instead and SteamVR remaps the Frame controllers.
- `IVRTrackedCamera` (OpenVR): HMD reports a camera but `Prop_CameraFrameLayout=0`, `GetCameraFrameSize` fails.
- Reading `/dev/videoN` directly: EBUSY while SteamVR runs. With SteamVR stopped it streams,
  but exposure is ~0 (V4L2 exposes only `analogue_gain`; std OV exposure regs aren't what XRService uses).
- Stopping SteamVR: `steamvr.service` is `Restart=always` and gets restarted by `steam.service`.
  `XRService --snapshotCamerasAndExit` works if SteamVR is masked first (`systemctl --user mask --runtime`).
- `/proc/<pid>/mem` can't read dmabuf mappings (EIO) — hence `pidfd_getfd`.
- Hardware H.264 encoder (`v4l2h264enc`, `h264_v4l2m2m`) fails here; scripts use libx264.
- YOLO attention as per-head ncnn `Gemm` (both inputs dynamic): ~21 ms per 64×648×648 product on Turnip. SDPA is ~5 ms.
- ncnn Vulkan options: fp16 arithmetic, winograd, shader local memory, subgroup ops all best at their defaults.
  No fp8 on the Adreno 750 (ncnn reports `fp8-cm=0`). int8 (`ncnn2table`/`ncnn2int8`, all layers still on the GPU):
  50 ms @576 vs 32 ms fp16; every int8 conv is ~1.7x slower (no cooperative matrix on Turnip).
- YOLO on the Hexagon NPU (QNN HTP): no FastRPC on the Frame. Valve's `dsp-sm8650` driver owns `/dev/cdsp`
  and `/dev/adsp` (DT `qcom,sm8650-hexagon`, no glink/fastrpc nodes, no remoteproc), and SteamVR's `dsp_service`
  boots its own stripped cDSP image (`drivers/cv/bin/linuxarm64/cdsp_sm8650.mbn`) that eyetracking (`-b CDSP`) talks
  to. Qualcomm's `cdsp.mbn` (has the unsigned-PD shell built in) sits unused in `/lib/firmware/qcom/sm8650`.


## Finding it if the IP changes

```sh
dig +short frame.local @224.0.0.251 -p 5353
nmap -p 22,32000 --open <your LAN>/24
```

## Refs

- https://partner.steamgames.com/doc/steamhardware/steamframe/setup
- https://partner.steamgames.com/doc/steamhardware/steamframe/debugging
