# guardian: people, cats and dogs over any VR app

OpenVR overlay app (`guardian/guardian.cpp`, stereo YOLO detector in `detector.h`) for playing without
passthrough: outlines people, cats and dogs within range, with class and confidence, on top of whatever game is
running. Each one is an upright sprite (rounded outline + label) facing you, world-locked in the standing universe.
Detections are matched into tracks by a ByteTrack-style tracker (`tracker.h`: per-track constant-velocity Kalman
filter for prediction, Hungarian matching on 3D distance, low-confidence detections only extend tracks). Box centers
and sizes follow a Catmull-Rom spline through each track's last 4 detections (sizes and distance
alpha-smoothed first, `--size-alpha 0.2`, `--dist-alpha 0.4`), played back one detection period behind (~300-350 ms
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

Build & run on the Frame (ncnn built in `~/steamframe/third_party`, see below). `make logs` tails the log.

```sh
make deploy-guardian   # from the Mac: ship + build (deploy.yml)
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

## Dead ends (so we don't retry them)

- YOLO attention as per-head ncnn `Gemm` (both inputs dynamic): ~21 ms per 64×648×648 product on Turnip. SDPA is ~5 ms.
- ncnn Vulkan options: fp16 arithmetic, winograd, shader local memory, subgroup ops all best at their defaults.
  No fp8 on the Adreno 750 (ncnn reports `fp8-cm=0`). int8 (`ncnn2table`/`ncnn2int8`, all layers still on the GPU):
  50 ms @576 vs 32 ms fp16; every int8 conv is ~1.7x slower (no cooperative matrix on Turnip).
- YOLO on the Hexagon NPU (QNN HTP): no FastRPC on the Frame. Valve's `dsp-sm8650` driver owns `/dev/cdsp`
  and `/dev/adsp` (DT `qcom,sm8650-hexagon`, no glink/fastrpc nodes, no remoteproc), and SteamVR's `dsp_service`
  boots its own stripped cDSP image (`drivers/cv/bin/linuxarm64/cdsp_sm8650.mbn`) that eyetracking (`-b CDSP`) talks
  to. Qualcomm's `cdsp.mbn` (has the unsigned-PD shell built in) sits unused in `/lib/firmware/qcom/sm8650`.
