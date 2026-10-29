# Roommate Radar: the app

OpenVR overlay app (Rust, `src/`, sources below) for playing without
passthrough: outlines people, cats and dogs within range, with their class, on top of whatever game is
running. Each one is an upright sprite (rounded outline + label) facing you, world-locked in the standing universe.
Left and right boxes are paired by an optimal assignment (`stereo.rs`) on edge agreement, a disparity plausible for
the box's height and class, and ZNCC, ignoring edges cut off by a view's edge; depth comes from the nearest cluster of
small-patch disparities over the object's middle, fused with the box edges' (`measure.rs` adds where the box meets
the floor, when the two agree). Each detection's noise is a thin ellipsoid along its viewing ray (stereo depth error
grows like z^2, bearing error like z), and a ByteTrack-style tracker (`tracker.rs`) matches on Mahalanobis distance
under it: one constant-velocity Kalman filter per track over 3D position and velocity, Hungarian matching, weak
detections (low confidence, one view) only extend tracks, no new track inside an existing one's gate, tracks that
become indistinguishable merge, and missing a track only counts where the views could have seen it. Boxes are drawn
at a cubic Hermite between the track's filter states (position + velocity), played back one detection period behind
(~300-350 ms total; drawing at the Kalman prediction for "now" instead looked worse), sized from the box's angular
size at the filtered distance, fading in and out. Prior art and the measurements still to take are in
[docs/detection-tracking-prior-art.md](../../docs/detection-tracking-prior-art.md). Boxes with a controller, or the
forearm leading to it, inside them (as seen from the camera, at any depth) are your arms and are ignored. Developer
builds write each detector frame (boxes, objects, measurements, tracks) to `/tmp/roommate-radar-trace.jsonl`
(`trace.rs`; at 32 MB it moves to `.1` and a new one starts, as /tmp is RAM).

- Installed like a Store app (see below): runs in the Steam Linux Runtime 4 container, started by Steam from the
  Library or, when SteamVR starts, by its driver (through Steam, below). Log: `/tmp/roommate-radar.log`.
- No root: the camera frames come from `driver_roommate_radar` (`src/driver/`), camserver's loop on a thread inside vrserver.
  vrserver is XRService's parent, so Yama (`ptrace_scope=1`) lets it `pidfd_getfd` XRService's camera buffers. It
  also copies `/persist`'s calibration to `/dev/shm/steamframe-calib`, as the container can't see `/persist`. Logs
  to `~/.local/share/Steam/logs/vrserver.txt` (`roommate_radar: ...`).
- First run (Steam has no install scripts on Linux, so roommate-radar does setup on every start, `register.rs`): registers
  `driver/` with SteamVR (`vrpathreg adddriver`) and writes `roommate-radar.vrmanifest`, then runs `bin/roommate-radar --register` before
  the overlay itself. SteamVR knows the running app as `steam.overlay.<SteamAppId>`, the key Steam's own manifest uses
  for a Store overlay; `--register` adds our manifest for it (launching `steam://rungameid/<SteamGameId>`) and turns
  SteamVR's own auto-launch for it off (below). The driver loads on SteamVR's next start: until then the dashboard says
  "Restart SteamVR to finish setup".
- Starting with SteamVR is the driver's job (`driver/autostart.rs`), not SteamVR's auto-launch: on the Frame SteamVR
  starts before Steam has logged in, Steam holds the launch SteamVR asks for until it has, then runs it ~20 ms before
  its UI can answer the launch's first step (ShowInterstitials), and it sits at "Launching…" for good (any auto-launched
  overlay; a Steam bug). The driver waits for Steam's login (`config/loginusers.vdf` newer than the running Steam,
  the pid in `~/.steam/steam.pid`), then 10 s, then sends `steam://rungameid/<gameid>` to `~/.steam/steam.pipe`. The
  app records its `gameid` (SteamGameId) in `roommate-radar.conf` at each start, next to `autostart` (the tab's
  switch).
- "Roommate Radar" tab in the SteamVR dashboard: on/off, start with SteamVR (on by default), only in games (runs from a game's launch, intro included, until it quits; passthrough itself isn't
  reported to apps), people, cats and dogs, ignore my hands, only near the play area (inside the SteamVR boundary or
  within 1 m of it, `IVRChaperoneSetup` live collision bounds; no boundary: everywhere), range, status. Saved in `~/.config/roommate-radar.conf`.
- Developer builds (`build.sh --developer`, the `developer` cargo feature; what `make deploy` installs) add the YOLO time to the
  tab's status and the confidence to each label. `make build` is the release build, without them.
- GPU sharing with the game: YOLO at 480 and 4 Hz (by default), on a low-priority queue (`NCNN_QUEUE_GLOBAL_PRIORITY=low`, set
  at start) so the game preempts it (msm has preemption on: 4 rings).

```
driver_roommate_radar (camserver  ──/dev/shm/steamframe-cams──▶ detector thread (detector.rs) ──▶  tracker + OpenVR
thread in vrserver): newest                                     rectify both views to              overlays (measure.rs,
normal-exposure SLAM L/R frames                                 head-aligned pinholes, YOLO        tracker.rs, boxes.rs)
+ capture time (CLOCK_MONOTONIC)                                (ncnn Vulkan) on the stacked
                                                                pair, stereo-match, depth=fB/d
```

Sources (`src/`, one cargo package): `main.rs` (main loop), `detector.rs` (the detector thread; `calib.rs`
rectification, `stereo.rs` matching and depth), `measure.rs` (detections to tracker measurements, your own arms
dropped), `tracker.rs`, `boxes.rs` (tracks as overlays), `panel.rs` (dashboard tab), `draw.rs` (CPU drawing, fontdue),
`textures.rs` (Vulkan overlay textures, ash), `settings.rs`, `register.rs` (`--register`), `trace.rs` (developer builds), `v3.rs`; `openvr.rs` and
`ncnn.rs` wrap the two C APIs. The driver (the package's `cdylib`): `driver/lib.rs` + `driver/camserver.rs`, and
`shm.rs` between it and roommate-radar. `assets/icon.svg` is the tab icon; the app embeds its rasters, `icon-128.rgba` (and `icon-176.rgba` for the first-run warning)
(the command is in `panel.rs`). Tests: `cargo test` (build.sh runs them in the SDK); on the Mac,
`cargo clippy --target aarch64-unknown-linux-gnu --all-targets` checks everything but the link.

- OpenVR through its C API: `openvr_sys.rs` is bindgen's output for `openvr_capi.h` (how to regenerate is at its top),
  and `openvr.rs` calls the interfaces' function tables (`VR_GetGenericInterface("FnTable:IVROverlay_028")`). It
  asserts the sizes of the structs it passes against `openvr.h`'s: `openvr_capi.h` can differ (`openvr.h` packs
  `Compositor_FrameTiming` to 4 on Linux, 188 bytes; `openvr_capi.h` doesn't).
- The driver API (`openvr_driver.h`) is C++ only, so `driver/lib.rs` lays out `IServerTrackedDeviceProvider_004` and
  `IVRDriverLog_001` as the C++ objects (vtable pointer first, Itanium ABI); a versioned interface's layout never
  changes.

- Calibration: `/persist/xrservice.json` (factory KB4 intrinsics + extrinsics) and
  `/persist/device_config.json` (`head` and XRService's `cv/cad_from_cal`). `calib.rs` chains
  them into head_from_cam per SLAM camera. Cam0 = `slam_right`, Cam1 = `slam_left`.
- The factory extrinsics are about a degree off: XRService refines them online (its log: "Current extrinsics
  differences to factory calibration ... cam0 0.84 deg, cam1 0.53 deg") but doesn't publish the result, and a degree
  between the views is 3-4 px of disparity at 480 (a person 4 m away read as 2.5 m). `selfcal.rs` estimates the views'
  relative rotation from the detector's own patch matches (`stereo::alignment`, every 8 runs, ~15 ms): pitch and roll
  from their row offsets (none in a rectified pair), yaw from the floor (the farthest thing a ray below the horizon can
  meet, at a distance known from the head pose). The detector rectifies with half of it on each camera; it's saved as
  `align` in the settings file for the next start. Measured on the Frame: steady over head angle and across reboots
  (0.1 px), drifting by hundredths of a degree a minute. The `kb_spline` lens model in the calibration only moves
  things by ~0.3 px.
- Camera timestamps are `TIMESTAMP_MONOTONIC`-flagged but run ~210–230 ms ahead of
  `CLOCK_MONOTONIC`; camserver tracks the offset from arrival times. `LATENCY_MS` (`detector.rs`) covers the rest.
- The rectified views share the head's orientation, so they're a rectified stereo pair with a
  105.6 mm horizontal baseline.

- Build (`build.sh`, also runnable directly): like a Store depot, in the Steam Runtime 4 arm64 SDK (`registry.gitlab.steamos.cloud/steamrt/
  steamrt4/sdk/arm64`, natively in the Mac's arm64 Docker Desktop) in `build-sdk/` (rustup and the toolchain in
  `build-sdk/rust/`, `rust-toolchain.toml` pins it). It lays out the app: the font,
  `models/`, `bin/roommate-radar` + `bin/libopenvr_api.so` (OpenVR SDK v2.15.6, `$ORIGIN` rpath; `build.rs` links it and
  ncnn), `driver/` (manifest, whose `name` SteamVR turns into `driver_<name>.so`, + `bin/linuxarm64/driver_roommate_radar.so`, which loads into the host's vrserver and needs
  only libc and libgcc_s). Nothing from the host: glslang is the SDK's static one, the font is shipped.
- Install (`deploy.yml`): what Valve's SteamOS Devkit Client does for an upload. `dist/roommate-radar` to
  `~/devkit-game/roommate_radar` (no `-` in a devkit game id), `roommate_radar-argv.json` (`./bin/roommate-radar`) and
  `roommate_radar-settings.json` (`compat_tool`
  `steamrt4-any`, the tool Steam maps Store apps native to Linux to), then `create-shortcut` sent to Steam over
  `~/.steam/steam.pipe` (`devkit-1 steam://devkit-1/<steam.token>/create-shortcut?response=...&gameid=roommate_radar`). Then, if
  rsync (`checksum`) changed anything under `driver/`, SteamVR is restarted (vrserver loads drivers at start; the new
  driver's autostart brings the app back); otherwise a running roommate-radar is stopped and started again through Steam
  (`steam://rungameid/<gameid>`).
- What differs from a Store install: a shortcut, not an app id (its `SteamAppId` is made up by Steam), so Steam doesn't
  generate the overlay's manifest entry and roommate-radar adds its own. With a real app id and the "Launch SteamVR
  Overlay" launch option, Steam provides it (`register.rs` then only finds it).

ncnn is the submodule `third_party/ncnn`: https://github.com/danthedaniel/ncnn, a history-less snapshot of upstream
c15904b plus our commits (`NCNN_QUEUE_GLOBAL_PRIORITY`, see above; C API calls for the GPU instance and winograd63).
`SIMPLEVK=OFF` because its stub Vulkan types clash with `vulkan.h`.
Models: Ultralytics' `yolo11n-grayscale.pt` (YOLO11n trained on grayscale COCO, 1-channel input;
https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo11n-grayscale.pt).
`yolo export model=yolo11n-grayscale.pt format=ncnn imgsz=<2S>,<S>` (ultralytics) → rename `yolo11n-grayscale_ncnn_model/model.ncnn.*`
to `models/yolo11n-grayscale_<S>.ncnn.{param,bin}`, then `python3 packages/roommate-radar/prep_model.py models/yolo11n-grayscale_<S>.ncnn`
→ `yolo11n-grayscale_<S>_rr.ncnn.*`, which roommate-radar loads (attention MatMuls → Vulkan SDPA, since ncnn's MatMul is CPU-only;
1/255 folded into the first conv). prep_model.py also takes the RGB `yolo11n.pt` export (GRAY2RGB folded in too).
All 80 classes are kept: a box is ours only when its best class is person, cat or dog (`detector.rs` `decode`).
The export bakes in the input size, so each S needs its own (build.sh exports the tab's 320, 480 and 640).
Grayscale vs RGB weights on 500 random COCO val2017 images (seed 0), gray, letterboxed and stacked two per 576x1152 input, ncnn
Vulkan on the Frame, pycocotools: mAP50-95 **35.8 vs 32.3** (mAP50 49.3 vs 45.4, AP small/medium/large 15.4/40.6/56.6 vs
13.6/37.9/53.0); speed identical (~26 ms median each, nothing else on the GPU).
YOLO11n on the Adreno 750 (Turnip) for the stacked S x 2S input, SteamVR running: 23 ms @480, 31.4 ms @576, 38.5 ms @640
(37 ms @576 before prep_model.py and the conv options).

## Dead ends (so we don't retry them)

- Cutting the model's class convs to person, cat and dog only: without the other 77 classes, nothing tells a chair
  scoring 0.3 as a person from one; chairs became people.

- Devkit `compat_tool` `SteamLinuxRuntime_4-arm64` (the devkit client's alias): Steam records the mapping but runs the
  app natively, as no tool has that name (the runtime registers as `steamlinuxruntime_steamrt4-arm64`). `steamrt4-any`
  works. `create-shortcut`'s `response=` must be a file in a new directory (`/tmp/create-shortcutXXXX/registered`,
  URL-encoded), or Steam says "bad response path".
- `IsApplicationInstalled(steam.overlay.<id>)` to see whether the manifest is there: SteamVR makes up an entry for a
  connected app (launch type `binary`, not a dashboard overlay), so it's always true. A manifest's entry is a dashboard
  overlay. And a manifest added after the overlay connected is skipped until SteamVR restarts ("already described in
  manifest builtin"), hence `--register` as its own process first.
- Camera frames without root from outside vrserver: as `steamos` you can open `/dev/video0`/`3` and `VIDIOC_QUERYBUF`
  them, but reopening XRService's dmabufs via `/proc/<pid>/fd/N` is ENXIO and the memfds behind its udmabufs are
  already closed, so only `pidfd_getfd` works, which Yama allows only to an ancestor (vrserver) or `CAP_SYS_PTRACE`.
- YOLO attention as per-head ncnn `Gemm` (both inputs dynamic): ~21 ms per 64×648×648 product on Turnip. SDPA is ~5 ms.
- ncnn Vulkan options: fp16 arithmetic, winograd, shader local memory, subgroup ops all best at their defaults.
  No fp8 on the Adreno 750 (ncnn reports `fp8-cm=0`). int8 (`ncnn2table`/`ncnn2int8`, all layers still on the GPU):
  50 ms @576 vs 32 ms fp16; every int8 conv is ~1.7x slower (no cooperative matrix on Turnip).
- YOLO on the Hexagon NPU (QNN HTP): no FastRPC on the Frame. Valve's `dsp-sm8650` driver owns `/dev/cdsp`
  and `/dev/adsp` (DT `qcom,sm8650-hexagon`, no glink/fastrpc nodes, no remoteproc), and SteamVR's `dsp_service`
  boots its own stripped cDSP image (`drivers/cv/bin/linuxarm64/cdsp_sm8650.mbn`) that eyetracking (`-b CDSP`) talks
  to. Qualcomm's `cdsp.mbn` (has the unsigned-PD shell built in) sits unused in `/lib/firmware/qcom/sm8650`.
