# Prior art: stereo depth, view matching, tracking and smoothing

Research notes (October 2026) on the five hard parts of Roommate Radar's pipeline:

1. depth for a detected box from the SLAM stereo pair;
2. matching left-view boxes to right-view boxes;
3. matching detections to tracks and removing duplicates;
4. removing jitter from boxes over time;
5. drawing boxes between detections.

Each section covers what the code does today, what the literature and open-source projects do, and what fits the
budget: a few ms of CPU per detector frame, and no GPU beyond YOLO itself, because a game shares the SoC. Section 6
proposes one coherent design. Section 7 lists the measurements that should decide between options.

Sources are linked inline. "(analysis)" marks arithmetic or reasoning of our own rather than a published result.

**Status.** The "Now" paragraphs below describe the code as it was when this was written. Since then, these have been
implemented:

- ray-aligned noise, Mahalanobis gating, one 6-state filter per track, and drawing from its states (sections 3–5);
- the matcher changes (section 2), patch-grid depth and floor contact (section 1);
- birth suppression, track merging, visibility-aware track management, the class posterior and fades;
- NMS among our classes only;
- the developer trace.

Still to do:

- the noise constants fitted from section 7's recordings (they are educated guesses, marked in `measure.rs` and
  `stereo.rs`);
- ROI matching at a higher focal length;
- LK sub-pixel refinement;
- a per-track height prior;
- IMM;
- fixed-lag smoothing;
- KLT between YOLO runs.

The maximum range is now 6 m, and the default resolution is 480 px.

**Found by measuring (October 2026), not in the literature above:** the dominant depth error was neither noise nor
matching but the rectification. The factory extrinsics are about 1° off between the two SLAM cameras (XRService refines
them online but doesn't publish them), which made stereo read 17–35% too close. `selfcal.rs` now estimates and corrects
that while running. The calibration app (`roommate-radar --calibrate`, `make deploy-calibrate`) records what the
pipeline sees of a person holding the left controller (tracked well only to ~1.6 m; tape distances beyond). It keeps no
images: only numbers, including patch-match offsets computed on the device.

## 0. Numbers that frame everything

Measured on the Frame:

- **SLAM cameras' native focal length:** the KB4 calibration in `/persist/xrservice.json` gives fx ≈ 328 px for
  `slam_right` and 327 px for `slam_left`, at 1056×1024.
- **Detector timings** from the log, at 640 px and 8 Hz with SteamVR running:

  | Stage | Time per run |
  |-|-|
  | YOLO | ≈ 47 ms |
  | Rectify, CPU, both views | 5–6 ms |
  | "Match": decode + NMS + `match_views` | ≈ 3 ms |
  | Capture → result age | 60–100 ms |

Stereo geometry: B = 105.6 mm and f = (S/2)/tan 50°. Depth error is σ_z = z²·σ_d/(f·B):

| | f (px) | f·B (px·m) | disparity at 2 / 4 / 8 m | σ_z at 4 m / 8 m, σ_d = 0.25 px |
|-|-|-|-|-|
| S = 320 | 134 | 14.2 | 7.1 / 3.5 / 1.8 px | 0.28 / 1.13 m |
| S = 480 | 201 | 21.3 | 10.6 / 5.3 / 2.7 px | 0.19 / 0.75 m |
| S = 640 | 268 | 28.4 | 14.2 / 7.1 / 3.5 px | 0.14 / 0.56 m |
| native center | 328 | 34.6 | 17.3 / 8.7 / 4.3 px | 0.12 / 0.46 m |

Three consequences:

- **At 8 m, stereo depth is the weakest measurement we have.** Bearing is good: σ_lateral ≈ z·σ_u/f, about 6 cm at
  8 m for 1 px. Depth is metres. The error ellipsoid is a long needle along the viewing ray, about 20× longer than it
  is wide at 8 m. The current isotropic model (`measure.rs`: σ = 0.05 + 0.02·z², and a 0.7 m sphere as the gate in
  `tracker.rs`) ignores that shape. Most of what follows is a consequence of fixing it.
- **The noise model doesn't depend on the S setting.** 0.02·z² corresponds to σ_d ≈ 0.28 px at S = 320, but 0.57 px
  at S = 640 (analysis).
- **The 640 view already samples as finely as the sensor does near the middle, and finer at the edges.** A pinhole
  view at f = 268 needs f/cos²θ px/rad at field angle θ, which exceeds the fisheye's ≈ 328 px/rad beyond about 25°
  (analysis). Matching at native resolution would therefore gain little over S = 640, but about 2.4× over S = 320.

## 1. Stereo depth for a box

### Now

`stereo.rs::refine_disparity` works at the rectified S × S resolution:

1. Take the initial disparity from the box edges: d0 = mean of Δx0 and Δx1.
2. Score NCC over the central 60% of the left box, subsampled to about 24×24, at d0 ± 6 px.
3. Fit a parabola for the sub-pixel peak, and accept it if NCC ≥ 0.6.

### Prior art

**Photometric alignment over a chosen region of interest.** Stereo R-CNN (Li, Chen, Shen, CVPR 2019,
[arXiv 1902.09738](https://arxiv.org/abs/1902.09738)) builds a coarse 3D box from paired detections, then refines
only the object-center depth by photometric alignment.

- It deliberately uses a **valid RoI**: the lower half of the box, between keypoints. The aim is to exclude
  background.
- The alignment step lifts KITTI AP_3d@0.7 from 7.75 to 32.0.
- Our NCC step is the same idea. The leverage is in *which pixels* go into the alignment.

**Learned object-level stereo** (Disp R-CNN, YOLOStereo3D, TLNet, Stereo CenterNet) all need desktop GPUs and
retraining. They don't fit. What transfers from Disp R-CNN
([CVPR 2020](https://openaccess.thecvf.com/content_CVPR_2020/html/Sun_Disp_R-CNN_Stereo_3D_Object_Detection_via_Shape_Prior_Guided_CVPR_2020_paper.html))
is the principle: compute disparity on foreground pixels only.

**Product practice.**

- **DepthAI / OAK-D**
  ([SpatialLocationCalculatorConfig](https://docs.luxonis.com/software-v3/depthai/depthai-components/messages/spatial_location_calculator_config.md))
  shrinks the box by a `BoundingBoxScaleFactor`, then takes the **median** of a dense depth map inside it. MIN and
  MODE are also offered, MIN for objects with holes.
- **ZED SDK** ([object detection](https://www.stereolabs.com/docs/object-detection/)) has a documented failure: the
  box snaps to a nearer occluder. Stereolabs' fix is to pass a segmentation mask.

**Robust aggregation.** Section 2 of the matching research covers this. The cheap, well-trodden form:

- lay a grid of about 16–40 small patches (7×7) over the shrunken box;
- run a 1-D search for each patch;
- keep a patch only if it has enough texture (gradient energy) and a unique peak (SGBM's `uniquenessRatio`,
  [OpenCV](https://docs.opencv.org/4.x/d2/d85/classcv_1_1StereoSGBM.html));
- take the **mode, or the nearest large cluster** of patch disparities, not one big window.

A person is usually the nearest large cluster in their box. Cost is about 20k MACs, well under 0.1 ms.

**Sub-pixel accuracy.**

- **Pixel locking.** Parabola fits are biased toward integer disparities
  ([Shimizu & Okutomi, ICCV 2001](https://scispace.com/papers/precise-sub-pixel-estimation-on-area-based-matching-2e6fty9vrx)).
  - At 1.8 px of disparity, a 0.1–0.2 px lock is 0.5–1 m at 8 m.
  - Their cheap fix: also fit with one image shifted by half a pixel, and average the two estimates.
- **Lucas-Kanade / ECC refinement of the single shift.** Gauss-Newton on (I_L − I_R(x + d)), with gain and bias
  normalized, reduces pixel locking substantially
  ([Stein, Huertas & Matthies, ICRA 2006](https://publications.ri.cmu.edu/attenuating-stereo-pixel-locking-via-affine-window-adaptation);
  [ECC](https://sites.google.com/site/georgeevangelidis/ecc)).
  - It also gives a variance for free: σ_d² ≈ σ_noise²/ΣI_x².
- **Achievable precision.** Theory allows about 1/20 px on well-textured patches
  ([Sabater, Morel & Almansa 2011](https://epubs.siam.org/doi/10.1137/100797849)). Expect 0.1–0.2 px on a textured
  torso in our dim frames, and 0.5 px or worse on plain clothing.
- **Census vs NCC.** Census is the most radiometrically robust cost
  ([Hirschmüller & Scharstein](https://vision.middlebury.edu/~schar/papers/evalcosts-pami08.pdf)). It is fine for the
  integer search, but its cost curve is steppy, so take the sub-pixel value from the LK/ECC step instead.

**Resolution.** σ_z ∝ 1/f. YOLO can stay at 320 while matching happens on a separately rectified *box ROI*.

- Use the existing KB4 map in `calib.rs` with a larger f, up to ≈ 270–330.
- Choose f per box so that the expected disparity is at least about 5 px. This is Gallup et al.'s
  [variable baseline/resolution stereo](https://www.cvlibs.net/projects/autonomous_vision_survey/literature/Gallup2008CVPR.pdf)
  idea: far boxes get high resolution.
- A person at 8 m is about 70×30 px at f = 270. Rectifying both views' ROIs is tens of thousands of pixels: well
  under 1 ms (analysis; the full 2×640² remap measures 5–6 ms).

**Depth cues that don't depend on the baseline.**

- **Ground contact.** In the standing universe the floor (y = 0) and the head pose are known exactly. Intersect the
  ray through the box's bottom-center with the floor.
  - Sensitivity is ∂z/∂v ≈ z²/(f·H) with H ≈ 1.6 m: effectively a 1.6 m baseline instead of 0.106 m.
  - At 8 m and S = 320, one pixel of box-bottom error is about 0.3 m; one pixel of disparity error is about 4.5 m.
  - Automotive work struggles with camera pitch here
    ([Ground Plane Matters, arXiv 2211.01556](https://arxiv.org/pdf/2211.01556); Mobileye
    [Stein, Mano, Shashua, IV 2003](https://cris.huji.ac.il/en/publications/vision-based-acc-with-a-single-camera-bounds-on-range-and-range-r/)).
    We have pitch for free from the VR tracking.
  - **Fails** when the feet are hidden (couch, table, bed), when the person is sitting or a pet is on furniture, or
    when the bottom edge is clipped by the view. Gate it: use it only when the bottom edge is unclipped and the
    result agrees with stereo within about 3σ.
- **Known height.** z = f·H/h_px.
  - MonoLoco ([ICCV 2019](https://openaccess.thecvf.com/content_ICCV_2019/html/Bertoni_MonoLoco_Monocular_3D_Pedestrian_Localization_and_Uncertainty_Estimation_ICCV_2019_paper.html))
    puts the error floor from adult-height spread at roughly 0.6 m on KITTI, and beats stereo for far pedestrians.
  - A population prior is useless for sitting people, children and pets.
  - The useful version is **per track**: learn each track's own height while it is close (1.5–3 m), where stereo is
    good to centimetres, then use it as a range measurement when it is far.
  - [Krejčí et al., FUSION 2024](https://arxiv.org/abs/2403.11978) put a height prior in a monocular UKF.

**Error model and bias.**

- Triangulation error is a full 3D Gaussian, not a scalar
  ([Matthies & Shafer 1987](https://www.ri.cmu.edu/publications/error-modeling-in-stereo-navigation)):
  - σ_lateral = z·σ_u/f;
  - σ_ray = z²·σ_d/(f·B).
- z = fB/d is convex, so noise biases range *far*: E[z] ≈ (fB/d)(1 + σ_d²/d²). That is +3 to +8% at 8 m.
- Corrections:
  - [Sibley, Matthies & Sukhatme](https://robotics.usc.edu/publications/463) (series expansion);
  - [Freundlich et al., CVPR 2015](https://openaccess.thecvf.com/content_cvpr_2015/html/Freundlich_Exact_Bias_Correction_2015_CVPR_paper.html)
    (exact, for quantized disparity);
  - or filter in disparity / inverse depth, where the noise is Gaussian
    ([6D-Vision, Franke et al. 2005](https://www.cvlibs.net/projects/autonomous_vision_survey/literature/Franke2005DAGM.pdf);
    [Civera et al., TRO 2008](https://www.doc.ic.ac.uk/%7Eajd/Publications/civera_etal_tro2008.pdf)).

**Dense stereo is not worth it.**

- SGBM at 320² is likely several to tens of ms on one ARM core. It is no more precise than sparse matching at the
  same f; it only helps segmentation.
- Learned dense stereo (HITNet, Fast-ACVNet, MobileStereoNet) is far beyond a shared Adreno.
- Headset depth APIs, such as Meta's Depth API, are coarse and recommended only to about 4 m
  ([UploadVR](https://uploadvr.com/quest-3-depth-api-mixed-reality-dynamic-occlusion)).

### What fits, ranked

1. **Anisotropic, per-measurement noise.** σ_ray from σ_d and f·B, which accounts for the S setting; σ_lateral from
   z/f. Pure math. The largest effect lands in the tracker (section 3).
2. **Ground-contact range**, gated against stereo, for boxes whose bottom edge is unclipped. Free.
3. **Patch grid + mode/nearest cluster** instead of one window. Keep d0 from the box edges as a separate, fused
   measurement: it is a silhouette disparity that is immune to background, only noisy.
4. **LK/ECC sub-pixel refinement**, with σ_d taken from its Hessian. At the very least, use Shimizu–Okutomi's
   half-shift trick.
5. **ROI rectification at f ≈ 270–330** for matching when S < 640.
6. **Per-track height → range at a distance.**
7. **Not now:** a YOLO-seg mask (GPU cost, re-export), dense or learned stereo.

## 2. Matching boxes across the two views

### Now

1. Per-view NMS, class-agnostic.
2. Greedy matching, left box by left box. Cost = |Δy|/h + relative Δh + relative Δw (+ 0.2 if the classes differ).
3. Gates: dy < 0.15, dsize < 0.5, disparity 0.3 px to f·B/0.2 m (71 px at S = 320).
4. Unmatched left boxes become direction-only `Obj`s, which `measure.rs` then drops. Right-only boxes are lost.

### Prior art

**OC Stereo** (Pon et al., ICRA 2020, [arXiv 1909.07566](https://arxiv.org/abs/1909.07566)) is the closest
published pipeline to ours. It uses an off-the-shelf 2D detector on both views, then:

- an **appearance cost**: SSIM of every candidate pair, with each crop resized to 128²;
- a **disparity gate predicted from box height**: depth correlates with box height, so pairs more than 3σ off are
  rejected;
- unmatched boxes are dropped as false positives.

Association took 9 ms of a 350 ms pipeline, and matched Stereo R-CNN's learned association in 2D AP.

**Stereo R-CNN** shares v and h between the two views' regressions. On a rectified pair, the top edges, bottom edges
and heights *must* agree, and Δx0 ≈ Δx1. We currently use those constraints only softly.

**Hungarian assignment vs greedy.** Optimal assignment matters exactly in the side-by-side case. Worked example
(analysis):

- Two people 0.6 m apart at 3 m are about 27 px apart in the image.
- A swapped pair has a disparity of about 31 px, i.e. z ≈ 0.45 m.
- The current gate accepts up to 71 px.
- `tracker::assign` already implements gated Hungarian matching.

**Class-size disparity gate** (OC Stereo's idea, made class-specific). A box h px tall on an object H m tall has
d ≈ h·B/H.

- Accept d ∈ [h·B/H_max, h·B/H_min]. Starting ranges: person 0.5–2.1 m, dog 0.25–1.0 m, cat 0.15–0.5 m.
- In the example, a 76 px person at 3 m allows 3.8–16 px, so 31 px is rejected.
- Occlusion and truncation only *shrink* h, so the lower bound is always safe.
- The gate also rejects people on TV screens and posters: small for their measured depth.

**Truncation.** A box clipped by the view edge, or by the fisheye's invalid region (the `Rectifier` marks it with
map x < 0, often well inside the S × S frame), has a wrong edge in that view.

- Use only edges that are unclipped in *both* views for disparity, dy and size.
- KITTI flags truncation separately for the same reason.

**Stereo agreement is evidence an object is real.** Keller, Enzweiler & Gavrila report 4–5× fewer false positives
at equal detection rate, stereo vs mono
([IV 2011](https://markus-enzweiler.de/downloads/publications/keller11_benchmark_iv.pdf)).

- Score fusion WBF-style ([arXiv 1910.13302](https://arxiv.org/abs/1910.13302)):
  - matched pair: mean or max of the two scores;
  - unmatched but inside both views' overlap: half the score;
  - unmatched and outside the other view's FOV: score unchanged, because it *couldn't* be seen there.
- Noisy-OR overconfides: the two views share the same network and nearly the same image.

**Detect in one view, search the other** (AMZ Formula Student,
[arXiv 1809.10099](https://ar5iv.arxiv.org/html/1809.10099), took the median of matched features).

- It halves the YOLO cost.
- It loses the right-only FOV, the second chance at detection, and the false-positive filter.
- Worth it only as a "low GPU" setting.

**Appearance from YOLO's own features** (DEFT, JDE-style): untrained COCO features separate two people in similar
clothes no better than pixel NCC. Skip.

### What fits, ranked

1. **Gated cost matrix + `tracker::assign`** instead of the greedy loop.
2. **Class-size disparity gate** in place of the 0.2 m floor.
3. **Truncation-aware edges**: a per-row and per-column valid range precomputed from the `Rectifier` map; per-edge
   residuals instead of a width difference.
4. **NCC in the cost**: run a cheap ZNCC for every gated pair (1–3 per box), add 1 − NCC to the cost, and floor it at
   about 0.3.
5. **Classify unmatched boxes by expected visibility** in the other view; fuse scores and vote on class across the
   pair.
6. **Use one-view detections** instead of discarding them. Range comes from ground contact or the track's height;
   otherwise apply a bearing-only update. They may only *extend* existing tracks, never start one, as in ByteTrack's
   second stage.

Cost for ≤ 20 boxes per view: well under 1 ms.

## 3. Detections to tracks, and deduplication

### Now

- **Tracker** (`tracker.rs`): ByteTrack-style.
  - Five independent constant-velocity Kalman filters per track: x, y, z, w, h.
  - Matching is Hungarian on Euclidean 3D center distance, under a fixed **0.7 m** gate: high-confidence detections
    (≥ 0.5) first, then low ones.
  - Birth needs 2 hits; a track dies 1 s after its last match.
- **Duplicates** are hidden at draw time (`boxes.rs` + `detector::duplicate`): a track whose angular box overlaps one
  already drawn is skipped.

### Root cause of the duplicates (analysis)

σ_ray reaches 0.55 m at 5 m and 1.3 m at 8 m. Beyond about 5.5 m, a 1σ depth error misses a 0.7 m sphere. The new
detection births a second track *along the same ray* while the old one coasts. The draw-time filter treats the
symptom.

### Prior art

**Mahalanobis gating with realistic covariance.**

- **Chiu et al.** ([arXiv 2001.05673](https://arxiv.org/abs/2001.05673)) use Mahalanobis distance with S = HPHᵀ + R.
  - On nuScenes at 2 Hz, AMOTA goes from 52.7 (3D IoU) to 56.1, and pedestrians from 58.7 to 75.5.
  - With hand-set rather than fitted covariances it drops to **41.7**. Covariance quality matters more than the
    choice of metric.
- **UCMCTrack** ([arXiv 2312.08952](https://arxiv.org/abs/2312.08952), [code](https://github.com/corfyi/UCMCTrack))
  is the closest analogue to our problem. It maps image-plane noise through the projection Jacobian into a
  *correlated, elongated* ground-plane R, and scores pairs with d² + ln|S|.
  - Ablation (MOT17-val IDF1):

    | Variant | IDF1 |
    |-|-|
    | IoU | 77.1 |
    | Mahalanobis with isotropic R | 75.9 (worse than IoU) |
    | + correlated R | 79.7 |
    | + process-noise compensation | 82.2 |

  - The ln|S| term stops a vague far track from claiming everything.
  - Runs at over 1000 FPS on CPU.
- **For us:**
  - One 6-state filter over 3D position and velocity.
  - R = σ_ray²·uuᵀ + σ_lat²·(I − uuᵀ), with u the unit ray from the eye at capture time.
  - Gate νᵀS⁻¹ν < χ²₃(0.99) = 11.3; cost d² + ln|S|; keep a hard cap of about 2 m as a sanity check.
  - A detection at the wrong depth along an existing track's ray then falls *inside* that track's gate and updates
    it, so the duplicate never forms.
- SimpleTrack ([arXiv 2111.09621](https://arxiv.org/abs/2111.09621)) prefers GIoU for lidar boxes with heading. That
  doesn't transfer to upright billboards.

**ByteTrack's second stage** ([arXiv 2110.06864](https://arxiv.org/abs/2110.06864)) is already implemented.

- SimpleTrack's refinement: low-score matches **keep the track alive but don't update its motion state**, because
  weak boxes are occluded or blurred.
- On WOD pedestrians, ID switches fell from 0.96% to 0.42%.
- We currently run the full update on them.

**OC-SORT** ([arXiv 2203.14360](https://arxiv.org/abs/2203.14360),
[ocsort.py](https://github.com/noahcao/OC_SORT/blob/master/trackers/ocsort_tracker/ocsort.py)):

- **ORU.** When a lost track is re-found, re-run the filter along a virtual constant-velocity path between the two
  observations. This removes the velocity and covariance drift built up while coasting.
- **OCR.** A last matching pass against unmatched tracks' *last observation* rather than their prediction. It catches
  a person who stopped.
- MOT17-val HOTA: 64.9 → 66.3 with ORU.
- At 4 Hz the cheap form is enough: on re-acquisition, refit velocity from the two observations and shrink P.

**Other SORT descendants**, for their transferable weak cues:

- **Hybrid-SORT** ([arXiv 2308.00783](https://arxiv.org/abs/2308.00783)):
  - Height consistency helps (+1.6 HOTA).
  - Width hurts (−5): "width varies irregularly due to pose changes or limb movements". Trust h over w.
- **BoT-SORT** ([arXiv 2206.14651](https://arxiv.org/abs/2206.14651)): w and h directly in the state, noise scaled by
  box size.
- **C-BIoU** ([arXiv 2211.14317](https://arxiv.org/abs/2211.14317)): a tight gate first, then a looser one for the
  leftovers. A 3D cascade of two χ² levels is the analogue.

**Low frame rate.**

- At 10× frame subsampling, ByteTrack falls from 67.5 to 57.6 HOTA on MOT17 and OC-SORT from 66.1 to 56.1
  ([ColTrack, arXiv 2308.05911](https://arxiv.org/abs/2308.05911)).
- The cause is image-space IoU going to zero. We match in world space with ego-motion removed, so 4 Hz is far less
  harmful to us (analysis).
- SimpleTrack saw the Kalman filter's advantage shrink at 2 Hz: smoothing gains are small at low rates.

**Appearance.**

- ReID helped a lot at 5 fps in dense traffic (BDD100K: ID switches 27.9k → 9.1k).
- It *hurt* at 30 fps on MOT17.
- With ≤ 3 objects it would only help re-acquisition. Skip it until logs show swaps.

**Keep tracks alive, but draw only what's verified.**

- [ImmortalTracker](https://arxiv.org/abs/2111.13672) never kills tracks but outputs only tracks matched this frame.
  On WOD, 2,890 of 2,891 vehicle ID switches were early termination, and immortal tracks removed 96% of them.
  SimpleTrack found early termination behind 91% of pedestrian ID switches.
- Our MAX_AGE does two jobs: "keep for re-association" and "show".
- Random-finite-set trackers make the detection probability depend on the state: zero outside the FOV
  ([LeGrand & Ferrari](https://lisc.mae.cornell.edu//LISCpapers/KL_Bounded_Fields-of-View_FISST.pdf)). A miss where
  nothing could be seen then costs no existence.
- IPDA ([Musicki et al. 1994](https://doi.org/10.1109/9.293185)) gives a cheap recursive existence probability r.
  - On a miss: r ← (1 − P_D·P_G)·r/(1 − P_D·P_G·r).
- For us, whether a predicted track is inside the rectified views is *known exactly* from the head pose.

**Deduplication beyond NMS.**

- **3D measurement fusion after stereo.** Merge measurements with (m₁ − m₂)ᵀ(R₁ + R₂)⁻¹(m₁ − m₂) < χ²₃, averaged
  by score (WBF). This catches a box split in one view but merged in the other.
- **Track-to-track test.** d² = Δxᵀ(P₁ + P₂)⁻¹Δx ([Bar-Shalom 1981](https://doi.org/10.1109/TAC.1981.1102635),
  without the cross-covariance, which makes it more conservative). Below χ²₃(0.99) for 2 frames: merge into the track
  with more hits.
- This replaces the draw-time angular filter with a fix in the state.

**Class flicker.**

- Today `tr.cls = d.cls` on every match.
- Use a per-track score vector decayed like `alpha()` instead, display its argmax with a switching margin
  ([Roboflow class lock](https://inference.roboflow.com/workflows/blocks/track_class_lock/) uses the same
  hysteresis), and make association class-*soft*.

### Found while reading the code (untested)

- **`detector::nms` is class-agnostic over all 80 COCO classes.** A person sitting on a couch whose box is ≥ 80%
  inside a higher-scoring "couch" box is suppressed by the containment test. The same goes for a cat on a bed or a dog
  in a "chair" box. The "laptop/tv on one object" reason only applies among classes we don't show. Suggested fix: run
  NMS among {person, cat, dog} only, after `decode` keeps just those three. This also cuts `decode`'s 80-way argmax
  loop, part of the ≈ 3 ms "match" time at 640.
- **Low-score detections run the full Kalman update** (see SimpleTrack above).

### What fits, ranked

1. **Ray-aligned R + Mahalanobis gate (χ²₃) + d² + ln|S|** in one 6-state position/velocity filter. Keep w and h
   separate, with h trusted over w. This fixes the duplicates at their source.
2. **Low-confidence matches only extend** a track (update `last_seen`, not the state).
3. **Split alive from shown.** FOV-aware P_D and an existence score; keep out-of-view tracks for seconds but never
   draw them; draw only tracks matched within about 0.5 s and predicted inside the views.
4. **Track merge test + no birth inside an existing track's gate.** Then delete the draw-time `duplicate` filter.
5. **3D measurement fusion** after stereo.
6. **Cheap ORU/OCR** on re-acquisition.
7. **Class posterior** with decay and hysteresis.
8. **Truncation-aware size**: skip or inflate the w/h update when the box touches a view edge, and never birth from a
   truncated box.

Total cost for ≤ 3 tracks: well under 0.1 ms.

## 4. Removing jitter from boxes over time

### Now

`tracker::remember` stores each match and smooths two things with an EMA:

- w and h;
- the range from the current eye, with α = 1 − 0.1^(dt/1 s).

Bearing is raw. The Kalman filter's state is used only for matching.

Using the *current* eye for both samples means your own head motion isn't smoothed away. That part is right.

### Problems with that (analysis)

- **The EMA lags a ramp.** Its time constant is τ = 1/ln 10 ≈ 0.43 s, so it lags τ·v.
  - Someone walking toward you at 1.4 m/s is drawn about 0.6 m too far away from the EMA alone, before the playback
    delay (section 5).
  - The error always points toward "farther than reality", which is the wrong direction for a safety overlay.
  - Anything with a velocity state removes ramp lag: Holt/double exponential smoothing (equal to a Kalman filter for
    prediction at 135× less cost, [LaViola 2003](https://www.eecs.ucf.edu/~jjl/pubs/vr2003_laviola.pdf)), α-β with
    Kalata's gains, or the tracker's own Kalman filter.
- **Two models of the same object.** The association filter and the display smoother disagree.

### Prior art

**1€ filter** (Casiez, Roussel, Vogel, CHI 2012; [page and tuning guide](https://cristal.univ-lille.fr/~casiez/1euro),
[code](https://github.com/casiez/OneEuroFilter)).

- A low-pass filter whose cutoff rises with speed: fc = fcmin + β·|ẋ|. Jitter is visible when things are slow, lag
  when they are fast.
- Tuning: with β = 0, lower fcmin until a still target stops jittering; then raise β until fast motion stops lagging.
- Always pass real timestamps; the parameters are otherwise frame-rate dependent.
- A good *last* stage on what is drawn. It still lags a ramp.

**IMM for stop-and-go.**

- [Schneider & Gavrila 2013](https://www.springerprofessional.de/en/pedestrian-path-prediction-with-recursive-bayesian-filters-a-com/4166076)
  compare constant-position + constant-velocity (CP + CV) models on pedestrians who stop and start.
- MATLAB's [`initcvimm`](https://www.mathworks.com/help/fusion/ref/initcvimm.html) ships the simple form: two CV
  filters with low and high Q.
- It stops the overshoot when someone halts, and makes "standing still", the commonest indoor state, rock solid. Cost
  is microseconds.

**Box size.**

- **Draw angular size, not metric size** (analysis).
  - The box's angular width and height are pixel-accurate. Metric w = θ·d inherits the z² depth noise.
  - Smooth θ, then size the quad as θ × displayed range. Range noise then shows only as stereo disparity, not as the
    box breathing.
- **Grow fast, shrink slow**, and freeze a dimension while its edge is clipped, so partial occlusion doesn't visibly
  shrink the box.
- Seq-NMS ([arXiv 1602.08465](https://arxiv.org/abs/1602.08465)) and the feature-level video detectors are offline
  or need retraining.

**Perception.**

- Jitter of world-locked content is perceptible at a few arcmin:
  - [Wilmott et al., ISMAR 2022](https://paper.nweon.com/?p=13308);
  - peak sensitivity of **0.77 arcmin at 12 Hz** ([Levulis et al. 2025](https://arxiv.org/abs/2504.16295)), where even
    sub-threshold jitter raised discomfort over 90-minute sessions.
- 2 cm of lateral noise at 3 m is about 23 arcmin (analysis), so raw bearing is visible and needs light smoothing
  too.
- Head-motion latency thresholds (about 15 ms) don't apply: the compositor reprojects world-locked overlays. What's
  left is object-motion lag, which has no published threshold.
- For a safety overlay the honest metric is *range error of an approaching person at display time*.

## 5. Drawing boxes between detections

### Now

`boxes::spline_at` draws a time-based Catmull-Rom curve through the last 4 (EMA'd) samples at t_play = now − lag,
where lag ≈ capture latency + one detection period (≈ 300–350 ms). Drawing at the Kalman prediction for "now" looked
worse.

### Prior art

**Game-networking entity interpolation is the same problem.**

- Valve's Source engine draws 100 ms in the past at 20 Hz snapshots, i.e. two periods
  ([Valve wiki](https://developer.valvesoftware.com/wiki/Interpolation)).
- Gaffer On Games recommends about 3 periods, plus jitter margin
  ([Snapshot interpolation](https://gafferongames.com/post/snapshot_interpolation/)).
- Our 1.3 periods is the aggressive end, which is fine because the detector doesn't "drop packets". A missed
  detection does need an explicit coast policy, though.
- Gaffer found **Hermite with sent velocities** visibly better than linear interpolation at 10 Hz. Extrapolating
  200 ms was "just not very good".
- That matches our finding: [Azuma & Bishop 1995](https://history.siggraph.org/?p=118761) show prediction error grows
  quickly with the horizon, and ours is 300+ ms on noisy stereo velocities.

**Catmull-Rom interpolates the noise.** It passes exactly through each sample, so with raw bearing every detection's
error lands on screen. Interpolating *filtered* states fixes that; so would an approximating B-spline.

**Fixed-lag smoothing.** We already choose to display at t − L, which is precisely what a fixed-lag Kalman (RTS)
smoother estimates optimally ([Labbe ch. 13](https://nbviewer.org/github/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/v2.0/13-Smoothing.ipynb);
[Särkkä](https://users.aalto.fi/~ssarkka/course_k2012/full_course_booklet_2012.pdf)).

- Run a forward Kalman filter over the last N = 4–8 detections, then a backward pass. That is about 10 µs per track.
- At render time, take a cubic Hermite between the two smoothed states (position + velocity) that bracket t_play.

**Partial extrapolation.** If a lag smaller than "latency + one period" is ever wanted, draw at t − L′ and
extrapolate the remainder with a velocity that decays to zero over about 200 ms. Use it for fast tracks only.

**Getting measurements between YOLO runs.** This is the only way to cut the "+1 period" lag itself.

- MediaPipe runs the detector on subsampled frames and a CPU box tracker on every frame
  ([Box Tracking](https://mediapipe.readthedocs.io/en/latest/solutions/box_tracking.html)).
- Median Flow ([Kalal et al., ICPR 2010](https://dspace.cvut.cz/handle/10467/9553)): pyramidal LK on a point grid in
  the box, a forward-backward check, then the median displacement and scale. It reports its own failures.
- MOSSE ([Bolme et al., CVPR 2010](https://mlanthology.org/cvpr/2010/bolme2010cvpr-visual)) runs at 669 fps on a
  2010 laptop CPU.
- Our cameras deliver normal-exposure frames at roughly 30–50 Hz, and ego-rotation per frame is known exactly, so
  flow converts straight to a world bearing.
- Tracked points seen in both views also give disparity.
- Cost estimate: under 1 ms per box per frame on a big core, plus the ROI remap (analysis).
- It is the largest engineering item: a Rust KLT with no OpenCV, about 300 lines. Do it only if measurements show
  lag, not jitter, is the complaint.

**Presentation.**

- Fade in over about 200 ms on confirmation, and fade out while coasting instead of popping. Apple fades people in
  ([Road to VR](https://www.roadtovr.com/?p=116165)).
- Hysteresis on range, play-area and confidence thresholds.
- Pad the box by k·σ_lateral so uncertainty reads as size, not motion.
- Give the label its own deadband + critically damped follower ([Unity SmoothDamp](https://docs.unity.com/en-us/engine/6000.3/script-reference/unityengine/mathf/smoothdamp)).
- Take billboard yaw from the smoothed position.

## 6. One design that follows from all of this

The pieces above fit together. Most of the "prototype" feel comes from three places: an isotropic noise model, two
models of each object (association filter + display smoother), and policies that are stacked fixes for symptoms
(draw-time dedup, the EMA, MAX_AGE doing two jobs).

```
per view   decode (person/cat/dog only) → NMS among those
stereo     gated cost matrix (class-size disparity gate, truncation-aware edge residuals, 1−ZNCC) → assign()
           unmatched: in the other view's FOV → weak; outside → keep score
depth      per pair: patch grid at f ≈ 270–330 on the box ROI, mode/nearest cluster, LK sub-pixel, σ_d from Hessian;
           fuse box-edge d0, ground contact (gated), track height (gated) by inverse variance
measure    z = (bearing, range) + R = σ_ray²uuᵀ + σ_lat²(I−uuᵀ); fuse measurements within χ²₃ (WBF)
track      one CV (or 2-model IMM) Kalman filter on 3D position/velocity + h, w (h trusted more);
           Mahalanobis χ²₃ gate, cost d² + ln|S|; high then low (low: extend only); birth suppressed inside gates;
           existence r with FOV-aware P_D; merge statistically identical tracks; class posterior
display    fixed-lag smoothed states → Hermite at t_play; size = smoothed angular size × range;
           shown iff r high ∧ matched ≤ 0.5 s ago ∧ inside the view; fade in/out; label on its own follower
```

What it deletes: the range/size EMA, the Catmull-Rom curve through raw samples, `duplicate()` at draw time, and the
isotropic σ.

### Order to do it in

1. **Recording + replay harness first** (section 7). Without it, every change below is tuned by eye on the headset.
   With it, variants run as `cargo test`-style replays on the Mac against the same data.
2. **Noise model + Mahalanobis tracker + display from filter states** (sections 3 and 4). This is the biggest visible
   change, and it is CPU math only.
3. **Matcher**: Hungarian, size gate, truncation, NCC; NMS on our classes only.
4. **Depth**: patch grid, LK, ground contact, ROI resolution.
5. **Existence/visibility, merge, class posterior, presentation.**
6. **Only if data says lag dominates:** KLT between YOLO runs.

## 7. Measurements to take

### Instrumentation to add first

Write one JSON line per detector frame to `/tmp/roommate-radar-trace.jsonl`, behind a developer-build flag:

- capture time, result time, head pose at capture, controller poses;
- every box in both views, with score, class and a clipped-edge flag;
- the stereo cost matrix and chosen pairs;
- per pair: d0, refined d, NCC peak and curve, σ_d;
- the measurement handed to the tracker;
- track states and covariances, and the drawn set.

Every Nth frame, also dump the S × 2S input as a PGM. That is enough to replay matching, depth, tracking and display
offline, and to label ground truth in [CVAT](https://www.cvat.ai/) for
[TrackEval](https://github.com/JonathonLuiten/TrackEval) (HOTA/IDF1).

### Ground truth for free: a controller

A Frame controller's tracked pose is millimetre-accurate.

- A second person holding one at chest height gives the true position for every scenario below, with no tape or
  timing.
- Check that the headset still tracks the controller at those ranges; it is tracked by the same cameras.
- **Switch "Ignore my hands" off for these runs.** Otherwise `measure::holds_controller` drops the walker's box: its
  forearm line runs from *your* shoulder to the controller.
- A tape measure + floor marks is the fallback.

### Scenes to stage

Run each at S = 320 and 640, and at 4 and 16 Hz where it matters.

| # | Scene | Answers |
|-|-|-|
| 1 | Person standing still at taped 1.5, 2, 3, 4, 6, 8 m, centered and near the view edge; 30 s each | σ_d and bias of each depth method (d0, current NCC, patch grid, ROI-res, ground contact, height); fits σ_ray(z), σ_lat(z); pixel locking (histogram of disparity fractions); box-edge jitter per edge |
| 2 | Same as 1, while you sway and step ±0.5 m | Ego-motion cleanly removed? |
| 3 | Walk toward you from 8 m to 1.5 m at a steady pace (about 1 and 1.5 m/s), and away | Range error at display time (the safety metric), duplicate births along the ray |
| 4 | Walk across your view at 3 m and 6 m; walk then stop at a mark; stand then start | Lag (cross-correlation shift), overshoot on stop, onset lag, need for IMM |
| 5 | Walk slowly across the full FOV, through the edge of each view | Depth jumps at truncation; time spent in one view only |
| 6 | Two similar-height people 0.3 / 0.6 / 1 m apart at 2, 4, 6 m; swap places | Stereo mis-pairs (greedy vs Hungarian vs size gate vs NCC), track swaps |
| 7 | Person sitting on a couch; half behind a table or doorway | The class-agnostic NMS suppression above; size gate's H_min; ground contact rejected correctly |
| 8 | Dog called across the room; cat walking; pet on furniture | Small-object depth; ground contact on floor vs furniture; class flicker |
| 9 | Turn away for 1, 3, 10 s and back, with the person still or having moved | Re-acquisition, ghost boxes, need for existence/visibility logic |
| 10 | Posters, a TV showing people, a mirror, coats on hooks; then an empty room for 5 min | False positives with and without the size-vs-stereo check; ghost-seconds |
| 11 | Your own arms and lap in view | Arm rejection still holds; arms don't steal matches |

### Numbers to compute from those runs

- **Depth:** per method and range, bias and σ against the tape or controller. This is the table that decides
  section 1's ranking.
- **Noise model consistency:** after fitting R and Q, the normalized innovation squared should average about 3 for
  3D position. That is the check Chiu et al. show matters most.
- **Jitter:**
  - static person: RMS of the high-passed drawn position, lateral in arcmin (compare with 0.77–3);
  - range in cm;
  - size in %.
- **Lag:** cross-correlation shift between drawn and true position while walking; and **range error at display time
  while approaching**.
- **Overshoot** after stopping: peak and settle time.
- **Product counts per minute:** duplicate-frames, ghost-seconds, show/hide flicker, class switches, drops on
  head-turn.
- **CPU:** p50/p99 of `match_views` + depth + tracker on big and little cores while a game runs. Also the GPU time of
  left-only S² vs stacked S × 2S YOLO, to decide on a "low GPU" mode.
