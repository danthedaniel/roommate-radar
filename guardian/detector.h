// guardian's stereo YOLO detector thread. Reads the two SLAM camera frames published by
// camserver, rectifies each into a virtual pinhole camera aligned with the head frame (so the pair is a rectified
// stereo pair), runs YOLO11n (ncnn, Vulkan) on both views stacked into one image, matches detections across views
// and triangulates depth from disparity. Results (head frame at capture time) go to g_dets.
#pragma once
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
#include <openxr/openxr.h>  // XrVector3f only

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <net.h>

#include "shm.h"

static const char* kCoco[80] = {
    "person",         "bicycle",    "car",           "motorcycle",    "airplane",     "bus",           "train",
    "truck",          "boat",       "traffic light", "fire hydrant",  "stop sign",    "parking meter", "bench",
    "bird",           "cat",        "dog",           "horse",         "sheep",        "cow",           "elephant",
    "bear",           "zebra",      "giraffe",       "backpack",      "umbrella",     "handbag",       "tie",
    "suitcase",       "frisbee",    "skis",          "snowboard",     "sports ball",  "kite",          "baseball bat",
    "baseball glove", "skateboard", "surfboard",     "tennis racket", "bottle",       "wine glass",    "cup",
    "fork",           "knife",      "spoon",         "bowl",          "banana",       "apple",         "sandwich",
    "orange",         "broccoli",   "carrot",        "hot dog",       "pizza",        "donut",         "cake",
    "chair",          "couch",      "potted plant",  "bed",           "dining table", "toilet",        "tv",
    "laptop",         "mouse",      "remote",        "keyboard",      "cell phone",   "microwave",     "oven",
    "toaster",        "sink",       "refrigerator",  "book",          "clock",        "vase",          "scissors",
    "teddy bear",     "hair drier", "toothbrush"};

static std::atomic<bool> g_quit{false};

static uint64_t now_ns() {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return uint64_t(ts.tv_sec) * 1000000000ull + ts.tv_nsec;
}

// ---------------------------------------------------------------------------------------------
// Detector options

struct DetectorOptions {
    std::string dump;  // directory for debug PGMs (empty = off)
    // default ../models/yolo11n-grayscale_<size>_gray.ncnn (exported at exactly size x 2*size, then prep_model.py)
    std::string model;
    std::string calib = "calib.txt";
    int size = 576;        // per-view rectified size (YOLO input is size x 2*size)
    float hfov_deg = 100;  // virtual camera field of view
    float conf = 0.25f;
    float iou = 0.5f;      // overlapping detections/tracks above this IoU keep only the most confident
    float contain = 0.8f;  // ...same if this fraction of the smaller box lies inside the larger one
    float latency_ms = 8;  // camserver arrival time -> mid-exposure; tune if boxes lag/lead head motion
    float max_hz = 0;      // run YOLO at most this often (0 = every camera frame pair)
};
static DetectorOptions g_det;
static std::atomic<bool> g_det_paused{false};  // skip frames (no GPU work) while set
static std::atomic<int> g_det_size{0};         // per-view size to run at (set from g_det.size); a change reloads
static std::atomic<float> g_det_hz{0};         // max_hz, changeable while running
static std::atomic<float> g_det_yolo_ms{0};    // mean YOLO time over the last stats period

// Parses one detector flag (argv[i], value via next()); false if it isn't one. Call finish_detector_options() after.
template <class Next> static bool parse_detector_option(const std::string& a, Next next) {
    if (a == "--dump") {
        g_det.dump = next();
    } else if (a == "--model") {
        g_det.model = next();
    } else if (a == "--calib") {
        g_det.calib = next();
    } else if (a == "--size") {
        g_det.size = atoi(next());
    } else if (a == "--hfov") {
        g_det.hfov_deg = float(atof(next()));
    } else if (a == "--conf") {
        g_det.conf = float(atof(next()));
    } else if (a == "--iou") {
        g_det.iou = float(atof(next()));
    } else if (a == "--contain") {
        g_det.contain = float(atof(next()));
    } else if (a == "--latency-ms") {
        g_det.latency_ms = float(atof(next()));
    } else if (a == "--max-hz") {
        g_det.max_hz = float(atof(next()));
    } else {
        return false;
    }
    return true;
}

static bool finish_detector_options() {
    if (g_det.size % 32) {
        fprintf(stderr, "--size must be a multiple of 32\n");
        return false;
    }
    g_det_size = g_det.size, g_det_hz = g_det.max_hz;
    return true;
}

// ---------------------------------------------------------------------------------------------
// Calibration (written by calib.py): Kannala-Brandt intrinsics + head_from_cam pose per SLAM camera

struct CamCal {
    float fx, fy, cx, cy, k[4];
    float R[9];  // head_from_cam rotation, row-major; camera frame is OpenCV (x right, y down, z fwd)
    float t[3];  // camera position in head frame (m)
};

static CamCal load_calib(const std::string& path, const char* name) {
    FILE* f = fopen(path.c_str(), "r");
    if (!f) {
        throw std::runtime_error("cannot open " + path + " (run calib.py)");
    }
    char line[2048];
    while (fgets(line, sizeof line, f)) {
        char n[64];
        CamCal c;
        float* v[] = {&c.fx,   &c.fy,   &c.cx,   &c.cy,   &c.k[0], &c.k[1], &c.k[2], &c.k[3], &c.R[0], &c.R[1],
                      &c.R[2], &c.R[3], &c.R[4], &c.R[5], &c.R[6], &c.R[7], &c.R[8], &c.t[0], &c.t[1], &c.t[2]};
        int off = 0;
        if (sscanf(line, "%63s%n", n, &off) != 1 || strcmp(n, name)) {
            continue;
        }
        char* p = line + off;
        for (float* x : v) {
            *x = strtof(p, &p);
        }
        fclose(f);
        return c;
    }
    fclose(f);
    throw std::runtime_error(std::string("camera ") + name + " not in " + path);
}

// ---------------------------------------------------------------------------------------------
// Rectification: virtual pinhole camera at the SLAM camera's position, axes aligned with the head
// (x right, y down, looking along head -z). Both views share orientation -> rectified stereo pair.

struct Rectifier {
    int S;
    float f;
    std::vector<float> map;  // (src x, src y) per output pixel; x < 0 = outside the fisheye

    Rectifier(const CamCal& c, int size, float hfov_deg) : S(size), map(size * size * 2) {
        f = (S / 2.0f) / std::tan(hfov_deg * float(M_PI) / 360.0f);
        for (int v = 0; v < S; v++) {
            for (int u = 0; u < S; u++) {
                // Ray in head frame for virtual pixel center (u, v).
                float hx = (u + 0.5f - S / 2.0f) / f, hy = -(v + 0.5f - S / 2.0f) / f, hz = -1;
                // cam_from_head = R^T
                float x = c.R[0] * hx + c.R[3] * hy + c.R[6] * hz;
                float y = c.R[1] * hx + c.R[4] * hy + c.R[7] * hz;
                float z = c.R[2] * hx + c.R[5] * hy + c.R[8] * hz;
                float* m = &map[(v * S + u) * 2];
                float r = std::sqrt(x * x + y * y), th = std::atan2(r, z);
                if (th > 88 * float(M_PI) / 180) {
                    m[0] = -1;
                    continue;
                }
                float t2 = th * th;
                float thd = th * (1 + t2 * (c.k[0] + t2 * (c.k[1] + t2 * (c.k[2] + t2 * c.k[3]))));
                float s = r > 1e-9f ? thd / r : 0;
                m[0] = c.fx * x * s + c.cx;
                m[1] = c.fy * y * s + c.cy;
                if (m[0] < 0 || m[1] < 0 || m[0] > shm::kW - 2 || m[1] > shm::kH - 2) {
                    m[0] = -1;
                }
            }
        }
    }

    void apply(const uint8_t* src, uint8_t* dst) const {
        for (int i = 0; i < S * S; i++) {
            float sx = map[i * 2], sy = map[i * 2 + 1];
            if (sx < 0) {
                dst[i] = 0;
                continue;
            }
            int x = int(sx), y = int(sy);
            float ax = sx - x, ay = sy - y;
            const uint8_t* p = src + y * shm::kStride + x;
            float top = p[0] + ax * (p[1] - p[0]);
            float bot = p[shm::kStride] + ax * (p[shm::kStride + 1] - p[shm::kStride]);
            dst[i] = uint8_t(top + ay * (bot - top) + 0.5f);
        }
    }
};

// ---------------------------------------------------------------------------------------------
// Detection + stereo

struct Det {
    float x0, y0, x1, y1, score;
    int cls;
};

struct Obj {
    int cls;
    float score, dist;
    XrVector3f corners[4];  // head frame at capture time: TL, TR, BR, BL
    bool stereo;            // false: seen in the left view only, corners are on the 1 m plane
    XrVector3f origin;      // camera position (head frame) the corners were projected from
};

struct DetectionSet {
    uint64_t id = 0;
    uint64_t t_capture_ns = 0;
    std::vector<Obj> objs;
};

static std::mutex g_det_mutex;
static DetectionSet g_dets;

static float iou(const Det& a, const Det& b) {
    float ix = std::max(0.f, std::min(a.x1, b.x1) - std::max(a.x0, b.x0));
    float iy = std::max(0.f, std::min(a.y1, b.y1) - std::max(a.y0, b.y0));
    float inter = ix * iy;
    return inter / ((a.x1 - a.x0) * (a.y1 - a.y0) + (b.x1 - b.x0) * (b.y1 - b.y0) - inter + 1e-6f);
}

// Duplicate if they overlap a lot (IoU) or one is mostly inside the other (intersection over the
// smaller area): a half-size box nested in a bigger one is only ~25% IoU but clearly the same object.
static bool duplicate(const Det& a, const Det& b) {
    float ix = std::max(0.f, std::min(a.x1, b.x1) - std::max(a.x0, b.x0));
    float iy = std::max(0.f, std::min(a.y1, b.y1) - std::max(a.y0, b.y0));
    float smaller = std::min((a.x1 - a.x0) * (a.y1 - a.y0), (b.x1 - b.x0) * (b.y1 - b.y0));
    return iou(a, b) > g_det.iou || ix * iy > g_det.contain * smaller;
}

// Class-agnostic: overlapping boxes of different classes (laptop/tv/keyboard on one object) keep only the best.
static void nms(std::vector<Det>& d) {
    std::sort(d.begin(), d.end(), [](const Det& a, const Det& b) {
        return a.score > b.score;
    });
    std::vector<Det> keep;
    for (const Det& x : d) {
        bool ok = true;
        for (const Det& k : keep) {
            if (duplicate(k, x)) {
                ok = false;
                break;
            }
        }
        if (ok) {
            keep.push_back(x);
        }
    }
    d.swap(keep);
}

// Sub-pixel disparity refinement by NCC of the left box interior along the (horizontal) epipolar line.
static float refine_disparity(const uint8_t* L, const uint8_t* R, int S, const Det& l, float d0) {
    float w = l.x1 - l.x0, h = l.y1 - l.y0;
    int x0 = int(l.x0 + 0.2f * w), x1 = int(l.x1 - 0.2f * w), y0 = int(l.y0 + 0.2f * h), y1 = int(l.y1 - 0.2f * h);
    int step = std::max(1, int(std::max(x1 - x0, y1 - y0) / 24));
    if (x1 - x0 < 6 || y1 - y0 < 6) {
        return d0;
    }
    const int R_ = 6;
    float score[2 * R_ + 1];
    for (int k = -R_; k <= R_; k++) {
        int d = int(std::lround(d0)) + k;
        double sl = 0, sr = 0, sll = 0, srr = 0, slr = 0;
        int n = 0;
        for (int y = y0; y < y1; y += step) {
            for (int x = x0; x < x1; x += step) {
                int xr = x - d;
                if (x < 0 || y < 0 || x >= S || y >= S || xr < 0 || xr >= S) {
                    continue;
                }
                double a = L[y * S + x], b = R[y * S + xr];
                sl += a;
                sr += b;
                sll += a * a;
                srr += b * b;
                slr += a * b;
                n++;
            }
        }
        double cov = slr - sl * sr / std::max(n, 1);
        double var = (sll - sl * sl / std::max(n, 1)) * (srr - sr * sr / std::max(n, 1));
        score[k + R_] = n > 20 && var > 1e-3 ? float(cov / std::sqrt(var)) : -1;
    }
    int best = int(std::max_element(score, score + 2 * R_ + 1) - score);
    if (score[best] < 0.6f || best == 0 || best == 2 * R_) {
        return d0;
    }
    float a = score[best - 1], b = score[best], c = score[best + 1];
    float sub = (a - c) / (2 * (a - 2 * b + c) + 1e-9f);
    return std::lround(d0) + (best - R_) + std::clamp(sub, -0.5f, 0.5f);
}

static void write_pgm(const std::string& path, const uint8_t* p, int w, int h) {
    FILE* f = fopen(path.c_str(), "wb");
    if (!f) {
        return;
    }
    fprintf(f, "P5\n%d %d\n255\n", w, h);
    fwrite(p, 1, size_t(w) * h, f);
    fclose(f);
}

// One run at size S, until g_det_size changes or quit.
static void detect_run(const int S) {
    const CamCal cal[2] = {load_calib(g_det.calib, shm::kCamNames[0]), load_calib(g_det.calib, shm::kCamNames[1])};
    Rectifier rect[2] = {Rectifier(cal[0], S, g_det.hfov_deg), Rectifier(cal[1], S, g_det.hfov_deg)};
    const float f = rect[0].f;
    const float baseline = cal[1].t[0] - cal[0].t[0];  // right cam is at +x in the head frame
    printf("detector: %dx%d views, f=%.1f px, baseline %.1f mm\n", S, S, f, baseline * 1000);

    ncnn::Net net;
    net.opt.use_vulkan_compute = ncnn::get_gpu_count() > 0;
    net.opt.num_threads = 2;
    // ~1 ms faster on the Adreno 750 (Turnip) at every size; ncnn's other Vulkan defaults measured best.
    net.opt.use_sgemm_convolution = false;
    net.opt.use_winograd63_convolution = false;
    const std::string model =
        g_det.model.empty() ? "../models/yolo11n-grayscale_" + std::to_string(S) + "_gray.ncnn" : g_det.model;
    if (net.load_param((model + ".param").c_str()) || net.load_model((model + ".bin").c_str())) {
        throw std::runtime_error("failed to load model " + model + " (see prep_model.py)");
    }
    printf("detector: ncnn %s\n", net.opt.use_vulkan_compute ? "Vulkan GPU" : "CPU");

    // camserver creates the file before sizing it; touching a mapping past its end is SIGBUS.
    int shm_fd = -1;
    struct stat st = {};
    while (!g_quit && ((shm_fd = open(shm::kPath, O_RDONLY)) < 0 || fstat(shm_fd, &st) ||
                       st.st_size < off_t(sizeof(shm::Header)))) {
        if (shm_fd >= 0) {
            close(shm_fd);
        }
        fprintf(stderr, "waiting for camserver (%s)\n", shm::kPath);
        sleep(1);
    }
    if (g_quit) {
        return;
    }
    auto* hdr = (const shm::Header*)mmap(nullptr, sizeof(shm::Header), PROT_READ, MAP_SHARED, shm_fd, 0);
    if (hdr == MAP_FAILED) {
        throw std::runtime_error(std::string("mmap ") + shm::kPath + ": " + strerror(errno));
    }

    // A torn read leaves garbage in its buffer, so frames are read into `scratch` and swapped in only when intact.
    std::vector<uint8_t> raw[2] = {std::vector<uint8_t>(shm::kStride * shm::kH),
                                   std::vector<uint8_t>(shm::kStride * shm::kH)};
    std::vector<uint8_t> scratch(shm::kStride * shm::kH);
    std::vector<uint8_t> input(size_t(S) * S * 2);
    uint64_t last_seq[2] = {0, 0}, t_ns[2] = {0, 0}, last_pair = 0;
    uint32_t fseq[2];
    uint64_t frames = 0, stat_t = now_ns(), dump_n = 0, last_run = 0;
    double t_rect = 0, t_net = 0, t_post = 0;
    bool printed_shape = false;

    while (!g_quit && g_det_size == S) {
        if (g_det_paused) {
            usleep(100000);
            continue;
        }
        if (float hz = g_det_hz; hz > 0) {
            uint64_t t = now_ns(), due = last_run + uint64_t(1e9 / hz);
            if (t < due) {
                usleep(std::min<uint64_t>((due - t) / 1000, 100000));
                continue;
            }
        }
        for (int i = 0; i < 2; i++) {
            if (shm::read(hdr->slots[i], last_seq[i], scratch.data(), t_ns[i], fseq[i])) {
                raw[i].swap(scratch);
            }
        }
        // Both cameras are hardware-synchronized: wait for a pair with the same capture time.
        if (t_ns[0] == 0 || t_ns[0] == last_pair || std::llabs(int64_t(t_ns[0]) - int64_t(t_ns[1])) > 2000000) {
            usleep(1000);
            continue;
        }
        last_pair = t_ns[0];

        uint64_t a = last_run = now_ns();
        rect[0].apply(raw[0].data(), input.data());
        rect[1].apply(raw[1].data(), input.data() + size_t(S) * S);
        // The SLAM frames are dim and low-contrast; stretch the 1st..99th percentile to full range.
        // Pixels outside the fisheye are 0 -> YOLO's letterbox gray (114), so they don't look like edges.
        uint32_t hist[256] = {};
        for (size_t i = 0; i < input.size(); i += 7) {
            hist[input[i]]++;
        }
        uint32_t total = 0, acc = 0;
        for (int v = 1; v < 256; v++) {
            total += hist[v];
        }
        int lo = 1, hi = 255;
        for (int v = 1; v < 256; v++) {
            acc += hist[v];
            if (acc < total / 100) {
                lo = v;
            }
            if (acc <= total * 99 / 100) {
                hi = v;
            }
        }
        uint8_t lut[256];
        lut[0] = 114;
        for (int v = 1; v < 256; v++) {
            lut[v] = uint8_t(std::clamp((v - lo) * 255 / std::max(hi - lo, 1), 0, 255));
        }
        for (uint8_t& p : input) {
            p = lut[p];
        }
        uint64_t b = now_ns();

        // prep_model.py folded GRAY2RGB and the 1/255 scale into the first conv.
        ncnn::Mat in = ncnn::Mat::from_pixels(input.data(), ncnn::Mat::PIXEL_GRAY, S, 2 * S);
        ncnn::Extractor ex = net.create_extractor();
        ex.input("in0", in);
        ncnn::Mat out;
        ex.extract("out0", out);
        uint64_t c = now_ns();
        if (!printed_shape) {
            printf("detector: out0 w=%d h=%d c=%d\n", out.w, out.h, out.c);
            printed_shape = true;
        }
        if (out.h < 4 + 80) {  // extract failed, or not a COCO head: out.row(4 + k) below would read past the end
            continue;
        }

        // Decode [4 + 80, N]: cx, cy, w, h in input pixels, then per-class sigmoid scores.
        std::vector<Det> views[2];
        for (int i = 0; i < out.w; i++) {
            int best = 0;
            float bs = 0;
            for (int k = 0; k < 80; k++) {
                float s = out.row(4 + k)[i];
                if (s > bs) {
                    bs = s;
                    best = k;
                }
            }
            if (bs < g_det.conf) {
                continue;
            }
            float cx = out.row(0)[i], cy = out.row(1)[i], w = out.row(2)[i], h = out.row(3)[i];
            int v = cy >= S;  // top half = left view, bottom half = right view
            float oy = v * S;
            Det d{cx - w / 2, std::max(cy - h / 2, oy) - oy, cx + w / 2, std::min(cy + h / 2, oy + S) - oy, bs, best};
            if (d.y1 - d.y0 > 4) {
                views[v].push_back(d);
            }
        }
        nms(views[0]);
        nms(views[1]);

        // Stereo match: same rows, similar size, positive disparity. Class-agnostic like nms() (the views
        // can disagree on laptop/tv for one object), with a mild preference for the same class.
        std::vector<Obj> objs;
        std::vector<bool> used(views[1].size());
        const uint8_t* L = input.data();
        const uint8_t* R = input.data() + size_t(S) * S;
        const XrVector3f origin{cal[0].t[0], cal[0].t[1], cal[0].t[2]};
        auto make_obj = [&](int cls, float score, float Z, float bx0, float by0, float bx1, float by1, bool stereo) {
            Obj o{cls, score, Z, {}, stereo, origin};
            float us[4] = {bx0, bx1, bx1, bx0}, vs[4] = {by0, by0, by1, by1};
            for (int k = 0; k < 4; k++) {
                float X = (us[k] - S / 2.0f) / f * Z, Y = (vs[k] - S / 2.0f) / f * Z;
                o.corners[k] = {origin.x + X, origin.y - Y, origin.z - Z};
            }
            return o;
        };
        for (const Det& l : views[0]) {
            int bi = -1;
            float bc = 1e9f;
            float lh = l.y1 - l.y0, lw = l.x1 - l.x0;
            for (size_t j = 0; j < views[1].size(); j++) {
                const Det& r = views[1][j];
                if (used[j]) {
                    continue;
                }
                float rh = r.y1 - r.y0, rw = r.x1 - r.x0;
                float dy = std::fabs((l.y0 + l.y1) - (r.y0 + r.y1)) / 2 / lh;
                float dsize = std::fabs(lh - rh) / std::max(lh, rh) + std::fabs(lw - rw) / std::max(lw, rw);
                float disp = ((l.x0 + l.x1) - (r.x0 + r.x1)) / 2;
                if (dy > 0.15f || dsize > 0.5f || disp < 0.3f || disp > f * baseline / 0.2f) {
                    continue;
                }
                float cost = dy + dsize + (r.cls != l.cls) * 0.2f;
                if (cost < bc) {
                    bc = cost;
                    bi = int(j);
                }
            }
            if (bi < 0) {
                // Direction-only measurement: the XR side can still update an existing track with it.
                objs.push_back(make_obj(l.cls, l.score, 1, l.x0, l.y0, l.x1, l.y1, false));
                continue;
            }
            used[bi] = true;
            const Det& r = views[1][bi];
            float d0 = ((l.x0 - r.x0) + (l.x1 - r.x1)) / 2;
            float d = refine_disparity(L, R, S, l, d0);
            float Z = f * baseline / std::max(d, 0.3f);
            // Average the two views' boxes (right box shifted by the disparity) in the left view.
            float bx0 = (l.x0 + r.x0 + d) / 2, bx1 = (l.x1 + r.x1 + d) / 2;
            float by0 = (l.y0 + r.y0) / 2, by1 = (l.y1 + r.y1) / 2;
            const Det& top = l.score >= r.score ? l : r;
            objs.push_back(make_obj(top.cls, top.score, Z, bx0, by0, bx1, by1, true));
        }
        uint64_t e = now_ns();

        {
            std::lock_guard<std::mutex> lk(g_det_mutex);
            g_dets.id++;
            g_dets.t_capture_ns = t_ns[0] - uint64_t(g_det.latency_ms * 1e6);
            g_dets.objs = objs;
        }

        if (!g_det.dump.empty() && frames % 30 == 0) {
            std::vector<uint8_t> img = input;
            for (int v = 0; v < 2; v++) {
                for (const Det& d : views[v]) {
                    int x0 = std::clamp(int(d.x0), 0, S - 1), x1 = std::clamp(int(d.x1), 0, S - 1);
                    int y0 = std::clamp(int(d.y0), 0, S - 1) + v * S, y1 = std::clamp(int(d.y1), 0, S - 1) + v * S;
                    for (int x = x0; x <= x1; x++) {
                        img[y0 * S + x] = img[y1 * S + x] = 255;
                    }
                    for (int y = y0; y <= y1; y++) {
                        img[y * S + x0] = img[y * S + x1] = 255;
                    }
                }
            }
            char path[512];
            snprintf(path, sizeof path, "%s/det_%04llu.pgm", g_det.dump.c_str(), (unsigned long long)dump_n++);
            write_pgm(path, img.data(), S, 2 * S);
            for (const Obj& o : objs) {
                if (o.stereo) {
                    printf("  %s %.2f at %.2fm\n", kCoco[o.cls], o.score, o.dist);
                }
            }
        }

        frames++;
        t_rect += (b - a) / 1e6;
        t_net += (c - b) / 1e6;
        t_post += (e - c) / 1e6;
        uint64_t t = now_ns();
        if (t - stat_t > 5000000000ull) {
            printf("detector: %.1f fps | rectify %.1f ms, yolo %.1f ms, match %.1f ms | %zu+%zu dets, %zu objects | "
                   "age %.0f ms\n",
                   frames / ((t - stat_t) / 1e9), t_rect / frames, t_net / frames, t_post / frames, views[0].size(),
                   views[1].size(),
                   size_t(std::count_if(objs.begin(), objs.end(),
                                        [](const Obj& o) {
                                            return o.stereo;
                                        })),
                   (t - t_ns[0]) / 1e6);
            fflush(stdout);
            g_det_yolo_ms = float(t_net / frames);
            frames = 0;
            t_rect = t_net = t_post = 0;
            stat_t = t;
        }
    }
    munmap((void*)hdr, sizeof(shm::Header));
    close(shm_fd);
}

static void detect_loop() {
    while (!g_quit) {
        detect_run(g_det_size);
    }
}
