// yoloxr: YOLO bounding boxes over Steam Frame passthrough.
//
// Detection thread: reads the two SLAM camera frames published by camserver, rectifies each into a
// virtual pinhole camera aligned with the head frame (so the pair is a rectified stereo pair),
// runs YOLO11n (ncnn, Vulkan) on both views stacked into one image, matches detections across
// views and triangulates depth from disparity.
//
// XR thread: OpenXR scene app with ALPHA_BLEND, so the runtime shows passthrough wherever we draw
// nothing. Boxes are placed in world space using the head pose at frame capture time, and drawn as
// thick outlines with a label.
//
// Usage: yoloxr [--no-xr] [--dump DIR] [--size 480] [--hfov 100] [--conf 0.35] [--latency-ms 8]
#define XR_USE_GRAPHICS_API_VULKAN
#define XR_USE_TIMESPEC
#include <fcntl.h>
#include <signal.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>
#include <vulkan/vulkan.h>
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <net.h>

#include "shm.h"
#include "third_party/xr_linear.h"
#include "third_party/stb_easy_font.h"

static const uint32_t kVertSpv[] = {
#include "box.vert.inc"
};
static const uint32_t kFragSpv[] = {
#include "box.frag.inc"
};

static const char* kCoco[80] = {
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light",
    "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
    "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard",
    "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase",
    "scissors", "teddy bear", "hair drier", "toothbrush"};

static std::atomic<bool> g_quit{false};

#define XR_CHECK(x)                                                                              \
    do {                                                                                         \
        XrResult r_ = (x);                                                                       \
        if (XR_FAILED(r_)) throw std::runtime_error(std::string(#x) + " failed: " + std::to_string(r_)); \
    } while (0)
#define VK_CHECK(x)                                                                              \
    do {                                                                                         \
        VkResult r_ = (x);                                                                       \
        if (r_ != VK_SUCCESS) throw std::runtime_error(std::string(#x) + " failed: " + std::to_string(r_)); \
    } while (0)

static uint64_t now_ns() {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return uint64_t(ts.tv_sec) * 1000000000ull + ts.tv_nsec;
}

// ---------------------------------------------------------------------------------------------
// Options

struct Options {
    bool xr = true;
    std::string dump;            // directory for debug PGMs (empty = off)
    std::string model;           // default ../models/yolo11n_<size>.ncnn (exported at exactly size x 2*size)
    std::string calib = "calib.txt";
    int size = 576;              // per-view rectified size (YOLO input is size x 2*size)
    float hfov_deg = 100;        // virtual camera field of view
    float conf = 0.25f;
    float iou = 0.5f;            // overlapping detections/tracks above this IoU keep only the most confident
    float contain = 0.8f;        // ...same if this fraction of the smaller box lies inside the larger one
    float latency_ms = 8;        // camserver arrival time -> mid-exposure; tune if boxes lag/lead head motion
    float smooth_ms = 80;        // displayed boxes glide toward the latest measurement with this time constant
    int confirm_hits = 3;        // detections needed before a track is shown (debounce)
    float hold_ms = 400;         // keep showing a track this long after its last detection, then fade out
    float focus_margin = 2;      // show a box (and label) only while the gaze hits the box grown by this many label heights
    bool eye_gaze = true;        // use eye tracking for the gaze direction (falls back to head forward)
};
static Options g_opt;

// ---------------------------------------------------------------------------------------------
// Calibration (written by calib.py): Kannala-Brandt intrinsics + head_from_cam pose per SLAM camera

struct CamCal {
    float fx, fy, cx, cy, k[4];
    float R[9];  // head_from_cam rotation, row-major; camera frame is OpenCV (x right, y down, z fwd)
    float t[3];  // camera position in head frame (m)
};

static CamCal load_calib(const std::string& path, const char* name) {
    FILE* f = fopen(path.c_str(), "r");
    if (!f) throw std::runtime_error("cannot open " + path + " (run calib.py)");
    char line[2048];
    while (fgets(line, sizeof line, f)) {
        char n[64];
        CamCal c;
        float* v[] = {&c.fx, &c.fy, &c.cx, &c.cy, &c.k[0], &c.k[1], &c.k[2], &c.k[3], &c.R[0], &c.R[1], &c.R[2],
                      &c.R[3], &c.R[4], &c.R[5], &c.R[6], &c.R[7], &c.R[8], &c.t[0], &c.t[1], &c.t[2]};
        int off = 0;
        if (sscanf(line, "%63s%n", n, &off) != 1 || strcmp(n, name)) continue;
        char* p = line + off;
        for (float* x : v) *x = strtof(p, &p);
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
        for (int v = 0; v < S; v++)
            for (int u = 0; u < S; u++) {
                // Ray in head frame for virtual pixel center (u, v).
                float hx = (u + 0.5f - S / 2.0f) / f, hy = -(v + 0.5f - S / 2.0f) / f, hz = -1;
                // cam_from_head = R^T
                float x = c.R[0] * hx + c.R[3] * hy + c.R[6] * hz;
                float y = c.R[1] * hx + c.R[4] * hy + c.R[7] * hz;
                float z = c.R[2] * hx + c.R[5] * hy + c.R[8] * hz;
                float* m = &map[(v * S + u) * 2];
                float r = std::sqrt(x * x + y * y), th = std::atan2(r, z);
                if (th > 88 * float(M_PI) / 180) { m[0] = -1; continue; }
                float t2 = th * th;
                float thd = th * (1 + t2 * (c.k[0] + t2 * (c.k[1] + t2 * (c.k[2] + t2 * c.k[3]))));
                float s = r > 1e-9f ? thd / r : 0;
                m[0] = c.fx * x * s + c.cx;
                m[1] = c.fy * y * s + c.cy;
                if (m[0] < 0 || m[1] < 0 || m[0] > shm::kW - 2 || m[1] > shm::kH - 2) m[0] = -1;
            }
    }

    void apply(const uint8_t* src, uint8_t* dst) const {
        for (int i = 0; i < S * S; i++) {
            float sx = map[i * 2], sy = map[i * 2 + 1];
            if (sx < 0) { dst[i] = 0; continue; }
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
    return iou(a, b) > g_opt.iou || ix * iy > g_opt.contain * smaller;
}

// Class-agnostic: overlapping boxes of different classes (laptop/tv/keyboard on one object) keep only the best.
static void nms(std::vector<Det>& d) {
    std::sort(d.begin(), d.end(), [](const Det& a, const Det& b) { return a.score > b.score; });
    std::vector<Det> keep;
    for (const Det& x : d) {
        bool ok = true;
        for (const Det& k : keep)
            if (duplicate(k, x)) { ok = false; break; }
        if (ok) keep.push_back(x);
    }
    d.swap(keep);
}

// Sub-pixel disparity refinement by NCC of the left box interior along the (horizontal) epipolar line.
static float refine_disparity(const uint8_t* L, const uint8_t* R, int S, const Det& l, float d0) {
    float w = l.x1 - l.x0, h = l.y1 - l.y0;
    int x0 = int(l.x0 + 0.2f * w), x1 = int(l.x1 - 0.2f * w), y0 = int(l.y0 + 0.2f * h), y1 = int(l.y1 - 0.2f * h);
    int step = std::max(1, int(std::max(x1 - x0, y1 - y0) / 24));
    if (x1 - x0 < 6 || y1 - y0 < 6) return d0;
    const int R_ = 6;
    float score[2 * R_ + 1];
    for (int k = -R_; k <= R_; k++) {
        int d = int(std::lround(d0)) + k;
        double sl = 0, sr = 0, sll = 0, srr = 0, slr = 0;
        int n = 0;
        for (int y = y0; y < y1; y += step)
            for (int x = x0; x < x1; x += step) {
                int xr = x - d;
                if (x < 0 || y < 0 || x >= S || y >= S || xr < 0 || xr >= S) continue;
                double a = L[y * S + x], b = R[y * S + xr];
                sl += a; sr += b; sll += a * a; srr += b * b; slr += a * b; n++;
            }
        double cov = slr - sl * sr / std::max(n, 1);
        double var = (sll - sl * sl / std::max(n, 1)) * (srr - sr * sr / std::max(n, 1));
        score[k + R_] = n > 20 && var > 1e-3 ? float(cov / std::sqrt(var)) : -1;
    }
    int best = int(std::max_element(score, score + 2 * R_ + 1) - score);
    if (score[best] < 0.6f || best == 0 || best == 2 * R_) return d0;
    float a = score[best - 1], b = score[best], c = score[best + 1];
    float sub = (a - c) / (2 * (a - 2 * b + c) + 1e-9f);
    return std::lround(d0) + (best - R_) + std::clamp(sub, -0.5f, 0.5f);
}

static void write_pgm(const std::string& path, const uint8_t* p, int w, int h) {
    FILE* f = fopen(path.c_str(), "wb");
    if (!f) return;
    fprintf(f, "P5\n%d %d\n255\n", w, h);
    fwrite(p, 1, size_t(w) * h, f);
    fclose(f);
}

static void detect_loop() {
    const CamCal cal[2] = {load_calib(g_opt.calib, shm::kCamNames[0]), load_calib(g_opt.calib, shm::kCamNames[1])};
    const int S = g_opt.size;
    Rectifier rect[2] = {Rectifier(cal[0], S, g_opt.hfov_deg), Rectifier(cal[1], S, g_opt.hfov_deg)};
    const float f = rect[0].f;
    const float baseline = cal[1].t[0] - cal[0].t[0];  // right cam is at +x in the head frame
    printf("detector: %dx%d views, f=%.1f px, baseline %.1f mm\n", S, S, f, baseline * 1000);

    ncnn::Net net;
    net.opt.use_vulkan_compute = ncnn::get_gpu_count() > 0;
    net.opt.num_threads = 2;
    if (net.load_param((g_opt.model + ".param").c_str()) || net.load_model((g_opt.model + ".bin").c_str()))
        throw std::runtime_error("failed to load model " + g_opt.model);
    printf("detector: ncnn %s\n", net.opt.use_vulkan_compute ? "Vulkan GPU" : "CPU");

    int shm_fd = -1;
    while (!g_quit && (shm_fd = open(shm::kPath, O_RDONLY)) < 0) {
        fprintf(stderr, "waiting for camserver (%s)\n", shm::kPath);
        sleep(1);
    }
    if (g_quit) return;
    auto* hdr = (const shm::Header*)mmap(nullptr, sizeof(shm::Header), PROT_READ, MAP_SHARED, shm_fd, 0);

    std::vector<uint8_t> raw[2] = {std::vector<uint8_t>(shm::kStride * shm::kH), std::vector<uint8_t>(shm::kStride * shm::kH)};
    std::vector<uint8_t> input(size_t(S) * S * 2);
    uint64_t last_seq[2] = {0, 0}, t_ns[2] = {0, 0}, last_pair = 0;
    uint32_t fseq[2];
    uint64_t frames = 0, stat_t = now_ns(), dump_n = 0;
    double t_rect = 0, t_net = 0, t_post = 0;
    bool printed_shape = false;

    while (!g_quit) {
        for (int i = 0; i < 2; i++) shm::read(hdr->slots[i], last_seq[i], raw[i].data(), t_ns[i], fseq[i]);
        // Both cameras are hardware-synchronized: wait for a pair with the same capture time.
        if (t_ns[0] == 0 || t_ns[0] == last_pair || std::llabs(int64_t(t_ns[0]) - int64_t(t_ns[1])) > 2000000) {
            usleep(1000);
            continue;
        }
        last_pair = t_ns[0];

        uint64_t a = now_ns();
        rect[0].apply(raw[0].data(), input.data());
        rect[1].apply(raw[1].data(), input.data() + size_t(S) * S);
        // The SLAM frames are dim and low-contrast; stretch the 1st..99th percentile to full range.
        // Pixels outside the fisheye are 0 -> YOLO's letterbox gray (114), so they don't look like edges.
        uint32_t hist[256] = {};
        for (size_t i = 0; i < input.size(); i += 7) hist[input[i]]++;
        uint32_t total = 0, acc = 0;
        for (int v = 1; v < 256; v++) total += hist[v];
        int lo = 1, hi = 255;
        for (int v = 1; v < 256; v++) { acc += hist[v]; if (acc < total / 100) lo = v; if (acc <= total * 99 / 100) hi = v; }
        uint8_t lut[256];
        lut[0] = 114;
        for (int v = 1; v < 256; v++) lut[v] = uint8_t(std::clamp((v - lo) * 255 / std::max(hi - lo, 1), 0, 255));
        for (uint8_t& p : input) p = lut[p];
        uint64_t b = now_ns();

        ncnn::Mat in = ncnn::Mat::from_pixels(input.data(), ncnn::Mat::PIXEL_GRAY2RGB, S, 2 * S);
        const float norm[3] = {1 / 255.f, 1 / 255.f, 1 / 255.f};
        in.substract_mean_normalize(nullptr, norm);
        ncnn::Extractor ex = net.create_extractor();
        ex.input("in0", in);
        ncnn::Mat out;
        ex.extract("out0", out);
        uint64_t c = now_ns();
        if (!printed_shape) {
            printf("detector: out0 w=%d h=%d c=%d\n", out.w, out.h, out.c);
            printed_shape = true;
        }

        // Decode [4 + 80, N]: cx, cy, w, h in input pixels, then per-class sigmoid scores.
        std::vector<Det> views[2];
        for (int i = 0; i < out.w; i++) {
            int best = 0;
            float bs = 0;
            for (int k = 0; k < 80; k++) {
                float s = out.row(4 + k)[i];
                if (s > bs) { bs = s; best = k; }
            }
            if (bs < g_opt.conf) continue;
            float cx = out.row(0)[i], cy = out.row(1)[i], w = out.row(2)[i], h = out.row(3)[i];
            int v = cy >= S;  // top half = left view, bottom half = right view
            float oy = v * S;
            Det d{cx - w / 2, std::max(cy - h / 2, oy) - oy, cx + w / 2, std::min(cy + h / 2, oy + S) - oy, bs, best};
            if (d.y1 - d.y0 > 4) views[v].push_back(d);
        }
        nms(views[0]);
        nms(views[1]);

        // Stereo match: same class, same rows, similar size, positive disparity.
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
                if (used[j] || r.cls != l.cls) continue;
                float rh = r.y1 - r.y0, rw = r.x1 - r.x0;
                float dy = std::fabs((l.y0 + l.y1) - (r.y0 + r.y1)) / 2 / lh;
                float dsize = std::fabs(lh - rh) / std::max(lh, rh) + std::fabs(lw - rw) / std::max(lw, rw);
                float disp = ((l.x0 + l.x1) - (r.x0 + r.x1)) / 2;
                if (dy > 0.15f || dsize > 0.5f || disp < 0.3f || disp > f * baseline / 0.2f) continue;
                float cost = dy + dsize;
                if (cost < bc) { bc = cost; bi = int(j); }
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
            objs.push_back(make_obj(l.cls, std::max(l.score, r.score), Z, bx0, by0, bx1, by1, true));
        }
        uint64_t e = now_ns();

        {
            std::lock_guard<std::mutex> lk(g_det_mutex);
            g_dets.id++;
            g_dets.t_capture_ns = t_ns[0] - uint64_t(g_opt.latency_ms * 1e6);
            g_dets.objs = objs;
        }

        if (!g_opt.dump.empty() && frames % 30 == 0) {
            std::vector<uint8_t> img = input;
            for (int v = 0; v < 2; v++)
                for (const Det& d : views[v]) {
                    int x0 = std::clamp(int(d.x0), 0, S - 1), x1 = std::clamp(int(d.x1), 0, S - 1);
                    int y0 = std::clamp(int(d.y0), 0, S - 1) + v * S, y1 = std::clamp(int(d.y1), 0, S - 1) + v * S;
                    for (int x = x0; x <= x1; x++) img[y0 * S + x] = img[y1 * S + x] = 255;
                    for (int y = y0; y <= y1; y++) img[y * S + x0] = img[y * S + x1] = 255;
                }
            char path[512];
            snprintf(path, sizeof path, "%s/det_%04llu.pgm", g_opt.dump.c_str(), (unsigned long long)dump_n++);
            write_pgm(path, img.data(), S, 2 * S);
            for (const Obj& o : objs)
                if (o.stereo) printf("  %s %.2f at %.2fm\n", kCoco[o.cls], o.score, o.dist);
        }

        frames++;
        t_rect += (b - a) / 1e6;
        t_net += (c - b) / 1e6;
        t_post += (e - c) / 1e6;
        uint64_t t = now_ns();
        if (t - stat_t > 5000000000ull) {
            printf("detector: %.1f fps | rectify %.1f ms, yolo %.1f ms, match %.1f ms | %zu+%zu dets, %zu objects | age %.0f ms\n",
                   frames / ((t - stat_t) / 1e9), t_rect / frames, t_net / frames, t_post / frames, views[0].size(),
                   views[1].size(), size_t(std::count_if(objs.begin(), objs.end(), [](const Obj& o) { return o.stereo; })),
                   (t - t_ns[0]) / 1e6);
            fflush(stdout);
            frames = 0;
            t_rect = t_net = t_post = 0;
            stat_t = t;
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Geometry

struct Vertex {
    float x, y, z, r, g, b, a;
};

static XrVector3f v3(float x, float y, float z) { return {x, y, z}; }
static XrVector3f add(XrVector3f a, XrVector3f b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
static XrVector3f sub(XrVector3f a, XrVector3f b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
static XrVector3f mul(XrVector3f a, float s) { return {a.x * s, a.y * s, a.z * s}; }
static XrVector3f cross(XrVector3f a, XrVector3f b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
static float len(XrVector3f a) { return std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z); }
static XrVector3f norm(XrVector3f a) { return mul(a, 1.0f / std::max(len(a), 1e-9f)); }

static void color_for(int cls, float out[3]) {
    float h = std::fmod(cls * 0.61803f, 1.0f) * 6, s = 0.75f, v = 1.0f;
    int i = int(h);
    float fr = h - i, p = v * (1 - s), q = v * (1 - s * fr), t = v * (1 - s * (1 - fr));
    float rgb[6][3] = {{v, t, p}, {q, v, p}, {p, v, t}, {p, q, v}, {t, p, v}, {v, p, q}};
    memcpy(out, rgb[i % 6], sizeof(float) * 3);
}

static void push_quad(std::vector<Vertex>& vs, XrVector3f a, XrVector3f b, XrVector3f c, XrVector3f d, const float rgba[4]) {
    // premultiplied alpha
    float r = rgba[0] * rgba[3], g = rgba[1] * rgba[3], bl = rgba[2] * rgba[3], al = rgba[3];
    for (XrVector3f p : {a, b, c, a, c, d}) vs.push_back({p.x, p.y, p.z, r, g, bl, al});
}

static void push_line(std::vector<Vertex>& vs, XrVector3f a, XrVector3f b, XrVector3f eye, float width, const float rgba[4]) {
    XrVector3f dir = norm(sub(b, a));
    a = sub(a, mul(dir, width / 2));
    b = add(b, mul(dir, width / 2));
    XrVector3f side = mul(norm(cross(dir, sub(eye, mul(add(a, b), 0.5f)))), width / 2);
    push_quad(vs, sub(a, side), add(a, side), add(b, side), sub(b, side), rgba);
}

// Label above the top-left corner, in the box's plane, in text units (y down) mapped to world by at().
struct LabelLayout {
    char text[96];
    float tw, th = 9, scale;
    XrVector3f origin, right, up;
    XrVector3f at(float x, float y) const { return add(origin, add(mul(right, x * scale), mul(up, -y * scale))); }
    void rect(XrVector3f out[4]) const {  // background, TL TR BR BL
        out[0] = at(-2, -2); out[1] = at(tw + 2, -2); out[2] = at(tw + 2, th + 1); out[3] = at(-2, th + 1);
    }
    float height() const { return (th + 3) * scale; }  // background height in meters
};

static LabelLayout layout_label(const XrVector3f* corners, int cls, float score, XrVector3f eye) {
    LabelLayout l;
    float dist = len(sub(add(corners[0], corners[2]), mul(eye, 2))) / 2;
    snprintf(l.text, sizeof l.text, "%s %d%% %.1fm", kCoco[cls], int(score * 100), dist);
    l.tw = float(stb_easy_font_width(l.text));
    l.right = norm(sub(corners[1], corners[0]));
    l.up = norm(sub(corners[0], corners[3]));
    l.scale = 0.022f * dist / l.th;  // text ~1.3 degrees tall
    l.origin = add(corners[0], mul(l.up, (l.th + 3) * l.scale));
    return l;
}

static void push_box(std::vector<Vertex>& vs, const XrVector3f* corners, int cls, float score, float alpha, XrVector3f eye) {
    float rgba[4] = {0, 0, 0, alpha};
    color_for(cls, rgba);
    float dist = len(sub(add(corners[0], corners[2]), mul(eye, 2))) / 2;
    float width = 0.004f * dist;  // ~0.23 degrees regardless of distance
    for (int k = 0; k < 4; k++) push_line(vs, corners[k], corners[(k + 1) % 4], eye, width, rgba);

    LabelLayout l = layout_label(corners, cls, score, eye);
    static char buf[64 * 1024];
    int quads = stb_easy_font_print(0, 0, l.text, nullptr, buf, sizeof buf);
    auto at = [&](float x, float y) { return l.at(x, y); };
    XrVector3f bg_rect[4];
    l.rect(bg_rect);
    float bg[4] = {0, 0, 0, 0.55f * alpha};
    push_quad(vs, bg_rect[0], bg_rect[1], bg_rect[2], bg_rect[3], bg);
    float fg[4] = {rgba[0], rgba[1], rgba[2], alpha};
    const float* q = (const float*)buf;  // 4 verts per quad: x, y, z, color(4 bytes)
    for (int i = 0; i < quads; i++, q += 16)
        push_quad(vs, at(q[0], q[1]), at(q[4], q[5]), at(q[8], q[9]), at(q[12], q[13]), fg);
}

// ---------------------------------------------------------------------------------------------
// OpenXR + Vulkan

struct Track;

struct Eye {
    XrSwapchain swapchain;
    int32_t w, h;
    std::vector<XrSwapchainImageVulkan2KHR> images;
    std::vector<VkImageView> views;
    std::vector<VkFramebuffer> fbs;
};

class App {
  public:
    void run();

  private:
    void init_xr();
    void init_vulkan();
    void init_session();
    void init_pipeline();
    void handle_events();
    void render_frame();
    void render_eye(Eye& e, const XrView& view, uint32_t nverts);
    void update_world_boxes();
    template <class T> T proc(const char* name) {
        PFN_xrVoidFunction f = nullptr;
        XR_CHECK(xrGetInstanceProcAddr(instance_, name, &f));
        return reinterpret_cast<T>(f);
    }

    XrInstance instance_ = XR_NULL_HANDLE;
    XrSystemId system_ = XR_NULL_SYSTEM_ID;
    XrSession session_ = XR_NULL_HANDLE;
    XrSpace app_space_ = XR_NULL_HANDLE, view_space_ = XR_NULL_HANDLE;
    XrSessionState state_ = XR_SESSION_STATE_UNKNOWN;
    bool running_ = false, exit_ = false, exit_requested_ = false;
    std::vector<Eye> eyes_;
    PFN_xrConvertTimespecTimeToTimeKHR to_xr_time_ = nullptr;

    VkInstance vk_instance_ = VK_NULL_HANDLE;
    VkPhysicalDevice phys_ = VK_NULL_HANDLE;
    VkDevice dev_ = VK_NULL_HANDLE;
    uint32_t qfam_ = 0;
    VkQueue queue_ = VK_NULL_HANDLE;
    VkFormat format_ = VK_FORMAT_UNDEFINED;
    VkRenderPass rp_ = VK_NULL_HANDLE;
    VkPipelineLayout layout_ = VK_NULL_HANDLE;
    VkPipeline pipe_ = VK_NULL_HANDLE;
    VkCommandPool pool_ = VK_NULL_HANDLE;
    VkCommandBuffer cmd_ = VK_NULL_HANDLE;
    VkFence fence_ = VK_NULL_HANDLE;
    VkBuffer vbuf_ = VK_NULL_HANDLE;
    VkDeviceMemory vmem_ = VK_NULL_HANDLE;
    Vertex* vmap_ = nullptr;
    const size_t vcap_ = 200000;

    uint64_t seen_det_id_ = 0;
    uint64_t next_track_id_ = 1;
    uint64_t selected_id_ = 0;  // the one track being shown (0 = none)
    std::vector<Track> tracks_;
    uint64_t last_anim_ns_ = 0;
    float frame_dt_ = 0;  // seconds since the previous render frame
    void animate_tracks();
    void suppress_overlapping_tracks(XrVector3f eye, XrQuaternionf orientation);

    // Eye gaze (XR_EXT_eye_gaze_interaction)
    bool gaze_supported_ = false;
    XrActionSet gaze_set_ = XR_NULL_HANDLE;
    XrAction gaze_action_ = XR_NULL_HANDLE;
    XrSpace gaze_space_ = XR_NULL_HANDLE;
    uint64_t gaze_frames_ = 0, gaze_tracked_ = 0, gaze_stat_ns_ = 0;
    void init_gaze();
    bool locate_gaze(XrTime t, XrVector3f& origin, XrVector3f& dir);
};

// A detected object in world (LOCAL) space, smoothed across detector frames.
struct Track {
    uint64_t id;
    int cls;
    float score;
    XrVector3f target[4];  // smoothed measurement
    XrVector3f shown[4];   // what's drawn; chases `target` every render frame
    int hits = 1;
    uint64_t last_seen_ns;
    float alpha = 0;
    float votes[80] = {};  // decaying per-class confidence; cls = argmax, so the label doesn't flip-flop
    float focus_alpha = 0; // eases toward "gaze on the box"; saccades don't make it blink
};

static XrVector3f center(const XrVector3f* c) { return mul(add(add(c[0], c[1]), add(c[2], c[3])), 0.25f); }
static XrVector3f lerp(XrVector3f a, XrVector3f b, float t) { return add(a, mul(sub(b, a), t)); }
static float dot(XrVector3f a, XrVector3f b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static float angle(XrVector3f a, XrVector3f b) { return std::acos(std::clamp(dot(a, b), -1.0f, 1.0f)); }

// Where the gaze ray hits the rectangle (TL, TR, BR, BL) grown by `margin` meters on every side, as the
// larger of the two axis offsets from the center relative to the grown half-size: 0 at the center, 1 on the
// grown edge, > 1 outside (infinity if the ray misses the plane). Rectangular margin: corners are square.
static float gaze_rect_score(const XrVector3f* c, XrVector3f origin, XrVector3f dir, float margin) {
    XrVector3f ex = sub(c[1], c[0]), ey = sub(c[3], c[0]);
    float w = len(ex), h = len(ey);
    ex = mul(ex, 1 / std::max(w, 1e-6f));
    ey = mul(ey, 1 / std::max(h, 1e-6f));
    XrVector3f n = cross(ex, ey);
    float denom = dot(dir, n);
    if (std::fabs(denom) < 1e-3f) return INFINITY;  // edge-on
    float t = dot(sub(c[0], origin), n) / denom;
    if (t <= 0) return INFINITY;  // rectangle is behind the gaze
    XrVector3f rel = sub(add(origin, mul(dir, t)), c[0]);
    float u = dot(rel, ex) - w / 2, v = dot(rel, ey) - h / 2;
    return std::max(std::fabs(u) / (w / 2 + margin), std::fabs(v) / (h / 2 + margin));
}

static int selftest() {
    // 1 m box, 2 m ahead (TL, TR, BR, BL), viewed from the origin; gaze aimed at points on the box plane.
    const XrVector3f box[4] = {{-0.5f, 0.5f, -2}, {0.5f, 0.5f, -2}, {0.5f, -0.5f, -2}, {-0.5f, -0.5f, -2}};
    const XrVector3f o{0, 0, 0};
    struct { XrVector3f dir; float margin; bool want; const char* what; } cases[] = {
        {{0, 0, -1}, 0, true, "through the center"},
        {norm({0.45f, -0.45f, -2}), 0, true, "inside, near a corner"},
        {norm({0.9f, 0, -2}), 0.3f, false, "0.4 m right, margin 0.3"},
        {norm({0.9f, 0, -2}), 0.5f, true, "0.4 m right, margin 0.5"},
        {norm({0.9f, 0.9f, -2}), 0.5f, true, "0.4 m off both edges (corner)"},  // rounded margin would miss
        {norm({0, -1.1f, -2}), 0.5f, false, "0.6 m below, margin 0.5"},
        {{0, 0, 1}, 10, false, "looking away"},
    };
    int fails = 0;
    for (auto& c : cases) {
        bool got = gaze_rect_score(box, o, c.dir, c.margin) <= 1;
        printf("%s %-32s got %d want %d\n", got == c.want ? "ok  " : "FAIL", c.what, got, c.want);
        fails += got != c.want;
    }
    // Best hit: a small box inside a big one wins when you look at the small one.
    const XrVector3f small[4] = {{0.2f, 0.2f, -2}, {0.4f, 0.2f, -2}, {0.4f, 0, -2}, {0.2f, 0, -2}};
    XrVector3f at_small = norm({0.3f, 0.1f, -2});
    bool ok = gaze_rect_score(small, o, at_small, 0.1f) < gaze_rect_score(box, o, at_small, 0.1f);
    printf("%s %-32s\n", ok ? "ok  " : "FAIL", "nested small box scores better");
    fails += !ok;
    return fails ? 1 : 0;
}

void App::init_xr() {
    std::vector<const char*> exts = {XR_KHR_VULKAN_ENABLE2_EXTENSION_NAME, XR_KHR_CONVERT_TIMESPEC_TIME_EXTENSION_NAME};
    uint32_t n = 0;
    XR_CHECK(xrEnumerateInstanceExtensionProperties(nullptr, 0, &n, nullptr));
    std::vector<XrExtensionProperties> avail(n, {XR_TYPE_EXTENSION_PROPERTIES});
    XR_CHECK(xrEnumerateInstanceExtensionProperties(nullptr, n, &n, avail.data()));
    bool has_gaze_ext = std::any_of(avail.begin(), avail.end(), [](const XrExtensionProperties& e) {
        return !strcmp(e.extensionName, XR_EXT_EYE_GAZE_INTERACTION_EXTENSION_NAME);
    });
    if (g_opt.eye_gaze && has_gaze_ext) exts.push_back(XR_EXT_EYE_GAZE_INTERACTION_EXTENSION_NAME);

    XrInstanceCreateInfo ci{XR_TYPE_INSTANCE_CREATE_INFO};
    strcpy(ci.applicationInfo.applicationName, "yoloxr");
    ci.applicationInfo.apiVersion = XR_MAKE_VERSION(1, 0, 0);
    ci.enabledExtensionCount = uint32_t(exts.size());
    ci.enabledExtensionNames = exts.data();
    XR_CHECK(xrCreateInstance(&ci, &instance_));

    XrSystemGetInfo si{XR_TYPE_SYSTEM_GET_INFO};
    si.formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;
    XR_CHECK(xrGetSystem(instance_, &si, &system_));

    if (exts.back() == std::string(XR_EXT_EYE_GAZE_INTERACTION_EXTENSION_NAME)) {
        XrSystemEyeGazeInteractionPropertiesEXT gp{XR_TYPE_SYSTEM_EYE_GAZE_INTERACTION_PROPERTIES_EXT};
        XrSystemProperties sp{XR_TYPE_SYSTEM_PROPERTIES};
        sp.next = &gp;
        XR_CHECK(xrGetSystemProperties(instance_, system_, &sp));
        gaze_supported_ = gp.supportsEyeGazeInteraction;
    }
    printf("xr: eye gaze %s\n", gaze_supported_ ? "supported" : g_opt.eye_gaze ? "unavailable, using head forward" : "disabled");

    XrEnvironmentBlendMode modes[8];
    XR_CHECK(xrEnumerateEnvironmentBlendModes(instance_, system_, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, 8, &n, modes));
    if (std::find(modes, modes + n, XR_ENVIRONMENT_BLEND_MODE_ALPHA_BLEND) == modes + n)
        throw std::runtime_error("runtime has no ALPHA_BLEND (passthrough) blend mode");
    to_xr_time_ = proc<PFN_xrConvertTimespecTimeToTimeKHR>("xrConvertTimespecTimeToTimeKHR");
}

void App::init_vulkan() {
    XrGraphicsRequirementsVulkan2KHR req{XR_TYPE_GRAPHICS_REQUIREMENTS_VULKAN2_KHR};
    XR_CHECK(proc<PFN_xrGetVulkanGraphicsRequirements2KHR>("xrGetVulkanGraphicsRequirements2KHR")(instance_, system_, &req));

    VkApplicationInfo app{VK_STRUCTURE_TYPE_APPLICATION_INFO};
    app.pApplicationName = "yoloxr";
    app.apiVersion = VK_API_VERSION_1_1;
    VkInstanceCreateInfo ici{VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};
    ici.pApplicationInfo = &app;
    XrVulkanInstanceCreateInfoKHR xci{XR_TYPE_VULKAN_INSTANCE_CREATE_INFO_KHR};
    xci.systemId = system_;
    xci.pfnGetInstanceProcAddr = &vkGetInstanceProcAddr;
    xci.vulkanCreateInfo = &ici;
    VkResult vr;
    XR_CHECK(proc<PFN_xrCreateVulkanInstanceKHR>("xrCreateVulkanInstanceKHR")(instance_, &xci, &vk_instance_, &vr));
    VK_CHECK(vr);

    XrVulkanGraphicsDeviceGetInfoKHR gi{XR_TYPE_VULKAN_GRAPHICS_DEVICE_GET_INFO_KHR};
    gi.systemId = system_;
    gi.vulkanInstance = vk_instance_;
    XR_CHECK(proc<PFN_xrGetVulkanGraphicsDevice2KHR>("xrGetVulkanGraphicsDevice2KHR")(instance_, &gi, &phys_));

    uint32_t nq = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(phys_, &nq, nullptr);
    std::vector<VkQueueFamilyProperties> qf(nq);
    vkGetPhysicalDeviceQueueFamilyProperties(phys_, &nq, qf.data());
    for (qfam_ = 0; qfam_ < nq && !(qf[qfam_].queueFlags & VK_QUEUE_GRAPHICS_BIT); qfam_++) {}

    float prio = 1;
    VkDeviceQueueCreateInfo qci{VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};
    qci.queueFamilyIndex = qfam_;
    qci.queueCount = 1;
    qci.pQueuePriorities = &prio;
    VkDeviceCreateInfo dci{VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO};
    dci.queueCreateInfoCount = 1;
    dci.pQueueCreateInfos = &qci;
    XrVulkanDeviceCreateInfoKHR xdci{XR_TYPE_VULKAN_DEVICE_CREATE_INFO_KHR};
    xdci.systemId = system_;
    xdci.pfnGetInstanceProcAddr = &vkGetInstanceProcAddr;
    xdci.vulkanPhysicalDevice = phys_;
    xdci.vulkanCreateInfo = &dci;
    XR_CHECK(proc<PFN_xrCreateVulkanDeviceKHR>("xrCreateVulkanDeviceKHR")(instance_, &xdci, &dev_, &vr));
    VK_CHECK(vr);
    vkGetDeviceQueue(dev_, qfam_, 0, &queue_);
}

void App::init_session() {
    XrGraphicsBindingVulkan2KHR binding{XR_TYPE_GRAPHICS_BINDING_VULKAN2_KHR};
    binding.instance = vk_instance_;
    binding.physicalDevice = phys_;
    binding.device = dev_;
    binding.queueFamilyIndex = qfam_;
    binding.queueIndex = 0;
    XrSessionCreateInfo sci{XR_TYPE_SESSION_CREATE_INFO};
    sci.next = &binding;
    sci.systemId = system_;
    XR_CHECK(xrCreateSession(instance_, &sci, &session_));

    XrReferenceSpaceCreateInfo rs{XR_TYPE_REFERENCE_SPACE_CREATE_INFO};
    rs.poseInReferenceSpace.orientation.w = 1;
    rs.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_LOCAL;
    XR_CHECK(xrCreateReferenceSpace(session_, &rs, &app_space_));
    rs.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_VIEW;
    XR_CHECK(xrCreateReferenceSpace(session_, &rs, &view_space_));

    uint32_t n = 0;
    XR_CHECK(xrEnumerateSwapchainFormats(session_, 0, &n, nullptr));
    std::vector<int64_t> formats(n);
    XR_CHECK(xrEnumerateSwapchainFormats(session_, n, &n, formats.data()));
    format_ = VkFormat(formats[0]);
    for (int64_t want : {VK_FORMAT_R8G8B8A8_SRGB, VK_FORMAT_B8G8R8A8_SRGB})
        if (std::find(formats.begin(), formats.end(), want) != formats.end()) { format_ = VkFormat(want); break; }

    XrViewConfigurationView vcv[2] = {{XR_TYPE_VIEW_CONFIGURATION_VIEW}, {XR_TYPE_VIEW_CONFIGURATION_VIEW}};
    XR_CHECK(xrEnumerateViewConfigurationViews(instance_, system_, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, 2, &n, vcv));
    init_pipeline();
    eyes_.resize(2);
    for (int i = 0; i < 2; i++) {
        Eye& e = eyes_[i];
        e.w = vcv[i].recommendedImageRectWidth;
        e.h = vcv[i].recommendedImageRectHeight;
        XrSwapchainCreateInfo sc{XR_TYPE_SWAPCHAIN_CREATE_INFO};
        sc.usageFlags = XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT;
        sc.format = format_;
        sc.sampleCount = 1;
        sc.width = e.w;
        sc.height = e.h;
        sc.faceCount = 1;
        sc.arraySize = 1;
        sc.mipCount = 1;
        XR_CHECK(xrCreateSwapchain(session_, &sc, &e.swapchain));
        XR_CHECK(xrEnumerateSwapchainImages(e.swapchain, 0, &n, nullptr));
        e.images.resize(n, {XR_TYPE_SWAPCHAIN_IMAGE_VULKAN2_KHR});
        XR_CHECK(xrEnumerateSwapchainImages(e.swapchain, n, &n, (XrSwapchainImageBaseHeader*)e.images.data()));
        for (auto& img : e.images) {
            VkImageViewCreateInfo vi{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
            vi.image = img.image;
            vi.viewType = VK_IMAGE_VIEW_TYPE_2D;
            vi.format = format_;
            vi.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
            VkImageView view;
            VK_CHECK(vkCreateImageView(dev_, &vi, nullptr, &view));
            e.views.push_back(view);
            VkFramebufferCreateInfo fi{VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO};
            fi.renderPass = rp_;
            fi.attachmentCount = 1;
            fi.pAttachments = &view;
            fi.width = e.w;
            fi.height = e.h;
            fi.layers = 1;
            VkFramebuffer fb;
            VK_CHECK(vkCreateFramebuffer(dev_, &fi, nullptr, &fb));
            e.fbs.push_back(fb);
        }
    }
    printf("xr: swapchain %dx%d format %d\n", eyes_[0].w, eyes_[0].h, format_);
    if (gaze_supported_) init_gaze();
}

void App::init_gaze() {
    XrActionSetCreateInfo asi{XR_TYPE_ACTION_SET_CREATE_INFO};
    strcpy(asi.actionSetName, "gaze");
    strcpy(asi.localizedActionSetName, "Gaze");
    XR_CHECK(xrCreateActionSet(instance_, &asi, &gaze_set_));
    XrActionCreateInfo ai{XR_TYPE_ACTION_CREATE_INFO};
    strcpy(ai.actionName, "gaze_pose");
    strcpy(ai.localizedActionName, "Gaze pose");
    ai.actionType = XR_ACTION_TYPE_POSE_INPUT;
    XR_CHECK(xrCreateAction(gaze_set_, &ai, &gaze_action_));

    XrPath profile, pose_path;
    XR_CHECK(xrStringToPath(instance_, "/interaction_profiles/ext/eye_gaze_interaction", &profile));
    XR_CHECK(xrStringToPath(instance_, "/user/eyes_ext/input/gaze_ext/pose", &pose_path));
    XrActionSuggestedBinding binding{gaze_action_, pose_path};
    XrInteractionProfileSuggestedBinding sb{XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING};
    sb.interactionProfile = profile;
    sb.countSuggestedBindings = 1;
    sb.suggestedBindings = &binding;
    XR_CHECK(xrSuggestInteractionProfileBindings(instance_, &sb));

    XrSessionActionSetsAttachInfo at{XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO};
    at.countActionSets = 1;
    at.actionSets = &gaze_set_;
    XR_CHECK(xrAttachSessionActionSets(session_, &at));

    XrActionSpaceCreateInfo sci{XR_TYPE_ACTION_SPACE_CREATE_INFO};
    sci.action = gaze_action_;
    sci.poseInActionSpace.orientation.w = 1;
    XR_CHECK(xrCreateActionSpace(session_, &sci, &gaze_space_));
}

// Gaze ray in app space at time t. False when eye tracking isn't available or not currently tracked.
bool App::locate_gaze(XrTime t, XrVector3f& origin, XrVector3f& dir) {
    if (!gaze_space_) return false;
    XrActiveActionSet active{gaze_set_, XR_NULL_PATH};
    XrActionsSyncInfo sync{XR_TYPE_ACTIONS_SYNC_INFO};
    sync.countActiveActionSets = 1;
    sync.activeActionSets = &active;
    if (XR_FAILED(xrSyncActions(session_, &sync))) return false;
    XrSpaceLocation loc{XR_TYPE_SPACE_LOCATION};
    if (XR_FAILED(xrLocateSpace(gaze_space_, app_space_, t, &loc))) return false;
    const XrSpaceLocationFlags need = XR_SPACE_LOCATION_ORIENTATION_VALID_BIT | XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT;
    bool ok = (loc.locationFlags & need) == need;

    gaze_frames_++;
    gaze_tracked_ += ok;
    uint64_t now = now_ns();
    if (now - gaze_stat_ns_ > 5000000000ull) {
        if (gaze_stat_ns_)
            printf("xr: eye gaze tracked in %.0f%% of frames%s\n", 100.0 * gaze_tracked_ / gaze_frames_,
                   state_ == XR_SESSION_STATE_FOCUSED ? "" : " (session not focused: SteamVR dashboard open?)");
        gaze_frames_ = gaze_tracked_ = 0;
        gaze_stat_ns_ = now;
    }
    if (!ok) return false;
    XrVector3f minus_z{0, 0, -1};
    XrQuaternionf_RotateVector3f(&dir, &loc.pose.orientation, &minus_z);
    origin = loc.pose.position;
    return true;
}

void App::init_pipeline() {
    VkAttachmentDescription att{};
    att.format = format_;
    att.samples = VK_SAMPLE_COUNT_1_BIT;
    att.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    att.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    att.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    att.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    att.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    att.finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    VkAttachmentReference ref{0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    VkSubpassDescription sp{};
    sp.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    sp.colorAttachmentCount = 1;
    sp.pColorAttachments = &ref;
    VkRenderPassCreateInfo rpi{VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO};
    rpi.attachmentCount = 1;
    rpi.pAttachments = &att;
    rpi.subpassCount = 1;
    rpi.pSubpasses = &sp;
    VK_CHECK(vkCreateRenderPass(dev_, &rpi, nullptr, &rp_));

    VkPushConstantRange pc{VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(XrMatrix4x4f)};
    VkPipelineLayoutCreateInfo pli{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    pli.pushConstantRangeCount = 1;
    pli.pPushConstantRanges = &pc;
    VK_CHECK(vkCreatePipelineLayout(dev_, &pli, nullptr, &layout_));

    auto module = [&](const uint32_t* code, size_t size) {
        VkShaderModuleCreateInfo mi{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
        mi.codeSize = size;
        mi.pCode = code;
        VkShaderModule m;
        VK_CHECK(vkCreateShaderModule(dev_, &mi, nullptr, &m));
        return m;
    };
    VkPipelineShaderStageCreateInfo stages[2] = {{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO},
                                                 {VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO}};
    stages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
    stages[0].module = module(kVertSpv, sizeof kVertSpv);
    stages[0].pName = "main";
    stages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    stages[1].module = module(kFragSpv, sizeof kFragSpv);
    stages[1].pName = "main";

    VkVertexInputBindingDescription bind{0, sizeof(Vertex), VK_VERTEX_INPUT_RATE_VERTEX};
    VkVertexInputAttributeDescription attrs[2] = {{0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0},
                                                  {1, 0, VK_FORMAT_R32G32B32A32_SFLOAT, 12}};
    VkPipelineVertexInputStateCreateInfo vin{VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};
    vin.vertexBindingDescriptionCount = 1;
    vin.pVertexBindingDescriptions = &bind;
    vin.vertexAttributeDescriptionCount = 2;
    vin.pVertexAttributeDescriptions = attrs;
    VkPipelineInputAssemblyStateCreateInfo ia{VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO};
    ia.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    VkPipelineViewportStateCreateInfo vp{VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO};
    vp.viewportCount = 1;
    vp.scissorCount = 1;
    VkPipelineRasterizationStateCreateInfo rs{VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO};
    rs.polygonMode = VK_POLYGON_MODE_FILL;
    rs.cullMode = VK_CULL_MODE_NONE;
    rs.lineWidth = 1;
    VkPipelineMultisampleStateCreateInfo ms{VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO};
    ms.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
    VkPipelineColorBlendAttachmentState blend{};
    blend.blendEnable = VK_TRUE;  // premultiplied alpha
    blend.srcColorBlendFactor = VK_BLEND_FACTOR_ONE;
    blend.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
    blend.colorBlendOp = VK_BLEND_OP_ADD;
    blend.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
    blend.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
    blend.alphaBlendOp = VK_BLEND_OP_ADD;
    blend.colorWriteMask = 0xf;
    VkPipelineColorBlendStateCreateInfo cb{VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO};
    cb.attachmentCount = 1;
    cb.pAttachments = &blend;
    VkDynamicState dyn[2] = {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
    VkPipelineDynamicStateCreateInfo ds{VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO};
    ds.dynamicStateCount = 2;
    ds.pDynamicStates = dyn;
    VkGraphicsPipelineCreateInfo gpi{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
    gpi.stageCount = 2;
    gpi.pStages = stages;
    gpi.pVertexInputState = &vin;
    gpi.pInputAssemblyState = &ia;
    gpi.pViewportState = &vp;
    gpi.pRasterizationState = &rs;
    gpi.pMultisampleState = &ms;
    gpi.pColorBlendState = &cb;
    gpi.pDynamicState = &ds;
    gpi.layout = layout_;
    gpi.renderPass = rp_;
    VK_CHECK(vkCreateGraphicsPipelines(dev_, VK_NULL_HANDLE, 1, &gpi, nullptr, &pipe_));

    VkCommandPoolCreateInfo cpi{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
    cpi.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    cpi.queueFamilyIndex = qfam_;
    VK_CHECK(vkCreateCommandPool(dev_, &cpi, nullptr, &pool_));
    VkCommandBufferAllocateInfo cai{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
    cai.commandPool = pool_;
    cai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    cai.commandBufferCount = 1;
    VK_CHECK(vkAllocateCommandBuffers(dev_, &cai, &cmd_));
    VkFenceCreateInfo fci{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
    VK_CHECK(vkCreateFence(dev_, &fci, nullptr, &fence_));

    VkBufferCreateInfo bi{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    bi.size = vcap_ * sizeof(Vertex);
    bi.usage = VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
    VK_CHECK(vkCreateBuffer(dev_, &bi, nullptr, &vbuf_));
    VkMemoryRequirements mr;
    vkGetBufferMemoryRequirements(dev_, vbuf_, &mr);
    VkPhysicalDeviceMemoryProperties mp;
    vkGetPhysicalDeviceMemoryProperties(phys_, &mp);
    VkMemoryAllocateInfo mai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
    mai.allocationSize = mr.size;
    const VkMemoryPropertyFlags want = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
    for (mai.memoryTypeIndex = 0; mai.memoryTypeIndex < mp.memoryTypeCount; mai.memoryTypeIndex++)
        if ((mr.memoryTypeBits >> mai.memoryTypeIndex & 1) &&
            (mp.memoryTypes[mai.memoryTypeIndex].propertyFlags & want) == want)
            break;
    VK_CHECK(vkAllocateMemory(dev_, &mai, nullptr, &vmem_));
    VK_CHECK(vkBindBufferMemory(dev_, vbuf_, vmem_, 0));
    VK_CHECK(vkMapMemory(dev_, vmem_, 0, VK_WHOLE_SIZE, 0, (void**)&vmap_));
}

void App::handle_events() {
    XrEventDataBuffer ev{XR_TYPE_EVENT_DATA_BUFFER};
    while (xrPollEvent(instance_, &ev) == XR_SUCCESS) {
        if (ev.type == XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED) {
            state_ = ((XrEventDataSessionStateChanged&)ev).state;
            printf("xr: session state %d\n", state_);
            if (state_ == XR_SESSION_STATE_READY) {
                XrSessionBeginInfo bi{XR_TYPE_SESSION_BEGIN_INFO};
                bi.primaryViewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
                XR_CHECK(xrBeginSession(session_, &bi));
                running_ = true;
            } else if (state_ == XR_SESSION_STATE_STOPPING) {
                XR_CHECK(xrEndSession(session_));
                running_ = false;
            } else if (state_ == XR_SESSION_STATE_EXITING || state_ == XR_SESSION_STATE_LOSS_PENDING) {
                exit_ = true;
            }
        } else if (ev.type == XR_TYPE_EVENT_DATA_INSTANCE_LOSS_PENDING) {
            exit_ = true;
        }
        ev = {XR_TYPE_EVENT_DATA_BUFFER};
    }
}

// Re-anchor the newest detections in LOCAL space using the head pose at their capture time.
void App::update_world_boxes() {
    DetectionSet d;
    {
        std::lock_guard<std::mutex> lk(g_det_mutex);
        if (g_dets.id == seen_det_id_) return;
        d = g_dets;
    }
    seen_det_id_ = d.id;
    timespec ts{time_t(d.t_capture_ns / 1000000000ull), long(d.t_capture_ns % 1000000000ull)};
    XrTime t;
    XR_CHECK(to_xr_time_(instance_, &ts, &t));
    XrSpaceLocation loc{XR_TYPE_SPACE_LOCATION};
    XR_CHECK(xrLocateSpace(view_space_, app_space_, t, &loc));
    if (!(loc.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT)) return;
    auto to_world = [&](XrVector3f p) {
        XrVector3f r;
        XrQuaternionf_RotateVector3f(&r, &loc.pose.orientation, &p);
        return add(r, loc.pose.position);
    };
    const uint64_t now = now_ns();

    // Associate measurements to tracks by direction (as seen from the capture-time camera), normalized by
    // the box's angular size, so near and far objects use the same gate. Stereo first: it carries depth.
    struct Meas { Obj o; XrVector3f origin, dir; float radius; };
    std::vector<Meas> ms;
    for (Obj o : d.objs) {
        for (XrVector3f& c : o.corners) c = to_world(c);
        XrVector3f origin = to_world(o.origin), ctr = center(o.corners);
        XrVector3f dir = norm(sub(ctr, origin));
        ms.push_back({o, origin, dir, std::max(angle(dir, norm(sub(o.corners[0], origin))), 0.02f)});
    }
    std::stable_sort(ms.begin(), ms.end(), [](const Meas& a, const Meas& b) { return a.o.stereo > b.o.stereo; });
    std::vector<bool> updated(tracks_.size());
    for (const Meas& m : ms) {
        int best = -1;
        float best_cost = 0.7f;  // gate: centers within 0.7 box half-diagonals
        for (size_t i = 0; i < tracks_.size(); i++) {
            const Track& tr = tracks_[i];
            if (updated[i]) continue;
            // Class-agnostic (YOLO relabels the same object between frames), mild preference for same class.
            float cost = angle(m.dir, norm(sub(center(tr.target), m.origin))) / m.radius + (tr.cls != m.o.cls) * 0.2f;
            if (m.o.stereo) {  // reject wildly different depths
                float dm = len(sub(center(m.o.corners), m.origin)), dt = len(sub(center(tr.target), m.origin));
                if (std::fabs(std::log(dm / std::max(dt, 1e-3f))) > 0.5f) continue;
            }
            if (cost < best_cost) { best_cost = cost; best = int(i); }
        }
        XrVector3f meas[4];
        if (m.o.stereo) {
            std::copy(m.o.corners, m.o.corners + 4, meas);
        } else {
            if (best < 0) continue;  // a single view can't start a track (no depth)
            // Put the 1 m-plane corners at the track's current distance.
            const XrVector3f* tc = tracks_[best].target;
            float s = len(sub(center(tc), m.origin)) / len(sub(center(m.o.corners), m.origin));
            for (int k = 0; k < 4; k++) meas[k] = add(m.origin, mul(sub(m.o.corners[k], m.origin), s));
        }
        if (best < 0) {
            Track tr{next_track_id_++, m.o.cls, m.o.score, {}, {}, 1, now, 0};
            tr.votes[m.o.cls] = m.o.score;
            std::copy(meas, meas + 4, tr.target);
            std::copy(meas, meas + 4, tr.shown);
            tracks_.push_back(tr);
            updated.push_back(true);
            continue;
        }
        Track& tr = tracks_[best];
        updated[best] = true;
        // ponytail: fixed-gain smoothing (depth from ~5-20 px disparity is noisy); a Kalman filter
        // with per-axis noise would track motion better if objects move fast
        for (int k = 0; k < 4; k++) tr.target[k] = lerp(tr.target[k], meas[k], 0.5f);
        tr.score = 0.7f * tr.score + 0.3f * m.o.score;
        for (float& v : tr.votes) v *= 0.9f;
        tr.votes[m.o.cls] += m.o.score;
        tr.cls = int(std::max_element(tr.votes, tr.votes + 80) - tr.votes);
        tr.hits++;
        tr.last_seen_ns = now;
    }
}

// Tracks that overlap as seen from the current eye (IoU of their projected boxes) keep only the most
// confident one. Catches duplicates the detector-side NMS can't: tracks spawned in different frames.
void App::suppress_overlapping_tracks(XrVector3f eye, XrQuaternionf orientation) {
    XrQuaternionf inv{-orientation.x, -orientation.y, -orientation.z, orientation.w};
    std::vector<Det> boxes;  // projected on the z = -1 plane of the eye frame
    for (const Track& tr : tracks_) {
        Det b{1e9f, 1e9f, -1e9f, -1e9f, tr.score, 0};
        for (const XrVector3f& c : tr.shown) {
            XrVector3f rel = sub(c, eye), p;
            XrQuaternionf_RotateVector3f(&p, &inv, &rel);
            if (p.z > -0.05f) { b.score = -1; break; }  // behind / at the eye: don't compare
            float x = p.x / -p.z, y = p.y / -p.z;
            b.x0 = std::min(b.x0, x); b.x1 = std::max(b.x1, x);
            b.y0 = std::min(b.y0, y); b.y1 = std::max(b.y1, y);
        }
        boxes.push_back(b);
    }
    std::vector<bool> drop(tracks_.size());
    for (size_t i = 0; i < tracks_.size(); i++)
        for (size_t j = i + 1; j < tracks_.size(); j++) {
            if (drop[i] || drop[j] || boxes[i].score < 0 || boxes[j].score < 0) continue;
            if (duplicate(boxes[i], boxes[j])) drop[tracks_[i].score < tracks_[j].score ? i : j] = true;
        }
    size_t k = 0;
    for (size_t i = 0; i < tracks_.size(); i++)
        if (!drop[i]) tracks_[k++] = tracks_[i];
    tracks_.resize(k);
}

// Per render frame: glide shown boxes toward their targets, fade in confirmed tracks, fade out stale ones.
void App::animate_tracks() {
    uint64_t now = now_ns();
    float dt = last_anim_ns_ ? std::min((now - last_anim_ns_) / 1e9f, 0.1f) : 0;
    last_anim_ns_ = now;
    frame_dt_ = dt;
    float k_pos = 1 - std::exp(-dt / (g_opt.smooth_ms / 1000));
    float k_alpha = 1 - std::exp(-dt / 0.1f);
    for (Track& tr : tracks_) {
        for (int k = 0; k < 4; k++) tr.shown[k] = lerp(tr.shown[k], tr.target[k], k_pos);
        bool live = tr.hits >= g_opt.confirm_hits && (now - tr.last_seen_ns) < g_opt.hold_ms * 1e6;
        tr.alpha += ((live ? 1.0f : 0.0f) - tr.alpha) * k_alpha;
    }
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                                 [&](const Track& tr) {
                                     return (now - tr.last_seen_ns) > g_opt.hold_ms * 1e6 && tr.alpha < 0.01f;
                                 }),
                  tracks_.end());
}

void App::render_eye(Eye& e, const XrView& view, uint32_t nverts) {
    uint32_t idx;
    XrSwapchainImageAcquireInfo ai{XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO};
    XR_CHECK(xrAcquireSwapchainImage(e.swapchain, &ai, &idx));
    XrSwapchainImageWaitInfo wi{XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO};
    wi.timeout = XR_INFINITE_DURATION;
    XR_CHECK(xrWaitSwapchainImage(e.swapchain, &wi));

    XrMatrix4x4f proj, to_view, view_m, vp;
    XrMatrix4x4f_CreateProjectionFov(&proj, GRAPHICS_VULKAN, view.fov, 0.05f, 100.0f);
    XrVector3f one{1, 1, 1};
    XrMatrix4x4f_CreateTranslationRotationScale(&to_view, &view.pose.position, &view.pose.orientation, &one);
    XrMatrix4x4f_InvertRigidBody(&view_m, &to_view);
    XrMatrix4x4f_Multiply(&vp, &proj, &view_m);

    VK_CHECK(vkResetCommandBuffer(cmd_, 0));
    VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
    bi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    VK_CHECK(vkBeginCommandBuffer(cmd_, &bi));
    VkClearValue clear{};  // transparent: passthrough shows through
    VkRenderPassBeginInfo rbi{VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO};
    rbi.renderPass = rp_;
    rbi.framebuffer = e.fbs[idx];
    rbi.renderArea = {{0, 0}, {uint32_t(e.w), uint32_t(e.h)}};
    rbi.clearValueCount = 1;
    rbi.pClearValues = &clear;
    vkCmdBeginRenderPass(cmd_, &rbi, VK_SUBPASS_CONTENTS_INLINE);
    if (nverts) {
        VkViewport v{0, 0, float(e.w), float(e.h), 0, 1};
        VkRect2D sc{{0, 0}, {uint32_t(e.w), uint32_t(e.h)}};
        vkCmdSetViewport(cmd_, 0, 1, &v);
        vkCmdSetScissor(cmd_, 0, 1, &sc);
        vkCmdBindPipeline(cmd_, VK_PIPELINE_BIND_POINT_GRAPHICS, pipe_);
        vkCmdPushConstants(cmd_, layout_, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof vp, &vp);
        VkDeviceSize off = 0;
        vkCmdBindVertexBuffers(cmd_, 0, 1, &vbuf_, &off);
        vkCmdDraw(cmd_, nverts, 1, 0, 0);
    }
    vkCmdEndRenderPass(cmd_);
    VK_CHECK(vkEndCommandBuffer(cmd_));
    VkSubmitInfo si{VK_STRUCTURE_TYPE_SUBMIT_INFO};
    si.commandBufferCount = 1;
    si.pCommandBuffers = &cmd_;
    VK_CHECK(vkQueueSubmit(queue_, 1, &si, fence_));
    // ponytail: wait per eye; trivial GPU work, keeps the single vertex buffer race-free
    VK_CHECK(vkWaitForFences(dev_, 1, &fence_, VK_TRUE, UINT64_MAX));
    VK_CHECK(vkResetFences(dev_, 1, &fence_));

    XrSwapchainImageReleaseInfo ri{XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
    XR_CHECK(xrReleaseSwapchainImage(e.swapchain, &ri));
}

void App::render_frame() {
    XrFrameState fs{XR_TYPE_FRAME_STATE};
    XrFrameWaitInfo fwi{XR_TYPE_FRAME_WAIT_INFO};
    XR_CHECK(xrWaitFrame(session_, &fwi, &fs));
    XrFrameBeginInfo fbi{XR_TYPE_FRAME_BEGIN_INFO};
    XR_CHECK(xrBeginFrame(session_, &fbi));

    XrCompositionLayerProjection layer{XR_TYPE_COMPOSITION_LAYER_PROJECTION};
    XrCompositionLayerProjectionView pv[2] = {{XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW},
                                              {XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW}};
    const XrCompositionLayerBaseHeader* layers[1] = {(XrCompositionLayerBaseHeader*)&layer};
    uint32_t layer_count = 0;

    if (fs.shouldRender) {
        XrViewState vs{XR_TYPE_VIEW_STATE};
        XrViewLocateInfo vli{XR_TYPE_VIEW_LOCATE_INFO};
        vli.viewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
        vli.displayTime = fs.predictedDisplayTime;
        vli.space = app_space_;
        XrView views[2] = {{XR_TYPE_VIEW}, {XR_TYPE_VIEW}};
        uint32_t n;
        XR_CHECK(xrLocateViews(session_, &vli, &vs, 2, &n, views));

        update_world_boxes();
        animate_tracks();
        XrVector3f eye = mul(add(views[0].pose.position, views[1].pose.position), 0.5f);
        suppress_overlapping_tracks(eye, views[0].pose.orientation);
        // Only show the object you're looking at (eye gaze if tracked, else head forward).
        // Per-track easing keeps saccades from blinking boxes.
        XrVector3f gaze_origin = eye, gaze_dir, minus_z{0, 0, -1};
        if (!locate_gaze(fs.predictedDisplayTime, gaze_origin, gaze_dir))
            XrQuaternionf_RotateVector3f(&gaze_dir, &views[0].pose.orientation, &minus_z);
        float k_gaze = 1 - std::exp(-std::min(frame_dt_, 0.1f) / 0.15f);
        std::vector<Vertex> verts;
        // Pick the one confirmed track the gaze hits best. Hit target: the box grown on every side by
        // --focus-margin label heights (scales with distance like the label), scored 0 at the center to 1 at
        // the grown edge, so a small object inside a big one wins when you look at it. The current pick
        // gets a bonus (hysteresis) and its label counts as a center hit, so reading it keeps it up.
        uint64_t best_id = 0;
        float best = 1;
        for (const Track& tr : tracks_) {
            if (tr.alpha < 0.01f) continue;
            LabelLayout label = layout_label(tr.shown, tr.cls, tr.score, eye);
            float s = gaze_rect_score(tr.shown, gaze_origin, gaze_dir, g_opt.focus_margin * label.height());
            if (tr.id == selected_id_) {
                XrVector3f rect[4];
                label.rect(rect);
                if (gaze_rect_score(rect, gaze_origin, gaze_dir, 0) <= 1) s = 0;
                s *= 0.8f;
            }
            if (s <= best) { best = s; best_id = tr.id; }
        }
        selected_id_ = best_id;
        for (Track& tr : tracks_) {
            tr.focus_alpha += ((tr.id == selected_id_ ? 1.0f : 0.0f) - tr.focus_alpha) * k_gaze;
            float a = tr.alpha * tr.focus_alpha;
            if (a > 0.01f) push_box(verts, tr.shown, tr.cls, tr.score, a, eye);
        }
        uint32_t nverts = uint32_t(std::min(verts.size(), vcap_));
        memcpy(vmap_, verts.data(), nverts * sizeof(Vertex));

        for (int i = 0; i < 2; i++) {
            render_eye(eyes_[i], views[i], nverts);
            pv[i].pose = views[i].pose;
            pv[i].fov = views[i].fov;
            pv[i].subImage.swapchain = eyes_[i].swapchain;
            pv[i].subImage.imageRect = {{0, 0}, {eyes_[i].w, eyes_[i].h}};
        }
        layer.layerFlags = XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT;
        layer.space = app_space_;
        layer.viewCount = 2;
        layer.views = pv;
        layer_count = 1;
    }

    XrFrameEndInfo fei{XR_TYPE_FRAME_END_INFO};
    fei.displayTime = fs.predictedDisplayTime;
    fei.environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_ALPHA_BLEND;
    fei.layerCount = layer_count;
    fei.layers = layers;
    XR_CHECK(xrEndFrame(session_, &fei));
}

void App::run() {
    init_xr();
    init_vulkan();
    init_session();
    bool logged_eyes = false;
    while (!exit_) {
        if (g_quit && !exit_requested_) {
            if (running_) xrRequestExitSession(session_);
            else exit_ = true;
            exit_requested_ = true;
        }
        handle_events();
        if (!running_) {
            usleep(10000);
            continue;
        }
        if (!logged_eyes && state_ >= XR_SESSION_STATE_VISIBLE) {
            // Eyes relative to VIEW space: tells whether VIEW is the head frame the calibration uses.
            XrViewState vs{XR_TYPE_VIEW_STATE};
            XrViewLocateInfo vli{XR_TYPE_VIEW_LOCATE_INFO};
            vli.viewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
            vli.displayTime = 1;
            XrTime t;
            timespec ts;
            clock_gettime(CLOCK_MONOTONIC, &ts);
            to_xr_time_(instance_, &ts, &t);
            vli.displayTime = t;
            vli.space = view_space_;
            XrView v[2] = {{XR_TYPE_VIEW}, {XR_TYPE_VIEW}};
            uint32_t n;
            if (XR_SUCCEEDED(xrLocateViews(session_, &vli, &vs, 2, &n, v)))
                for (int i = 0; i < 2; i++)
                    printf("xr: eye %d in VIEW: pos (%.4f %.4f %.4f) rot (%.3f %.3f %.3f %.3f)\n", i, v[i].pose.position.x,
                           v[i].pose.position.y, v[i].pose.position.z, v[i].pose.orientation.x, v[i].pose.orientation.y,
                           v[i].pose.orientation.z, v[i].pose.orientation.w);
            logged_eyes = true;
        }
        render_frame();
    }
    if (session_) xrDestroySession(session_);
    if (instance_) xrDestroyInstance(instance_);
}

// ---------------------------------------------------------------------------------------------

int main(int argc, char** argv) {
    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        auto next = [&]() -> const char* {
            if (i + 1 >= argc) throw std::runtime_error("missing value for " + a);
            return argv[++i];
        };
        if (a == "--selftest") return selftest();
        else if (a == "--no-xr") g_opt.xr = false;
        else if (a == "--dump") g_opt.dump = next();
        else if (a == "--model") g_opt.model = next();
        else if (a == "--calib") g_opt.calib = next();
        else if (a == "--size") g_opt.size = atoi(next());
        else if (a == "--hfov") g_opt.hfov_deg = float(atof(next()));
        else if (a == "--conf") g_opt.conf = float(atof(next()));
        else if (a == "--iou") g_opt.iou = float(atof(next()));
        else if (a == "--contain") g_opt.contain = float(atof(next()));
        else if (a == "--latency-ms") g_opt.latency_ms = float(atof(next()));
        else if (a == "--smooth-ms") g_opt.smooth_ms = std::max(1.0f, float(atof(next())));
        else if (a == "--confirm-hits") g_opt.confirm_hits = atoi(next());
        else if (a == "--hold-ms") g_opt.hold_ms = float(atof(next()));
        else if (a == "--focus-margin") g_opt.focus_margin = float(atof(next()));
        else if (a == "--no-eye-gaze") g_opt.eye_gaze = false;
        else { fprintf(stderr, "unknown option %s\n", a.c_str()); return 2; }
    }
    if (g_opt.size % 32) { fprintf(stderr, "--size must be a multiple of 32\n"); return 2; }
    if (g_opt.model.empty()) g_opt.model = "../models/yolo11n_" + std::to_string(g_opt.size) + ".ncnn";
    setvbuf(stdout, nullptr, _IOLBF, 0);
    signal(SIGINT, [](int) { g_quit = true; });
    signal(SIGTERM, [](int) { g_quit = true; });

    std::thread det([] {
        try {
            detect_loop();
        } catch (const std::exception& e) {
            fprintf(stderr, "detector: %s\n", e.what());
            g_quit = true;
        }
    });
    int rc = 0;
    try {
        if (g_opt.xr) App().run();
        else while (!g_quit) usleep(100000);
    } catch (const std::exception& e) {
        fprintf(stderr, "xr: %s\n", e.what());
        rc = 1;
    }
    g_quit = true;
    det.join();
    ncnn::destroy_gpu_instance();
    return rc;
}
