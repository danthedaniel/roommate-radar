// guardian's stereo YOLO detector thread. Reads the two SLAM camera frames published by
// camserver, rectifies each into a virtual pinhole camera aligned with the head frame (so the pair is a rectified
// stereo pair, calib.h), runs YOLO11n (ncnn, Vulkan) on both views stacked into one image, matches detections across
// views and triangulates depth from disparity (stereo.h). Results (head frame at capture time) go to g_dets.
#pragma once
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <net.h>

#include "calib.h"
#include "shm.h"
#include "stereo.h"

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

// Seconds from `then` to `now` (CLOCK_MONOTONIC ns), signed: negating an unsigned difference wraps around.
static float seconds(uint64_t now, uint64_t then) {
    return float(int64_t(now - then)) / 1e9f;
}

// ---------------------------------------------------------------------------------------------
// Detector options

struct DetectorOptions {
    std::string dump;  // directory for debug PGMs (empty = off)
    // default models/yolo11n-grayscale_<size>_gray.ncnn (exported at exactly size x 2*size, then prep_model.py)
    std::string model;
    std::string calib = "calib.txt";
    int size = 576;        // per-view rectified size (YOLO input is size x 2*size)
    float hfov_deg = 100;  // virtual camera field of view
    float conf = 0.25f;
    float iou = 0.5f;       // overlapping detections/tracks above this IoU keep only the most confident
    float contain = 0.8f;   // ...same if this fraction of the smaller box lies inside the larger one
    float min_box = 0.05f;  // drop boxes whose larger side is under this fraction of the view (a cat at 3 m: ~6% long)
    float latency_ms = 8;   // camserver arrival time -> mid-exposure; tune if boxes lag/lead head motion
    float max_hz = 0;       // run YOLO at most this often (0 = every camera frame pair)
};
static DetectorOptions g_det;
static std::atomic<bool> g_det_paused{false};  // skip frames (no GPU work) while set
static std::atomic<int> g_det_size{0};         // per-view size to run at (set from g_det.size); a change reloads
static std::atomic<float> g_det_hz{0};         // max_hz, changeable while running
static std::atomic<int> g_det_loading{0};    // size being loaded (model, then its first run compiles shaders); 0 = done
static std::atomic<float> g_det_yolo_ms{0};  // mean YOLO time over the last stats period

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
    } else if (a == "--min-box") {
        g_det.min_box = float(atof(next()));
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

// The frames driver_guardian publishes, once it has; nullptr on quit.
static const shm::Header* open_frames() {
    // driver_guardian creates the file before sizing it; touching a mapping past its end is SIGBUS.
    int fd = -1;
    struct stat st = {};
    for (bool logged = false; !g_quit && ((fd = open(shm::kPath, O_RDONLY)) < 0 || fstat(fd, &st) ||
                                          st.st_size < off_t(sizeof(shm::Header)));
         logged = true) {
        if (fd >= 0) {
            close(fd);
        }
        if (!logged) {
            fprintf(stderr, "waiting for driver_guardian (%s; restart SteamVR after the first run)\n", shm::kPath);
        }
        sleep(1);
    }
    if (g_quit) {
        return nullptr;
    }
    void* p = mmap(nullptr, sizeof(shm::Header), PROT_READ, MAP_SHARED, fd, 0);
    close(fd);
    if (p == MAP_FAILED) {
        throw std::runtime_error(std::string("mmap ") + shm::kPath + ": " + strerror(errno));
    }
    return (const shm::Header*)p;
}

// The SLAM frames are dim and low-contrast; stretch the 1st..99th percentile to full range.
// Pixels outside the fisheye are 0 -> YOLO's letterbox gray (114), so they don't look like edges.
static void stretch(std::vector<uint8_t>& input) {
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
}

// YOLO's output [4 + 80, N] (cx, cy, w, h in input pixels, then per-class sigmoid scores) as boxes in each S x S view.
static void decode(const ncnn::Mat& out, int S, std::vector<Det> views[2]) {
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
        // > 4 px tall: not a sliver clipped at the seam between the views
        if (d.y1 - d.y0 > 4 && std::max(d.x1 - d.x0, d.y1 - d.y0) >= g_det.min_box * S) {
            views[v].push_back(d);
        }
    }
}

// --dump: the stacked input with each view's boxes as a PGM in g_det.dump, and the stereo objects to stdout.
static void dump(const std::vector<uint8_t>& input, const std::vector<Det> views[2], const std::vector<Obj>& objs,
                 int S) {
    static uint64_t n = 0;
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
    snprintf(path, sizeof path, "%s/det_%04llu.pgm", g_det.dump.c_str(), (unsigned long long)n++);
    if (FILE* f = fopen(path, "wb")) {
        fprintf(f, "P5\n%d %d\n255\n", S, 2 * S);
        fwrite(img.data(), 1, img.size(), f);
        fclose(f);
    }
    for (const Obj& o : objs) {
        if (o.stereo) {
            printf("  %s %.2f at %.2fm\n", kCoco[o.cls], o.score, o.dist);
        }
    }
}

// One run at size S, until g_det_size changes or quit.
static void detect_run(const int S) {
    g_det_loading = S;
    const uint64_t t_load = now_ns();
    ncnn::Net net;
    net.opt.use_vulkan_compute = ncnn::get_gpu_count() > 0;
    net.opt.num_threads = 2;
    // ~1 ms faster on the Adreno 750 (Turnip) at every size; ncnn's other Vulkan defaults measured best.
    net.opt.use_sgemm_convolution = false;
    net.opt.use_winograd63_convolution = false;
    const std::string model =
        g_det.model.empty() ? "models/yolo11n-grayscale_" + std::to_string(S) + "_gray.ncnn" : g_det.model;
    if (net.load_param((model + ".param").c_str()) || net.load_model((model + ".bin").c_str())) {
        throw std::runtime_error("failed to load model " + model + " (see prep_model.py)");
    }
    printf("detector: ncnn %s\n", net.opt.use_vulkan_compute ? "Vulkan GPU" : "CPU");

    const shm::Header* hdr = open_frames();
    if (!hdr) {
        return;
    }
    // After the frames: calib.txt comes from the driver's copy of the factory calibration (guardian.py)
    const CamCal cal[2] = {load_calib(g_det.calib, shm::kCamNames[0]), load_calib(g_det.calib, shm::kCamNames[1])};
    Rectifier rect[2] = {Rectifier(cal[0], S, g_det.hfov_deg), Rectifier(cal[1], S, g_det.hfov_deg)};
    const float f = rect[0].f;
    const float baseline = cal[1].t[0] - cal[0].t[0];  // right cam is at +x in the head frame
    printf("detector: %dx%d views, f=%.1f px, baseline %.1f mm\n", S, S, f, baseline * 1000);

    // A torn read leaves garbage in its buffer, so frames are read into `scratch` and swapped in only when intact.
    std::vector<uint8_t> raw[2] = {std::vector<uint8_t>(shm::kStride * shm::kH),
                                   std::vector<uint8_t>(shm::kStride * shm::kH)};
    std::vector<uint8_t> scratch(shm::kStride * shm::kH);
    std::vector<uint8_t> input(size_t(S) * S * 2);
    uint64_t last_seq[2] = {0, 0}, t_ns[2] = {0, 0}, last_pair = 0;
    uint64_t frames = 0, stat_t = now_ns(), last_run = 0;
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
            if (shm::read(hdr->slots[i], last_seq[i], scratch.data(), t_ns[i])) {
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
        stretch(input);
        uint64_t b = now_ns();

        // prep_model.py folded GRAY2RGB and the 1/255 scale into the first conv.
        ncnn::Mat in = ncnn::Mat::from_pixels(input.data(), ncnn::Mat::PIXEL_GRAY, S, 2 * S);
        ncnn::Extractor ex = net.create_extractor();
        ex.input("in0", in);
        ncnn::Mat out;
        ex.extract("out0", out);
        if (g_det_loading) {
            printf("detector: %dpx ready %.1f s after loading started\n", S, (now_ns() - t_load) / 1e9);
            g_det_loading = 0;
        }
        uint64_t c = now_ns();
        if (!printed_shape) {
            printf("detector: out0 w=%d h=%d c=%d\n", out.w, out.h, out.c);
            printed_shape = true;
        }
        if (out.h < 4 + 80) {  // extract failed, or not a COCO head: out.row(4 + k) below would read past the end
            continue;
        }

        std::vector<Det> views[2];
        decode(out, S, views);
        nms(views[0]);
        nms(views[1]);

        const std::vector<Obj> objs = match_stereo(views, input.data(), input.data() + size_t(S) * S, S, f, baseline,
                                                   {cal[0].t[0], cal[0].t[1], cal[0].t[2]});
        uint64_t e = now_ns();

        {
            std::lock_guard<std::mutex> lk(g_det_mutex);
            g_dets.id++;
            g_dets.t_capture_ns = t_ns[0] - uint64_t(g_det.latency_ms * 1e6);
            g_dets.objs = objs;
        }

        if (!g_det.dump.empty() && frames % 30 == 0) {
            dump(input, views, objs, S);
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
}

static void detect_loop() {
    while (!g_quit) {
        detect_run(g_det_size);
    }
}
