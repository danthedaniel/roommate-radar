// yoloxr: YOLO bounding boxes over Steam Frame passthrough.
//
// Detection thread: detector.h (stereo YOLO on the SLAM cameras).
//
// XR thread: OpenXR scene app with ALPHA_BLEND, so the runtime shows passthrough wherever we draw
// nothing. Boxes are placed in world space using the head pose at frame capture time, and drawn as
// thick outlines with a label.
//
// Usage: yoloxr [--no-xr] [--dump DIR] [--size 480] [--hfov 100] [--conf 0.35] [--latency-ms 8]
//        yoloxr --gaze-cal 1.5   (9-dot eye gaze accuracy check, dots 1.5 m away; no detector)
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

#include "detector.h"

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

// ---------------------------------------------------------------------------------------------
// Options

struct Options {
    bool xr = true;
    float smooth_ms = 80;        // displayed boxes glide toward the latest measurement with this time constant
    int confirm_hits = 3;        // detections needed before a track is shown (debounce)
    float hold_ms = 400;         // keep showing a track this long after its last detection, then fade out
    float focus_margin = 2;      // show a box (and label) only while the gaze hits the box grown by this many label heights
    bool eye_gaze = true;        // use eye tracking for the gaze direction (falls back to head forward)
    float gaze_cal_m = 0;        // > 0: run the 9-dot gaze calibration with the dots this far away instead
};
static Options g_opt;

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

// One line of text centered on `c`, `height` meters tall, in the plane of unit vectors right/up.
static LabelLayout layout_text(const char* text, XrVector3f c, XrVector3f right, XrVector3f up, float height) {
    LabelLayout l;
    snprintf(l.text, sizeof l.text, "%s", text);
    l.tw = float(stb_easy_font_width(l.text));
    l.right = right;
    l.up = up;
    l.scale = height / l.th;
    l.origin = add(c, add(mul(right, -l.tw / 2 * l.scale), mul(up, l.th / 2 * l.scale)));
    return l;
}

static void push_label(std::vector<Vertex>& vs, LabelLayout& l, const float rgb[3], float alpha) {
    static char buf[64 * 1024];
    int quads = stb_easy_font_print(0, 0, l.text, nullptr, buf, sizeof buf);
    XrVector3f bg_rect[4];
    l.rect(bg_rect);
    float bg[4] = {0, 0, 0, 0.55f * alpha};
    push_quad(vs, bg_rect[0], bg_rect[1], bg_rect[2], bg_rect[3], bg);
    float fg[4] = {rgb[0], rgb[1], rgb[2], alpha};
    const float* q = (const float*)buf;  // 4 verts per quad: x, y, z, color(4 bytes)
    for (int i = 0; i < quads; i++, q += 16)
        push_quad(vs, l.at(q[0], q[1]), l.at(q[4], q[5]), l.at(q[8], q[9]), l.at(q[12], q[13]), fg);
}

static void push_box(std::vector<Vertex>& vs, const XrVector3f* corners, int cls, float score, float alpha, XrVector3f eye) {
    float rgba[4] = {0, 0, 0, alpha};
    color_for(cls, rgba);
    float dist = len(sub(add(corners[0], corners[2]), mul(eye, 2))) / 2;
    float width = 0.004f * dist;  // ~0.23 degrees regardless of distance
    for (int k = 0; k < 4; k++) push_line(vs, corners[k], corners[(k + 1) % 4], eye, width, rgba);
    LabelLayout l = layout_label(corners, cls, score, eye);
    push_label(vs, l, rgba, alpha);
}

// Ring from radius r0 to r1 around c (r0 = 0: disc) in the plane of unit vectors x/y, covering `frac` of a turn
// clockwise from 12 o'clock.
static void push_ring(std::vector<Vertex>& vs, XrVector3f c, XrVector3f x, XrVector3f y, float r0, float r1, float frac,
                      const float rgba[4]) {
    int n = int(std::ceil(48 * frac));
    auto at = [&](int i, float r) {
        float a = 2 * float(M_PI) * frac * i / n;
        return add(c, add(mul(x, r * std::sin(a)), mul(y, r * std::cos(a))));
    };
    for (int i = 0; i < n; i++) push_quad(vs, at(i, r0), at(i, r1), at(i + 1, r1), at(i + 1, r0), rgba);
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

    // Pointer: the aim ray of the controller whose grip was clicked last (else the other one), else eye gaze
    // (XR_EXT_eye_gaze_interaction), else head forward
    bool gaze_supported_ = false;
    XrActionSet input_set_ = XR_NULL_HANDLE;
    XrAction gaze_action_ = XR_NULL_HANDLE, aim_action_ = XR_NULL_HANDLE, grip_action_ = XR_NULL_HANDLE;
    XrPath hand_path_[2] = {};  // left, right
    XrSpace gaze_space_ = XR_NULL_HANDLE, aim_space_[2] = {};
    uint64_t gaze_frames_ = 0, gaze_tracked_ = 0, gaze_stat_ns_ = 0;
    int hand_ = 1;         // controller whose grip was clicked last
    int pointer_ = -2;     // what pointed last frame: 0 left, 1 right, -1 gaze (logged on change)
    void init_input();
    bool locate_ray(XrSpace space, XrTime t, XrVector3f& origin, XrVector3f& dir);
    bool locate_gaze(XrTime t, XrVector3f& origin, XrVector3f& dir);

    // 9-dot gaze calibration (--gaze-cal)
    struct CalSample { float t, off[2]; XrVector3f origin; };  // s into the dot, gaze - target (deg), gaze origin in VIEW
    int cal_step_ = -1;                     // -1: intro, 0..8: showing dot kCalOrder[step], 9: done
    uint64_t cal_t0_ = 0;                   // start of the current step
    XrPosef cal_frame_{{0, 0, 0, 1}, {0, 0, 0}};  // dots are world-fixed relative to the head pose when they started
    std::vector<CalSample> cal_samples_[9];
    float cal_off_[9][2];                   // per-dot median offset (deg), NAN without data
    char cal_result_[96] = "";
    FILE* cal_csv_ = nullptr;
    void calibrate(XrTime t, bool tracked, XrVector3f gaze_origin, XrVector3f gaze_dir, std::vector<Vertex>& verts);
    void cal_report();
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

// Gaze calibration grid: 3x3 dots kCalDeg apart, center first then clockwise from top-left. Each dot shows for
// kCalDotS; samples between kCalWin0 and kCalWin1 into it (after the saccade, before the next one) count.
static const float kCalDeg = 15, kCalIntroS = 4, kCalDotS = 2.5f, kCalWin0 = 0.8f, kCalWin1 = 2.3f;
static const int kCalOrder[9] = {4, 0, 1, 2, 5, 8, 7, 6, 3};
static void cal_target(int k, float out[2]) { out[0] = (k % 3 - 1) * kCalDeg; out[1] = (1 - k / 3) * kCalDeg; }

// Direction v (app space) as angles in degrees in the head frame: x = atan(right/forward), y = atan(up/forward).
static void head_angles(XrQuaternionf head, XrVector3f v, float out[2]) {
    XrQuaternionf inv{-head.x, -head.y, -head.z, head.w};
    XrVector3f h;
    XrQuaternionf_RotateVector3f(&h, &inv, &v);
    out[0] = std::atan2(h.x, -h.z) * 180 / float(M_PI);
    out[1] = std::atan2(h.y, -h.z) * 180 / float(M_PI);
}

// Point at head-frame angles (ax, ay) degrees on the plane `dist` ahead of the pose f.
static XrVector3f cal_point(XrPosef f, float ax, float ay, float dist) {
    XrVector3f p{std::tan(ax * float(M_PI) / 180) * dist, std::tan(ay * float(M_PI) / 180) * dist, -dist}, r;
    XrQuaternionf_RotateVector3f(&r, &f.orientation, &p);
    return add(f.position, r);
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
    // Gaze calibration: dots placed from a tilted head read back at their grid angles; a gaze 1 deg to the
    // right of the center dot reads +1 deg x.
    XrVector3f ax = norm({1, 2, 3});
    float s = std::sin(0.35f);
    XrPosef head{{ax.x * s, ax.y * s, ax.z * s, std::cos(0.35f)}, {0.1f, 1.6f, -0.3f}};
    float worst = 0, a[2], tgt[2];
    for (int k = 0; k < 9; k++) {
        cal_target(k, tgt);
        head_angles(head.orientation, sub(cal_point(head, tgt[0], tgt[1], 1.5f), head.position), a);
        worst = std::max({worst, std::fabs(a[0] - tgt[0]), std::fabs(a[1] - tgt[1])});
    }
    XrVector3f right1{std::sin(float(M_PI) / 180), 0, -std::cos(float(M_PI) / 180)}, g;
    XrQuaternionf_RotateVector3f(&g, &head.orientation, &right1);
    head_angles(head.orientation, g, a);
    ok = worst < 1e-3f && std::fabs(a[0] - 1) < 1e-3f && std::fabs(a[1]) < 1e-3f;
    printf("%s %-32s worst %.5f, right (%.4f %.4f)\n", ok ? "ok  " : "FAIL", "gaze-cal grid angles", worst, a[0], a[1]);
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
    init_input();
}

void App::init_input() {
    XrActionSetCreateInfo asi{XR_TYPE_ACTION_SET_CREATE_INFO};
    strcpy(asi.actionSetName, "input");
    strcpy(asi.localizedActionSetName, "Input");
    XR_CHECK(xrCreateActionSet(instance_, &asi, &input_set_));
    auto path = [&](const char* s) {
        XrPath p;
        XR_CHECK(xrStringToPath(instance_, s, &p));
        return p;
    };
    auto suggest = [&](const char* profile, std::vector<XrActionSuggestedBinding> b) {
        XrInteractionProfileSuggestedBinding sb{XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING};
        sb.interactionProfile = path(profile);
        sb.countSuggestedBindings = uint32_t(b.size());
        sb.suggestedBindings = b.data();
        return xrSuggestInteractionProfileBindings(instance_, &sb);
    };
    XrActionCreateInfo ai{XR_TYPE_ACTION_CREATE_INFO};
    ai.actionType = XR_ACTION_TYPE_POSE_INPUT;
    if (gaze_supported_) {
        strcpy(ai.actionName, "gaze_pose");
        strcpy(ai.localizedActionName, "Gaze pose");
        XR_CHECK(xrCreateAction(input_set_, &ai, &gaze_action_));
        XR_CHECK(suggest("/interaction_profiles/ext/eye_gaze_interaction",
                         {{gaze_action_, path("/user/eyes_ext/input/gaze_ext/pose")}}));
    }
    XrPath* hands = hand_path_;
    hands[0] = path("/user/hand/left");
    hands[1] = path("/user/hand/right");
    strcpy(ai.actionName, "aim_pose");
    strcpy(ai.localizedActionName, "Pointer");
    ai.countSubactionPaths = 2;
    ai.subactionPaths = hands;
    XR_CHECK(xrCreateAction(input_set_, &ai, &aim_action_));
    ai.actionType = XR_ACTION_TYPE_BOOLEAN_INPUT;
    strcpy(ai.actionName, "use_hand");
    strcpy(ai.localizedActionName, "Point with this hand");
    XR_CHECK(xrCreateAction(input_set_, &ai, &grip_action_));
    std::vector<XrActionSuggestedBinding> aim = {{aim_action_, path("/user/hand/left/input/aim/pose")},
                                                 {aim_action_, path("/user/hand/right/input/aim/pose")}};
    XR_CHECK(suggest("/interaction_profiles/khr/simple_controller", aim));
    // SteamVR remaps Frame controllers; index squeeze = Frame grip (the side trigger)
    aim.push_back({grip_action_, path("/user/hand/left/input/squeeze/value")});
    aim.push_back({grip_action_, path("/user/hand/right/input/squeeze/value")});
    XR_CHECK(suggest("/interaction_profiles/valve/index_controller", aim));

    XrSessionActionSetsAttachInfo at{XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO};
    at.countActionSets = 1;
    at.actionSets = &input_set_;
    XR_CHECK(xrAttachSessionActionSets(session_, &at));

    XrActionSpaceCreateInfo sci{XR_TYPE_ACTION_SPACE_CREATE_INFO};
    sci.poseInActionSpace.orientation.w = 1;
    if (gaze_action_) {
        sci.action = gaze_action_;
        XR_CHECK(xrCreateActionSpace(session_, &sci, &gaze_space_));
    }
    sci.action = aim_action_;
    for (int h = 0; h < 2; h++) {
        sci.subactionPath = hands[h];
        XR_CHECK(xrCreateActionSpace(session_, &sci, &aim_space_[h]));
    }
}

// Pose's -Z ray in app space at time t. False if the space isn't currently tracked.
bool App::locate_ray(XrSpace space, XrTime t, XrVector3f& origin, XrVector3f& dir) {
    XrSpaceLocation loc{XR_TYPE_SPACE_LOCATION};
    if (XR_FAILED(xrLocateSpace(space, app_space_, t, &loc))) return false;
    const XrSpaceLocationFlags need = XR_SPACE_LOCATION_ORIENTATION_VALID_BIT | XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT |
                                      XR_SPACE_LOCATION_POSITION_VALID_BIT;
    if ((loc.locationFlags & need) != need) return false;
    XrVector3f minus_z{0, 0, -1};
    XrQuaternionf_RotateVector3f(&dir, &loc.pose.orientation, &minus_z);
    origin = loc.pose.position;
    return true;
}

// Eye gaze ray in app space at time t. False when eye tracking isn't available or not currently tracked.
bool App::locate_gaze(XrTime t, XrVector3f& origin, XrVector3f& dir) {
    if (!gaze_space_) return false;
    XrVector3f o;
    bool ok = locate_ray(gaze_space_, t, o, dir);

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
    if (ok) origin = o;
    return ok;
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

static const char* kCalCsv = "/tmp/gaze_cal.csv";

// --gaze-cal: show the dots one at a time with a countdown ring, record gaze - target, then show the result.
// Offsets are measured in the head frame from the gaze origin, against world-fixed dots, so head motion during
// a dot doesn't count as gaze error (but gaze/head timing mismatches do, as they would for the boxes).
void App::calibrate(XrTime t, bool tracked, XrVector3f go, XrVector3f gd, std::vector<Vertex>& verts) {
    const float d = g_opt.gaze_cal_m, deg = d * float(M_PI) / 180;  // ~1 degree in meters at the dots
    XrSpaceLocation head{XR_TYPE_SPACE_LOCATION};
    XR_CHECK(xrLocateSpace(view_space_, app_space_, t, &head));
    bool head_ok = head.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT;
    tracked = tracked && head_ok;
    uint64_t now = now_ns();
    if (!cal_t0_) cal_t0_ = now;
    float el = (now - cal_t0_) / 1e9f;
    if (cal_step_ < 0) {
        // Head-locked instructions until gaze has been tracked for kCalIntroS straight (headset on, session focused).
        if (head_ok) cal_frame_ = head.pose;
        if (!tracked) cal_t0_ = now;
        else if (el > kCalIntroS) {
            cal_step_ = 0;
            cal_t0_ = now;
            el = 0;
            if ((cal_csv_ = fopen(kCalCsv, "w")))
                fprintf(cal_csv_, "dot,t,target_x,target_y,gaze_x,gaze_y,origin_x,origin_y,origin_z\n");
        }
    } else if (cal_step_ < 9 && el > kCalDotS) {
        if (++cal_step_ == 9) cal_report();
        cal_t0_ = now;
        el = 0;
    }

    XrVector3f x, y, ux{1, 0, 0}, uy{0, 1, 0};
    XrQuaternionf_RotateVector3f(&x, &cal_frame_.orientation, &ux);
    XrQuaternionf_RotateVector3f(&y, &cal_frame_.orientation, &uy);
    const float white[4] = {1, 1, 1, 1}, dim[4] = {1, 1, 1, 0.25f}, cyan[4] = {0.3f, 0.9f, 1, 1},
                orange[4] = {1, 0.6f, 0.1f, 1}, green[4] = {0.3f, 1, 0.3f, 0.8f};
    if (cal_step_ < 0) {
        LabelLayout a = layout_text("Gaze check: look at each dot until its ring runs out", cal_point(cal_frame_, 0, 2, d),
                                    x, y, 1.3f * deg);
        LabelLayout b = layout_text(tracked ? "Keep your head still" : "Waiting for eye tracking...",
                                    cal_point(cal_frame_, 0, -2, d), x, y, 1.3f * deg);
        push_label(verts, a, white, 1);
        push_label(verts, b, white, 1);
        return;
    }
    if (cal_step_ < 9) {
        int k = kCalOrder[cal_step_];
        float tgt[2], g[2], a[2];
        cal_target(k, tgt);
        XrVector3f p = cal_point(cal_frame_, tgt[0], tgt[1], d);
        push_ring(verts, p, x, y, 0, 0.3f * deg, 1, white);
        push_ring(verts, p, x, y, 1.1f * deg, 1.4f * deg, 1, dim);
        push_ring(verts, p, x, y, 1.1f * deg, 1.4f * deg, 1 - el / kCalDotS, cyan);
        if (!tracked) return;
        head_angles(head.pose.orientation, gd, g);
        head_angles(head.pose.orientation, sub(p, go), a);
        XrQuaternionf inv{-head.pose.orientation.x, -head.pose.orientation.y, -head.pose.orientation.z, head.pose.orientation.w};
        XrVector3f rel = sub(go, head.pose.position), o;
        XrQuaternionf_RotateVector3f(&o, &inv, &rel);
        cal_samples_[k].push_back({el, {g[0] - a[0], g[1] - a[1]}, o});
        if (cal_csv_)
            fprintf(cal_csv_, "%d,%.4f,%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f\n", k, el, a[0], a[1], g[0], g[1], o.x, o.y, o.z);
        return;
    }
    // Done: each dot with a line to where the gaze actually landed, the summary, and a live gaze cursor.
    for (int k = 0; k < 9; k++) {
        float tgt[2];
        cal_target(k, tgt);
        XrVector3f p = cal_point(cal_frame_, tgt[0], tgt[1], d);
        push_ring(verts, p, x, y, 0, 0.3f * deg, 1, white);
        if (std::isnan(cal_off_[k][0])) continue;
        XrVector3f q = cal_point(cal_frame_, tgt[0] + cal_off_[k][0], tgt[1] + cal_off_[k][1], d);
        push_line(verts, p, q, head.pose.position, 0.1f * deg, orange);
        push_ring(verts, q, x, y, 0, 0.25f * deg, 1, orange);
    }
    LabelLayout r = layout_text(cal_result_, cal_point(cal_frame_, 0, -kCalDeg - 4, d), x, y, 1.3f * deg);
    push_label(verts, r, white, 1);
    XrVector3f n = cross(x, y), c = cal_point(cal_frame_, 0, 0, d);  // n faces the viewer
    float den = dot(gd, n);
    if (tracked && den < -1e-3f) push_ring(verts, add(go, mul(gd, dot(sub(c, go), n) / den)), x, y, 0, 0.2f * deg, 1, green);
}

static float median(std::vector<float> v) {
    std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
    return v[v.size() / 2];
}

// Per dot: median offset over the sample window (robust to blinks and stray saccades) and spread = median
// distance of the samples from it. Then the mean offset over all dots (a constant bias a fixed correction
// would remove) and the mean error with and without it.
void App::cal_report() {
    if (cal_csv_) fclose(cal_csv_);
    cal_csv_ = nullptr;
    printf("gaze-cal: dots %.2f m away; offset = gaze - target in degrees, +x right, +y up\n", g_opt.gaze_cal_m);
    printf("gaze-cal: dot  target      offset          |off|  spread     n\n");
    std::vector<float> ox, oy, oz;
    float bias[2] = {}, err = 0, resid = 0;
    int nv = 0;
    for (int k = 0; k < 9; k++) {
        float tgt[2];
        cal_target(k, tgt);
        std::vector<float> x, y, dist;
        for (const CalSample& s : cal_samples_[k])
            if (s.t >= kCalWin0 && s.t <= kCalWin1) {
                x.push_back(s.off[0]);
                y.push_back(s.off[1]);
                ox.push_back(s.origin.x);
                oy.push_back(s.origin.y);
                oz.push_back(s.origin.z);
            }
        cal_off_[k][0] = cal_off_[k][1] = NAN;
        if (x.size() < 10) {
            printf("gaze-cal: %3d  (%+3.0f %+3.0f)  no gaze (%zu samples)\n", k, tgt[0], tgt[1], x.size());
            continue;
        }
        float mx = median(x), my = median(y);
        for (size_t i = 0; i < x.size(); i++) dist.push_back(std::hypot(x[i] - mx, y[i] - my));
        cal_off_[k][0] = mx;
        cal_off_[k][1] = my;
        printf("gaze-cal: %3d  (%+3.0f %+3.0f)  (%+5.2f %+5.2f)  %5.2f  %5.2f  %5zu\n", k, tgt[0], tgt[1], mx, my,
               std::hypot(mx, my), median(dist), x.size());
        bias[0] += mx;
        bias[1] += my;
        err += std::hypot(mx, my);
        nv++;
    }
    if (!nv) {
        snprintf(cal_result_, sizeof cal_result_, "No gaze data");
        printf("gaze-cal: no gaze data\n");
        return;
    }
    bias[0] /= nv;
    bias[1] /= nv;
    err /= nv;
    for (int k = 0; k < 9; k++)
        if (!std::isnan(cal_off_[k][0])) resid += std::hypot(cal_off_[k][0] - bias[0], cal_off_[k][1] - bias[1]) / nv;
    printf("gaze-cal: mean offset (%+.2f %+.2f) deg, mean error %.2f deg, %.2f deg after removing the mean offset\n",
           bias[0], bias[1], err, resid);
    printf("gaze-cal: gaze origin in VIEW (%.1f %.1f %.1f) mm; samples in %s\n", median(ox) * 1000, median(oy) * 1000,
           median(oz) * 1000, kCalCsv);
    snprintf(cal_result_, sizeof cal_result_, "Mean offset %.1f %s %.1f %s, error %.1f deg (%.1f without the offset)",
             std::fabs(bias[0]), bias[0] >= 0 ? "right" : "left", std::fabs(bias[1]), bias[1] >= 0 ? "up" : "down", err, resid);
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
        // Only show the object you're pointing at: controller aim rays if any controller is tracked, else eye
        // gaze if tracked, else head forward. Per-track easing keeps saccades from blinking boxes.
        XrActiveActionSet active{input_set_, XR_NULL_PATH};
        XrActionsSyncInfo sync{XR_TYPE_ACTIONS_SYNC_INFO};
        sync.countActiveActionSets = 1;
        sync.activeActionSets = &active;
        bool synced = XR_SUCCEEDED(xrSyncActions(session_, &sync));
        XrVector3f gaze_origin = eye, gaze_dir, minus_z{0, 0, -1};
        bool gaze_tracked = synced && locate_gaze(fs.predictedDisplayTime, gaze_origin, gaze_dir);
        if (!gaze_tracked) XrQuaternionf_RotateVector3f(&gaze_dir, &views[0].pose.orientation, &minus_z);
        for (int h = 0; h < 2 && synced; h++) {
            XrActionStateGetInfo gi{XR_TYPE_ACTION_STATE_GET_INFO, nullptr, grip_action_, hand_path_[h]};
            XrActionStateBoolean st{XR_TYPE_ACTION_STATE_BOOLEAN};
            if (XR_SUCCEEDED(xrGetActionStateBoolean(session_, &gi, &st)) && st.currentState && st.changedSinceLastSync)
                hand_ = h;
        }
        XrVector3f ray_o = gaze_origin, ray_d = gaze_dir;
        int pointer = -1;
        for (int h : {hand_, 1 - hand_})
            if (synced && locate_ray(aim_space_[h], fs.predictedDisplayTime, ray_o, ray_d)) {
                pointer = h;
                break;
            }
        if (pointer != pointer_) printf("xr: pointing with %s\n", pointer < 0 ? "gaze" : pointer ? "right controller" : "left controller");
        pointer_ = pointer;
        float k_gaze = 1 - std::exp(-std::min(frame_dt_, 0.1f) / 0.15f);
        std::vector<Vertex> verts;
        // Pick the one confirmed track a ray hits best. Hit target: the box grown on every side by
        // --focus-margin label heights (scales with distance like the label), scored 0 at the center to 1 at
        // the grown edge, so a small object inside a big one wins when you point at it. The current pick
        // gets a bonus (hysteresis) and its label counts as a center hit, so reading it keeps it up.
        uint64_t best_id = 0;
        float best = 1;
        for (const Track& tr : tracks_) {
            if (tr.alpha < 0.01f) continue;
            LabelLayout label = layout_label(tr.shown, tr.cls, tr.score, eye);
            float s = gaze_rect_score(tr.shown, ray_o, ray_d, g_opt.focus_margin * label.height());
            if (tr.id == selected_id_) {
                XrVector3f rect[4];
                label.rect(rect);
                if (gaze_rect_score(rect, ray_o, ray_d, 0) <= 1) s = 0;
                s *= 0.8f;
            }
            if (s <= best) { best = s; best_id = tr.id; }
        }
        selected_id_ = best_id;
        // Laser (green, unlike SteamVR's): ends at the picked box's depth along the ray, else 3 m out.
        if (pointer >= 0) {
            float d = 3;
            for (const Track& tr : tracks_)
                if (tr.id == selected_id_) d = std::max(0.1f, dot(sub(center(tr.shown), ray_o), ray_d));
            const float rgba[4] = {0.2f, 1, 0.3f, 0.7f};
            push_line(verts, ray_o, add(ray_o, mul(ray_d, d)), eye, 0.002f, rgba);
        }
        for (Track& tr : tracks_) {
            tr.focus_alpha += ((tr.id == selected_id_ ? 1.0f : 0.0f) - tr.focus_alpha) * k_gaze;
            float a = tr.alpha * tr.focus_alpha;
            if (a > 0.01f) push_box(verts, tr.shown, tr.cls, tr.score, a, eye);
        }
        if (g_opt.gaze_cal_m > 0) calibrate(fs.predictedDisplayTime, gaze_tracked, gaze_origin, gaze_dir, verts);
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
        else if (parse_detector_option(a, next)) continue;
        else if (a == "--smooth-ms") g_opt.smooth_ms = std::max(1.0f, float(atof(next())));
        else if (a == "--confirm-hits") g_opt.confirm_hits = atoi(next());
        else if (a == "--hold-ms") g_opt.hold_ms = float(atof(next()));
        else if (a == "--focus-margin") g_opt.focus_margin = float(atof(next()));
        else if (a == "--no-eye-gaze") g_opt.eye_gaze = false;
        else if (a == "--gaze-cal") g_opt.gaze_cal_m = float(atof(next()));
        else { fprintf(stderr, "unknown option %s\n", a.c_str()); return 2; }
    }
    if (!finish_detector_options()) return 2;
    setvbuf(stdout, nullptr, _IOLBF, 0);
    signal(SIGINT, [](int) { g_quit = true; });
    signal(SIGTERM, [](int) { g_quit = true; });

    std::thread det;  // the calibration needs no detector (no tracks -> no boxes)
    if (g_opt.gaze_cal_m <= 0)
        det = std::thread([] {
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
    if (det.joinable()) det.join();
    ncnn::destroy_gpu_instance();
    return rc;
}
