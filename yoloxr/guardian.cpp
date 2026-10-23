// guardian: outlines people, cats and dogs over whatever VR app is running (OpenVR overlays), so you see them
// without passthrough. Same detector as yoloxr (detector.h), at a low size and rate to leave the GPU to the game.
//
// Each confirmed track is a box overlay (rounded outline) plus a label overlay (class, confidence): upright sprites
// turned to face you every compositor frame, in the standing universe. Detections feed a ByteTrack-style tracker
// (tracker.h: Kalman filter per track), and each box is drawn where its track predicts the person is now, not where
// the cameras saw them a few hundred ms ago.
//
// A "Guardian" tab in the SteamVR dashboard toggles it and what it shows; settings persist in ~/.config/guardian.conf.
//
// Usage: guardian [--max-dist 3] [--size 320] [--max-hz 4] [--smooth-ms 80] [detector options, detector.h]
//        guardian --no-detector   (only log the VR app's frame stats, as a baseline)
//        guardian --register   (add guardian.vrmanifest to SteamVR, auto-launch with SteamVR, start it now)
//        guardian --selftest
#include <limits.h>
#include <signal.h>
#include <stdlib.h>
#include <ft2build.h>
#include FT_FREETYPE_H
#include <openvr.h>

#include <array>
#include <thread>
#include <unordered_map>

#include "detector.h"
#include "tracker.h"

static const char* kAppKey = "steamframe.guardian";
static const int kMaxOverlays = 8;
static float g_smooth_s = 0.08f;      // drawn boxes glide to the track estimate with this time constant
static bool g_detector = true;        // --no-detector: just log frame stats (baseline for the detector's cost)
static const float kLabelH = 0.07f;   // label height (m), just above the box
static const char* kFont = "/usr/share/fonts/noto/NotoSans-Medium.ttf";

// Set from the dashboard tab, saved to g_conf.
struct Settings {
    bool enabled = true, people = true, pets = true;  // pets: cats and dogs
    bool hands = true;                                // ignore boxes around a controller: your hands (holds_controller)
    float max_dist = 3;                               // ignore detections farther than this (m, horizontal)
};
static Settings g_set;
static std::string g_conf;  // empty: don't save (selftest)

static void load_settings() {
    FILE* f = fopen(g_conf.c_str(), "r");
    if (!f) return;
    char k[32];
    float v;
    while (fscanf(f, " %31[^=]=%f", k, &v) == 2) {
        std::string key = k;
        if (key == "enabled") g_set.enabled = v != 0;
        else if (key == "people") g_set.people = v != 0;
        else if (key == "pets") g_set.pets = v != 0;
        else if (key == "hands") g_set.hands = v != 0;
        else if (key == "max_dist") g_set.max_dist = v;
    }
    fclose(f);
}

static void save_settings() {
    FILE* f = g_conf.empty() ? nullptr : fopen(g_conf.c_str(), "w");
    if (!f) return;
    fprintf(f, "enabled=%d\npeople=%d\npets=%d\nhands=%d\nmax_dist=%.1f\n", g_set.enabled, g_set.people, g_set.pets,
            g_set.hands, g_set.max_dist);
    fclose(f);
}

static bool wanted(int cls) { return cls == 0 ? g_set.people : (cls == 15 || cls == 16) && g_set.pets; }  // person, cat, dog

// Your own hands (YOLO often calls them "person"): a controller inside the box, i.e. within its rectangle grown by
// 15 cm and within 40 cm of its depth. Box and controllers are in the head frame at capture time.
static bool holds_controller(const Obj& o, const std::vector<XrVector3f>& controllers) {
    const float m = 0.15f;
    for (const XrVector3f& p : controllers)
        if (p.x > o.corners[0].x - m && p.x < o.corners[1].x + m && p.y > o.corners[3].y - m &&
            p.y < o.corners[0].y + m && std::fabs(p.z - o.corners[0].z) < 0.4f)
            return true;
    return false;
}

static XrVector3f v_add(XrVector3f a, XrVector3f b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
static XrVector3f v_sub(XrVector3f a, XrVector3f b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
static XrVector3f v_mul(XrVector3f a, float s) { return {a.x * s, a.y * s, a.z * s}; }
static float v_len(XrVector3f a) { return std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z); }
static XrVector3f v_center(const XrVector3f* c) { return v_mul(v_add(v_add(c[0], c[1]), v_add(c[2], c[3])), 0.25f); }

static XrVector3f apply(const vr::HmdMatrix34_t& m, XrVector3f p) {
    return {m.m[0][0] * p.x + m.m[0][1] * p.y + m.m[0][2] * p.z + m.m[0][3],
            m.m[1][0] * p.x + m.m[1][1] * p.y + m.m[1][2] * p.z + m.m[1][3],
            m.m[2][0] * p.x + m.m[2][1] * p.y + m.m[2][2] * p.z + m.m[2][3]};
}

// Upright quad at c facing the head position h (overlay +z points at the viewer).
static vr::HmdMatrix34_t facing(XrVector3f c, XrVector3f h) {
    float zx = h.x - c.x, zz = h.z - c.z, n = std::hypot(zx, zz);
    if (n < 1e-4f) zx = 0, zz = 1, n = 1;
    zx /= n, zz /= n;
    return {{{zz, 0, zx, c.x}, {0, 1, 0, c.y}, {-zx, 0, zz, c.z}}};
}

static void class_rgb(int cls, uint8_t rgb[3]) {
    static const uint8_t person[3] = {255, 69, 58}, pet[3] = {255, 184, 0};
    memcpy(rgb, cls == 0 ? person : pet, 3);
}

// Straight-alpha "over" of color rgb at coverage a onto RGBA pixel p.
static void over(uint8_t* p, const uint8_t rgb[3], float a) {
    if (a <= 0) return;
    float da = p[3] / 255.f, oa = a + da * (1 - a);
    for (int c = 0; c < 3; c++) p[c] = uint8_t(std::lround((rgb[c] * a + p[c] * da * (1 - a)) / oa));
    p[3] = uint8_t(std::lround(oa * 255));
}

// Signed distance (px, < 0 inside) from (x, y) to a w x h rounded rect inset by `inset`, corner radius r.
static float rrect_sd(float x, float y, float w, float h, float inset, float r) {
    float qx = std::fabs(x - w / 2.f) - (w / 2.f - inset - r), qy = std::fabs(y - h / 2.f) - (h / 2.f - inset - r);
    return std::hypot(std::max(qx, 0.f), std::max(qy, 0.f)) + std::min(std::max(qx, qy), 0.f) - r;
}

// Anti-aliased rounded outline, `line` px wide (thinner and fainter along the sides, bold at the corners), over a
// barely-there fill. Sized to the box's aspect.
static std::vector<uint8_t> outline(int w, int h, int cls, float line) {
    uint8_t rgb[3];
    class_rgb(cls, rgb);
    std::vector<uint8_t> px(size_t(w) * h * 4);
    const float m = float(std::min(w, h)), r = m * 0.08f, corner = m * 0.2f;
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++) {
            float fx = x + 0.5f, fy = y + 0.5f, sd = rrect_sd(fx, fy, w, h, 1, r);
            bool c = std::fabs(fx - w / 2.f) > w / 2.f - corner && std::fabs(fy - h / 2.f) > h / 2.f - corner;
            float lw = c ? line * 2.5f : line;  // stroke lies just inside the edge: sd in [-lw, 0]
            float a = std::clamp(lw / 2 + 0.5f - std::fabs(sd + lw / 2), 0.f, 1.f) * (c ? 1 : 0.6f);
            over(&px[(size_t(y) * w + x) * 4], rgb, std::max(a, sd < 0 ? 0.06f : 0.f));
        }
    return px;
}

static FT_Face font() {
    static FT_Library lib;
    static FT_Face face;
    if (!face) {
        if (FT_Init_FreeType(&lib) || FT_New_Face(lib, kFont, 0, &face)) throw std::runtime_error(std::string("can't load ") + kFont);
    }
    return face;
}

// Draws (or with px == nullptr, measures) UTF-8 text `size` px tall with its baseline at y; returns the pen x after it.
static int draw_text(std::vector<uint8_t>* px, int w, int h, int x, int y, const char* s, const uint8_t rgb[3], int size) {
    FT_Face f = font();
    FT_Set_Pixel_Sizes(f, 0, size);
    for (; *s; s++) {
        uint32_t cp = (unsigned char)*s;
        if (cp >= 0xE0 && s[1] && s[2]) cp = (cp & 0x0F) << 12 | (s[1] & 0x3F) << 6 | (s[2] & 0x3F), s += 2;
        else if (cp >= 0xC0 && s[1]) cp = (cp & 0x1F) << 6 | (s[1] & 0x3F), s += 1;
        if (FT_Load_Char(f, cp, px ? FT_LOAD_RENDER : FT_LOAD_DEFAULT)) continue;
        const FT_GlyphSlot g = f->glyph;
        for (int r = 0; px && r < int(g->bitmap.rows); r++)
            for (int c = 0; c < int(g->bitmap.width); c++) {
                int X = x + g->bitmap_left + c, Y = y - g->bitmap_top + r;
                if (X >= 0 && Y >= 0 && X < w && Y < h)
                    over(&(*px)[(size_t(Y) * w + X) * 4], rgb, g->bitmap.buffer[r * g->bitmap.pitch + c] / 255.f);
            }
        x += int(g->advance.x >> 6);
    }
    return x;
}

// Dark translucent pill: class-colored dot, class name in white, confidence in the class color.
static std::vector<uint8_t> label(const char* name, int pct, int cls, int& w, int& h) {
    uint8_t rgb[3], white[3] = {245, 245, 247}, bg[3] = {16, 16, 20};
    class_rgb(cls, rgb);
    char num[16];
    snprintf(num, sizeof num, "%d%%", pct);
    h = 64;
    const int dot = 14, gap = 12, space = 12, pad = h / 2 - 6;
    int x_name = pad + dot + gap, x_num = draw_text(nullptr, 0, 0, x_name, 0, name, white, 40) + space;
    w = draw_text(nullptr, 0, 0, x_num, 0, num, rgb, 40) + pad;
    std::vector<uint8_t> px(size_t(w) * h * 4);
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++) {
            uint8_t* p = &px[(size_t(y) * w + x) * 4];
            over(p, bg, 0.72f * std::clamp(0.5f - rrect_sd(x + 0.5f, y + 0.5f, w, h, 0, h / 2.f), 0.f, 1.f));
            over(p, rgb, std::clamp(dot / 2.f + 0.5f - std::hypot(x + 0.5f - (pad + dot / 2.f), y + 0.5f - h / 2.f), 0.f, 1.f));
        }
    FT_Face f = font();
    int base = h / 2 + int((f->size->metrics.ascender + f->size->metrics.descender) >> 7);  // vertically centered
    draw_text(&px, w, h, x_name, base, name, white, 40);
    draw_text(&px, w, h, x_num, base, num, rgb, 40);
    return px;
}

// ---------------------------------------------------------------------------------------------
// Dashboard tab: header with a status line, then one row per setting. Drawn on the CPU like the labels; the
// dashboard's laser pointer arrives as mouse events in texture pixels.

namespace panel {

const int W = 1000, H = 680, kPad = 48, kTop = 190, kRowH = 92;
enum Ctl { kEnabled, kPeople, kPets, kHands, kRangeMinus, kRangePlus };
const char* kRows[] = {"Enabled", "People", "Cats and dogs", "Ignore my hands", "Range"};
const int kToggleW = 104, kToggleH = 56, kBtnR = 28;

static float row_cy(int row) { return kTop + (row + 0.5f) * kRowH; }
static float minus_cx() { return W - kPad - kBtnR - 220.f; }
static float plus_cx() { return W - kPad - kBtnR; }

// Which control (texture pixel, y down) is under the pointer: the whole row for toggles, the buttons for range.
static int hit(float x, float y) {
    int row = int(std::floor((y - kTop) / kRowH));
    if (x < kPad || x > W - kPad || row < 0 || row > 4) return -1;
    if (row < 4) return row;
    if (std::hypot(x - minus_cx(), y - row_cy(4)) < kBtnR + 12) return kRangeMinus;
    if (std::hypot(x - plus_cx(), y - row_cy(4)) < kBtnR + 12) return kRangePlus;
    return -1;
}

static void press(int ctl) {
    switch (ctl) {
        case kEnabled: g_set.enabled = !g_set.enabled; break;
        case kPeople: g_set.people = !g_set.people; break;
        case kPets: g_set.pets = !g_set.pets; break;
        case kHands: g_set.hands = !g_set.hands; break;
        case kRangeMinus: g_set.max_dist = std::max(1.0f, g_set.max_dist - 0.5f); break;
        case kRangePlus: g_set.max_dist = std::min(8.0f, g_set.max_dist + 0.5f); break;
        default: return;
    }
    save_settings();
}

static void fill_rrect(std::vector<uint8_t>& px, float x0, float y0, float w, float h, float r, const uint8_t rgb[3], float a) {
    for (int y = std::max(0, int(y0) - 1); y < std::min(H, int(y0 + h) + 2); y++)
        for (int x = std::max(0, int(x0) - 1); x < std::min(W, int(x0 + w) + 2); x++)
            over(&px[(size_t(y) * W + x) * 4], rgb, a * std::clamp(0.5f - rrect_sd(x + 0.5f - x0, y + 0.5f - y0, w, h, 0, r), 0.f, 1.f));
}

static void fill_circle(std::vector<uint8_t>& px, float cx, float cy, float r, const uint8_t rgb[3], float a) {
    fill_rrect(px, cx - r, cy - r, 2 * r, 2 * r, r, rgb, a);
}

static std::vector<uint8_t> render(int hover, const std::string& status) {
    static const uint8_t bg[3] = {20, 20, 24}, white[3] = {245, 245, 247}, grey[3] = {150, 150, 160},
                         off[3] = {68, 68, 76}, btn[3] = {52, 52, 60};
    uint8_t accent[3];
    class_rgb(0, accent);
    std::vector<uint8_t> px(size_t(W) * H * 4);
    fill_rrect(px, 0, 0, W, H, 32, bg, 0.94f);
    draw_text(&px, W, H, kPad, 100, "Guardian", white, 56);
    draw_text(&px, W, H, kPad, 150, status.c_str(), grey, 30);
    bool on[4] = {g_set.enabled, g_set.people, g_set.pets, g_set.hands};
    for (int row = 0; row < 5; row++) {
        float cy = row_cy(row);
        if (hover == row) fill_rrect(px, kPad - 20, cy - kRowH / 2.f + 4, W - 2 * kPad + 40, kRowH - 8, 18, white, 0.05f);
        if (row) fill_rrect(px, kPad, cy - kRowH / 2.f, W - 2 * kPad, 1.5f, 0, white, 0.08f);
        bool dim = row > 0 && !g_set.enabled;  // the other settings don't matter while it's off
        draw_text(&px, W, H, kPad, int(cy + 13), kRows[row], dim ? grey : white, 36);
        if (row < 4) {  // switch: accent track with the knob right when on
            float x0 = W - kPad - kToggleW, y0 = cy - kToggleH / 2.f;
            fill_rrect(px, x0, y0, kToggleW, kToggleH, kToggleH / 2.f, on[row] ? accent : off, dim ? 0.5f : 1);
            fill_circle(px, on[row] ? x0 + kToggleW - kToggleH / 2.f : x0 + kToggleH / 2.f, cy, kToggleH / 2.f - 5, white, dim ? 0.6f : 1);
            continue;
        }
        for (int b = 0; b < 2; b++) {  // range: (-) value (+)
            float cx = b ? plus_cx() : minus_cx();
            fill_circle(px, cx, cy, kBtnR, btn, hover == kRangeMinus + b ? 1 : 0.8f);
            fill_rrect(px, cx - 11, cy - 1.5f, 22, 3, 1.5f, white, 1);
            if (b) fill_rrect(px, cx - 1.5f, cy - 11, 3, 22, 1.5f, white, 1);
        }
        char v[16];
        snprintf(v, sizeof v, "%.1f m", g_set.max_dist);
        int tw = draw_text(nullptr, 0, 0, 0, 0, v, white, 36);
        draw_text(&px, W, H, int((minus_cx() + plus_cx()) / 2 - tw / 2.f), int(cy + 13), v, dim ? grey : white, 36);
    }
    return px;
}

// Tab icon: the box's corner brackets on a dark tile.
static std::vector<uint8_t> icon(int& size) {
    size = 128;
    std::vector<uint8_t> px(size_t(size) * size * 4), box = outline(80, 96, 0, 5);
    static const uint8_t bg[3] = {20, 20, 24};
    for (int y = 0; y < size; y++)
        for (int x = 0; x < size; x++) {
            uint8_t* p = &px[(size_t(y) * size + x) * 4];
            over(p, bg, std::clamp(0.5f - rrect_sd(x + 0.5f, y + 0.5f, size, size, 0, 24), 0.f, 1.f));
            int bx = x - 24, by = y - 16;
            if (bx >= 0 && by >= 0 && bx < 80 && by < 96) over(p, &box[(size_t(by) * 80 + bx) * 4], box[(size_t(by) * 80 + bx) * 4 + 3] / 255.f);
        }
    return px;
}

}  // namespace panel

static vr::HmdMatrix34_t head_pose(float seconds_from_now, bool& ok) {
    vr::TrackedDevicePose_t p;
    vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, seconds_from_now, &p, 1);
    ok = p.bPoseIsValid;
    return p.mDeviceToAbsoluteTracking;
}

// Positions (standing universe) of the tracked controllers, seconds_from_now (< 0: in the past).
static std::vector<XrVector3f> controllers(float seconds_from_now) {
    vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount];
    vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, seconds_from_now, poses,
                                                    vr::k_unMaxTrackedDeviceCount);
    std::vector<XrVector3f> out;
    for (uint32_t i = 0; i < vr::k_unMaxTrackedDeviceCount; i++)
        if (poses[i].bPoseIsValid && vr::VRSystem()->GetTrackedDeviceClass(i) == vr::TrackedDeviceClass_Controller) {
            const vr::HmdMatrix34_t& m = poses[i].mDeviceToAbsoluteTracking;
            out.push_back({m.m[0][3], m.m[1][3], m.m[2][3]});
        }
    return out;
}

// Detections this frame that guardian cares about, in the standing universe (head pose at capture): stereo (has
// depth), a wanted class, not around a controller (`ctrl`, standing universe, at capture), within range.
static std::vector<track::Meas> measurements(const std::vector<Obj>& objs, const vr::HmdMatrix34_t& head,
                                             const std::vector<XrVector3f>& ctrl) {
    XrVector3f h{head.m[0][3], head.m[1][3], head.m[2][3]};
    std::vector<XrVector3f> ctrl_head;  // head_from_world = R^T (p - t)
    for (XrVector3f p : g_set.hands ? ctrl : std::vector<XrVector3f>{}) {
        XrVector3f d = v_sub(p, h);
        ctrl_head.push_back({head.m[0][0] * d.x + head.m[1][0] * d.y + head.m[2][0] * d.z,
                             head.m[0][1] * d.x + head.m[1][1] * d.y + head.m[2][1] * d.z,
                             head.m[0][2] * d.x + head.m[1][2] * d.y + head.m[2][2] * d.z});
    }
    std::vector<track::Meas> ms;
    for (const Obj& o : objs) {
        if (!o.stereo || !wanted(o.cls) || holds_controller(o, ctrl_head)) continue;
        XrVector3f w[4];
        for (int k = 0; k < 4; k++) w[k] = apply(head, o.corners[k]);
        XrVector3f c = v_center(w);
        if (std::hypot(c.x - h.x, c.z - h.z) > g_set.max_dist) continue;
        // ponytail: isotropic position noise growing like stereo depth error (~Z^2); per-axis noise along the view
        // ray would track lateral motion tighter
        float sigma = 0.05f + 0.02f * o.dist * o.dist;
        ms.push_back({{c.x, c.y, c.z, v_len(v_sub(w[1], w[0])), v_len(v_sub(w[3], w[0]))}, sigma, o.score, o.cls});
    }
    return ms;
}

// Every 5 s: the running VR app's presented/dropped/reprojected frames since the last log and its GPU frame time.
static void log_frame_stats() {
    static vr::Compositor_CumulativeStats prev{};
    static uint64_t last = 0;
    uint64_t now = now_ns();
    if (now - last < 5000000000ull || !vr::VRCompositor()) return;
    last = now;
    vr::Compositor_CumulativeStats st{};
    vr::VRCompositor()->GetCumulativeStats(&st, sizeof st);
    vr::Compositor_FrameTiming ft[90];
    ft[0].m_nSize = sizeof ft[0];
    uint32_t n = vr::VRCompositor()->GetFrameTimings(ft, 90);
    float sum = 0, mx = 0;
    for (uint32_t i = 0; i < n; i++) sum += ft[i].m_flTotalRenderGpuMs, mx = std::max(mx, ft[i].m_flTotalRenderGpuMs);
    if (st.m_nPid == prev.m_nPid)
        printf("frames: %u presented, %u dropped, %u reprojected | gpu %.1f ms avg, %.1f max (last %u)\n",
               st.m_nNumFramePresents - prev.m_nNumFramePresents, st.m_nNumDroppedFrames - prev.m_nNumDroppedFrames,
               st.m_nNumReprojectedFrames - prev.m_nNumReprojectedFrames, n ? sum / n : 0.f, mx, n);
    prev = st;
}

static int register_app() {
    // VR_Init would start SteamVR outside its systemd service, which then can't come up until that one is killed.
    if (system("pgrep -x vrserver > /dev/null")) { fprintf(stderr, "SteamVR isn't running\n"); return 1; }
    vr::EVRInitError err;
    vr::VR_Init(&err, vr::VRApplication_Utility);
    if (err) { fprintf(stderr, "VR_Init: %s\n", vr::VR_GetVRInitErrorAsEnglishDescription(err)); return 1; }
    char path[PATH_MAX];
    if (!realpath("guardian.vrmanifest", path)) { perror("guardian.vrmanifest"); return 1; }
    auto* apps = vr::VRApplications();
    vr::EVRApplicationError e = apps->AddApplicationManifest(path, false);
    printf("AddApplicationManifest(%s): %s\n", path, apps->GetApplicationsErrorNameFromEnum(e));
    e = apps->SetApplicationAutoLaunch(kAppKey, true);
    printf("SetApplicationAutoLaunch: %s\n", apps->GetApplicationsErrorNameFromEnum(e));
    if (!apps->GetApplicationProcessId(kAppKey)) {
        e = apps->LaunchApplication(kAppKey);
        printf("LaunchApplication: %s\n", apps->GetApplicationsErrorNameFromEnum(e));
    }
    vr::VR_Shutdown();
    return e ? 1 : 0;
}

static int selftest() {
    int fails = 0;
    auto check = [&](bool ok, const char* what) { printf("%s %s\n", ok ? "ok  " : "FAIL", what); fails += !ok; };
    // Box 2 m ahead of an identity head: a quad facing the head has +z toward it, and x right.
    vr::HmdMatrix34_t m = facing({0, 1, -2}, {0, 1.6f, 0});
    check(std::fabs(m.m[2][2] - 1) < 1e-5f && std::fabs(m.m[0][0] - 1) < 1e-5f, "quad ahead faces the head");
    m = facing({2, 1, 0}, {0, 1.6f, 0});  // box to the right: +z points to -x
    check(std::fabs(m.m[0][2] + 1) < 1e-5f && std::fabs(m.m[1][1] - 1) < 1e-5f, "quad to the right faces the head");

    fails += track::selftest([](bool ok, const char* what) { printf("%s %s\n", ok ? "ok  " : "FAIL", what); });

    // Label: pill sized to the text (transparent outside its rounded ends), white text, colored dot.
    int lw, lh;
    std::vector<uint8_t> px = label("person", 87, 0, lw, lh);
    auto at = [&](const std::vector<uint8_t>& v, int w, int x, int y) { return &v[(size_t(y) * w + x) * 4]; };
    size_t white = 0;
    for (size_t i = 0; i < px.size(); i += 4) white += px[i] > 230 && px[i + 1] > 230 && px[i + 3] > 230;
    check(lh == 64 && lw > 3 * lh && at(px, lw, 0, 0)[3] == 0 && at(px, lw, lw / 2, 1)[3] > 150 && white > 200 &&
              at(px, lw, lh / 2 - 6 + 7, lh / 2)[1] == 69,
          "label texture");
    // Box: bold opaque corners, fainter sides, nearly clear middle, nothing outside the rounded corner.
    std::vector<uint8_t> bx = outline(256, 512, 0, 4);
    check(at(bx, 256, 0, 0)[3] == 0 && at(bx, 256, 3, 40)[3] > 230 && at(bx, 256, 3, 256)[3] > 100 &&
              at(bx, 256, 3, 256)[3] < 180 && at(bx, 256, 128, 256)[3] < 20,
          "box texture");

    // Dashboard: rows and buttons hit-test to their controls, pressing them changes the settings.
    check(panel::hit(500, panel::row_cy(panel::kPeople)) == panel::kPeople &&
              panel::hit(panel::plus_cx(), panel::row_cy(4)) == panel::kRangePlus && panel::hit(500, 50) == -1,
          "dashboard hit test");
    panel::press(panel::kPeople);
    for (int i = 0; i < 20; i++) panel::press(panel::kRangePlus);
    bool pressed = !g_set.people && g_set.max_dist == 8 && !wanted(0) && wanted(16);
    panel::press(panel::kPeople);
    g_set.max_dist = 3;
    std::vector<uint8_t> pn = panel::render(panel::kPets, "Watching");
    check(pressed && g_set.people && at(pn, panel::W, 0, 0)[3] == 0 && at(pn, panel::W, 500, 5)[3] > 200,
          "dashboard press and render");

    // Measurements: a person 2 m ahead is kept, in the standing universe; a far one, a cup and a hand holding a
    // controller are not. A controller between you and the person doesn't hide them (different depth).
    vr::HmdMatrix34_t head = {{{1, 0, 0, 0}, {0, 1, 0, 1.6f}, {0, 0, 1, 0}}};
    auto obj = [](int cls, float z, float x) {
        Obj o{cls, 0.9f, z, {{x - 0.3f, 0.5f, -z}, {x + 0.3f, 0.5f, -z}, {x + 0.3f, -1, -z}, {x - 0.3f, -1, -z}}, true, {}};
        return o;
    };
    Obj hand{0, 0.8f, 0.5f, {{-0.1f, -0.25f, -0.5f}, {0.1f, -0.25f, -0.5f}, {0.1f, -0.45f, -0.5f}, {-0.1f, -0.45f, -0.5f}}, true, {}};
    std::vector<XrVector3f> ctrl = {{0, 1.25f, -0.5f}, {0.2f, 1.3f, -0.6f}};  // one in the hand box, one before the person
    std::vector<track::Meas> ms = measurements({hand, obj(0, 2, 0.2f), obj(0, 5, 0), obj(41, 1, 0)}, head, ctrl);
    bool hand_kept_without = measurements({hand}, head, {}).size() == 1;
    g_set.hands = false;
    bool hand_kept_when_off = measurements({hand}, head, ctrl).size() == 1;
    g_set.hands = true;
    check(ms.size() == 1 && hand_kept_without && hand_kept_when_off,
          "box around a controller, far person and cup dropped; person behind a controller kept");
    check(ms.size() == 1 && std::fabs(ms[0].v[0] - 0.2f) < 1e-5f && std::fabs(ms[0].v[1] - 1.35f) < 1e-5f &&
              std::fabs(ms[0].v[3] - 0.6f) < 1e-5f && std::fabs(ms[0].v[4] - 1.5f) < 1e-5f,
          "head frame -> standing universe center and size");
    return fails ? 1 : 0;
}

int main(int argc, char** argv) {
    g_det.size = 320;
    g_det.max_hz = 4;
    g_det.conf = 0.1f;  // ByteTrack wants the weak detections too; only >= --track-thresh ones start tracks
    track::Tracker tracker;
    if (const char* home = getenv("HOME")) g_conf = std::string(home) + "/.config/guardian.conf";
    load_settings();
    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        auto next = [&]() -> const char* {
            if (i + 1 >= argc) throw std::runtime_error("missing value for " + a);
            return argv[++i];
        };
        if (a == "--selftest") return g_conf.clear(), g_set = {}, selftest();
        else if (a == "--register") return register_app();
        else if (a == "--max-dist") g_set.max_dist = float(atof(next()));
        else if (a == "--confirm-hits") tracker.prm.min_hits = atoi(next());
        else if (a == "--hold-ms") tracker.prm.max_age = float(atof(next())) / 1000;
        else if (a == "--track-thresh") tracker.prm.high = float(atof(next()));
        else if (a == "--gate") tracker.prm.gate = float(atof(next()));
        else if (a == "--accel") tracker.prm.accel = float(atof(next()));
        else if (a == "--smooth-ms") g_smooth_s = float(atof(next())) / 1000;
        else if (a == "--no-detector") g_detector = false;
        else if (parse_detector_option(a, next)) continue;
        else { fprintf(stderr, "unknown option %s\n", a.c_str()); return 2; }
    }
    if (!finish_detector_options()) return 2;
    setvbuf(stdout, nullptr, _IOLBF, 0);
    signal(SIGINT, [](int) { g_quit = true; });
    signal(SIGTERM, [](int) { g_quit = true; });

    vr::EVRInitError err;
    vr::VR_Init(&err, vr::VRApplication_Overlay);
    if (err) { fprintf(stderr, "VR_Init: %s\n", vr::VR_GetVRInitErrorAsEnglishDescription(err)); return 1; }
    vr::VROverlayHandle_t overlays[kMaxOverlays], labels[kMaxOverlays];
    int tex_h[kMaxOverlays] = {}, tex_cls[kMaxOverlays] = {};  // what each box overlay's texture was made for
    std::string label_text[kMaxOverlays];
    for (int i = 0; i < kMaxOverlays * 2; i++) {
        std::string key = std::string(kAppKey) + (i < kMaxOverlays ? ".box" : ".label") + std::to_string(i % kMaxOverlays);
        if (vr::VROverlay()->CreateOverlay(key.c_str(), "guardian", i < kMaxOverlays ? &overlays[i] : &labels[i - kMaxOverlays])) {
            fprintf(stderr, "CreateOverlay %s failed\n", key.c_str());
            return 1;
        }
    }

    vr::VROverlayHandle_t dash, dash_icon;
    if (vr::VROverlay()->CreateDashboardOverlay((std::string(kAppKey) + ".dashboard").c_str(), "Guardian", &dash, &dash_icon)) {
        fprintf(stderr, "CreateDashboardOverlay failed\n");
        return 1;
    }
    {
        int size;
        std::vector<uint8_t> px = panel::icon(size);
        vr::VROverlay()->SetOverlayRaw(dash_icon, px.data(), size, size, 4);
        vr::VROverlay()->SetOverlayWidthInMeters(dash, 1.5f);
        vr::VROverlay()->SetOverlayInputMethod(dash, vr::VROverlayInputMethod_Mouse);
        vr::HmdVector2_t scale{float(panel::W), float(panel::H)};
        vr::VROverlay()->SetOverlayMouseScale(dash, &scale);
        px = panel::render(-1, "Starting");
        vr::VROverlay()->SetOverlayRaw(dash, px.data(), panel::W, panel::H, 4);
    }
    int hover = -1;
    std::string panel_status;  // what the panel was last drawn with ("" = redraw)

    std::thread det;
    if (g_detector) det = std::thread([] {
        try {
            detect_loop();
        } catch (const std::exception& e) {
            fprintf(stderr, "detector: %s\n", e.what());
            g_quit = true;
        }
    });

    std::unordered_map<uint64_t, std::array<float, 5>> shown;  // per track id: what's drawn (smoothed estimate)
    uint64_t seen_det_id = 0, last_det_ns = 0, last_frame_ns = now_ns();
    int shown_before = 0;
    while (!g_quit) {
        // Once per compositor frame; sleep instead if there's no frame to sync to.
        if (vr::VROverlay()->WaitFrameSync(100) != vr::VROverlayError_None) usleep(11000);
        log_frame_stats();
        vr::VREvent_t ev;
        while (vr::VRSystem()->PollNextEvent(&ev, sizeof ev))
            if (ev.eventType == vr::VREvent_Quit) { vr::VRSystem()->AcknowledgeQuit_Exiting(); g_quit = true; }
        while (vr::VROverlay()->PollNextOverlayEvent(dash, &ev, sizeof ev)) {
            // Mouse y is up from the bottom of the texture.
            int c = panel::hit(ev.data.mouse.x, panel::H - ev.data.mouse.y);
            if (ev.eventType == vr::VREvent_MouseMove && c != hover) hover = c, panel_status.clear();
            if (ev.eventType == vr::VREvent_MouseButtonDown && c >= 0) panel::press(c), panel_status.clear();
        }
        g_det_paused = !g_set.enabled;
        if (!g_set.enabled) tracker.tracks.clear();

        DetectionSet d;
        {
            std::lock_guard<std::mutex> lk(g_det_mutex);
            if (g_dets.id != seen_det_id) d = g_dets;
        }
        uint64_t now = now_ns();
        bool ok;
        if (d.id) {
            seen_det_id = d.id;
            last_det_ns = now;
            // Negative prediction = head pose in the past, at capture time.
            float ago = -(now - d.t_capture_ns) / 1e9f;
            vr::HmdMatrix34_t head = head_pose(ago, ok);
            if (ok) tracker.update(measurements(d.objs, head, controllers(ago)), d.t_capture_ns / 1e9);
        }

        vr::HmdMatrix34_t head = head_pose(0, ok);
        XrVector3f h{head.m[0][3], head.m[1][3], head.m[2][3]};
        float k = 1 - std::exp(-(now - last_frame_ns) / 1e9f / std::max(g_smooth_s, 1e-3f));
        last_frame_ns = now;
        std::unordered_map<uint64_t, std::array<float, 5>> still_shown;
        std::string where;  // for the log: each shown box relative to the head
        int n = 0;
        for (const track::Track& t : tracker.tracks) {
            // Confirmed and matched recently (the tracker drops stale tracks only when the next detections come).
            if (!t.confirmed || now / 1e9 - t.last_seen > tracker.prm.max_age || n == kMaxOverlays) continue;
            std::array<float, 5> est;
            tracker.estimate(t, now / 1e9, est.data());
            auto it = shown.find(t.id);
            std::array<float, 5> p = it == shown.end() ? est : it->second;
            for (int i = 0; i < 5; i++) p[i] += (est[i] - p[i]) * k;
            still_shown[t.id] = p;
            float w = p[3], ht = p[4];
            {
                // Bearing from the head's horizontal forward (-z), + = right; height relative to the eyes.
                float vx = p[0] - h.x, vz = p[2] - h.z, fx = -head.m[0][2], fz = -head.m[2][2];
                char b[96];
                snprintf(b, sizeof b, " | %s %.0f%% %.1fm %+.0fdeg %+.1fm", kCoco[t.cls], t.score * 100, std::hypot(vx, vz),
                         std::atan2(fx * vz - fz * vx, fx * vx + fz * vz) * 180 / float(M_PI), p[1] - h.y);
                where += b;
            }
            // 256 texels across; height in steps of 8 so the texture is only redrawn when the aspect really changes
            int pw = 256, ph = std::clamp(int(std::lround(pw * ht / w / 8)) * 8, 32, 1024);
            if (ph != tex_h[n] || t.cls != tex_cls[n]) {
                std::vector<uint8_t> px = outline(pw, ph, t.cls, std::clamp(pw * 0.008f / w, 1.5f, 8.f));  // ~8 mm line
                vr::VROverlay()->SetOverlayRaw(overlays[n], px.data(), pw, ph, 4);
                tex_h[n] = ph, tex_cls[n] = t.cls;
            }
            vr::VROverlay()->SetOverlayWidthInMeters(overlays[n], w);
            vr::HmdMatrix34_t m = facing({p[0], p[1], p[2]}, h);
            vr::VROverlay()->SetOverlayTransformAbsolute(overlays[n], vr::TrackingUniverseStanding, &m);

            int pct = int(std::lround(t.score * 100));
            std::string text = std::string(kCoco[t.cls]) + std::to_string(pct);
            if (label_text[n] != text) {
                int lw, lh;
                std::vector<uint8_t> px = label(kCoco[t.cls], pct, t.cls, lw, lh);
                vr::VROverlay()->SetOverlayRaw(labels[n], px.data(), lw, lh, 4);
                vr::VROverlay()->SetOverlayWidthInMeters(labels[n], kLabelH * lw / lh);
                label_text[n] = text;
            }
            m.m[1][3] += ht / 2 + kLabelH / 2 + 0.015f;  // same sprite plane, just above the box
            vr::VROverlay()->SetOverlayTransformAbsolute(labels[n], vr::TrackingUniverseStanding, &m);
            if (n >= shown_before) vr::VROverlay()->ShowOverlay(overlays[n]), vr::VROverlay()->ShowOverlay(labels[n]);
            n++;
        }
        for (int i = n; i < shown_before; i++) vr::VROverlay()->HideOverlay(overlays[i]), vr::VROverlay()->HideOverlay(labels[i]);
        static uint64_t last_where_ns = 0;
        if (n != shown_before || (n && now - last_where_ns > 2000000000ull)) {
            printf("guardian: showing %d%s\n", n, where.c_str());
            last_where_ns = now;
        }
        shown_before = n;
        shown.swap(still_shown);

        if (vr::VROverlay()->IsOverlayVisible(dash)) {
            char st[128];
            if (!g_set.enabled) snprintf(st, sizeof st, "Off");
            else if (!g_detector) snprintf(st, sizeof st, "Detector off (--no-detector)");
            else if (now - last_det_ns > 2000000000ull) snprintf(st, sizeof st, "Waiting for camera frames");
            else snprintf(st, sizeof st, "Watching  ·  YOLO %.0f ms  ·  %d in view", g_det_yolo_ms.load(), n);
            if (panel_status != st) {
                panel_status = st;
                std::vector<uint8_t> px = panel::render(hover, panel_status);
                vr::VROverlay()->SetOverlayRaw(dash, px.data(), panel::W, panel::H, 4);
            }
        }
    }
    if (det.joinable()) det.join();
    vr::VR_Shutdown();
    ncnn::destroy_gpu_instance();
    return 0;
}
