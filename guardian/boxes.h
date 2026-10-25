// The tracks as OpenVR overlays: per confirmed track a box (rounded outline) and a label (class, confidence), upright
// sprites turned to face you every compositor frame, in the standing universe. Each box's center and size follow a
// cubic spline (time-based Catmull-Rom) through its track's last detections, played back behind the newest one so
// there is always a segment to be in.
#pragma once
#include <openvr.h>

#include <numeric>

#include "detector.h"
#include "draw.h"
#include "textures.h"
#include "tracker.h"

// A box (center p, width w, height h; standing universe) as seen from the head position: azimuth/elevation (rad) of
// its edges, so duplicate() can tell two tracks of one person apart from two people.
static Det angular_box(const float* p, float w, float h, XrVector3f head) {
    float d = std::hypot(p[0] - head.x, p[2] - head.z), az = std::atan2(p[0] - head.x, head.z - p[2]),
          el = std::atan2(p[1] - head.y, d), hw = std::atan2(w / 2, d), hh = std::atan2(h / 2, d);
    return {az - hw, el - hh, az + hw, el + hh, 0, 0};
}

// Upright quad at c facing the head position h (overlay +z points at the viewer).
static vr::HmdMatrix34_t facing(XrVector3f c, XrVector3f h) {
    float zx = h.x - c.x, zz = h.z - c.z, n = std::hypot(zx, zz);
    if (n < 1e-4f) {
        zx = 0, zz = 1, n = 1;
    }
    zx /= n, zz /= n;
    return {{{zz, 0, zx, c.x}, {0, 1, 0, c.y}, {-zx, 0, zz, c.z}}};
}

// Cubic Hermite through the samples with Catmull-Rom tangents scaled by the real sample spacing (one-sided at the
// ends), so uneven detector timing doesn't kink the path. Clamps to the first/last sample outside their span.
static track::Sample spline_at(const std::deque<track::Sample>& s, double t) {
    if (t <= s.front().t) {
        return s.front();
    }
    if (t >= s.back().t) {
        return s.back();
    }
    size_t i = 0;
    while (s[i + 1].t <= t) {
        i++;
    }
    auto tangent = [&](size_t k, int c) {  // per second
        size_t a = k ? k - 1 : k, b = std::min(k + 1, s.size() - 1);
        return (s[b].v[c] - s[a].v[c]) / float(s[b].t - s[a].t);
    };
    float dt = float(s[i + 1].t - s[i].t), u = float(t - s[i].t) / dt, u2 = u * u, u3 = u2 * u;
    track::Sample r{t, {}};
    for (int c = 0; c < 5; c++) {
        r.v[c] = (2 * u3 - 3 * u2 + 1) * s[i].v[c] + (u3 - 2 * u2 + u) * dt * tangent(i, c) +
                 (-2 * u3 + 3 * u2) * s[i + 1].v[c] + (u3 - u2) * dt * tangent(i + 1, c);
    }
    return r;
}

namespace boxes {

const int kMax = 8;           // overlays of each kind
const float kLabelH = 0.07f;  // label height (m), just above the box
static vr::VROverlayHandle_t outlines[kMax], labels[kMax];
static int tex_h[kMax], tex_cls[kMax];  // what each box overlay's texture was made for
static std::string label_text[kMax];
static int shown = 0;

static bool create() {
    for (auto [kind, ovs] : {std::pair{".box", outlines}, std::pair{".label", labels}}) {
        for (int i = 0; i < kMax; i++) {
            std::string key = std::string("steamframe.guardian") + kind + std::to_string(i);
            if (vr::VROverlay()->CreateOverlay(key.c_str(), "guardian", &ovs[i])) {
                fprintf(stderr, "CreateOverlay %s failed\n", key.c_str());
                return false;
            }
        }
    }
    return true;
}

// Shows the tracker's confirmed tracks at playback time t_play (s) to the head at `head`; returns how many.
static int show(const track::Tracker& tracker, const vr::HmdMatrix34_t& head, uint64_t now, double t_play,
                Textures& textures) {
    XrVector3f h{head.m[0][3], head.m[1][3], head.m[2][3]};
    std::string where;  // for the log: each shown box relative to the head
    int n = 0;
    // Most recently matched first: when a depth jump misses the tracker's gate, the new track takes over and the
    // old one, still coasting, would be drawn as a second box just in front of or behind it.
    std::vector<size_t> order(tracker.tracks.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](size_t a, size_t b) {
        const track::Track &x = tracker.tracks[a], &y = tracker.tracks[b];
        return x.last_seen != y.last_seen ? x.last_seen > y.last_seen : x.score > y.score;
    });
    std::vector<Det> drawn;  // angular boxes (rad) seen from the head, for duplicate()
    for (size_t ti : order) {
        const track::Track& t = tracker.tracks[ti];
        // Confirmed and matched recently (the tracker drops stale tracks only when the next detections come).
        if (!t.confirmed || now / 1e9 - t.last_seen > tracker.prm.max_age || n == kMax) {
            continue;
        }
        track::Sample sp = spline_at(t.history, t_play);
        const float* p = sp.v;
        float w = std::max(p[3], 0.05f), ht = std::max(p[4], 0.05f);
        Det a = angular_box(p, w, ht, h);
        if (std::any_of(drawn.begin(), drawn.end(), [&](const Det& b) {
                return duplicate(a, b);
            })) {
            continue;
        }
        drawn.push_back(a);
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
            textures.set(outlines[n], px, pw, ph);
            tex_h[n] = ph, tex_cls[n] = t.cls;
        }
        vr::VROverlay()->SetOverlayWidthInMeters(outlines[n], w);
        vr::HmdMatrix34_t m = facing({p[0], p[1], p[2]}, h);
        vr::VROverlay()->SetOverlayTransformAbsolute(outlines[n], vr::TrackingUniverseStanding, &m);

        int pct = int(std::lround(t.score * 100));
        std::string text = std::string(kCoco[t.cls]) + std::to_string(pct);
        if (label_text[n] != text) {
            int lw, lh;
            std::vector<uint8_t> px = label(kCoco[t.cls], pct, t.cls, lw, lh);
            textures.set(labels[n], px, lw, lh);
            vr::VROverlay()->SetOverlayWidthInMeters(labels[n], kLabelH * lw / lh);
            label_text[n] = text;
        }
        m.m[1][3] += ht / 2 + kLabelH / 2 + 0.015f;  // same sprite plane, just above the box
        vr::VROverlay()->SetOverlayTransformAbsolute(labels[n], vr::TrackingUniverseStanding, &m);
        if (n >= shown) {
            vr::VROverlay()->ShowOverlay(outlines[n]), vr::VROverlay()->ShowOverlay(labels[n]);
        }
        n++;
    }
    for (int i = n; i < shown; i++) {
        vr::VROverlay()->HideOverlay(outlines[i]), vr::VROverlay()->HideOverlay(labels[i]);
    }
    static uint64_t last_where_ns = 0;
    if (n != shown || (n && now - last_where_ns > 2000000000ull)) {
        printf("guardian: showing %d%s\n", n, where.c_str());
        last_where_ns = now;
    }
    return shown = n;
}

}  // namespace boxes
