// The detector's objects (head frame at capture) as the tracker's measurements (standing universe), keeping only what
// guardian shows: stereo, a wanted class, in range, not your own arms or hands.
#pragma once
#include <openvr.h>

#include "detector.h"
#include "settings.h"
#include "tracker.h"

// person, cat, dog
static bool wanted(int cls) {
    return cls == 0 ? g_set.people : (cls == 15 || cls == 16) && g_set.pets;
}

static XrVector3f v_add(XrVector3f a, XrVector3f b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}
static XrVector3f v_sub(XrVector3f a, XrVector3f b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}
static XrVector3f v_mul(XrVector3f a, float s) {
    return {a.x * s, a.y * s, a.z * s};
}

// Your own arms and hands (YOLO often calls them "person"; it doesn't know the controller, so an arm box can stop at
// the wrist and leave it out): a point on the forearm, from 40% of the way from the shoulder to a controller up to the
// controller, inside the box: within its rectangle scaled to that point's depth (seen from the camera) and grown by
// 15 cm, and from 40 cm behind its depth to 80 cm in front of it (arm boxes measure too far: cut off by the image edge
// differently in each view, or merged with what's behind the arm). Box and controllers are in the head frame at
// capture time.
static bool holds_controller(const Obj& o, const std::vector<XrVector3f>& controllers) {
    const float m = 0.15f;
    const XrVector3f c = o.origin;
    const XrVector3f* b = o.corners;
    for (const XrVector3f& hand : controllers) {
        const XrVector3f shoulder{std::copysign(0.18f, hand.x), -0.25f, 0};  // ponytail: one body size for everyone
        for (int k = 4; k <= 10; k++) {
            XrVector3f p = v_add(shoulder, v_mul(v_sub(hand, shoulder), k / 10.f));
            float s = (p.z - c.z) / (b[0].z - c.z);
            if (s > 0 && p.x > c.x + (b[0].x - c.x) * s - m && p.x < c.x + (b[1].x - c.x) * s + m &&
                p.y > c.y + (b[3].y - c.y) * s - m && p.y < c.y + (b[0].y - c.y) * s + m && p.z - b[0].z > -0.4f &&
                p.z - b[0].z < 0.8f) {
                return true;
            }
        }
    }
    return false;
}

// Your own arm or lap, controller or not: the box reaches the bottom 10% of the view, is within 1.2 m, and its top is
// below eye level (anyone else that close shows their head). Head frame, like holds_controller.
static bool own_arm(const Obj& o) {
    const XrVector3f* b = o.corners;
    float Z = o.origin.z - b[3].z, edge = std::tan(g_det.hfov_deg * float(M_PI) / 360);
    return Z > 0 && Z < 1.2f && (o.origin.y - b[3].y) / Z > 0.8f * edge && b[0].y < o.origin.y;
}

static float v_len(XrVector3f a) {
    return std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
}
static XrVector3f v_center(const XrVector3f* c) {
    return v_mul(v_add(v_add(c[0], c[1]), v_add(c[2], c[3])), 0.25f);
}

static XrVector3f apply(const vr::HmdMatrix34_t& m, XrVector3f p) {
    return {m.m[0][0] * p.x + m.m[0][1] * p.y + m.m[0][2] * p.z + m.m[0][3],
            m.m[1][0] * p.x + m.m[1][1] * p.y + m.m[1][2] * p.z + m.m[1][3],
            m.m[2][0] * p.x + m.m[2][1] * p.y + m.m[2][2] * p.z + m.m[2][3]};
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
        if (!o.stereo || !wanted(o.cls) || holds_controller(o, ctrl_head) || (g_set.hands && own_arm(o))) {
            continue;
        }
        XrVector3f w[4];
        for (int k = 0; k < 4; k++) {
            w[k] = apply(head, o.corners[k]);
        }
        XrVector3f c = v_center(w);
        float dist = std::hypot(c.x - h.x, c.z - h.z);
        if (dist > g_set.max_dist || (o.cls == 0 && dist < 0.5f)) {  // a "person" closer than 50 cm is you
            continue;
        }
        // ponytail: isotropic position noise growing like stereo depth error (~Z^2); per-axis noise along the view
        // ray would track lateral motion tighter
        float sigma = 0.05f + 0.02f * o.dist * o.dist;
        ms.push_back({{c.x, c.y, c.z, v_len(v_sub(w[1], w[0])), v_len(v_sub(w[3], w[0]))},
                      sigma,
                      o.score,
                      o.cls,
                      {h.x, h.y, h.z}});
    }
    return ms;
}
