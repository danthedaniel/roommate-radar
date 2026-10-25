// Stereo for the detector: matches YOLO's boxes in the two rectified views (a rectified pair, horizontal epipolar
// lines) and triangulates their depth from disparity, into head-frame objects.
#pragma once
#include <openxr/openxr.h>  // XrVector3f only

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

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

// Matches the left view's boxes to the right view's: same rows, similar size, positive disparity. Class-agnostic like
// nms() (the views can disagree on laptop/tv for one object), with a mild preference for the same class. L, R: the
// S x S views; f (px) and baseline (m) of the pair; origin: the left camera in the head frame.
static std::vector<Obj> match_stereo(const std::vector<Det> views[2], const uint8_t* L, const uint8_t* R, int S,
                                     float f, float baseline, XrVector3f origin) {
    std::vector<Obj> objs;
    std::vector<bool> used(views[1].size());
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
    return objs;
}
