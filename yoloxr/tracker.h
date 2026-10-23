// ByteTrack-style multi-object tracker in 3D (guardian). Each track is a constant-velocity Kalman filter over
// [x, y, z, w, h] (box center in the standing universe and box size, meters); axes are independent, so it's five
// 2-state filters (position, velocity) rather than one 10-state one. Per detector frame:
//   1. predict every track to the frame's capture time,
//   2. match tracks to confident detections (Hungarian, cost = 3D center distance, gated),
//   3. match the tracks left over to low-confidence detections (keeps a track through a weak frame),
//   4. unmatched confident detections start tentative tracks; tentative tracks that miss are dropped, confirmed
//      ones after max_age without a match.
// Each track also keeps its last few matched detections (history) for drawing; the filter's predictions are only
// used for matching.
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <vector>

namespace track {

struct Meas {
    float v[5];     // x, y, z, w, h (m)
    float sigma;    // position measurement std dev (m); stereo depth gets worse with distance
    float score;
    int cls;
};

struct Params {
    float high = 0.5f;       // detections at least this confident can start tracks (step 2); the rest only extend (3)
    float gate = 0.7f;       // max 3D distance (m) between a track's prediction and a detection to match them
    float max_age = 1.0f;    // drop a track this long (s) after its last match
    int min_hits = 2;        // matches before a track is confirmed (shown)
    float accel = 2.0f;      // process noise: std dev of a person's acceleration (m/s^2)
    float size_rate = 0.3f;  // process noise for box size (m/s^2 equivalent)
    float size_sigma = 0.1f; // box size measurement std dev (m)
    size_t history = 4;      // matched detections kept per track
};

// One axis: position p, velocity v, covariance P.
struct Axis {
    float p = 0, v = 0, P[2][2] = {{1, 0}, {0, 1}};

    void init(float z, float r, float v_var) { p = z, v = 0, P[0][0] = r * r, P[0][1] = P[1][0] = 0, P[1][1] = v_var; }

    // Constant velocity with white-noise acceleration of std dev a.
    void predict(float dt, float a) {
        p += v * dt;
        float q = a * a, p00 = P[0][0] + dt * (P[0][1] + P[1][0]) + dt * dt * P[1][1], p01 = P[0][1] + dt * P[1][1];
        P[0][0] = p00 + q * dt * dt * dt / 3;
        P[0][1] = P[1][0] = p01 + q * dt * dt / 2;
        P[1][1] += q * dt;
    }

    void update(float z, float r) {
        float s = P[0][0] + r * r, k0 = P[0][0] / s, k1 = P[1][0] / s, y = z - p;
        p += k0 * y, v += k1 * y;
        float p00 = (1 - k0) * P[0][0], p01 = (1 - k0) * P[0][1], p11 = P[1][1] - k1 * P[0][1];
        P[0][0] = p00, P[0][1] = P[1][0] = p01, P[1][1] = p11;
    }
};

// One matched detection: capture time (s) and x, y, z, w, h.
struct Sample {
    double t;
    float v[5];
};

struct Track {
    uint64_t id;
    Axis axis[5];
    double t;          // time the filter state is at (s)
    double last_seen;  // time of the last matched detection (s)
    int hits = 1, cls;
    float score;
    bool confirmed = false;  // reached min_hits; stays set while the track lives
    std::deque<Sample> history;  // last prm.history matched detections, oldest first
};

// Minimum-cost assignment of rows to columns (Hungarian, O(n^2 m)); match[row] = column or -1. Pairs costing more
// than `gate` are left unmatched.
inline std::vector<int> assign(const std::vector<std::vector<float>>& cost, float gate) {
    size_t n = cost.size(), m = n ? cost[0].size() : 0;
    std::vector<int> match(n, -1);
    if (!n || !m) return match;
    bool flip = n > m;  // the algorithm wants rows <= columns
    size_t R = flip ? m : n, C = flip ? n : m;
    auto a = [&](size_t i, size_t j) {  // 1-based; over-gate pairs get a large finite cost so they're picked last
        float c = flip ? cost[j - 1][i - 1] : cost[i - 1][j - 1];
        return c > gate ? gate * 100 : c;
    };
    const float inf = std::numeric_limits<float>::infinity();
    std::vector<float> u(R + 1), v(C + 1);
    std::vector<size_t> p(C + 1), way(C + 1);
    for (size_t i = 1; i <= R; i++) {
        p[0] = i;
        size_t j0 = 0;
        std::vector<float> minv(C + 1, inf);
        std::vector<bool> used(C + 1);
        do {
            used[j0] = true;
            size_t i0 = p[j0], j1 = 0;
            float delta = inf;
            for (size_t j = 1; j <= C; j++)
                if (!used[j]) {
                    float cur = a(i0, j) - u[i0] - v[j];
                    if (cur < minv[j]) minv[j] = cur, way[j] = j0;
                    if (minv[j] < delta) delta = minv[j], j1 = j;
                }
            for (size_t j = 0; j <= C; j++)
                if (used[j]) u[p[j]] += delta, v[j] -= delta;
                else minv[j] -= delta;
            j0 = j1;
        } while (p[j0]);
        do {
            size_t j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0);
    }
    for (size_t j = 1; j <= C; j++) {
        if (!p[j]) continue;
        size_t row = flip ? j - 1 : p[j] - 1, col = flip ? p[j] - 1 : j - 1;
        if (cost[row][col] <= gate) match[row] = int(col);
    }
    return match;
}

class Tracker {
  public:
    Params prm;
    std::vector<Track> tracks;

    // One detector frame captured at time t (s).
    void update(const std::vector<Meas>& dets, double t) {
        for (Track& tr : tracks) {
            float dt = float(std::max(0.0, t - tr.t));
            for (int k = 0; k < 5; k++) tr.axis[k].predict(dt, k < 3 ? prm.accel : prm.size_rate);
            tr.t = std::max(tr.t, t);
        }
        std::vector<int> high, low;
        for (int i = 0; i < int(dets.size()); i++) (dets[i].score >= prm.high ? high : low).push_back(i);

        std::vector<bool> matched(tracks.size());
        std::vector<int> unused_high = match(dets, high, matched, t);
        match(dets, low, matched, t);
        for (int i : unused_high) {
            const Meas& d = dets[i];
            Track tr{next_id_++, {}, t, t, 1, d.cls, d.score, prm.min_hits <= 1, {}};
            for (int k = 0; k < 5; k++) tr.axis[k].init(d.v[k], k < 3 ? d.sigma : prm.size_sigma, k < 3 ? 1.0f : 0.1f);
            remember(tr, d, t);
            tracks.push_back(tr);
            matched.push_back(true);
        }
        size_t n = 0;
        for (size_t i = 0; i < tracks.size(); i++) {
            const Track& tr = tracks[i];
            bool drop = (!tr.confirmed && !matched[i]) || t - tr.last_seen > prm.max_age;
            if (!drop) tracks[n++] = tr;
        }
        tracks.resize(n);
    }

  private:
    uint64_t next_id_ = 1;

    void remember(Track& tr, const Meas& d, double t) {
        Sample s{t, {}};
        std::copy(d.v, d.v + 5, s.v);
        if (tr.history.empty() || t > tr.history.back().t) tr.history.push_back(s);
        while (tr.history.size() > prm.history) tr.history.pop_front();
    }

    // Matches still-unmatched tracks to dets[which]; updates the matched tracks. Returns the unmatched detections.
    std::vector<int> match(const std::vector<Meas>& dets, const std::vector<int>& which, std::vector<bool>& matched, double t) {
        std::vector<int> rows;  // unmatched track indices
        for (size_t i = 0; i < tracks.size(); i++)
            if (!matched[i]) rows.push_back(int(i));
        std::vector<std::vector<float>> cost(rows.size(), std::vector<float>(which.size()));
        for (size_t r = 0; r < rows.size(); r++)
            for (size_t c = 0; c < which.size(); c++) {
                const Axis* a = tracks[rows[r]].axis;
                const float* z = dets[which[c]].v;
                cost[r][c] = std::sqrt((a[0].p - z[0]) * (a[0].p - z[0]) + (a[1].p - z[1]) * (a[1].p - z[1]) +
                                       (a[2].p - z[2]) * (a[2].p - z[2]));
            }
        std::vector<int> m = assign(cost, prm.gate);
        std::vector<bool> used(which.size());
        for (size_t r = 0; r < rows.size(); r++) {
            if (m[r] < 0) continue;
            Track& tr = tracks[rows[r]];
            const Meas& d = dets[which[m[r]]];
            for (int k = 0; k < 5; k++) tr.axis[k].update(d.v[k], k < 3 ? d.sigma : prm.size_sigma);
            tr.last_seen = t, tr.cls = d.cls, tr.score = d.score;
            remember(tr, d, t);
            if (++tr.hits >= prm.min_hits) tr.confirmed = true;
            matched[rows[r]] = true;
            used[m[r]] = true;
        }
        std::vector<int> rest;
        for (size_t c = 0; c < which.size(); c++)
            if (!used[c]) rest.push_back(which[c]);
        return rest;
    }
};

// Checks for guardian --selftest. Returns the number of failures.
inline int selftest(void (*report)(bool ok, const char* what)) {
    int fails = 0;
    auto check = [&](bool ok, const char* what) { report(ok, what); fails += !ok; };

    // Hungarian beats greedy: greedy takes the cheapest pair (0,0)=1 and is left with (1,1)=10; optimum is 2+2.
    std::vector<int> m = assign({{1, 2}, {2, 10}}, 100);
    check(m[0] == 1 && m[1] == 0, "assignment is optimal, not greedy");
    m = assign({{0.1f, 5}, {5, 5}, {5, 0.2f}}, 1);  // more rows than columns, one row gated out
    check(m[0] == 0 && m[1] == -1 && m[2] == 1, "assignment: rectangular, gated");

    // A person walking at 1 m/s along x, detected at 4 Hz with depth noise: confirmed on the 2nd frame, velocity
    // learned, the last 4 detections kept.
    Tracker trk;
    uint32_t seed = 1;
    auto noise = [&]() { seed = seed * 1664525u + 1013904223u; return (int(seed >> 9) % 2001 - 1000) / 1000.f * 0.05f; };
    for (int i = 0; i < 12; i++) {
        double t = i * 0.25;
        trk.update({{{float(t) + noise(), 1, -2 + noise(), 0.5f, 1.7f}, 0.05f, 0.8f, 0}}, t);
        if (i == 0) check(trk.tracks.size() == 1 && !trk.tracks[0].confirmed, "new track is tentative");
        if (i == 1) check(trk.tracks.size() == 1 && trk.tracks[0].confirmed, "second hit confirms it");
    }
    check(trk.tracks.size() == 1 && std::fabs(trk.tracks[0].axis[0].v - 1) < 0.25f &&
              trk.tracks[0].history.size() == 4 && trk.tracks[0].history.front().t == 8 * 0.25 &&
              trk.tracks[0].history.back().t == 11 * 0.25,
          "velocity learned, last 4 detections kept");

    // A weak (low-confidence) detection keeps the track; on its own it starts nothing.
    trk.update({{{3.0f, 1, -2, 0.5f, 1.7f}, 0.05f, 0.2f, 0}, {{-3, 1, -2, 0.5f, 1.7f}, 0.05f, 0.2f, 0}}, 3.0);
    check(trk.tracks.size() == 1 && trk.tracks[0].last_seen == 3.0, "low-confidence detection extends, doesn't start");

    // Two people crossing paths keep their ids (prediction + optimal assignment), then both expire.
    Tracker two;
    for (int i = 0; i < 12; i++) {
        float t = i * 0.25f, x = -1.5f + t;  // A moves +x, B moves -x; they pass at t = 1.5
        two.update({{{x, 1, -2, 0.5f, 1.7f}, 0.05f, 0.9f, 0}, {{-x, 1, -2.4f, 0.5f, 1.7f}, 0.05f, 0.9f, 0}}, t);
    }
    bool kept = two.tracks.size() == 2 && two.tracks[0].axis[0].v > 0.5f && two.tracks[1].axis[0].v < -0.5f;
    two.update({}, 2.75 + 1.1);
    check(kept && two.tracks.empty(), "crossing tracks keep their ids, then expire");
    return fails;
}

}  // namespace track
