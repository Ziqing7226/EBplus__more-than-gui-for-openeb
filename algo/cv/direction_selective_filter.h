// algo/cv/direction_selective_filter.h — 8-direction motion labelling.
//
// Inspired by jAER AbstractDirectionSelectiveFilter (design §4.3.8). jAER
// operates on orientation+polarity-typed events and searches only the two
// directions perpendicular to the edge orientation out to search_distance_
// pixels, restricted to the same orientation+polarity channel. This port
// provides an orientation-aware path (classify(e, ori) / process with an
// orientation array) plus a raw 8-neighbour fallback (classify(e)) that is a
// degraded mode when no upstream orientation pass is available.
//
// Per-event motion quantities (delay, distance, speed, velocity vector) and
// global low-pass translation / rotation / expansion estimates are computed
// following jAER's MotionVectors. Header-only.

#ifndef GUI_ALGO_CV_DIRECTION_SELECTIVE_FILTER_H
#define GUI_ALGO_CV_DIRECTION_SELECTIVE_FILTER_H

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

#include <metavision/sdk/base/utils/timestamp.h>

#include "algo/common/event.h"
#include "algo/common/event_packet.h"

namespace gui_algo {

/// @brief Labels events with one of 8 motion directions from a time surface.
class DirectionSelectiveFilter {
public:
    static constexpr int kNumDirections = 8;
    static constexpr int kNumOrientations = 4;  // upstream edge orientations
    static constexpr int kMaxSearchDistance = 12;

    /// @brief Direction index in [0,7]: 0=E,1=NE,2=N,3=NW,4=W,5=SW,6=S,7=SE.
    /// -1 = undetermined.

    DirectionSelectiveFilter(int width, int height)
        : width_(width), height_(height),
          cx_(static_cast<double>(width) / 2.0),
          cy_(static_cast<double>(height) / 2.0),
          // -1 = "never seen" sentinel (0 is a legal timestamp, §四-低8)
          surface_(static_cast<std::size_t>(width) * height, -1),
          ori_surface_(static_cast<std::size_t>(kNumDirections) * width * height, -1) {
        global_hist_.fill(0);
    }

    void set_time_window_us(int v) { time_window_us_ = clamp_i(v, 1000, 50000); }
    void set_enable_global_mode(bool v) { enable_global_mode_ = v; }
    /// @brief jAER minDtThreshold: minimum delta time (us) for a past event to
    /// be considered (filters noise / multiple spikes). Default 100.
    void set_min_dt_us(int v) { min_dt_us_ = clamp_i(v, 0, 1000000); }
    /// @brief jAER searchDistance: pixels searched perpendicular to the
    /// orientation (orientation-aware path). Default 3.
    void set_search_distance(int v) { search_distance_ = clamp_i(v, 1, kMaxSearchDistance); }
    /// @brief jAER tauLow: time constant (ms) of the global-motion low-pass
    /// filters. Default 100.
    void set_tau_low_ms(int v) {
        tau_low_ms_ = clamp_i(v, 1, 100000);
        const double tu = static_cast<double>(tau_low_ms_) * 1000.0;
        tx_.tau_us = tu; ty_.tau_us = tu; rot_.tau_us = tu; exp_.tau_us = tu;
    }

    int time_window_us() const { return time_window_us_; }
    bool enable_global_mode() const { return enable_global_mode_; }
    int min_dt_us() const { return min_dt_us_; }
    int search_distance() const { return search_distance_; }
    int tau_low_ms() const { return tau_low_ms_; }
    int width() const { return width_; }
    int height() const { return height_; }

    /// @brief Raw/degraded classify: inspects the 8 immediate neighbours.
    /// @return Direction index in [0,7] or -1 if no supporting neighbour.
    int classify(const Event& e) {
        int dir = -1;
        if (e.x < width_ && e.y < height_) {
            surface_[idx_of(e.x, e.y)] = e.t;  // own timestamp first (jAER order)
            const DirResult r = compute_direction(e);
            dir = r.dir;
            if (dir >= 0) {
                if (enable_global_mode_) ++global_hist_[dir];
                update_motion(e, r);
            }
        }
        // Decay toward zero every batch: the overlay is meant to show
        // RECENT direction distribution ("over the current batch" per the
        // accessor comment), but the counters only ever grew -- the text
        // climbed without bound (signed overflow on long sessions) and the
        // dominant direction locked in early. Same per-batch decay the
        // orientation filter's histogram uses.
        if (enable_global_mode_) {
            for (auto& cnt_bin : global_hist_) cnt_bin = (cnt_bin * 9) / 10;
        }
        return dir;
    }

    /// @brief Orientation-aware classify: searches only the two directions
    /// perpendicular to the edge orientation (jAER unitDirs[o]/unitDirs[(o+4)%8]
    /// with unitDirs starting at down; in this class's E-based table that maps
    /// to {(ori+2)%8, (ori+6)%8}) out to search_distance_, restricted to the
    /// same orientation+polarity channel.
    /// If @p ori is not in [0,3], falls back to the raw 8-neighbour path.
    int classify(const Event& e, int ori) {
        if (ori < 0 || ori >= kNumOrientations) return classify(e);
        int dir = -1;
        if (e.x < width_ && e.y < height_) {
            const int pol = e.p ? 1 : 0;
            // jAER order (rbodo AbstractMotionFlowIMU.extractEventInfo ->
            // isInvalidTimestamp): the event's own timestamp is written to
            // lastTimesMap[x][y][type] BEFORE the perpendicular search.
            // Functionally equivalent for s >= 1 (the own pixel is never
            // searched), kept for strict flow alignment.
            ori_surface_[idx_of_ori(e.x, e.y, ori, pol)] = e.t;
            const DirResult r = compute_direction_ori(e, ori, pol);
            dir = r.dir;
            if (dir >= 0) {
                if (enable_global_mode_) ++global_hist_[dir];
                update_motion(e, r);
            }
        }
        return dir;
    }

    /// @brief Classifies a batch (raw fallback); fills @p out (resized to count).
    void process(const Event* events, std::size_t count, std::vector<int>& out) {
        out.resize(count);
        for (std::size_t i = 0; i < count; ++i) {
            out[i] = classify(events[i]);
        }
    }

    /// @brief Classifies a batch with per-event orientations (orientation-aware).
    /// @p oris[i] is the upstream orientation of events[i] (-1 => raw fallback).
    void process(const Event* events, const int* oris, std::size_t count,
                 std::vector<int>& out) {
        out.resize(count);
        for (std::size_t i = 0; i < count; ++i) {
            out[i] = classify(events[i], oris[i]);
        }
    }

    /// @brief Processes an event packet (raw fallback; updates state + histogram).
    void process(EventPacket& events) {
        for (const auto& e : events) classify(e);
    }

    /// @brief Dominant global direction over the current batch (legacy raw-mode
    /// histogram argmax), or -1 if global mode is disabled / no events recorded.
    /// For proper jAER-style motion use translation() / rotation() / expansion().
    int global_direction() const {
        if (!enable_global_mode_) return -1;
        int best = -1, best_count = 0;
        for (int i = 0; i < kNumDirections; ++i) {
            if (global_hist_[i] > best_count) {
                best_count = global_hist_[i];
                best = i;
            }
        }
        return best;
    }

    const std::array<int, kNumDirections>& global_histogram() const {
        return global_hist_;
    }

    /// @brief Global translational motion (px/s), low-pass filtered.
    struct Vec2 { float x{0.0F}; float y{0.0F}; };
    Vec2 translation() const {
        return Vec2{static_cast<float>(tx_.value), static_cast<float>(ty_.value)};
    }
    /// @brief Global rotation about the sensor centre (rad/s), low-pass filtered.
    float rotation() const { return static_cast<float>(rot_.value); }
    /// @brief Global expansion/contraction about the sensor centre (1/s),
    /// low-pass filtered (1/Tcoll).
    float expansion() const { return static_cast<float>(exp_.value); }

    /// @brief Per-event motion quantities of the most recently classified event.
    Metavision::timestamp last_delay_us() const { return last_delay_us_; }
    int last_distance_px() const { return last_distance_px_; }
    float last_speed_pps() const { return last_speed_pps_; }
    Vec2 last_velocity() const { return Vec2{last_vx_, last_vy_}; }

    /// @brief Motion angle (deg) for a direction index — the direction the edge
    /// moves to (opposite of the source). Returns -1 for invalid input.
    static float direction_angle(int dir) {
        if (dir < 0 || dir >= kNumDirections) return -1.0F;
        return static_cast<float>(((dir + 4) % kNumDirections) * 45);
    }
    /// @brief Source angle (deg) for a direction index — where the previous
    /// event came from. Returns -1 for invalid input.
    static float source_angle(int dir) {
        if (dir < 0 || dir >= kNumDirections) return -1.0F;
        return static_cast<float>(dir) * 45.0F;
    }

    void reset() {
        std::fill(surface_.begin(), surface_.end(), -1);
        std::fill(ori_surface_.begin(), ori_surface_.end(), -1);
        global_hist_.fill(0);
        tx_.reset(); ty_.reset(); rot_.reset(); exp_.reset();
        last_delay_us_ = 0;
        last_distance_px_ = 0;
        last_speed_pps_ = 0.0F;
        last_vx_ = 0.0F;
        last_vy_ = 0.0F;
    }

private:
    static int clamp_i(int v, int lo, int hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }

    std::size_t idx_of(int x, int y) const {
        return static_cast<std::size_t>(y) * width_ + x;
    }
    /// Orientation+polarity channel index (8 channels). ori in [0,3], pol in {0,1}.
    std::size_t idx_of_ori(int x, int y, int ori, int pol) const {
        return static_cast<std::size_t>(ori * 2 + pol)
                   * static_cast<std::size_t>(width_) * height_
             + static_cast<std::size_t>(y) * width_ + x;
    }

    // 8 direction step vectors: 0=E,1=NE,2=N,3=NW,4=W,5=SW,6=S,7=SE.
    static constexpr int kDirDx[kNumDirections] = {1, 1, 0, -1, -1, -1, 0, 1};
    static constexpr int kDirDy[kNumDirections] = {0, -1, -1, -1, 0, 1, 1, 1};

    struct DirResult {
        int dir{-1};
        Metavision::timestamp delay{0};  // dt to the supporting neighbour (us)
        int distance{0};                 // pixels to the supporting neighbour
        double speed_px_s{0.0};          // jAER aggregate speed (px/s) when
                                         // distance==0 (aggregate path)
    };

    /// Raw/degraded: search the 8 immediate neighbours for the smallest positive
    /// recency within the time window (and above min_dt_us_).
    DirResult compute_direction(const Event& e) const {
        const Metavision::timestamp win = time_window_us_;
        DirResult best;
        best.delay = win;
        for (int d = 0; d < kNumDirections; ++d) {
            const int nx = e.x + kDirDx[d];
            const int ny = e.y + kDirDy[d];
            if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) continue;
            const Metavision::timestamp lt = surface_[idx_of(nx, ny)];
            if (lt < 0) continue;  // -1 = never seen
            const Metavision::timestamp rec = e.t - lt;
            if (rec <= min_dt_us_ || rec > win) continue;
            if (rec < best.delay) {
                best.delay = rec;
                best.dir = d;
                best.distance = 1;
            }
        }
        if (best.dir < 0) best.delay = 0;
        return best;
    }

    /// Orientation-aware: search only the two directions perpendicular to the
    /// edge orientation out to search_distance_ pixels, restricted to the same
    /// orientation+polarity channel (per spec).
    /// 方向表换算（§一-1.2，修正 90° 错位）：jAER unitDirs 从 down 起
    /// (unitDirs[0]=(0,-1))，朝向通道 o 搜 unitDirs[o] 与 unitDirs[(o+4)%8]，
    /// 即 o=0（水平边缘）搜 down/up —— 垂直于边缘。本表 0=E（kDirDx/kDirDy），
    /// 同一搜索对应 {(ori+2)%8, (ori+6)%8}（ori=0 → N/S）。
    DirResult compute_direction_ori(const Event& e, int ori, int pol) const {
        // jAER rbodo DirectionSelectiveFlow.filterPacket: for each of the two
        // directions perpendicular to the edge, iterate s = 1..searchDistance,
        // COUNT the in-window matches (n1/n2) and SUM their speeds
        // (s/dt, px/us). The motion direction is the one with MORE evidence;
        // on a tie the SLOWER average speed wins; speed = sum/count * 1e6
        // (px/s). This replaces the former single-minimum-delay decision,
        // which picked one nearest neighbour instead of the aggregate.
        const Metavision::timestamp win = time_window_us_;
        const int dirs[2] = {(ori + 2) % kNumDirections,
                             (ori + 6) % kNumDirections};
        DirResult best;
        best.delay = 0;
        int n[2] = {0, 0};
        double speed_sum[2] = {0.0, 0.0};
        for (int dd = 0; dd < 2; ++dd) {
            const int d = dirs[dd];
            const int ddx = kDirDx[d];
            const int ddy = kDirDy[d];
            for (int s = 1; s <= search_distance_; ++s) {
                const int nx = e.x + s * ddx;
                const int ny = e.y + s * ddy;
                if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) continue;
                const Metavision::timestamp lt = ori_surface_[idx_of_ori(nx, ny, ori, pol)];
                if (lt < 0) continue;  // -1 = never seen
                const Metavision::timestamp rec = e.t - lt;
                if (rec <= min_dt_us_ || rec > win) continue;
                ++n[dd];
                speed_sum[dd] += static_cast<double>(s) / static_cast<double>(rec);
                best.delay = std::max(best.delay, rec);
            }
        }
        int chosen = -1;
        if (n[0] > 0 || n[1] > 0) {
            if (n[0] > n[1] || (n[0] == n[1] &&
                (n[0] == 0 || speed_sum[0] / n[0] <= speed_sum[1] / n[1]))) {
                chosen = 0;
            } else {
                chosen = 1;
            }
            best.dir = dirs[chosen];
            best.distance = 0;  // aggregate — per-neighbour distance not unique
            // jAER aggregate speed: sum(s/dt)/n * 1e6 (px/s).
            best.speed_px_s =
                static_cast<double>(n[chosen]) > 0
                    ? speed_sum[chosen] / static_cast<double>(n[chosen]) * 1e6
                    : 0.0;
            best.delay = best.delay > 0 ? best.delay : 1;
        }
        return best;
    }

    /// Per-event motion quantities + global low-pass estimates (jAER-style).
    void update_motion(const Event& e, const DirResult& r) {
        last_delay_us_ = r.delay;
        last_distance_px_ = r.distance;
        // Aggregate path (orientation-aware): the speed was already computed
        // in compute_direction_ori as the chosen side's mean s/dt (px/s).
        // Raw path (distance>0): speed = distance/delay (single neighbour).
        double speed = r.speed_px_s;
        if (r.distance > 0 && r.delay > 0) {
            speed = static_cast<double>(r.distance) /
                    static_cast<double>(r.delay) * 1e6;
        }
        if (r.delay <= 0 || speed <= 0.0 || r.dir < 0) {
            last_speed_pps_ = 0.0F;
            last_vx_ = 0.0F;
            last_vy_ = 0.0F;
            return;
        }
        last_speed_pps_ = static_cast<float>(speed);
        const int d = r.dir;
        // Unit vector of the source direction (diagonals normalised by sqrt(2)).
        const double len = (d % 2 == 0) ? 1.0 : std::sqrt(2.0);
        const double ux = static_cast<double>(kDirDx[d]) / len;
        const double uy = static_cast<double>(kDirDy[d]) / len;
        // Velocity points opposite to the source (motion direction).
        const double vx = -speed * ux;
        const double vy = -speed * uy;
        last_vx_ = static_cast<float>(vx);
        last_vy_ = static_cast<float>(vy);
        tx_.filter(vx, e.t);
        ty_.filter(vy, e.t);

        const double rx = static_cast<double>(e.x) - cx_;
        const double ry = static_cast<double>(e.y) - cy_;
        const double r2 = rx * rx + ry * ry;
        if (r2 > 0.0) {
            // Tangential contribution: (-vx*ry + vy*rx) / r^2 (rad/s).
            const double dphi = (-vx * ry + vy * rx) / r2;
            rot_.filter(dphi, e.t);
        }
        // Expansion excludes a small singular region around the centre.
        if (std::fabs(rx) >= 2.0 || std::fabs(ry) >= 2.0) {
            // Radial contribution: (vx*rx + vy*ry) / r^2 (1/Tcoll).
            const double dradial = (vx * rx + vy * ry) / r2;
            exp_.filter(dradial, e.t);
        }
    }

    /// First-order low-pass filter with time constant tau (us), à la jAER
    /// LowpassFilter.
    struct LowPass {
        double value{0.0};
        double tau_us{100000.0};  // 100 ms default
        Metavision::timestamp last_t{0};
        bool init{false};

        void filter(double input, Metavision::timestamp t) {
            if (!init) {
                value = input;
                last_t = t;
                init = true;
                return;
            }
            if (t > last_t) {
                const double dt = static_cast<double>(t - last_t);
                const double a = 1.0 - std::exp(-dt / tau_us);
                value += (input - value) * a;
                last_t = t;
            }
            // dt <= 0 (same / out-of-order timestamp): keep the old value,
            // matching jAER LowpassFilter (fac=0) — do NOT adopt the new
            // sample (§一-1.2; same-timestamp packets would otherwise let the
            // last sample overwrite the estimate repeatedly).
        }
        void reset() {
            value = 0.0;
            last_t = 0;
            init = false;
        }
    };

    int width_;
    int height_;
    double cx_{0.0};  // sensor center x (OPT-32)
    double cy_{0.0};  // sensor center y (OPT-32)
    int time_window_us_{10000};  // jAER maxDtThreshold 默认 100000us（此处
                                 // 10000 为有意收紧，10× 差异）；jAER
                                 // useAvgDtEnabled（默认 true，用平均 dt
                                 // 而非最小 recency）未移植
    bool enable_global_mode_{true};
    int min_dt_us_{100};       // jAER minDtThreshold
    int search_distance_{3};   // jAER searchDistance
    int tau_low_ms_{100};      // jAER tauLow

    // Single-channel surface for the raw/degraded fallback.
    std::vector<Metavision::timestamp> surface_;
    // 8-channel (orientation*2 + polarity) surface for the orientation-aware
    // path (jAER NUM_INPUT_TYPES = 4 orientations * 2 polarities).
    std::vector<Metavision::timestamp> ori_surface_;
    std::array<int, kNumDirections> global_hist_{};

    LowPass tx_, ty_, rot_, exp_;

    Metavision::timestamp last_delay_us_{0};
    int last_distance_px_{0};
    float last_speed_pps_{0.0F};
    float last_vx_{0.0F};
    float last_vy_{0.0F};
};

} // namespace gui_algo

#endif // GUI_ALGO_CV_DIRECTION_SELECTIVE_FILTER_H
