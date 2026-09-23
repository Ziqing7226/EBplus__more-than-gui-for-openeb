// algo/cv/sparse_optical_flow.h — sparse event-based optical flow estimation.
//
// Self-developed (design §4.3.9), inspired by the jAER rbodo optical-flow suite
// and ClusterBasedOpticalFlow. Four modes:
//   LocalPlanes  — local (x,y,t) plane fit (Benoit 2015); velocity = grad(T)/|grad(T)|².
//                  注释定档（audit §一-2.2，有意不改实现）：jAER LocalPlanesFlow
//                  对每个像素施加 50ms 不应期（同一像素 50ms 内只出一个流
//                  向量）；本实现逐事件拟合，输出密度高 1-2 个数量级。这是
//                  输出密度差异而非计算错误。
//   LucasKanade  — ✅ 移植自 jAER LucasKanadeFlow (Benosman 2012).
//                  Per-event flow from an event-count histogram: central
//                  first-order spatial derivatives of the histogram, event
//                  density as the temporal derivative, a structure tensor
//                  over the neighborhood, and eigenvalue-based reliability
//                  gating (reject / 1D normal-flow / full 2D LK solve).
//   BlockMatch   — multi-scale coarse-to-fine pyramid with diamond
//                  (LDSP/SDSP) search; ABMOF/PatchMatchFlow-style block
//                  matching between consecutive accumulation frames on a
//                  downsampled grid.
//   ClusterOF    — centroid-cluster trajectory velocity (reuses §4.3.11 params).
// Output: vector<FlowVector> { x, y, vx, vy, confidence } (vx/vy in px/s).
// Header-only.

#ifndef GUI_ALGO_CV_SPARSE_OPTICAL_FLOW_H
#define GUI_ALGO_CV_SPARSE_OPTICAL_FLOW_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include <opencv2/core.hpp>

#include <metavision/sdk/base/utils/timestamp.h>

#include "algo/common/event.h"
#include "algo/common/event_packet.h"

namespace gui_algo {

/// @brief A sparse optical-flow vector (position + velocity + confidence).
struct FlowVector {
    float x{0.0F};
    float y{0.0F};
    float vx{0.0F};          ///< px/s
    float vy{0.0F};          ///< px/s
    float confidence{0.0F};  ///< [0,1]
    FlowVector() = default;
    FlowVector(float x_, float y_, float vx_, float vy_, float c_)
        : x(x_), y(y_), vx(vx_), vy(vy_), confidence(c_) {}
};

/// @brief Multi-mode sparse optical-flow estimator.
class SparseOpticalFlow {
public:
    enum class Mode { LocalPlanes, LucasKanade, BlockMatch, ClusterOF };

    SparseOpticalFlow(int width, int height, Mode mode = Mode::LocalPlanes)
        : width_(width), height_(height), mode_(mode),
          sae_(static_cast<std::size_t>(width) * height, 0) {}

    void set_mode(Mode m) { mode_ = m; }
    Mode mode() const { return mode_; }

    // LocalPlanes + LucasKanade (shared time window) ----------------------
    void set_time_window_us(int v) { time_window_us_ = clamp_i(v, 1000, 100000); }
    void set_spatial_radius_px(int v) { spatial_radius_px_ = clamp_i(v, 3, 30); }
    void set_min_events_per_cluster(int v) {
        min_events_per_cluster_ = clamp_i(v, 3, 100);
    }
    // LucasKanade ----------------------------------------------------------
    void set_block_size(int v) { block_size_ = clamp_i(v, 4, 64); }
    void set_step(int v) { step_ = clamp_i(v, 1, 32); }
    void set_lk_thr(double v) { lk_thr_ = v < 0.0 ? 0.0 : v; }
    // BlockMatch -----------------------------------------------------------
    void set_downsample_factor(int v) { downsample_factor_ = clamp_i(v, 1, 8); }
    void set_block_match_time_window_us(int v) {
        bm_time_window_us_ = clamp_i(v, 1000, 200000);
    }
    void set_search_radius_px(int v) { search_radius_px_ = clamp_i(v, 1, 16); }
    void set_num_scales(int v) { num_scales_ = clamp_i(v, 1, 5); }
    void set_max_slice_value(int v) { max_slice_value_ = clamp_i(v, 1, 255); }
    void set_valid_pix_occupancy(double v) {
        valid_pix_occupancy_ = v < 0.0 ? 0.0 : (v > 1.0 ? 1.0 : v);
    }
    void set_weight_distance(double v) {
        weight_distance_ = v < 0.0 ? 0.0 : (v > 1.0 ? 1.0 : v);
    }
    void set_max_sad_distance(double v) {
        max_sad_distance_ = v < 0.0 ? 0.0 : (v > 1.0 ? 1.0 : v);
    }
    void set_use_diamond_search(bool v) { use_diamond_search_ = v; }
    void set_grad_thr(double v) { grad_thr_ = v < 0.0 ? 0.0 : v; }
    // ClusterOF ------------------------------------------------------------
    void set_min_track_len(int v) { min_track_len_ = clamp_i(v, 3, 100); }
    void set_cluster_size_px(int v) { cluster_size_px_ = clamp_i(v, 3, 50); }
    void set_cluster_time_us(int v) { cluster_time_us_ = clamp_i(v, 1000, 50000); }
    void set_cluster_ema_alpha(float v) { cluster_ema_alpha_ = v < 0.0F ? 0.0F : (v > 1.0F ? 1.0F : v); }

    int time_window_us() const { return time_window_us_; }
    int spatial_radius_px() const { return spatial_radius_px_; }
    int min_events_per_cluster() const { return min_events_per_cluster_; }
    int block_size() const { return block_size_; }
    int step() const { return step_; }
    double lk_thr() const { return lk_thr_; }
    int downsample_factor() const { return downsample_factor_; }
    int block_match_time_window_us() const { return bm_time_window_us_; }
    int search_radius_px() const { return search_radius_px_; }
    int num_scales() const { return num_scales_; }
    int max_slice_value() const { return max_slice_value_; }
    double valid_pix_occupancy() const { return valid_pix_occupancy_; }
    double weight_distance() const { return weight_distance_; }
    double max_sad_distance() const { return max_sad_distance_; }
    bool use_diamond_search() const { return use_diamond_search_; }
    double grad_thr() const { return grad_thr_; }
    int min_track_len() const { return min_track_len_; }
    float cluster_ema_alpha() const { return cluster_ema_alpha_; }
    int width() const { return width_; }
    int height() const { return height_; }

    /// @brief Estimates flow for a batch of events; appends to @p out.
    void process(const Event* events, std::size_t count,
                 std::vector<FlowVector>& out) {
        switch (mode_) {
            case Mode::LocalPlanes: run_local_planes(events, count, out); break;
            case Mode::LucasKanade: run_lucas_kanade(events, count, out); break;
            case Mode::BlockMatch:  run_block_match(events, count, out); break;
            case Mode::ClusterOF:   run_cluster_of(events, count, out); break;
        }
    }

    /// @brief Convenience overload taking an event packet.
    void process(EventPacket& events, std::vector<FlowVector>& out) {
        process(events.data(), events.size(), out);
    }

    void reset() {
        std::fill(sae_.begin(), sae_.end(), 0);
        cur_.release();
        prev_.release();
        last_match_t_ = 0;
        last_emit_t_ = 0;
        of_clusters_.clear();
        for (auto& dq : lk_ts_) dq.clear();
    }

private:
    static int clamp_i(int v, int lo, int hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }
    std::size_t idx_of(int x, int y) const {
        return static_cast<std::size_t>(y) * width_ + x;
    }

    // --- LocalPlanes: plane fit t = a*x + b*y + c, v = grad/|grad|^2 -----
    void run_local_planes(const Event* events, std::size_t count,
                          std::vector<FlowVector>& out) {
        const Metavision::timestamp win = time_window_us_;
        const int r = spatial_radius_px_;
        lp_xs_.reserve((2 * r + 1) * (2 * r + 1));
        lp_ys_.reserve((2 * r + 1) * (2 * r + 1));
        lp_ts_.reserve((2 * r + 1) * (2 * r + 1));
        for (std::size_t i = 0; i < count; ++i) {
            const Event& e = events[i];
            if (e.x >= width_ || e.y >= height_) continue;
            lp_xs_.clear(); lp_ys_.clear(); lp_ts_.clear();
            auto& xs = lp_xs_;
            auto& ys = lp_ys_;
            auto& ts = lp_ts_;
            for (int dy = -r; dy <= r; ++dy) {
                const int ny = e.y + dy;
                if (ny < 0 || ny >= height_) continue;
                for (int dx = -r; dx <= r; ++dx) {
                    const int nx = e.x + dx;
                    if (nx < 0 || nx >= width_) continue;
                    const Metavision::timestamp lt = sae_[idx_of(nx, ny)];
                    if (lt == 0) continue;
                    const Metavision::timestamp diff = e.t - lt;
                    if (diff < 0 || diff > win) continue;
                    xs.push_back(static_cast<double>(nx));
                    ys.push_back(static_cast<double>(ny));
                    ts.push_back(static_cast<double>(lt));
                }
            }
            if (static_cast<int>(xs.size()) < min_events_per_cluster_) {
                sae_[idx_of(e.x, e.y)] = e.t;
                continue;
            }
            double a = 0.0, b = 0.0, c = 0.0;
            if (!fit_plane(xs, ys, ts, a, b, c)) {
                sae_[idx_of(e.x, e.y)] = e.t;
                continue;
            }
            // jAER LocalPlanesFlow.velFromPar: skip when both spatial
            // gradients are below threshold (flat fit → unreliable velocity).
            // Component-wise check (not a²+b²) so a single strong gradient
            // direction still yields a valid 1D velocity along that axis.
            if (std::fabs(a) < grad_thr_ && std::fabs(b) < grad_thr_) {
                sae_[idx_of(e.x, e.y)] = e.t;
                continue;
            }
            const double denom = a * a + b * b;
            const double vx = (a / denom) * 1e6;   // px/s
            const double vy = (b / denom) * 1e6;
            const double conf = plane_confidence(xs, ys, ts, a, b, c, win);
            if (conf > 0.0) {
                out.push_back(FlowVector(static_cast<float>(e.x),
                                         static_cast<float>(e.y),
                                         static_cast<float>(vx),
                                         static_cast<float>(vy),
                                         static_cast<float>(conf)));
            }
            sae_[idx_of(e.x, e.y)] = e.t;
        }
    }

    static double det3(const double m[3][3]) {
        return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
             - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
             + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    }

    static bool fit_plane(const std::vector<double>& xs,
                          const std::vector<double>& ys,
                          const std::vector<double>& ts,
                          double& a, double& b, double& c) {
        const double n = static_cast<double>(xs.size());
        double sx = 0, sy = 0, st = 0, sxx = 0, syy = 0, sxy = 0, sxt = 0, syt = 0;
        for (std::size_t i = 0; i < xs.size(); ++i) {
            sx += xs[i]; sy += ys[i]; st += ts[i];
            sxx += xs[i] * xs[i]; syy += ys[i] * ys[i];
            sxy += xs[i] * ys[i];
            sxt += xs[i] * ts[i]; syt += ys[i] * ts[i];
        }
        double m[3][3] = {{sxx, sxy, sx}, {sxy, syy, sy}, {sx, sy, n}};
        double r[3] = {sxt, syt, st};
        const double det = det3(m);
        if (std::fabs(det) < 1e-9) return false;
        double m0[3][3] = {{r[0], m[0][1], m[0][2]},
                           {r[1], m[1][1], m[1][2]},
                           {r[2], m[2][1], m[2][2]}};
        double m1[3][3] = {{m[0][0], r[0], m[0][2]},
                           {m[1][0], r[1], m[1][2]},
                           {m[2][0], r[2], m[2][2]}};
        double m2[3][3] = {{m[0][0], m[0][1], r[0]},
                           {m[1][0], m[1][1], r[1]},
                           {m[2][0], m[2][1], r[2]}};
        a = det3(m0) / det;
        b = det3(m1) / det;
        c = det3(m2) / det;
        return true;
    }

    static double plane_confidence(const std::vector<double>& xs,
                                   const std::vector<double>& ys,
                                   const std::vector<double>& ts,
                                   double a, double b, double c,
                                   Metavision::timestamp win) {
        double sse = 0.0;
        for (std::size_t i = 0; i < xs.size(); ++i) {
            const double pred = a * xs[i] + b * ys[i] + c;
            const double d = ts[i] - pred;
            sse += d * d;
        }
        const double rmse = std::sqrt(sse / static_cast<double>(xs.size()));
        const double conf = 1.0 - rmse / static_cast<double>(win);
        return conf < 0.0 ? 0.0 : (conf > 1.0 ? 1.0 : conf);
    }

    // --- LucasKanade: ✅ 移植自 jAER LucasKanadeFlow (Benosman 2012) ------
    // Per-event flow from an event-count histogram (per-pixel per-polarity
    // timestamp deques). Central first-order spatial derivatives of the
    // histogram, event density as the temporal derivative, structure tensor
    // over the neighborhood, eigenvalue-based reliability gating.
    void ensure_lk_ts() {
        const std::size_t n = static_cast<std::size_t>(width_) * height_ * 2;
        if (lk_ts_.size() != n) {
            lk_ts_.assign(n, {});
        }
    }
    std::size_t lk_idx(int x, int y, int p) const {
        return (static_cast<std::size_t>(y) * width_ + x) * 2 + static_cast<std::size_t>(p);
    }
    void prune_lk_deque(std::vector<Metavision::timestamp>& dq,
                        Metavision::timestamp cutoff) {
        auto it = std::lower_bound(dq.begin(), dq.end(), cutoff);
        if (it != dq.begin()) dq.erase(dq.begin(), it);
    }

    void run_lucas_kanade(const Event* events, std::size_t count,
                          std::vector<FlowVector>& out) {
        ensure_lk_ts();
        const Metavision::timestamp win = time_window_us_;
        const int sr = search_radius_px_;   // neighborhood radius (jAER searchDistance; jAER 默认 3)
        const int d = 1;                     // border margin for central first-order
        // 单位清理版时间缩放：jAER 为 it = cnt×40 启发式；此处按窗口归一化
        // 到 px/s（默认 win=20000 时为 cnt×50）。与下方 0.5 中心差分叠加，
        // 默认窗口下本实现幅值 ≈ 2.5× jAER（§二-2.2，有意，非移植 bug）。
        const double scale = 1.0e6 / static_cast<double>(win);  // px/window -> px/s
        for (std::size_t i = 0; i < count; ++i) {
            const Event& e = events[i];
            if (e.x >= width_ || e.y >= height_) continue;
            const int p = e.p & 1;
            const Metavision::timestamp cutoff = (e.t > win) ? (e.t - win) : 0;
            // Add event timestamp to the histogram, pruning the pixel's own
            // deque FIRST: events in the sr+d border band `continue` below
            // without ever reaching the neighborhood prune, so a persistent
            // edge-band pixel's deque grew without bound (unbounded memory
            // in long sessions whose activity hugs the border).
            auto& own = lk_ts_[lk_idx(e.x, e.y, p)];
            own.push_back(e.t);
            prune_lk_deque(own, cutoff);
            // Need margin of sr + d from border for central first-order derivatives.
            if (e.x < sr + d || e.x >= width_ - sr - d ||
                e.y < sr + d || e.y >= height_ - sr - d) {
                continue;
            }
            // Prune old timestamps in the extended neighborhood (sr + d).
            for (int jj = -sr - d; jj <= sr + d; ++jj) {
                for (int ii = -sr - d; ii <= sr + d; ++ii) {
                    auto& dq = lk_ts_[lk_idx(e.x + ii, e.y + jj, p)];
                    prune_lk_deque(dq, cutoff);
                }
            }
            // Accumulate structure tensor over the (2*sr+1) x (2*sr+1) neighborhood.
            // Center pixel index in the flattened neighborhood (matches jAER currPix).
            const int w2 = 2 * sr + 1;
            const int curr = sr * w2 + sr;
            double sx2 = 0.0, sy2 = 0.0, sxy = 0.0, sxt = 0.0, syt = 0.0;
            double center_ix = 0.0, center_iy = 0.0, center_it = 0.0;
            int ii_flat = 0;
            for (int jjj = -sr; jjj <= sr; ++jjj) {
                for (int iii = -sr; iii <= sr; ++iii) {
                    const int nx = e.x + iii;
                    const int ny = e.y + jjj;
                    const double cnt =
                        static_cast<double>(lk_ts_[lk_idx(nx, ny, p)].size());
                    // Central first-order spatial derivatives. 单位清理版：
                    // jAER CentralFiniteDifferenceFirstOrder 无 0.5 因子；
                    // 此处的 0.5 使结构张量 ×0.25（特征值缩小 4 倍，等效
                    // jAER thr=4），与上方时间缩放叠加后幅值 ≈ 2.5× jAER
                    // （§二-2.2，有意，非移植 bug）。
                    const double cnt_xp =
                        static_cast<double>(lk_ts_[lk_idx(nx + 1, ny, p)].size());
                    const double cnt_xm =
                        static_cast<double>(lk_ts_[lk_idx(nx - 1, ny, p)].size());
                    const double cnt_yp =
                        static_cast<double>(lk_ts_[lk_idx(nx, ny + 1, p)].size());
                    const double cnt_ym =
                        static_cast<double>(lk_ts_[lk_idx(nx, ny - 1, p)].size());
                    const double ix = (cnt_xp - cnt_xm) * 0.5;
                    const double iy = (cnt_yp - cnt_ym) * 0.5;
                    const double it = cnt * scale;
                    sx2 += ix * ix;
                    sy2 += iy * iy;
                    sxy += ix * iy;
                    sxt += ix * it;
                    syt += iy * it;
                    if (ii_flat == curr) {
                        center_ix = ix;
                        center_iy = iy;
                        center_it = it;
                    }
                    ++ii_flat;
                }
            }
            // Eigenvalues of structure tensor [[sx2, sxy], [sxy, sy2]].
            const double p_eig = sx2 + sy2;
            const double q_eig = sx2 * sy2 - sxy * sxy;
            const double disc = p_eig * p_eig - 4.0 * q_eig;
            if (disc < 0.0 || !std::isfinite(disc)) continue;
            const double tmp = std::sqrt(disc);
            const double lam1 = (p_eig + tmp) * 0.5;
            const double lam2 = (p_eig - tmp) * 0.5;
            double vx = 0.0, vy = 0.0;
            if (lam1 < lk_thr_ || !std::isfinite(lam1)) {
                // Reject: gradient too small.
                continue;
            } else if (lam2 < lk_thr_) {
                // 1D normal flow (aperture problem): v = -It * gradI / |gradI|^2.
                const double norm2 = center_ix * center_ix + center_iy * center_iy;
                if (norm2 < 1e-12) continue;
                vx = -center_it * center_ix / norm2;
                vy = -center_it * center_iy / norm2;
            } else {
                // Full 2D LK solve: v = S^{-1} * [sxt, syt].
                if (std::fabs(q_eig) < 1e-12) continue;
                vx = (sxy * syt - sy2 * sxt) / q_eig;
                vy = (sxy * sxt - sx2 * syt) / q_eig;
            }
            const double conf = lam1 > 0.0 ? lam2 / lam1 : 0.0;
            // 差异：被门控拒绝的事件不输出（jAER 对拒绝事件输出零向量）。
            out.push_back(FlowVector(static_cast<float>(e.x),
                                     static_cast<float>(e.y),
                                     static_cast<float>(vx),
                                     static_cast<float>(vy),
                                     static_cast<float>(conf)));
        }
    }

    // --- BlockMatch: multi-scale coarse-to-fine with diamond search -----
    // Ported from jAER PatchMatchFlow: numScales-level pyramid, LDSP/SDSP
    // diamond search, normalized SAD with dispersion blend and valid-pixel
    // occupancy gate. Sign convention: result points to the past slice, so
    // motion (velocity) is the negated search offset (C6).
    void run_block_match(const Event* events, std::size_t count,
                         std::vector<FlowVector>& out) {
        ensure_bm_mats();
        const int ds = downsample_factor_;
        for (std::size_t i = 0; i < count; ++i) {
            const Event& e = events[i];
            const int dx = e.x / ds;
            const int dy = e.y / ds;
            if (dx >= 0 && dx < bm_w_ && dy >= 0 && dy < bm_h_) {
                // Saturate at max_slice_value_ (jAER byte-slice semantics) so
                // SAD normalization lands in [0, 1].
                float v = cur_.at<float>(dy, dx) + 1.0F;
                if (v > static_cast<float>(max_slice_value_))
                    v = static_cast<float>(max_slice_value_);
                cur_.at<float>(dy, dx) = v;
                if (e.t > last_match_t_) last_match_t_ = e.t;
            }
        }
        if (last_match_t_ - last_emit_t_ < bm_time_window_us_) return;
        if (prev_.empty()) {
            prev_ = cur_.clone();
            cur_.setTo(0.0F);
            last_emit_t_ = last_match_t_;
            return;
        }
        // Real inter-frame dt (Minor fixed-dt fix).
        const double dt_us = static_cast<double>(last_match_t_ - last_emit_t_);
        if (dt_us <= 0.0) {
            prev_ = cur_.clone();
            cur_.setTo(0.0F);
            last_emit_t_ = last_match_t_;
            return;
        }
        const double dt_s = dt_us * 1e-6;
        // Build pyramid; cap levels by image size so the block fits.
        const int mb = 8;
        const int mstep = 4;
        int avail_levels = 1;
        int cur_w = bm_w_, cur_h = bm_h_;
        while (avail_levels < num_scales_) {
            const int new_w = cur_w / 2;
            const int new_h = cur_h / 2;
            if (new_w <= mb || new_h <= mb) break;
            cur_w = new_w; cur_h = new_h;
            ++avail_levels;
        }
        std::vector<cv::Mat> pyr_cur = build_bm_pyramid(cur_, avail_levels);
        std::vector<cv::Mat> pyr_prev = build_bm_pyramid(prev_, avail_levels);
        const int sr0 = std::max(1, search_radius_px_ / ds);
        for (int by = mb / 2; by < bm_h_ - mb / 2; by += mstep) {
            for (int bx = mb / 2; bx < bm_w_ - mb / 2; bx += mstep) {
                int motion_x = 0, motion_y = 0;  // level-0 (bm frame) coords
                double best_sad = 0.0;
                bool valid = true;
                for (int s = avail_levels - 1; s >= 0; --s) {
                    const cv::Mat& cm = pyr_cur[static_cast<std::size_t>(s)];
                    const cv::Mat& pm = pyr_prev[static_cast<std::size_t>(s)];
                    const int bx_s = bx >> s;
                    const int by_s = by >> s;
                    const int init_x = motion_x >> s;
                    const int init_y = motion_y >> s;
                    const int sr_s = std::max(1, sr0 >> s);
                    int dx = 0, dy = 0;
                    // Search center = -init (search offset points to the past
                    // slice, motion is the opposite).
                    const double sad = use_diamond_search_
                        ? diamond_search(cm, pm, bx_s, by_s,
                                         -init_x, -init_y, sr_s, mb, dx, dy)
                        : exhaustive_search(cm, pm, bx_s, by_s,
                                            -init_x, -init_y, sr_s, mb, dx, dy);
                    if (sad >= max_sad_distance_) { valid = false; break; }
                    // motion at this scale = init - dx (init is coarser motion,
                    // -dx is the refinement). Convert back to level-0 coords.
                    // *(1<<s), not << s: left-shifting a negative value is UB
                    // in C++17 (§四-低13).
                    motion_x = (init_x - dx) * (1 << s);
                    motion_y = (init_y - dy) * (1 << s);
                    best_sad = sad;
                }
                if (!valid) continue;
                // C6: velocity = -search_offset * ds / dt. motion_x already
                // holds the negated search offset (motion past→current).
                const double vx = static_cast<double>(motion_x) * ds / dt_s;
                const double vy = static_cast<double>(motion_y) * ds / dt_s;
                double conf = 1.0 - best_sad;
                if (conf < 0.0) conf = 0.0;
                if (conf > 1.0) conf = 1.0;
                out.push_back(FlowVector(static_cast<float>(bx * ds),
                                         static_cast<float>(by * ds),
                                         static_cast<float>(vx),
                                         static_cast<float>(vy),
                                         static_cast<float>(conf)));
            }
        }
        prev_ = cur_.clone();
        cur_.setTo(0.0F);
        last_emit_t_ = last_match_t_;
    }

    // Normalized SAD between block at (bx,by) in cur and (bx+off_x,by+off_y)
    // in prev. Returns a value in [0,1] (M16), or a large sentinel for OOB
    // candidates (Minor OOB fix) or insufficient valid pixels (M16).
    double bm_eval_sad(const cv::Mat& cur, const cv::Mat& prev,
                       int bx, int by, int off_x, int off_y, int mb) const {
        const int half = mb / 2;
        const int block_area = (2 * half + 1) * (2 * half + 1);
        const int min_valid =
            static_cast<int>(valid_pix_occupancy_ * static_cast<double>(block_area));
        double sum_dist = 0.0;
        int valid_cur = 0, valid_prev = 0;
        for (int dy = -half; dy <= half; ++dy) {
            for (int dx = -half; dx <= half; ++dx) {
                const int cxp = bx + dx, cyp = by + dy;
                const int pxp = cxp + off_x, pyp = cyp + off_y;
                if (cxp < 0 || cxp >= cur.cols || cyp < 0 || cyp >= cur.rows)
                    return std::numeric_limits<double>::max();
                if (pxp < 0 || pxp >= prev.cols || pyp < 0 || pyp >= prev.rows)
                    return std::numeric_limits<double>::max();
                const double cv = static_cast<double>(cur.at<float>(cyp, cxp));
                const double pv = static_cast<double>(prev.at<float>(pyp, pxp));
                sum_dist += std::fabs(cv - pv);
                if (cv > 0.0) ++valid_cur;
                if (pv > 0.0) ++valid_prev;
            }
        }
        if (valid_cur < min_valid || valid_prev < min_valid)
            return std::numeric_limits<double>::max();
        const double sad_normalizer =
            1.0 / (static_cast<double>(block_area) * static_cast<double>(max_slice_value_));
        const double final_distance = sad_normalizer * (
            sum_dist * weight_distance_ +
            std::fabs(static_cast<double>(valid_cur - valid_prev)) * (1.0 - weight_distance_));
        return final_distance;
    }

    // LDSP (8 points at distance 2 + center) then SDSP (4 points at distance
    // 1 + center). Returns best refinement (best_dx, best_dy) relative to the
    // search center and the corresponding normalized SAD. (M15)
    double diamond_search(const cv::Mat& cur, const cv::Mat& prev,
                          int bx, int by, int center_x, int center_y,
                          int sr, int mb, int& best_dx, int& best_dy) const {
        static const int LDSP[9][2] = {{0,-2},{-1,-1},{1,-1},{-2,0},{0,0},
                                        {2,0},{-1,1},{1,1},{0,2}};
        static const int SDSP[5][2] = {{0,-1},{-1,0},{0,0},{1,0},{0,1}};
        int cur_dx = 0, cur_dy = 0;
        double cur_sad = bm_eval_sad(cur, prev, bx, by, center_x, center_y, mb);
        bool improved = true;
        int iters = 0;
        while (improved && iters < 16) {
            improved = false;
            ++iters;
            for (const auto& off : LDSP) {
                const int ndx = cur_dx + off[0];
                const int ndy = cur_dy + off[1];
                if (ndx < -sr || ndx > sr || ndy < -sr || ndy > sr) continue;
                const double sad = bm_eval_sad(cur, prev, bx, by,
                                                center_x + ndx, center_y + ndy, mb);
                if (sad < cur_sad) {
                    cur_sad = sad; cur_dx = ndx; cur_dy = ndy;
                    improved = true;
                }
            }
        }
        for (const auto& off : SDSP) {
            const int ndx = cur_dx + off[0];
            const int ndy = cur_dy + off[1];
            if (ndx < -sr || ndx > sr || ndy < -sr || ndy > sr) continue;
            const double sad = bm_eval_sad(cur, prev, bx, by,
                                            center_x + ndx, center_y + ndy, mb);
            if (sad < cur_sad) {
                cur_sad = sad; cur_dx = ndx; cur_dy = ndy;
            }
        }
        best_dx = cur_dx;
        best_dy = cur_dy;
        return cur_sad;
    }

    // Exhaustive search fallback (M15). Returns best refinement and SAD.
    double exhaustive_search(const cv::Mat& cur, const cv::Mat& prev,
                             int bx, int by, int center_x, int center_y,
                             int sr, int mb, int& best_dx, int& best_dy) const {
        double best_sad = bm_eval_sad(cur, prev, bx, by, center_x, center_y, mb);
        int bx_best = 0, by_best = 0;
        for (int sdy = -sr; sdy <= sr; ++sdy) {
            for (int sdx = -sr; sdx <= sr; ++sdx) {
                if (sdx == 0 && sdy == 0) continue;
                const double sad = bm_eval_sad(cur, prev, bx, by,
                                                center_x + sdx, center_y + sdy, mb);
                if (sad < best_sad) {
                    best_sad = sad; bx_best = sdx; by_best = sdy;
                }
            }
        }
        best_dx = bx_best;
        best_dy = by_best;
        return best_sad;
    }

    std::vector<cv::Mat> build_bm_pyramid(const cv::Mat& base, int levels) const {
        std::vector<cv::Mat> pyr;
        pyr.push_back(base);
        for (int i = 1; i < levels; ++i) {
            pyr.push_back(downsample_2x(pyr.back()));
        }
        return pyr;
    }

    cv::Mat downsample_2x(const cv::Mat& src) const {
        const int w = src.cols / 2;
        const int h = src.rows / 2;
        if (w < 1 || h < 1) return src.clone();
        cv::Mat dst(h, w, src.type(), 0.0F);
        for (int y = 0; y < h; ++y) {
            const float* sr0 = src.ptr<float>(y * 2);
            const float* sr1 = src.ptr<float>(y * 2 + 1);
            float* dr = dst.ptr<float>(y);
            for (int x = 0; x < w; ++x) {
                dr[x] = (sr0[x * 2] + sr0[x * 2 + 1] +
                         sr1[x * 2] + sr1[x * 2 + 1]) * 0.25F;
            }
        }
        return dst;
    }

    void ensure_bm_mats() {
        const int ds = downsample_factor_;
        const int w = (width_ + ds - 1) / ds;
        const int h = (height_ + ds - 1) / ds;
        if (bm_w_ != w || bm_h_ != h || cur_.empty()) {
            bm_w_ = w; bm_h_ = h;
            cur_ = cv::Mat::zeros(h, w, CV_32FC1);
            prev_ = cv::Mat();
        }
    }

    // --- ClusterOF: centroid trajectory velocity -------------------------
    struct OFCluster {
        float cx{0.0F}, cy{0.0F};
        Metavision::timestamp last_t{0};
        float prev_cx{0.0F}, prev_cy{0.0F};
        Metavision::timestamp prev_t{0};
        int mass{0};
        int history_len{0};
    };

    void run_cluster_of(const Event* events, std::size_t count,
                        std::vector<FlowVector>& out) {
        const float cs = static_cast<float>(cluster_size_px_);
        for (std::size_t i = 0; i < count; ++i) {
            const Event& e = events[i];
            int best = -1;
            float best_d2 = cs * cs;
            for (int k = 0; k < static_cast<int>(of_clusters_.size()); ++k) {
                const OFCluster& cl = of_clusters_[k];
                if (e.t - cl.last_t > cluster_time_us_) continue;
                const float ddx = cl.cx - static_cast<float>(e.x);
                const float ddy = cl.cy - static_cast<float>(e.y);
                const float d2 = ddx * ddx + ddy * ddy;
                if (d2 < best_d2) { best_d2 = d2; best = k; }
            }
            if (best < 0) {
                OFCluster cl;
                cl.cx = static_cast<float>(e.x);
                cl.cy = static_cast<float>(e.y);
                cl.last_t = e.t;
                cl.mass = 1;
                cl.history_len = 1;
                // Cap the cluster count (§四-待验证2): noise bursts could
                // grow the list into the thousands and make every event's
                // association O(n). Evict the weakest (then oldest) cluster.
                if (of_clusters_.size() >= kMaxOFClusters) {
                    std::size_t victim = 0;
                    for (std::size_t k = 1; k < of_clusters_.size(); ++k) {
                        const OFCluster& a = of_clusters_[k];
                        const OFCluster& b = of_clusters_[victim];
                        if (a.mass < b.mass ||
                            (a.mass == b.mass && a.last_t < b.last_t)) {
                            victim = k;
                        }
                    }
                    of_clusters_[victim] = cl;
                } else {
                    of_clusters_.push_back(cl);
                }
            } else {
                OFCluster& cl = of_clusters_[best];
                // Record previous position before EMA update.
                if (cl.history_len == 0 ||
                    e.t - cl.prev_t >= cluster_time_us_) {
                    cl.prev_cx = cl.cx;
                    cl.prev_cy = cl.cy;
                    cl.prev_t = cl.last_t;
                }
                const float a = cluster_ema_alpha_;  // 质心 EMA 系数，对应 jAER RCT locationMixingFactor（非 spatialSmoothingFactor，见成员注释）
                cl.cx = cl.cx * (1.0F - a) + static_cast<float>(e.x) * a;
                cl.cy = cl.cy * (1.0F - a) + static_cast<float>(e.y) * a;
                cl.last_t = e.t;
                ++cl.mass;
                ++cl.history_len;
                if (cl.history_len >= min_track_len_ && e.t > cl.prev_t) {
                    const double dt_s =
                        static_cast<double>(e.t - cl.prev_t) * 1e-6;
                    if (dt_s > 0.0) {
                        const double vx =
                            static_cast<double>(cl.cx - cl.prev_cx) / dt_s;
                        const double vy =
                            static_cast<double>(cl.cy - cl.prev_cy) / dt_s;
                        out.push_back(FlowVector(cl.cx, cl.cy,
                                                 static_cast<float>(vx),
                                                 static_cast<float>(vy), 1.0F));
                    }
                }
            }
        }
        // Prune stale clusters.
        const Metavision::timestamp last_t = count > 0 ? events[count - 1].t : 0;
        std::vector<OFCluster> kept;
        kept.reserve(of_clusters_.size());
        for (auto& cl : of_clusters_) {
            if (last_t - cl.last_t <= cluster_time_us_ * 4) kept.push_back(cl);
        }
        of_clusters_.swap(kept);
    }

    int width_;
    int height_;
    Mode mode_;

    // LocalPlanes + LucasKanade params ------------------------------------
    int time_window_us_{20000};  // jAER maxDtThreshold 实为默认 50000（此处 20000 为有意收紧）
    int spatial_radius_px_{8};
    int min_events_per_cluster_{10};
    int block_size_{16};
    int step_{8};
    double lk_thr_{1.0};   // jAER LucasKanadeFlow eigenvalue threshold

    // BlockMatch params ---------------------------------------------------
    int downsample_factor_{2};
    int bm_time_window_us_{20000};
    int search_radius_px_{4};  // jAER searchDistance 默认 3（此处 4 为有意）
    int num_scales_{3};
    int max_slice_value_{15};
    double valid_pix_occupancy_{0.01};
    double weight_distance_{0.95};
    double max_sad_distance_{0.5};
    bool use_diamond_search_{true};
    double grad_thr_{1e-6};
    int bm_w_{0};
    int bm_h_{0};
    cv::Mat cur_;
    cv::Mat prev_;
    Metavision::timestamp last_match_t_{0};
    Metavision::timestamp last_emit_t_{0};

    // ClusterOF params (reuse §4.3.11 defaults) ---------------------------
    int min_track_len_{5};
    int cluster_size_px_{10};
    int cluster_time_us_{5000};
    // 质心 EMA 系数，对应 jAER RectangularClusterTracker locationMixingFactor
    // （数值巧合一致）。jAER ClusterBasedOpticalFlow 的 spatialSmoothingFactor
    // 是网格流场平滑系数，本模式未实现该网格平滑，此前注释张冠李戴（§二-2.2）。
    float cluster_ema_alpha_{0.05F};
    static constexpr std::size_t kMaxOFClusters = 256;  // ClusterOF 簇数上限
    std::vector<OFCluster> of_clusters_;

    // Shared time surface (Surface of Active Events) ----------------------
    std::vector<Metavision::timestamp> sae_;

    // LocalPlanes: reused per-event (x,y,t) sample buffers (OPT-24).
    std::vector<double> lp_xs_;
    std::vector<double> lp_ys_;
    std::vector<double> lp_ts_;

    // LucasKanade: per-pixel per-polarity timestamp deques (event-count
    // histogram). Lazily allocated by ensure_lk_ts() when LK mode runs.
    std::vector<std::vector<Metavision::timestamp>> lk_ts_;
};

} // namespace gui_algo

#endif // GUI_ALGO_CV_SPARSE_OPTICAL_FLOW_H
