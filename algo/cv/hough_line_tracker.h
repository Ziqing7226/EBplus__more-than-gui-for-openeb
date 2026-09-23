// algo/cv/hough_line_tracker.h — event-driven incremental Hough line tracking.
//
// ✅ 移植自 jAER HoughLineTracker (net.sf.jaer.eventprocessing.tracking.
// HoughLineTracker)。事件驱动增量霍夫直线变换：维护 2D 累加器 (ρ, θ)，θ ∈ [0, π)；
// 逐事件对每个 θ 角度箱计算 ρ = x·cos(θ) + y·sin(θ) 并投票；每包将整个累加器乘以
// hough_decay_factor（与 jAER 一致）；取**全局单峰**（jAER rhoMaxIndex/thetaMaxIndex
// 语义），ρ 用一阶低通、θ 用 180° 周期角低通平滑（jAER rhoFilter +
// AngularLowpassFilter, tauMs=10），输出一条贯穿画面的平滑直线。
// favorVerticalAngleRangeDeg 限制 θ 只在垂直方向 ±range 内投票（jAER
// allowedThetaNumber，默认 90° = 全范围）。对应设计 §4.3.14。
//
// 与 jAER 的差异：
//   * 衰减时机：jAER 先投票再衰减；本实现先衰减再投票（P1 前即如此，维持）。
//   * ρ 未中心化（jAER 投票前先减 sx2/sy2 质心）。
//   * 输出角度为线方向角（θ+90°）；jAER 输出法线角 θ。
//   * 低通用连续时间常数 τ（set_sample_dt 推进），jAER 为 ms 时间戳直调——
//     离散化公式一致（fac = dt/τ clamp [0,1]）。
// Header-only.

#ifndef GUI_ALGO_CV_HOUGH_LINE_TRACKER_H
#define GUI_ALGO_CV_HOUGH_LINE_TRACKER_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

#include <metavision/sdk/base/utils/timestamp.h>

#include "algo/common/event.h"
#include "algo/common/event_packet.h"
#include "algo/common/filter/lowpass.h"

namespace gui_algo {

/// @brief Detected Hough line segment (single smoothed line, jAER semantics).
struct HoughLine {
    cv::Point2f start;
    cv::Point2f end;
    float angle{0.0f};    ///< Orientation in degrees, [0, 180).
    int track_id{-1};     ///< Unused since the P1 single-line rework (kept for API).
};

/// @brief Event-driven incremental Hough line tracker, ported from jAER.
///
/// Maintains a 2D accumulator (ρ, θ). Each event votes for ρ at every θ bin.
/// The accumulator decays per packet. The SINGLE global maximum is low-pass
/// filtered (ρ linear, θ circular with a 180° period) and emitted as one
/// image-spanning line segment — jAER's lane/line-tracking output semantics.
class HoughLineTracker {
public:
    HoughLineTracker(int width, int height,
                     int num_theta_bins = 90,
                     int num_rho_bins = 0,
                     int threshold = 30,
                     float hough_decay_factor = 0.6F,
                     double output_tau_ms = 10.0,
                     double favor_vertical_range_deg = 90.0)
        : width_(width), height_(height),
          num_theta_bins_(num_theta_bins),
          threshold_(threshold),
          hough_decay_factor_(hough_decay_factor),
          output_tau_s_(std::max(0.001, output_tau_ms) * 1e-3),
          favor_vertical_range_deg_(
              std::min(90.0, std::max(5.0, favor_vertical_range_deg))) {
        if (num_theta_bins_ < 1) num_theta_bins_ = 1;
        rebuild(num_rho_bins);
        apply_output_tau();
    }

    /// @brief Processes an event packet; returns the single smoothed line
    ///        (empty until the accumulator maximum reaches the threshold).
    std::vector<HoughLine> process(const EventPacket& packet) {
        std::vector<HoughLine> result;
        if (packet.empty()) return result;
        const Metavision::timestamp cur_t = packet[packet.size() - 1].t;
        if (last_t_ >= 0) {
            apply_decay();
        }
        last_t_ = cur_t;
        for (const Event& e : packet) {
            if (e.x >= width_ || e.y >= height_) continue;
            accumulate(e.x, e.y);
        }
        // jAER filterPacket tail: single global max -> rho/theta low-pass.
        int ti = 0, ri = 0;
        if (!global_max(ti, ri)) return result;
        const double dt_s = last_filter_t_ < 0
            ? 0.0
            : std::max(0.0, static_cast<double>(cur_t - last_filter_t_) * 1e-6);
        last_filter_t_ = cur_t;
        rho_lp_.set_sample_dt(dt_s);
        theta_lp_.set_sample_dt(dt_s);
        const double rho_px = (ri + 0.5) * static_cast<double>(rho_step_) -
                              static_cast<double>(rho_max_);
        const double theta_deg = 180.0 * ti / num_theta_bins_;
        const double rho_f = rho_lp_.process(rho_px);
        const double theta_f = theta_lp_.process(theta_deg);
        result.push_back(to_segment(rho_f, theta_f));
        return result;
    }

    // Parameter accessors ---------------------------------------------------
    int num_theta_bins() const { return num_theta_bins_; }
    int num_rho_bins() const { return num_rho_bins_; }
    int threshold() const { return threshold_; }
    /// @brief Read-only access to the θ-ρ accumulator (θ major, ρ minor).
    const std::vector<float>& accum() const { return accum_; }
    void set_num_theta_bins(int v) {
        if (v < 1) v = 1;
        if (v == num_theta_bins_) return;
        num_theta_bins_ = v;
        rebuild(num_rho_bins_);
    }
    void set_num_rho_bins(int v) {
        if (v < 0) v = 0;
        rebuild(v);
    }
    void set_threshold(int v) { threshold_ = v; }
    float hough_decay_factor() const { return hough_decay_factor_; }
    void set_hough_decay_factor(float v) {
        hough_decay_factor_ = v < 0.0F ? 0.0F : (v > 1.0F ? 1.0F : v);
    }
    double output_tau_ms() const { return output_tau_s_ * 1e3; }
    void set_output_tau_ms(double v) {
        output_tau_s_ = std::max(0.001, v) * 1e-3;
        apply_output_tau();
    }
    double favor_vertical_range_deg() const { return favor_vertical_range_deg_; }
    void set_favor_vertical_range_deg(double v) {
        favor_vertical_range_deg_ = std::min(90.0, std::max(5.0, v));
        compute_allowed_theta();
    }
    /// Smoothed outputs (jAER getRhoPixelsFiltered / getThetaDegFiltered).
    double rho_filtered() const { return rho_lp_.value(); }
    double theta_filtered_deg() const { return theta_lp_.value(); }

    void reset() {
        std::fill(accum_.begin(), accum_.end(), 0.0f);
        last_t_ = -1;
        last_filter_t_ = -1;
        rho_lp_.reset();
        theta_lp_.reset();
    }

private:
    /// Circular one-pole low-pass for an angle with period 180° (jAER
    /// AngularLowpassFilter semantics: the update step is the SHORTEST
    /// signed distance crossing the 0/180 cut, and the state wraps into
    /// [0, period)).
    class AngularLowpass {
    public:
        void set_sample_dt(double dt) { dt_ = std::max(0.0, dt); }
        void set_tau_s(double tau) { tau_ = std::max(1e-6, tau); }
        void reset() { val_ = 0.0; init_ = false; }
        double value() const { return val_; }
        double process(double x) {
            if (!init_) { val_ = x; init_ = true; return val_; }
            // Shortest signed distance x - val_ on the 180° circle.
            double d = std::fmod(x - val_, kPeriod);
            if (d > kPeriod * 0.5) d -= kPeriod;
            if (d < -kPeriod * 0.5) d += kPeriod;
            double fac = dt_ / tau_;
            if (fac > 1.0) fac = 1.0;
            val_ += d * fac;
            if (val_ >= kPeriod) val_ -= kPeriod;
            if (val_ < 0.0) val_ += kPeriod;
            return val_;
        }
    private:
        static constexpr double kPeriod = 180.0;
        double val_{0.0};
        double dt_{0.033};
        double tau_{0.01};
        bool init_{false};
    };

    void rebuild(int num_rho_bins_hint) {
        cos_.assign(static_cast<std::size_t>(num_theta_bins_), 0.0f);
        sin_.assign(static_cast<std::size_t>(num_theta_bins_), 0.0f);
        for (int i = 0; i < num_theta_bins_; ++i) {
            const double th = kPi * i / num_theta_bins_;
            cos_[static_cast<std::size_t>(i)] = static_cast<float>(std::cos(th));
            sin_[static_cast<std::size_t>(i)] = static_cast<float>(std::sin(th));
        }
        const double diag = std::sqrt(static_cast<double>(width_) * width_ +
                                      static_cast<double>(height_) * height_);
        rho_max_ = static_cast<float>(diag);
        if (num_rho_bins_hint > 0) {
            num_rho_bins_ = num_rho_bins_hint;
        } else {
            num_rho_bins_ = std::max(1, static_cast<int>(2.0 * diag));
        }
        rho_step_ = (2.0f * rho_max_) / static_cast<float>(num_rho_bins_);
        accum_.assign(static_cast<std::size_t>(num_theta_bins_) *
                          static_cast<std::size_t>(num_rho_bins_),
                      0.0f);
        last_t_ = -1;
        last_filter_t_ = -1;
        rho_lp_.reset();
        theta_lp_.reset();
        compute_allowed_theta();
    }

    /// jAER getAllowedThetaNumber: round(range/180 * nTheta); voting is
    /// restricted to [0, allowed) and (nTheta-allowed, nTheta) — theta=0 is
    /// a vertical line in this (and jAER's) convention.
    void compute_allowed_theta() {
        allowed_theta_ = std::max(1, static_cast<int>(std::lround(
            favor_vertical_range_deg_ / 180.0 * num_theta_bins_)));
        if (allowed_theta_ > num_theta_bins_) allowed_theta_ = num_theta_bins_;
    }

    inline std::size_t idx(int ti, int ri) const {
        return static_cast<std::size_t>(ti) * static_cast<std::size_t>(num_rho_bins_) +
               static_cast<std::size_t>(ri);
    }

    /// @brief Incremental Hough vote within the allowed theta band (jAER
    /// addEvent: two loops around the vertical theta=0 / theta=π ends).
    void accumulate(int x, int y) {
        const float xf = static_cast<float>(x);
        const float yf = static_cast<float>(y);
        auto vote = [&](int ti) {
            const float rho = xf * cos_[static_cast<std::size_t>(ti)] +
                              yf * sin_[static_cast<std::size_t>(ti)];
            int ri = static_cast<int>((rho + rho_max_) / rho_step_);
            if (ri < 0) ri = 0;
            else if (ri >= num_rho_bins_) ri = num_rho_bins_ - 1;
            accum_[idx(ti, ri)] += 1.0f;
        };
        for (int ti = 0; ti < allowed_theta_; ++ti) vote(ti);
        for (int ti = num_theta_bins_ - allowed_theta_ + 1; ti < num_theta_bins_; ++ti) {
            vote(ti);
        }
    }

    /// @brief Per-packet multiplicative decay (jAER decayAccumArray style):
    /// multiply every accumulator cell by hough_decay_factor_ once per packet.
    void apply_decay() {
        const float f = hough_decay_factor_;
        for (float& v : accum_) v *= f;
    }

    /// @brief Single global maximum over the allowed theta band (jAER
    /// rhoMaxIndex/thetaMaxIndex). Returns false below the threshold.
    bool global_max(int& out_ti, int& out_ri) const {
        float best = -1.0f;
        int bt = 0, br = 0;
        for (int ti = 0; ti < num_theta_bins_; ++ti) {
            const bool near_zero = ti < allowed_theta_;
            const bool near_pi = ti >= num_theta_bins_ - allowed_theta_ + 1;
            if (!near_zero && !near_pi) continue;
            const std::size_t base = static_cast<std::size_t>(ti) * num_rho_bins_;
            for (int ri = 0; ri < num_rho_bins_; ++ri) {
                const float v = accum_[base + static_cast<std::size_t>(ri)];
                if (v > best) { best = v; bt = ti; br = ri; }
            }
        }
        if (best < static_cast<float>(threshold_)) return false;
        out_ti = bt;
        out_ri = br;
        return true;
    }

    /// @brief Converts smoothed (ρ, θ°) into a line segment spanning the image.
    HoughLine to_segment(double rho, double theta_deg) const {
        const double th = kPi * theta_deg / 180.0;
        const double cos_t = std::cos(th);
        const double sin_t = std::sin(th);
        HoughLine hl;
        if (std::abs(sin_t) > std::abs(cos_t)) {
            const double y0 = rho / sin_t;
            const double y1 = (rho - static_cast<double>(width_) * cos_t) / sin_t;
            hl.start = cv::Point2f(0.0f, static_cast<float>(y0));
            hl.end = cv::Point2f(static_cast<float>(width_), static_cast<float>(y1));
        } else {
            const double x0 = rho / cos_t;
            const double x1 = (rho - static_cast<double>(height_) * sin_t) / cos_t;
            hl.start = cv::Point2f(static_cast<float>(x0), 0.0f);
            hl.end = cv::Point2f(static_cast<float>(x1), static_cast<float>(height_));
        }
        double deg = std::fmod(theta_deg + 90.0, 180.0);
        if (deg < 0.0) deg += 180.0;
        hl.angle = static_cast<float>(deg);
        return hl;
    }

    static constexpr double kPi = 3.14159265358979323846;

    int width_;
    int height_;
    int num_theta_bins_;
    int num_rho_bins_{1};
    int threshold_;
    float hough_decay_factor_{0.6F};  // jAER houghDecayFactor default (per-packet)
    double output_tau_s_{0.01};       // jAER tauMs=10 output low-pass
    double favor_vertical_range_deg_{90.0};
    int allowed_theta_{0};
    float rho_max_{0.0f};
    float rho_step_{1.0f};
    std::vector<float> cos_;
    std::vector<float> sin_;
    std::vector<float> accum_;
    Metavision::timestamp last_t_{-1};
    Metavision::timestamp last_filter_t_{-1};
    LowPassFilter rho_lp_;   // jAER rhoFilter
    AngularLowpass theta_lp_;  // jAER AngularLowpassFilter(180)

    /// jAER HoughLineTracker.setTauMs sends tau to BOTH output filters
    /// (rhoFilter.setTauMs + thetaFilter.setTauMs). The port used to store
    /// output_tau_s_ without ever forwarding it, leaving the GUI knob dead
    /// (rho ran at the 10 Hz ctor default, theta at a hardcoded 10 ms).
    void apply_output_tau() {
        rho_lp_.set_cutoff_hz(1.0 / (2.0 * M_PI * output_tau_s_));
        theta_lp_.set_tau_s(output_tau_s_);
    }
};

} // namespace gui_algo

#endif // GUI_ALGO_CV_HOUGH_LINE_TRACKER_H
