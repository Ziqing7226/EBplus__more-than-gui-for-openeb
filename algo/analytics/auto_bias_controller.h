// algo/analytics/auto_bias_controller.h — Dual-loop adaptive bias control.
//
// Design §4.4.6 (reworked 2026-08-21). Two decoupled control loops on the
// two hardware-tunable sensitivity biases (bias_diff_on / bias_diff_off —
// the only writable biases, integers):
//
//   common mode  : total event rate  r = r_on + r_off  kept inside
//                  [rate_min, rate_max] — above the cap both biases rise
//                  (less sensitive), below the floor both fall;
//   differential : polarity balance  b = (r_on - r_off) / r  kept inside
//                  ±balance_tol — only the DOMINANT polarity's bias rises
//                  (bias_diff_on gates ON events, bias_diff_off gates OFF;
//                  self-effect dominates cross-effect, arXiv:2501.18788).
//
// No PID gains: the hardware itself is the integrator (each command moves
// the bias and stays until the next command), so a plain deadband + step
// law with integer output is the whole controller. Robustness against the
// bias→rate response lag: act only after kActivateTicks consecutive
// violating ticks, then observe for kHoldTicks before acting again.
// Control tick is 30 Hz (kTickUs); rates are measured over a sliding
// kWindowUs window of per-batch polarity counts. Header-only, no Qt.
//
// Tuning (2026-08-21): fast-convergence profile — max_step default 32,
// step gain 16, 2 violating ticks to arm, 6-tick hold (~200 ms). Overshoot
// protection is the deadband structure itself (steps shrink as the error
// approaches the band edge) plus the hold, not a small step cap.
//
// Homing (2026-08-21): while BOTH constraints are satisfied (rate inside
// the band, polarity balance within tolerance), the controller emits a
// home command every kHomeIntervalTicks (~1 s). The controller does not
// know the register values, so the command carries no deltas; the
// hardware side (BiasApplier::home) steps each bias by HALF the remaining
// distance (clamped) toward the caller-configured targets — the
// Prophesee factory default 0, the DAVIS reference operating points
// (1535/1025), or the DVXplorer contrast defaults (9/9) — and no-ops at
// the target. Any violation immediately suspends homing and hands control
// back to the correction loops.

#ifndef GUI_ALGO_ANALYTICS_AUTO_BIAS_CONTROLLER_H
#define GUI_ALGO_ANALYTICS_AUTO_BIAS_CONTROLLER_H

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <utility>

namespace gui_algo {

/// @brief A bias adjustment command (integer deltas, applied verbatim by
///        the hardware side which clamps to the LL bias range).
struct BiasCommand {
    int delta_on{0};    ///< Delta for bias_diff_on (positive = less sensitive)
    int delta_off{0};   ///< Delta for bias_diff_off
    bool active{false}; ///< True when this command requests a real adjustment
    /// @brief Home command: step both biases toward the hardware-side home
    ///        targets (half the remaining distance per command; the caller
    ///        configures the targets). delta_on/delta_off are unused —
    ///        the hardware side knows the current register values.
    bool home{false};
};

/// @brief Dual-loop adaptive camera bias controller (rate band + polarity
///        balance), 30 Hz control tick, integer bias deltas.
class AutoBiasController {
public:
    /// Measured-window length and the minimum span before the first
    /// decision (avoids acting on a near-empty window after (re)enable).
    static constexpr std::int64_t kWindowUs = 400'000;
    static constexpr std::int64_t kMinSpanUs = 200'000;
    /// Control tick (30 Hz).
    static constexpr std::int64_t kTickUs = 33'333;
    /// Consecutive violating ticks before acting / observation ticks after.
    static constexpr int kActivateTicks = 2;
    static constexpr int kHoldTicks = 6;
    /// Homing cadence: one bias unit toward 0 every ~1 s while BOTH
    /// constraints are satisfied.
    static constexpr int kHomeIntervalTicks = 30;
    /// Polarity-balance deadband (normalized ON-OFF difference).
    static constexpr double kBalanceTol = 0.15;
    /// Default per-command bias-delta cap (also the homing step ceiling).
    static constexpr int kDefaultMaxStep = 32;

    /// @brief Constructs the controller.
    /// @param rate_min_mev,rate_max_mev Target total-rate band in Mev/s
    ///        (max has no artificial ceiling — the hardware bias range is
    ///        the real limit).
    /// @param max_step Maximum per-command bias delta, [1, 100].
    AutoBiasController(float rate_min_mev = 1.0F, float rate_max_mev = 50.0F,
                       int max_step = kDefaultMaxStep)
        : max_step_(clamp_step(max_step)) {
        set_rate_bounds(rate_min_mev, rate_max_mev);
    }

    /// @brief Sets the target rate band. The min is floored at 0.05; the
    ///        pair is rejected (previous band kept) when it would not form
    ///        a valid lo < hi band.
    bool set_rate_bounds(float lo_mev, float hi_mev) {
        const float lo = std::max(lo_mev, 0.05F);
        if (!(hi_mev > lo)) return false;
        rate_min_ = lo;
        rate_max_ = hi_mev;
        return true;
    }
    float rate_min_mev() const { return rate_min_; }
    float rate_max_mev() const { return rate_max_; }

    void set_max_step(int s) { max_step_ = clamp_step(s); }
    int max_step() const { return max_step_; }

    /// @brief Feeds one batch increment (counts of ON / OFF events whose
    ///        timestamps all fall at @p t_us or just before). O(1).
    void accumulate(std::int64_t t_us, std::uint32_t n_on, std::uint32_t n_off) {
        if (win_.empty() || t_us > win_.back().t) {
            win_.push_back({t_us, n_on, n_off});
        } else { // non-monotonic batch (source seek/loop) — fold into the
            auto& last = win_.back();  // newest entry; the window stays sorted
            last.n_on += n_on;
            last.n_off += n_off;
        }
        acc_on_ += n_on;
        acc_off_ += n_off;
        prune(t_us);
    }

    /// @brief One control tick. Returns the command (inactive while the
    ///        window is cold, inside both deadbands, or during the
    ///        post-action hold).
    BiasCommand update(std::int64_t now_us) {
        prune(now_us);
        // Hold: observe the effect of the last command before acting again.
        if (hold_ticks_ > 0) {
            --hold_ticks_;
            return {};
        }
        if (!window_ready()) {
            viol_ticks_ = 0;
            return {};
        }
        const double span_s =
            static_cast<double>(now_us - win_.front().t) * 1.0e-6;
        const double r_on = acc_on_ * 1.0e-6 / span_s;
        const double r_off = acc_off_ * 1.0e-6 / span_s;
        const double total = r_on + r_off;

        int d_on = 0, d_off = 0;
        // Common mode: total rate inside the band (deadband = the band).
        if (total > rate_max_) {
            const double e = (total - rate_max_) / rate_max_;
            const int s = step(e);
            d_on += s;
            d_off += s;
        } else if (total < rate_min_) {
            const double e = (rate_min_ - total) / rate_min_;
            const int s = step(e);
            d_on -= s;
            d_off -= s;
        }
        // Differential mode: only meaningful with enough events to trust
        // the polarity ratio (a handful of events says nothing).
        if (acc_on_ + acc_off_ >= 64) {
            const double b = (r_on - r_off) / total;
            if (b > kBalanceTol) {
                d_on += step(std::abs(b) - kBalanceTol);
            } else if (b < -kBalanceTol) {
                d_off += step(std::abs(b) - kBalanceTol);
            }
        }

        const bool needs_action = d_on != 0 || d_off != 0;
        if (!needs_action) {
            viol_ticks_ = 0;
            // Both constraints satisfied: slowly home toward the factory
            // default (0). Suspended during the post-action hold (the early
            // return above) and after every correction (home_ticks_ reset).
            if (++home_ticks_ >= kHomeIntervalTicks) {
                home_ticks_ = 0;
                BiasCommand cmd;
                cmd.active = true;
                cmd.home = true;
                return cmd;
            }
            return {};
        }
        // Persistency filter: act only on sustained violations so a single
        // noisy window cannot kick the biases.
        if (++viol_ticks_ < kActivateTicks) return {};
        viol_ticks_ = 0;
        hold_ticks_ = kHoldTicks;
        home_ticks_ = 0;
        // Drop the measurement window: every sample in it predates the bias
        // change, so acting on it again after the hold would stack steps on
        // stale data and overshoot. Fresh post-action data re-arms in
        // kMinSpanUs.
        win_.clear();
        acc_on_ = 0;
        acc_off_ = 0;
        BiasCommand cmd;
        cmd.delta_on = std::clamp(d_on, -max_step_, max_step_);
        cmd.delta_off = std::clamp(d_off, -max_step_, max_step_);
        cmd.active = true;
        return cmd;
    }

    /// @brief Measured rates over the current window (0 before it fills).
    double rate_on_mev() const { return window_ready() ? measured(true) : 0.0; }
    double rate_off_mev() const { return window_ready() ? measured(false) : 0.0; }
    bool window_ready() const {
        return !win_.empty() &&
               win_.back().t - win_.front().t >= kMinSpanUs;
    }

    /// @brief Resets all state (disable / re-enable / source change).
    void reset() {
        win_.clear();
        acc_on_ = 0;
        acc_off_ = 0;
        viol_ticks_ = 0;
        hold_ticks_ = 0;
        home_ticks_ = 0;
    }

private:
    struct Inc {
        std::int64_t t;
        std::uint32_t n_on, n_off;
    };

    static int clamp_step(int s) { return std::clamp(s, 1, 100); }
    /// Step law: proportional to the normalized violation, always ≥1 (an
    /// integer bias write of 0 is a no-op), capped by max_step_.
    int step(double violation) const {
        const double v = std::max(0.0, violation);
        return std::clamp(static_cast<int>(std::lround(16.0 * v)) + 1, 1, max_step_);
    }
    void prune(std::int64_t now_us) {
        while (!win_.empty() && now_us - win_.front().t > kWindowUs) {
            acc_on_ -= win_.front().n_on;
            acc_off_ -= win_.front().n_off;
            win_.pop_front();
        }
    }
    double measured(bool on) const {
        const double span_s =
            static_cast<double>(win_.back().t - win_.front().t) * 1.0e-6;
        if (span_s <= 0.0) return 0.0;
        const double n = on ? acc_on_ : acc_off_;
        return n * 1.0e-6 / span_s;
    }

    float rate_min_{1.0F};
    float rate_max_{10.0F};
    int max_step_{8};
    std::deque<Inc> win_;
    std::uint64_t acc_on_{0}, acc_off_{0};
    int viol_ticks_{0};
    int hold_ticks_{0};
    int home_ticks_{0};
};

} // namespace gui_algo

#endif // GUI_ALGO_ANALYTICS_AUTO_BIAS_CONTROLLER_H
