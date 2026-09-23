// gui/app/bias_applier.h — hardware side of the AutoBias controller.
//
// Owns the I_LL_Biases interaction for auto_bias (§4.4.6 rework): locates
// the two writable sensitivity axes, snapshots them on attach, applies
// integer deltas with clamping to the hardware range, and restores the
// snapshot on disable. Axis selection (Phase 5): attach() matches the
// "diff_on"/"diff_off" substrings (Prophesee + DAVIS, same convention as
// the calibration wizard's LCD noise-floor override); attach_axes() binds
// exact names — the DVXplorer contrast_on/contrast_off thresholds. Per-axis
// delta signs accommodate polarity-inverted hardware (DAVIS OFF axis,
// both DVXplorer contrast axes). apply() re-READS the current register
// value before each write, so a manual edit in the Biases panel silently
// becomes the new baseline — the controller never fights the user.
// Header-only.

#ifndef GUI_APP_BIAS_APPLIER_H
#define GUI_APP_BIAS_APPLIER_H

#include <algorithm>
#include <string>
#include <utility>

#include <metavision/hal/facilities/i_ll_biases.h>

namespace gui {

class BiasApplier {
public:
    enum class Status {
        Ok,        ///< Both deltas applied in full.
        Clamped,   ///< Applied, but at least one bias hit its range limit.
        NoBias,    ///< Sensor exposes neither requested axis — not attachable.
        Error,     ///< Facility call failed (device went away, …).
    };

    /// @brief Locates the diff biases (substring match — Prophesee + DAVIS),
    ///        reads their ranges and snapshots the current values. Returns
    ///        false when the sensor does not expose them (the caller should
    ///        keep auto_bias inactive).
    bool attach(Metavision::I_LL_Biases* biases) {
        return attach_impl(biases, "diff_on", "diff_off", false);
    }

    /// @brief Same, binding EXACT bias names — the DVXplorer
    ///        contrast_on/contrast_off thresholds.
    bool attach_axes(Metavision::I_LL_Biases* biases, const std::string& on_name,
                     const std::string& off_name) {
        return attach_impl(biases, on_name, off_name, true);
    }

    bool attached() const { return biases_ != nullptr; }

    /// @brief Sets the homing destination for the two diff biases.
    ///        Prophesee diff biases are relative offsets with factory default
    ///        0, so homing toward 0 is correct there (the default). DAVIS
    ///        biases are absolute operating points with non-zero reference
    ///        defaults — the caller supplies those after attach().
    void set_home_targets(int on, int off) { home_on_ = on; home_off_ = off; }

    /// @brief Sets the sign of the OFF-axis delta. On Prophesee, increasing
    ///        bias_diff_off suppresses OFF events (+1, the default). On
    ///        DAVIS346 the polarity is inverted — measured on hardware:
    ///        higher diff_off yields MORE OFF events — so DAVIS passes -1
    ///        and every OFF-axis delta is negated.
    void set_off_delta_sign(int sign) { off_sign_ = (sign < 0) ? -1 : 1; }

    /// @brief Sets the sign of the ON-axis delta. +1 everywhere as of the
    ///        25981fd hardware correction: on the DVXplorer a HIGHER
    ///        contrast threshold yields FEWER events, so the controller's
    ///        "raise the bias to reduce the rate" maps to +1 too (an
    ///        earlier revision negated both DVX axes and was reverted on
    ///        hardware). The OFF axis differs (DAVIS -1, see above).
    void set_on_delta_sign(int sign) { on_sign_ = (sign < 0) ? -1 : 1; }

    /// @brief Applies integer deltas: reads the CURRENT register values,
    ///        adds the deltas, clamps to the hardware range, writes back.
    Status apply(int delta_on, int delta_off) {
        if (!biases_) return Status::NoBias;
        delta_on *= on_sign_;
        delta_off *= off_sign_;
        try {
            const int cur_on = biases_->get(name_on_);
            const int cur_off = biases_->get(name_off_);
            const int target_on = std::clamp(cur_on + delta_on, lo_on_, hi_on_);
            const int target_off = std::clamp(cur_off + delta_off, lo_off_, hi_off_);
            if (target_on != cur_on) biases_->set(name_on_, target_on);
            if (target_off != cur_off) biases_->set(name_off_, target_off);
            const bool clamped =
                target_on != cur_on + delta_on || target_off != cur_off + delta_off;
            return clamped ? Status::Clamped : Status::Ok;
        } catch (const std::exception&) {
            return Status::Error;
        }
    }

    /// @brief Moves both diff biases toward their homing targets (0 by
    ///        default — the Prophesee factory default; see
    ///        set_home_targets()). The per-bias step is HALF the remaining
    ///        distance, clipped to [1, @p cap]: far from the target it
    ///        converges fast (binary-search decay), near it slows to single
    ///        units, and it can never overshoot past the target. A step that
    ///        large may push the event rate out of band — the correction
    ///        loops pull it back within one hold cycle, and homing is
    ///        suspended while they do.
    ///        Returns true when a register actually changed (already at the
    ///        target → false, no write).
    bool home(int cap) {
        if (!biases_) return false;
        try {
            const int cur_on = biases_->get(name_on_);
            const int cur_off = biases_->get(name_off_);
            const int tgt_on = step_toward(cur_on, home_on_, cap, lo_on_, hi_on_);
            const int tgt_off = step_toward(cur_off, home_off_, cap, lo_off_, hi_off_);
            if (tgt_on == cur_on && tgt_off == cur_off) return false;
            if (tgt_on != cur_on) biases_->set(name_on_, tgt_on);
            if (tgt_off != cur_off) biases_->set(name_off_, tgt_off);
            return true;
        } catch (const std::exception&) {
            return false;
        }
    }

    /// @brief Writes the attach-time snapshot back. Returns false when the
    ///        facility calls fail (device already gone).
    bool restore() {
        if (!biases_) return false;
        try {
            biases_->set(name_on_, saved_on_);
            biases_->set(name_off_, saved_off_);
            return true;
        } catch (const std::exception&) {
            return false;
        }
    }

    void detach() {
        biases_ = nullptr;
        name_on_.clear();
        name_off_.clear();
    }

    /// Shared attach: @p exact selects exact-name binding (contrast axes)
    /// vs substring matching (diff biases). Resets the per-axis signs — a
    /// fresh attach starts from the Prophesee convention until the caller
    /// overrides for inverted hardware.
    bool attach_impl(Metavision::I_LL_Biases* biases, const std::string& match_on,
                     const std::string& match_off, bool exact) {
        const auto matches = [&exact, &match_on, &match_off](const std::string& name,
                                                             bool on_axis) {
            const std::string& want = on_axis ? match_on : match_off;
            return exact ? (name == want) : (name.find(want) != std::string::npos);
        };
        detach();
        on_sign_ = 1;
        off_sign_ = 1;
        if (!biases) return false;
        try {
            std::string name_on, name_off;
            int lo_on = 0, hi_on = 0, lo_off = 0, hi_off = 0;
            int cur_on = 0, cur_off = 0;
            for (const auto& [name, value] : biases->get_all_biases()) {
                if (name_on.empty() && matches(name, true)) {
                    Metavision::LL_Bias_Info info;
                    if (!biases->get_bias_info(name, info)) continue;
                    const auto range = info.get_bias_range();
                    if (range.second <= range.first) continue;
                    name_on = name; lo_on = range.first; hi_on = range.second;
                    cur_on = value;
                } else if (name_off.empty() && matches(name, false)) {
                    Metavision::LL_Bias_Info info;
                    if (!biases->get_bias_info(name, info)) continue;
                    const auto range = info.get_bias_range();
                    if (range.second <= range.first) continue;
                    name_off = name; lo_off = range.first; hi_off = range.second;
                    cur_off = value;
                }
            }
            if (name_on.empty() || name_off.empty()) return false;
            biases_ = biases;
            name_on_ = std::move(name_on);
            name_off_ = std::move(name_off);
            lo_on_ = lo_on; hi_on_ = hi_on;
            lo_off_ = lo_off; hi_off_ = hi_off;
            saved_on_ = cur_on;
            saved_off_ = cur_off;
            return true;
        } catch (const std::exception&) {
            return false;
        }
    }

    /// Half the distance from @p v to @p target, clipped to [1, cap],
    /// applied toward @p target and clamped to the writable range [lo, hi]
    /// (the target may sit outside the range — then we stop at the nearest
    /// limit).
    static int step_toward(int v, int target, int cap, int lo, int hi) {
        if (v == target) return v;  // already home — the min-step must not kick in
        const int dist = std::abs(target - v);
        const int step = std::clamp(dist / 2, 1, cap);
        int next = v > target ? v - step : v + step;
        if (v < target && next > target) next = target;
        if (v > target && next < target) next = target;
        return std::clamp(next, lo, hi);
    }

    Metavision::I_LL_Biases* biases_{nullptr};
    std::string name_on_, name_off_;
    int lo_on_{0}, hi_on_{0}, lo_off_{0}, hi_off_{0};
    int saved_on_{0}, saved_off_{0};
    int home_on_{0}, home_off_{0};
    int on_sign_{1};
    int off_sign_{1};
};

} // namespace gui

#endif // GUI_APP_BIAS_APPLIER_H
