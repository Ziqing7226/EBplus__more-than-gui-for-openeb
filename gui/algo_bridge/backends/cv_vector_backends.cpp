// gui/algo_bridge/backends/cv_vector_backends.cpp — result-vector detectors
// (design §3.4). Split from the former algo_backend.cpp monolith.

#include "algo_bridge/algo_backend.h"
#include "algo_bridge/backends/backend_common.h"

#include <cmath>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "algo/cv/hough_line_tracker.h"
#include "algo/cv/hough_circle_tracker.h"
#include "algo/cv/line_segment_detector.h"
#include "algo/cv/orientation_cluster.h"
#include "algo/cv/cluster_lif.h"

using namespace gui::backend_detail;

namespace gui {

// ===========================================================================
// Group C: Result-vector detectors (process returns vector<Result>)
// ===========================================================================

/// HoughLineTracker backend — detected lines as overlay lines.
///
/// Complex algorithm (design §4.3.14): the Hough accumulator is built at ROI
/// dimensions (not sensor dimensions) to bound memory and per-frame scan cost.
/// Events are cropped to ROI-relative coordinates; detected line endpoints are
/// shifted back by the ROI origin so they render at the correct sensor position.
class HoughLineBackend final : public AlgoBackend {
    int sensor_w_{0}, sensor_h_{0};
    ProcessRegion roi_;
    int num_theta_bins_{90};
    int num_rho_bins_{0};
    int threshold_{30};
    // Persisted so rebuild() re-applies it (the algo ctor takes it directly;
    // accumulator_decay_us was removed algo-side as a dead param, §7.3).
    float hough_decay_factor_{0.6F};
    double output_tau_ms_{10.0};
    double favor_vertical_range_deg_{90.0};
    std::unique_ptr<gui_algo::HoughLineTracker> algo_;
    std::vector<Metavision::EventCD> passthrough_;
    std::vector<gui_algo::Event> roi_events_;
    std::vector<gui_algo::HoughLine> last_;
    Preprocessor preproc_;
public:
    HoughLineBackend(int w, int h) : sensor_w_(w), sensor_h_(h) {
        roi_.compute(sensor_w_, sensor_h_);
        preproc_.halve_coords_ = true;
        rebuild();
    }
    void rebuild() {
        const int aw = roi_.enabled ? roi_.rw : sensor_w_;
        const int ah = roi_.enabled ? roi_.rh : sensor_h_;
        preproc_.init(aw, ah);
        const int f = preproc_.factor();
        algo_ = std::make_unique<gui_algo::HoughLineTracker>(
            aw / f, ah / f, num_theta_bins_, num_rho_bins_, threshold_,
            hough_decay_factor_, output_tau_ms_, favor_vertical_range_deg_);
    }
    void set_param(const std::string& k, const std::string& v) override {
        if (preproc_.set_param(k, v)) {
            if (k == "preproc_downsample") rebuild();
            return;
        }
        // Snapshot ROI state BEFORE mutation so the rebuild below only runs
        // when the effective dimensions actually change (audit §五-D4): ROI
        // position moves (x/y) don't change the accumulator size, and
        // apply_global_roi fires 5 set_param calls — rebuilding on each one
        // would clear the accumulator 5 times.
        const bool prev_roi_enabled = roi_.enabled;
        const int prev_roi_rw = roi_.rw;
        const int prev_roi_rh = roi_.rh;
        bool need_rebuild = false;
        bool roi_changed = false;
        if (k == "threshold") { threshold_ = to_i(v); if (algo_) algo_->set_threshold(threshold_); }
        else if (k == "num_theta_bins") { num_theta_bins_ = to_i(v); need_rebuild = true; }
        else if (k == "num_rho_bins") { num_rho_bins_ = to_i(v); need_rebuild = true; }
        else if (k == "hough_decay_factor") {
            hough_decay_factor_ = static_cast<float>(to_d(v));
            if (algo_) algo_->set_hough_decay_factor(hough_decay_factor_);
        }
        else if (k == "output_tau_ms") { output_tau_ms_ = to_d(v); if (algo_) algo_->set_output_tau_ms(output_tau_ms_); }
        else if (k == "favor_vertical_range_deg") { favor_vertical_range_deg_ = to_d(v); if (algo_) algo_->set_favor_vertical_range_deg(favor_vertical_range_deg_); }
        // Phase 2.6: roi_* keys intentionally not handled.
        if (need_rebuild) { roi_.compute(sensor_w_, sensor_h_); rebuild(); }
        else if (roi_changed) {
            const int old_aw = prev_roi_enabled ? prev_roi_rw : sensor_w_;
            const int old_ah = prev_roi_enabled ? prev_roi_rh : sensor_h_;
            roi_.compute(sensor_w_, sensor_h_);
            const int new_aw = roi_.enabled ? roi_.rw : sensor_w_;
            const int new_ah = roi_.enabled ? roi_.rh : sensor_h_;
            if (new_aw != old_aw || new_ah != old_ah) rebuild();
        }
    }
    std::string get_param(const std::string& k) const override {
        auto pp = preproc_.get_param(k); if (!pp.empty()) return pp;
        if (k == "threshold" && algo_) return from_i(algo_->threshold());
        if (k == "num_theta_bins") return from_i(num_theta_bins_);
        if (k == "num_rho_bins") return from_i(num_rho_bins_);
        if (k == "hough_decay_factor" && algo_) return from_d(algo_->hough_decay_factor());
        if (k == "output_tau_ms") return from_d(output_tau_ms_);
        if (k == "favor_vertical_range_deg") return from_d(favor_vertical_range_deg_);
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        const auto* ev = as_events(passthrough_.data());
        std::size_t n = passthrough_.size();
        if (roi_.enabled && roi_.rw > 0 && roi_.rh > 0) {
            crop_to_roi(ev, n, roi_, &preproc_, roi_events_);
            ev = roi_events_.data();
            n = roi_events_.size();
        } else if (preproc_.active() && n > 0) {
            auto [p, m] = preproc_.apply(ev, n);
            roi_events_.assign(p, p + m);
            ev = roi_events_.data();
            n = m;
        }
        gui_algo::EventPacket pkt(ev, n);
        last_ = algo_->process(pkt);
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        // Shift ROI-relative endpoints back to sensor coordinates.
        const int dx = (roi_.enabled && roi_.rw > 0) ? roi_.x0 : 0;
        const int dy = (roi_.enabled && roi_.rh > 0) ? roi_.y0 : 0;
        const int f = preproc_.factor();
        for (const auto& hl : last_) {
            OverlayLine l;
            l.x1 = static_cast<int>(hl.start.x) * f + dx;
            l.y1 = static_cast<int>(hl.start.y) * f + dy;
            l.x2 = static_cast<int>(hl.end.x) * f + dx;
            l.y2 = static_cast<int>(hl.end.y) * f + dy;
            r.lines.push_back(l);
        }
        // Phase 2.6 debug D-2: the Hough θ-ρ aux display was deleted (user
        // decision — it was a jAER-style debug view that was never visible
        // in baseline practice: the window zoom view covered it every frame).
        r.status = "hough_line: " + std::to_string(last_.size()) + " lines" +
                   std::string(roi_.enabled ? " (ROI)" : " (full)");
        return r;
    }
    void reset() override {
        if (algo_) algo_->reset();
        passthrough_.clear(); roi_events_.clear(); last_.clear();
    }
    void set_sensor_dimensions(int w, int h) override {
        sensor_w_ = w; sensor_h_ = h;
        roi_.compute(sensor_w_, sensor_h_); rebuild();
    }
};

/// HoughCircleTracker backend — detected circles as overlay circles.
///
/// Complex algorithm (design §4.3.15): the 3D accumulator (a, b, r) is built
/// at the backend's CURRENT dimensions (set_sensor_dimensions), NOT the raw
/// sensor dimensions. At full sensor scale (e.g. 1280×720) with
/// max_radius=50 the accumulator would be ~42M cells (168 MB) and find_peaks
/// would scan every cell per frame — a memory/CPU blowup that freezes the
/// GUI. The unified-ROI automation (Phase 2.6 debug D-7 default-ROI list)
/// resizes this backend to the ROI window via AlgoInstance::set_unified_roi,
/// restoring the bound (256×144 → ~1.7M cells at f=1). Event cropping and
/// coordinate translation are done by AlgoInstance::push_events; the backend
/// sees the ROI window as its effective sensor.
class HoughCircleBackend final : public AlgoBackend {
    int sensor_w_{0}, sensor_h_{0};
    ProcessRegion roi_;
    // min_radius / accumulator_decay_us were removed algo-side as dead
    // params (§三-32); the ctor no longer takes them.
    int max_radius_{30};
    int threshold_{15};  // jAER default since P9
    // Adaptive defaults: while the user hasn't explicitly set a param,
    // rebuild() derives it from the WORKING resolution (post-ROI,
    // post-downsample) so the detector stays sane across ROI sizes:
    //   max_radius = min(w,h)/8  (45px at full 720p+downsample, 8px in a
    //                            128px downsampled ROI)
    //   threshold  = 0.25 * 2π * max_radius  (≈ a quarter of the circle
    //                            perimeter in votes; ≈ jAER's 15 at r=8)
    bool radius_user_set_{false};
    // jAER params (persisted so rebuild() preserves them)
    float decay_{1.0f};
    int buffer_length_{4000};
    int nr_max_{1};
    bool decay_mode_{true};
    bool loc_depression_{true};
    std::unique_ptr<gui_algo::HoughCircleTracker> algo_;
    std::vector<Metavision::EventCD> passthrough_;
    std::vector<gui_algo::Event> roi_events_;
    std::vector<gui_algo::HoughCircle> last_;
    /// Throttle: only run find_peaks every kMinProcessIntervalUs to avoid
    /// CPU saturation. Between runs the last cached result is returned.
    /// 50ms = 20Hz update, smooth enough for visual tracking.
    static constexpr Metavision::timestamp kMinProcessIntervalUs = 50000;
    Metavision::timestamp last_process_t_{0};
    Preprocessor preproc_;
public:
    HoughCircleBackend(int w, int h) : sensor_w_(w), sensor_h_(h) {
        roi_.compute(sensor_w_, sensor_h_);
        preproc_.halve_coords_ = true;
        rebuild();
    }
    void rebuild() {
        const int aw = roi_.enabled ? roi_.rw : sensor_w_;
        const int ah = roi_.enabled ? roi_.rh : sensor_h_;
        preproc_.init(aw, ah);
        const int f = preproc_.factor();
        const int w = aw / f, h = ah / f;
        // Adaptive defaults for untouched params (see member comments):
        // derive from the working resolution so a small ROI doesn't keep
        // full-sensor parameters (and vice versa).
        if (!radius_user_set_) {
            max_radius_ = std::max(5, std::min(500, std::min(w, h) / 8));
        }
        // threshold: plain jAER value (default 15) — the former adaptive
        // 0.25*2πr default (≈71 at the adaptive radius 45) sat 3-5x above
        // jAER and, combined with the over-decay bug, made live detection
        // nearly impossible (P9).
        algo_ = std::make_unique<gui_algo::HoughCircleTracker>(
            w, h, max_radius_, threshold_,
            decay_, buffer_length_, nr_max_, decay_mode_, loc_depression_);
    }
    void set_param(const std::string& k, const std::string& v) override {
        if (preproc_.set_param(k, v)) {
            if (k == "preproc_downsample") rebuild();
            return;
        }
        // Snapshot ROI state BEFORE mutation so the rebuild below only runs
        // when the effective dimensions actually change (audit §五-D4):
        // apply_global_roi fires 5 set_param calls; rebuilding on each one
        // would clear the 3D accumulator 5 times.
        const bool prev_roi_enabled = roi_.enabled;
        const int prev_roi_rw = roi_.rw;
        const int prev_roi_rh = roi_.rh;
        bool need_rebuild = false;
        bool roi_changed = false;
        // Note: AlgoInstance replays the REGISTRY DEFAULTS through set_param
        // at construction — that must not count as "user set", so the flag
        // only flips when the value differs from the registered default.
        if (k == "radius") { max_radius_ = to_i(v); if (max_radius_ != 30) radius_user_set_ = true; need_rebuild = true; }
        else if (k == "threshold") { threshold_ = to_i(v); if (algo_) algo_->set_threshold(threshold_); }
        else if (k == "decay") { decay_ = static_cast<float>(to_d(v)); if (algo_) algo_->set_decay(decay_); }
        else if (k == "buffer_length") { buffer_length_ = to_i(v); if (algo_) algo_->set_buffer_length(buffer_length_); }
        else if (k == "nr_max") { nr_max_ = to_i(v); if (algo_) algo_->set_nr_max(nr_max_); }
        else if (k == "decay_mode") { decay_mode_ = to_b(v); if (algo_) algo_->set_decay_mode(decay_mode_); }
        else if (k == "loc_depression") { loc_depression_ = to_b(v); if (algo_) algo_->set_loc_depression(loc_depression_); }
        // Phase 2.6: roi_* keys intentionally not handled.
        if (need_rebuild) { roi_.compute(sensor_w_, sensor_h_); rebuild(); }
        else if (roi_changed) {
            const int old_aw = prev_roi_enabled ? prev_roi_rw : sensor_w_;
            const int old_ah = prev_roi_enabled ? prev_roi_rh : sensor_h_;
            roi_.compute(sensor_w_, sensor_h_);
            const int new_aw = roi_.enabled ? roi_.rw : sensor_w_;
            const int new_ah = roi_.enabled ? roi_.rh : sensor_h_;
            if (new_aw != old_aw || new_ah != old_ah) rebuild();
        }
    }
    std::string get_param(const std::string& k) const override {
        auto pp = preproc_.get_param(k); if (!pp.empty()) return pp;
        if (k == "radius") return from_i(max_radius_);
        if (k == "threshold") return from_i(threshold_);
        if (k == "decay" && algo_) return from_d(algo_->decay());
        if (k == "buffer_length" && algo_) return from_i(algo_->buffer_length());
        if (k == "nr_max" && algo_) return from_i(algo_->nr_max());
        if (k == "decay_mode" && algo_) return from_b(algo_->decay_mode());
        if (k == "loc_depression" && algo_) return from_b(algo_->loc_depression());
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        // Throttle (audit §五-G4): EVERY packet feeds the accumulator via
        // accumulate_only(); only the expensive O(W×H×R) find_peaks scan is
        // throttled to ~20Hz. Previously the whole process() call was
        // skipped, which silently dropped the events between scans AND
        // stretched the decay dt (collapsing the accumulator by ~0.2× per
        // kept packet).
        //
        // §11.2-H: pass passthrough_.back().t explicitly to accumulate_only
        // so the algo's last_t_ stays monotonic even when ROI/preproc
        // filtering removes the packet's tail event or empties the packet
        // entirely. Without this, last_t_ stalls on the filtered tail and
        // the next non-empty packet sees an inflated dt, distorting the
        // decay factor (1/(0.0001*decay*dt)).
        const Metavision::timestamp cur_t =
            passthrough_.empty() ? last_process_t_ : passthrough_.back().t;
        const auto* ev = as_events(passthrough_.data());
        std::size_t n = passthrough_.size();
        if (roi_.enabled && roi_.rw > 0 && roi_.rh > 0) {
            crop_to_roi(ev, n, roi_, &preproc_, roi_events_);
            ev = roi_events_.data();
            n = roi_events_.size();
        } else if (preproc_.active() && n > 0) {
            auto [p, m] = preproc_.apply(ev, n);
            roi_events_.assign(p, p + m);
            ev = roi_events_.data();
            n = m;
        }
        gui_algo::EventPacket pkt(ev, n);
        algo_->accumulate_only(pkt, cur_t);
        if (last_process_t_ > 0 && cur_t - last_process_t_ < kMinProcessIntervalUs) {
            return;  // keep last_ cached result
        }
        last_process_t_ = cur_t;
        last_ = algo_->find_peaks();
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        // Shift ROI-relative centers back to sensor coordinates.
        const int dx = (roi_.enabled && roi_.rw > 0) ? roi_.x0 : 0;
        const int dy = (roi_.enabled && roi_.rh > 0) ? roi_.y0 : 0;
        const int f = preproc_.factor();
        for (const auto& c : last_) {
            OverlayCircle oc;
            oc.cx = static_cast<int>(c.center.x) * f + dx;
            oc.cy = static_cast<int>(c.center.y) * f + dy;
            oc.r = static_cast<int>(c.radius) * f;
            r.circles.push_back(oc);
        }
        // Phase 2.6 debug D-2: the per-pixel accumulator aux display was
        // deleted (user decision — never visible in baseline practice).
        r.status = "hough_circle: " + std::to_string(last_.size()) + " circles" +
                   std::string(roi_.enabled ? " (ROI)" : " (full)") +
                   // Effective params may be resolution-adaptive (when the
                   // user hasn't set them) — surface them so the status line
                   // shows what the detector actually runs with.
                   " r=" + std::to_string(max_radius_) +
                   " thr=" + std::to_string(threshold_);
        return r;
    }
    void reset() override {
        if (algo_) algo_->reset();
        passthrough_.clear(); roi_events_.clear(); last_.clear();
        last_process_t_ = 0;
    }
    void set_sensor_dimensions(int w, int h) override {
        sensor_w_ = w; sensor_h_ = h;
        roi_.compute(sensor_w_, sensor_h_); rebuild();
    }
};

/// LineSegmentDetector (ELiSeD) backend — detected segments as overlay lines.
class LineSegmentBackend final : public AlgoBackend {
    gui_algo::LineSegmentDetector algo_;
    std::vector<Metavision::EventCD> passthrough_;
    std::vector<gui_algo::LineSegment> last_;
    RoiFilter roi_;
    std::vector<gui_algo::Event> roi_buf_;
public:
    LineSegmentBackend(int w, int h) : algo_(w, h) { roi_.init(w, h); }
    void set_param(const std::string& k, const std::string& v) override {
        if (roi_.set_param(k, v)) return;
        if (k == "min_length") algo_.set_min_line_length_px(to_i(v));
        else if (k == "gap") algo_.set_max_line_gap_px(to_i(v));
    }
    std::string get_param(const std::string& k) const override {
        auto r = roi_.get_param(k); if (!r.empty()) return r;
        if (k == "min_length") return from_i(algo_.min_line_length_px());
        if (k == "gap") return from_i(algo_.max_line_gap_px());
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        auto [ev, n] = roi_.apply(as_events(passthrough_.data()), passthrough_.size(), roi_buf_);
        gui_algo::EventPacket pkt(ev, n);
        last_ = algo_.process(pkt);
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        for (const auto& seg : last_) {
            OverlayLine l;
            l.x1 = static_cast<int>(seg.start.x); l.y1 = static_cast<int>(seg.start.y);
            l.x2 = static_cast<int>(seg.end.x); l.y2 = static_cast<int>(seg.end.y);
            r.lines.push_back(l);
        }
        r.status = "elised: " + std::to_string(last_.size()) + " segments" +
                   std::string(roi_.region.enabled ? " (ROI)" : "");
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); last_.clear(); }
    void set_sensor_dimensions(int w, int h) override {
        roi_.set_sensor_dimensions(w, h);
        const int ml = algo_.min_line_length_px();
        const int gap = algo_.max_line_gap_px();
        const int age = algo_.max_age_us();
        algo_ = gui_algo::LineSegmentDetector(w, h, ml, gap);
        algo_.set_max_age_us(age);
    }
};

/// OrientationCluster backend — detected orientation clusters as overlay lines.
class OrientationClusterBackend final : public AlgoBackend {
    gui_algo::OrientationCluster algo_;
    std::vector<Metavision::EventCD> passthrough_;
    std::vector<gui_algo::OrientationClusterResult> last_;
    RoiFilter roi_;
    std::vector<gui_algo::Event> roi_buf_;
public:
    OrientationClusterBackend(int w, int h) : algo_(w, h) { roi_.init(w, h); }
    void set_param(const std::string& k, const std::string& v) override {
        if (roi_.set_param(k, v)) return;
        if (k == "dt") algo_.set_dt(static_cast<float>(to_d(v)));
        else if (k == "factor") algo_.set_factor(static_cast<float>(to_d(v)));
        else if (k == "rf_width") algo_.set_rf_width(to_i(v));
        else if (k == "rf_height") algo_.set_rf_height(to_i(v));
        else if (k == "tolerance") algo_.set_tolerance(static_cast<float>(to_d(v)));
        else if (k == "ori") algo_.set_ori(static_cast<float>(to_d(v)));
        else if (k == "neighbor_thr") algo_.set_neighbor_thr(static_cast<float>(to_d(v)));
        else if (k == "thr_gradient") algo_.set_thr_gradient(static_cast<float>(to_d(v)));
        else if (k == "history_factor") algo_.set_history_factor(static_cast<float>(to_d(v)));
        else if (k == "use_opposite_polarity") algo_.set_use_opposite_polarity(to_b(v));
        else if (k == "ori_history_enabled") algo_.set_ori_history_enabled(to_b(v));
        else if (k == "display_length") algo_.set_display_length(to_i(v));
    }
    std::string get_param(const std::string& k) const override {
        auto r = roi_.get_param(k); if (!r.empty()) return r;
        if (k == "dt") return from_d(algo_.dt());
        if (k == "factor") return from_d(algo_.factor());
        if (k == "rf_width") return from_i(algo_.rf_width());
        if (k == "rf_height") return from_i(algo_.rf_height());
        if (k == "tolerance") return from_d(algo_.tolerance());
        if (k == "ori") return from_d(algo_.ori());
        if (k == "neighbor_thr") return from_d(algo_.neighbor_thr());
        if (k == "thr_gradient") return from_d(algo_.thr_gradient());
        if (k == "history_factor") return from_d(algo_.history_factor());
        if (k == "use_opposite_polarity") return from_b(algo_.use_opposite_polarity());
        if (k == "ori_history_enabled") return from_b(algo_.ori_history_enabled());
        if (k == "display_length") return from_i(algo_.display_length());
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        auto [ev, n] = roi_.apply(as_events(passthrough_.data()), passthrough_.size(), roi_buf_);
        gui_algo::EventPacket pkt(ev, n);
        last_ = algo_.process(pkt);
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        for (const auto& c : last_) {
            OverlayLine l;
            l.x1 = static_cast<int>(c.centroid.x); l.y1 = static_cast<int>(c.centroid.y);
            l.x2 = static_cast<int>(c.centroid.x) + static_cast<int>(std::cos(c.orientation) * c.size);
            l.y2 = static_cast<int>(c.centroid.y) + static_cast<int>(std::sin(c.orientation) * c.size);
            r.lines.push_back(l);
        }
        r.status = "orient_cluster: " + std::to_string(last_.size()) + " clusters" +
                   std::string(roi_.region.enabled ? " (ROI)" : "");
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); last_.clear(); }
    void set_sensor_dimensions(int w, int h) override {
        roi_.set_sensor_dimensions(w, h);
        const float dt = algo_.dt();
        const float factor = algo_.factor();
        const int rf = algo_.rf_width();
        const int tl = algo_.display_length();
        algo_ = gui_algo::OrientationCluster(w, h);
        algo_.set_dt(dt);
        algo_.set_factor(factor);
        algo_.set_rf_width(rf);
        algo_.set_display_length(tl);
    }
};

/// ClusterLIF backend — LIF clusters as overlay boxes.
class ClusterLifBackend final : public AlgoBackend {
    gui_algo::ClusterLIF algo_;
    std::vector<Metavision::EventCD> passthrough_;
    std::vector<gui_algo::LifCluster> last_;
    RoiFilter roi_;
    std::vector<gui_algo::Event> roi_buf_;
public:
    ClusterLifBackend(int w, int h) : algo_(w, h) { roi_.init(w, h); }
    void set_param(const std::string& k, const std::string& v) override {
        if (roi_.set_param(k, v)) return;
        if (k == "tau_ms") algo_.set_tau_ms(static_cast<float>(to_d(v)));
        else if (k == "threshold") algo_.set_threshold(static_cast<float>(to_d(v)));
        else if (k == "receptive_field_size_pixels") algo_.set_receptive_field_size_pixels(to_i(v));
        else if (k == "initial_potential_percent") algo_.set_initial_potential_percent(static_cast<float>(to_d(v)));
        else if (k == "jump_after_firing_percent") algo_.set_jump_after_firing_percent(static_cast<float>(to_d(v)));
    }
    std::string get_param(const std::string& k) const override {
        auto r = roi_.get_param(k); if (!r.empty()) return r;
        if (k == "tau_ms") return from_d(algo_.tau_ms());
        if (k == "threshold") return from_d(algo_.threshold());
        if (k == "receptive_field_size_pixels") return from_i(algo_.receptive_field_size_pixels());
        if (k == "initial_potential_percent") return from_d(algo_.initial_potential_percent());
        if (k == "jump_after_firing_percent") return from_d(algo_.jump_after_firing_percent());
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        auto [ev, n] = roi_.apply(as_events(passthrough_.data()), passthrough_.size(), roi_buf_);
        gui_algo::EventPacket pkt(ev, n);
        last_ = algo_.process(pkt);
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        for (const auto& c : last_) {
            OverlayBox box;
            box.x = static_cast<int>(c.position.x) - 5;
            box.y = static_cast<int>(c.position.y) - 5;
            box.w = 10; box.h = 10;
            r.boxes.push_back(box);
        }
        r.status = "lif: " + std::to_string(last_.size()) + " clusters" +
                   std::string(roi_.region.enabled ? " (ROI)" : "");
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); last_.clear(); }
    void set_sensor_dimensions(int w, int h) override {
        roi_.set_sensor_dimensions(w, h);
        const float tau = algo_.tau_ms();
        const float thr = algo_.threshold();
        const float rv = algo_.reset_value();
        const int rf = algo_.receptive_field_size_pixels();
        const float ini = algo_.initial_potential_percent();
        const float jump = algo_.jump_after_firing_percent();
        algo_ = gui_algo::ClusterLIF(w, h, tau, thr, rv, rf, ini, jump);
    }
};


// --- Per-category factory (called by create_algo_backend in backend_factory.cpp)
std::unique_ptr<AlgoBackend> create_cv_vector_backend(const std::string& name,
                                          int width, int height) {
    if (name == "hough_line")                  return std::make_unique<HoughLineBackend>(width, height);
    if (name == "hough_circle")                return std::make_unique<HoughCircleBackend>(width, height);
    if (name == "line_segment")                return std::make_unique<LineSegmentBackend>(width, height);
    if (name == "orientation_cluster")         return std::make_unique<OrientationClusterBackend>(width, height);
    if (name == "cluster_lif")                 return std::make_unique<ClusterLifBackend>(width, height);
    return nullptr;
}

} // namespace gui
