// gui/algo_bridge/backends/cv_backends.cpp — in-place filters + overlay detectors
// (design §3.4). Split from the former algo_backend.cpp monolith.

#include "algo_bridge/algo_backend.h"
#include "algo_bridge/backends/backend_common.h"

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "algo/cv/hot_pixel_filter.h"
#include "algo/cv/optical_gyro.h"
#include "algo/cv/object_tracker.h"
#include "algo/cv/corner_detector.h"
#include "algo/cv/blob_detector.h"
#include "algo/cv/sparse_optical_flow.h"
#include "algo/cv/dense_optical_flow.h"
#include "algo/cv/dl_optical_flow.h"

using namespace gui::backend_detail;

namespace gui {

// ===========================================================================
// Group A: In-place event filters (compact / modify events)
// ===========================================================================

/// HotPixelFilter backend — learns hot-pixel mask + compacts events.
class HotPixelFilterBackend final : public AlgoBackend {
    gui_algo::HotPixelFilter algo_;
    std::vector<Metavision::EventCD> buf_;
    RoiFilter roi_;
    std::vector<gui_algo::Event> roi_buf_;
    std::size_t last_kept_{0};
public:
    HotPixelFilterBackend(int w, int h) : algo_(w, h) { roi_.init(w, h); }
    void set_param(const std::string& k, const std::string& v) override {
        if (roi_.set_param(k, v)) return;
        if (k == "learning_window_s") algo_.set_learning_window_s(to_d(v));
        else if (k == "enable_fpn_correction") algo_.set_enable_fpn_correction(to_b(v));
        else if (k == "fpn_alpha") algo_.set_fpn_alpha(to_d(v));
        else if (k == "fpn_mixing_factor") algo_.set_fpn_mixing_factor(to_d(v));
    }
    std::string get_param(const std::string& k) const override {
        auto r = roi_.get_param(k); if (!r.empty()) return r;
        if (k == "learning_window_s") return from_d(algo_.learning_window_s());
        if (k == "enable_fpn_correction") return from_b(algo_.enable_fpn_correction());
        if (k == "fpn_alpha") return from_d(algo_.fpn_alpha());
        if (k == "fpn_mixing_factor") return from_d(algo_.fpn_mixing_factor());
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        buf_.assign(b, e);
        auto [ev_c, n] = roi_.apply(as_events(buf_.data()), buf_.size(), roi_buf_);
        auto* ev = const_cast<gui_algo::Event*>(ev_c);
        algo_.learn(ev, n);
        last_kept_ = algo_.process(ev, n);
        // pull_result reads from buf_: if roi_.apply() filtered into another
        // buffer (roi_buf_ or Preprocessor::buf_), copy the compacted result
        // back; otherwise the in-place compact already happened in buf_.
        if (ev_c != as_events(buf_.data())) {
            buf_.assign(reinterpret_cast<const Metavision::EventCD*>(ev_c),
                        reinterpret_cast<const Metavision::EventCD*>(ev_c + last_kept_));
        } else {
            buf_.resize(last_kept_);
        }
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events.assign(buf_.data(), buf_.data() + last_kept_);
        r.status = "hot_pixel: " + std::to_string(algo_.hot_pixel_count()) + " hot px, kept " +
                   std::to_string(last_kept_) + std::string(roi_.region.enabled ? " (ROI)" : "");
        return r;
    }
    void reset() override { algo_.reset(); buf_.clear(); roi_buf_.clear(); last_kept_ = 0; }
    std::pair<const Metavision::EventCD*, std::size_t> output_span() const override {
        return {buf_.data(), last_kept_};
    }
    void set_sensor_dimensions(int w, int h) override {
        roi_.set_sensor_dimensions(w, h);
        const double lw = algo_.learning_window_s();
        const bool fpn = algo_.enable_fpn_correction();
        const double fa = algo_.fpn_alpha();
        const double fm = algo_.fpn_mixing_factor();
        algo_ = gui_algo::HotPixelFilter(w, h);
        algo_.set_learning_window_s(lw);
        algo_.set_enable_fpn_correction(fpn);
        algo_.set_fpn_alpha(fa);
        algo_.set_fpn_mixing_factor(fm);
    }
};

/// OpticalGyro (EIS) backend — modifies event coordinates in place.
class OpticalGyroBackend final : public AlgoBackend {
    gui_algo::OpticalGyro algo_;
    std::vector<Metavision::EventCD> buf_;
    RoiFilter roi_;
    std::vector<gui_algo::Event> roi_buf_;
public:
    OpticalGyroBackend(int w, int h) : algo_(w, h) { roi_.init(w, h); }
    void set_param(const std::string& k, const std::string& v) override {
        if (roi_.set_param(k, v)) return;
        if (k == "stabilize") algo_.set_stabilization_strength(to_b(v) ? 1.0F : 0.0F);
        else if (k == "smoothing_window_ms") algo_.set_smoothing_window_ms(static_cast<float>(to_d(v)));
        else if (k == "rotation_enabled") algo_.set_rotation_enabled(to_b(v));
    }
    std::string get_param(const std::string& k) const override {
        auto r = roi_.get_param(k); if (!r.empty()) return r;
        if (k == "stabilize") return from_b(algo_.stabilization_strength() > 0.0F);
        if (k == "smoothing_window_ms") return from_d(algo_.smoothing_window_ms());
        if (k == "rotation_enabled") return from_b(algo_.rotation_enabled());
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        buf_.assign(b, e);
        auto [ev_c, n] = roi_.apply(as_events(buf_.data()), buf_.size(), roi_buf_);
        auto* ev = const_cast<gui_algo::Event*>(ev_c);
        gui_algo::MutableEventPacket pkt(ev, n);
        algo_.process(pkt);
        // pull_result reads from buf_: if roi_.apply() filtered into another
        // buffer, copy the processed events back; otherwise they are in buf_.
        if (ev_c != as_events(buf_.data())) {
            buf_.assign(reinterpret_cast<const Metavision::EventCD*>(ev_c),
                        reinterpret_cast<const Metavision::EventCD*>(ev_c + n));
        } else {
            buf_.resize(n);
        }
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = buf_;
        const auto m = algo_.smoothed_motion();
        // Translation vector (jAER OpticalGyro.drawVector):
        // arrow from chip center showing cumulative shift.
        const int cx = algo_.width() / 2;
        const int cy = algo_.height() / 2;
        const float scale = 5.0F;  // px/px for visibility
        OverlayLine tl;
        tl.x1 = cx; tl.y1 = cy;
        tl.x2 = cx + static_cast<int>(m.dx * scale);
        tl.y2 = cy + static_cast<int>(m.dy * scale);
        r.lines.push_back(tl);
        // Rotation indicator (jAER rotationAngle): an arc segment.
        if (std::fabs(m.dtheta) > 1e-3F) {
            const int R = 50;
            const float a0 = -static_cast<float>(kPiF) * 0.25F;
            const float a1 = a0 + m.dtheta * static_cast<float>(kPiF) / 180.0F * 2.0F;
            OverlayLine rl;
            rl.x1 = cx + static_cast<int>(std::cos(a0) * R);
            rl.y1 = cy + static_cast<int>(std::sin(a0) * R);
            rl.x2 = cx + static_cast<int>(std::cos(a1) * R);
            rl.y2 = cy + static_cast<int>(std::sin(a1) * R);
            r.lines.push_back(rl);
        }
        // Motion text overlay.
        OverlayText t;
        t.x = 10; t.y = 20;
        t.text = "EIS trans=(" + std::to_string(static_cast<int>(m.dx)) + "," +
                  std::to_string(static_cast<int>(m.dy)) + ") rot=" +
                  std::to_string(static_cast<int>(m.dtheta)) + "deg";
        r.texts.push_back(t);
        r.status = "EIS: shift=(" + std::to_string(m.dx) + "," +
                   std::to_string(m.dy) + ") rot=" + std::to_string(m.dtheta) + "deg" +
                   std::string(roi_.region.enabled ? " (ROI)" : "");
        return r;
    }
    void reset() override { algo_.reset(); buf_.clear(); roi_buf_.clear(); }
    std::pair<const Metavision::EventCD*, std::size_t> output_span() const override {
        return {buf_.data(), buf_.size()};
    }
    void set_sensor_dimensions(int w, int h) override {
        roi_.set_sensor_dimensions(w, h);
        const float str = algo_.stabilization_strength();
        const float sw = algo_.smoothing_window_ms();
        const bool rot = algo_.rotation_enabled();
        algo_ = gui_algo::OpticalGyro(w, h);
        algo_.set_stabilization_strength(str);
        algo_.set_smoothing_window_ms(sw);
        algo_.set_rotation_enabled(rot);
    }
};

// ===========================================================================
// Group B: Overlay detectors (process events, produce overlay data)
// ===========================================================================

/// ObjectTracker backend — tracked objects as overlay boxes + trajectories.
class ObjectTrackerBackend final : public AlgoBackend {
    gui_algo::ObjectTracker algo_;
    std::vector<Metavision::EventCD> passthrough_;
    RoiFilter roi_;
    std::vector<gui_algo::Event> roi_buf_;
public:
    ObjectTrackerBackend(int w, int h) : algo_(w, h) { roi_.init(w, h); }
    void set_param(const std::string& k, const std::string& v) override {
        if (roi_.set_param(k, v)) return;
        if (k == "cluster_size_fraction") algo_.set_cluster_size_fraction(to_d(v));
        else if (k == "max_clusters") algo_.set_max_clusters(to_i(v));
        else if (k == "cluster_time_us") algo_.set_cluster_time_us(to_i(v));
        else if (k == "min_cluster_events") algo_.set_min_cluster_events(to_i(v));
        else if (k == "max_lost_age_s") algo_.set_max_lost_age_s(to_d(v));
        else if (k == "enable_velocity_prediction") algo_.set_enable_velocity_prediction(to_b(v));
        else if (k == "location_mixing_factor") algo_.set_location_mixing_factor(static_cast<float>(to_d(v)));
        else if (k == "predictive_velocity_factor") algo_.set_predictive_velocity_factor(static_cast<float>(to_d(v)));
        else if (k == "mass_decay_tau_us") algo_.set_mass_decay_tau_us(to_i(v));
        else if (k == "threshold_mass_for_visible") algo_.set_threshold_mass_for_visible(static_cast<float>(to_d(v)));
    }
    std::string get_param(const std::string& k) const override {
        auto r = roi_.get_param(k); if (!r.empty()) return r;
        if (k == "cluster_size_fraction") return from_d(algo_.cluster_size_fraction());
        if (k == "max_clusters") return from_i(algo_.max_clusters());
        if (k == "cluster_time_us") return from_i(algo_.cluster_time_us());
        if (k == "min_cluster_events") return from_i(algo_.min_cluster_events());
        if (k == "max_lost_age_s") return from_d(algo_.max_lost_age_s());
        if (k == "enable_velocity_prediction") return from_b(algo_.enable_velocity_prediction());
        if (k == "location_mixing_factor") return from_d(algo_.location_mixing_factor());
        if (k == "predictive_velocity_factor") return from_d(algo_.predictive_velocity_factor());
        if (k == "mass_decay_tau_us") return from_i(algo_.mass_decay_tau_us());
        if (k == "threshold_mass_for_visible") return from_d(algo_.threshold_mass_for_visible());
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        auto [ev, n] = roi_.apply(as_events(passthrough_.data()), passthrough_.size(), roi_buf_);
        algo_.process(ev, n);
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        const auto& objs = algo_.objects();
        r.boxes.reserve(objs.size());
        r.lines.reserve(objs.size());
        r.texts.reserve(objs.size());
        r.trajectories.reserve(objs.size());
        for (const auto& o : objs) {
            if (!o.visible) continue;
            OverlayBox box;
            box.x = o.bbox.x; box.y = o.bbox.y;
            box.w = o.bbox.width; box.h = o.bbox.height;
            box.id = o.id;
            r.boxes.push_back(box);
            // Velocity arrow (jAER Cluster.drawVelocityVector).
            OverlayLine v;
            v.x1 = static_cast<int>(o.x);
            v.y1 = static_cast<int>(o.y);
            // Scale velocity (px/s) to a visible length (assume 30ms window).
            v.x2 = v.x1 + static_cast<int>(o.vx * 0.03F);
            v.y2 = v.y1 + static_cast<int>(o.vy * 0.03F);
            r.lines.push_back(v);
            // Trajectory (jAER ClusterPath).
            if (!o.trajectory.empty()) {
                OverlayTrajectory tr;
                tr.id = o.id;
                tr.points.reserve(o.trajectory.size());
                for (const auto& p : o.trajectory) {
                    tr.points.emplace_back(static_cast<int>(p.x),
                                            static_cast<int>(p.y));
                }
                r.trajectories.push_back(tr);
            }
            OverlayText t;
            t.x = o.bbox.x; t.y = o.bbox.y > 12 ? o.bbox.y - 12 : 0;
            t.text = "#" + std::to_string(o.id) + " v=(" +
                     std::to_string(static_cast<int>(o.vx)) + "," +
                     std::to_string(static_cast<int>(o.vy)) + ")";
            r.texts.push_back(t);
        }
        r.status = "tracker: " + std::to_string(objs.size()) + " objects" +
                   std::string(roi_.region.enabled ? " (ROI)" : "");
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); }
    void set_sensor_dimensions(int w, int h) override {
        roi_.set_sensor_dimensions(w, h);
        // Rebuild sensor-sized internal state, re-applying the tuned params
        // (the auto-ROI resize hits the live instance).
        const double frac = algo_.cluster_size_fraction();
        const int ct = algo_.cluster_time_us();
        const int mce = algo_.min_cluster_events();
        const double mla = algo_.max_lost_age_s();
        const bool evp = algo_.enable_velocity_prediction();
        const float lmf = algo_.location_mixing_factor();
        const float pvf = algo_.predictive_velocity_factor();
        const int tau = algo_.mass_decay_tau_us();
        const float tmv = algo_.threshold_mass_for_visible();
        const int mc = algo_.max_clusters();
        algo_ = gui_algo::ObjectTracker(w, h);
        algo_.set_cluster_size_fraction(frac);
        algo_.set_cluster_time_us(ct);
        algo_.set_min_cluster_events(mce);
        algo_.set_max_lost_age_s(mla);
        algo_.set_enable_velocity_prediction(evp);
        algo_.set_location_mixing_factor(lmf);
        algo_.set_predictive_velocity_factor(pvf);
        algo_.set_mass_decay_tau_us(tau);
        algo_.set_threshold_mass_for_visible(tmv);
        algo_.set_max_clusters(mc);
    }
};

/// CornerDetector backend — corners as overlay points.
class CornerDetectorBackend final : public AlgoBackend {
    gui_algo::CornerDetector algo_;
    std::vector<Metavision::EventCD> passthrough_;
    RoiFilter roi_;
    std::vector<gui_algo::Event> roi_buf_;
public:
    CornerDetectorBackend(int w, int h) : algo_(w, h) { roi_.init(w, h); }
    void set_param(const std::string& k, const std::string& v) override {
        if (roi_.set_param(k, v)) return;
        if (k == "mode") {
            int m = to_i(v);
            if (m >= 0 && m <= 3) algo_.set_mode(static_cast<gui_algo::CornerDetector::Mode>(m));
        } else if (k == "min_score") algo_.set_threshold(to_d(v));
        else if (k == "arc_corner_range_us") algo_.set_arc_corner_range_us(to_i(v));
        else if (k == "arc_min_response_us") algo_.set_arc_min_response_us(to_d(v));
    }
    std::string get_param(const std::string& k) const override {
        auto r = roi_.get_param(k); if (!r.empty()) return r;
        if (k == "mode") return from_i(static_cast<int>(algo_.mode()));
        if (k == "min_score") return from_d(algo_.threshold());
        if (k == "arc_corner_range_us") return from_i(algo_.arc_corner_range_us());
        if (k == "arc_min_response_us") return from_d(algo_.arc_min_response_us());
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        auto [ev, n] = roi_.apply(as_events(passthrough_.data()), passthrough_.size(), roi_buf_);
        algo_.process(ev, n);
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        for (const auto& c : algo_.corners()) {
            OverlayPoint p;
            p.x = static_cast<int>(c.x);
            p.y = static_cast<int>(c.y);
            p.strength = c.strength;
            r.points.push_back(p);
        }
        r.status = "corners: " + std::to_string(algo_.corners().size()) +
                   std::string(roi_.region.enabled ? " (ROI)" : "");
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); }
    void set_sensor_dimensions(int w, int h) override {
        roi_.set_sensor_dimensions(w, h);
        const auto m = algo_.mode();
        const double thr = algo_.threshold();
        const int tr = algo_.track_radius_px();
        const int arc_r = algo_.arc_corner_range_us();
        const double arc_resp = algo_.arc_min_response_us();
        algo_ = gui_algo::CornerDetector(w, h, m);
        algo_.set_threshold(thr);
        algo_.set_track_radius_px(tr);
        algo_.set_arc_corner_range_us(arc_r);
        algo_.set_arc_min_response_us(arc_resp);
    }
};

/// BlobDetector backend — blobs as overlay boxes.
class BlobDetectorBackend final : public AlgoBackend {
    gui_algo::BlobDetector algo_;
    std::vector<Metavision::EventCD> passthrough_;
    RoiFilter roi_;
    std::vector<gui_algo::Event> roi_buf_;
public:
    BlobDetectorBackend(int w, int h) : algo_(w, h) { roi_.init(w, h); }
    void set_param(const std::string& k, const std::string& v) override {
        if (roi_.set_param(k, v)) return;
        if (k == "threshold") algo_.set_threshold(static_cast<int>(to_d(v)));
        else if (k == "learning_rate") algo_.set_learning_rate(to_d(v));
        else if (k == "accumulation_ms") algo_.set_accumulation_ms(to_d(v));
        else if (k == "min_area") algo_.set_min_area(to_i(v));
    }
    std::string get_param(const std::string& k) const override {
        auto r = roi_.get_param(k); if (!r.empty()) return r;
        if (k == "threshold") return from_d(algo_.threshold());
        if (k == "learning_rate") return from_d(algo_.learning_rate());
        if (k == "accumulation_ms") return from_d(algo_.accumulation_ms());
        if (k == "min_area") return from_i(algo_.min_area());
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        auto [ev, n] = roi_.apply(as_events(passthrough_.data()), passthrough_.size(), roi_buf_);
        algo_.process(ev, n);
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        for (const auto& blob : algo_.blobs()) {
            OverlayBox box;
            box.x = blob.bbox.x; box.y = blob.bbox.y;
            box.w = blob.bbox.width; box.h = blob.bbox.height;
            r.boxes.push_back(box);
        }
        r.status = "blobs: " + std::to_string(algo_.blobs().size()) +
                   std::string(roi_.region.enabled ? " (ROI)" : "");
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); }
    void set_sensor_dimensions(int w, int h) override {
        roi_.set_sensor_dimensions(w, h);
        const int thr = algo_.threshold();
        const double lr = algo_.learning_rate();
        const double acc = algo_.accumulation_ms();
        const int ma = algo_.min_area();
        algo_ = gui_algo::BlobDetector(w, h);  // sensor-sized background map
        algo_.set_threshold(thr);
        algo_.set_learning_rate(lr);
        algo_.set_accumulation_ms(acc);
        algo_.set_min_area(ma);
    }
};

/// SparseOpticalFlow backend — flow vectors as pps-scaled overlay arrows.
/// jAER rbodo drawVector scheme: arrow length = speed(px/s) * pps_scale
/// (user-tunable, default 0.03) with a ±0.5 px origin jitter so overlapping
/// vectors stay distinguishable (AbstractDirectionSelectiveFilter
/// jitterVectorLocations, default on).
class SparseOpticalFlowBackend final : public AlgoBackend {
    gui_algo::SparseOpticalFlow algo_;
    std::vector<Metavision::EventCD> passthrough_;
    std::vector<gui_algo::FlowVector> flows_;
    RoiFilter roi_;
    std::vector<gui_algo::Event> roi_buf_;
    // Per-mode arrow scale, calibrated on screw.raw (median speed -> ~12px
    // arrow): LP 0.003, LK 0.7, BM 0.09, CO 0.05.
    double pps_scale_[4]{0.003, 0.7, 0.09, 0.05};
    int arrow_grid_px_{24};
public:
    SparseOpticalFlowBackend(int w, int h)
        : algo_(w, h, gui_algo::SparseOpticalFlow::Mode::LocalPlanes) { roi_.init(w, h); }
    void set_param(const std::string& k, const std::string& v) override {
        if (roi_.set_param(k, v)) return;
        if (k == "mode") {
            int m = to_i(v);
            if (m >= 0 && m <= 3) algo_.set_mode(static_cast<gui_algo::SparseOpticalFlow::Mode>(m));
        } else if (k == "search_radius") algo_.set_search_radius_px(to_i(v));
        else if (k == "time_window_us") algo_.set_time_window_us(to_i(v));
        else if (k == "cluster_ema_alpha") algo_.set_cluster_ema_alpha(static_cast<float>(to_d(v)));
        else if (k == "pps_scale_lp" || k == "pps_scale_lk" ||
                 k == "pps_scale_bm" || k == "pps_scale_co") {
            // Slot follows the PARAM KEY, not the current mode — the
            // instance ctor replays all defaults before any mode switch, so
            // keying by mode() made the four writes clobber one slot.
            pps_scale_[k == "pps_scale_lp" ? 0
                     : k == "pps_scale_lk" ? 1
                     : k == "pps_scale_bm" ? 2 : 3] = to_d(v);
        }
        else if (k == "arrow_grid_px") arrow_grid_px_ = to_i(v);
        else if (k == "lk_thr") algo_.set_lk_thr(to_d(v));
        else if (k == "bm_time_window_us") algo_.set_block_match_time_window_us(to_i(v));
        else if (k == "downsample_factor") algo_.set_downsample_factor(to_i(v));
        else if (k == "num_scales") algo_.set_num_scales(to_i(v));
        else if (k == "max_slice_value") algo_.set_max_slice_value(to_i(v));
        else if (k == "valid_pix_occupancy") algo_.set_valid_pix_occupancy(to_d(v));
        else if (k == "weight_distance") algo_.set_weight_distance(to_d(v));
    }
    std::string get_param(const std::string& k) const override {
        auto r = roi_.get_param(k); if (!r.empty()) return r;
        if (k == "mode") return from_i(static_cast<int>(algo_.mode()));
        if (k == "search_radius") return from_i(algo_.search_radius_px());
        if (k == "time_window_us") return from_i(algo_.time_window_us());
        if (k == "cluster_ema_alpha") return from_d(algo_.cluster_ema_alpha());
        if (k == "pps_scale_lp" || k == "pps_scale_lk" ||
            k == "pps_scale_bm" || k == "pps_scale_co") {
            return from_d(pps_scale_[k == "pps_scale_lp" ? 0
                                   : k == "pps_scale_lk" ? 1
                                   : k == "pps_scale_bm" ? 2 : 3]);
        }
        if (k == "arrow_grid_px") return from_i(arrow_grid_px_);
        if (k == "lk_thr") return from_d(algo_.lk_thr());
        if (k == "bm_time_window_us") return from_i(algo_.block_match_time_window_us());
        if (k == "downsample_factor") return from_i(algo_.downsample_factor());
        if (k == "num_scales") return from_i(algo_.num_scales());
        if (k == "max_slice_value") return from_i(algo_.max_slice_value());
        if (k == "valid_pix_occupancy") return from_d(algo_.valid_pix_occupancy());
        if (k == "weight_distance") return from_d(algo_.weight_distance());
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        auto [ev, n] = roi_.apply(as_events(passthrough_.data()),
                                   passthrough_.size(), roi_buf_);
        std::vector<gui_algo::FlowVector> fresh;
        algo_.process(ev, n, fresh);
        if (!fresh.empty()) {
            flows_ = std::move(fresh);
        } else if (algo_.mode() != gui_algo::SparseOpticalFlow::Mode::BlockMatch) {
            // Per-batch modes reflect the current batch (empty batch -> no
            // vectors). BlockMatch only emits once per bm_time_window_us —
            // hold the last result between emission boundaries instead of
            // blanking every intermediate batch.
            flows_.clear();
        }
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        // Fixed-grid arrows: average the flow vectors inside each grid cell
        // and draw ONE arrow per cell (cell centre), length = mean speed *
        // pps_scale capped at the grid spacing so arrows never overflow
        // their cell. Keeps the field readable instead of a dense tangle.
        const int g = std::max(4, arrow_grid_px_);
        const int gw = (algo_.width() / g) + 1;
        const int gh = (algo_.height() / g) + 1;
        std::vector<double> sx(static_cast<std::size_t>(gw) * gh, 0.0);
        std::vector<double> sy(static_cast<std::size_t>(gw) * gh, 0.0);
        std::vector<int> cnt(static_cast<std::size_t>(gw) * gh, 0);
        for (const auto& f : flows_) {
            if (!std::isfinite(f.vx) || !std::isfinite(f.vy)) continue;
            const int cx = static_cast<int>(f.x) / g;
            const int cy = static_cast<int>(f.y) / g;
            if (cx >= gw || cy >= gh) continue;
            const std::size_t ci = static_cast<std::size_t>(cy) * gw + cx;
            sx[ci] += f.vx;
            sy[ci] += f.vy;
            ++cnt[ci];
        }
        r.flow_arrows.reserve(static_cast<std::size_t>(gw) * gh);
        for (int cy = 0; cy < gh; ++cy) {
            for (int cx = 0; cx < gw; ++cx) {
                const std::size_t ci = static_cast<std::size_t>(cy) * gw + cx;
                if (cnt[ci] == 0) continue;
                const float vx = static_cast<float>(sx[ci] / cnt[ci]);
                const float vy = static_cast<float>(sy[ci] / cnt[ci]);
                const float len = std::hypot(vx, vy);
                if (len <= 0.0f) continue;
                float scale = static_cast<float>(pps_scale_[static_cast<int>(algo_.mode())]);
                const float draw_len = len * scale;
                if (draw_len > static_cast<float>(g)) scale = static_cast<float>(g) / len;
                const int ax = cx * g + g / 2;
                const int ay = cy * g + g / 2;
                OverlayFlowArrow a;
                a.x1 = ax;
                a.y1 = ay;
                a.x2 = ax + static_cast<int>(vx * scale);
                a.y2 = ay + static_cast<int>(vy * scale);
                r.flow_arrows.push_back(a);
            }
        }
        r.status = "flow: " + std::to_string(r.flow_arrows.size()) + " vectors" +
                   std::string(roi_.region.enabled ? " (ROI)" : "");
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); flows_.clear(); }
    void set_sensor_dimensions(int w, int h) override {
        roi_.set_sensor_dimensions(w, h);
        // Persist tuned params across the rebuild (the auto-ROI resize hits
        // the live instance).
        const auto m = algo_.mode();
        const int tw = algo_.time_window_us();
        const int sr = algo_.search_radius_px();
        const int bm_tw = algo_.block_match_time_window_us();
        const double lk_thr = algo_.lk_thr();
        const float ema = algo_.cluster_ema_alpha();
        algo_ = gui_algo::SparseOpticalFlow(w, h, m);
        algo_.set_time_window_us(tw);
        algo_.set_search_radius_px(sr);
        algo_.set_block_match_time_window_us(bm_tw);
        algo_.set_lk_thr(lk_thr);
        algo_.set_cluster_ema_alpha(ema);
    }
private:
    /// ±0.5 px uniform jitter (jAER jitterAmountPixels default).
    static float jitter() { return (static_cast<float>(std::rand() % 1000) / 999.0F - 0.5F); }
};

/// DenseOpticalFlow backend — per-pixel flow map rendered as HSV-colored
/// points (hue = direction, brightness = magnitude, scaled by confidence).
class DenseOpticalFlowBackend final : public AlgoBackend {
    gui_algo::DenseOpticalFlow algo_;
    // Per-mode arrow scale, calibrated on screw.raw: PF 0.002, TG 0.007,
    // TM 0.009 (median speed -> ~12px arrow).
    double pps_scale_[3]{0.002, 0.007, 0.009};
    int arrow_grid_px_{24};
    std::vector<Metavision::EventCD> passthrough_;
    RoiFilter roi_;
    std::vector<gui_algo::Event> roi_buf_;
public:
    DenseOpticalFlowBackend(int w, int h)
        : algo_(w, h, gui_algo::DenseOpticalFlow::Mode::PlaneFitting) { roi_.init(w, h); }
    void set_param(const std::string& k, const std::string& v) override {
        if (roi_.set_param(k, v)) return;
        if (k == "mode") {
            const int m = to_i(v);
            if (m >= 0 && m <= 2)
                algo_.set_mode(static_cast<gui_algo::DenseOpticalFlow::Mode>(m));
        } else if (k == "time_window_us") algo_.set_time_window_us(to_i(v));
        else if (k == "spatial_radius")  algo_.set_spatial_radius_px(to_i(v));
        else if (k == "max_velocity_px_s") algo_.set_max_velocity_px_s(static_cast<float>(to_d(v)));
        else if (k == "pps_scale_pf" || k == "pps_scale_tg" ||
                 k == "pps_scale_tm") {
            // Slot follows the PARAM KEY, not the current mode — the
            // instance ctor replays all defaults before any mode switch, so
            // keying by mode() made the three writes clobber one slot.
            pps_scale_[k == "pps_scale_pf" ? 0
                     : k == "pps_scale_tg" ? 1 : 2] = to_d(v);
        }
        else if (k == "arrow_grid_px") arrow_grid_px_ = to_i(v);
    }
    std::string get_param(const std::string& k) const override {
        auto r = roi_.get_param(k); if (!r.empty()) return r;
        if (k == "mode") return from_i(static_cast<int>(algo_.mode()));
        if (k == "time_window_us") return from_i(algo_.time_window_us());
        if (k == "spatial_radius") return from_i(algo_.spatial_radius_px());
        if (k == "max_velocity_px_s") return from_d(algo_.max_velocity_px_s());
        if (k == "pps_scale_pf" || k == "pps_scale_tg" ||
            k == "pps_scale_tm") {
            return from_d(pps_scale_[k == "pps_scale_pf" ? 0
                                   : k == "pps_scale_tg" ? 1 : 2]);
        }
        if (k == "arrow_grid_px") return from_i(arrow_grid_px_);
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        auto [ev, n] = roi_.apply(as_events(passthrough_.data()), passthrough_.size(), roi_buf_);
        algo_.process(ev, ev + n);
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        cv::Mat flow, conf;
        algo_.get_flow(flow, conf);
        // Fixed-grid arrows: average the per-pixel flow inside each grid
        // cell and draw ONE arrow per cell (cell centre), length = mean
        // speed * pps_scale capped at the grid spacing.
        const int g = std::max(4, arrow_grid_px_);
        const int gw = (algo_.width() / g) + 1;
        const int gh = (algo_.height() / g) + 1;
        std::vector<double> sx(static_cast<std::size_t>(gw) * gh, 0.0);
        std::vector<double> sy(static_cast<std::size_t>(gw) * gh, 0.0);
        std::vector<int> cnt(static_cast<std::size_t>(gw) * gh, 0);
        for (int y = 0; y < algo_.height(); ++y) {
            const cv::Vec2f* frow = flow.ptr<cv::Vec2f>(y);
            const float* crow = conf.ptr<float>(y);
            for (int x = 0; x < algo_.width(); ++x) {
                if (crow[x] <= 0.0f) continue;
                const float vx = frow[x][0], vy = frow[x][1];
                if (vx == 0.0f && vy == 0.0f) continue;
                if (!std::isfinite(vx) || !std::isfinite(vy)) continue;
                const int cx = x / g;
                const int cy = y / g;
                if (cx >= gw || cy >= gh) continue;
                const std::size_t ci = static_cast<std::size_t>(cy) * gw + cx;
                sx[ci] += vx;
                sy[ci] += vy;
                ++cnt[ci];
            }
        }
        int count = 0;
        r.flow_arrows.reserve(static_cast<std::size_t>(gw) * gh);
        for (int cy = 0; cy < gh; ++cy) {
            for (int cx = 0; cx < gw; ++cx) {
                const std::size_t ci = static_cast<std::size_t>(cy) * gw + cx;
                if (cnt[ci] == 0) continue;
                const float vx = static_cast<float>(sx[ci] / cnt[ci]);
                const float vy = static_cast<float>(sy[ci] / cnt[ci]);
                const float len = std::hypot(vx, vy);
                if (len <= 0.0f) continue;
                float scale = static_cast<float>(pps_scale_[static_cast<int>(algo_.mode())]);
                const float draw_len = len * scale;
                if (draw_len > static_cast<float>(g)) scale = static_cast<float>(g) / len;
                const int ax = cx * g + g / 2;
                const int ay = cy * g + g / 2;
                OverlayFlowArrow a;
                a.x1 = ax;
                a.y1 = ay;
                a.x2 = ax + static_cast<int>(vx * scale);
                a.y2 = ay + static_cast<int>(vy * scale);
                r.flow_arrows.push_back(a);
                ++count;
            }
        }
        r.status = "dense flow: " + std::to_string(count) + " cells" +
                   std::string(roi_.region.enabled ? " (ROI)" : "");
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); }
    void set_sensor_dimensions(int w, int h) override {
        roi_.set_sensor_dimensions(w, h);
        // Persist tuned params across the rebuild (the auto-ROI resize hits
        // the live instance).
        const auto m = algo_.mode();
        const int tw = algo_.time_window_us();
        const int sr = algo_.spatial_radius_px();
        const float mv = algo_.max_velocity_px_s();
        algo_ = gui_algo::DenseOpticalFlow(w, h, m);
        algo_.set_time_window_us(tw);
        algo_.set_spatial_radius_px(sr);
        algo_.set_max_velocity_px_s(mv);
    }
private:
    /// HSV (h,s,v in [0,1]) -> RGB (r,g,b in [0,255]). Mirrors the sparse-flow
    /// backend's helper (kept local to avoid cross-backend coupling).
    static void hsv_to_rgb(float h, float s, float v,
                           std::uint8_t& r, std::uint8_t& g, std::uint8_t& b) {
        const float c = v * s;
        const float hp = h * 6.0f;
        const float x = c * (1.0f - std::fabs(std::fmod(hp, 2.0f) - 1.0f));
        float rf = 0.0f, gf = 0.0f, bf = 0.0f;
        if (hp < 1.0f)      { rf = c; gf = x; bf = 0.0f; }
        else if (hp < 2.0f) { rf = x; gf = c; bf = 0.0f; }
        else if (hp < 3.0f) { rf = 0.0f; gf = c; bf = x; }
        else if (hp < 4.0f) { rf = 0.0f; gf = x; bf = c; }
        else if (hp < 5.0f) { rf = x; gf = 0.0f; bf = c; }
        else                { rf = c; gf = 0.0f; bf = x; }
        const float m = v - c;
        r = static_cast<std::uint8_t>((rf + m) * 255.0f + 0.5f);
        g = static_cast<std::uint8_t>((gf + m) * 255.0f + 0.5f);
        b = static_cast<std::uint8_t>((bf + m) * 255.0f + 0.5f);
    }
};


/// DLOpticalFlow backend — EVFlowNet (Zhu 2018 / Stoffregen ECCV 2020)
/// deep dense optical flow: one 5-bin voxel per accumulation window in,
/// per-pixel (u, v) displacement out, rendered as an HSV-coded frame
/// (Standalone window; reuse of the E2VIDInference runtime selection means
/// the status line reports dev=gpu|cpu like event_to_video).
class DLOpticalFlowBackend final : public AlgoBackend {
    gui_algo::DLOpticalFlow algo_;
    std::string model_path_;
    int device_{0};  ///< 0=Auto, 1=CPU, 2=GPU (§4.4.2-GPU)
    int output_fps_{30};
    std::vector<Metavision::EventCD> passthrough_;
    RoiFilter roi_;
    std::vector<gui_algo::Event> roi_buf_;
public:
    DLOpticalFlowBackend(int w, int h)
        : algo_(w, h, 30) { roi_.init(w, h); }
    void set_param(const std::string& k, const std::string& v) override {
        if (roi_.set_param(k, v)) return;
        if (k == "model_path") {
            model_path_ = v;
            if (!model_path_.empty()) algo_.set_model_path(model_path_);
        } else if (k == "device") {
            device_ = to_i(v);
            algo_.set_device(device_);
        } else if (k == "output_fps") {
            output_fps_ = to_i(v);
            algo_.set_output_fps(output_fps_);
        } else if (k == "max_velocity_px_s") {
            algo_.set_max_velocity_px_s(static_cast<float>(to_d(v)));
        }
    }
    std::string get_param(const std::string& k) const override {
        auto r = roi_.get_param(k); if (!r.empty()) return r;
        if (k == "model_path") return model_path_;
        if (k == "device") return from_i(device_);
        if (k == "output_fps") return from_i(output_fps_);
        if (k == "max_velocity_px_s") return from_d(algo_.max_velocity_px_s());
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        auto [ev, n] = roi_.apply(as_events(passthrough_.data()), passthrough_.size(), roi_buf_);
        algo_.process(ev, n);
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        cv::Mat frame = algo_.get_frame();
        if (!frame.empty()) {
            r.has_frame = true;
            r.frame = frame.clone();
        }
        r.status = "flow_dl";
        if (algo_.is_model_loaded()) {
            r.status += " model=loaded dev=" + algo_.active_runtime();
        } else {
            r.status += " model=missing";
        }
        r.status += std::string(roi_.region.enabled ? " (ROI)" : "");
        return r;
    }
    void reset() override { algo_.reset(); passthrough_.clear(); }
    void set_sensor_dimensions(int w, int h) override {
        roi_.set_sensor_dimensions(w, h);
        // Persist tuned params across the rebuild (the auto-ROI resize hits
        // the live instance).
        const std::string mp = model_path_;
        const int dev = device_;
        const int fps = output_fps_;
        const float mv = algo_.max_velocity_px_s();
        algo_ = gui_algo::DLOpticalFlow(w, h, fps);
        model_path_ = mp;
        device_ = dev;
        algo_.set_device(device_);
        algo_.set_max_velocity_px_s(mv);
        if (!model_path_.empty()) algo_.set_model_path(model_path_);
    }
};


// --- Per-category factory (called by create_algo_backend in backend_factory.cpp)
std::unique_ptr<AlgoBackend> create_cv_backend(const std::string& name,
                                          int width, int height) {
    if (name == "hot_pixel_filter")            return std::make_unique<HotPixelFilterBackend>(width, height);
    if (name == "optical_gyro")                return std::make_unique<OpticalGyroBackend>(width, height);
    if (name == "object_tracker")              return std::make_unique<ObjectTrackerBackend>(width, height);
    if (name == "corner_detector")             return std::make_unique<CornerDetectorBackend>(width, height);
    if (name == "blob_detector")               return std::make_unique<BlobDetectorBackend>(width, height);
    if (name == "sparse_optical_flow")         return std::make_unique<SparseOpticalFlowBackend>(width, height);
    if (name == "dense_optical_flow")          return std::make_unique<DenseOpticalFlowBackend>(width, height);
    if (name == "dl_optical_flow")             return std::make_unique<DLOpticalFlowBackend>(width, height);
    return nullptr;
}

} // namespace gui
