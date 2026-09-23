// algo/cv/optical_gyro.h — Electronic Image Stabilization (EIS).
//
// ✅ 移植自 jAER OpticalGyro. Cluster-based camera motion estimation:
// mass-weighted mean translation from cluster displacements (location -
// birthLocation), small-angle least-squares rotation (joint solve, gated
// by rotation_enabled_), IIR low-pass filtering of estimates, and inverse-
// transform EIS compensation applied to incoming events. Corresponds to
// design §4.3.23. Header-only.

#ifndef GUI_ALGO_CV_OPTICAL_GYRO_H
#define GUI_ALGO_CV_OPTICAL_GYRO_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <metavision/sdk/base/utils/timestamp.h>

#include "algo/common/event.h"
#include "algo/common/event_packet.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace gui_algo {

/// @brief Estimated global camera motion (translation + rotation).
struct MotionEstimate {
    float dx{0.0f};       ///< Translation in x (pixels).
    float dy{0.0f};       ///< Translation in y (pixels).
    float dtheta{0.0f};   ///< Rotation (degrees).
};

/// @brief Electronic image stabiliser estimating camera motion from cluster
///        tracking and compensating incoming events by the inverse transform.
///
/// ✅ 移植自 jAER OpticalGyro (net.sf.jaer.eventprocessing.tracking).
/// Maintains an internal cluster tracker, computes mass-weighted mean
/// translation from cluster displacements (location - birthLocation,
/// OpticalGyro.java:175-176), solves a joint small-angle least-squares
/// rotation+translation (when rotation_enabled_), low-pass filters the
/// estimates, and applies the inverse transform to events for EIS.
class OpticalGyro {
public:
    OpticalGyro(int width, int height,
                float stabilization_strength = 1.0f,
                float smoothing_window_ms = 100.0f)
        : width_(width), height_(height),
          strength_(clamp_f(stabilization_strength, 0.0f, 1.0f)),
          smoothing_ms_(clamp_f(smoothing_window_ms, 1.0f, 10000.0f)),
          sx2_(static_cast<float>(width) * 0.5f),
          sy2_(static_cast<float>(height) * 0.5f) {}

    /// @brief Tracks events, updates the gyro estimate, and compensates event
    ///        coordinates in-place by the accumulated inverse motion.
    void process(MutableEventPacket& packet) {
        if (packet.empty()) return;
        // Feed events to the internal cluster tracker (jAER super.filterPacket).
        Metavision::timestamp last_t = 0;
        for (const Event& e : packet) {
            if (e.x >= width_ || e.y >= height_) continue;
            track_event(e);
            last_t = e.t;
        }
        // Update gyro estimate from cluster displacements (jAER update(t)).
        update_gyro(last_t);
        // Apply EIS inverse transform to events (jAER transformEvent).
        if (strength_ > 0.0f) {
            apply_eis(packet);
        }
    }

    // Motion accessors ------------------------------------------------------
    /// @brief Filtered cumulative motion estimate (translation in px, rotation
    ///        in degrees). This is the jAER translation/rotationAngle output.
    MotionEstimate smoothed_motion() const {
        return MotionEstimate{trans_x_filt_, trans_y_filt_,
                              rot_filt_ * static_cast<float>(180.0 / M_PI)};
    }
    MotionEstimate total_motion() const { return smoothed_motion(); }

    // Parameter accessors ---------------------------------------------------
    float stabilization_strength() const { return strength_; }
    float smoothing_window_ms() const { return smoothing_ms_; }
    void set_stabilization_strength(float v) {
        strength_ = clamp_f(v, 0.0f, 1.0f);
    }
    /// @brief Enables/disables rotation estimation. When disabled (default
    /// follows jAER opticalGyroRotationEnabled=false), only translation is
    /// estimated and applied, avoiding ill-conditioned LS solves on pure-
    /// translation scenes.
    void set_rotation_enabled(bool v) { rotation_enabled_ = v; }
    bool rotation_enabled() const { return rotation_enabled_; }
    void set_smoothing_window_ms(float v) {
        smoothing_ms_ = clamp_f(v, 1.0f, 10000.0f);
    }

    /// @brief Sensor width (px) — used by backends for overlay rendering.
    int width() const { return width_; }
    /// @brief Sensor height (px) — used by backends for overlay rendering.
    int height() const { return height_; }

    void reset() {
        clusters_.clear();
        trans_x_filt_ = 0.0f;
        trans_y_filt_ = 0.0f;
        rot_filt_ = 0.0f;
        rot_for_transform_ = 0.0f;
        last_filter_t_ = 0;
    }

private:
    static float clamp_f(float v, float lo, float hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }

    /// @brief A tracked cluster with birth location and IIR-smoothed position.
    /// Mirrors jAER RectangularClusterTracker.Cluster (simplified: nearest-
    /// neighbour association, IIR location update, exponential mass decay).
    struct GyroCluster {
        float birth_x{0.0F};
        float birth_y{0.0F};
        float x{0.0F};
        float y{0.0F};
        float vx{0.0F};
        float vy{0.0F};
        float mass{0.0F};
        Metavision::timestamp birth_t{0};
        Metavision::timestamp last_t{0};
        bool became_visible{false};  ///< jAER: birth reset on first visibility
    };

    /// @brief Nearest-neighbour IIR cluster tracker (like jAER RCT mode).
    void track_event(const Event& e) {
        const float cs = static_cast<float>(cluster_size_px_);
        int best = -1;
        float best_d2 = cs * cs;
        for (int k = 0; k < static_cast<int>(clusters_.size()); ++k) {
            GyroCluster& c = clusters_[k];
            if (e.t - c.last_t > cluster_time_us_) continue;
            const float dx = c.x - static_cast<float>(e.x);
            const float dy = c.y - static_cast<float>(e.y);
            const float d2 = dx * dx + dy * dy;
            if (d2 < best_d2) { best_d2 = d2; best = k; }
        }
        if (best < 0) {
            if (static_cast<int>(clusters_.size()) < max_clusters_) {
                GyroCluster c;
                c.birth_x = c.x = static_cast<float>(e.x);
                c.birth_y = c.y = static_cast<float>(e.y);
                c.mass = 1.0F;
                c.birth_t = c.last_t = e.t;
                clusters_.push_back(c);
            }
        } else {
            GyroCluster& c = clusters_[best];
            const float a = location_mixing_factor_;
            // Per-event velocity: IIR-smoothed position increment per second
            // (jAER Cluster computes velocity from the same IIR position;
            // the velocity drives the gainVelocity feed-forward term in
            // transformEvent, OpticalGyro.java:120-131).
            if (e.t > c.last_t && c.last_t > 0) {
                const float dt_s = static_cast<float>(e.t - c.last_t) * 1e-6F;
                if (dt_s > 0.0F) {
                    const float ivx = (static_cast<float>(e.x) - c.x) / dt_s;
                    const float ivy = (static_cast<float>(e.y) - c.y) / dt_s;
                    const float v_alpha = std::min(1.0F, dt_s / 0.05F);  // 50ms tau
                    c.vx += v_alpha * (ivx - c.vx);
                    c.vy += v_alpha * (ivy - c.vy);
                }
            }
            c.x = c.x * (1.0F - a) + static_cast<float>(e.x) * a;
            c.y = c.y * (1.0F - a) + static_cast<float>(e.y) * a;
            // jAER Cluster.updateMass (RectangularClusterTracker): the
            // STORED mass decays per event and then the event adds 1, so
            // the steady-state mass is rate × tau (bounded). The original
            // port only decayed on READ (mass_now), letting the stored mass
            // grow to the cumulative event count — the age feed-forward
            // below then multiplies a huge mass by the cluster age and the
            // int conversion overflows (UB on x86: INT_MIN), throwing the
            // whole stabilized frame onto one border.
            if (e.t > c.last_t) {
                c.mass *= std::exp(-static_cast<float>(e.t - c.last_t) /
                                   static_cast<float>(mass_decay_tau_us_));
            }
            c.mass += 1.0F;
            c.last_t = e.t;
        }
    }

    /// @brief Mass with exponential decay (jAER Cluster.getMassNow).
    float mass_now(const GyroCluster& c, Metavision::timestamp t) const {
        if (t <= c.last_t) return c.mass;
        const double dt = static_cast<double>(t - c.last_t);
        return c.mass * static_cast<float>(
            std::exp(-dt / static_cast<double>(mass_decay_tau_us_)));
    }

    /// @brief Whether a cluster is visible (recent + enough mass).
    /// Mirrors jAER Cluster.isVisible.
    bool is_visible(const GyroCluster& c, Metavision::timestamp t) const {
        if (t - c.last_t > cluster_time_us_) return false;
        return mass_now(c, t) > min_visible_mass_;
    }

    /// @brief Computes the instantaneous camera motion from cluster
    ///        displacements and low-pass filters it.
    /// Mirrors jAER OpticalGyro.update() + SmallAngleTransformFinder.update().
    /// Translation: weighted mean of (location - birthLocation) over visible
    /// clusters, weighted by mass. Rotation: small-angle LS fit
    /// H=[[1,-a],[a,1]] mapping present→birth, solved jointly for a, Tx, Ty.
    void update_gyro(Metavision::timestamp t) {
        // Prune stale clusters.
        std::vector<GyroCluster> kept;
        kept.reserve(clusters_.size());
        for (auto& c : clusters_) {
            if (t - c.last_t <= cluster_time_us_ * 4) kept.push_back(std::move(c));
        }
        clusters_.swap(kept);

        // Accumulate mass-weighted translation + small-angle rotation LS.
        float weight_sum = 0.0F;
        float avgxloc = 0.0F, avgyloc = 0.0F;
        float avgxv = 0.0F, avgyv = 0.0F;
        double age_sum = 0.0;
        // SmallAngleTransformFinder accumulators (w=1 per cluster, as in jAER).
        double qy2 = 0.0, qx2 = 0.0, qy = 0.0, qx = 0.0;
        double px = 0.0, py = 0.0, pxqy = 0.0, pyqx = 0.0;
        int w_sum = 0;
        int n_visible = 0;
        for (auto& c : clusters_) {
            if (!is_visible(c, t)) continue;
            // jAER RCT.java:2358-2361: 簇首次可见时把 birthLocation 重置为
            // 当前位置（位移基线不含簇未成熟期的噪声）。
            if (!c.became_visible) {
                c.became_visible = true;
                c.birth_x = c.x;
                c.birth_y = c.y;
            }
            const float w = mass_now(c, t);
            weight_sum += w;
            // 位移 = location − birthLocation（OpticalGyro.java:175-176）。
            // jAER 的 velocityPPt 仅用于 transformEvent 的 gainVelocity 项，
            // 不存在位移的速度外推——此前的自研外推已删除（§二-2.1）。
            avgxloc += (c.x - c.birth_x) * w;
            avgyloc += (c.y - c.birth_y) * w;
            // jAER velocityPPt: mass-weighted mean cluster velocity.
            avgxv += c.vx * w;
            avgyv += c.vy * w;
            age_sum += static_cast<double>(t - c.birth_t) * w;
            // Small-angle LS accumulators (centered on sensor midpoint).
            const double ppx = static_cast<double>(c.birth_x - sx2_);
            const double ppy = static_cast<double>(c.birth_y - sy2_);
            const double qqx = static_cast<double>(c.x - sx2_);
            const double qqy = static_cast<double>(c.y - sy2_);
            qy2 += qqy * qqy;
            qx2 += qqx * qqx;
            qx += qqx;
            qy += qqy;
            px += ppx;
            py += ppy;
            pxqy += ppx * qqy;
            pyqx += ppy * qqx;
            ++w_sum;
            ++n_visible;
        }
        if (weight_sum <= 0.0F || n_visible == 0) return;

        // Translation (mass-weighted mean displacement, negated to map
        // present→birth, matching jAER filterTransform(-avgxloc, ...)).
        avgxloc /= weight_sum;
        avgyloc /= weight_sum;
        // jAER: averageClusterAge = ageSum/weightSum (weighted mean cluster
        // age); velocityPPt = weighted mean velocity.
        // Saturate instead of trusting upstream boundedness: the age feeds
        // a position offset, so a runaway value would shove every event off
        // frame (clamped to the border) rather than crash.
        average_cluster_age_ = std::clamp(
            age_sum / static_cast<double>(weight_sum), 0.0, 3.6e12);  // ≤ 1 h
        vel_pp_x_ = avgxv / weight_sum;
        vel_pp_y_ = avgyv / weight_sum;
        float inst_tx = -avgxloc;
        float inst_ty = -avgyloc;
        float inst_rot = 0.0F;

        // Rotation: joint small-angle LS solve (jAER SmallAngleTransformFinder).
        // Needs ≥3 visible clusters for a non-degenerate solve.
        // Gated by rotation_enabled_ (jAER opticalGyroRotationEnabled defaults
        // to false — pure-translation scenes do not benefit from rotation).
        // 差异（§二-2.1）：rotation 开启但可见簇 <3（或 LS 近奇异）时，jAER
        // 完全冻结估计；本实现仍用上方 mass-weighted 平移更新滤波器。
        if (rotation_enabled_ && n_visible >= 3) {
            const double aden = (qy2 + qx2)
                - ((qy * qy + qx * qx) / static_cast<double>(w_sum));
            if (std::fabs(aden) > 1e-10) {
                const double anum =
                    (((qy * (px - qx) - qx * (py - qy))
                      / static_cast<double>(w_sum))
                     - pxqy + pyqx);
                inst_rot = static_cast<float>(anum / aden);
                // Translation from the joint solve (NOT negated — the LS
                // already produces the present→birth mapping).
                inst_tx = static_cast<float>(
                    ((px - qx) + inst_rot * qy) / static_cast<double>(w_sum));
                inst_ty = static_cast<float>(
                    ((py - qy) - inst_rot * qx) / static_cast<double>(w_sum));
                // jAER computes cosAngle/sinAngle from the OLD rotationAngle
                // BEFORE filterTransform updates it (OpticalGyro.java:312-314).
                // Save the pre-update rot_filt_ for use in apply_eis().
                rot_for_transform_ = rot_filt_;
            }
        }

        // IIR low-pass filter (jAER LowpassFilter / LowpassFilter2D).
        const float alpha = compute_filter_alpha(t);
        trans_x_filt_ += (inst_tx - trans_x_filt_) * alpha;
        trans_y_filt_ += (inst_ty - trans_y_filt_) * alpha;
        rot_filt_ += (inst_rot - rot_filt_) * alpha;
        last_filter_t_ = t;
    }

    /// @brief Applies the inverse motion transform to events for EIS.
    /// p = R(a) * q + T (maps present location q back to birth location p),
    /// matching jAER OpticalGyro.transformEvent.
    void apply_eis(MutableEventPacket& packet) {
        // Use rot_for_transform_ (the OLD rotationAngle, jAER cosAngle/sinAngle)
        // rather than the just-updated rot_filt_, matching jAER's timing where
        // cosAngle/sinAngle are computed BEFORE filterTransform updates
        // rotationAngle (OpticalGyro.java:312-314). Translation uses the new
        // filtered value, as in jAER (filterTransform updates translation
        // before transformEvent reads it).
        const float ca = std::cos(rot_for_transform_ * strength_);
        const float sa = std::sin(rot_for_transform_ * strength_);
        const float tx = trans_x_filt_ * strength_;
        const float ty = trans_y_filt_ * strength_;
        // jAER gainVelocity feed-forward (OpticalGyro.transformEvent):
        // time * velocityPPt * gainVelocity — the velocity term cancels the
        // low-pass lag on fast motion. gainVelocity = strength_.
        // UNIT FIX (2026-08-23): jAER's velocityPPt is px/us and time is us
        // (px/us * us = px). Our vel_pp_ is px/s, so the feed-forward was
        // ~1e6x too large — events were thrown off-frame and clamped to the
        // border, wiping out the main display. Convert px/s -> px/us.
        const float vtx = static_cast<float>(average_cluster_age_) * vel_pp_x_ * 1e-6f * strength_;
        const float vty = static_cast<float>(average_cluster_age_) * vel_pp_y_ * 1e-6f * strength_;
        const float xmax = static_cast<float>(width_ - 1);
        const float ymax = static_cast<float>(height_ - 1);
        for (Event& e : packet) {
            if (e.x >= width_ || e.y >= height_) continue;
            const float x = static_cast<float>(e.x) - sx2_;
            const float y = static_cast<float>(e.y) - sy2_;
            float rx = ca * x - sa * y + tx + vtx + sx2_;
            float ry = sa * x + ca * y + ty + vty + sy2_;
            if (rx < 0.0f) rx = 0.0f;
            else if (rx > xmax) rx = xmax;
            if (ry < 0.0f) ry = 0.0f;
            else if (ry > ymax) ry = ymax;
            e.x = static_cast<std::uint16_t>(rx);
            e.y = static_cast<std::uint16_t>(ry);
        }
    }

    /// @brief IIR filter coefficient, jAER LowpassFilter 线性式：
    /// fac = min(1, dt_us / (tau_ms*1000))；dt <= 0 时 fac = 0 保持原值
    /// （此前的 1-exp(-dt/tau) 指数式在 dt>=tau 时只走 63%，且时间回退时
    /// 会跳变到新值——均与 jAER 不同，§二-2.1）。
    float compute_filter_alpha(Metavision::timestamp t) {
        if (last_filter_t_ == 0) return 1.0F;  // first sample: initialise
        if (t <= last_filter_t_) return 0.0F;  // jAER: dt<=0 → keep old value
        const float dt_us = static_cast<float>(t - last_filter_t_);
        const float tau_us = smoothing_ms_ * 1000.0F;
        const float fac = dt_us / tau_us;
        return fac > 1.0F ? 1.0F : fac;
    }

    int width_;
    int height_;
    float strength_;
    float smoothing_ms_;
    bool rotation_enabled_{false};  // jAER opticalGyroRotationEnabled default
    float sx2_;  // sensor center x (for centering coordinates)
    float sy2_;  // sensor center y
    // NOTE: non-const so the class stays move-assignable (backends reconstruct
    // it in place on sensor-dimension changes).

    // Filtered cumulative motion estimates (jAER translation / rotationAngle).
    float trans_x_filt_{0.0F};
    float trans_y_filt_{0.0F};
    float rot_filt_{0.0F};    // radians
    // Rotation angle used for the EIS transform (jAER cosAngle/sinAngle).
    // Holds the OLD rotationAngle (before the current filterTransform update),
    // since jAER computes cosAngle/sinAngle BEFORE updating rotationAngle.
    float rot_for_transform_{0.0F};
    // jAER gainVelocity feed-forward state (OpticalGyro.transformEvent):
    // velocityPPt (mass-weighted mean cluster velocity) and the weighted
    // average cluster age — the velocity term cancels the low-pass lag.
    float vel_pp_x_{0.0F};
    float vel_pp_y_{0.0F};
    int average_cluster_age_{0};
    Metavision::timestamp last_filter_t_{0};

    // Cluster tracker parameters — 自研默认值，与 jAER RectangularClusterTracker
    // 不同（§二-2.1）：mixing 0.2 vs jAER 0.05（位置响应慢 4 倍）、
    // mass_decay_tau 100000 vs 10000、min_visible_mass 2.0 vs 10~30、
    // max_clusters 100 vs 10；cluster_time_us_ 在 jAER 中无对应物。
    int cluster_size_px_{15};
    int cluster_time_us_{10000};
    int mass_decay_tau_us_{100000};
    float min_visible_mass_{2.0F};
    float location_mixing_factor_{0.2F};
    int max_clusters_{100};

    std::vector<GyroCluster> clusters_;
};

} // namespace gui_algo

#endif // GUI_ALGO_CV_OPTICAL_GYRO_H
