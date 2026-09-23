// algo/cv/object_tracker.h — event-level multi-object tracking.
//
// Port of jAER RectangularClusterTracker (RCT) semantics (design §4.3.11).
// The former self-developed Median / Kalman / MultiHypothesis extension
// modes were removed (2026-08-22, user decision) — jAER has no counterpart
// for them. Proliferation fixes aligned with jAER:
//   * max clusters hard-capped at 10 (jAER maxNumClusters; ours was 2000).
//   * cluster size is a FRACTION of the sensor's max dimension (jAER
//     clusterSize, initDefault 0.15 → ~192 px radius on a 1280 px sensor),
//     not an absolute pixel count — the old absolute 10 px gate fragmented
//     single objects into grids of clusters.
//   * association is per-axis rectangular containment within the radius
//     (jAER |dx|<=radiusX && |dy|<=radiusY), purely spatial — the old
//     cluster_time_us staleness gate could not associate and spawned
//     duplicates; cluster_time_us now only drives the recent-position
//     window / trajectory cadence.
//   * thresholdMassForVisibleCluster default 30 (jAER initDefault).
// dynamicSizeEnabled / growMergedSizeEnabled / surroundInhibition are NOT
// ported — jAER defaults them to false (rect-overlap merging IS ported).
// Output: vector<TrackedObject>. Header-only.

#ifndef GUI_ALGO_CV_OBJECT_TRACKER_H
#define GUI_ALGO_CV_OBJECT_TRACKER_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

#include <opencv2/core.hpp>

#include <metavision/sdk/base/utils/timestamp.h>

#include "algo/common/event.h"
#include "algo/common/event_packet.h"
#include "algo/cv/cluster_interface.h"
#include "algo/cv/cluster_path_point.h"

namespace gui_algo {

/// @brief A tracked object snapshot emitted by the tracker.
struct TrackedObject {
    int id{-1};
    float x{0.0F};
    float y{0.0F};
    float vx{0.0F};
    float vy{0.0F};
    cv::Rect bbox;
    std::vector<ClusterPathPoint> trajectory;
    float age{0.0F};      ///< seconds since birth
    bool visible{false};
};

/// @brief Event-level object tracker (jAER RectangularClusterTracker port).
class ObjectTracker {
public:
    ObjectTracker(int width, int height)
        : width_(width), height_(height),
          radius_(static_cast<float>(cluster_size_fraction_ *
                                     std::max(width_, height_))) {}

    // Parameters (defaults per jAER RCT / design §4.3.11) ------------------
    void set_cluster_size_fraction(double v) {
        cluster_size_fraction_ = clamp_d(v, 0.01, 0.5);
        radius_ = static_cast<float>(cluster_size_fraction_ *
                                     std::max(width_, height_));
    }
    void set_cluster_time_us(int v) { cluster_time_us_ = clamp_i(v, 1000, 50000); }
    void set_min_cluster_events(int v) { min_cluster_events_ = clamp_i(v, 10, 500); }
    void set_max_lost_age_s(double v) { max_lost_age_s_ = clamp_d(v, 0.1, 5.0); }
    void set_enable_velocity_prediction(bool v) { enable_velocity_prediction_ = v; }
    void set_location_mixing_factor(float v) {
        location_mixing_factor_ = clamp_f(v, 0.0F, 1.0F);
    }
    void set_predictive_velocity_factor(float v) {
        predictive_velocity_factor_ = clamp_f(v, 0.0F, 10.0F);
    }
    void set_mass_decay_tau_us(int v) {
        mass_decay_tau_us_ = clamp_i(v, 1, 1000000);
    }
    void set_threshold_mass_for_visible(float v) {
        threshold_mass_for_visible_ = clamp_f(v, 0.0F, 1000000.0F);
    }
    void set_max_clusters(int v) { max_clusters_ = clamp_i(v, 1, 100); }

    double cluster_size_fraction() const { return cluster_size_fraction_; }
    int cluster_time_us() const { return cluster_time_us_; }
    int min_cluster_events() const { return min_cluster_events_; }
    double max_lost_age_s() const { return max_lost_age_s_; }
    bool enable_velocity_prediction() const { return enable_velocity_prediction_; }
    float location_mixing_factor() const { return location_mixing_factor_; }
    float predictive_velocity_factor() const { return predictive_velocity_factor_; }
    int mass_decay_tau_us() const { return mass_decay_tau_us_; }
    float threshold_mass_for_visible() const { return threshold_mass_for_visible_; }
    int max_clusters() const { return max_clusters_; }
    float radius() const { return radius_; }
    int width() const { return width_; }
    int height() const { return height_; }

    /// @brief Processes a batch of events, updating clusters.
    void process(const Event* events, std::size_t count) {
        // First packet after construction/reset: anchor prev_batch_t_ to the
        // first event (-1 sentinel). With large-timestamp sources (live camera
        // t≈1e9us, cropped playback) a 0 initial value would age every cluster
        // by t0 and instantly prune them all on the next packet (§四-M1).
        if (prev_batch_t_ < 0 && count > 0) {
            prev_batch_t_ = events[0].t;
        }
        Metavision::timestamp last_t = prev_batch_t_;
        prune_lost();
        for (auto& c : clusters_) c.begin_batch();
        for (std::size_t i = 0; i < count; ++i) {
            const Event& e = events[i];
            if (e.x >= width_ || e.y >= height_) continue;
            if (e.t > last_t) last_t = e.t;
            // jAER association: nearest cluster whose RECTANGLE contains the
            // event (per-axis |dx|<=r && |dy|<=r), purely spatial.
            int best = -1;
            float best_d2 = radius_ * radius_;
            for (int k = 0; k < static_cast<int>(clusters_.size()); ++k) {
                const float dx = std::abs(clusters_[k].x() - static_cast<float>(e.x));
                const float dy = std::abs(clusters_[k].y() - static_cast<float>(e.y));
                if (dx > radius_ || dy > radius_) continue;
                const float d2 = dx * dx + dy * dy;
                if (d2 < best_d2) { best_d2 = d2; best = k; }
            }
            if (best < 0) {
                // jAER maxNumClusters hard cap: when the list is full, new
                // seeds are simply rejected.
                if (static_cast<int>(clusters_.size()) < max_clusters_) {
                    clusters_.emplace_back(next_id_++, e, radius_,
                                           cluster_time_us_,
                                           enable_velocity_prediction_,
                                           max_lost_us(),
                                           location_mixing_factor_,
                                           predictive_velocity_factor_,
                                           mass_decay_tau_us_,
                                           threshold_mass_for_visible_);
                }
            } else {
                clusters_[best].update(e);
            }
        }
        const Metavision::timestamp dt = last_t - prev_batch_t_;
        if (dt > 0) {
            for (auto& c : clusters_) c.age(dt);
        }
        prev_batch_t_ = last_t;
        merge_clusters();
        emit();
    }

    /// @brief Processes an event packet.
    void process(EventPacket& events) {
        process(events.data(), events.size());
    }

    /// @brief Returns the most recently emitted tracked objects.
    const std::vector<TrackedObject>& objects() const { return tracked_; }

    void reset() {
        clusters_.clear();
        tracked_.clear();
        next_id_ = 0;
        prev_batch_t_ = -1;  // -1 sentinel: re-anchor on the next first event
    }

private:
    static int clamp_i(int v, int lo, int hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }
    static double clamp_d(double v, double lo, double hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }
    static float clamp_f(float v, float lo, float hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }

    Metavision::timestamp max_lost_us() const {
        return static_cast<Metavision::timestamp>(max_lost_age_s_ * 1e6);
    }

    /// @brief Concrete cluster implementing ClusterInterface.
    class Cluster : public ClusterInterface {
    public:
        struct Recent { float x, y; Metavision::timestamp t; };

        Cluster(int id, const Event& seed, float radius, int cluster_time_us,
                bool enable_velocity_prediction,
                Metavision::timestamp max_lost_us,
                float location_mixing_factor,
                float predictive_velocity_factor,
                Metavision::timestamp mass_decay_tau_us,
                float threshold_mass_for_visible)
            : id_(id), last_t_(seed.t),
              radius_cap_(radius),
              cluster_time_us_(cluster_time_us),
              enable_velocity_prediction_(enable_velocity_prediction),
              max_lost_us_(max_lost_us),
              location_mixing_factor_(location_mixing_factor),
              predictive_velocity_factor_(predictive_velocity_factor),
              mass_decay_tau_us_(mass_decay_tau_us),
              threshold_mass_for_visible_(threshold_mass_for_visible),
              x_(static_cast<float>(seed.x)),
              y_(static_cast<float>(seed.y)),
              prev_x_(x_), prev_y_(y_), prev_pos_t_(seed.t),
              bbox_(static_cast<int>(seed.x), static_cast<int>(seed.y), 1, 1) {
            recent_.push_back({x_, y_, seed.t});
            maybe_push_trajectory(seed);
        }

        // ClusterInterface implementation --------------------------------
        void update(const Event& e) override {
            const Metavision::timestamp prev_t = last_t_;
            since_last_us_ = 0;
            last_t_ = e.t;
            // Leaky mass: mass = 1 + mass * exp(-dt / tau) (jAER
            // clusterMassDecayTauUs).
            {
                // jAER updateMass accumulates only across distinct
                // timestamps; simultaneous events (ALPDATA frames, seek
                // bursts) would pump the mass +1 per event with no decay
                // and instantly cross the visibility threshold.
                if (e.t > prev_t) {
                    mass_ = 1.0F + mass_ * std::exp(
                        -static_cast<float>(e.t - prev_t) /
                        static_cast<float>(mass_decay_tau_us_));
                }
            }
            push_recent(e);
            update_rct(e);
            update_velocity(e);
            maybe_push_trajectory(e);
        }

        float distance(const Event& e) const override {
            const float dx = x_ - static_cast<float>(e.x);
            const float dy = y_ - static_cast<float>(e.y);
            return std::sqrt(dx * dx + dy * dy);
        }

        bool is_visible() const override {
            return get_mass_now(last_t_) >= threshold_mass_for_visible_ &&
                   since_last_us_ <= max_lost_us_;
        }

        void age(Metavision::timestamp dt_us) override {
            since_last_us_ += dt_us;
            age_us_ += dt_us;
            if (dt_us <= 0) return;
            if (enable_velocity_prediction_) {
                const float dt_s = static_cast<float>(dt_us) * 1e-6F;
                // Clamp the per-batch extrapolation to ±radius
                // (§四-S1): without this, a noisy velocity estimate
                // integrated over a whole packet gap threw the cluster
                // hundreds of px away and every packet spawned a new cluster.
                // jAER updateClusterLocations: velocity * dt * factor,
                // once per packet -- the predictive_velocity_factor lives
                // here, not in the per-event path.
                const float dx = vx_ * dt_s * predictive_velocity_factor_;
                const float dy = vy_ * dt_s * predictive_velocity_factor_;
                x_ += (dx < -radius_cap_) ? -radius_cap_ :
                      (dx > radius_cap_ ? radius_cap_ : dx);
                y_ += (dy < -radius_cap_) ? -radius_cap_ :
                      (dy > radius_cap_ ? radius_cap_ : dy);
            }
        }

        float x() const override { return x_; }
        float y() const override { return y_; }
        float vx() const override { return vx_; }
        float vy() const override { return vy_; }
        cv::Rect bbox() const override {
            // jAER RCT draws the cluster as a rectangle of radiusX*2 x
            // radiusY*2 centred on the cluster location (aspectRatio=1 ->
            // square). The event-accumulated bbox_ ballooned to the whole
            // frame under a large radius and dense scenes; the fixed
            // radius box is the jAER visual.
            const int s = static_cast<int>(radius_cap_);
            return cv::Rect(static_cast<int>(x_) - s,
                            static_cast<int>(y_) - s, 2 * s, 2 * s);
        }
        const std::vector<ClusterPathPoint>& trajectory() const override {
            return trajectory_;
        }
        float mass() const override { return mass_; }
        Metavision::timestamp age_us() const override { return age_us_; }

        // Cluster-specific accessors ------------------------------------
        int id() const { return id_; }
        Metavision::timestamp last_t() const { return last_t_; }
        Metavision::timestamp since_last_us() const { return since_last_us_; }
        float get_mass_now(Metavision::timestamp t) const {
            return mass_ * std::exp(static_cast<float>(last_t_ - t) /
                                    static_cast<float>(mass_decay_tau_us_));
        }
        void begin_batch() {}
        bool should_prune() const {
            // Hard timeout: no events for longer than max_lost_us_.
            if (since_last_us_ > max_lost_us_) return true;
            // Mass-based pruning: decayed mass below threshold for at least
            // one mass_decay_tau_us_.
            const float decayed = get_mass_now(last_t_ + since_last_us_);
            return decayed < threshold_mass_for_visible_ &&
                   since_last_us_ >= mass_decay_tau_us_;
        }

        void absorb(Cluster& o) {
            const float m1 = mass_;
            const float m2 = o.mass_;
            const float tot = m1 + m2;
            if (tot > 0.0F) {
                x_ = (x_ * m1 + o.x_ * m2) / tot;
                y_ = (y_ * m1 + o.y_ * m2) / tot;
                vx_ = (vx_ * m1 + o.vx_ * m2) / tot;
                vy_ = (vy_ * m1 + o.vy_ * m2) / tot;
            }
            mass_ += o.mass_;
            bbox_ = bbox_ | o.bbox_;
            if (o.last_t_ > last_t_) last_t_ = o.last_t_;
        }

        /// Per-axis radius for merge tests (set by the tracker).
        void set_merge_radius(float r) { radius_cap_ = r; }
        float merge_radius() const { return radius_cap_; }

    private:
        void push_recent(const Event& e) {
            recent_.push_back({static_cast<float>(e.x),
                               static_cast<float>(e.y), e.t});
            while (!recent_.empty() && (e.t - recent_.front().t) > cluster_time_us_) {
                recent_.pop_front();
            }
            if (recent_.size() > 64) recent_.pop_front();
        }

        void update_rct(const Event& e) {
            const float m = location_mixing_factor_;
            // Per-event IIR location mixing only (jAER Cluster.addEvent ->
            // updatePosition mixes and measures, it never advances the
            // location; the velocity extrapolation happens ONCE PER PACKET
            // in updateClusterLocations -- our age() override). Advancing
            // per event on top of the per-packet advance doubled the lead
            // and fed the extrapolated displacement straight into the
            // velocity estimator (positive feedback).
            x_ = (1.0F - m) * x_ + m * static_cast<float>(e.x);
            y_ = (1.0F - m) * y_ + m * static_cast<float>(e.y);
        }

        void update_velocity(const Event& e) {
            if (prev_pos_t_ > 0 && e.t > prev_pos_t_) {
                const float dt_s =
                    static_cast<float>(e.t - prev_pos_t_) * 1e-6F;
                if (dt_s > 0.0F) {
                    // jAER RCT velocityTauMs=100ms first-order low-pass on the
                    // instantaneous velocity (alpha = min(1, dt/tau)). Without
                    // it the IIR-smoothed per-event position step (~0.05 px)
                    // divided by a us-scale dt produced huge velocity spikes
                    // that broke tracking via age() extrapolation (§四-S1).
                    const float alpha = std::min(1.0F, dt_s / 0.1F);
                    vx_ += alpha * ((x_ - prev_x_) / dt_s - vx_);
                    vy_ += alpha * ((y_ - prev_y_) / dt_s - vy_);
                }
            }
            prev_x_ = x_;
            prev_y_ = y_;
            prev_pos_t_ = e.t;
        }

        void maybe_push_trajectory(const Event& e) {
            if (trajectory_.empty() || e.t - last_traj_t_ >= cluster_time_us_) {
                trajectory_.push_back(ClusterPathPoint(x_, y_, vx_, vy_, e.t,
                                                       radius_cap_));
                if (trajectory_.size() > 500) trajectory_.erase(trajectory_.begin());
                last_traj_t_ = e.t;
            }
        }

        int id_;
        Metavision::timestamp last_t_;
        Metavision::timestamp since_last_us_{0};
        Metavision::timestamp age_us_{0};
        Metavision::timestamp last_traj_t_{0};
        float mass_{1.0F};
        float radius_cap_{10.0F};  // per-cluster radius snapshot (merge/extrapolation)
        int cluster_time_us_;
        bool enable_velocity_prediction_;
        Metavision::timestamp max_lost_us_;
        float location_mixing_factor_;
        float predictive_velocity_factor_;
        Metavision::timestamp mass_decay_tau_us_;
        float threshold_mass_for_visible_;
        float x_, y_, vx_{0.0F}, vy_{0.0F};
        float prev_x_, prev_y_;
        Metavision::timestamp prev_pos_t_;
        cv::Rect bbox_;
        std::deque<Recent> recent_;
        std::vector<ClusterPathPoint> trajectory_;
    };

    void prune_lost() {
        std::vector<Cluster> kept;
        kept.reserve(clusters_.size());
        for (auto& c : clusters_) {
            if (!c.should_prune()) kept.push_back(std::move(c));
        }
        clusters_.swap(kept);
    }

    void merge_clusters() {
        if (clusters_.size() < 2) return;
        // jAER rectangle overlap: |dx| < r1+r2 AND |dy| < r1+r2, restarting
        // the loop after each merge. Cap passes to avoid O(n^2).
        for (auto& c : clusters_) c.set_merge_radius(radius_);
        for (int pass = 0; pass < 10; ++pass) {
            bool merged = false;
            for (std::size_t i = 0; i < clusters_.size(); ++i) {
                for (std::size_t j = i + 1; j < clusters_.size();) {
                    const float dx = std::abs(clusters_[i].x() - clusters_[j].x());
                    const float dy = std::abs(clusters_[i].y() - clusters_[j].y());
                    const float rsum = clusters_[i].merge_radius() +
                                       clusters_[j].merge_radius();
                    if (dx < rsum && dy < rsum) {
                        clusters_[i].absorb(clusters_[j]);
                        clusters_.erase(clusters_.begin() +
                                        static_cast<long>(j));
                        merged = true;
                    } else {
                        ++j;
                    }
                }
            }
            if (!merged) break;
        }
    }

    void emit() {
        tracked_.clear();
        for (const auto& c : clusters_) {
            if (static_cast<int>(c.mass()) < min_cluster_events_) continue;
            if (!c.is_visible()) continue;
            TrackedObject o;
            o.id = c.id();
            o.x = c.x();
            o.y = c.y();
            o.vx = c.vx();
            o.vy = c.vy();
            o.bbox = c.bbox();
            o.trajectory = c.trajectory();
            o.age = static_cast<float>(c.age_us()) * 1e-6F;
            o.visible = true;
            tracked_.push_back(o);
        }
    }

    int width_;
    int height_;
    // jAER semantics: cluster size = FRACTION of the sensor's max dimension
    // (clusterSize initDefault 0.15); the per-axis radius is fraction*maxdim.
    // jAER clusterSize default 0.15 was tuned for ~304px DVS240 chips
    // (radius ≈ 46 px). On a 1280 px sensor 0.15 gives a 192 px radius that
    // absorbs the whole scene into one cluster (verified on screw.raw).
    // Scaled proportionally: 0.15 * 304/1280 ≈ 0.036 -> ~46 px on 1280 px.
    double cluster_size_fraction_{0.036};
    float radius_{10.0F};
    int cluster_time_us_{5000};
    int min_cluster_events_{50};
    double max_lost_age_s_{1.0};
    bool enable_velocity_prediction_{true};
    float location_mixing_factor_{0.05F};
    float predictive_velocity_factor_{1.0F};
    int mass_decay_tau_us_{10000};
    float threshold_mass_for_visible_{30.0F};  // jAER initDefault
    int max_clusters_{10};                     // jAER maxNumClusters

    std::vector<Cluster> clusters_;
    std::vector<TrackedObject> tracked_;
    int next_id_{0};
    Metavision::timestamp prev_batch_t_{-1};  // -1 = no packet seen yet
};

} // namespace gui_algo

#endif // GUI_ALGO_CV_OBJECT_TRACKER_H
