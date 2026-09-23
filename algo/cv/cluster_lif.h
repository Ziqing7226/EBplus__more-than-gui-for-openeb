// algo/cv/cluster_lif.h — LIF neuron grid clustering.
//
// ✅ 移植自 jAER BlurringTunnelFilter (ch.unizh.ini.jaer.projects.
// einsteintunnel.sensoryprocessing.BlurringTunnelFilter). 对应设计 §4.3.18。
//
// jAER 的 BlurringTunnelFilter / NeuronGroup 将同时发放（共燃）的多个 LIF
// 神经元按连通分量分组成一个簇（NeuronGroup）。本实现忠实移植其核心逻辑：
//
//   - 粗粒度神经元网格：感受野大小 receptiveFieldSizePixels（默认 8），
//     半步长 halfReceptiveFieldSizePixels = receptiveFieldSizePixels/2。
//     神经元 (i,j) 的中心像素 = ((i+1)*half, (j+1)*half)，其感受野覆盖
//     [i*half, (i+2)*half-1] × [j*half, (j+2)*half-1]。相邻神经元共享
//     一半感受野区域，提高空间分辨率。网格维度 numOfNeuronsX/Y 按 jAER
//     initFilter 的公式计算（width%half==0 时 -1）。
//   - 每个事件驱动 4 个重叠感受野神经元：subIndexX = x/half（==numOfNeuronsX
//     时减一），subIndexY = y/half（同上），4 个神经元为 (sx,sy)、
//     (sx,sy-1)、(sx-1,sy)、(sx-1,sy-1)，带边界守卫（与 jAER blurring()
//     一致）。每个被驱动的神经元按 LIF 积分：膜电位 +1.0 并按 tau 指数
//     泄漏，越过阈值即发放并部分下降（保留残余电位）。
//   - 初始膜电位 = initial_potential_percent * threshold / 100（有意修正
//     jAER LIFNeuron.reset 漏除 100 的上游 bug：jAER 实为
//     MPInitialPercnetTh * MPThreshold，默认 50×15=750；本实现按百分数
//     语义计算，默认 50% → 7.5）。
//   - 发放后处理 = MP -= max(jump_after_firing_percent * threshold / 100,
//     1.0)（jAER MPJumpAfterFiringPercentTh，默认 10% → 1.5，下限 1.0）。
//   - 非单调时间戳：时间回退时 MP=0（与 jAER incrementMP 一致）。
//
// 在每个事件包处理完成后按 jAER updateNeurons 的 inside/border 全包围语义
// 分组：只有全部邻居均在发放的神经元是 INSIDE，组 = INSIDE 神经元 + 其直接
// 发放邻居（BORDER）；无 INSIDE 核的稀疏发放不成组（P3：旧 4-CC 近似会产出
// jAER 没有的小簇）。簇质心按 jAER NeuronGroup.add 折叠：成员按发放时间升序
// 逐个并入，较旧一侧乘 exp(dt/tau) 时间折扣，权重 = numSpikes
// (effectiveMP)；并记录神经元数 (size)、总发放数 (mass)、外接框；再按最近邻
// 匹配跨包跟踪并估计速度。输出: vector<LifCluster>，每个含
// (track_id, cx, cy, size, mass, vx, vy)。Header-only.

#ifndef GUI_ALGO_CV_CLUSTER_LIF_H
#define GUI_ALGO_CV_CLUSTER_LIF_H

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

#include <metavision/sdk/base/utils/timestamp.h>

#include "algo/common/event.h"
#include "algo/common/event_packet.h"
#include "algo/common/lif_integrator.h"

namespace gui_algo {

/// @brief A cluster emitted when one or more co-firing LIF neurons form a
/// connected component (移植自 jAER BlurringTunnelFilter NeuronGroup).
struct LifCluster {
    cv::Point2f position;             ///< Mass-weighted centroid (cx, cy) in sensor pixels.
    double potential{0.0};            ///< Membrane potential at firing (== threshold).
    int track_id{-1};                 ///< Persistent track id, -1 if untracked.
    int size{0};                      ///< Number of distinct firing neurons.
    int mass{0};                      ///< Total firings summed over the component.
    float vx{0.0F};                   ///< Track velocity x (px/s).
    float vy{0.0F};                   ///< Track velocity y (px/s).
    cv::Rect bbox;                    ///< Bounding box of firing neuron centres (sensor px).
    Metavision::timestamp last_t{0};  ///< Timestamp of the batch that produced it.
};

/// @brief LIF neuron grid clustering with connected-component grouping,
/// faithfully ported from jAER BlurringTunnelFilter.
class ClusterLIF {
public:
    /// @brief Constructs the cluster finder.
    /// @param width,height Sensor dimensions (pixels).
    /// @param tau_ms LIF membrane time constant (jAER MPTimeConstantUs, us).
    /// @param threshold Firing threshold (jAER MPThreshold).
    /// @param reset_value Post-fire potential when jump_after_firing_percent
    ///        == 0 (legacy full-reset; ignored when jump > 0).
    /// @param receptive_field_size_pixels Receptive field side length in
    ///        sensor pixels (jAER receptiveFieldSizePixels, default 8).
    /// @param initial_potential_percent Initial MP as percent of threshold
    ///        (jAER MPInitialPercnetTh, default 50).
    /// @param jump_after_firing_percent MP drop after firing as percent of
    ///        threshold (jAER MPJumpAfterFiringPercentTh, default 10).
    ClusterLIF(int width, int height,
               float tau_ms = 22.0f,
               float threshold = 15.0f,
               float reset_value = 0.0f,
               int receptive_field_size_pixels = 8,
               float initial_potential_percent = 50.0f,
               float jump_after_firing_percent = 10.0f)
        : width_(width), height_(height),
          tau_us_(static_cast<Metavision::timestamp>(tau_ms * 1000.0f)),
          threshold_(static_cast<double>(threshold)),
          reset_value_(static_cast<double>(reset_value)),
          receptive_field_size_pixels_(receptive_field_size_pixels),
          initial_potential_percent_(static_cast<double>(initial_potential_percent)),
          jump_after_firing_percent_(static_cast<double>(jump_after_firing_percent)),
          // Placeholder; rebuild_grid() (called below) reassigns lif_ to the
          // correctly-sized coarse grid. Constructed with valid args so the
          // member is initialised before reassignment.
          lif_(1, 1, tau_us_, threshold_, reset_value_, 1000,
               static_cast<double>(initial_potential_percent),
               static_cast<double>(jump_after_firing_percent)),
          track_tol_px_(10.0f),
          stale_us_(1000000) {
        rebuild_grid();
    }

    /// @brief Processes an event packet and returns one cluster per
    /// 4-connected component of co-firing LIF neurons (移植自 jAER
    /// BlurringTunnelFilter).
    std::vector<LifCluster> process(const EventPacket& packet) {
        std::vector<LifCluster> result;
        if (packet.empty()) return result;
        if (num_neurons_x_ <= 0 || num_neurons_y_ <= 0) return result;

        // 1) Drive the LIF integrator: route each event to its 4 overlapping
        //    receptive-field neurons (jAER blurring), collecting firing
        //    neurons into a binary mask + per-neuron firing count (used as
        //    the centroid/mass weight, mirroring jAER's
        //    effectiveMP = numSpikes weighting in NeuronGroup.add).
        std::fill(fire_mask_.begin(), fire_mask_.end(), uint8_t(0));
        std::fill(fire_count_.begin(), fire_count_.end(), 0);
        Metavision::timestamp batch_t = 0;
        for (const Event& e : packet) {
            if (e.x >= width_ || e.y >= height_) continue;
            if (e.t > batch_t) batch_t = e.t;
            route_event(e);
        }

        // 2) jAER inside/border full-enclosure grouping (updateNeurons): a
        //    neuron is FIRING_INSIDE only when ALL its grid neighbours are
        //    firing; its firing neighbours become the group BORDER. Groups
        //    form ONLY around an inside neuron — sparse firing with no fully
        //    enclosed core yields no cluster at all (the former 4-CC
        //    approximation produced small clusters jAER never emits).
        const std::size_t n = static_cast<std::size_t>(num_neurons_x_) *
                              num_neurons_y_;
        // Eligible mask: inside neurons + firing neurons adjacent to an
        // inside neuron (jAER updateGroup adds exactly the inside neuron and
        // its firing neighbours).
        eligible_.assign(n, 0);
        inside_.assign(n, 0);
        bool any_inside = false;
        for (int y = 0; y < num_neurons_y_; ++y) {
            for (int x = 0; x < num_neurons_x_; ++x) {
                const std::size_t idx =
                    static_cast<std::size_t>(y) * num_neurons_x_ + x;
                if (!fire_mask_[idx]) continue;
                bool all = true;
                if (x > 0 && !fire_mask_[idx - 1]) all = false;
                if (x + 1 < num_neurons_x_ &&
                    !fire_mask_[idx + 1]) all = false;
                if (y > 0 && !fire_mask_[idx - num_neurons_x_]) all = false;
                if (y + 1 < num_neurons_y_ &&
                    !fire_mask_[idx + num_neurons_x_]) all = false;
                if (all) {
                    inside_[idx] = 1;
                    any_inside = true;
                    eligible_[idx] = 1;
                    if (x > 0) eligible_[idx - 1] = 1;
                    if (x + 1 < num_neurons_x_) eligible_[idx + 1] = 1;
                    if (y > 0) eligible_[idx - num_neurons_x_] = 1;
                    if (y + 1 < num_neurons_y_) eligible_[idx + num_neurons_x_] = 1;
                }
            }
        }
        if (!any_inside) return result;  // jAER emits no group without a core

        // 4-connectivity BFS within the eligible set.
        label_.assign(n, -1);
        auto& label = label_;
        struct Member { double px, py, wt; Metavision::timestamp ts; };
        struct Comp {
            std::vector<Member> members;
            double cx{0.0}, cy{0.0};      // time-discounted weighted centroid
            double total_mp{0.0};         // discounted effectiveMP sum
            int raw_mass{0};              // raw spike count sum
            int minx{0}, miny{0}, maxx{0}, maxy{0};
        };
        std::vector<Comp> comps;
        std::vector<std::size_t> stack;
        stack.reserve(n);
        static const int dx[4] = {-1, 1, 0, 0};
        static const int dy[4] = {0, 0, -1, 1};
        const std::size_t w = static_cast<std::size_t>(num_neurons_x_);
        const int wi = num_neurons_x_;
        const int hi = num_neurons_y_;
        for (int y = 0; y < hi; ++y) {
            for (int x = 0; x < wi; ++x) {
                const std::size_t idx = static_cast<std::size_t>(y) * w + x;
                if (!eligible_[idx] || label[idx] >= 0) continue;
                const int cid = static_cast<int>(comps.size());
                comps.emplace_back();
                Comp& c = comps.back();
                c.minx = neuron_centre_x(x); c.maxx = c.minx;
                c.miny = neuron_centre_y(y); c.maxy = c.miny;
                stack.clear();
                stack.push_back(idx);
                label[idx] = cid;
                while (!stack.empty()) {
                    const std::size_t cur = stack.back();
                    stack.pop_back();
                    const int cx = static_cast<int>(cur % w);
                    const int cy = static_cast<int>(cur / w);
                    const int px = neuron_centre_x(cx);
                    const int py = neuron_centre_y(cy);
                    c.members.push_back(Member{
                        static_cast<double>(px), static_cast<double>(py),
                        static_cast<double>(fire_count_[cur]), fire_t_[cur]});
                    if (px < c.minx) c.minx = px;
                    if (px > c.maxx) c.maxx = px;
                    if (py < c.miny) c.miny = py;
                    if (py > c.maxy) c.maxy = py;
                    for (int k = 0; k < 4; ++k) {
                        const int nx = cx + dx[k];
                        const int ny = cy + dy[k];
                        if (nx < 0 || ny < 0 || nx >= wi || ny >= hi) continue;
                        const std::size_t nidx =
                            static_cast<std::size_t>(ny) * w + nx;
                        if (!eligible_[nidx] || label[nidx] >= 0) continue;
                        label[nidx] = cid;
                        stack.push_back(nidx);
                    }
                }
            }
        }

        // 2b) jAER NeuronGroup.add centroid: fold members in ascending spike
        //     time; the OLDER side of each pair is discounted by
        //     exp(dt/tau) (jAER leakyFactor), weight = numSpikes
        //     (effectiveMP when MPJumpAfterFiring > 0).
        const double tau = static_cast<double>(tau_us_);
        for (auto& c : comps) {
            std::sort(c.members.begin(), c.members.end(),
                      [](const Member& a, const Member& b) { return a.ts < b.ts; });
            double lx = 0.0, ly = 0.0, total = 0.0;
            Metavision::timestamp gt = 0;
            bool first = true;
            int raw_mass = 0;
            for (const auto& m : c.members) {
                raw_mass += static_cast<int>(m.wt);
                if (first) {
                    lx = m.px; ly = m.py; total = m.wt; gt = m.ts;
                    first = false;
                    continue;
                }
                if (gt < m.ts) {
                    const double leaky = std::exp(
                        static_cast<double>(gt - m.ts) / tau);
                    const double prev = total;
                    total = m.wt + prev * leaky;
                    lx = (m.px * m.wt + lx * prev * leaky) / total;
                    ly = (m.py * m.wt + ly * prev * leaky) / total;
                    gt = m.ts;
                } else {
                    const double leaky = std::exp(
                        static_cast<double>(m.ts - gt) / tau);
                    const double prev = total;
                    total = prev + m.wt * leaky;
                    lx = (m.px * m.wt * leaky + lx * prev) / total;
                    ly = (m.py * m.wt * leaky + ly * prev) / total;
                }
            }
            c.cx = lx; c.cy = ly; c.total_mp = total;
        }
        // 3) Drop stale tracks, then build one cluster per component and match
        //    it to the nearest existing track (nearest-neighbour association).
        prune_tracks(batch_t);
        result.reserve(comps.size());
        for (const auto& c : comps) {
            LifCluster out;
            out.position = cv::Point2f(static_cast<float>(c.cx),
                                       static_cast<float>(c.cy));
            out.potential = threshold_;
            out.size = static_cast<int>(c.members.size());
            out.mass = c.raw_mass;
            out.bbox = cv::Rect(c.minx, c.miny,
                                c.maxx - c.minx + 1,
                                c.maxy - c.miny + 1);
            out.last_t = batch_t;
            out.track_id = associate(out);
            result.push_back(out);
        }
        return result;
    }

    // Parameter accessors ---------------------------------------------------
    float tau_ms() const {
        return static_cast<float>(tau_us_) / 1000.0f;
    }
    float threshold() const { return static_cast<float>(threshold_); }
    float reset_value() const { return static_cast<float>(reset_value_); }
    int receptive_field_size_pixels() const {
        return receptive_field_size_pixels_;
    }
    float initial_potential_percent() const {
        return static_cast<float>(initial_potential_percent_);
    }
    float jump_after_firing_percent() const {
        return static_cast<float>(jump_after_firing_percent_);
    }
    int num_neurons_x() const { return num_neurons_x_; }
    int num_neurons_y() const { return num_neurons_y_; }

    void set_tau_ms(float v) {
        // A zero tau turns the decay exp((gt - ts)/tau) into 0/0 = NaN at
        // the first pair of equal timestamps (ALPDATA-style constant-t
        // streams). The GUI range clamps to >= 1 ms, this guards the API.
        if (v < 0.001f) v = 0.001f;
        tau_us_ = static_cast<Metavision::timestamp>(v * 1000.0f);
        lif_.set_tau_us(tau_us_);
    }
    void set_threshold(float v) {
        threshold_ = static_cast<double>(v);
        lif_.set_threshold(threshold_);
    }
    void set_reset_value(float v) { reset_value_ = static_cast<double>(v); }
    void set_initial_potential_percent(float v) {
        initial_potential_percent_ = static_cast<double>(v);
        lif_.set_initial_potential_percent(initial_potential_percent_);
    }
    void set_jump_after_firing_percent(float v) {
        jump_after_firing_percent_ = static_cast<double>(v);
        lif_.set_jump_after_firing_percent(jump_after_firing_percent_);
    }
    /// @brief Changes the receptive field size and rebuilds the neuron grid
    /// (jAER setReceptiveFieldSizePixels → initFilter). All neuron state is
    /// lost; tracks are preserved.
    void set_receptive_field_size_pixels(int v) {
        if (v == receptive_field_size_pixels_) return;
        receptive_field_size_pixels_ = v;
        rebuild_grid();
    }

    /// @brief Returns the membrane-potential grid of the coarse neuron array
    /// (size = num_neurons_x() * num_neurons_y(), CV_32F-compatible layout).
    const std::vector<double>& potential_grid() const {
        return lif_.potential_grid();
    }

    void reset() {
        lif_.clear();
        tracks_.clear();
        next_track_id_ = 0;
        std::fill(fire_mask_.begin(), fire_mask_.end(), uint8_t(0));
        std::fill(fire_count_.begin(), fire_count_.end(), 0);
    }

private:
    struct Track {
        int id{-1};
        cv::Point2f last_position;
        Metavision::timestamp last_t{0};
        float vx{0.0F};
        float vy{0.0F};
    };

    /// @brief (Re)builds the coarse neuron grid from the sensor dimensions and
    /// the current receptive_field_size_pixels_ (jAER initFilter). Sizes the
    /// LifIntegrator, fire_mask_, and fire_count_ to the new grid.
    void rebuild_grid() {
        if (receptive_field_size_pixels_ < 2) {
            receptive_field_size_pixels_ = 2;
        }
        half_rf_ = receptive_field_size_pixels_ / 2;

        // jAER initFilter neuron-count formula: when the sensor size divides
        // evenly by half_rf, subtract one (the last cell has no neuron
        // beyond the boundary); otherwise floor-divide.
        if (width_ % half_rf_ == 0) {
            num_neurons_x_ = width_ / half_rf_ - 1;
        } else {
            num_neurons_x_ = width_ / half_rf_;
        }
        if (height_ % half_rf_ == 0) {
            num_neurons_y_ = height_ / half_rf_ - 1;
        } else {
            num_neurons_y_ = height_ / half_rf_;
        }
        if (num_neurons_x_ < 1) num_neurons_x_ = 1;
        if (num_neurons_y_ < 1) num_neurons_y_ = 1;

        // Reassign the integrator to the new coarse-grid dimensions. The
        // LifIntegrator is trivially copy/move-assignable (POD + vectors).
        lif_ = LifIntegrator(num_neurons_x_, num_neurons_y_,
                             tau_us_, threshold_, reset_value_,
                             1000,
                             initial_potential_percent_,
                             jump_after_firing_percent_);
        const std::size_t n = static_cast<std::size_t>(num_neurons_x_) *
                              num_neurons_y_;
        fire_mask_.assign(n, 0);
        fire_count_.assign(n, 0);
        fire_t_.assign(n, 0);
        eligible_.assign(n, 0);
        inside_.assign(n, 0);
    }

    /// @brief Routes an event to its 4 overlapping receptive-field neurons
    /// (jAER blurring): each addressed neuron receives +1.0 via the LIF
    /// integrator. Firing neurons are flagged in fire_mask_ and their
    /// per-packet spike count accumulated in fire_count_.
    void route_event(const Event& e) {
        // subIndexX/Y locate the 2×2 neuron block whose RFs cover (x,y).
        int sx = e.x / half_rf_;
        if (sx == num_neurons_x_) --sx;
        int sy = e.y / half_rf_;
        if (sy == num_neurons_y_) --sy;
        if (sx < 0) sx = 0;
        if (sy < 0) sy = 0;

        // 4 overlapping RF neurons with jAER boundary guards.
        if (sx != num_neurons_x_ && sy != num_neurons_y_) {
            feed_neuron(sx, sy, e);
        }
        if (sx != num_neurons_x_ && sy != 0) {
            feed_neuron(sx, sy - 1, e);
        }
        if (sx != 0 && sy != num_neurons_y_) {
            feed_neuron(sx - 1, sy, e);
        }
        if (sy != 0 && sx != 0) {
            feed_neuron(sx - 1, sy - 1, e);
        }
    }

    /// @brief Drives a single neuron with the event and records any spike.
    void feed_neuron(int nx, int ny, const Event& e) {
        const bool fired = lif_.add_event(
            static_cast<std::uint16_t>(nx),
            static_cast<std::uint16_t>(ny),
            e.p, e.t);
        if (fired) {
            const std::size_t idx =
                static_cast<std::size_t>(ny) * num_neurons_x_ + nx;
            fire_mask_[idx] = 1;
            ++fire_count_[idx];
            fire_t_[idx] = e.t;  // last spike time (NeuronGroup.add discount)
        }
    }

    /// @brief Neuron (i,j) centre in sensor pixels (jAER: (i+1)*half_rf).
    int neuron_centre_x(int i) const { return (i + 1) * half_rf_; }
    int neuron_centre_y(int j) const { return (j + 1) * half_rf_; }

    void prune_tracks(Metavision::timestamp now) {
        if (tracks_.size() <= 1) return;
        std::vector<Track> kept;
        kept.reserve(tracks_.size());
        // Keep tracks updated within stale_us_ of the current batch time.
        // (now < tr.last_t → non-monotonic clock → negative diff ≤ stale: keep.)
        for (auto& tr : tracks_) {
            if ((now - tr.last_t) <= stale_us_) kept.push_back(std::move(tr));
        }
        if (kept.empty() && !tracks_.empty()) kept.push_back(std::move(tracks_.front()));
        tracks_.swap(kept);
    }

    int associate(LifCluster& c) {
        const float tol2 = track_tol_px_ * track_tol_px_;
        int best_id = -1;
        float best_d2 = tol2;
        Track* best_track = nullptr;
        for (auto& tr : tracks_) {
            const float dx = tr.last_position.x - c.position.x;
            const float dy = tr.last_position.y - c.position.y;
            const float d2 = dx * dx + dy * dy;
            if (d2 < best_d2) { best_d2 = d2; best_id = tr.id; best_track = &tr; }
        }
        if (best_id < 0) {
            best_id = next_track_id_++;
            tracks_.push_back(Track{best_id, c.position, c.last_t, 0.0F, 0.0F});
            c.vx = 0.0F;
            c.vy = 0.0F;
        } else {
            const float dt_s = (c.last_t > best_track->last_t)
                ? static_cast<float>(c.last_t - best_track->last_t) * 1e-6F
                : 0.0F;
            float vx = 0.0F;
            float vy = 0.0F;
            if (dt_s > 0.0F) {
                vx = (c.position.x - best_track->last_position.x) / dt_s;
                vy = (c.position.y - best_track->last_position.y) / dt_s;
            }
            best_track->vx = vx;
            best_track->vy = vy;
            best_track->last_position = c.position;
            best_track->last_t = c.last_t;
            c.vx = vx;
            c.vy = vy;
        }
        return best_id;
    }

    int width_;
    int height_;
    Metavision::timestamp tau_us_;
    double threshold_;
    double reset_value_;
    int receptive_field_size_pixels_;
    int half_rf_{0};
    int num_neurons_x_{0};
    int num_neurons_y_{0};
    double initial_potential_percent_;
    double jump_after_firing_percent_;
    LifIntegrator lif_;
    float track_tol_px_;
    Metavision::timestamp stale_us_;
    std::vector<uint8_t> fire_mask_;
    std::vector<int> fire_count_;
    std::vector<Metavision::timestamp> fire_t_;  // per-neuron last spike time
    std::vector<uint8_t> eligible_;              // inside + their firing neighbours
    std::vector<uint8_t> inside_;
    std::vector<int> label_;  // reused per-packet label buffer (OPT-27)
    std::vector<Track> tracks_;
    int next_track_id_{0};
};

} // namespace gui_algo

#endif // GUI_ALGO_CV_CLUSTER_LIF_H
