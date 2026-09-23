// algo/cv/line_segment_detector.h — ELiSeD event-level line segment detection.
//
// 完整移植 jAER ELiSeD (EBCCSP2016, ch.unizh.ini.jaer.projects.elised) 的
// 持久 LineSupport 支撑区机制 (P2, 2026-08-22)：
//   * 8000 事件环形缓冲 (jAER bufferSize) 驱动逐像素 LevelLinePixel 的
//     出生/过期；过期像素从所属支撑区移除，支撑区随之为空则删除。
//   * 逐事件 assignGradient：per-polarity 时间戳图上的 3x3 Sobel，邻居缺失
//     或超 maxAge 时取对侧镜像像素 (predictTimestamps，默认开)，按实际使用
//     核权重归一化 (sumAbsSobelXFieldsUsed)。
//   * assignSupportRegion：sobel 邻域内找朝向一致 (toleranceAngle) 的候选，
//     最优 = 最老支撑区；像素加入支撑区前过密度门 (addOnlyIfDensityHigh /
//     minDensity / orAddIfDensityIncreases)，支撑区之间按
//     mergeOnlyIfLinesAligned + distanceSegmentToSegment(maxDistance) 合并，
//     合并到较老支撑区 (jAER merge 到 creationTime 更早者)。
//   * 每包 checkSupport：宽度 (次要轴) ≥ widthToSplit 的支撑区分裂
//     (split: 移除全部像素再逐个重新指派)。
//   * LineSupport 用图像矩 (m00..m02) 增量维护：质心、主轴朝向
//     (0.5*atan(2*u11/(u20-u02)) + u02>u20 的 90° 修正)、length/width =
//     sqrt(6*(trace±disc))、端点 = 质心 ± 主轴单位向量 * length/2。
//   * isLineSegment = m00 >= minLineSupport。
// 标注层 (jAER drawLineSegments)：黑色粗线段 + 质心处可选朝向/宽度文字
// (annotateAngles / annotateWidth，默认关)。
//
// 参考: C. Brandli, J. Strubel, S. Keller, D. Scaramuzza, T. Delbruck,
// "ELiSeD - An Event-Based Line Segment Detector," EBCCSP 2016.
// Header-only.

#ifndef GUI_ALGO_CV_LINE_SEGMENT_DETECTOR_H
#define GUI_ALGO_CV_LINE_SEGMENT_DETECTOR_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <unordered_set>
#include <vector>

#include <opencv2/core.hpp>

#include <metavision/sdk/base/utils/timestamp.h>

#include "algo/common/event.h"
#include "algo/common/event_packet.h"

namespace gui_algo {

/// @brief Detected line segment (jAER LineSupport output).
struct LineSegment {
    cv::Point2f start;
    cv::Point2f end;
    float angle{0.0f};    ///< Orientation in degrees, [0, 180).
    int track_id{-1};     ///< Unused (jAER has no segment tracking); kept for API.
    float width{0.0f};    ///< Minor-axis width in px (jAER getWidth).
    float length{0.0f};   ///< Major-axis length in px (jAER getLength).
};

/// @brief ELiSeD line segment detector — full jAER LineSupport port (P2).
class LineSegmentDetector {
public:
    static constexpr int kDefaultMaxAgeUs = 40000;   // jAER maxAge
    static constexpr int kDefaultBufferSize = 2500;  // paper: "typically 2500 to 8000"

    LineSegmentDetector(int width, int height,
                        int min_line_length_px = 20,
                        int max_line_gap_px = 5)
        : width_(width), height_(height),
          min_line_length_px_(min_line_length_px),
          max_line_gap_px_(max_line_gap_px),
          on_ts_(static_cast<std::size_t>(width) * height, -1),
          off_ts_(static_cast<std::size_t>(width) * height, -1) {
        pixelmap_.resize(static_cast<std::size_t>(width) * height);
        buf_count_.assign(static_cast<std::size_t>(width) * height, 0);
    }

    /// @brief Processes an event packet; returns line segments (jAER
    /// filterPacket: addEvent per event, then split check, then draw).
    std::vector<LineSegment> process(const EventPacket& packet) {
        std::vector<LineSegment> result;
        if (packet.empty()) return result;
        LineSupport::min_support_ = static_cast<float>(min_line_length_px_);
        for (const Event& e : packet) {
            if (e.x >= width_ || e.y >= height_) continue;
            add_event(e);
        }
        check_support();
        const Metavision::timestamp now = packet[packet.size() - 1].t;
        for (auto* ls : supports_) {
            if (ls == nullptr || !ls->is_line_segment()) continue;
            if (now - ls->latest_update() > static_cast<Metavision::timestamp>(max_age_us_)) {
                continue;  // stale support: edge moved away, drop the segment
            }
            ls->update_endpoints();
            LineSegment out;
            out.start = ls->endpoint1();
            out.end = ls->endpoint2();
            out.angle = ls->orientation_deg();
            out.width = ls->width();
            out.length = ls->length();
            result.push_back(out);
        }
        return result;
    }

    // Parameter accessors ---------------------------------------------------
    int min_line_length_px() const { return min_line_length_px_; }
    int max_line_gap_px() const { return max_line_gap_px_; }
    int max_age_us() const { return max_age_us_; }
    int num_orientations() const { return 0; }  // legacy accessor (obsoleted)
    void set_min_line_length_px(int v) { min_line_length_px_ = v; }
    void set_max_line_gap_px(int v) { max_line_gap_px_ = v; }
    void set_max_age_us(int v) { max_age_us_ = v; }
    void set_num_orientations(int /*v*/) {}  // legacy accessor (obsoleted)

    void reset() {
        std::fill(on_ts_.begin(), on_ts_.end(),
                  static_cast<Metavision::timestamp>(-1));
        std::fill(off_ts_.begin(), off_ts_.end(),
                  static_cast<Metavision::timestamp>(-1));
        for (auto* ls : supports_) delete ls;
        supports_.clear();
        for (auto& p : pixelmap_) p = LevelLinePixel();
        index_buffer_.clear();
        std::fill(buf_count_.begin(), buf_count_.end(), 0);
        next_support_id_ = 0;
    }

private:
    static constexpr int kSobelRadius = 1;  // sobelWidth=3 -> radius 1
    static constexpr float kPi = 3.14159265358979323846F;
    // jAER defaults
    static constexpr float kToleranceAngleDeg = 30.0F;
    static constexpr float kMaxDistancePx = 3.5F;
    static constexpr bool kMergeOnlyIfLinesAligned = true;
    static constexpr bool kAddOnlyIfDensityHigh = true;
    static constexpr float kMinDensity = 0.3F;
    static constexpr bool kOrAddIfDensityIncreases = true;
    static constexpr float kMinSupportForDensityTest = 100.0F;
    static constexpr float kWidthToSplit = 6.0F;
    static constexpr bool kSplittingActive = true;

    /// A pixel resident in the ring buffer with a possibly-assigned level line.
    struct LevelLinePixel {
        bool buffered{false};
        bool has_ll{false};
        float angle{0.0F};      // level-line angle in degrees (-180, 180]
        float angle90{0.0F};    // angle mod 180 ([-90,90)) for merge tests
        float magnitude{0.0F};
        Metavision::timestamp ts{0};
        short polarity{0};
        std::uint16_t x{0}, y{0};
        int support{-1};        // index into supports_ (-1 = none)

        void clear() {
            buffered = false; has_ll = false; support = -1;
            angle = angle90 = magnitude = 0.0F; ts = 0; polarity = 0;
        }
    };

    /// Persistent line support region (jAER LineSupport): incremental image
    /// moments + level-line angle aggregation.
    class LineSupport {
    public:
        LineSupport(int id) : id_(id) {
            creation_time_ = 0;
            latest_update_ = 0;
        }

        void add(const LevelLinePixel& p, int idx) {
            if (creation_time_ == 0) creation_time_ = p.ts;
            if (p.ts > latest_update_) latest_update_ = p.ts;
            pixel_ids_.push_back(idx);
            ++m00_;
            // Exact integer accumulation (jAER LineSupport keeps int
            // moments): float sums of x² reach ~1e6 per term and lose
            // integer precision past 2^24, and the central-moment
            // subtraction below then amplifies the rounding into
            // negative-variance garbage (width() = NaN) on thin lines.
            m10_ += p.x;
            m01_ += p.y;
            m11_ += static_cast<std::int64_t>(p.x) * p.y;
            m20_ += static_cast<std::int64_t>(p.x) * p.x;
            m02_ += static_cast<std::int64_t>(p.y) * p.y;
            // distinguishOpposingGradients default TRUE in jAER: use angle.
            sum_angle_unit_x_ += std::cos(p.angle * kPi / 180.0F);
            sum_angle_unit_y_ += std::sin(p.angle * kPi / 180.0F);
            ll_magnitude_sum_ += p.magnitude;
            update_properties();  // jAER LineSupport.add ends with updateLineProperties()
        }

        void remove(const LevelLinePixel& p) {
            --m00_;
            m10_ -= p.x;
            m01_ -= p.y;
            m11_ -= static_cast<std::int64_t>(p.x) * p.y;
            m20_ -= static_cast<std::int64_t>(p.x) * p.x;
            m02_ -= static_cast<std::int64_t>(p.y) * p.y;
            sum_angle_unit_x_ -= std::cos(p.angle * kPi / 180.0F);
            sum_angle_unit_y_ -= std::sin(p.angle * kPi / 180.0F);
            ll_magnitude_sum_ -= p.magnitude;
            update_properties();
        }

        /// jAER LineSupport.merge appends the second segment's pixels and
        /// repoints each pixel's support to this segment. Merging only the
        /// moments (the former behaviour) left this.m00 counting pixels that
        /// were destroyed with the other segment — mass never reached zero,
        /// supports accumulated unboundedly on dense scenes (verified on
        /// screw.raw: 5.6k -> 17.6k supports across 4 windows). This variant
        /// repoints the pixels via the caller-supplied pixel map.
        void merge(LineSupport& other, int this_support_idx,
                   std::vector<LevelLinePixel>& pixelmap) {
            m00_ += other.m00_;
            m10_ += other.m10_;
            m01_ += other.m01_;
            m11_ += other.m11_;
            m20_ += other.m20_;
            m02_ += other.m02_;
            sum_angle_unit_x_ += other.sum_angle_unit_x_;
            sum_angle_unit_y_ += other.sum_angle_unit_y_;
            ll_magnitude_sum_ += other.ll_magnitude_sum_;
            if (other.latest_update_ > latest_update_) latest_update_ = other.latest_update_;
            // Repoint the other segment's pixels to THIS support and absorb
            // them into this pixel list (jAER add() per pixel).
            for (const int id : other.pixel_ids()) {
                if (pixelmap[static_cast<std::size_t>(id)].support == other_support_idx_) {
                    pixelmap[static_cast<std::size_t>(id)].support = this_support_idx;
                    pixel_ids_.push_back(id);
                }
            }
            other_support_idx_ = -1;
            update_properties();
        }

        float ll_angle_deg() const {
            if (m00_ == 0) return 0.0F;
            float t = std::atan2(sum_angle_unit_x_, sum_angle_unit_y_) * 180.0F / kPi;
            if (t == -180.0F) t = 180.0F;
            return t;
        }

        void update_properties() {
            if (m00_ == 0) { length_ = 0.0F; return; }
            const double n = static_cast<double>(m00_);
            center_x_ = static_cast<float>(m10_ / n);
            center_y_ = static_cast<float>(m01_ / n);
            // Central moments in double: the m20/m00 − cx² form subtracts
            // two large near-equal numbers, which is where the float
            // accumulation error used to surface (negative variance ⇒
            // sqrt(negative) ⇒ NaN width, orientation garbage).
            const double u20 = m20_ / n - static_cast<double>(center_x_) * center_x_;
            const double u02 = m02_ / n - static_cast<double>(center_y_) * center_y_;
            const double u11 = m11_ / n - static_cast<double>(center_x_) * center_y_;
            float ori = 0.0F;
            if (std::fabs(u20 - u02) > 1e-6) {
                ori = 0.5F * static_cast<float>(
                    std::atan((2.0 * u11) / (u20 - u02)) * 180.0 / kPi);
            }
            if (u02 > u20) {
                ori -= 90.0F;
                if (ori < -90.0F) ori += 180.0F;
            }
            orientation_deg_ = ori;
            const double a = u20, b = 2.0 * u11, c = u02;
            const double disc = std::sqrt(b * b + (a - c) * (a - c));
            // The quadratic-form widths are theoretically >= 0; rounding at
            // extreme aspect ratios can still probe a hair negative.
            width_ = static_cast<float>(
                std::sqrt(std::max(0.0, 6.0 * (a + c - disc))));
            length_ = static_cast<float>(
                std::sqrt(std::max(0.0, 6.0 * (a + c + disc))));
            update_endpoints();
        }

        void update_endpoints() {
            const float unit_x = std::cos(orientation_deg_ * kPi / 180.0F);
            const float unit_y = std::sin(orientation_deg_ * kPi / 180.0F);
            const float s = length_ * 0.5F;
            ep1_ = cv::Point2f(center_x_ + unit_x * s, center_y_ + unit_y * s);
            ep2_ = cv::Point2f(center_x_ - unit_x * s, center_y_ - unit_y * s);
        }

        bool is_line_segment() const {
            return static_cast<float>(m00_) >= min_support_; }
        int id() const { return id_; }
        float center_x() const { return center_x_; }
        float center_y() const { return center_y_; }
        float orientation_deg() const { return orientation_deg_; }
        float width() const { return width_; }
        float length() const { return length_; }
        float mass() const { return m00_; }
        Metavision::timestamp creation_time() const { return creation_time_; }
        Metavision::timestamp latest_update() const { return latest_update_; }
        void set_self_support_idx(int i) { other_support_idx_ = i; }
        const std::vector<int>& pixel_ids() const { return pixel_ids_; }
        cv::Point2f endpoint1() const { return ep1_; }
        cv::Point2f endpoint2() const { return ep2_; }

        static float min_support_;

    private:
        int id_;
        int other_support_idx_{-1};  // this support's index in supports_ (merge repoint)
        std::vector<int> pixel_ids_;  // member pixel indices (jAER HashSet)
        Metavision::timestamp creation_time_{0};
        Metavision::timestamp latest_update_{0};
        int m00_{0};
        std::int64_t m10_{0}, m01_{0}, m11_{0}, m20_{0}, m02_{0};
        float sum_angle_unit_x_{0}, sum_angle_unit_y_{0};
        float ll_magnitude_sum_{0};
        float center_x_{0}, center_y_{0};
        float orientation_deg_{0};
        float width_{0}, length_{0};
        cv::Point2f ep1_, ep2_;
    };

    // ------------------------------------------------------------------
    // jAER ELiSeD event pipeline
    // ------------------------------------------------------------------
    void add_event(const Event& e) {
        // jAER addEvent: update per-polarity timestamp map (gradient source).
        std::vector<Metavision::timestamp>& ts_map = e.p ? on_ts_ : off_ts_;
        ts_map[static_cast<std::size_t>(e.y) * width_ + e.x] = e.t;

        // bufferEvent: evict the oldest buffered index, then (re)assign this
        // pixel's gradient and support.
        if (index_buffer_.size() >= kDefaultBufferSize) {
            const int old_idx = index_buffer_.front();
            index_buffer_.pop_front();
            if (--buf_count_[static_cast<std::size_t>(old_idx)] <= 0) {
                remove_event(old_idx);
            }
        }
        const int idx = static_cast<int>(e.y) * width_ + e.x;
        index_buffer_.push_back(idx);
        ++buf_count_[static_cast<std::size_t>(idx)];
        LevelLinePixel& llp = pixelmap_[static_cast<std::size_t>(idx)];
        llp.buffered = true;
        llp.x = e.x;
        llp.y = e.y;
        llp.polarity = e.p;
        llp.ts = e.t;
        // Recompute the level line if this pixel has a gradient.
        const bool boundary = is_boundary(e.x, e.y);
        if (boundary) {
            if (llp.has_ll && llp.support >= 0) remove_pixel_from_support(idx);
            llp.has_ll = false;
            llp.support = -1;
            return;
        }
        if (llp.support >= 0) remove_pixel_from_support(idx);
        assign_gradient(llp, e);
        assign_support_region(idx);
    }

    void remove_event(int idx) {
        LevelLinePixel& llp = pixelmap_[static_cast<std::size_t>(idx)];
        llp.buffered = false;
        if (!llp.buffered && llp.support >= 0) {
            remove_pixel_from_support(idx);
        }
        llp.clear();
    }

    bool is_boundary(std::uint16_t x, std::uint16_t y) const {
        return static_cast<int>(x) < kSobelRadius ||
               static_cast<int>(y) < kSobelRadius ||
               static_cast<int>(x) >= width_ - kSobelRadius ||
               static_cast<int>(y) >= height_ - kSobelRadius;
    }

    /// jAER assignGradient (useTimestampGradient=true, sobelWidth=3):
    /// per-polarity 3x3 Sobel on timestamps, with predictTimestamps mirror
    /// fallback and kernel-weight normalization.
    void assign_gradient(LevelLinePixel& llp, const Event& e) {
        static constexpr int kFiltX[9] = {-1, 0, 1, -2, 0, 2, -1, 0, 1};
        static constexpr int kFiltY[9] = {-1, -2, -1, 0, 0, 0, 1, 2, 1};
        const std::vector<Metavision::timestamp>& ts_map = e.p ? on_ts_ : off_ts_;
        const Metavision::timestamp max_age =
            static_cast<Metavision::timestamp>(max_age_us_);
        const int px = e.x;
        const int py = e.y;
        float sum_abs_x = 0.0F, sum_abs_y = 0.0F;
        float sx = 0.0F, sy = 0.0F;
        for (int h = 0; h < 3; ++h) {
            for (int w = 0; w < 3; ++w) {
                int nx = px - kSobelRadius + w;
                int ny = py - kSobelRadius + h;
                Metavision::timestamp nt = ts_map[static_cast<std::size_t>(ny) * width_ + nx];
                Metavision::timestamp delta = nt - e.t;
                if (nt < 0 || delta > max_age || delta < -max_age) {
                    // predictTimestamps (jAER default ON): mirror across the
                    // pixel (x+radius-w, y+radius-h).
                    const int mx = px + kSobelRadius - w;
                    const int my = py + kSobelRadius - h;
                    if (mx >= 0 && my >= 0 && mx < width_ && my < height_) {
                        nt = ts_map[static_cast<std::size_t>(my) * width_ + mx];
                        delta = nt - e.t;
                        if (nt < 0 || delta > max_age || delta < -max_age) {
                            delta = 0;
                        } else {
                            sum_abs_x += static_cast<float>(kFiltX[w + h * 3] < 0
                                ? -kFiltX[w + h * 3] : kFiltX[w + h * 3]);
                            sum_abs_y += static_cast<float>(kFiltY[w + h * 3] < 0
                                ? -kFiltY[w + h * 3] : kFiltY[w + h * 3]);
                        }
                    } else {
                        delta = 0;
                    }
                } else {
                    sum_abs_x += static_cast<float>(kFiltX[w + h * 3] < 0
                        ? -kFiltX[w + h * 3] : kFiltX[w + h * 3]);
                    sum_abs_y += static_cast<float>(kFiltY[w + h * 3] < 0
                        ? -kFiltY[w + h * 3] : kFiltY[w + h * 3]);
                }
                sx += static_cast<float>(delta) * static_cast<float>(kFiltX[w + h * 3]);
                sy += static_cast<float>(delta) * static_cast<float>(kFiltY[w + h * 3]);
            }
        }
        if (sum_abs_x < 1e-6f || sum_abs_y < 1e-6f) {
            llp.has_ll = false;
            llp.support = -1;
            return;
        }
        const float gx = sx / sum_abs_x;
        const float gy = sy / sum_abs_y;
        // jAER: vx = -gy, vy = gx; theta = atan2(vx, vy).
        const float vx = -gy;
        const float vy = gx;
        float theta = std::atan2(vx, vy) * 180.0F / kPi;
        if (theta == -180.0F) theta = 180.0F;  // atan2 boundary -> +180
        const float mag = std::fabs(gx) + std::fabs(gy);
        llp.angle = theta;
        llp.angle90 = std::fmod(theta, 180.0F);
        if (llp.angle90 < -90.0F) llp.angle90 += 180.0F;
        else if (llp.angle90 > 90.0F) llp.angle90 -= 180.0F;
        llp.magnitude = mag;
        llp.has_ll = true;
    }

    /// jAER assignSupportRegion: find same-orientation candidates in the
    /// sobel neighbourhood, pick the oldest support as best fit, merge or
    /// extend, and (re)assign.
    void assign_support_region(int idx) {
        LevelLinePixel& pixel = pixelmap_[static_cast<std::size_t>(idx)];
        if (!pixel.has_ll) return;
        const int px = pixel.x;
        const int py = pixel.y;
        std::vector<int> candidates;
        int best_support = -1;
        for (int v = py - kSobelRadius; v <= py + kSobelRadius; ++v) {
            for (int u = px - kSobelRadius; u <= px + kSobelRadius; ++u) {
                if (u < 0 || v < 0 || u >= width_ || v >= height_) continue;
                if (u == px && v == py) continue;
                LevelLinePixel& nb = pixelmap_[static_cast<std::size_t>(v) * width_ + u];
                if (!nb.has_ll || nb.polarity != pixel.polarity) continue;  // distinguish ON/OFF
                const float ang = nb.support >= 0
                    ? supports_[static_cast<std::size_t>(nb.support)]->ll_angle_deg()
                    : nb.angle;
                // jAER: level-line angles are 180-periodic; the diff is taken
                // modulo 180 so -180 vs -90 (the same direction) match.
                float diff = std::fabs(pixel.angle - ang);
                diff = std::fmod(diff, 180.0F);
                if (diff > 90.0F) diff = 180.0F - diff;
                if (diff < kToleranceAngleDeg) {
                    candidates.push_back(static_cast<int>(v) * width_ + u);
                    if (nb.support >= 0) {
                        if (best_support < 0 ||
                            supports_[static_cast<std::size_t>(nb.support)]->creation_time() <
                            supports_[static_cast<std::size_t>(best_support)]->creation_time()) {
                            best_support = nb.support;
                        }
                    }
                }
            }
        }
        if (candidates.empty()) return;
        if (best_support < 0) {
            // Paper ALGORITHM 1: the CURRENT pixel is also a candidate and
            // findOldestSupport finds the pixel's OWN support if it has one —
            // a pixel already assigned to a support NEVER triggers a new
            // support (the former code created a new support and moved the
            // pixel into it whenever its neighbourhood had none, fragmenting
            // and accumulating supports on every moving edge).
            if (pixel.support >= 0) {
                best_support = pixel.support;
            } else if (static_cast<int>(candidates.size()) >= 1) {
                best_support = new_support();
            } else {
                return;
            }
        } else if (pixel.support >= 0 && pixel.support != best_support) {
            LineSupport& best = *supports_[static_cast<std::size_t>(best_support)];
            LineSupport& other = *supports_[static_cast<std::size_t>(pixel.support)];
            const float new_den = density_combined(best, other);
            const bool aligned_ok = !kMergeOnlyIfLinesAligned ||
                distance_segment_to_segment(best, other) <= kMaxDistancePx;
            if (aligned_ok && density_ok(best, other, new_den)) {
                if (best.creation_time() < other.creation_time()) {
                    other.set_self_support_idx(pixel.support);
                    best.merge(other, best_support, pixelmap_);
                    destroy_support(pixel.support);
                } else {
                    best.set_self_support_idx(best_support);
                    other.merge(best, pixel.support, pixelmap_);
                    destroy_support(best_support);
                    best_support = pixel.support;
                }
            }
        }
        // Attach this pixel to best_support (jAER addPixelToSupport).
        if (best_support >= 0 && pixel.support != best_support) {
            if (pixel.support >= 0) remove_pixel_from_support(idx);
            add_pixel_to_support(idx, best_support);
        }
        // Iterate remaining candidates: merge / extend them too.
        for (const int cidx : candidates) {
            LevelLinePixel& c = pixelmap_[static_cast<std::size_t>(cidx)];
            if (!c.has_ll) continue;
            if (c.support == best_support) continue;
            if (c.support >= 0) {
                LineSupport& best = *supports_[static_cast<std::size_t>(best_support)];
                LineSupport& other = *supports_[static_cast<std::size_t>(c.support)];
                const float new_den = density_combined(best, other);
                const bool aligned_ok = !kMergeOnlyIfLinesAligned ||
                    distance_segment_to_segment(best, other) <= kMaxDistancePx;
                if (aligned_ok && density_ok(best, other, new_den)) {
                    if (best.creation_time() < other.creation_time()) {
                        other.set_self_support_idx(c.support);
                        best.merge(other, best_support, pixelmap_);
                        destroy_support(c.support);
                    } else {
                        best.set_self_support_idx(best_support);
                        other.merge(best, c.support, pixelmap_);
                        destroy_support(best_support);
                        best_support = c.support;
                    }
                }
            } else {
                // Unassigned candidate: extend the best fit if the density
                // gate passes (jAER getDensityAdded).
                LineSupport& best = *supports_[static_cast<std::size_t>(best_support)];
                const float new_den = density_added(best, c);
                if (!kAddOnlyIfDensityHigh || best.mass() < kMinSupportForDensityTest ||
                    new_den >= kMinDensity ||
                    (kOrAddIfDensityIncreases && new_den > best_density(best))) {
                    add_pixel_to_support(cidx, best_support);
                }
            }
        }
    }

    void add_pixel_to_support(int idx, int support_idx) {
        LevelLinePixel& llp = pixelmap_[static_cast<std::size_t>(idx)];
        if (llp.support >= 0) remove_pixel_from_support(idx);
        supports_[static_cast<std::size_t>(support_idx)]->add(llp, idx);
        llp.support = support_idx;
    }

    void remove_pixel_from_support(int idx) {
        LevelLinePixel& llp = pixelmap_[static_cast<std::size_t>(idx)];
        const int sup = llp.support;
        if (sup < 0) return;
        supports_[static_cast<std::size_t>(sup)]->remove(llp);
        llp.support = -1;
        if (supports_[static_cast<std::size_t>(sup)]->mass() <= 0.0F) {
            destroy_support(sup);
        }
    }

    int new_support() {
        auto* ls = new LineSupport(next_support_id_++);
        supports_.push_back(ls);
        return static_cast<int>(supports_.size()) - 1;
    }

    void destroy_support(int support_idx) {
        if (support_idx < 0 || support_idx >= static_cast<int>(supports_.size())) return;
        LineSupport* ls = supports_[static_cast<std::size_t>(support_idx)];
        if (ls == nullptr) return;
        // Detach the support's OWN pixels (jAER HashSet iteration) — the
        // former whole-map scan was O(W*H) per destroy and, with supports
        // churning every packet, dominated the runtime on dense scenes.
        for (const int id : ls->pixel_ids()) {
            if (pixelmap_[static_cast<std::size_t>(id)].support == support_idx) {
                pixelmap_[static_cast<std::size_t>(id)].support = -1;
            }
        }
        delete ls;
        supports_[static_cast<std::size_t>(support_idx)] = nullptr;
    }

    /// jAER LineSupport density helpers (simplified; mass-normalized).
    static float density_combined(const LineSupport& a, const LineSupport& b) {
        const float m = a.mass() + b.mass();
        return m > 0.0F ? m / (a.length() + b.length() + 1.0F) : 0.0F;
    }
    static float density_added(const LineSupport& a, const LevelLinePixel& /*p*/) {
        const float m = a.mass() + 1.0F;
        return m / (a.length() + 1.0F);
    }
    static float best_density(const LineSupport& a) {
        return a.mass() / (a.length() + 1.0F);
    }
    static bool density_ok(const LineSupport& a, const LineSupport& b,
                           float new_den) {
        if (!kAddOnlyIfDensityHigh) return true;
        if (std::max(a.mass(), b.mass()) < kMinSupportForDensityTest) return true;
        return new_den >= kMinDensity ||
               (kOrAddIfDensityIncreases &&
                new_den > std::max(best_density(a), best_density(b)));
    }

    /// jAER distanceSegmentToSegment: distance from the SMALLER segment's
    /// center to the line through the BIGGER segment.
    static float distance_segment_to_segment(const LineSupport& a,
                                             const LineSupport& b) {
        const LineSupport* smaller = (a.mass() < b.mass()) ? &b : &a;
        const LineSupport* bigger  = (a.mass() < b.mass()) ? &a : &b;
        const float ux = std::cos(bigger->orientation_deg() * kPi / 180.0F);
        const float uy = std::sin(bigger->orientation_deg() * kPi / 180.0F);
        const float dx = smaller->center_x() - bigger->center_x();
        const float dy = smaller->center_y() - bigger->center_y();
        // Perpendicular distance to the line through the bigger segment.
        return std::fabs(dx * uy - dy * ux);
    }

    /// jAER checkSupport: split supports whose width >= widthToSplit.
    void check_support() {
        if (!kSplittingActive) return;
        std::vector<int> splits;
        for (int i = 0; i < static_cast<int>(supports_.size()); ++i) {
            if (supports_[static_cast<std::size_t>(i)] != nullptr &&
                supports_[static_cast<std::size_t>(i)]->width() >= kWidthToSplit) {
                splits.push_back(i);
            }
        }
        for (const int i : splits) split_support(i);
    }

    /// jAER split: remove every support pixel, then re-assign each.
    void split_support(int support_idx) {
        LineSupport* ls = supports_[static_cast<std::size_t>(support_idx)];
        if (ls == nullptr) return;
        std::vector<int> pixels;
        for (const int id : ls->pixel_ids()) {
            if (pixelmap_[static_cast<std::size_t>(id)].support == support_idx) {
                pixels.push_back(id);
            }
        }
        for (const int idx : pixels) remove_pixel_from_support(idx);
        for (const int idx : pixels) assign_support_region(idx);
    }

    int width_;
    int height_;
    int min_line_length_px_;
    int max_line_gap_px_;
    int max_age_us_{kDefaultMaxAgeUs};
    std::vector<Metavision::timestamp> on_ts_;
    std::vector<Metavision::timestamp> off_ts_;
    std::deque<int> index_buffer_;                 // jAER ring buffer of indices
    // Per-pixel count of events currently in the ring buffer (the paper's
    // AS "number of buffered events with the pixel coordinate"). A pixel is
    // only purged from its support region when this count reaches zero —
    // while it still has buffered events its level-line angle and support
    // membership persist (paper III-B).
    std::vector<int> buf_count_;
    std::vector<LevelLinePixel> pixelmap_;         // per-pixel LevelLinePixel
    std::vector<LineSupport*> supports_;           // live LineSupport regions
    int next_support_id_{0};
};

// Static member definition (min line support = the registered min_length).
float LineSegmentDetector::LineSupport::min_support_ = 20.0F;

} // namespace gui_algo

#endif // GUI_ALGO_CV_LINE_SEGMENT_DETECTOR_H
