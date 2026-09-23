// algo/cv/dl_optical_flow.h — deep-learning dense optical flow.
//
// EV-FlowNet architecture (Zhu et al. 2018) retrained with the sim-to-real
// techniques of Stoffregen et al., "Reducing the Sim-to-Real Gap for Event
// Cameras" (ECCV 2020). The model is a stateless feed-forward UNet: one
// 5-bin voxel grid in, one 2-channel per-pixel (u, v) DISPLACEMENT out
// (pixels within the accumulation window; divide by the window duration to
// get px/s — the reference inference.py does the same).
//
// Reuses E2VIDInference as the generic ONNX model runner (OpenVINO GPU →
// ONNX Runtime CPU runtime selection, §4.4.2-GPU) and EventVoxelGrid via it.
// Visualization follows the reference flow2bgr_np (Alex Zhu / EV-FlowNet
// convention): hue = direction, value = speed.
//
// Header-only, no Qt dependency.

#ifndef GUI_ALGO_CV_DL_OPTICAL_FLOW_H
#define GUI_ALGO_CV_DL_OPTICAL_FLOW_H

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "algo/common/event.h"
#include "algo/analytics/e2vid/e2vid_inference.h"

namespace gui_algo {

class DLOpticalFlow {
public:
    /// @param width,height Sensor (or ROI) dimensions the flow is produced at.
    /// @param output_fps Flow output rate in Hz, [1, 120].
    DLOpticalFlow(int width, int height, int output_fps = 30)
        : inference_(width, height, 5),
          output_fps_(clamp_fps(output_fps)) {}

    /// @brief Accumulates events; consumed at the next due get_frame().
    /// With no model loaded the events are dropped right here: get_frame()
    /// early-returns without reaching the buffer clear, so buffering them
    /// anyway would grow the buffer without bound (the default model path
    /// is repo-relative and a launch from another working directory loads
    /// nothing).
    void process(const Event* events, std::size_t n) {
        if (events == nullptr || n == 0) return;
        if (!inference_.is_model_loaded()) return;
        event_buffer_.insert(event_buffer_.end(), events, events + n);
    }

    /// @brief Produces one HSV-coded dense flow frame (CV_8UC3, BGR for the
    /// GUI) when the output cadence is due and a model is loaded; returns an
    /// empty Mat otherwise.
    cv::Mat get_frame() {
        if (!inference_.is_model_loaded() || event_buffer_.empty()) {
            return {};
        }
        const std::uint64_t now_t = event_buffer_.back().t;
        if (last_frame_t_ >= 0 &&
            now_t < last_frame_t_ + frame_interval_us()) {
            return {};  // not due yet (fps gate)
        }
        const std::uint64_t t0 = event_buffer_.front().t;
        cv::Mat flow = inference_.infer(
            event_buffer_.data(), event_buffer_.size());
        event_buffer_.clear();
        last_frame_t_ = static_cast<std::int64_t>(now_t);
        // The heuristic fallback (8UC1) means the neural model is gone —
        // emit nothing rather than nonsense.
        if (flow.empty() || flow.type() != CV_32FC2) {
            return {};
        }
        flow = inference_.crop_to_sensor(flow);

        // Displacement within the window → speed in px/s (reference:
        // inference.py divides by the voxel window duration).
        double dt_s = static_cast<double>(now_t - t0) * 1e-6;
        if (dt_s < 1e-6) dt_s = 1e-6;

        // Alex Zhu / EV-FlowNet HSV convention (reference flow2bgr_np):
        // hue = direction (angle rotated by π), value = speed.
        std::vector<cv::Mat> xy;
        cv::split(flow, xy);
        cv::Mat mag, ang;
        cv::cartToPolar(xy[0], xy[1], mag, ang);
        mag.convertTo(mag, CV_32F, 1.0 / dt_s);  // px/s
        // angle ∈ [0, 2π] → +π → wrap to [0, 2π) → hue ∈ [0, 180).
        cv::Mat hue = ang + static_cast<float>(CV_PI);
        cv::subtract(hue, cv::Scalar(2.0 * CV_PI), hue,
                     hue >= 2.0 * CV_PI);
        hue *= 90.0f / static_cast<float>(CV_PI);
        cv::Mat sat(hue.size(), CV_32F, cv::Scalar(255.0f));
        // Value normalization: max_velocity_px_s_ > 0 pins the scale
        // manually; 0 (default) auto-scales per frame to the max magnitude,
        // like the reference flow2bgr_np(max_magnitude=None).
        float scale = max_velocity_px_s_;
        if (scale <= 0.0f) {
            double mmax = 0.0;
            cv::minMaxLoc(mag, nullptr, &mmax);
            scale = static_cast<float>(std::max(mmax, 1e-3));
        }
        cv::Mat val = cv::min(mag / scale, 1.0f) * 255.0f;

        cv::Mat hsv, bgr;
        std::vector<cv::Mat> hsv_ch = {hue, sat, val};
        cv::merge(hsv_ch, hsv);                 // CV_32FC3 (H, S, V)
        hsv.convertTo(hsv, CV_8UC3, 1.0, 0.0);  // interleaved 8UC3 HSV
        cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
        return bgr;
    }

    // Parameters -----------------------------------------------------------
    void set_model_path(const std::string& path) {
        inference_.load_model(path);
    }
    const std::string& model_path() const { return inference_.model_path(); }
    bool is_model_loaded() const { return inference_.is_model_loaded(); }
    const std::string& active_runtime() const {
        return inference_.active_runtime();
    }

    /// Inference device policy: 0=Auto, 1=CPU, 2=GPU (§4.4.2-GPU).
    void set_device(int d) {
        inference_.set_device(static_cast<E2VIDInference::Device>(d));
    }
    int device() const { return static_cast<int>(inference_.device()); }

    void set_output_fps(int fps) { output_fps_ = clamp_fps(fps); }
    int output_fps() const { return output_fps_; }

    /// HSV value scale (px/s). > 0 pins the speed→brightness mapping
    /// manually; 0 (default) auto-scales per frame to the frame's max speed
    /// (reference flow2bgr_np behavior).
    void set_max_velocity_px_s(float v) { max_velocity_px_s_ = v; }
    float max_velocity_px_s() const { return max_velocity_px_s_; }

    /// Minimum interval between output frames in us.
    std::int64_t frame_interval_us() const {
        return static_cast<std::int64_t>(1.0e6 / output_fps_);
    }

    void reset() {
        inference_.reset();
        event_buffer_.clear();
        last_frame_t_ = -1;  // -1 = no frame yet: skip the gate on the first
    }

    int width() const { return inference_.width(); }
    int height() const { return inference_.height(); }

private:
    static int clamp_fps(int fps) {
        if (fps < 1) return 1;
        if (fps > 120) return 120;
        return fps;
    }

    E2VIDInference inference_;  ///< generic ONNX runner (2-ch flow model)
    std::vector<Event> event_buffer_;
    int output_fps_;
    float max_velocity_px_s_{0.0f};
    std::int64_t last_frame_t_{-1};
};

}  // namespace gui_algo

#endif  // GUI_ALGO_CV_DL_OPTICAL_FLOW_H
