// gui/davis/auto_exposure.h — DAVIS auto-exposure decision, ported from the
// reference computeAutomaticExposure (dv-processing, Apache-2.0).
//
// Pure function of the frame histogram + the current exposure, so the
// regulation law is unit-testable on synthetic frames (dark/bright/mid).
// The caller programs the returned exposure into the APS_EXPOSURE register.

#ifndef GUI_DAVIS_AUTO_EXPOSURE_H
#define GUI_DAVIS_AUTO_EXPOSURE_H

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <opencv2/core.hpp>

namespace gui::davis {

/// Exposure bounds (µs) — the register holds 22 bits of ADC-clock ticks.
inline constexpr double kExposureMinUs = 1.0;
inline constexpr double kExposureMaxUs = 4194303.0;

/// @brief Reference auto-exposure law.
/// @param image      current grayscale frame (CV_8UC1; callers must
///                   convert color frames to gray first — the reference
///                   meters on grayscale only)
/// @param current_us exposure used for this frame (µs)
/// @return the exposure to program for the next frame (µs)
inline double auto_exposure_step(const cv::Mat& image, double current_us) {
    if (image.empty()) return current_us;

    double hist[256] = {0};
    const int channels = image.channels();
    double pixels = 0;
    for (int row = 0; row < image.rows; ++row) {
        const auto* line = image.ptr<std::uint8_t>(row);
        for (int col = 0; col < image.cols; ++col) {
            ++hist[line[col * channels]];
            pixels += 1;
        }
    }
    if (pixels <= 0) return current_us;

    // Under/over-exposure fractions: below 10% / above 90% brightness.
    double frac_low = 0, frac_high = 0;
    for (int i = 0; i < 26; ++i) frac_low += hist[i];      // < 0.10 * 256
    for (int i = 230; i < 256; ++i) frac_high += hist[i];  // > 0.90 * 256
    frac_low /= pixels;
    frac_high /= pixels;

    const double err_low = frac_low - 0.33;
    const double err_high = frac_high - 0.33;
    const bool low = frac_low >= 0.33;
    const bool high = frac_high >= 0.33;

    double next = current_us;
    if (low && !high) {
        // Underexposed but not overexposed: open up.
        next = current_us + std::llround(14000.0 * std::pow(err_low, 1.65));
        if (next == current_us) ++next;
    } else if (high && !low) {
        // Overexposed but not underexposed: close down.
        next = current_us - std::llround(14000.0 * std::pow(err_high, 1.65));
        if (next == current_us) --next;
    } else {
        // Both (or neither) at the limits: steer the mean sample value
        // toward the middle bin.
        double num = 0, den = 0;
        for (int i = 0; i < 256; ++i) {
            const int bin = std::min(i / 52, 4);  // 5 bins over 0..255
            num += (bin + 1.0) * hist[i];
            den += hist[i];
        }
        const double msv = den >= 1.0 ? num / den : 2.5;
        const double msv_err = 2.5 - msv;
        double divisor = 1.0;
        if (std::fabs(err_low) < 0.1 || std::fabs(err_high) < 0.1) divisor = 5;
        if (std::fabs(err_low) < 0.05 || std::fabs(err_high) < 0.05) divisor = 10;
        if (msv_err > 0.1) {
            next = current_us + std::llround(100.0 * msv_err * msv_err / divisor);
            if (next == current_us) ++next;
        } else if (msv_err < -0.1) {
            next = current_us - std::llround(100.0 * msv_err * msv_err / divisor);
            if (next == current_us) --next;
        }
    }
    return std::clamp(next, kExposureMinUs, kExposureMaxUs);
}

/// ADC-clock ticks for an exposure in µs (register encoding).
inline std::uint32_t exposure_ticks(double exposure_us, double adc_clock_hz) {
    const double ticks = std::clamp(exposure_us * adc_clock_hz, 1.0,
                                    kExposureMaxUs * adc_clock_hz);
    return static_cast<std::uint32_t>(ticks);
}

} // namespace gui::davis

#endif // GUI_DAVIS_AUTO_EXPOSURE_H
