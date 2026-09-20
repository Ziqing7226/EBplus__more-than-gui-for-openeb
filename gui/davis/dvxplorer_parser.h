// gui/davis/dvxplorer_parser.h — wire-format decoder for inivation DVXplorer
// cameras. Ported (C++17, dependency-free) from dv-processing 2.0.4
// io/camera/parsers/dvxplorer_parser.hpp (Apache-2.0).
//
// The DVXplorer stream shares the 16-bit LE word family with the DAVIS
// (bit 15 = timestamp, code 0 special incl. TS reset, code 7 = TS wrap) but
// the pixel data uses mgroup compression:
//   code 1 → X column address latch (10 bits; 1023 = reset lastX to 0)
//   code 4 → Y-group address latch: two groups, base = (data & 0x3F) × 8,
//            second group = base ± ((data >> 6) & 0x1F) × 8 (bit 11 = minus)
//   code 2 → 8-pixel group from Y-group 2
//   code 3 → 8-pixel group from Y-group 1
//   pixel-group word: data[7:0] = 8-pixel presence mask (LSB = lowest row),
//   bit 8 = polarity (0 = ON/positive, 1 = OFF/negative)
// The stream is rebased to start at 0 after each device timestamp reset.

#ifndef GUI_DAVIS_DVXPLORER_PARSER_H
#define GUI_DAVIS_DVXPLORER_PARSER_H

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include <metavision/sdk/base/events/event_cd.h>

#include "imu_decoder.h"
#include "imu_types.h"

namespace gui::davis {

class DvxParser {
public:
    using EventSink = std::function<void(const Metavision::EventCD*, const Metavision::EventCD*)>;

    DvxParser() = default;

    /// @param width/@param height expected DVS resolution; events outside are
    ///        dropped. (Unlike DAVIS, DVXplorer has no axis-swap orientation
    ///        bit — the reference applies only flip controls, which this
    ///        events-only port does not expose.)
    DvxParser(int width, int height);

    void parse(const std::uint8_t* data, std::size_t size, const EventSink& sink);

    /// Deferred variant used by the device layer: decodes WITHOUT invoking
    /// the event sink (the IMU sink still fires inline — cheap and
    /// latency-critical), leaving the decoded events for swap_batch().
    /// Keeps the USB reaping thread fast under an event flood: the heavy
    /// per-batch pipeline runs on the BatchWorker thread instead.
    void decode(const std::uint8_t* data, std::size_t size);

    /// Swaps the decoded batch out into @p slot (and the slot's recycled
    /// buffer in) so the caller can queue it elsewhere.
    void swap_batch(std::vector<Metavision::EventCD>& slot) { batch_.swap(slot); }

    [[nodiscard]] bool time_initialized() const { return t0_set_; }
    void reset();

    /// Completed IMU6 samples (accel/gyro/temp) — invoked from the USB
    /// thread whenever the IMU stream is enabled on the device.
    void set_imu_sink(const ImuSink& sink) { imu_.set_sink(sink); }

private:
    void update_timestamp(std::int64_t ts);

    int width_{0};
    int height_{0};

    bool t0_set_{false};
    std::int64_t wrap_add_{0};
    std::int64_t t0_{0};
    std::int64_t current_{0};
    std::int16_t last_x_{0};
    std::int16_t last_yg1_{0};
    std::int16_t last_yg2_{0};

    // DVXplorer: X/Y tags carry Y first; BMI160 temperature formula;
    // accel range code at Scale Config bits [4:3].
    ImuDecoder imu_{true, true, 3, 0x07, false,
                    ImuDecoder::AxisConvention::Dvxplorer};

    std::vector<Metavision::EventCD> batch_;
};

} // namespace gui::davis

#endif // GUI_DAVIS_DVXPLORER_PARSER_H
