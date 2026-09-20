// gui/davis/imu_decoder.h — IMU6 sequence decoder shared by the DAVIS and
// DVXplorer wire parsers. Ported from dv-processing 2.0.4 parsers
// (Apache-2.0): the device streams each sample as
//   special data 5        → IMU start (sequence reset)
//   code 5, misc8 code 3  → IMU Scale Config (in-band scales + data-type mask)
//   code 5, misc8 code 0  → data byte (14-byte sequence: 7 × high/low pairs)
//   special data 7        → IMU end (emit only when the sequence is complete)
// The two chips order the X/Y tags differently (the DVXplorer's IMU tags
// carry Y first, DAVIS carries X first), and the temperature formula depends
// on the IMU chip family. Reference flip controls are not exposed by this
// port, so no flip conversion is applied (the reference skips it too when
// flip == flip-control).
//
// Coordinate system (docs.inivation.com hardware-advanced-usage/imu.html):
// the camera frame is X right, Y up, Z toward the lens viewed from the BACK,
// the units are g and dps — and the wire convention is PER FAMILY (the
// cameras carry different IMU chips: DAVIS346 = InvenSense MPU-6500,
// DVXplorer = Bosch BMI160), each pinned by the hardware axis-rotation
// test on the real camera:
//  - DVXplorer: negate gyro_y, everything else passes through (a
//    reflection — three rounds of sign builds jointly admitted exactly
//    this solution).
//  - DAVIS family: all three rotation senses read inverted under the DVX
//    convention — the exact complement, i.e. the IMU frame sits rotated
//    180 deg about the camera's Y. That is a PROPER rotation (a rigid
//    remount, not a mirror): accel_x/accel_z/gyro_x/gyro_z negate and the
//    accel stays consistent with the gyro.
// The inivation gyroscope-convention sentence ("counter-clockwise along
// the increasing axis, which does not follow the right-hand rule") is a
// diagram-reading note, not a numeric transform.

#ifndef GUI_DAVIS_IMU_DECODER_H
#define GUI_DAVIS_IMU_DECODER_H

#include <cstdint>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstdio>

#include "imu_types.h"

namespace gui::davis {

class ImuDecoder {
public:
    /// @param swap_xy true for DVXplorer (its data tags carry Y before X).
    ///        @param bmi160_temp true for the DVXplorer formula
    ///        (raw/512 + 23); false for the DAVIS family (InvenSense
    ///        6500/9250: raw/333.87 + 21, otherwise raw/340 + 35).
    ///        @param accel_shift 3 for DVXplorer (accel range code at
    ///        Scale Config bits [4:3]) and 2 for DAVIS (bits [3:2]).
    /// @param invenSense_gyro selects the gyro range-code semantics of the
    /// scale-config word: inivation/InvenSense (DAVIS) codes ascend with
    /// range (0=±250 … 3=±2000 dps, dv-processing davis_parser.hpp);
    /// BMI160 (DVXplorer) codes descend (0=±2000 … 4=±125 dps).
    /// @param invenSense_gyro selects the gyro range-code encoding: the
    /// DAVIS/InvenSense layout codes ASCEND with range (0=±250 … 3=±2000
    /// dps, dv-processing davis_parser.hpp) and the register
    /// IMU_GYRO_FULL_SCALE uses the same encoding; the DVXplorer BMI160
    /// codes DESCEND (0=±2000 … 4=±125 dps).
    /// Per-family wire convention, pinned on hardware (see the header).
    enum class AxisConvention {
        Dvxplorer,  ///< negate gyro_y only.
        Davis,      ///< IMU rotated 180 deg about the camera's Y: negate
                    ///< accel_x/accel_z/gyro_x/gyro_z (a proper remount).
    };
    ImuDecoder(bool swap_xy, bool bmi160_temp, int accel_shift, int gyro_mask,
               bool invenSense_gyro, AxisConvention convention)
        : swap_xy_(swap_xy), bmi160_temp_(bmi160_temp), accel_shift_(accel_shift),
          gyro_mask_(gyro_mask), invenSense_gyro_(invenSense_gyro),
          convention_(convention) {}

    void set_model(ImuModel model) { model_ = model; }
    void set_sink(const ImuSink& sink) { sink_ = sink; }

    /// IMU start (special data 5).
    void start() {
        count_ = 0;
        type_ = 0;
        sample_ = ImuSample{};
    }

    /// IMU Scale Config (code 5, misc8 code 3). @p data is the word's full
    /// 12-bit data field. Bits: [10:8] type mask (1 temp / 2 gyro / 4 accel),
    /// [5:3] accel range (0=±2 g … 3=±16 g), [2:0] gyro range (0=±2000 …
    /// 4=±125 °/s, descending).
    void scale_config(std::uint16_t data) {
        // EBPLUS_IMU_TRACE=1 prints every Scale Config word and the decoded
        // codes — field diagnostics for scale/word-layout mismatches.
        if (std::getenv("EBPLUS_IMU_TRACE") != nullptr) {
            std::fprintf(stderr,
                "[imu] scale word 0x%03X: accel code %d, gyro code %d, type %d\n",
                data, (data >> accel_shift_) & 0x03,
                data & gyro_mask_, (data >> 5) & 0x07);
        }
        accel_scale_ = 65536.0F / static_cast<float>(4 * (1 << ((data >> accel_shift_) & 0x03)));
        const int gyro_code = static_cast<int>(data & gyro_mask_);
        if (invenSense_gyro_) {
            // InvenSense (DAVIS): codes ASCEND with range (register
            // IMU_GYRO_FULL_SCALE uses the same encoding) —
            // 0 - ±250 dps - 131 LSB/°/s … 3 - ±2000 dps - 16.4 LSB/°/s
            // (dv-processing davis_parser.hpp calculateIMUGyroScale).
            gyro_scale_ = 65536.0F / static_cast<float>(500 * (1 << gyro_code));
        } else {
            // BMI160 (DVXplorer): codes DESCEND —
            // 0 - ±2000 dps … 4 - ±125 dps - 262 LSB/°/s.
            const auto clamped = std::min<int>(gyro_code, 4);
            gyro_scale_ = 65536.0F / static_cast<float>(250 * (1 << (4 - clamped)));
        }
        type_ = static_cast<std::uint8_t>(data >> 5) & 0x07;
        if (type_ & 0x04) {
            count_ = 0;  // Accelerometer first.
        } else if (type_ & 0x01) {
            count_ = 6;  // Temperature only.
        } else if (type_ & 0x02) {
            count_ = 8;  // Gyroscope only.
        } else {
            count_ = 14;  // Nothing enabled — samples will be discarded.
        }
    }

    /// One data byte (code 5, misc8 code 0): the high half at even sequence
    /// positions, completing the big-endian int16 at odd positions.
    void data_byte(std::uint8_t byte) {
        // fprintf(stderr, "[dbg] data_byte c=%d b=%02X\n", (int)count_, (unsigned)byte);
        switch (count_) {
            case 0: case 2: case 4: case 6: case 8: case 10: case 12:
                tmp_ = byte;
                break;
            case 1: accel_tag1() = scaled16(accel_scale_, byte); break;
            case 3: accel_tag3() = scaled16(accel_scale_, byte); break;
            case 5: {
                sample_.accel_z = scaled16(accel_scale_, byte);
                // Sequence continues with temperature only when enabled.
                if ((type_ & 0x01) == 0) count_ += (type_ & 0x02) ? 2 : 8;
                break;
            }
            case 7: {
                const auto raw = static_cast<std::int16_t>((tmp_ << 8) | byte);
                if (bmi160_temp_) {
                    sample_.temperature = (static_cast<float>(raw) / 512.0F) + 23.0F;
                } else if (model_ == ImuModel::InvenSense6500_9250) {
                    sample_.temperature = (static_cast<float>(raw) / 333.87F) + 21.0F;
                } else {
                    sample_.temperature = (static_cast<float>(raw) / 340.0F) + 35.0F;
                }
                // Sequence continues with gyro only when enabled.
                if ((type_ & 0x02) == 0) count_ += 6;
                break;
            }
            case 9: gyro_tag1() = scaled16(gyro_scale_, byte); break;
            case 11: gyro_tag2() = scaled16(gyro_scale_, byte); break;
            case 13: sample_.gyro_z = scaled16(gyro_scale_, byte); break;
            default: break;  // Invalid sequence position — wait for end.
        }
        count_++;
    }

    /// IMU end (special data 7): emit when the sequence is complete.
    /// @param t stream time of the end marker (rebased µs).
    void end(std::int64_t t) {
        if (count_ == 14) {
            sample_.t = t;
            sample_.valid = true;
            // Per-family display convention (see the header comment).
            if (convention_ == AxisConvention::Dvxplorer) {
                sample_.gyro_y = -sample_.gyro_y;
            } else {  // Davis: rigid 180 deg remount about the camera's Y.
                sample_.accel_x = -sample_.accel_x;
                sample_.accel_z = -sample_.accel_z;
                sample_.gyro_x = -sample_.gyro_x;
                sample_.gyro_z = -sample_.gyro_z;
            }
            if (sink_) sink_(sample_);
        }
    }

    void reset() {
        count_ = 0;
        type_ = 0;
        tmp_ = 0;
        sample_ = ImuSample{};
    }

private:
    float& accel_tag1() { return swap_xy_ ? sample_.accel_y : sample_.accel_x; }
    float& accel_tag3() { return swap_xy_ ? sample_.accel_x : sample_.accel_y; }
    float& gyro_tag1() { return swap_xy_ ? sample_.gyro_y : sample_.gyro_x; }
    float& gyro_tag2() { return swap_xy_ ? sample_.gyro_x : sample_.gyro_y; }

    float scaled16(float scale, std::uint8_t low) const {
        return static_cast<float>(static_cast<std::int16_t>((tmp_ << 8) | low)) / scale;
    }

    bool swap_xy_{false};
    bool bmi160_temp_{false};
    int accel_shift_{3};
    int gyro_mask_{0x07};
    bool invenSense_gyro_{true};
    AxisConvention convention_{AxisConvention::Davis};
    ImuModel model_{ImuModel::BoschBMI160};
    ImuSink sink_;

    std::uint8_t count_{0};
    std::uint8_t type_{0};
    std::uint8_t tmp_{0};
    float accel_scale_{8192.0F};  // ±4 g — replaced by in-band Scale Config.
    float gyro_scale_{65.536F};   // ±500 °/s — replaced by in-band Scale Config.
    ImuSample sample_;
};

} // namespace gui::davis

#endif // GUI_DAVIS_IMU_DECODER_H
