// gui/recorder/aedat4_writer.h — AEDAT4 (DV-native) event recording for the
// inivation live sources (Phase 4). Produces the byte-level layout of the
// DV v2.0 recorder (verified against real files and our AEDAT4 reader):
//   "#!AER-DAT4.0\r\n" + [u32 ioHeaderSize][IOHeader flatbuffer]
//   packets: {i32 streamID = 0; i32 size} + body [u32 fbSize]["EVTS" fb]
//   trailing FileDataTable ("FTAB") + IOHeader.dataTablePosition back-patch
// Packets are uncompressed (compression NONE); events accumulate and flush
// as one EventPacket per threshold to keep packet overhead low.

#ifndef GUI_RECORDER_AEDAT4_WRITER_H
#define GUI_RECORDER_AEDAT4_WRITER_H

#include <cstdint>
#include <cstdio>
#include <mutex>
#include <string>
#include <vector>

#include <metavision/sdk/base/events/event_cd.h>

#include "davis/aps_decoder.h"
#include "davis/imu_types.h"

namespace gui {

class Aedat4Writer {
public:
    /// FileDataTable record per written packet.
    struct Entry {
        std::int32_t sid;   ///< stream ID (0 events, 1 IMU, 2 APS frames)
        std::int32_t size;
        std::int64_t num;
        std::int64_t ts0;
        std::int64_t ts1;
        std::int64_t offset;  ///< absolute file offset of the packet BODY
                              ///< (dv FileDataDefinition.ByteOffset — the
                              ///< field dv seeks by; without it dv reads
                              ///< every packet at file position 0).
    };

    Aedat4Writer() = default;
    ~Aedat4Writer();

    Aedat4Writer(const Aedat4Writer&) = delete;
    Aedat4Writer& operator=(const Aedat4Writer&) = delete;

    /// @brief Opens @p path and writes the header. @p source names the
    ///        camera (recorded in the stream-info XML). @p imu_stream /
    ///        @p aps_stream declare the side streams this recording will
    ///        produce — a stream that is not declared cannot be misread as
    ///        present by players (e.g. DVXplorer has no APS at all).
    bool open(const std::string& path, int width, int height,
              const std::string& source,
              bool imu_stream = true, bool aps_stream = true);
    /// @brief Appends a batch (called from the USB thread). Accumulates and
    ///        flushes a packet once the threshold is reached.
    void write(const Metavision::EventCD* begin, const Metavision::EventCD* end);
    /// @brief Appends one IMU6 sample (stream 1, DV "IMU " type — flatbuffer
    ///        layout per dv's data/imu.fbs: IMUPacket { elements: [IMU] }).
    ///        Accumulates and flushes a packet every kImuFlushSamples samples.
    void write_imu(const davis::ImuSample& s);
    /// @brief Appends one APS grayscale frame (stream 2, DV "FRME" type —
    ///        dv's data/frame.fbs Frame, OPENCV_8U_C1). One packet per frame.
    void write_aps(const davis::ApsFrame& f);
    /// @brief Flushes the pending packet and writes the data table. Safe to
    ///        call twice.
    void close();

    [[nodiscard]] bool is_open() const {
        std::lock_guard<std::mutex> lock(mtx_);
        return file_ != nullptr;
    }
    [[nodiscard]] std::uint64_t events_written() const {
        std::lock_guard<std::mutex> lock(mtx_);
        return total_events_;
    }

private:
    void flush_locked();
    void flush_imu_locked();

    std::FILE* file_{nullptr};
    mutable std::mutex mtx_;
    std::vector<Metavision::EventCD> pending_;
    std::vector<davis::ImuSample> imu_pending_;
    static constexpr std::size_t kImuFlushSamples = 64;
    std::vector<Entry> entries_;
    std::streamoff table_pos_field_{0};  ///< IOHeader dataTablePosition slot.
    std::streamoff byte_offset_{0};      ///< Running file position of the next
                                         ///< packet header (FTAB ByteOffset
                                         ///< source; mtx_-guarded like all
                                         ///< mutable state).
    std::uint64_t total_events_{0};
};

} // namespace gui

#endif // GUI_RECORDER_AEDAT4_WRITER_H
