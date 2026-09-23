// gui/app/aedat4_file_source.h — playback source for inivation DV .aedat4
// recordings (e.g. DAVIS346). Reads ONLY the EVTS (polarity event) streams —
// frame/IMU/trigger packets are skipped, matching the GUI's pure event stream
// scope. Layout verified byte-level against DV v2.0 recorder output:
//   "#!AER-DAT4.0\r\n" + [u32 ioHeaderSize][IOHeader flatbuffer]
//   packets: {i32 streamID; i32 size} + body [u32 fbSize][flatbuffer]
//   EVTS event = {i64 t (µs); i16 x; i16 y; u8 polarity; 3 pad} × count
// Packets may be LZ4-framed (IOHeader.compression 1/2); zstd is rejected.

#ifndef GUI_APP_AEDAT4_FILE_SOURCE_H
#define GUI_APP_AEDAT4_FILE_SOURCE_H

#include "external_file_source.h"

#include <cstdint>
#include <fstream>
#include <map>
#include <string>
#include <vector>

namespace gui {

class Aedat4FileSource : public ExternalFileSource {
public:
    explicit Aedat4FileSource(const std::string& path) { path_ = path; }

    void open() override;
    void run(EventSink sink, DoneFn done) override;

    bool has_imu() const override { return has_imu_; }
    bool has_aps() const override { return has_aps_; }
    void set_imu_sink(ImuSink sink) override { imu_sink_ = std::move(sink); }
    void set_aps_sink(ApsSink sink) override { aps_sink_ = std::move(sink); }
    /// Invoked on the reader thread the first time an IMU / APS packet is
    /// actually decoded (true = IMU, false = APS) — side-stream presence by
    /// content, not by stream declaration.
    void set_side_stream_discovered(std::function<void(bool)> fn) {
        side_stream_discovered_ = std::move(fn);
    }

    /// Position-gated frame for replay: decodes the APS packet nearest at
    /// or before @p position_us using the packet index built during open().
    bool read_aps_frame_for_position(std::int64_t position_us,
                                     davis::ApsFrame& out) override;
    /// Exception barrier over the impl (runs on the GUI thread — an
    /// escaping exception would terminate the process).
    bool read_aps_frame_for_position_impl(std::int64_t position_us,
                                          davis::ApsFrame& out);

private:
    struct PacketInfo {
        std::int64_t byte_offset{0}; // packet body (after the 8-byte header)
        std::int32_t size{0};
        std::int64_t num_elements{0};
        std::int64_t ts_start{0};
        std::int64_t ts_end{0};
    };

    /// Parses the FileDataTable (optional) for duration / event-count metadata.
    void parse_data_table(std::ifstream& file, std::streamoff table_pos);

    std::streamoff first_packet_offset_{0};
    std::streamoff file_size_{0};
    /// End of the packet stream (start of the FileDataTable when known).
    std::streamoff stream_end_{-1};
    int compression_{0}; // IOHeader.compression (0 none, 1/2 lz4, 3/4 zstd)
    /// streamID → true when that stream carries EVTS packets.
    std::map<std::int32_t, bool> stream_is_events_;
    std::map<std::int32_t, bool> stream_is_imu_;
    std::map<std::int32_t, bool> stream_is_aps_;
    bool has_imu_{false};
    bool has_aps_{false};
    ImuSink imu_sink_;
    ApsSink aps_sink_;
    std::function<void(bool)> side_stream_discovered_;
    /// Shared normalization clock (first timestamp of the file); IMU
    /// samples decoded before the first event wait here for it.
    bool ev_t0_known_{false};
    std::int64_t ev_t0_{0};
public:
    bool ev_t0_known() const { return ev_t0_known_; }
    std::int64_t ev_t0() const { return ev_t0_; }
private:
    std::vector<davis::ImuSample> imu_pending_norm_;

    void decode_imu_body(const std::uint8_t* pd, std::size_t pn);
    void decode_frame_body(const std::uint8_t* pd, std::size_t pn,
                           davis::ApsFrame* out = nullptr);

    /// APS packet index for position-gated replay: (recorded timestamp,
    /// packet header offset) — built arithmetically during parse_data_table
    /// (the FTAB lists every packet's size, so offsets are cumulative).
    struct ApsIndexEntry {
        std::int64_t t{0};
        std::streamoff offset{0};
    };
    std::vector<ApsIndexEntry> aps_index_;
    std::ifstream aps_read_;          // dedicated handle (GUI-thread reads)
    std::int64_t aps_served_ts_{-1};  // raw ts of the last served packet
    davis::ApsFrame aps_served_;
};

} // namespace gui

#endif // GUI_APP_AEDAT4_FILE_SOURCE_H
