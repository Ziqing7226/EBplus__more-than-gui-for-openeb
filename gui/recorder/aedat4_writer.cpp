// gui/recorder/aedat4_writer.cpp — see aedat4_writer.h.
//
// Flatbuffers are built forward with explicit offsets (equivalent to what
// our reader walks in aedat4_file_source.cpp):
//   buffer = [u32 rootOffset][ident "XXXX"][payload...]
//   table  = [i32 vtableOffset][fields...]; the offset is SIGNED — a vtable
//   placed after its table yields a negative value (dv's FTAB does this).
//   vector = [u32 count][elements]; string = [u32 length][bytes][0].
// EventPacket: one field, VT4 = vector of 16-byte structs
//   {i64 t; i16 x; i16 y; u8 polarity; 3 pad}. IOHeader: VT4 = i32
//   compression (0 = NONE), VT6 = i64 dataTablePosition, VT8 = string
//   sourceInfo. DataTable ("FTAB"): VT4 = vector of dv FileDataDefinition
//   entry tables, each {VT4 = i64 byteOffset (absolute, packet body),
//   VT6 = struct{i32 streamID; i32 size}, VT8 = i64 numEvents,
//   VT10 = i64 tsStart, VT12 = i64 tsEnd} — dv's reader seeks by ByteOffset.

#include "aedat4_writer.h"

#include <algorithm>
#include <cstring>

namespace gui {
namespace {

constexpr char kMagic[14] = {'#', '!', 'A', 'E', 'R', '-', 'D', 'A', 'T', '4',
                             '.', '0', '\r', '\n'};
constexpr std::size_t kFlushEvents = 2048;

void put_u16(std::vector<std::uint8_t>& b, std::size_t pos, std::uint16_t v) {
    b[pos] = static_cast<std::uint8_t>(v);
    b[pos + 1] = static_cast<std::uint8_t>(v >> 8);
}
void put_u32(std::vector<std::uint8_t>& b, std::size_t pos, std::uint32_t v) {
    b[pos] = static_cast<std::uint8_t>(v);
    b[pos + 1] = static_cast<std::uint8_t>(v >> 8);
    b[pos + 2] = static_cast<std::uint8_t>(v >> 16);
    b[pos + 3] = static_cast<std::uint8_t>(v >> 24);
}
void put_i32_at(std::vector<std::uint8_t>& b, std::size_t pos, std::int32_t v) {
    put_u32(b, pos, static_cast<std::uint32_t>(v));
}
/// EventPacket — flatbuffers uoffsets point FORWARD (target = pos + rel):
/// [0] u32 root → table@8; [4] "EVTS"; table@8 {soffset −8 → vtable@16;
/// VT4@12 → vector@28}; vtable@16; [28] count; [32] structs (16 B each,
/// 8-aligned — flatbuffers vectors store elements directly at count+4).
void build_event_packet(const std::vector<Metavision::EventCD>& evs,
                        std::vector<std::uint8_t>& out) {
    const std::size_t count = evs.size();
    out.assign(32 + 16 * count, 0);

    put_u32(out, 0, 8);                    // root → table
    std::memcpy(out.data() + 4, "EVTS", 4);
    put_i32_at(out, 8, -8);                // vtable = table − (−8) = 16
    put_u32(out, 12, 16);                  // VT4: field@12 + 16 = vector@28
    put_u16(out, 16, 6);                   // vtable size
    put_u16(out, 18, 8);                   // table size
    put_u16(out, 20, 4);                   // VT4 offset in table
    put_u32(out, 28, static_cast<std::uint32_t>(count));  // vector count
    for (std::size_t i = 0; i < count; ++i) {
        std::size_t pos = 28 + 4 + 16 * i;
        const std::int64_t t = evs[i].t;
        std::memcpy(out.data() + pos, &t, 8);
        const std::int16_t x = static_cast<std::int16_t>(evs[i].x);
        const std::int16_t y = static_cast<std::int16_t>(evs[i].y);
        std::memcpy(out.data() + pos + 8, &x, 2);
        std::memcpy(out.data() + pos + 10, &y, 2);
        out[pos + 12] = evs[i].p ? 1 : 0;
    }
}

/// IMUPacket — dv data/imu.fbs: file_identifier "IMUS"; root table
/// IMUPacket { elements: [IMU] (vector of TABLES, native_inline) }. Every
/// sample table shares ONE vtable (identical field sets), placed BEFORE the
/// tables (soffset negative). Table (52 B): [soffset][ts @4][temp @12]
/// [ax @16][ay @20][az @24][gx @28][gy @32][gz @36][mx @40][my @44][mz @48]
/// (magnetometer stays 0 — absent on our chips). Non-overlapping layout:
/// root@0, ident@4, IMUPacket table@8 (8..60), vtable@60 (60..86),
/// vector@88, sample tables after the element slots.
void build_imu_packet(const std::vector<davis::ImuSample>& samples,
                      std::vector<std::uint8_t>& out) {
    const std::size_t n = samples.size();
    const std::size_t vtable_pos = 60;
    const std::size_t vector_pos = 88;
    const std::size_t elems_pos = vector_pos + 4;
    const std::size_t first_table = elems_pos + 4 * n;
    out.assign(first_table + n * 52, 0);

    put_u32(out, 0, 8);                    // root → table@8
    std::memcpy(out.data() + 4, "IMUS", 4);
    // soffset = table − vtable (NEGATIVE — the vtable sits after the table;
    // the reader computes vtable = table − soffset).
    put_i32_at(out, 8, static_cast<std::int32_t>(8 - vtable_pos));
    put_u32(out, 12, static_cast<std::uint32_t>(vector_pos - 12));   // VT4 → vector
    // vtable entries are SAMPLE-TABLE-relative (52 B): [VT4=4 ts]
    // [VT5=12 temp][VT6=16 ax][VT7=20 ay][VT8=24 az][VT9=28 gx][VT10=32 gy]
    // [VT11=36 gz][VT12=40 mx][VT13=44 my][VT14=48 mz]
    put_u16(out, vtable_pos, 26);
    put_u16(out, vtable_pos + 2, 52);
    for (int f = 0; f < 11; ++f) {
        put_u16(out, vtable_pos + 4 + 2 * f,
                static_cast<std::uint16_t>(f == 0 ? 4 : 12 + 4 * (f - 1)));
    }
    put_u32(out, vector_pos, static_cast<std::uint32_t>(n));
    for (std::size_t i = 0; i < n; ++i) {
        const std::size_t slot = elems_pos + 4 * i;
        const std::size_t table = first_table + i * 52;
        put_u32(out, slot, static_cast<std::uint32_t>(table - slot));
        put_i32_at(out, table, static_cast<std::int32_t>(static_cast<std::int64_t>(table) - vtable_pos));
        std::memcpy(out.data() + table + 4, &samples[i].t, 8);
        std::memcpy(out.data() + table + 12, &samples[i].temperature, 4);
        std::memcpy(out.data() + table + 16, &samples[i].accel_x, 4);
        std::memcpy(out.data() + table + 20, &samples[i].accel_y, 4);
        std::memcpy(out.data() + table + 24, &samples[i].accel_z, 4);
        std::memcpy(out.data() + table + 28, &samples[i].gyro_x, 4);
        std::memcpy(out.data() + table + 32, &samples[i].gyro_y, 4);
        std::memcpy(out.data() + table + 36, &samples[i].gyro_z, 4);
    }
}

/// Frame — dv data/frame.fbs: file_identifier "FRME"; root table Frame.
/// Written fields: VT4 timestamp, VT5 tsSOF, VT6 tsEOF, VT9 format (i8,
/// OPENCV_8U_C1 = 0), VT10 sizeX, VT11 sizeY, VT14 pixels vector, VT16
/// source (i8, SENSOR = 1); SOE/EOE/position/exposure keep defaults
/// (omitted → vtable entry 0). Table (44 B): [soffset][ts @4][sof @12]
/// [eof @20][pixdisp @28][format @32][sizeX @34][sizeY @36][posX @38]
/// [posY @40][source @42]; vtable@52 (30 B); pixels vector@84.
void build_frame_packet(const davis::ApsFrame& f, std::vector<std::uint8_t>& out) {
    // dv data/frame.fbs: file_identifier "FRME"; root table Frame. Fields in
    // declaration order (VT = 4 + 2*index): ts VT4, tsSOF VT6, tsEOF VT8,
    // soe VT10, eoe VT12, format VT14, sizeX VT16, sizeY VT18, posX VT20,
    // posY VT22, pixels VT24, exposure VT26, source VT28.
    //
    // Builder convention (matching our Fb reader): a vtable entry at byte
    // (4 + 2*index) holds the TABLE-RELATIVE offset of the field slot; a
    // scalar slot holds the value, an indirect slot holds the u32
    // displacement from the slot to the target.
    //
    // Layout: root@0, ident@4, table@8 (48 B), vtable@56 (30 B),
    // pixels vector@88. Table slots: ts rel8, sof rel16, eof rel24,
    // pixels-disp rel32, format rel36, sizeX rel38, sizeY rel40,
    // posX rel42, posY rel44, source rel46. soffset = 8 - 56 = -48.
    const std::size_t vtable_pos = 56;
    const std::size_t vector_pos = 88;
    const std::uint8_t* pix = f.image.data;
    const int channels = f.image.channels();
    const std::size_t npix = static_cast<std::size_t>(f.image.rows) *
                             static_cast<std::size_t>(f.image.cols) *
                             static_cast<std::size_t>(channels);
    out.assign(vector_pos + 4 + npix, 0);

    put_u32(out, 0, 8);                    // root → table@8
    std::memcpy(out.data() + 4, "FRME", 4);
    put_i32_at(out, 8, static_cast<std::int32_t>(8 - vtable_pos));
    std::memcpy(out.data() + 16, &f.t, 8);           // VT4 ts
    std::memcpy(out.data() + 24, &f.t, 8);           // VT6 tsSOF
    std::memcpy(out.data() + 32, &f.t, 8);           // VT8 tsEOF
    put_u32(out, 40, static_cast<std::uint32_t>(vector_pos - 40));   // VT24 → pixels
    out[44] = channels == 3 ? 16 : 0;  // VT14: dv FrameFormat OPENCV_8U_C1(0) /
                                       // OPENCV_8U_C3(16) — 2 is OPENCV_16U_C1
    const std::int16_t w = static_cast<std::int16_t>(f.image.cols);
    const std::int16_t h = static_cast<std::int16_t>(f.image.rows);
    std::memcpy(out.data() + 46, &w, 2);             // VT16 sizeX
    std::memcpy(out.data() + 48, &h, 2);             // VT18 sizeY
    out[54] = 1;                                     // VT28 source SENSOR
    // vtable@56: [vtsz=30][tsz=48][VT4→8][VT6→16][VT8→24][VT10→0][VT12→0]
    // [VT14→36][VT16→38][VT18→40][VT20→42][VT22→44][VT24→32][VT26→0]
    // [VT28→46]
    put_u16(out, vtable_pos, 30);
    put_u16(out, vtable_pos + 2, 48);
    put_u16(out, vtable_pos + 4, 8);
    put_u16(out, vtable_pos + 6, 16);
    put_u16(out, vtable_pos + 8, 24);
    put_u16(out, vtable_pos + 10, 0);
    put_u16(out, vtable_pos + 12, 0);
    put_u16(out, vtable_pos + 14, 36);
    put_u16(out, vtable_pos + 16, 38);
    put_u16(out, vtable_pos + 18, 40);
    put_u16(out, vtable_pos + 20, 42);
    put_u16(out, vtable_pos + 22, 44);
    put_u16(out, vtable_pos + 24, 32);
    put_u16(out, vtable_pos + 26, 0);
    put_u16(out, vtable_pos + 28, 46);
    put_u32(out, vector_pos, static_cast<std::uint32_t>(npix));
    if (npix) std::memcpy(out.data() + vector_pos + 4, pix, npix);
}

/// IOHeader — [0] u32 root → table@12 (table+12 8-aligned); [4] "IOHE";
/// table@12 {soffset −20 → vtable@32; VT4 i32 compression@16; VT8 u32@20 →
/// string after the table; VT6 i64 dataTablePosition@24}; vtable@32; string
/// (u32 len + bytes + 0) after the vtable. Returns the buffer offset of the
/// dataTablePosition i64 (the caller patches it once the region is placed).
std::size_t build_io_header(std::vector<std::uint8_t>& out, std::int64_t table_position,
                            const std::string& xml) {
    const std::size_t slen = xml.size();
    const std::size_t str_len_pos = 44;
    const std::size_t total = str_len_pos + 4 + slen + 1;
    out.assign(total, 0);

    put_u32(out, 0, 12);                   // root → table@12
    std::memcpy(out.data() + 4, "IOHE", 4);
    put_i32_at(out, 12, -20);              // vtable = 12 + 20 = 32
    put_i32_at(out, 16, 0);                // VT4: compression NONE
    put_u32(out, 20, static_cast<std::uint32_t>(str_len_pos - 20));  // VT8 → string
    std::memcpy(out.data() + 24, &table_position, 8);                // VT6
    put_u16(out, 32, 10);                  // vtable size
    put_u16(out, 34, 20);                  // table size
    put_u16(out, 36, 4);                   // VT4 offset
    put_u16(out, 38, 12);                  // VT6 offset
    put_u16(out, 40, 8);                   // VT8 offset
    put_u32(out, str_len_pos, static_cast<std::uint32_t>(slen));
    std::memcpy(out.data() + str_len_pos + 4, xml.data(), slen);
    out[total - 1] = 0;
    return 24;  // dataTablePosition i64 sits at table(12) + 12
}

/// DataTable — [0] u32 root → table@8; [4] "FTAB"; table@8 {soffset −8 →
/// vtable@16; VT4 u32@12 → vector@24}; vtable@16; [24] count; [28] slots
/// (u32 forward displacements); entry tables per dv FileDataTable.fbs
/// (E ≡ 0 mod 8, vtable after each entry, soffset negative):
///   FileDataDefinition { ByteOffset: int64; PacketInfo: {i32 streamID;
///   i32 size} (native_inline); NumElements: int64; TimestampStart: int64;
///   TimestampEnd: int64 }
/// layout: [soffset][pad4][PacketInfo @8][ByteOffset @16][num @24]
/// [ts0 @32][ts1 @40 (ends @48)], table size 48, vtable@48 (14 B).
/// ByteOffset is the absolute file offset of the packet body — dv's reader
/// seeks by it (writer.hpp passes mByteOffset + sizeof(PacketHeader));
/// leaving it absent (0) makes dv read every packet from the file head.
void build_data_table(std::vector<std::uint8_t>& out,
                      const std::vector<Aedat4Writer::Entry>& entries) {
    const std::size_t slots = 28 + 4 * entries.size();
    out.reserve(slots + entries.size() * 64 + 16);
    out.assign(slots, 0);

    put_u32(out, 0, 8);    // root → table
    std::memcpy(out.data() + 4, "FTAB", 4);
    put_i32_at(out, 8, -8);  // vtable = 8 + 8 = 16
    put_u32(out, 12, 12);    // VT4: field@12 + 12 = vector@24
    put_u16(out, 16, 6);
    put_u16(out, 18, 8);
    put_u16(out, 20, 4);
    put_u32(out, 24, static_cast<std::uint32_t>(entries.size()));

    for (std::size_t i = 0; i < entries.size(); ++i) {
        while (out.size() % 8 != 0) out.push_back(0);
        const std::size_t slot = 28 + 4 * i;
        const std::size_t entry = out.size();
        const std::size_t vtable = entry + 48;
        out.resize(vtable + 14);
        put_u32(out, slot, static_cast<std::uint32_t>(entry - slot));
        put_i32_at(out, entry, static_cast<std::int32_t>(entry - vtable));
        // VT6: PacketInfo native_inline struct {i32 streamID; i32 size} @8.
        put_i32_at(out, entry + 8, entries[i].sid);
        put_i32_at(out, entry + 12, entries[i].size);
        // VT4: ByteOffset i64 @16.
        std::memcpy(out.data() + entry + 16, &entries[i].offset, 8);
        // VT8/VT10/VT12: num/ts0/ts1 i64 @24/32/40.
        std::memcpy(out.data() + entry + 24, &entries[i].num, 8);
        std::memcpy(out.data() + entry + 32, &entries[i].ts0, 8);
        std::memcpy(out.data() + entry + 40, &entries[i].ts1, 8);
        put_u16(out, vtable, 14);
        put_u16(out, vtable + 2, 48);
        put_u16(out, vtable + 4, 16);   // VT4  ByteOffset
        put_u16(out, vtable + 6, 8);    // VT6  PacketInfo
        put_u16(out, vtable + 8, 24);   // VT8  NumElements
        put_u16(out, vtable + 10, 32);  // VT10 TimestampStart
        put_u16(out, vtable + 12, 40);  // VT12 TimestampEnd
    }
}

} // namespace

Aedat4Writer::~Aedat4Writer() {
    close();
}

bool Aedat4Writer::open(const std::string& path, int width, int height,
                        const std::string& source,
                        bool imu_stream, bool aps_stream) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (file_) return false;
    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return false;

    // Stream-info XML — the DV recorder's outInfo shape (our reader's
    // parse_out_info consumes typeIdentifier/source/sizeX/sizeY).
    std::string xml = "<dv version=\"2.0\">\n";
    xml += "    <node name=\"outInfo\" path=\"/mainloop/Recorder/outInfo/\">\n";
    xml += "        <node name=\"0\" path=\"/mainloop/Recorder/outInfo/0/\">\n";
    xml += "            <attr key=\"compression\" type=\"string\">NONE</attr>\n";
    xml += "            <attr key=\"originalModuleName\" type=\"string\">capture</attr>\n";
    xml += "            <attr key=\"originalOutputName\" type=\"string\">events</attr>\n";
    xml += "            <attr key=\"typeDescription\" type=\"string\">Array of events (polarity ON/OFF).</attr>\n";
    xml += "            <attr key=\"typeIdentifier\" type=\"string\">EVTS</attr>\n";
    xml += "            <node name=\"info\" path=\"/mainloop/Recorder/outInfo/0/info/\">\n";
    xml += "                <attr key=\"eventsPixelArrangement\" type=\"int\">0</attr>\n";
    xml += "                <attr key=\"sizeX\" type=\"int\">" + std::to_string(width) + "</attr>\n";
    xml += "                <attr key=\"sizeY\" type=\"int\">" + std::to_string(height) + "</attr>\n";
    xml += "                <attr key=\"source\" type=\"string\">" + source + "</attr>\n";
    xml += "            </node>\n";
    xml += "        </node>\n";
    // Side streams are declared only when they will actually be recorded —
    // players treat a declared stream as present.
    if (imu_stream) {
    xml += "        <node name=\"1\" path=\"/mainloop/Recorder/outInfo/1/\">\n";
    xml += "            <attr key=\"compression\" type=\"string\">NONE</attr>\n";
    xml += "            <attr key=\"originalModuleName\" type=\"string\">capture</attr>\n";
    xml += "            <attr key=\"originalOutputName\" type=\"string\">imu</attr>\n";
    xml += "            <attr key=\"typeDescription\" type=\"string\">IMU samples.</attr>\n";
    xml += "            <attr key=\"typeIdentifier\" type=\"string\">IMU </attr>\n";
    xml += "            <node name=\"info\" path=\"/mainloop/Recorder/outInfo/1/info/\">\n";
    xml += "                <attr key=\"source\" type=\"string\">\"" + source + "\"</attr>\n";
    xml += "            </node>\n";
    xml += "        </node>\n";
    }
    if (aps_stream) {
    // Stream 2 = APS frames (grayscale 8-bit).
    xml += "        <node name=\"2\" path=\"/mainloop/Recorder/outInfo/2/\">\n";
    xml += "            <attr key=\"compression\" type=\"string\">NONE</attr>\n";
    xml += "            <attr key=\"originalModuleName\" type=\"string\">capture</attr>\n";
    xml += "            <attr key=\"originalOutputName\" type=\"string\">frames</attr>\n";
    xml += "            <attr key=\"typeDescription\" type=\"string\">APS frames.</attr>\n";
    xml += "            <attr key=\"typeIdentifier\" type=\"string\">FRME</attr>\n";
    xml += "            <node name=\"info\" path=\"/mainloop/Recorder/outInfo/2/info/\">\n";
    xml += "                <attr key=\"sizeX\" type=\"int\">" + std::to_string(width) + "</attr>\n";
    xml += "                <attr key=\"sizeY\" type=\"int\">" + std::to_string(height) + "</attr>\n";
    xml += "                <attr key=\"source\" type=\"string\">\"" + source + "\"</attr>\n";
    xml += "            </node>\n";
    xml += "        </node>\n";
    }
    xml += "    </node>\n";
    xml += "</dv>\n";

    std::vector<std::uint8_t> header;
    // dataTablePosition placeholder — patched at close() once the region
    // offset is known.
    const std::size_t field_in_io = build_io_header(header, 0, xml);
    table_pos_field_ = 14 + 4 + static_cast<std::streamoff>(field_in_io);

    bool ok = std::fwrite(kMagic, 1, sizeof(kMagic), f) == sizeof(kMagic);
    const std::uint32_t io_size = static_cast<std::uint32_t>(header.size());
    ok = ok && std::fwrite(&io_size, 1, 4, f) == 4;
    ok = ok && std::fwrite(header.data(), 1, header.size(), f) == header.size();
    if (!ok) {
        std::fclose(f);
        return false;
    }
    file_ = f;
    byte_offset_ = std::ftell(f);
    total_events_ = 0;
    pending_.clear();
    entries_.clear();
    return true;
}

void Aedat4Writer::write(const Metavision::EventCD* begin,
                         const Metavision::EventCD* end) {
    if (begin == nullptr || end == nullptr || begin >= end) return;
    std::lock_guard<std::mutex> lock(mtx_);
    if (!file_) return;
    // Chunk at the flush threshold so a huge incoming batch becomes several
    // bounded packets (DV commits packets frequently for seekability).
    while (begin < end) {
        const std::size_t room = kFlushEvents - pending_.size();
        const std::size_t take = std::min<std::size_t>(
            static_cast<std::size_t>(end - begin), room);
        pending_.insert(pending_.end(), begin, begin + take);
        begin += take;
        if (pending_.size() >= kFlushEvents) flush_locked();
    }
}

void Aedat4Writer::write_imu(const davis::ImuSample& s) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!file_) return;
    imu_pending_.push_back(s);
    if (imu_pending_.size() >= kImuFlushSamples) flush_imu_locked();
}

void Aedat4Writer::write_aps(const davis::ApsFrame& f) {
    if (f.image.empty() ||
        (f.image.type() != CV_8UC1 && f.image.type() != CV_8UC3)) {
        return;
    }
    std::lock_guard<std::mutex> lock(mtx_);
    if (!file_) return;

    std::vector<std::uint8_t> packet;
    build_frame_packet(f, packet);
    const auto fb_size = static_cast<std::int32_t>(packet.size());
    const auto body_size = static_cast<std::int32_t>(packet.size() + 4);
    const std::int32_t header[3] = {2, body_size, fb_size};
    const auto body_offset = static_cast<std::int64_t>(byte_offset_ + 8);
    if (std::fwrite(header, 4, 3, file_) != 3 ||
        std::fwrite(packet.data(), 1, packet.size(), file_) != packet.size()) {
        return;
    }
    byte_offset_ += 8 + body_size;
    entries_.push_back({2, body_size, 1, f.t, f.t, body_offset});
}

void Aedat4Writer::flush_imu_locked() {
    if (imu_pending_.empty() || !file_) return;
    std::vector<std::uint8_t> packet;
    build_imu_packet(imu_pending_, packet);
    const auto fb_size = static_cast<std::int32_t>(packet.size());
    const auto body_size = static_cast<std::int32_t>(packet.size() + 4);
    const std::int32_t header[3] = {1, body_size, fb_size};
    const std::int64_t ts0 = imu_pending_.front().t;
    const std::int64_t ts1 = imu_pending_.back().t;
    const auto n = static_cast<std::int64_t>(imu_pending_.size());
    imu_pending_.clear();
    const auto body_offset = static_cast<std::int64_t>(byte_offset_ + 8);
    if (std::fwrite(header, 4, 3, file_) != 3 ||
        std::fwrite(packet.data(), 1, packet.size(), file_) != packet.size()) {
        return;
    }
    byte_offset_ += 8 + body_size;
    entries_.push_back({1, body_size, n, std::min(ts0, ts1), std::max(ts0, ts1),
                        body_offset});
}

void Aedat4Writer::flush_locked() {
    if (pending_.empty() || !file_) return;

    std::vector<std::uint8_t> packet;
    build_event_packet(pending_, packet);
    // Body = [u32 fbSize][flatbuffer]; packet header = {i32 streamID}{i32
    // bodySize} — the real recorder files carry size = fbSize + 4.
    const auto fb_size = static_cast<std::int32_t>(packet.size());
    const auto body_size = static_cast<std::int32_t>(packet.size() + 4);
    const std::int32_t header[3] = {0, body_size, fb_size};
    std::int64_t ts0 = pending_.front().t;
    std::int64_t ts1 = pending_.back().t;
    if (ts1 < ts0) std::swap(ts0, ts1);
    const auto body_offset = static_cast<std::int64_t>(byte_offset_ + 8);
    if (std::fwrite(header, 4, 3, file_) != 3 ||
        std::fwrite(packet.data(), 1, packet.size(), file_) != packet.size()) {
        return;  // write failure: keep pending_ (a later flush retries)
    }
    byte_offset_ += 8 + body_size;
    entries_.push_back({0, body_size, static_cast<std::int64_t>(pending_.size()),
                        ts0, ts1, body_offset});
    total_events_ += pending_.size();
    pending_.clear();
}

void Aedat4Writer::close() {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!file_) return;
    flush_locked();
    flush_imu_locked();

    const std::streamoff region_start = std::ftell(file_);
    std::vector<std::uint8_t> table;
    build_data_table(table, entries_);
    const auto region = static_cast<std::uint32_t>(table.size());
    const bool ok = std::fwrite(&region, 1, 4, file_) == 4 &&
                    std::fwrite(table.data(), 1, table.size(), file_) == table.size();
    if (ok && std::fseek(file_, table_pos_field_, SEEK_SET) == 0) {
        std::fwrite(&region_start, 8, 1, file_);
    }
    std::fflush(file_);
    std::fclose(file_);
    file_ = nullptr;
    pending_.clear();
    imu_pending_.clear();
    entries_.clear();
}

} // namespace gui
