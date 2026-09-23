// gui/app/aedat4_file_source.cpp — see aedat4_file_source.h for the format.
//
// dv's flatbuffer buffers are standard flatbuffers tables with one deviation:
// the 4-char file identifier sits at bytes [4:8] right after the root uoffset
// (instead of at the end). Field access follows the standard rules — scalar
// fields are inline at table+offset, offset fields (string/vector/table) hold
// a u32 displacement relative to the field position.

#include "aedat4_file_source.h"

#include <QXmlStreamReader>

#include <algorithm>
#include <cstring>
#include <limits>
#include <stdexcept>

#include "lz4_frame_decoder.h"

#include <opencv2/core.hpp>

namespace gui {
namespace {

constexpr char kMagic[14] = {'#', '!', 'A', 'E', 'R', '-', 'D', 'A', 'T', '4',
                             '.', '0', '\r', '\n'};
constexpr std::size_t kMaxIoHeaderBytes = 64u * 1024 * 1024;
constexpr std::uint32_t kMaxPacketBytes = 256u * 1024 * 1024;

// --- Minimal flatbuffers reader -------------------------------------------

struct Fb {
    const std::uint8_t* d{nullptr};
    std::size_t n{0};

    bool valid(std::size_t off, std::size_t len) const {
        return off <= n && len <= n - off;
    }
    std::uint32_t u32(std::size_t off) const {
        std::uint32_t v;
        std::memcpy(&v, d + off, 4);
        return v;
    }
    std::uint16_t u16(std::size_t off) const {
        std::uint16_t v;
        std::memcpy(&v, d + off, 2);
        return v;
    }
    std::int16_t i16(std::size_t off) const {
        std::int16_t v;
        std::memcpy(&v, d + off, 2);
        return v;
    }
    std::int32_t i32(std::size_t off) const {
        std::int32_t v;
        std::memcpy(&v, d + off, 4);
        return v;
    }
    std::int64_t i64(std::size_t off) const {
        std::int64_t v;
        std::memcpy(&v, d + off, 8);
        return v;
    }
    float f32(std::size_t off) const {
        float v;
        std::memcpy(&v, d + off, 4);
        return v;
    }
    // Root table position; buffer = [u32 rootUoffset][ident@4..8]...
    bool root(std::size_t& table) const {
        if (n < 8) return false;
        table = u32(0);
        return valid(table, 4);
    }
    // Absolute position of field @p vt (vtable offset), or 0 when absent.
    // The vtable shorts are [vtsz][tsz][VT4][VT6]…, i.e. field VT sits at
    // byte offset exactly VT from the vtable start. The soffset is SIGNED —
    // a vtable placed after its table yields a negative value (dv's FTAB
    // entries do exactly that).
    std::size_t field(std::size_t table, unsigned vt) const {
        const std::int32_t soffset = i32(table);
        if (soffset == std::numeric_limits<std::int32_t>::min()) return 0;
        const std::int64_t vtable64 =
            static_cast<std::int64_t>(table) - static_cast<std::int64_t>(soffset);
        if (vtable64 < 0 || vtable64 + 4 > static_cast<std::int64_t>(n)) return 0;
        const std::size_t vtable = static_cast<std::size_t>(vtable64);
        const std::size_t vtsz = u16(vtable);
        if (vtsz < vt + 2 || !valid(vtable, vt + 2)) return 0;
        const std::uint16_t off = u16(vtable + vt);
        if (off == 0) return 0;
        if (!valid(table + off, 1)) return 0;
        return table + off;
    }
    // Inline scalar at field position.
    template <typename T>
    T scalar(std::size_t table, unsigned vt, T def) const {
        const std::size_t pos = field(table, vt);
        if (pos == 0) return def;
        T v;
        std::memcpy(&v, d + pos, sizeof(T));
        return v;
    }
    // Indirect reference (string/vector/table): target = pos + u32@pos.
    std::size_t indirect(std::size_t table, unsigned vt) const {
        const std::size_t pos = field(table, vt);
        if (pos == 0) return 0;
        const std::uint32_t rel = u32(pos);
        const std::size_t target = pos + rel;
        return valid(target, 1) ? target : 0;
    }
    std::string string(std::size_t table, unsigned vt) const {
        const std::size_t pos = indirect(table, vt);
        if (pos == 0) return {};
        const std::uint32_t len = u32(pos);
        if (!valid(pos + 4, len)) return {};
        return {reinterpret_cast<const char*>(d + pos + 4), len};
    }
    // Vector start: [u32 count][...]; returns 0 when absent.
    std::size_t vector(std::size_t table, unsigned vt) const {
        return indirect(table, vt);
    }
};

bool read_exact(std::ifstream& f, void* dst, std::size_t n) {
    f.read(reinterpret_cast<char*>(dst), static_cast<std::streamsize>(n));
    return f.good() && static_cast<std::size_t>(f.gcount()) == n;
}

std::uint32_t read_u32(std::ifstream& f, bool& ok) {
    std::uint8_t b[4];
    ok = read_exact(f, b, 4);
    return static_cast<std::uint32_t>(b[0]) | (static_cast<std::uint32_t>(b[1]) << 8) |
           (static_cast<std::uint32_t>(b[2]) << 16) |
           (static_cast<std::uint32_t>(b[3]) << 24);
}

// --- outInfo XML → stream map ----------------------------------------------

struct StreamInfo {
    QString type;
    int width{0};
    int height{0};
    QString source;
};

/// Walks the shallow <dv><node name="outInfo"><node name="N">…</node>… tree.
/// Each element pushes a frame carrying the attributes it collected; on a
/// stream node's end tag the frame is stored, and info subnodes merge into
/// their parent stream.
std::map<std::int32_t, StreamInfo> parse_out_info(const QString& xml_text) {
    std::map<std::int32_t, StreamInfo> streams;
    struct Frame {
        QString element;
        QString node_name;
        QString attr_key;
        QString text;
        std::map<QString, QString> attrs;
        QString attr(const char* key) const {
            const auto it = attrs.find(QLatin1String(key));
            return it == attrs.end() ? QString() : it->second;
        }
    };
    std::vector<Frame> stack;
    QXmlStreamReader xml(xml_text);
    while (!xml.atEnd() && xml.error() == QXmlStreamReader::NoError) {
        const auto tok = xml.readNext();
        if (tok == QXmlStreamReader::StartElement) {
            Frame f;
            f.element = xml.name().toString();
            if (f.element == QStringLiteral("node") ||
                f.element == QStringLiteral("attr")) {
                f.node_name = xml.attributes().value(QStringLiteral("name")).toString();
                f.attr_key = xml.attributes().value(QStringLiteral("key")).toString();
            }
            stack.push_back(std::move(f));
        } else if (tok == QXmlStreamReader::Characters) {
            if (!stack.empty()) stack.back().text += xml.text().toString();
        } else if (tok == QXmlStreamReader::EndElement) {
            if (stack.empty()) break;
            Frame f = std::move(stack.back());
            stack.pop_back();
            if (f.element == QStringLiteral("attr")) {
                if (!stack.empty()) stack.back().attrs[f.attr_key] = f.text.trimmed();
            } else if (f.element == QStringLiteral("node")) {
                const QString type = f.attr("typeIdentifier");
                if (!type.isEmpty()) {
                    // A stream node: id comes from the node name. Merge — the
                    // info subnode may already have filled the resolution.
                    bool ok = false;
                    const std::int32_t id = f.node_name.toInt(&ok);
                    if (ok) {
                        StreamInfo& s = streams[id];
                        s.type = type;
                        if (s.source.isEmpty()) s.source = f.attr("source");
                        if (s.width == 0) s.width = f.attr("sizeX").toInt();
                        if (s.height == 0) s.height = f.attr("sizeY").toInt();
                    }
                } else if (!f.attr("source").isEmpty() ||
                           !f.attr("sizeX").isEmpty()) {
                    // info subnode: merge its values into the ENCLOSING
                    // stream frame's attrs — the stream node has not
                    // committed yet (its end tag comes after the info node's).
                    if (!stack.empty()) {
                        Frame& parent = stack.back();
                        if (!f.attr("source").isEmpty()) {
                            parent.attrs[QStringLiteral("source")] = f.attr("source");
                        }
                        if (!f.attr("sizeX").isEmpty()) {
                            parent.attrs[QStringLiteral("sizeX")] = f.attr("sizeX");
                        }
                        if (!f.attr("sizeY").isEmpty()) {
                            parent.attrs[QStringLiteral("sizeY")] = f.attr("sizeY");
                        }
                    }
                }
            }
        }
    }
    return streams;
}

} // namespace

// ---------------------------------------------------------------------------

void Aedat4FileSource::open() {
    std::ifstream file(path_, std::ios::binary);
    if (!file) throw std::runtime_error("Cannot open AEDAT4 file: " + path_);
    file.seekg(0, std::ios::end);
    file_size_ = file.tellg();
    file.seekg(0);

    char magic[sizeof(kMagic)];
    if (file_size_ < static_cast<std::streamoff>(sizeof(kMagic)) ||
        !read_exact(file, magic, sizeof(kMagic)) ||
        std::memcmp(magic, kMagic, sizeof(kMagic)) != 0) {
        throw std::runtime_error("Not an AEDAT4 file (bad header): " + path_);
    }
    bool ok = false;
    const std::uint32_t hdr_size = read_u32(file, ok);
    if (!ok || hdr_size == 0 || hdr_size > kMaxIoHeaderBytes ||
        static_cast<std::streamoff>(18 + hdr_size) >= file_size_) {
        throw std::runtime_error("Corrupt AEDAT4 header: " + path_);
    }
    std::vector<std::uint8_t> io(hdr_size);
    if (!read_exact(file, io.data(), hdr_size)) {
        throw std::runtime_error("Truncated AEDAT4 header: " + path_);
    }
    const Fb io_fb{io.data(), io.size()};
    std::size_t table = 0;
    if (!io_fb.root(table)) throw std::runtime_error("Corrupt AEDAT4 IOHeader: " + path_);
    compression_ = io_fb.scalar<std::int32_t>(table, 4, 0);
    const std::int64_t table_pos = io_fb.scalar<std::int64_t>(table, 6, -1);
    const QString xml_text = QString::fromStdString(io_fb.string(table, 8));
    if (compression_ >= 3) {
        throw std::runtime_error(
            "AEDAT4 file uses ZSTD packet compression, which is not supported "
            "(DV v2.x LZ4/NONE files are): " + path_);
    }

    const auto streams = parse_out_info(xml_text);
    for (const auto& [id, info] : streams) {
        stream_is_events_[id] = (info.type == QStringLiteral("EVTS"));
        stream_is_imu_[id] = (info.type.trimmed() == QStringLiteral("IMU"));
        stream_is_aps_[id] = (info.type == QStringLiteral("FRME"));
        // has_imu_/has_aps_ are finalized from the data table's actual
        // packet list (parse_data_table); declarations alone would mark a
        // DVXplorer recording as having APS just because the writer's XML
        // template mentions the stream.
        if (stream_is_events_[id]) {
            meta_.width = info.width > 0 ? info.width : meta_.width;
            meta_.height = info.height > 0 ? info.height : meta_.height;
            if (meta_.serial.isEmpty()) meta_.serial = info.source;
        }
    }
    bool has_events = false;
    for (const auto& [id, is_events] : stream_is_events_) {
        has_events = has_events || is_events;
    }
    if (!has_events) {
        throw std::runtime_error(
            "AEDAT4 file contains no event (EVTS) stream — only non-event "
            "data (frames/IMU/triggers): " + path_);
    }
    if (meta_.width <= 0 || meta_.height <= 0) {
        throw std::runtime_error("AEDAT4 event stream has no resolution: " + path_);
    }
    first_packet_offset_ = 18 + static_cast<std::streamoff>(hdr_size);
    meta_.integrator = QStringLiteral("inivation");
    meta_.plugin_name = QStringLiteral("AEDAT4");
    meta_.encoding_format = QStringLiteral("EVTS");

    if (table_pos > 0 && table_pos < file_size_) {
        stream_end_ = static_cast<std::streamoff>(table_pos);
        parse_data_table(file, static_cast<std::streamoff>(table_pos));
    }
}

void Aedat4FileSource::parse_data_table(std::ifstream& file, std::streamoff pos) {
    file.clear();
    file.seekg(pos);
    bool ok = false;
    const std::uint32_t region = read_u32(file, ok);
    if (!ok || region < 16 || pos + 4 + static_cast<std::streamoff>(region) > file_size_) {
        return; // metadata stays unknown; the stream path recomputes duration
    }
    std::vector<std::uint8_t> buf(region);
    if (!read_exact(file, buf.data(), region)) return;
    const Fb fb{buf.data(), buf.size()};
    std::size_t table = 0;
    if (!fb.root(table)) return;
    const std::size_t vec = fb.vector(table, 4);
    if (vec == 0 || !fb.valid(vec, 4)) return;
    const std::uint32_t count = fb.u32(vec);
    if (count == 0 || count > (buf.size() - vec - 4) / 4) return;

    std::int64_t total = 0;
    std::int64_t ts_min = std::numeric_limits<std::int64_t>::max();
    std::int64_t ts_max = std::numeric_limits<std::int64_t>::min();
    // The FTAB lists every packet in file order with its body size, so the
    // packet header offsets are purely cumulative from the first packet —
    // an APS (timestamp, offset) index falls out arithmetically, no I/O.
    std::streamoff cursor = first_packet_offset_;
    aps_index_.clear();
    for (std::uint32_t i = 0; i < count; ++i) {
        const std::size_t slot = vec + 4 + 4 * i;
        const std::uint32_t rel = fb.u32(slot);
        const std::size_t entry = slot + rel;
        if (!fb.valid(entry, 4)) return;
        // PacketInfo is an inline struct {i32 StreamID; i32 Size} at VT6.
        const std::size_t info = fb.field(entry, 6);
        if (info == 0 || !fb.valid(info, 8)) return;
        const std::int32_t sid = fb.i32(info);
        const std::int32_t psize = fb.i32(info + 4);
        const std::int64_t ts0 = fb.scalar<std::int64_t>(entry, 10, 0);
        const std::int64_t ts1 = fb.scalar<std::int64_t>(entry, 12, 0);
        // Side-stream presence follows the ACTUAL packet content, not the
        // XML declaration (writers may declare streams they never filled —
        // e.g. a DVXplorer recording declares FRME it cannot produce).
        if (stream_is_imu_[sid]) has_imu_ = true;
        if (stream_is_aps_[sid]) {
            has_aps_ = true;
            if (psize > 0) aps_index_.push_back({ts0, cursor});
        }
        auto it = stream_is_events_.find(sid);
        if (it != stream_is_events_.end() && it->second) {
            const std::int64_t n = fb.scalar<std::int64_t>(entry, 8, 0);
            total += n;
            ts_min = std::min(ts_min, ts0);
            ts_max = std::max(ts_max, ts1);
        }
        cursor += 8 + static_cast<std::streamoff>(psize);
    }
    if (total > 0 && ts_max >= ts_min) {
        meta_.worst_case_events = total;
        meta_.duration_us = ts_max - ts_min;
    }
}

void Aedat4FileSource::decode_imu_body(const std::uint8_t* pd, std::size_t pn) {
    if (pn < 8 || !imu_sink_) return;
    const std::uint32_t declared = [&] {
        std::uint32_t v; std::memcpy(&v, pd, 4); return v; }();
    if (declared < 8 || declared > pn) return;
    const Fb fb{pd + 4, declared};
    std::size_t table = 0;
    if (!fb.root(table)) return;
    const std::size_t vec = fb.vector(table, 4);
    if (vec == 0 || !fb.valid(vec, 4)) return;
    const std::uint32_t count = fb.u32(vec);
    if (count == 0 || (pn - vec - 4) / 4 < count) return;
    for (std::uint32_t i = 0; i < count; ++i) {
        const std::size_t slot = vec + 4 + 4 * i;
        const std::uint32_t rel = fb.u32(slot);
        const std::size_t elem = slot + rel;
        if (!fb.valid(elem, 52)) return;
        davis::ImuSample s;
        s.t = fb.i64(elem + 4);
        if (ev_t0_known_) s.t -= ev_t0_;
        s.temperature = fb.f32(elem + 12);
        s.accel_x = fb.f32(elem + 16);
        s.accel_y = fb.f32(elem + 20);
        s.accel_z = fb.f32(elem + 24);
        s.gyro_x = fb.f32(elem + 28);
        s.gyro_y = fb.f32(elem + 32);
        s.gyro_z = fb.f32(elem + 36);
        s.valid = true;
        if (!has_imu_) {
            has_imu_ = true;  // discovered by content (FTAB entries of
                              // older builds mislabel every packet as events)
            if (side_stream_discovered_) side_stream_discovered_(true);
        }
        if (!ev_t0_known_) {
            // File order can place IMU packets before the first event —
            // hold them until the shared normalization clock is known.
            if (imu_pending_norm_.size() < 8192) imu_pending_norm_.push_back(s);
            continue;
        }
        imu_sink_(s);
    }
}

void Aedat4FileSource::decode_frame_body(const std::uint8_t* pd, std::size_t pn,
                                         davis::ApsFrame* out) {
    if (pn < 8 || (!aps_sink_ && !out)) return;
    const std::uint32_t declared = [&] {
        std::uint32_t v; std::memcpy(&v, pd, 4); return v; }();
    if (declared < 8 || declared > pn) return;
    const Fb fb{pd + 4, declared};
    std::size_t table = 0;
    if (!fb.root(table)) return;
    // dv frame.fbs field order (VT = 4 + 2*fieldIndex): ts=VT4,
    // tsSOF=VT6, tsEOF=VT8, soe=VT10, eoe=VT12, format=VT14,
    // sizeX=VT16, sizeY=VT18, posX=VT20, posY=VT22, pixels=VT24,
    // exposure=VT26, source=VT28.
    const std::int64_t ts = fb.scalar<std::int64_t>(table, 4, 0);
    // The format slot is written as a single byte (frame.fbs enum; the
    // writer byte-packs the table), so read it as uint8 — a 4-byte read
    // would swallow the sizeX bytes and drop every frame.
    const auto format = fb.scalar<std::uint8_t>(table, 14, static_cast<std::uint8_t>(0));
    // dv FrameFormat: OPENCV_8U_C3 = 16; the value 2 is OPENCV_16U_C1 in the
    // schema but our writer mislabeled C3 frames as 2 before the fix — keep
    // accepting it so existing recordings stay replayable.
    const int channels =
        (format == 16 || format == 2) ? 3 : 1;  // OPENCV_8U_C3 (16 | legacy 2) / C1
    const std::int16_t w = fb.scalar<std::int16_t>(table, 16, 0);
    const std::int16_t h = fb.scalar<std::int16_t>(table, 18, 0);
    const std::size_t vec = fb.vector(table, 24);
    if (vec == 0 || w <= 0 || h <= 0 || !fb.valid(vec, 4)) return;
    const std::uint32_t count = fb.u32(vec);
    if (count != static_cast<std::uint32_t>(w) * static_cast<std::uint32_t>(h) *
                     static_cast<std::uint32_t>(channels) ||
        (pn - vec - 4) < count) {
        return;
    }
    if (!has_aps_) {
        has_aps_ = true;
        if (side_stream_discovered_) side_stream_discovered_(false);
    }
    davis::ApsFrame frame;
    frame.t = ts;
    frame.width = w;
    frame.height = h;
    frame.image = cv::Mat(h, w, channels == 3 ? CV_8UC3 : CV_8UC1);
    std::memcpy(frame.image.data, pd + 4 + vec + 4, count);
    frame.valid = true;
    if (out) *out = std::move(frame);
    else aps_sink_(frame);
}

bool Aedat4FileSource::read_aps_frame_for_position(std::int64_t position_us,
                                                   davis::ApsFrame& out) {
    // Runs on the GUI thread (the APS window's 30 Hz tick). A corrupt LZ4
    // frame header claims an arbitrary content size and the decoder's
    // reserve() can throw — an exception escaping into the Qt event loop
    // would terminate the process (the run() path already has this guard;
    // keep parity here and treat any failure as "no frame").
    try {
        return read_aps_frame_for_position_impl(position_us, out);
    } catch (const std::exception&) {
        return false;
    } catch (...) {
        return false;
    }
}

bool Aedat4FileSource::read_aps_frame_for_position_impl(std::int64_t position_us,
                                                        davis::ApsFrame& out) {
    if (aps_index_.empty() || !ev_t0_known_) return false;
    const std::int64_t target = position_us + ev_t0_;
    if (target < aps_index_.front().t) {
        aps_served_ts_ = -1;
        aps_served_ = {};
        return false;
    }
    // Newest packet at or before the target.
    std::size_t lo = 0, hi = aps_index_.size();
    while (lo + 1 < hi) {
        const std::size_t mid = (lo + hi) / 2;
        if (aps_index_[mid].t <= target) lo = mid;
        else hi = mid;
    }
    if (aps_index_[lo].t == aps_served_ts_ && !aps_served_.image.empty()) {
        out = aps_served_;  // position unchanged since the last query
        return true;
    }
    // Decode that one packet (its header is re-read for the exact size).
    if (!aps_read_.is_open()) {
        aps_read_.open(path_, std::ios::binary);
        if (!aps_read_) return false;
    }
    aps_read_.clear();
    aps_read_.seekg(aps_index_[lo].offset);
    std::uint8_t header[8];
    if (!read_exact(aps_read_, header, 8)) return false;
    const std::uint32_t size =
        static_cast<std::uint32_t>(header[4]) |
        (static_cast<std::uint32_t>(header[5]) << 8) |
        (static_cast<std::uint32_t>(header[6]) << 16) |
        (static_cast<std::uint32_t>(header[7]) << 24);
    if (size == 0 || size > kMaxPacketBytes) return false;
    std::vector<std::uint8_t> body(size);
    if (!read_exact(aps_read_, body.data(), size)) return false;
    const std::uint8_t* pd = body.data();
    std::size_t pn = size;
    std::vector<std::uint8_t> plain;
    if (compression_ != 0) {
        plain.clear();
        std::string lz4_err;
        if (!lz4_decompress_frame(pd, pn, plain, lz4_err)) return false;
        if (plain.size() < 8) return false;
        pd = plain.data();
        pn = plain.size();
    }
    davis::ApsFrame frame;
    decode_frame_body(pd, pn, &frame);
    if (!frame.valid) return false;
    aps_served_ts_ = aps_index_[lo].t;
    aps_served_ = frame;
    out = std::move(frame);
    return true;
}

void Aedat4FileSource::run(EventSink sink, DoneFn done) {
    std::string error;
    try {
        std::ifstream file(path_, std::ios::binary);
        if (!file) throw std::runtime_error("Cannot reopen AEDAT4 file: " + path_);

        std::vector<std::uint8_t> body;
        std::vector<std::uint8_t> plain;
        std::vector<Metavision::EventCD> batch;
        // One normalization clock for every stream: the first timestamp of
        // the file (event or IMU, whichever decodes first) becomes zero, so
        // replay-side consumers can align IMU samples with the event
        // playback position.
        ev_t0_known_ = false;
        ev_t0_ = 0;
        std::int64_t last_t = std::numeric_limits<std::int64_t>::min();

        const std::streamoff stream_end =
            stream_end_ > 0 ? std::min(stream_end_, file_size_) : file_size_;
        file.seekg(first_packet_offset_);
        for (;;) {
            if (stop_.load(std::memory_order_relaxed)) break;
            if (file.tellg() + 8 > stream_end) break; // packet area done
            std::uint8_t header[8];
            if (!read_exact(file, header, 8)) break; // clean EOF
            const std::int32_t sid =
                static_cast<std::int32_t>(header[0]) |
                (static_cast<std::int32_t>(header[1]) << 8) |
                (static_cast<std::int32_t>(header[2]) << 16) |
                (static_cast<std::int32_t>(header[3]) << 24);
            std::uint32_t size = static_cast<std::uint32_t>(header[4]) |
                                 (static_cast<std::uint32_t>(header[5]) << 8) |
                                 (static_cast<std::uint32_t>(header[6]) << 16) |
                                 (static_cast<std::uint32_t>(header[7]) << 24);
            if (size > kMaxPacketBytes) throw std::runtime_error("AEDAT4 packet too large");
            auto ev_it = stream_is_events_.find(sid);
            const bool is_imu = stream_is_imu_[sid];
            const bool is_aps = stream_is_aps_[sid];
            if (ev_it == stream_is_events_.end() || !ev_it->second) {
                if (!is_imu && !is_aps) {
                    // Other non-event data (trigger) — skipped entirely.
                    file.seekg(static_cast<std::streamoff>(size), std::ios::cur);
                    continue;
                }
                // IMU / APS frame packets: decompress like events, decode,
                // surface through the sinks. LZ4-framing is packet-wide.
                body.resize(size);
                if (size > 0 && !read_exact(file, body.data(), size)) {
                    throw std::runtime_error("Truncated AEDAT4 packet");
                }
                const std::uint8_t* pd = body.data();
                std::size_t pn = size;
                if (compression_ != 0) {
                    plain.clear();
                    std::string lz4_err;
                    if (!lz4_decompress_frame(pd, pn, plain, lz4_err)) {
                        throw std::runtime_error("AEDAT4 LZ4 packet: " + lz4_err);
                    }
                    if (plain.size() < 8) {
                        throw std::runtime_error("AEDAT4 LZ4 packet too short");
                    }
                    pd = plain.data();
                    pn = plain.size();
                }
                if (is_imu) decode_imu_body(pd, pn);
                else decode_frame_body(pd, pn);
                continue;
            }
            body.resize(size);
            if (size > 0 && !read_exact(file, body.data(), size)) {
                throw std::runtime_error("Truncated AEDAT4 packet");
            }
            const std::uint8_t* pd = body.data();
            std::size_t pn = size;
            if (compression_ != 0) {
                // dv compresses the ENTIRE body (size prefix + flatbuffer)
                // as one LZ4 frame; the decompressed data is
                // [u32 fbSize][flatbuffer] just like an uncompressed body.
                plain.clear();
                std::string lz4_err;
                if (!lz4_decompress_frame(pd, pn, plain, lz4_err)) {
                    throw std::runtime_error("AEDAT4 LZ4 packet: " + lz4_err);
                }
                if (plain.size() < 8) {
                    throw std::runtime_error("AEDAT4 LZ4 packet too short");
                }
                pd = plain.data();
                pn = plain.size();
            }
            if (pn < 8) throw std::runtime_error("AEDAT4 event packet too short");
            // Body = [u32 fbSize][flatbuffer] in both the plain and the
            // decompressed case; trailing padding after the flatbuffer is
            // possible for compressed packets.
            std::uint32_t declared = 0;
            std::memcpy(&declared, pd, 4);
            if (declared < 8 || declared > pn) {
                throw std::runtime_error("Corrupt AEDAT4 event packet");
            }
            const Fb fb{pd + 4, declared};
            std::size_t table = 0;
            if (!fb.root(table)) throw std::runtime_error("Corrupt AEDAT4 event packet");
            const std::uint32_t ident = fb.u32(4);
            if (ident != 0x53545645u) { // "EVTS"
                throw std::runtime_error("Unexpected packet type in AEDAT4 event stream");
            }
            const std::size_t vec = fb.vector(table, 4);
            if (vec == 0 || !fb.valid(vec, 4)) continue; // empty packet
            const std::uint32_t count = fb.u32(vec);
            if (count == 0) continue;
            if ((pn - vec - 4) / 16 < count) {
                throw std::runtime_error("AEDAT4 event packet truncated");
            }
            batch.clear();
            batch.reserve(count);
            const std::size_t first = vec + 4;
            bool out_of_order = last_t != std::numeric_limits<std::int64_t>::min();
            for (std::uint32_t i = 0; i < count; ++i) {
                const std::size_t off = first + 16 * i;
                std::int64_t t = fb.i64(off);
                const std::int16_t x = fb.i16(off + 8);
                const std::int16_t y = fb.i16(off + 10);
                const std::uint8_t pol = fb.d[off + 12];
                if (!ev_t0_known_) {
                    ev_t0_known_ = true;
                    ev_t0_ = t; // normalize to start at 0 (all streams)
                    last_t = std::numeric_limits<std::int64_t>::min();
                    out_of_order = false;
                    // IMU samples decoded before the first event arrive in
                    // file order — emit them now, normalized.
                    for (const auto& p : imu_pending_norm_) {
                        auto s = p;
                        s.t -= ev_t0_;
                        if (imu_sink_) imu_sink_(s);
                    }
                    imu_pending_norm_.clear();
                }
                t -= ev_t0_;
                if (x < 0 || x >= meta_.width || y < 0 || y >= meta_.height) continue;
                Metavision::EventCD ev;
                ev.t = static_cast<Metavision::timestamp>(t);
                ev.x = static_cast<std::uint16_t>(x);
                ev.y = static_cast<std::uint16_t>(y);
                ev.p = pol ? 1 : 0;
                if (t < last_t) out_of_order = true;
                last_t = std::max(last_t, t);
                batch.push_back(ev);
            }
            if (batch.empty()) continue;
            if (out_of_order) {
                std::stable_sort(batch.begin(), batch.end(),
                                 [](const auto& a, const auto& b) { return a.t < b.t; });
            }
            sink(batch.data(), batch.data() + batch.size());
        }
    } catch (const std::exception& e) {
        error = e.what();
    } catch (...) {
        error = "Unknown error while reading the AEDAT4 file";
    }
    done(error);
}

} // namespace gui
