// gui/app/alpdata_file_source.cpp — see alpdata_file_source.h for the format.
//
// The HDF5 subset below is keyed to what the Alpsentek recorder emits:
// superblock v0, v1 object headers (with 0x0010 continuation blocks — the
// recorder's "nmsg" count is unreliable, so message areas are walked by
// size), old-style symbol-table groups (B-tree v1 + SNOD + local heap),
// contiguous-storage datasets, and v1 attributes (fixed / float / GCOL-backed
// variable-length strings). Anything outside that subset fails with a clear
// error instead of decoding garbage.

#include "alpdata_file_source.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <limits>
#include <map>
#include <optional>
#include <stdexcept>

namespace gui {
namespace {

[[noreturn]] void fail(const std::string& what) {
    throw std::runtime_error(what);
}

std::size_t pad8(std::size_t n) { return (n + 7) / 8 * 8; }

// --- bounded file access ----------------------------------------------------

class FileReader {
public:
    explicit FileReader(const std::string& path) : file_(path, std::ios::binary) {
        if (!file_) fail("Cannot open ALPDATA file: " + path);
        file_.seekg(0, std::ios::end);
        size_ = static_cast<std::uint64_t>(file_.tellg());
    }
    std::uint64_t size() const { return size_; }

    void read(std::uint64_t off, void* dst, std::size_t n) const {
        if (off > size_ || n > size_ - off) {
            fail("ALPDATA: read past end of file (off=" + std::to_string(off) +
                 " n=" + std::to_string(n) + " size=" + std::to_string(size_) + ")");
        }
        file_.clear();
        file_.seekg(static_cast<std::streamoff>(off));
        file_.read(reinterpret_cast<char*>(dst), static_cast<std::streamsize>(n));
        if (file_.gcount() != static_cast<std::streamsize>(n)) {
            fail("ALPDATA: short read");
        }
    }
    std::vector<std::uint8_t> read_vec(std::uint64_t off, std::size_t n) const {
        std::vector<std::uint8_t> v(n);
        if (n > 0) read(off, v.data(), n);
        return v;
    }
    std::uint64_t u64(std::uint64_t off) const {
        std::uint8_t b[8];
        read(off, b, 8);
        std::uint64_t v;
        std::memcpy(&v, b, 8);
        return v;
    }
    std::uint32_t u32(std::uint64_t off) const {
        std::uint8_t b[4];
        read(off, b, 4);
        std::uint32_t v;
        std::memcpy(&v, b, 4);
        return v;
    }
    std::uint16_t u16(std::uint64_t off) const {
        std::uint8_t b[2];
        read(off, b, 2);
        std::uint16_t v;
        std::memcpy(&v, b, 2);
        return v;
    }

private:
    mutable std::ifstream file_;
    std::uint64_t size_{0};
};

constexpr std::uint64_t kHeadMagic = 0xEEF2F2F2F2F2F2F2ull;
constexpr std::uint64_t kTailMagic = 0xEEF3F3F3F3F3F3F3ull;
constexpr std::size_t kMaxMessageBytes = 64u * 1024 * 1024;
constexpr std::size_t kMaxDatasetBytes = 4u * 1024 * 1024 * 1024ull - 1;
constexpr std::size_t kMaxDatasets = 4'000'000;
constexpr int kMaxWalkDepth = 8;

// --- HDF5-lite ----------------------------------------------------------------

struct H5Message {
    std::uint16_t type{0};
    std::vector<std::uint8_t> data;
};

struct DatasetRecord {
    std::string name;
    std::uint64_t addr{0};
    std::uint64_t size{0};
};

struct AttrValue {
    enum class Kind { Int, Float, Str } kind{Kind::Int};
    std::int64_t i{0};
    double f{0};
    std::string s;
};

class Hdf5Lite {
public:
    explicit Hdf5Lite(const std::string& path) : f_(path), path_(path) {
        parse_superblock();
    }

    void walk() { walk_object(root_object_, std::string(), 0); }

    const std::vector<DatasetRecord>& datasets() const { return datasets_; }

    /// Attribute lookup by NAME (geometry lives on the 'bin' group, device
    /// info on the root group — the same attribute name is not reused with
    /// different values in this writer's files).
    const AttrValue* find_attr(const std::string& name) const {
        auto it = attrs_by_name_.find(name);
        return it == attrs_by_name_.end() ? nullptr : &it->second;
    }

private:
    void parse_superblock() {
        if (f_.size() < 96) fail("ALPDATA: file too small for HDF5");
        if (f_.u64(0) != 0x0A1A0A0D46444889ull) { // "\x89HDF\r\n\x1a\n"
            fail("Not an ALPDATA file (missing HDF5 signature): " + path_);
        }
        std::uint8_t b[8];
        f_.read(8, b, 8);
        if (b[0] != 0) {
            fail("ALPDATA: unsupported HDF5 superblock version " +
                 std::to_string(b[0]));
        }
        if (b[5] != 8 || b[6] != 8) { // bytes 13/14: size of offsets/lengths
            fail("ALPDATA: unsupported HDF5 offset/length size");
        }
        if (f_.u64(24) != 0) fail("ALPDATA: non-zero HDF5 base address is unsupported");
        root_object_ = f_.u64(64); // root symbol table entry @56: {name_off, object}
    }

    std::vector<H5Message> read_message_area(std::uint64_t addr, std::uint64_t len,
                                             int depth) {
        std::vector<H5Message> msgs;
        if (len > kMaxMessageBytes) fail("ALPDATA: oversized HDF5 message area");
        std::uint64_t p = addr;
        const std::uint64_t end = addr + len;
        while (p + 8 <= end) {
            const std::uint16_t type = f_.u16(p);
            const std::uint16_t size = f_.u16(p + 2);
            if (size > kMaxMessageBytes || p + 8 + size > end) break;
            const std::uint64_t next = p + pad8(8 + size);
            if (next <= p) break;
            if (type == 0x0010) {
                // Object header continuation: {u64 offset, u64 length}.
                if (depth > kMaxWalkDepth) fail("ALPDATA: continuation loop");
                const std::uint64_t cont_off = f_.u64(p + 8);
                const std::uint64_t cont_len = f_.u64(p + 16);
                auto cont = read_message_area(cont_off, cont_len, depth + 1);
                msgs.insert(msgs.end(), cont.begin(), cont.end());
            } else if (!(type == 0x0000 && size == 0)) {
                H5Message m;
                m.type = type;
                m.data = f_.read_vec(p + 8, size);
                msgs.push_back(std::move(m));
            }
            p = next;
        }
        return msgs;
    }

    std::vector<H5Message> read_object(std::uint64_t addr) {
        if (addr == 0 || addr == std::numeric_limits<std::uint64_t>::max() ||
            addr + 16 > f_.size()) {
            fail("ALPDATA: invalid HDF5 object address");
        }
        if (f_.u16(addr) != 0x0001) {
            fail("ALPDATA: unsupported HDF5 object header version");
        }
        const std::uint64_t hdr_size = f_.u32(addr + 8);
        return read_message_area(addr + 16, hdr_size, 0);
    }

    static const H5Message* find_msg(const std::vector<H5Message>& msgs,
                                     std::uint16_t type) {
        for (const auto& m : msgs) {
            if (m.type == type) return &m;
        }
        return nullptr;
    }

    /// Resolves a vlen-string reference {u32 len, u64 gcol addr, u32 idx}.
    std::string resolve_gcol(std::uint64_t gaddr, std::uint32_t idx) {
        auto& cache = gcol_cache_[gaddr];
        auto it = cache.find(idx);
        if (it != cache.end()) return it->second;
        if (gaddr + 16 > f_.size()) fail("ALPDATA: corrupt global heap reference");
        if (f_.u32(gaddr) != 0x4C4F4347u) fail("ALPDATA: missing global heap"); // "GCOL"
        std::uint64_t p = gaddr + 16;
        while (p + 16 <= f_.size()) {
            const std::uint64_t obj_idx = f_.u64(p);
            const std::uint64_t obj_len = f_.u64(p + 8);
            if (obj_len > kMaxMessageBytes) fail("ALPDATA: corrupt global heap object");
            if (static_cast<std::uint32_t>(obj_idx) == idx) {
                std::vector<std::uint8_t> bytes =
                    f_.read_vec(p + 16, static_cast<std::size_t>(obj_len));
                std::string s(reinterpret_cast<const char*>(bytes.data()), bytes.size());
                cache.emplace(idx, s);
                return s;
            }
            p += 16 + pad8(static_cast<std::size_t>(obj_len));
        }
        fail("ALPDATA: global heap object not found");
    }

    std::optional<std::pair<std::string, AttrValue>> parse_attr(
        const std::vector<std::uint8_t>& raw) {
        if (raw.size() < 8) fail("ALPDATA: corrupt attribute");
        const std::size_t name_size = raw[2] | (static_cast<std::size_t>(raw[3]) << 8);
        const std::size_t dt_size = raw[4] | (static_cast<std::size_t>(raw[5]) << 8);
        const std::size_t ds_size = raw[6] | (static_cast<std::size_t>(raw[7]) << 8);
        std::size_t p = 8;
        if (p + name_size > raw.size()) fail("ALPDATA: corrupt attribute name");
        const char* name_p = reinterpret_cast<const char*>(raw.data()) + p;
        const std::string name(name_p, strnlen(name_p, name_size));
        p += pad8(name_size);
        // dt_size == 0 with p == raw.size() would otherwise pass the check
        // below and read the class byte one past the buffer, and the
        // cls 0/1 branch reads 4 bytes at p+4 — demand the full datatype
        // message plus the size field up front.
        if (p >= raw.size() || dt_size == 0 || p + dt_size > raw.size()) {
            fail("ALPDATA: corrupt attribute datatype");
        }
        const std::uint8_t cls = raw[p] & 0x0F;
        std::size_t dt_data_size = 0;
        if (cls == 0 || cls == 1) {
            if (dt_size < 8) fail("ALPDATA: corrupt attribute datatype");
            std::uint32_t sz;
            std::memcpy(&sz, raw.data() + p + 4, 4);
            dt_data_size = sz;
        } else if (cls == 9) {
            dt_data_size = 16; // {u32 len; u64 gheap addr; u32 idx}
        } else {
            return std::nullopt; // unsupported attribute datatype — ignore
        }
        p += pad8(dt_size);
        p += ds_size; // dataspace (rank/dims) is not needed for scalars
        if (p >= raw.size()) return std::nullopt;
        AttrValue v;
        if (cls == 0 && dt_data_size == 4 && raw.size() - p >= 4) {
            std::uint32_t x;
            std::memcpy(&x, raw.data() + p, 4);
            v.kind = AttrValue::Kind::Int;
            v.i = x;
        } else if (cls == 0 && dt_data_size == 8 && raw.size() - p >= 8) {
            std::uint64_t x;
            std::memcpy(&x, raw.data() + p, 8);
            v.kind = AttrValue::Kind::Int;
            v.i = static_cast<std::int64_t>(x);
        } else if (cls == 1 && dt_data_size == 8 && raw.size() - p >= 8) {
            std::memcpy(&v.f, raw.data() + p, 8);
            v.kind = AttrValue::Kind::Float;
        } else if (cls == 9 && raw.size() - p >= 16) {
            std::uint32_t len, idx;
            std::uint64_t gaddr;
            std::memcpy(&len, raw.data() + p, 4);
            std::memcpy(&gaddr, raw.data() + p + 4, 8);
            std::memcpy(&idx, raw.data() + p + 12, 4);
            v.kind = AttrValue::Kind::Str;
            v.s = resolve_gcol(gaddr, idx).substr(0, len);
        } else {
            return std::nullopt;
        }
        return std::make_pair(name, v);
    }

    void collect_attributes(const std::vector<H5Message>& msgs) {
        for (const auto& m : msgs) {
            if (m.type != 0x000C) continue;
            auto v = parse_attr(m.data);
            if (v.has_value()) attrs_by_name_.emplace(v->first, v->second);
        }
    }

    /// B-tree v1 children: entries are {key u64, child u64} pairs followed by
    /// a final key (the node type/level bytes are written inconsistently by
    /// this recorder, so children are identified by signature peek instead).
    std::vector<std::uint64_t> btree_children(std::uint64_t addr, int depth) {
        if (depth > kMaxWalkDepth) fail("ALPDATA: B-tree too deep");
        if (addr + 24 > f_.size()) fail("ALPDATA: corrupt B-tree node");
        if (f_.u32(addr) != 0x45455254u) fail("ALPDATA: missing B-tree node"); // "TREE"
        const std::uint64_t ne = f_.u16(addr + 6);
        if (ne > 65535) fail("ALPDATA: corrupt B-tree entry count");
        std::vector<std::uint64_t> children;
        std::uint64_t p = addr + 24;
        for (std::uint64_t i = 0; i < ne; ++i) {
            children.push_back(f_.u64(p + 8)); // skip the key
            p += 16;
        }
        return children;
    }

    void walk_object(std::uint64_t addr, const std::string& path, int depth) {
        if (depth > kMaxWalkDepth) fail("ALPDATA: HDF5 hierarchy too deep");
        const auto msgs = read_object(addr);
        collect_attributes(msgs);
        const H5Message* symtab = find_msg(msgs, 0x0011);
        const H5Message* dspace = find_msg(msgs, 0x0001);
        const H5Message* dtype = find_msg(msgs, 0x0003);
        const H5Message* layout = find_msg(msgs, 0x0008);
        if (symtab != nullptr) {
            if (symtab->data.size() < 16) fail("ALPDATA: corrupt symbol table message");
            std::uint64_t btree, heap;
            std::memcpy(&btree, symtab->data.data(), 8);
            std::memcpy(&heap, symtab->data.data() + 8, 8);
            if (heap + 32 > f_.size() || f_.u32(heap) != 0x50414548u) { // "HEAP"
                fail("ALPDATA: missing local heap");
            }
            const std::uint64_t heap_data = f_.u64(heap + 24);
            walk_btree_named(btree, heap_data, path, depth + 1);
        } else if (dspace != nullptr && dtype != nullptr && layout != nullptr) {
            DatasetRecord rec;
            rec.name = path;
            if (layout->data.size() < 18) fail("ALPDATA: corrupt storage message");
            std::memcpy(&rec.addr, layout->data.data() + 2, 8);
            std::memcpy(&rec.size, layout->data.data() + 10, 8);
            if (rec.addr == 0 || rec.size == 0 || rec.size > kMaxDatasetBytes) {
                fail("ALPDATA: invalid dataset storage");
            }
            if (datasets_.size() > kMaxDatasets) fail("ALPDATA: too many datasets");
            datasets_.push_back(std::move(rec));
        }
        // Objects with neither a symbol table nor dataset messages are
        // committed datatypes / reserved — ignored.
    }

    void walk_btree_named(std::uint64_t btree_addr, std::uint64_t heap_data,
                          const std::string& path, int depth) {
        for (const std::uint64_t child : btree_children(btree_addr, depth)) {
            if (child == 0 || child == std::numeric_limits<std::uint64_t>::max() ||
                child + 8 > f_.size()) {
                continue; // undefined pointers
            }
            const std::uint32_t sig = f_.u32(child);
            if (sig == 0x45455254u) { // "TREE"
                walk_btree_named(child, heap_data, path, depth + 1);
            } else if (sig == 0x444F4E53u) { // "SNOD"
                const std::uint64_t ne = f_.u16(child + 6);
                if (ne > 65535) fail("ALPDATA: corrupt symbol table node");
                std::uint64_t p = child + 8;
                for (std::uint64_t i = 0; i < ne; ++i) {
                    const std::uint64_t name_off = f_.u64(p);
                    const std::uint64_t obj = f_.u64(p + 8);
                    p += 40; // {name u64, obj u64, cache u32, res u32, scratch 16}
                    if (heap_data + name_off >= f_.size()) {
                        fail("ALPDATA: name offset outside heap");
                    }
                    std::string name;
                    {
                        const std::vector<std::uint8_t> chunk =
                            f_.read_vec(heap_data + name_off, 64);
                        for (std::size_t k = 0; k < chunk.size() && chunk[k] != 0; ++k) {
                            name.push_back(static_cast<char>(chunk[k]));
                        }
                    }
                    walk_object(obj, path.empty() ? name : path + "/" + name, depth + 1);
                }
            }
        }
    }

    FileReader f_;
    std::string path_;
    std::uint64_t root_object_{0};
    std::vector<DatasetRecord> datasets_;
    std::map<std::string, AttrValue> attrs_by_name_;
    std::map<std::uint64_t, std::map<std::uint32_t, std::string>> gcol_cache_;
};

// --- V5.0 frame record ---------------------------------------------------------

struct FrameHeader {
    bool is_evs{false};
    std::uint16_t header_size{0};
    std::uint32_t fpga_size{0};
    std::uint16_t rows{0};
    std::uint16_t cols{0};
    std::uint64_t ts{0};
};

bool parse_frame_header(const std::uint8_t* d, std::size_t n, FrameHeader& out) {
    if (n < 48) return false;
    std::uint64_t magic;
    std::memcpy(&magic, d, 8);
    if (magic != kHeadMagic) return false;
    std::memcpy(&out.header_size, d + 0x0A, 2);
    std::uint32_t data_type;
    std::memcpy(&data_type, d + 0x0C, 4);
    out.is_evs = (data_type & 0x8000u) != 0;
    std::memcpy(&out.ts, d + 0x10, 8);
    std::memcpy(&out.fpga_size, d + 0x20, 4);
    std::memcpy(&out.rows, d + 0x28, 2);
    std::memcpy(&out.cols, d + 0x2A, 2);
    return true;
}

std::uint64_t u64le(const std::uint8_t* p) {
    std::uint64_t v;
    std::memcpy(&v, p, 8);
    return v;
}

} // namespace

// ---------------------------------------------------------------------------

void AlpdataFileSource::open() {
    Hdf5Lite h5(path_);
    h5.walk();

    const auto* up = h5.find_attr("up_event_value");
    const auto* down = h5.find_attr("down_event_value");
    const auto* zero = h5.find_attr("zero_event_value");
    if (up != nullptr) up_value_ = up->i;
    if (down != nullptr) down_value_ = down->i;
    if (zero != nullptr) zero_value_ = zero->i;
    if (up_value_ == down_value_ || zero_value_ == up_value_ ||
        zero_value_ == down_value_) {
        fail("ALPDATA: inconsistent up/down/zero event value mapping");
    }
    // data_type names the EVS plane encoding ('normal_v4' = 2-bit polarity
    // plane); encode_type ('ALPIX_V4') is the sensor generation and irrelevant.
    const auto* dtype = h5.find_attr("data_type");
    if (dtype != nullptr && dtype->kind == AttrValue::Kind::Str &&
        dtype->s.find("normal") == std::string::npos) {
        fail("ALPDATA: unsupported EVS encoding '" + dtype->s +
             "' (only normal_2bit event planes are supported)");
    }

    // Zero-padded fixed-width names → lexicographic order == chronological.
    auto datasets = h5.datasets();
    std::sort(datasets.begin(), datasets.end(),
              [](const DatasetRecord& a, const DatasetRecord& b) {
                  return a.name < b.name;
              });

    FileReader f(path_);
    struct Raw {
        FrameHeader hdr;
        std::uint64_t addr;
        std::uint64_t size;
    };
    std::vector<Raw> raws;
    raws.reserve(datasets.size());
    for (const auto& ds : datasets) {
        const std::vector<std::uint8_t> head =
            f.read_vec(ds.addr, static_cast<std::size_t>(std::min<std::uint64_t>(ds.size, 48)));
        FrameHeader hdr;
        if (!parse_frame_header(head.data(), head.size(), hdr)) continue;
        raws.push_back({hdr, ds.addr, ds.size});
    }
    if (raws.empty()) {
        fail("ALPDATA file contains no frame records: " + path_);
    }

    // Unfold the 32-bit device tick counter (long recordings wrap at 2^32),
    // then convert ticks to µs. The V5.0 doc claims a 1 µs FPGA counter, but
    // the recordings disprove it: both files' tick deltas are exactly
    // 199 938/frame while their wall-clock durations (filename start time vs
    // last-write mtime) are 2.04 s / 240 frames and 200.03 s / 24 008 frames
    // → 199 938 ticks ≙ 8.33 ms = the 24 MHz sensor master clock (mclk in
    // APX014BA_MOD_S1.ini; the 7.9 GB file matches to within 0.06 %). 1 tick
    // = 125/3000 µs; scaling first, then normalizing to the first frame.
    constexpr std::int64_t kTickNum = 125;   // ticks * kTickNum / kTickDen → µs
    constexpr std::int64_t kTickDen = 3000;  // (24 MHz = 3000/125 per µs)
    auto tick_to_us = [](std::int64_t ticks) {
        return (ticks * kTickNum + kTickDen / 2) / kTickDen;
    };
    std::uint64_t wraps = 0;
    std::uint64_t prev_raw = 0;
    std::int64_t first_us = -1;
    std::int64_t last_us = 0;
    std::vector<std::int64_t> evs_uss;
    evs_uss.reserve(raws.size());
    std::int64_t worst_pixels = 0;
    for (const auto& r : raws) {
        if (r.hdr.ts < prev_raw) ++wraps;
        prev_raw = r.hdr.ts;
        if (!r.hdr.is_evs) continue; // APS image frame — skipped
        const std::int64_t ext = static_cast<std::int64_t>(r.hdr.ts + (wraps << 32));
        const std::int64_t t = tick_to_us(ext);
        if (first_us < 0) first_us = t;
        last_us = t;
        evs_uss.push_back(t);
        frames_.push_back({r.addr, r.size, t - first_us});
        worst_pixels += static_cast<std::int64_t>(r.hdr.rows) * r.hdr.cols;
    }
    if (frames_.empty()) {
        fail("ALPDATA file contains no EVS event frames — only non-event (APS) "
             "data: " + path_);
    }

    // Geometry: bin-group attributes (width=cols, height=rows), falling back
    // to the first frame's header.
    const auto* wattr = h5.find_attr("width");
    const auto* hattr = h5.find_attr("height");
    if (wattr != nullptr && hattr != nullptr && wattr->i > 0 && hattr->i > 0) {
        meta_.width = static_cast<int>(wattr->i);
        meta_.height = static_cast<int>(hattr->i);
    } else {
        meta_.width = raws.front().hdr.cols;
        meta_.height = raws.front().hdr.rows;
    }
    meta_.duration_us = last_us - first_us;
    meta_.worst_case_events = worst_pixels;
    if (const auto* dev = h5.find_attr("device_type")) {
        if (dev->kind == AttrValue::Kind::Str) meta_.serial = QString::fromStdString(dev->s);
    }
    meta_.integrator = QStringLiteral("Alpsentek");
    meta_.plugin_name = QStringLiteral("ALPDATA");
    meta_.encoding_format =
        dtype != nullptr && dtype->kind == AttrValue::Kind::Str
            ? QString::fromStdString(dtype->s)
            : QStringLiteral("normal_2bit");

    // Playback hint: one accumulation window per recorded frame (all pixels
    // of a frame share its timestamp).
    if (evs_uss.size() >= 2) {
        std::vector<std::int64_t> periods;
        periods.reserve(evs_uss.size() - 1);
        for (std::size_t i = 1; i < evs_uss.size(); ++i) {
            const std::int64_t d = evs_uss[i] - evs_uss[i - 1];
            if (d > 0) periods.push_back(d);
        }
        if (!periods.empty()) {
            std::sort(periods.begin(), periods.end());
            const std::int64_t median = periods[periods.size() / 2];
            meta_.accumulation_hint_us =
                std::min<std::int64_t>(std::max<std::int64_t>(median, 1000), 500000);
        }
    }
}

void AlpdataFileSource::run(EventSink sink, DoneFn done) {
    std::string error;
    try {
        FileReader f(path_);
        std::vector<Metavision::EventCD> batch;
        for (std::size_t fi = 0; fi < frames_.size(); ++fi) {
            if (stop_.load(std::memory_order_relaxed)) break;
            const FrameInfo& fr = frames_[fi];
            if (fr.size > kMaxDatasetBytes) fail("ALPDATA: frame record too large");
            const std::vector<std::uint8_t> data =
                f.read_vec(fr.addr, static_cast<std::size_t>(fr.size));
            FrameHeader hdr;
            if (!parse_frame_header(data.data(), data.size(), hdr) || !hdr.is_evs) {
                fail("ALPDATA: frame " + std::to_string(fi) + " is not an EVS record");
            }
            if (data.size() < hdr.header_size + 8 ||
                u64le(data.data() + data.size() - 8) != kTailMagic) {
                fail("ALPDATA: frame " + std::to_string(fi) + " has a bad tail marker");
            }
            const std::uint64_t rows = hdr.rows;
            const std::uint64_t cols = hdr.cols;
            if (rows == 0 || cols == 0 || rows % 16 != 0 || (rows * cols) % 4 != 0) {
                fail("ALPDATA: unsupported frame geometry " + std::to_string(cols) +
                     "x" + std::to_string(rows));
            }
            const std::uint64_t plane_bytes = rows * cols / 4;
            const std::uint64_t tiles = rows / 16;
            const std::uint64_t row_bytes = cols / 4;
            const std::uint64_t overhead = hdr.fpga_size - plane_bytes;
            if (hdr.fpga_size < plane_bytes || data.size() < hdr.header_size + hdr.fpga_size ||
                overhead != tiles * 32) {
                fail("ALPDATA: unsupported FPGA payload layout (frame " +
                     std::to_string(fi) + ")");
            }
            const std::uint64_t tile_stride = hdr.fpga_size / tiles;
            const std::uint8_t* fpga = data.data() + hdr.header_size;
            batch.clear();
            batch.reserve(static_cast<std::size_t>(rows * cols));
            for (std::uint64_t k = 0; k < tiles; ++k) {
                for (std::uint64_t i = 0; i < 16; ++i) {
                    const std::uint8_t* src =
                        fpga + k * tile_stride + 18 + i * row_bytes; // 18B tile header
                    const std::uint16_t y = static_cast<std::uint16_t>(k * 16 + i);
                    for (std::uint64_t byte_i = 0; byte_i < row_bytes; ++byte_i) {
                        const std::uint8_t b = src[byte_i];
                        if (b == 0) continue; // 4 empty pixels
                        const std::uint64_t x0 = byte_i * 4;
                        for (int q = 0; q < 4; ++q) {
                            const unsigned v = (b >> (2 * q)) & 3;
                            if (v == static_cast<unsigned>(zero_value_)) continue;
                            short p;
                            if (v == static_cast<unsigned>(up_value_)) {
                                p = 1;
                            } else if (v == static_cast<unsigned>(down_value_)) {
                                p = 0;
                            } else {
                                continue;
                            }
                            Metavision::EventCD ev;
                            ev.t = fr.t_us;
                            ev.x = static_cast<std::uint16_t>(x0 + q);
                            ev.y = y;
                            ev.p = p;
                            // The SDK frame generator and the algorithm
                            // backends index buffers sized by the bin-group
                            // geometry WITHOUT per-event bounds checks; a
                            // frame header disagreeing with the bin-group
                            // attributes (corrupt/heterogeneous file) must
                            // not reach them (mirrors the AEDAT4 reader).
                            if (ev.x >= meta_.width || ev.y >= meta_.height) {
                                continue;
                            }
                            batch.push_back(ev);
                        }
                    }
                }
            }
            if (!batch.empty()) sink(batch.data(), batch.data() + batch.size());
        }
    } catch (const std::exception& e) {
        error = e.what();
    } catch (...) {
        error = "Unknown error while reading the ALPDATA file";
    }
    done(error);
}

} // namespace gui
