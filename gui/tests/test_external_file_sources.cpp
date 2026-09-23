// gui/tests/test_external_file_sources.cpp — unit tests for the non-SDK file
// playback sources (AEDAT4 from inivation DV, ALPDATA from Alpsentek) and the
// built-in LZ4 frame decoder they rely on.
//
// Fixtures are assembled byte-by-byte exactly as the recorders write them
// (layouts reverse-engineered from real recordings); end-to-end checks run
// each source through the ExternalFileSource factory. Real-recording smoke
// tests run only when EBPLUS_TEST_AEDAT4 / EBPLUS_TEST_ALPDATA point at files.

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <tuple>
#include <vector>

#include <metavision/sdk/base/events/event_cd.h>

#include "app/external_file_source.h"
#include "app/lz4_frame_decoder.h"

namespace {

// --- byte buffer helpers -----------------------------------------------------

struct Buf {
    std::vector<std::uint8_t> b;

    Buf() = default;
    explicit Buf(std::size_t n) { b.resize(n); }

    std::size_t size() const { return b.size(); }
    void u8(std::uint8_t v) { b.push_back(v); }
    void u16(std::uint16_t v) {
        b.push_back(static_cast<std::uint8_t>(v & 0xFF));
        b.push_back(static_cast<std::uint8_t>((v >> 8) & 0xFF));
    }
    void u32(std::uint32_t v) {
        for (int i = 0; i < 4; ++i) {
            b.push_back(static_cast<std::uint8_t>((v >> (8 * i)) & 0xFF));
        }
    }
    void u64(std::uint64_t v) {
        for (int i = 0; i < 8; ++i) {
            b.push_back(static_cast<std::uint8_t>((v >> (8 * i)) & 0xFF));
        }
    }
    void i64(std::int64_t v) { u64(static_cast<std::uint64_t>(v)); }
    void i32(std::int32_t v) { u32(static_cast<std::uint32_t>(v)); }
    void raw(const std::string& s) { b.insert(b.end(), s.begin(), s.end()); }
    void zeros(std::size_t n) { b.insert(b.end(), n, 0); }
    void align8() { while (b.size() % 8) u8(0); }
    void patch32(std::size_t at, std::uint32_t v) {
        for (int i = 0; i < 4; ++i) {
            b[at + i] = static_cast<std::uint8_t>((v >> (8 * i)) & 0xFF);
        }
    }
    void patch64(std::size_t at, std::uint64_t v) {
        for (int i = 0; i < 8; ++i) {
            b[at + i] = static_cast<std::uint8_t>((v >> (8 * i)) & 0xFF);
        }
    }
};

bool write_file(const std::string& path, const Buf& buf) {
    FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return false;
    const std::size_t n = std::fwrite(buf.b.data(), 1, buf.b.size(), f);
    std::fclose(f);
    return n == buf.b.size();
}

// --- LZ4 builders (simple payloads) --------------------------------------------
//
// Frames are built per the lz4 frame spec v1.6: magic | FLG | BD | HC (the
// 1-byte header checksum is UNCONDITIONAL) | blocks | optional 4-byte
// content checksum. FLG bits: 0x40 = version 01, 0x20 = block independence,
// 0x10 = block checksums, 0x08 = content size, 0x04 = content checksum.

// Frame header prologue: magic, FLG, BD, HC byte.
Buf lz4_header(std::uint8_t flg) {
    Buf out;
    out.u32(0x184D2204);
    out.u8(flg);
    out.u8(0x40); // block max 64K
    out.u8(0x00); // header checksum byte (always present, not verified)
    return out;
}

// LZ4 frame with a single UNCOMPRESSED block (high bit of the block size set).
Buf lz4_frame_stored(const std::string& payload) {
    Buf out = lz4_header(0x60); // version 01, block-independent
    out.u32(static_cast<std::uint32_t>(payload.size()) | 0x80000000u);
    out.raw(payload);
    out.u32(0); // EndMark
    return out;
}

// One LZ4 block sequence: literals "abcde" + match (offset 5, len 5) →
// "abcdeabcde".
Buf lz4_frame_one_match() {
    Buf out = lz4_header(0x60);
    Buf block;
    block.u8(0x51); // literal len 5, match len (1 + 4)
    block.raw("abcde");
    block.u8(5);
    block.u8(0); // offset = 5
    out.u32(static_cast<std::uint32_t>(block.size()));
    out.b.insert(out.b.end(), block.b.begin(), block.b.end());
    out.u32(0);
    return out;
}

} // namespace

// ---------------------------------------------------------------------------

TEST(Lz4Decoder, StoredFrame) {
    const std::string payload = "ALOHA LZ4 STORED BLOCK 0123456789";
    const Buf frame = lz4_frame_stored(payload);
    std::vector<std::uint8_t> out;
    std::string err;
    ASSERT_TRUE(gui::lz4_decompress_frame(frame.b.data(), frame.b.size(), out, err))
        << err;
    ASSERT_EQ(out.size(), payload.size());
    EXPECT_TRUE(std::equal(payload.begin(), payload.end(), out.begin()));
}

TEST(Lz4Decoder, BlockWithMatch) {
    const Buf frame = lz4_frame_one_match();
    std::vector<std::uint8_t> out;
    std::string err;
    ASSERT_TRUE(gui::lz4_decompress_frame(frame.b.data(), frame.b.size(), out, err))
        << err;
    ASSERT_EQ(out.size(), 10u);
    EXPECT_EQ(std::string(out.begin(), out.end()), "abcdeabcde");
}

TEST(Lz4Decoder, HeaderChecksumIsUnconditional) {
    // A frame WITHOUT the mandatory HC byte after the descriptor must be
    // rejected — catching a decoder that treats HC as optional (FLG bit 2 is
    // the CONTENT checksum flag, not "header checksum present").
    Buf frame = lz4_frame_stored("payload");
    frame.b.erase(frame.b.begin() + 6); // drop magic(4)+FLG+BD+HC's last byte
    std::vector<std::uint8_t> out;
    std::string err;
    EXPECT_FALSE(gui::lz4_decompress_frame(frame.b.data(), frame.b.size(), out, err));
    EXPECT_FALSE(err.empty());
}

TEST(Lz4Decoder, DvStyleBlockLinkedCrossBlockMatch) {
    // DV compresses with LZ4F_blockLinked (FLG 0x40): a compressed block may
    // reference the PREVIOUS block's output. Block 1 stores "abcde"; block 2
    // is a zero-literal match (offset 5, len 5) into block 1's output.
    Buf frame = lz4_header(0x40);
    frame.u32(5u | 0x80000000u);
    frame.raw("abcde");
    Buf block;
    block.u8(0x01); // literal len 0, match len (1 + 4)
    block.u8(5);
    block.u8(0); // offset = 5 → resolves into block 1's output
    frame.u32(static_cast<std::uint32_t>(block.size()));
    frame.b.insert(frame.b.end(), block.b.begin(), block.b.end());
    frame.u32(0);
    std::vector<std::uint8_t> out;
    std::string err;
    ASSERT_TRUE(gui::lz4_decompress_frame(frame.b.data(), frame.b.size(), out, err))
        << err;
    ASSERT_EQ(out.size(), 10u);
    EXPECT_EQ(std::string(out.begin(), out.end()), "abcdeabcde");
}

TEST(Lz4Decoder, ContentChecksumSkipped) {
    // FLG 0x64 = version 01 + independent blocks + content checksum: the
    // 4-byte xxh32 after the EndMark must be consumed, not rejected as
    // trailing bytes.
    Buf frame = lz4_header(0x64);
    frame.u32(static_cast<std::uint32_t>(11) | 0x80000000u);
    frame.raw("checksummed");
    frame.u32(0);          // EndMark
    frame.u32(0xDEADBEEF); // content checksum placeholder
    std::vector<std::uint8_t> out;
    std::string err;
    ASSERT_TRUE(gui::lz4_decompress_frame(frame.b.data(), frame.b.size(), out, err))
        << err;
    ASSERT_EQ(out.size(), 11u);
    EXPECT_EQ(std::string(out.begin(), out.end()), "checksummed");
}

TEST(Lz4Decoder, MultiBlockFrame) {
    // One frame carrying TWO uncompressed blocks — exercises the intra-frame
    // block loop (real packets larger than the declared block maximum are
    // split into several blocks by the compressor).
    const std::string p1(100, 'a');
    const std::string p2(100, 'b');
    Buf frame = lz4_header(0x60);
    frame.u32(static_cast<std::uint32_t>(p1.size()) | 0x80000000u);
    frame.raw(p1);
    frame.u32(static_cast<std::uint32_t>(p2.size()) | 0x80000000u);
    frame.raw(p2);
    frame.u32(0); // EndMark
    std::vector<std::uint8_t> out;
    std::string err;
    ASSERT_TRUE(gui::lz4_decompress_frame(frame.b.data(), frame.b.size(), out, err))
        << err;
    ASSERT_EQ(out.size(), p1.size() + p2.size());
    EXPECT_EQ(std::string(out.begin(), out.begin() + 100), p1);
    EXPECT_EQ(std::string(out.begin() + 100, out.end()), p2);
}

TEST(Lz4Decoder, RejectsGarbage) {
    std::vector<std::uint8_t> out;
    std::string err;
    const std::uint8_t junk[16] = {0};
    EXPECT_FALSE(gui::lz4_decompress_frame(junk, sizeof(junk), out, err));
    EXPECT_FALSE(err.empty());
}

// ---------------------------------------------------------------------------
// AEDAT4 fixture
// ---------------------------------------------------------------------------

namespace {

using EventTuple = std::tuple<std::int64_t, std::int16_t, std::int16_t, std::uint8_t>;

// EVTS packet body: [u32 fbSize][fb] with the dv identifier placement
// (ident at [4:8]) and standard flatbuffers field semantics.
Buf evts_packet_body(const std::vector<EventTuple>& evs) {
    Buf b;
    b.u32(16);    // rootUoffset
    b.raw("EVTS");
    b.u16(6);     // vtable size
    b.u16(8);     // table size
    b.u16(4);     // VT4 (elements) offset within table
    b.align8();
    b.u32(8);     // soffset → vtable at 8
    b.u32(4);     // vector displacement → vector at (16+4)+4 = 24
    b.u32(static_cast<std::uint32_t>(evs.size()));
    for (const auto& [t, x, y, p] : evs) {
        b.i64(t);
        b.u16(static_cast<std::uint16_t>(x));
        b.u16(static_cast<std::uint16_t>(y));
        b.u8(p);
        b.zeros(3);
    }
    return b;
}

// Minimal non-event packet body (never parsed — streams are skipped by type).
Buf frme_packet_body() {
    Buf b;
    b.u32(16);
    b.raw("FRME");
    b.u16(4);
    b.u16(8);
    b.align8();
    b.u32(4);
    b.u32(0);
    b.u32(0);
    return b;
}

void append_packet(Buf& f, std::int32_t sid, const Buf& body) {
    f.u32(static_cast<std::uint32_t>(sid));
    f.u32(static_cast<std::uint32_t>(body.size() + 4));
    f.u32(static_cast<std::uint32_t>(body.size()));
    f.b.insert(f.b.end(), body.b.begin(), body.b.end());
}

const char* kAedat4Xml =
    "<dv version=\"2.0\">\n"
    "  <node name=\"outInfo\" path=\"/mainloop/Recorder/outInfo/\">\n"
    "    <node name=\"0\" path=\"/mainloop/Recorder/outInfo/0/\">\n"
    "      <attr key=\"typeIdentifier\" type=\"string\">EVTS</attr>\n"
    "      <node name=\"info\">\n"
    "        <attr key=\"sizeX\" type=\"int\">346</attr>\n"
    "        <attr key=\"sizeY\" type=\"int\">260</attr>\n"
    "        <attr key=\"source\" type=\"string\">DAVIS346_X</attr>\n"
    "      </node>\n"
    "    </node>\n"
    "    <node name=\"1\" path=\"/mainloop/Recorder/outInfo/1/\">\n"
    "      <attr key=\"typeIdentifier\" type=\"string\">FRME</attr>\n"
    "    </node>\n"
    "  </node>\n"
    "</dv>\n";

// IOHeader flatbuffer: compression i32 (VT4), dataTablePosition i64 (VT6),
// infoNode string (VT8). Ident at [4:8]; the XML string is appended after
// the returned buffer.
Buf ioheader_fb(std::int32_t compression, std::size_t* table_pos_patch) {
    Buf b;
    b.u32(24);    // rootUoffset
    b.raw("IOHE");
    b.u16(10);    // vtable size
    b.u16(20);    // table size
    b.u16(4);     // compression offset
    b.u16(12);    // dataTablePosition offset
    b.u16(8);     // infoNode offset
    b.align8();
    b.u32(16);    // soffset → vtable at 8
    b.u32(static_cast<std::uint32_t>(compression)); // inline scalar @ 28
    b.u32(20);    // string displacement → string at 24+8+20 = 52
    *table_pos_patch = b.size();
    b.i64(0);     // dataTablePosition @ 36 (patched)
    b.zeros(8);   // pad to 52
    return b;
}

Buf string_payload(const std::string& s) {
    Buf b;
    b.u32(static_cast<std::uint32_t>(s.size()));
    b.raw(s);
    return b;
}

// FileDataTable region: [u32 size][fb]. Vector of uoffsets → entry tables
// with ByteOffset i64 (VT4), PacketInfo struct 8B (VT6), NumElements i64
// (VT8), TimestampStart i64 (VT10), TimestampEnd i64 (VT12).
struct FtabEntry {
    std::int32_t sid;
    std::int64_t num;
    std::int64_t ts0;
    std::int64_t ts1;
};

Buf ftab_region(const std::vector<FtabEntry>& entries) {
    Buf b;
    const std::size_t size_pos = 0;
    // Buffer = [u32 region size][fb]; fb-relative positions = b - 4.
    b.u32(0);   // region size (patched)          b[0]  = fb[-4]
    b.u32(16);  // rootUoffset                    b[4]  = fb[0]
    b.raw("FTAB");                                 // b[8]  = fb[4]
    b.u16(6);   // vtable size (fb[8])
    b.u16(8);   // table size (fb[10])
    b.u16(4);   // VT4 offset (fb[12])
    b.zeros(2); // pad fb[14..16]
    b.u32(8);   // soffset @fb[16] → vtable at fb[8]
    b.u32(4);   // vector displacement: vector at (16+4)+4 = fb[24]
    b.u32(static_cast<std::uint32_t>(entries.size())); // count @fb[24]
    const std::size_t uoff_base = b.size();
    for (std::size_t i = 0; i < entries.size(); ++i) b.u32(0);
    b.align8();
    std::vector<std::size_t> entry_pos;
    for (std::size_t i = 0; i < entries.size(); ++i) {
        b.align8();
        entry_pos.push_back(b.size());
        b.u32(0);               // soffset (patched)
        b.i64(0);               // ByteOffset (VT4, inline)
        b.i32(entries[i].sid);  // PacketInfo (VT6, inline struct)
        b.i32(0);
        b.i64(entries[i].num);  // VT8
        b.i64(entries[i].ts0);  // VT10
        b.i64(entries[i].ts1);  // VT12
    }
    const std::size_t vtable_pos = b.size();
    b.u16(14);
    b.u16(44);
    b.u16(4);
    b.u16(12);
    b.u16(20);
    b.u16(28);
    b.u16(36);
    for (std::size_t i = 0; i < entries.size(); ++i) {
        b.patch32(uoff_base + 4 * i,
                  static_cast<std::uint32_t>(entry_pos[i] - (uoff_base + 4 * i)));
        b.patch32(entry_pos[i], static_cast<std::uint32_t>(entry_pos[i] - vtable_pos));
    }
    b.patch32(size_pos, static_cast<std::uint32_t>(b.size() - 4));
    return b;
}

} // namespace

TEST(Aedat4Source, EndToEndMiniFile) {
    // Events on stream 0 (epoch-scale µs), a skipped FRME packet on stream 1,
    // and an LZ4-framed EVTS packet. FTAB provides duration + event count.
    const std::int64_t base = 1789561623898715ll;
    Buf file;
    file.raw("#!AER-DAT4.0\r\n");
    const std::size_t hdr_size_pos = file.size();
    file.u32(0);
    const std::size_t io_start = file.size();
    std::size_t table_pos_patch = 0;
    Buf io = ioheader_fb(1 /* LZ4 */, &table_pos_patch);
    const Buf xml = string_payload(kAedat4Xml);
    io.b.insert(io.b.end(), xml.b.begin(), xml.b.end());
    file.patch32(hdr_size_pos, static_cast<std::uint32_t>(io.size()));
    file.b.insert(file.b.end(), io.b.begin(), io.b.end());

    // Packet 1: FRME on stream 1 — with compression=1 every packet body is
    // an LZ4 frame (real dv files compress frames too); the source decodes
    // the frame stream now, so the body must be well-formed LZ4.
    {
        const Buf frme = frme_packet_body();
        const Buf lz4_frme = lz4_frame_stored(
            std::string(frme.b.begin(), frme.b.end()));
        // Same 2-u32 direct form as the EVTS packets below (the reader reads
        // {sid}{size} + body, decompressing the body as one LZ4 frame).
        file.u32(1);
        file.u32(static_cast<std::uint32_t>(lz4_frme.size()));
        file.b.insert(file.b.end(), lz4_frme.b.begin(), lz4_frme.b.end());
    }
    // Packet 2: EVTS on stream 0, LZ4-framed body (compression=1 → every
    // packet body is an LZ4 frame around [u32 fbSize][fb]).
    Buf prefixed2;
    prefixed2.u32(0);
    {
        Buf fb2 = evts_packet_body({
            {base + 500, 5, 6, 1},
            {base + 700, 345, 259, 0},
            {base + 1000, 0, 0, 1},
        });
        prefixed2.b.insert(prefixed2.b.end(), fb2.b.begin(), fb2.b.end());
        prefixed2.patch32(0, static_cast<std::uint32_t>(fb2.size()));
    }
    const Buf lz4_p2 = lz4_frame_stored(
        std::string(prefixed2.b.begin(), prefixed2.b.end()));
    file.u32(0);
    file.u32(static_cast<std::uint32_t>(lz4_p2.size()));
    file.b.insert(file.b.end(), lz4_p2.b.begin(), lz4_p2.b.end());
    // Packet 3: EVTS on stream 0, LZ4-framed body. dv compresses the whole
    // body ([u32 fbSize][fb]) as one LZ4 frame.
    const Buf raw_body = evts_packet_body({
        {base + 1200, 100, 100, 0},
        {base + 1100, 4, 4, 1},
    });
    Buf prefixed_body;
    prefixed_body.u32(static_cast<std::uint32_t>(raw_body.size()));
    prefixed_body.b.insert(prefixed_body.b.end(), raw_body.b.begin(),
                           raw_body.b.end());
    const Buf lz4_body = lz4_frame_stored(
        std::string(prefixed_body.b.begin(), prefixed_body.b.end()));
    file.u32(0);
    file.u32(static_cast<std::uint32_t>(lz4_body.size()));
    file.b.insert(file.b.end(), lz4_body.b.begin(), lz4_body.b.end());

    const std::size_t table_pos = file.size();
    const Buf ftab = ftab_region({
        {0, 4, base + 500, base + 1200},
        {1, 0, 0, 0},
    });
    file.b.insert(file.b.end(), ftab.b.begin(), ftab.b.end());
    file.patch64(io_start + table_pos_patch, table_pos);

    const std::string path = "/tmp/ebplus_test_mini.aedat4";
    ASSERT_TRUE(write_file(path, file));

    auto src = gui::try_open_external_file(path);
    ASSERT_NE(src, nullptr);
    src->open(); // must not throw
    EXPECT_EQ(src->meta().plugin_name, QString("AEDAT4"));
    EXPECT_EQ(src->meta().width, 346);
    EXPECT_EQ(src->meta().height, 260);
    EXPECT_EQ(src->meta().serial, QString("DAVIS346_X"));
    EXPECT_EQ(src->meta().duration_us, 700);
    EXPECT_EQ(src->meta().worst_case_events, 4);

    std::vector<Metavision::EventCD> events;
    std::string done_err = "pending";
    src->run([&events](const Metavision::EventCD* b, const Metavision::EventCD* e) {
        events.insert(events.end(), b, e);
    }, [&done_err](const std::string& err) { done_err = err; });
    EXPECT_TRUE(done_err.empty()) << done_err;
    // 3 uncompressed + 2 LZ4 events; the FRME packet contributes nothing.
    ASSERT_EQ(events.size(), 5u);
    EXPECT_EQ(events[0].t, 0); // normalized to the first event
    EXPECT_EQ(events[0].x, 5);
    EXPECT_EQ(events[0].y, 6);
    EXPECT_EQ(events[0].p, 1);
    EXPECT_EQ(events[1].t, 200);
    EXPECT_EQ(events[2].t, 500);
    // LZ4 packet events, re-sorted into ascending time by the source.
    EXPECT_EQ(events[3].t, 600);
    EXPECT_EQ(events[3].x, 4);
    EXPECT_EQ(events[4].t, 700);
    EXPECT_EQ(events[4].p, 0);
    EXPECT_EQ(events[4].x, 100);

    EXPECT_EQ(gui::try_open_external_file("/tmp/some.raw"), nullptr);
    EXPECT_TRUE(gui::is_external_file_extension("x.AEDAT4"));
    EXPECT_FALSE(gui::is_external_file_extension("x.raw"));
}

TEST(Aedat4Source, RejectsNonEventFile) {
    Buf file;
    file.raw("#!AER-DAT4.0\r\n");
    const std::size_t hdr_size_pos = file.size();
    file.u32(0);
    std::size_t table_pos_patch = 0;
    Buf io = ioheader_fb(0, &table_pos_patch);
    const char* xml =
        "<dv version=\"2.0\"><node name=\"outInfo\">"
        "<node name=\"0\"><attr key=\"typeIdentifier\" type=\"string\">FRME</attr>"
        "</node></node></dv>";
    const Buf xmlb = string_payload(xml);
    io.b.insert(io.b.end(), xmlb.b.begin(), xmlb.b.end());
    file.patch32(hdr_size_pos, static_cast<std::uint32_t>(io.size()));
    file.b.insert(file.b.end(), io.b.begin(), io.b.end());
    append_packet(file, 0, frme_packet_body());

    const std::string path = "/tmp/ebplus_test_frames_only.aedat4";
    ASSERT_TRUE(write_file(path, file));
    auto src = gui::try_open_external_file(path);
    ASSERT_NE(src, nullptr);
    EXPECT_THROW(src->open(), std::runtime_error);
}

// ---------------------------------------------------------------------------
// ALPDATA fixture (minimal HDF5: root → bin → three frame datasets)
// ---------------------------------------------------------------------------

namespace {

constexpr std::size_t kPad8(std::size_t n) { return (n + 7) / 8 * 8; }

constexpr std::uint64_t kHeadMagic = 0xEEF2F2F2F2F2F2F2ull;
constexpr std::uint64_t kTailMagic = 0xEEF3F3F3F3F3F3F3ull;

struct H5Fixture {
    Buf f;
    // (file position, target address) applied once everything is placed.
    std::vector<std::pair<std::size_t, std::uint64_t>> refs;

    void ref64(std::size_t* pos) {  // reserve an address slot
        f.align8();
        *pos = f.size();
        f.u64(0);
    }

    // V1 object header from (type, payload) messages.
    std::size_t object_header(const std::vector<std::pair<std::uint16_t, Buf>>& msgs) {
        f.align8();
        const std::size_t at = f.size();
        std::size_t hdr_size = 0;
        for (const auto& m : msgs) hdr_size += kPad8(8 + m.second.size());
        f.u8(1);  // version
        f.u8(0);  // reserved
        f.u16(static_cast<std::uint16_t>(msgs.size()));
        f.u16(1); // refcount
        f.u16(0); // pad → hdrsize lands at +8 as in real files
        f.u32(static_cast<std::uint32_t>(hdr_size));
        f.u32(0); // pad to the 16-byte prefix
        for (const auto& m : msgs) {
            f.u16(m.first);
            f.u16(static_cast<std::uint16_t>(m.second.size()));
            f.u8(0);
            f.u8(0);
            f.u8(0);
            f.u8(0);
            f.b.insert(f.b.end(), m.second.b.begin(), m.second.b.end());
            f.align8();
        }
        return at;
    }

    // Attribute message body: v1 header + name + dtype + dspace + data.
    // cls: '0' fixed, '1' float, '9' vlen string.
    Buf attr_msg(const std::string& name, char cls, const Buf& data,
                 std::uint32_t dt_data_size) {
        Buf b;
        b.u8(1);
        b.u8(0);
        b.u16(static_cast<std::uint16_t>(name.size() + 1));
        Buf dt;
        dt.u8(static_cast<std::uint8_t>(0x10 | (cls - '0')));
        dt.u8(0);
        dt.u8(0);
        dt.u8(0);
        dt.u32(dt_data_size);
        if (cls == '0') dt.u32(dt_data_size * 8 << 16); // bit offset 0, precision
        const std::size_t dt_size = dt.size();
        b.u16(static_cast<std::uint16_t>(dt_size));
        b.u16(8); // dataspace size (scalar)
        b.raw(name);
        b.u8(0);
        b.align8();
        b.b.insert(b.b.end(), dt.b.begin(), dt.b.end());
        b.align8();
        b.u8(1); // dataspace version
        b.u8(0); // rank 0 (scalar)
        b.u8(1); // flags
        b.zeros(5);
        b.b.insert(b.b.end(), data.b.begin(), data.b.end());
        b.align8();
        return b;
    }

    Buf attr_u32(const std::string& name, std::uint32_t v) {
        Buf d;
        d.u32(v);
        return attr_msg(name, '0', d, 4);
    }

    Buf attr_vlen(const std::string& name, std::uint64_t gaddr, std::uint32_t idx,
                  std::uint32_t len) {
        Buf d;
        d.u32(len);
        d.u64(gaddr);
        d.u32(idx);
        return attr_msg(name, '9', d, 16);
    }

    // V5.0 EVS frame record: 160B header + tiles + 8B tail.
    Buf evs_frame(std::uint64_t ts, std::uint16_t rows, std::uint16_t cols,
                  const std::vector<std::uint8_t>& plane) {
        Buf b;
        b.u64(kHeadMagic);
        b.u8(0x01);          // imageType: EVS
        b.u8(0x01);          // version
        b.u16(160);          // header size
        b.u32(0x000B8020u);  // DataType: EVS normal-2bit
        b.u64(ts);
        b.u64(1);            // frame id
        b.u32(static_cast<std::uint32_t>(plane.size() + 32)); // FPGADataSize
        b.u32(112);          // ExtendDataSize
        b.u16(rows);
        b.u16(cols);
        b.u32(0);
        b.zeros(112);        // ExtendData
        const std::size_t tiles = rows / 16;
        const std::size_t payload_tile = plane.size() / tiles;
        for (std::size_t k = 0; k < tiles; ++k) {
            b.u16(0xFFFF);   // tile sync
            b.u16(0);
            b.u32(static_cast<std::uint32_t>(ts));
            b.u16(0);
            b.u32(0);
            b.u32(0);
            b.b.insert(b.b.end(), plane.begin() + k * payload_tile,
                       plane.begin() + (k + 1) * payload_tile);
            b.u16(0);
            b.u32(0);
            b.u16(static_cast<std::uint16_t>(payload_tile + 32)); // tile length
            b.u16(0);
            b.u16(0xFFFF);
            b.u16(0x0101);
        }
        b.u64(kTailMagic);
        return b;
    }

    Buf aps_frame(std::uint64_t ts) {
        Buf b;
        b.u64(kHeadMagic);
        b.u8(0x00);          // imageType: APS — skipped by the reader
        b.u8(0x01);
        b.u16(160);
        b.u32(0x000B0601u);  // DataType: APS 10-bit (no EVS bit)
        b.u64(ts);
        b.u64(2);
        b.u32(64);
        b.u32(112);
        b.u16(16);
        b.u16(64);
        b.u32(0);
        b.zeros(112);
        b.zeros(64);
        b.u64(kTailMagic);
        return b;
    }
};

} // namespace

TEST(AlpdataSource, EndToEndMiniFile) {
    // Sensor 64×16; two EVS frames (480 / 24480 ticks = 20 / 1020 µs at the
    // 24 MHz device counter → 1000 µs period) around one APS frame (skipped).
    // Painted pixels exercise up/down/zero mapping, the tile payload layout,
    // and the GCOL-backed string attribute.
    constexpr std::uint16_t kRows = 16, kCols = 64;
    std::vector<std::uint8_t> plane(kRows * kCols / 4, 0);
    auto set_px = [&](int x, int y, unsigned v) {
        std::uint8_t* byte = &plane[y * (kCols / 4) + x / 4];
        const int shift = (x % 4) * 2;
        *byte = static_cast<std::uint8_t>((*byte & ~(3 << shift)) | (v << shift));
    };
    set_px(0, 0, 2); // up → p=1
    set_px(1, 0, 1); // down → p=0
    set_px(2, 0, 2); // up
    set_px(3, 1, 2);
    set_px(4, 1, 1);
    for (int x = 0; x < 4; ++x) set_px(x, 15, 1); // bottom row of tile 0

    H5Fixture hb;
    Buf& f = hb.f;

    // Superblock v0 (root entry patched once the root object header exists).
    const std::size_t sb_root_obj_pos = 56 + 8;
    f.zeros(96);
    f.b[0] = 0x89; f.b[1] = 'H'; f.b[2] = 'D'; f.b[3] = 'F';
    f.b[4] = '\r'; f.b[5] = '\n'; f.b[6] = 0x1a; f.b[7] = '\n';
    f.b[13] = 8; // size of offsets
    f.b[14] = 8; // size of lengths

    // GCOL collection: header (16B) + one object {idx=1, len=5, "014BA"}.
    f.align8();
    const std::size_t gcol_pos = f.size();
    {
        f.raw("GCOL");
        f.u8(1);
        f.zeros(3);
        f.u64(64);
        f.u64(1);
        f.u64(5);
        f.raw("014BA");
        f.align8();
    }

    // Root group object header: symtab FIRST (fixed slot position) +
    // GCOL-backed device_type attribute.
    std::vector<std::pair<std::uint16_t, Buf>> root_msgs;
    root_msgs.emplace_back(0x0011, Buf(16)); // symtab payload (patched)
    root_msgs.emplace_back(0x000C, hb.attr_vlen("device_type", gcol_pos, 1, 5));
    const std::size_t root_oh = hb.object_header(root_msgs);
    const std::size_t root_symtab_data = root_oh + 16 + 8;

    // Root B-tree (1 child → root SNOD) + local heap ("bin"). In real files
    // B-tree children of groups are always SNOD leaf nodes.
    f.align8();
    const std::size_t root_btree = f.size();
    {
        f.raw("TREE");
        f.u8(0);
        f.u8(0);
        f.u16(1);
        f.u64(0xFFFFFFFFFFFFFFFFull);
        f.u64(0xFFFFFFFFFFFFFFFFull);
        f.u64(0);                                   // key
        hb.refs.emplace_back(f.size(), 0);          // child → root SNOD
        f.u64(8);                                   // final key
    }
    f.align8();
    const std::size_t root_snod = f.size();
    {
        f.raw("SNOD");
        f.u8(1);
        f.u8(0);
        f.u16(1);
        f.u64(0);                                   // name offset: "bin"
        hb.refs.emplace_back(f.size(), 0);          // object → bin OH
        f.u64(0);
        f.u32(1);
        f.u32(0);
        f.zeros(16);
    }
    f.align8();
    const std::size_t root_heap = f.size();
    {
        f.raw("HEAP");
        f.u8(0);
        f.zeros(3);
        f.u64(8); // "bin\0" + pad
        f.u64(0);
        const std::size_t data_addr_slot = f.size();
        f.u64(0);                                   // data address (patched)
        const std::size_t data_at = f.size();
        f.raw("bin");
        f.u8(0);
        f.align8();
        hb.refs.emplace_back(data_addr_slot, data_at);
    }

    // bin group: symtab FIRST (fixed slot position) + 5 fixed attributes
    // (attribute slots vary in size with the name length, so the symtab
    // must come first for a stable offset).
    std::vector<std::pair<std::uint16_t, Buf>> bin_msgs;
    bin_msgs.emplace_back(0x0011, Buf(16)); // symtab payload (patched)
    bin_msgs.emplace_back(0x000C, hb.attr_u32("width", kCols));
    bin_msgs.emplace_back(0x000C, hb.attr_u32("height", kRows));
    bin_msgs.emplace_back(0x000C, hb.attr_u32("up_event_value", 2));
    bin_msgs.emplace_back(0x000C, hb.attr_u32("down_event_value", 1));
    bin_msgs.emplace_back(0x000C, hb.attr_u32("zero_event_value", 0));
    const std::size_t bin_oh = hb.object_header(bin_msgs);
    const std::size_t bin_symtab_data = bin_oh + 16 + 8;

    // bin B-tree (1 child → SNOD) + SNOD with three dataset entries + heap.
    f.align8();
    const std::size_t bin_btree = f.size();
    {
        f.raw("TREE");
        f.u8(0);
        f.u8(0);
        f.u16(1);
        f.u64(0xFFFFFFFFFFFFFFFFull);
        f.u64(0xFFFFFFFFFFFFFFFFull);
        f.u64(0);
        hb.refs.emplace_back(f.size(), 0);          // child → SNOD
        f.u64(8);
    }
    f.align8();
    const std::size_t bin_snod = f.size();
    {
        f.raw("SNOD");
        f.u8(1);
        f.u8(0);
        f.u16(3);
        for (int i = 0; i < 3; ++i) {
            f.u64(static_cast<std::uint64_t>(i) * 11); // name offsets
            hb.refs.emplace_back(f.size(), 0);         // dataset object header
            f.u64(0);
            f.u32(1);
            f.u32(0);
            f.zeros(16);
        }
    }
    f.align8();
    const std::size_t bin_heap = f.size();
    {
        f.raw("HEAP");
        f.u8(0);
        f.zeros(3);
        f.u64(40); // 3 names × 11 bytes = 33, padded
        f.u64(0);
        hb.refs.emplace_back(f.size(), 0);          // data address
        f.u64(0);
        const std::size_t data_at = f.size();
        for (int i = 0; i < 3; ++i) {
            char name[12];
            std::snprintf(name, sizeof(name), "%010d", i);
            f.raw(name);
            f.u8(0);
        }
        f.align8();
        hb.refs.back().second = data_at;
    }

    // Dataset payloads + object headers (layout addresses patched).
    std::vector<Buf> payload;
    payload.push_back(hb.evs_frame(480, kRows, kCols, plane));    // 480 ticks = 20 µs @ 24 MHz
    payload.push_back(hb.aps_frame(1440)); // 60 µs, between the EVS frames
    payload.push_back(hb.evs_frame(24480, kRows, kCols, plane)); // 1020 µs → period 1000 µs
    std::size_t ds_pos[3], data_pos[3];
    for (int i = 0; i < 3; ++i) {
        f.align8();
        data_pos[i] = f.size();
        f.b.insert(f.b.end(), payload[i].b.begin(), payload[i].b.end());

        Buf dspace, dtype, layout;
        dspace.u8(1);
        dspace.u8(1);
        dspace.u8(1);
        dspace.zeros(5);
        dspace.u64(payload[i].size());
        dspace.u64(payload[i].size());
        dtype.u8(0x10);
        dtype.u8(0);
        dtype.u8(0);
        dtype.u8(0);
        dtype.u32(1);
        dtype.u32(8u << 16);
        dtype.u32(0);
        layout.u8(3);
        layout.u8(1);
        layout.u64(0); // address (patched)
        layout.u64(payload[i].size());
        layout.zeros(6);
        std::vector<std::pair<std::uint16_t, Buf>> msgs;
        msgs.emplace_back(0x0001, std::move(dspace));
        msgs.emplace_back(0x0003, std::move(dtype));
        msgs.emplace_back(0x0008, std::move(layout));
        f.align8();
        ds_pos[i] = f.size();
        hb.object_header(msgs);
        hb.refs.emplace_back(ds_pos[i] + 72 + 8 + 2, data_pos[i]); // layout addr
    }

    // Resolve references.
    hb.refs.emplace_back(sb_root_obj_pos, root_oh);
    hb.refs.emplace_back(root_symtab_data, root_btree);
    hb.refs.emplace_back(root_symtab_data + 8, root_heap);
    hb.refs.emplace_back(root_btree + 24 + 8, root_snod);
    hb.refs.emplace_back(root_snod + 8 + 8, bin_oh); // SNOD entry object
    hb.refs.emplace_back(bin_symtab_data, bin_btree);
    hb.refs.emplace_back(bin_symtab_data + 8, bin_heap);
    hb.refs.emplace_back(bin_btree + 24 + 8, bin_snod);
    for (int i = 0; i < 3; ++i) {
        hb.refs.emplace_back(bin_snod + 8 + 40 * i + 8, ds_pos[i]);
    }
    for (const auto& [pos, value] : hb.refs) {
        f.patch64(pos, value);
    }

    const std::string path = "/tmp/ebplus_test_mini.alpdata";
    ASSERT_TRUE(write_file(path, f));

    auto src = gui::try_open_external_file(path);
    ASSERT_NE(src, nullptr);
    src->open();
    EXPECT_EQ(src->meta().width, 64);
    EXPECT_EQ(src->meta().height, 16);
    EXPECT_EQ(src->meta().serial, QString("014BA")); // via GCOL vlen string
    EXPECT_EQ(src->meta().plugin_name, QString("ALPDATA"));
    EXPECT_EQ(src->meta().duration_us, 1000);
    EXPECT_EQ(src->meta().accumulation_hint_us, 1000);
    EXPECT_EQ(src->meta().worst_case_events, 2 * kRows * kCols);

    std::vector<Metavision::EventCD> events;
    std::string done_err = "pending";
    src->run([&events](const Metavision::EventCD* b, const Metavision::EventCD* e) {
        events.insert(events.end(), b, e);
    }, [&done_err](const std::string& err) { done_err = err; });
    EXPECT_TRUE(done_err.empty()) << done_err;
    // 9 painted pixels per EVS frame × 2 frames; the APS frame is skipped.
    ASSERT_EQ(events.size(), 18u);
    EXPECT_EQ(events[0].t, 0);
    EXPECT_EQ(events[0].x, 0);
    EXPECT_EQ(events[0].y, 0);
    EXPECT_EQ(events[0].p, 1);
    EXPECT_EQ(events[1].x, 1);
    EXPECT_EQ(events[1].p, 0);
    EXPECT_EQ(events[2].x, 2);
    EXPECT_EQ(events[2].p, 1);
    EXPECT_EQ(events[3].x, 3);
    EXPECT_EQ(events[3].y, 1);
    EXPECT_EQ(events[4].x, 4);
    EXPECT_EQ(events[4].y, 1);
    EXPECT_EQ(events[4].p, 0);
    for (int i = 5; i < 9; ++i) { // row 15: x=0..3, polarity down
        EXPECT_EQ(events[i].y, 15);
        EXPECT_EQ(events[i].x, i - 5);
        EXPECT_EQ(events[i].p, 0);
    }
    // Second EVS frame at t=1000 (period = accumulation hint).
    EXPECT_EQ(events[9].t, 1000);
    EXPECT_EQ(events[9].x, 0);
    EXPECT_EQ(events[9].y, 0);
    for (const auto& ev : events) {
        ASSERT_LT(ev.x, 64);
        ASSERT_LT(ev.y, 16);
    }
}

// ---------------------------------------------------------------------------
// Real-recording smoke tests (enabled via environment variables)
// ---------------------------------------------------------------------------

TEST(ExternalSources, RealAedat4Recording) {
    const char* path = std::getenv("EBPLUS_TEST_AEDAT4");
    if (!path || !*path) {
        GTEST_SKIP() << "EBPLUS_TEST_AEDAT4 not set";
    }
    auto src = gui::try_open_external_file(path);
    ASSERT_NE(src, nullptr);
    src->open();
    EXPECT_GT(src->meta().width, 0);
    EXPECT_GT(src->meta().duration_us, 0);
    std::vector<Metavision::EventCD> events;
    std::string err = "pending";
    src->run([&events](const Metavision::EventCD* b, const Metavision::EventCD* e) {
        events.insert(events.end(), b, e);
    }, [&err](const std::string& e) { err = e; });
    EXPECT_TRUE(err.empty()) << err;
    EXPECT_GT(events.size(), 0u);
    for (std::size_t i = 1; i < events.size(); ++i) {
        ASSERT_LE(events[i - 1].t, events[i].t) << "unsorted at " << i;
    }
}

TEST(ExternalSources, RealAlpdataRecording) {
    const char* path = std::getenv("EBPLUS_TEST_ALPDATA");
    if (!path || !*path) {
        GTEST_SKIP() << "EBPLUS_TEST_ALPDATA not set";
    }
    auto src = gui::try_open_external_file(path);
    ASSERT_NE(src, nullptr);
    src->open();
    EXPECT_GT(src->meta().width, 0);
    EXPECT_GT(src->meta().duration_us, 0);
    EXPECT_GT(src->meta().accumulation_hint_us, 0);
    std::vector<Metavision::EventCD> events;
    std::string err = "pending";
    src->run([&events](const Metavision::EventCD* b, const Metavision::EventCD* e) {
        events.insert(events.end(), b, e);
    }, [&err](const std::string& e) { err = e; });
    EXPECT_TRUE(err.empty()) << err;
    EXPECT_GT(events.size(), 0u);
    for (std::size_t i = 1; i < events.size(); ++i) {
        ASSERT_LE(events[i - 1].t, events[i].t) << "unsorted at " << i;
    }
}
