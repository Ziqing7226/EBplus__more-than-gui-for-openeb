// gui/tests/test_aedat4_writer.cpp — Phase 4 round-trip: Aedat4Writer output
// must be readable by our AEDAT4 file source with event-exact fidelity (the
// reader was validated byte-level against real DV files, so this transitively
// validates the writer against the DV-native format).

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <vector>

#include "app/aedat4_file_source.h"
#include "recorder/aedat4_writer.h"

#include <opencv2/core.hpp>

namespace {

std::vector<Metavision::EventCD> read_all(const std::string& path,
                                          gui::ExternalFileMeta& meta) {
    gui::Aedat4FileSource source(path);
    source.open();
    meta = source.meta();
    std::vector<Metavision::EventCD> out;
    std::string error;
    source.run([&out](const Metavision::EventCD* b, const Metavision::EventCD* e) {
                  out.insert(out.end(), b, e);
              },
              [&error](const std::string& err) { error = err; });
    if (!error.empty()) ADD_FAILURE() << "reader error: " << error;
    return out;
}

} // namespace

TEST(Aedat4Writer, RoundTripEventExact) {
    const std::string path =
        (std::filesystem::temp_directory_path() / "ebplus_aedat4_rt.aedat4").string();

    // Three batches (forces several packets): timestamps start at 0 so the
    // reader's t0 normalization is the identity; both polarities; in-bounds
    // coordinates only (the reader drops out-of-range events).
    std::vector<std::vector<Metavision::EventCD>> batches(3);
    std::int64_t t = 0;
    for (int b = 0; b < 3; ++b) {
        for (int i = 0; i < 977; ++i) {  // 977: prime, non-multiple of 16
            Metavision::EventCD ev;
            ev.t = t;
            ev.x = static_cast<std::uint16_t>((i * 7) % 346);
            ev.y = static_cast<std::uint16_t>((i * 13) % 260);
            ev.p = (i + b) % 2;
            batches[static_cast<std::size_t>(b)].push_back(ev);
            t += 17;
        }
    }

    gui::Aedat4Writer writer;
    ASSERT_TRUE(writer.open(path, 346, 260, "TEST-0001"));
    for (const auto& batch : batches) writer.write(batch.data(), batch.data() + batch.size());
    writer.close();
    ASSERT_EQ(writer.events_written(), 3u * 977);

    gui::ExternalFileMeta meta;
    const auto read_back = read_all(path, meta);
    ASSERT_EQ(read_back.size(), batches[0].size() + batches[1].size() + batches[2].size());
    EXPECT_EQ(meta.width, 346);
    EXPECT_EQ(meta.height, 260);
    EXPECT_EQ(meta.integrator, QStringLiteral("inivation"));
    EXPECT_EQ(meta.encoding_format, QStringLiteral("EVTS"));
    EXPECT_GT(meta.duration_us, 0);

    std::size_t k = 0;
    for (const auto& batch : batches) {
        for (const auto& ev : batch) {
            SCOPED_TRACE(k);
            EXPECT_EQ(read_back[k].t, ev.t);
            EXPECT_EQ(read_back[k].x, ev.x);
            EXPECT_EQ(read_back[k].y, ev.y);
            EXPECT_EQ(read_back[k].p, ev.p);
            ++k;
        }
    }
    std::filesystem::remove(path);
}

TEST(Aedat4Writer, RoundTripImuAndApsStreams) {
    // The side streams must survive write→read exactly: IMU samples land on
    // the IMU sink, the APS frame on the APS sink, events stay event-exact.
    const std::string path =
        (std::filesystem::temp_directory_path() / "ebplus_aedat4_side.aedat4").string();

    gui::Aedat4Writer writer;
    ASSERT_TRUE(writer.open(path, 346, 260, "TEST-0002"));

    std::vector<Metavision::EventCD> evs;
    for (int i = 0; i < 10; ++i) {
        // NB: assign by member name — Metavision::EventCD's declaration
        // order is {x, y, p, t} (t LAST), so brace-init with a leading
        // timestamp silently assigns the wrong members.
        Metavision::EventCD ev;
        ev.t = 1000LL * i;
        ev.x = static_cast<std::uint16_t>(i);
        ev.y = static_cast<std::uint16_t>(2 * i);
        ev.p = 1;
        evs.push_back(ev);
    }
    writer.write(evs.data(), evs.data() + evs.size());

    // 100 samples > the 64-sample packet threshold → several IMU packets.
    std::vector<gui::davis::ImuSample> imus;
    for (int i = 0; i < 100; ++i) {
        gui::davis::ImuSample s;
        s.t = 500LL * i;
        s.temperature = 30.0F + i;
        s.accel_x = -0.9F + 0.01F * i;
        s.accel_y = 0.2F;
        s.accel_z = 0.3F;
        s.gyro_x = 0.5F + i;
        s.gyro_y = 1.4F;
        s.gyro_z = -0.4F;
        s.valid = true;
        imus.push_back(s);
        writer.write_imu(s);
    }

    gui::davis::ApsFrame frame;
    frame.t = 7000;
    frame.width = 8;
    frame.height = 4;
    frame.image = cv::Mat(4, 8, CV_8UC1);
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 8; ++c) {
            frame.image.at<std::uint8_t>(r, c) = static_cast<std::uint8_t>(r * 8 + c);
        }
    }
    frame.valid = true;
    writer.write_aps(frame);
    writer.close();

    gui::Aedat4FileSource source(path);
    source.open();
    ASSERT_TRUE(source.has_imu());
    ASSERT_TRUE(source.has_aps());

    std::vector<gui::davis::ImuSample> imu_out;
    std::vector<gui::davis::ApsFrame> aps_out;
    source.set_imu_sink([&](const gui::davis::ImuSample& s) { imu_out.push_back(s); });
    source.set_aps_sink([&](const gui::davis::ApsFrame& f) { aps_out.push_back(f); });

    std::vector<Metavision::EventCD> ev_out;
    std::string error;
    source.run([&](const Metavision::EventCD* b, const Metavision::EventCD* e) {
                   ev_out.insert(ev_out.end(), b, e);
               },
               [&](const std::string& err) { error = err; });
    EXPECT_TRUE(error.empty()) << error;

    ASSERT_EQ(ev_out.size(), evs.size());
    for (std::size_t i = 0; i < evs.size(); ++i) {
        EXPECT_EQ(ev_out[i].t, evs[i].t);
        EXPECT_EQ(ev_out[i].x, evs[i].x);
    }
    ASSERT_EQ(imu_out.size(), imus.size());
    for (std::size_t i = 0; i < imus.size(); ++i) {
        EXPECT_EQ(imu_out[i].t, imus[i].t);
        EXPECT_FLOAT_EQ(imu_out[i].accel_x, imus[i].accel_x);
        EXPECT_FLOAT_EQ(imu_out[i].accel_y, imus[i].accel_y);
        EXPECT_FLOAT_EQ(imu_out[i].accel_z, imus[i].accel_z);
        EXPECT_FLOAT_EQ(imu_out[i].gyro_x, imus[i].gyro_x);
        EXPECT_FLOAT_EQ(imu_out[i].gyro_y, imus[i].gyro_y);
        EXPECT_FLOAT_EQ(imu_out[i].gyro_z, imus[i].gyro_z);
        EXPECT_FLOAT_EQ(imu_out[i].temperature, imus[i].temperature);
    }
    ASSERT_EQ(aps_out.size(), 1u);
    EXPECT_EQ(aps_out[0].t, 7000);
    ASSERT_EQ(aps_out[0].image.rows, 4);
    ASSERT_EQ(aps_out[0].image.cols, 8);
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 8; ++c) {
            EXPECT_EQ(aps_out[0].image.at<std::uint8_t>(r, c),
                      static_cast<std::uint8_t>(r * 8 + c));
        }
    }
}

TEST(Aedat4Writer, LargeStreamSpawnsMultiplePackets) {
    const std::string path =
        (std::filesystem::temp_directory_path() / "ebplus_aedat4_multi.aedat4").string();

    gui::Aedat4Writer writer;
    ASSERT_TRUE(writer.open(path, 640, 480, "TEST-MULTI"));
    // > the 2048-event flush threshold → at least two packets, each recorded
    // in the data table.
    std::vector<Metavision::EventCD> big;
    for (int i = 0; i < 5000; ++i) {
        Metavision::EventCD ev;
        ev.t = 10 * i;
        ev.x = static_cast<std::uint16_t>(i % 640);
        ev.y = static_cast<std::uint16_t>((i * 3) % 480);
        ev.p = i % 2;
        big.push_back(ev);
    }
    writer.write(big.data(), big.data() + big.size());
    writer.close();

    gui::ExternalFileMeta meta;
    const auto read_back = read_all(path, meta);
    ASSERT_EQ(read_back.size(), big.size());
    for (std::size_t i = 0; i < big.size(); ++i) {
        EXPECT_EQ(read_back[i].t, big[i].t);
        EXPECT_EQ(read_back[i].x, big[i].x);
        EXPECT_EQ(read_back[i].y, big[i].y);
        EXPECT_EQ(read_back[i].p, big[i].p);
    }
    EXPECT_EQ(meta.duration_us, 10 * 4999);
    std::filesystem::remove(path);
}

TEST(Aedat4Writer, WriteAfterCloseIsIgnored) {
    const std::string path =
        (std::filesystem::temp_directory_path() / "ebplus_aedat4_closed.aedat4").string();
    gui::Aedat4Writer writer;
    ASSERT_TRUE(writer.open(path, 10, 10, "X"));
    writer.close();
    ASSERT_FALSE(writer.is_open());

    std::vector<Metavision::EventCD> evs(10, Metavision::EventCD{1, 1, 1, 1});
    writer.write(evs.data(), evs.data() + evs.size());  // no-op, no crash

    gui::ExternalFileMeta meta;
    const auto read_back = read_all(path, meta);
    EXPECT_TRUE(read_back.empty());  // nothing was recorded
    std::filesystem::remove(path);
}

TEST(Aedat4Writer, RoundTripColorFrameAndFormatByte) {
    // Color frames must round-trip channel-exact, and the on-disk format
    // byte must be dv's OPENCV_8U_C3 (16) — the pre-fix writer emitted 2,
    // which the schema reads as OPENCV_16U_C1 (DV rejected/decoded wrong).
    const std::string path =
        (std::filesystem::temp_directory_path() / "ebplus_aedat4_c3.aedat4").string();

    gui::Aedat4Writer writer;
    ASSERT_TRUE(writer.open(path, 8, 4, "TEST-C3"));

    gui::davis::ApsFrame frame;
    frame.t = 5000;
    frame.width = 8;
    frame.height = 4;
    frame.image = cv::Mat(4, 8, CV_8UC3);
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 8; ++c) {
            frame.image.at<cv::Vec3b>(r, c) = cv::Vec3b(
                static_cast<std::uint8_t>(r * 10),
                static_cast<std::uint8_t>(c * 5),
                static_cast<std::uint8_t>(r * 3 + c));
        }
    }
    frame.valid = true;
    writer.write_aps(frame);
    writer.close();

    // Byte-level: the FRME packet's format byte (packet buffer offset 44,
    // i.e. 40 bytes past the ident) must be 16. Skip the IOHeader region —
    // its XML contains the literal "FRME" as the stream typeIdentifier.
    std::ifstream in(path, std::ios::binary);
    const std::vector<char> bytes{std::istreambuf_iterator<char>(in), {}};
    in.close();
    ASSERT_GT(bytes.size(), 18u);
    const std::size_t io_size = static_cast<std::size_t>(
        static_cast<std::uint8_t>(bytes[14]) |
        (static_cast<std::uint8_t>(bytes[15]) << 8) |
        (static_cast<std::uint8_t>(bytes[16]) << 16) |
        (static_cast<std::uint8_t>(bytes[17]) << 24));
    std::size_t frme = bytes.size();
    for (std::size_t i = 18 + io_size; i + 4 < bytes.size(); ++i) {
        if (bytes[i] == 'F' && bytes[i + 1] == 'R' && bytes[i + 2] == 'M' &&
            bytes[i + 3] == 'E') {
            frme = i;
            break;
        }
    }
    ASSERT_LT(frme + 41, bytes.size()) << "no FRME packet found";
    EXPECT_EQ(static_cast<std::uint8_t>(bytes[frme + 40]), 16)
        << "color frame format must be dv OPENCV_8U_C3";

    gui::Aedat4FileSource source(path);
    source.open();
    ASSERT_TRUE(source.has_aps());
    std::vector<gui::davis::ApsFrame> aps_out;
    source.set_aps_sink([&](const gui::davis::ApsFrame& f) { aps_out.push_back(f); });
    std::string error;
    source.run([](const Metavision::EventCD*, const Metavision::EventCD*) {},
               [&](const std::string& err) { error = err; });
    EXPECT_TRUE(error.empty()) << error;
    ASSERT_EQ(aps_out.size(), 1u);
    ASSERT_EQ(aps_out[0].image.type(), CV_8UC3);
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 8; ++c) {
            EXPECT_EQ(aps_out[0].image.at<cv::Vec3b>(r, c),
                      frame.image.at<cv::Vec3b>(r, c));
        }
    }
    std::filesystem::remove(path);
}

namespace {

std::uint16_t rd16(const std::vector<std::uint8_t>& b, std::size_t at) {
    return static_cast<std::uint16_t>(b[at]) |
           static_cast<std::uint16_t>(static_cast<std::uint16_t>(b[at + 1]) << 8);
}
std::uint32_t rd32(const std::vector<std::uint8_t>& b, std::size_t at) {
    std::uint32_t v = 0;
    for (int i = 0; i < 4; ++i) v |= static_cast<std::uint32_t>(b[at + i]) << (8 * i);
    return v;
}
std::int64_t rd64(const std::vector<std::uint8_t>& b, std::size_t at) {
    std::uint64_t v = 0;
    for (int i = 0; i < 8; ++i) v |= static_cast<std::uint64_t>(b[at + i]) << (8 * i);
    return static_cast<std::int64_t>(v);
}

} // namespace

TEST(Aedat4Writer, DataTableByteOffsetsPointAtPackets) {
    // dv's reader seeks every packet by FileDataDefinition.ByteOffset (the
    // absolute offset of the packet BODY, written as mByteOffset +
    // sizeof(PacketHeader)). Without that field dv reads each packet at file
    // position 0 and rejects the file outright. This walks the FTAB with an
    // independent vtable lookup (NOT our reader, which ignores ByteOffset)
    // and checks each entry's ByteOffset lands on its own packet header.
    const std::string path =
        (std::filesystem::temp_directory_path() / "ebplus_aedat4_off.aedat4").string();

    gui::Aedat4Writer writer;
    ASSERT_TRUE(writer.open(path, 346, 260, "TEST-OFFSET"));
    for (int pkt = 0; pkt < 3; ++pkt) {
        std::vector<Metavision::EventCD> evs(2500);  // > 2048 flush threshold
        for (std::size_t i = 0; i < evs.size(); ++i) {
            evs[i].t = 1000LL * pkt + static_cast<std::int64_t>(i);
            evs[i].x = static_cast<std::uint16_t>(i % 346);
            evs[i].y = static_cast<std::uint16_t>(i % 260);
            evs[i].p = 1;
        }
        writer.write(evs.data(), evs.data() + evs.size());
    }
    writer.close();

    std::ifstream in(path, std::ios::binary);
    ASSERT_TRUE(in.good());
    const std::vector<std::uint8_t> bytes{std::istreambuf_iterator<char>(in),
                                          std::istreambuf_iterator<char>()};
    in.close();
    ASSERT_GT(bytes.size(), 14u + 4u);
    EXPECT_EQ(std::string(bytes.begin(), bytes.begin() + 14), "#!AER-DAT4.0\r\n");

    // IOHeader buffer sits at [14 magic][u32 ioSize]; its dataTablePosition
    // i64 is at IOHeader-relative offset 24 (writer's build_io_header).
    const std::size_t io_at = 14 + 4;
    const std::size_t ftab_at = static_cast<std::size_t>(rd64(bytes, io_at + 24));
    ASSERT_LT(ftab_at + 4 + 8, bytes.size());
    ASSERT_EQ(rd32(bytes, ftab_at + 4), 8u);  // FTAB root → table@8
    EXPECT_EQ(std::string(bytes.begin() + ftab_at + 8, bytes.begin() + ftab_at + 12),
              "FTAB");

    // table@8: soffset −8 → vtable@16; VT4@12 → vector@24. All buffer
    // offsets sit after the region-size u32, i.e. at ftab_at + 4 + rel.
    const std::size_t vec = ftab_at + 4 + 24;
    const std::uint32_t count = rd32(bytes, vec);
    ASSERT_GE(count, 3u);
    std::int64_t prev_end = 0;
    for (std::uint32_t i = 0; i < count; ++i) {
        const std::size_t slot = vec + 4 + 4 * i;
        const std::size_t entry = slot + rd32(bytes, slot);
        // vtable lookup: vtable = entry − soffset; the soffset is NEGATIVE
        // (the vtable trails the table), so this lands at entry + |soffset|.
        const std::int32_t soffset = static_cast<std::int32_t>(rd32(bytes, entry));
        ASSERT_LT(soffset, 0);  // vtable trails the table in dv's FTAB
        const std::size_t vtable = entry + static_cast<std::size_t>(-soffset);
        const std::uint16_t off_info = rd16(bytes, vtable + 6);
        const std::uint16_t off_byte = rd16(bytes, vtable + 4);
        ASSERT_NE(off_info, 0) << "entry " << i << ": PacketInfo missing";
        ASSERT_NE(off_byte, 0) << "entry " << i << ": ByteOffset missing";
        const std::int32_t sid = static_cast<std::int32_t>(rd32(bytes, entry + off_info));
        const std::int32_t size = static_cast<std::int32_t>(rd32(bytes, entry + off_info + 4));
        const std::int64_t byte_offset = rd64(bytes, entry + off_byte);
        ASSERT_GT(byte_offset, 0) << "entry " << i;
        // dv seeks byte_offset − sizeof(PacketHeader) to read the header:
        // it must carry exactly this entry's stream id and body size.
        ASSERT_GE(static_cast<std::size_t>(byte_offset), 8u);
        const std::size_t header_at = static_cast<std::size_t>(byte_offset) - 8;
        EXPECT_EQ(static_cast<std::int32_t>(rd32(bytes, header_at)), sid)
            << "entry " << i;
        EXPECT_EQ(static_cast<std::int32_t>(rd32(bytes, header_at + 4)), size)
            << "entry " << i;
        EXPECT_GT(byte_offset, prev_end) << "entries must ascend";
        prev_end = byte_offset + size;
    }
    EXPECT_LE(prev_end, static_cast<std::int64_t>(ftab_at));
    std::filesystem::remove(path);
}
