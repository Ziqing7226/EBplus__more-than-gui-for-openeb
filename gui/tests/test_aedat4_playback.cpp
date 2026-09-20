// gui/tests/test_aedat4_playback.cpp — headless GUI-integration check for
// AEDAT4 replay: drives the REAL CameraController + PlaybackController +
// RecorderController stack against a real recording (env-gated, offscreen).
//
// Adaptation contract under test:
//   open_file  → connected / file-source / capabilities (events-only file:
//                imu/aps/trigger/esp all absent)
//   duration   → PlaybackController::query_duration == FTAB metadata
//   playback   → auto-start, position progress, EOF
//   seek       → back to 0 replays; second EOF
//   loop       → wraps past EOF instead of stopping
//   recording  → refused with the live-cameras-only message
//
// Run against a real file:  EBPLUS_TEST_AEDAT4=<path> ./test_aedat4_playback

#include <gtest/gtest.h>

#include <QApplication>
#include <QCheckBox>
#include <QPushButton>

#include <limits>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <thread>

#include "app/aedat4_file_source.h"
#include "app/camera_controller.h"
#include "recorder/aedat4_writer.h"

#include <opencv2/core.hpp>
#include <filesystem>
#include "recorder/playback_controller.h"
#include "recorder/recorder_controller.h"
#include "recorder/record_dialog.h"

namespace {

// Pumps Qt events until @p pred fires or @p timeout_ms elapses.
bool wait_for(const std::function<bool()>& pred, int timeout_ms) {
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
        if (pred()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return pred();
}

} // namespace

TEST(Aedat4Playback, RealFileGuiIntegration) {
    const char* path = std::getenv("EBPLUS_TEST_AEDAT4");
    if (!path || !*path) {
        GTEST_SKIP() << "EBPLUS_TEST_AEDAT4 not set";
    }

    gui::CameraController controller;
    gui::PlaybackController playback;
    playback.set_camera(&controller);

    // --- open + adaptation state -----------------------------------------
    Metavision::timestamp opened_duration = 0;
    QObject::connect(&playback, &gui::PlaybackController::opened,
                     [&](Metavision::timestamp d) { opened_duration = d; });
    ASSERT_TRUE(playback.open_file(QString::fromUtf8(path)));
    QCoreApplication::processEvents(QEventLoop::AllEvents, 50);
    EXPECT_GT(opened_duration, 1000000) << "opened() must report the FTAB duration";
    EXPECT_TRUE(controller.is_connected());
    EXPECT_TRUE(controller.is_file_source());
    EXPECT_FALSE(controller.is_inivation_source());
    // DVXplorer DXAS0115 recordings (the two files under test) — assert
    // against the file's own metadata, never a baked-in sensor assumption.
    EXPECT_GT(controller.sensor_info().width, 0);
    EXPECT_GT(controller.sensor_info().height, 0);
    EXPECT_EQ(controller.sensor_info().encoding_format, QStringLiteral("EVTS"));

    // The recording defines its own stream set (legacy files are
    // events-only; new builds record IMU/APS too) — only the HAL-facility
    // panels are absent for every AEDAT4 file. Consistency pin: if IMU/APS
    // samples were decoded, the capability flag must be up (side-stream
    // presence is discovered by content, never from declarations).
    const auto caps = controller.source_capabilities();
    EXPECT_FALSE(caps.trigger);
    EXPECT_FALSE(caps.esp);

    // The controller mirrors the same FTAB-derived metadata the opened()
    // signal reported.
    EXPECT_EQ(opened_duration, controller.external_duration_hint());

    // --- playback pass 1: EOF must be reached ----------------------------
    std::atomic<bool> eof{false};
    std::atomic<long> frames{0};
    std::atomic<Metavision::timestamp> last_pos{0};
    QObject::connect(controller.frame_pipeline(), &gui::FramePipeline::file_eof_reached,
                     [&eof]() { eof.store(true); });
    QObject::connect(controller.frame_pipeline(), &gui::FramePipeline::file_position_changed,
                     [&](Metavision::timestamp pos, Metavision::timestamp) {
                         last_pos.store(pos);
                         frames.fetch_add(1);
                     });
    // playing state must be ON right after open (auto-start)
    QCoreApplication::processEvents(QEventLoop::AllEvents, 50);
    const bool eof1 = wait_for([&]() { return eof.load(); }, 30000);
    EXPECT_TRUE(eof1);

    // Content/capability consistency after a full pass.
    EXPECT_EQ(controller.source_capabilities().imu,
              controller.imu_sample_count() > 0);
    EXPECT_EQ(controller.source_capabilities().aps,
              controller.aps_frame_count() > 0);

    // --- seek back to 0: replaying after EOF must run to EOF again -------
    // (EOF pauses the player; seek does not auto-resume — the user presses
    // Play. The test does the same.)
    ASSERT_TRUE(playback.seek(0));
    eof.store(false);
    playback.play();
    EXPECT_TRUE(wait_for([&]() { return eof.load(); }, 30000));

    // --- loop: from EOF, enabling loop must wrap instead of stopping -----
    // In loop mode the generator emits `looped` at the wrap point (never
    // eof_reached), and keeps advancing through a second wrap.
    std::atomic<bool> looped{false};
    QObject::connect(controller.frame_pipeline(), &gui::FramePipeline::file_looped,
                     [&looped]() { looped.store(true); });
    playback.set_loop(true);
    playback.play();
    ASSERT_TRUE(wait_for([&]() { return looped.load(); }, 30000))
        << "loop playback never wrapped";
    // Keep running: a second wrap proves continuous looping (no stop).
    looped.store(false);
    EXPECT_TRUE(wait_for([&]() { return looped.load(); }, 30000))
        << "loop playback stopped after the first wrap";
    playback.pause();
    playback.set_loop(false);

    // --- recording a replay must be refused with the friendly message ----
    gui::RecorderController recorder;
    QString recorder_error;
    QObject::connect(&recorder, &gui::RecorderController::error,
                     [&](const QString& e) { recorder_error = e; });
    EXPECT_FALSE(recorder.start(&controller, "/tmp/ebplus_should_not_record.aedat4"));
    EXPECT_TRUE(recorder_error.contains(QStringLiteral("live")))
        << recorder_error.toStdString();

    // --- teardown (controller dtor) must not hang or crash ----------------
    playback.set_camera(nullptr);
}


TEST(Aedat4Playback, SideStreamsFeedControllerSlots) {
    // A recording WITH IMU/APS streams (new-build format): replay must
    // surface the capabilities and feed the SAME controller ring / frame
    // slots the live device feeds — that is what makes the IMU window and
    // the APS window work during replay.
    const std::string side_path =
        (std::filesystem::temp_directory_path() / "ebplus_playback_side.aedat4").string();
    {
        gui::Aedat4Writer writer;
        ASSERT_TRUE(writer.open(side_path, 640, 480, "DXAS0115"));
        std::vector<Metavision::EventCD> evs;
        for (int i = 0; i < 2000; ++i) {
            Metavision::EventCD ev;
            ev.t = 1000LL * i;
            ev.x = static_cast<std::uint16_t>(i % 640);
            ev.y = static_cast<std::uint16_t>((i * 3) % 480);
            ev.p = i % 2;
            evs.push_back(ev);
        }
        writer.write(evs.data(), evs.data() + evs.size());
        for (int i = 0; i < 200; ++i) {
            gui::davis::ImuSample s;
            s.t = 10000LL * i;  // stays inside the event span (0..2 s)
            s.accel_x = -0.9F; s.accel_y = 0.2F; s.accel_z = 0.3F;
            s.gyro_x = 0.5F; s.gyro_y = 1.4F; s.gyro_z = -0.4F;
            s.temperature = 33.0F;
            s.valid = true;
            writer.write_imu(s);
        }
        gui::davis::ApsFrame f;
        f.t = 5000;
        f.width = 8;
        f.height = 4;
        f.image = cv::Mat(4, 8, CV_8UC1, cv::Scalar(77));
        f.valid = true;
        writer.write_aps(f);
        writer.close();
    }

    gui::CameraController controller;
    gui::PlaybackController playback;
    playback.set_camera(&controller);
    ASSERT_TRUE(playback.open_file(QString::fromStdString(side_path)));
    EXPECT_TRUE(controller.is_connected());
    const auto caps = controller.source_capabilities();
    EXPECT_TRUE(caps.imu);
    EXPECT_TRUE(caps.aps);
    EXPECT_TRUE(wait_for([&]() { return controller.imu_sample_count() > 0; }, 15000));
    EXPECT_GT(controller.imu_sample_count(), 0);
    EXPECT_TRUE(wait_for([&]() { return controller.aps_frame_count() > 0; }, 15000));
    const auto aps = controller.latest_aps_frame();
    EXPECT_TRUE(aps.valid);
    EXPECT_EQ(aps.image.cols, 8);
    EXPECT_EQ(aps.image.rows, 4);

    // Mid-playback: the drain is position-gated — only samples at or before
    // the current playback position are served (the attitude animates with
    // the playback instead of jumping to the end).
    ASSERT_TRUE(wait_for([&]() {
        return controller.file_playback_position_us() > 500000;  // past 0.5 s
    }, 10000));
    std::int64_t mid_cursor = std::numeric_limits<std::int64_t>::min();
    const auto mid = controller.drain_imu(mid_cursor);
    ASSERT_FALSE(mid.empty());
    const Metavision::timestamp served_t = mid.back().t;
    EXPECT_LE(served_t, controller.file_playback_position_us());
    EXPECT_LT(served_t, 2000000);  // the recording runs ~2 s of IMU

    // Let the playback run to the end, then check the late-consumer path.
    ASSERT_TRUE(wait_for([&]() {
        return controller.file_playback_position_us() >= 1990000;
    }, 20000));

    // A consumer attached AFTER the replay finished (e.g. the IMU window
    // opened late) must still receive the retained samples — unlike live
    // devices, a replay never delivers them again.
    std::int64_t fresh = std::numeric_limits<std::int64_t>::min();
    const auto backlog = controller.drain_imu(fresh);
    EXPECT_GT(backlog.size(), 0u);
    EXPECT_EQ(backlog.size(), 200u);  // the whole recording's IMU stream
    playback.set_camera(nullptr);
}

TEST(RecordDialog, SideStreamCheckboxes) {
    gui::RecordDialog dialog;
    dialog.set_aedat4_mode(true);
    dialog.set_side_stream_capabilities(true, true);
    dialog.show();
    // Both available in AEDAT4 mode: visible and included by default.
    EXPECT_TRUE(dialog.findChild<QCheckBox*>()->isVisibleTo(&dialog));
    const auto boxes = dialog.findChildren<QCheckBox*>();
    ASSERT_GE(boxes.size(), 3);  // biases + IMU + APS
    QCheckBox* imu = nullptr;
    QCheckBox* aps = nullptr;
    QCheckBox* biases = nullptr;
    for (QCheckBox* box : boxes) {
        if (box->text().contains(QStringLiteral("IMU"))) imu = box;
        else if (box->text().contains(QStringLiteral("APS"))) aps = box;
        else if (box->text().contains(QStringLiteral("biases"))) biases = box;
    }
    ASSERT_NE(imu, nullptr);
    ASSERT_NE(aps, nullptr);
    ASSERT_NE(biases, nullptr);
    EXPECT_TRUE(imu->isVisibleTo(&dialog));
    EXPECT_TRUE(aps->isVisibleTo(&dialog));
    EXPECT_TRUE(imu->isChecked());
    EXPECT_TRUE(aps->isChecked());
    EXPECT_TRUE(biases->isVisibleTo(&dialog));

    // APS-less source (DVXplorer): only the IMU row shows.
    dialog.set_side_stream_capabilities(true, false);
    EXPECT_TRUE(imu->isVisibleTo(&dialog));
    EXPECT_FALSE(aps->isVisibleTo(&dialog));

    // Unchecking propagates through the confirm signal.
    int imu_choice = -1, aps_choice = -1;
    QObject::connect(&dialog, &gui::RecordDialog::start_recording,
                     [&](const QString&, bool, bool i, bool a) {
                         imu_choice = i ? 1 : 0;
                         aps_choice = a ? 1 : 0;
                     });
    imu->setChecked(false);
    aps->setChecked(false);
    QPushButton* start_btn = nullptr;
    const auto buttons = dialog.findChildren<QPushButton*>();
    for (QPushButton* btn : buttons) {
        if (btn->text() == QStringLiteral("Start")) start_btn = btn;
    }
    ASSERT_NE(start_btn, nullptr);
    start_btn->click();  // default output path is prefilled — accepted
    QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
    EXPECT_EQ(imu_choice, 0);
    EXPECT_EQ(aps_choice, 0);

    // RAW mode (Prophesee source): no side-stream rows at all.
    dialog.set_aedat4_mode(false);
    EXPECT_FALSE(imu->isVisibleTo(&dialog));
    EXPECT_FALSE(aps->isVisibleTo(&dialog));
}

int main(int argc, char** argv) {
    qputenv("QT_QPA_PLATFORM", "offscreen");
    QApplication app(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
