// gui/recorder/recorder_controller.h — real-time RAW recording (design §3.3.1).
//
// Wraps I_EventsStream::log_raw_data() to record the live camera stream to a
// RAW file. Recording is only available for live cameras (not file playback).
// A QTimer reports the elapsed recording time once per second.

#ifndef GUI_RECORDER_RECORDER_CONTROLLER_H
#define GUI_RECORDER_RECORDER_CONTROLLER_H

#include <QObject>
#include <QPointer>
#include <QString>
#include <QTimer>
#include <atomic>
#include <chrono>
#include <memory>

#include <metavision/sdk/stream/raw_evt2_event_file_writer.h>

#include "recorder/aedat4_writer.h"

namespace gui {

class CameraController;
class FramePipeline;

class RecorderController : public QObject {
    Q_OBJECT
public:
    explicit RecorderController(QObject* parent = nullptr);
    ~RecorderController();

    /// @brief Starts recording the live camera stream to @p path (RAW format).
    /// @return true on success.
    bool start(CameraController* controller, const QString& path);
    /// @brief Starts processed-stream recording (Phase 2.5 step 5): writes
    /// the display-path-preprocessed events (filter/undistort/downsample as
    /// currently configured) instead of the SDK raw log. Falls back to raw
    /// spans when all stages are off mid-recording (continuous output).
    bool start_processed(CameraController* controller, const QString& path,
                         FramePipeline* fp);
    void stop();

    bool is_recording() const { return recording_; }

signals:
    void recording_started(const QString& path);
    void recording_stopped(const QString& path);
    void elapsed(std::chrono::seconds s);
    void error(const QString& msg);

private:
    bool recording_{false};
    bool processed_mode_{false};
    /// Phase 4: inivation sources record to AEDAT4 (DV-native format) via
    /// the controller's raw-device tap. The tap lambda holds a shared_ptr:
    /// an in-flight batch on the USB thread keeps the writer alive while
    /// stop() tears the recording down on the GUI thread.
    bool aedat4_mode_{false};
    std::shared_ptr<Aedat4Writer> aedat4_writer_;
    /// Side-stream state before the recording auto-enabled them — restored
    /// on stop() so a recording never permanently changes the user's panel.
    bool imu_was_enabled_{false};
    bool aps_was_enabled_{false};
    QString path_;
    QTimer timer_;           ///< Emits elapsed() once per second.
    QTimer flush_timer_;     ///< Calls I_EventsStream::get_latest_raw_data() to flush buffers.
    QPointer<CameraController> controller_;
    /// Processed-stream recording (Phase 2.5 step 5).
    FramePipeline* fp_{nullptr};
    std::unique_ptr<Metavision::RAWEvt2EventFileWriter> writer_;
    bool writer_error_reported_{false};
    /// Events written in processed mode (telemetry: the status line shows
    /// the live count so an empty/failed recording is visible immediately).
    std::atomic<std::uint64_t> written_events_{0};
    std::chrono::steady_clock::time_point start_time_;

public:
    /// @brief Events written so far (processed + AEDAT4 modes; 0 in SDK
    /// raw mode).
    std::uint64_t events_written() const { return written_events_.load(); }
    /// @brief True while a processed-mode recording is active.
    bool is_processed_recording() const { return recording_ && processed_mode_; }
private:
};

} // namespace gui

#endif // GUI_RECORDER_RECORDER_CONTROLLER_H
