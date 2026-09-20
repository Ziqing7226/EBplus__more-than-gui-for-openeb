// gui/recorder/recorder_controller.cpp

#include "recorder_controller.h"

#include <metavision/hal/facilities/i_events_stream.h>
#include <metavision/sdk/stream/camera.h>

#include "app/camera_controller.h"
#include "app/frame_pipeline.h"

namespace gui {

RecorderController::RecorderController(QObject* parent) : QObject(parent) {
    connect(&timer_, &QTimer::timeout, this, [this]() {
        emit elapsed(std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - start_time_));
    });
    flush_timer_.setInterval(20);
    connect(&flush_timer_, &QTimer::timeout, this, [this]() {
        if (!recording_ || !controller_) return;
        auto* cam = controller_->camera_handle();
        if (!cam) return;
        try {
            if (auto* stream = cam->get_device().get_facility<Metavision::I_EventsStream>()) {
                stream->get_latest_raw_data();
            }
        } catch (...) {}
    });
}

RecorderController::~RecorderController() {
    stop();
}

bool RecorderController::start(CameraController* controller, const QString& path) {
    if (recording_ || !controller || path.isEmpty()) {
        return false;
    }
    // Recording is only meaningful for live cameras: a file-playback source
    // has no underlying hardware stream to record.
    if (controller->is_file_source()) {
        emit error(tr("Recording is only available for live cameras."));
        return false;
    }
    // Recording while the camera is not streaming produces an empty file
    // (audit §六-C5) — require an active stream.
    if (!controller->is_running()) {
        emit error(tr("Camera is not streaming. Start streaming before recording."));
        return false;
    }
    // Phase 4: inivation devices (DAVIS/DVXplorer) have no SDK event stream
    // — record the raw device stream into an AEDAT4 (DV-native) file.
    if (controller->is_inivation_source()) {
        const auto& info = controller->sensor_info();
        aedat4_writer_ = std::make_shared<Aedat4Writer>();
        if (!aedat4_writer_->open(path.toStdString(), info.width, info.height,
                                  info.serial.toStdString(),
                                  include_imu_,
                                  include_aps_ && controller->source_capabilities().aps)) {
            aedat4_writer_.reset();
            emit error(tr("Failed to open recording file:\n%1").arg(path));
            return false;
        }
        written_events_ = 0;
        // The record dialog chooses which side streams the file includes:
        // enable the requested streams for the recording; the user's panel
        // state is restored on stop().
        imu_was_enabled_ = controller->imu_enabled();
        aps_was_enabled_ = controller->aps_enabled();
        if (include_imu_ && !imu_was_enabled_) controller->set_imu_enabled(true);
        if (include_aps_ && controller->source_capabilities().aps &&
            !aps_was_enabled_) {
            controller->set_aps_enabled(true);
        }
        auto writer = aedat4_writer_;  // in-flight batches outlive stop()
        controller->set_raw_tap([this, writer](const Metavision::EventCD* b,
                                               const Metavision::EventCD* e) {
            writer->write(b, e);
            written_events_ += static_cast<std::uint64_t>(e - b);
        });
        // The recorded IMU/APS streams follow the device streams (written
        // regardless of the GUI consumption checkboxes, like events).
        controller->set_imu_tap([writer](const davis::ImuSample& s) {
            writer->write_imu(s);
        });
        controller->set_aps_tap([writer](const davis::ApsFrame& f) {
            writer->write_aps(f);
        });
        controller_ = controller;
        path_ = path;
        recording_ = true;
        aedat4_mode_ = true;
        start_time_ = std::chrono::steady_clock::now();
        timer_.start(1000);
        emit recording_started(path);
        return true;
    }
    auto* camera = controller->camera_handle();
    if (!camera) {
        emit error(tr("No camera connected."));
        return false;
    }
    Metavision::I_EventsStream* stream = nullptr;
    try {
        stream = camera->get_device().get_facility<Metavision::I_EventsStream>();
        if (!stream) {
            emit error(tr("Recording is not supported by this device."));
            return false;
        }
        if (!stream->log_raw_data(path.toStdString())) {
            emit error(tr("Failed to open recording file:\n%1").arg(path));
            return false;
        }
    } catch (const std::exception& e) {
        emit error(QString::fromUtf8(e.what()));
        return false;
    }
    controller_ = controller;
    path_ = path;
    recording_ = true;
    start_time_ = std::chrono::steady_clock::now();
    timer_.start(1000);
    // Per the I_EventsStream contract: "The writing of each buffer of event
    // will have to be triggered by calls to get_latest_raw_data." Drive that
    // from a high-frequency timer so events actually land in the file.
    flush_timer_.start(20);
    emit recording_started(path);
    return true;
}

bool RecorderController::start_processed(CameraController* controller,
                                         const QString& path, FramePipeline* fp) {
    if (recording_ || !controller || !fp || path.isEmpty()) {
        return false;
    }
    // Same guards as raw recording: live source + active stream only. Any
    // live source works here (the processed stream comes from the pipeline,
    // not the SDK camera) — including inivation devices.
    if (controller->is_file_source()) {
        emit error(tr("Recording is only available for live cameras."));
        return false;
    }
    if (!controller->is_running()) {
        emit error(tr("Camera is not streaming. Start streaming before recording."));
        return false;
    }
    int w = 0, h = 0;
    try {
        const auto& info = controller->sensor_info();
        w = info.width;
        h = info.height;
    } catch (...) {
        emit error(tr("Sensor geometry unavailable for processed recording."));
        return false;
    }
    if (w <= 0 || h <= 0) {
        emit error(tr("Invalid sensor geometry (%1x%2) for processed recording.").arg(w).arg(h));
        return false;
    }
    try {
        writer_ = std::make_unique<Metavision::RAWEvt2EventFileWriter>(w, h, path.toStdString());
    } catch (const std::exception& e) {
        emit error(QString::fromUtf8(e.what()));
        writer_.reset();
        return false;
    }
    // RAWEvt2EventFileWriter only LOGS (does not throw) when the output file
    // cannot be opened — without this check the recording would run with the
    // writer silently discarding every event and no file ever appearing.
    if (!writer_->is_open()) {
        emit error(tr("Failed to open recording file:\n%1").arg(path));
        writer_.reset();
        return false;
    }
    fp_ = fp;
    fp_->set_processed_events_listener(
        [this](const Metavision::EventCD* b, const Metavision::EventCD* e) {
            // Runs on the SDK CD thread under the pipeline's
            // display_preproc_mutex_. The EVT2 encoder throws on
            // non-increasing timestamps (Evt3 glitch) — report once and skip
            // the batch instead of letting it escape the SDK callback.
            try {
                if (writer_) {
                    writer_->add_events(b, e);
                    written_events_ += static_cast<std::uint64_t>(e - b);
                }
            } catch (const std::exception& ex) {
                if (!writer_error_reported_) {
                    writer_error_reported_ = true;
                    emit error(tr("Processed recording dropped a batch: %1")
                                   .arg(QString::fromUtf8(ex.what())));
                }
            } catch (...) {}
        });
    controller_ = controller;
    path_ = path;
    recording_ = true;
    processed_mode_ = true;
    written_events_ = 0;
    start_time_ = std::chrono::steady_clock::now();
    timer_.start(1000);
    emit recording_started(path);
    return true;
}

void RecorderController::stop() {
    if (!recording_) {
        return;
    }
    recording_ = false;
    timer_.stop();
    flush_timer_.stop();
    if (aedat4_mode_) {
        // Unhook the tap first (no NEW batches), then close — the shared_ptr
        // in an in-flight tap lambda keeps the writer alive until that batch
        // completes, and close() itself is mutex-serialized with write().
        if (controller_) {
            controller_->set_raw_tap(nullptr);
            controller_->set_imu_tap(nullptr);
            controller_->set_aps_tap(nullptr);
            // Restore the user's side-stream panel state (the recording
            // enabled them automatically).
            if (!imu_was_enabled_) controller_->set_imu_enabled(false);
            if (controller_->source_capabilities().aps && !aps_was_enabled_) {
                controller_->set_aps_enabled(false);
            }
            imu_was_enabled_ = false;
            aps_was_enabled_ = false;
        }
        if (aedat4_writer_) {
            aedat4_writer_->close();
            aedat4_writer_.reset();
        }
        aedat4_mode_ = false;
    } else if (processed_mode_) {
        // Unhook BEFORE closing the writer: clearing the listener takes the
        // pipeline's display_preproc_mutex_, so no in-flight callback can
        // touch the writer during/after close.
        if (fp_) {
            fp_->set_processed_events_listener(nullptr);
            fp_ = nullptr;
        }
        if (writer_) {
            try { writer_->close(); } catch (...) {}
            writer_.reset();
        }
        processed_mode_ = false;
    } else if (controller_) {
        if (auto* cam = controller_->camera_handle()) {
            if (auto* stream = cam->get_device().get_facility<Metavision::I_EventsStream>()) {
                // Drain the final hardware buffer before closing the log.
                // stop_log_raw_data only flushes already-pulled data to
                // disk; events that arrived between the last flush_timer
                // tick (20 ms) and now would otherwise be lost.
                try { stream->get_latest_raw_data(); } catch (...) {}
                // stop_log_raw_data must run even if get_latest_raw_data
                // threw, otherwise the RAW file is left without a clean
                // footer and the most recent events are lost.
                try { stream->stop_log_raw_data(); } catch (...) {}
            }
        }
    }
    QString p = path_;
    path_.clear();
    controller_ = nullptr;
    emit recording_stopped(p);
}

} // namespace gui
