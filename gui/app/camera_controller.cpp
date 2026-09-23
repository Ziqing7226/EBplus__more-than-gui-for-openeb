// gui/app/camera_controller.cpp

#include "camera_controller.h"

#include <QMetaObject>
#include <QString>

#include <filesystem>

#include <cstdint>

#include <metavision/sdk/stream/camera_error_code.h>
#include <metavision/sdk/stream/camera_exception.h>
#include <metavision/sdk/stream/file_config_hints.h>

namespace gui {

namespace {
// OOM guard for file playback (audit §六-C2a). RAW Evt3 encodes events at
// ~8 bytes/event on average (CD events dominate; headers/time-high words
// amortized), so file_size / 8 is a rough event-count estimate. Buffered
// Metavision::EventCD is 16 bytes/event, so 150M events ≈ 2.4 GB resident
// in the FileFrameGenerator buffer — warn above that, but never block the
// open: the user decides whether to continue.
constexpr unsigned long long kEvt3BytesPerEventEstimate = 8;
constexpr unsigned long long kWarnEventCount = 150'000'000;
} // namespace

CameraController::CameraController(QObject* parent)
    : QObject(parent), frame_pipeline_(nullptr), statistics_(nullptr) {
    // Mirror the file-playback position for IMU-replay gating (the IMU
    // window animates with the playback instead of jumping to the end).
    connect(&frame_pipeline_, &FramePipeline::file_position_changed, this,
            [this](Metavision::timestamp pos, Metavision::timestamp) {
                file_playback_pos_.store(pos, std::memory_order_relaxed);
            });
    // Surface the FileFrameGenerator's OOM guard (audit §六-C2b) through
    // the existing warning chain (status bar in MainWindow). The signal
    // is emitted from the SDK streaming thread; Qt queues it here.
    connect(&frame_pipeline_, &FramePipeline::file_buffer_truncated,
            this, [this]() {
                emit runtime_warning(
                    tr("Event buffer memory limit reached; events beyond "
                       "this point were discarded."));
            });
}

CameraController::~CameraController() {
    teardown();
}

std::vector<std::pair<QString, QString>> CameraController::list_online_sources() {
    std::vector<std::pair<QString, QString>> out;
    try {
        const auto sources = Metavision::Camera::list_online_sources();
        for (const auto& kv : sources) {
            QString type_label;
            switch (kv.first) {
                case Metavision::OnlineSourceType::EMBEDDED: type_label = "Embedded"; break;
                case Metavision::OnlineSourceType::USB:      type_label = "USB"; break;
                case Metavision::OnlineSourceType::REMOTE:   type_label = "Remote"; break;
                default:                                     type_label = "Other"; break;
            }
            for (const auto& serial : kv.second) {
                out.emplace_back(type_label, QString::fromStdString(serial));
            }
        }
    } catch (const Metavision::CameraException&) {
        // ignore — return empty list
    }
#if GUI_HAVE_DAVIS
    for (const auto& device : davis::find_devices()) {
        out.emplace_back(QStringLiteral("DAVIS"), QString::fromStdString(device.serial));
    }
    for (const auto& device : davis::find_dvx_devices()) {
        out.emplace_back(QStringLiteral("DVXplorer"), QString::fromStdString(device.serial));
    }
#endif
    return out;
}

bool CameraController::connect_first_available() {
    teardown();
    try {
        // Use from_serial("") instead of from_first_available(). The latter
        // internally calls Camera::list_online_sources() (full local + remote
        // scan) to locate a camera — redundant with the list already shown in
        // the Devices panel. from_serial("") delegates to
        // DeviceDiscovery::open("") which opens the first available *local*
        // camera directly, skipping both the redundant scan and the slow
        // remote discovery.
        auto cam = Metavision::Camera::from_serial(std::string());
        setup_camera(std::move(cam), false);
        return true;
    } catch (const Metavision::CameraException& e) {
#if GUI_HAVE_DAVIS
        // No Metavision camera — fall back to live DAVIS devices.
        const auto devices = davis::find_devices();
        if (!devices.empty()) {
            return connect_davis(devices.front());
        }
        const auto dvx_devices = davis::find_dvx_devices();
        if (!dvx_devices.empty()) {
            return connect_dvx(dvx_devices.front());
        }
#endif
        // teardown() already destroyed the previous camera/pipeline but never
        // emits disconnected() — do so here so the UI cleans up its stale
        // connection state (status bar, panels, playback controls) before
        // the error dialog appears.
        emit disconnected();
        emit error(QString::fromUtf8(e.what()));
        return false;
    }
}

bool CameraController::connect_serial(const std::string& serial) {
    teardown();
    try {
        auto cam = Metavision::Camera::from_serial(serial);
        setup_camera(std::move(cam), false);
        return true;
    } catch (const Metavision::CameraException& e) {
#if GUI_HAVE_DAVIS
        for (const auto& device : davis::find_devices()) {
            if (device.serial == serial) {
                return connect_davis(device);
            }
        }
        for (const auto& device : davis::find_dvx_devices()) {
            if (device.serial == serial) {
                return connect_dvx(device);
            }
        }
#endif
        emit disconnected();
        emit error(QString::fromUtf8(e.what()));
        return false;
    }
}

bool CameraController::connect_file(const std::string& path) {
    // Non-SDK formats (AEDAT4 / ALPDATA) never reach Metavision::Camera —
    // the external reader streams EventCD batches into the same pipeline.
    if (is_external_file_extension(path)) {
        return connect_external_file(try_open_external_file(path));
    }
    teardown();
    // OOM guard (audit §六-C2a): estimate the event count from the file
    // size BEFORE opening (RAW Evt3 ≈ 8 bytes/event) and warn if the
    // buffer would grow huge. Non-blocking: the file still opens.
    unsigned long long estimated_events = 0;
    {
        std::error_code ec;
        const auto size = std::filesystem::file_size(path, ec);
        if (!ec) {
            estimated_events = size / kEvt3BytesPerEventEstimate;
        }
    }
    try {
        // Always use real_time_playback=false: read all events as fast as
        // possible and buffer them in the FileFrameGenerator. Playback rate
        // is controlled by the FileFrameGenerator's QTimer, not by the SDK's
        // delivery rate.
        Metavision::FileConfigHints hints;
        hints.real_time_playback(false);
        auto cam = Metavision::Camera::from_file(path, hints);
        setup_camera(std::move(cam), true);
        if (estimated_events > kWarnEventCount) {
            emit runtime_warning(
                tr("Very large file (est. %1M events): playback may use a "
                   "lot of memory.").arg(estimated_events / 1'000'000));
        }
        return true;
    } catch (const Metavision::CameraException& e) {
        emit disconnected();
        emit error(QString::fromUtf8(e.what()));
        return false;
    }
}

#if GUI_HAVE_DAVIS
bool CameraController::connect_davis(const davis::DeviceDescriptor& descriptor) {
    teardown();
    try {
        davis_device_ = std::make_unique<davis::Device>(descriptor);
        davis_biases_ = std::make_unique<davis::DavisLLBiases>(*davis_device_);
    } catch (const std::exception& e) {
        davis_device_.reset();
        davis_biases_.reset();
        emit disconnected();
        emit error(QString::fromUtf8(e.what()));
        return false;
    }

    is_file_ = false;
    sensor_info_ = SensorInfo{};
    sensor_info_.width = davis_device_->width();
    sensor_info_.height = davis_device_->height();
    sensor_info_.serial = QString::fromStdString(davis_device_->serial());
    sensor_info_.integrator = QStringLiteral("inivation");
    sensor_info_.plugin_name = QStringLiteral("DAVIS");
    sensor_info_.encoding_format = QString::fromStdString(davis_device_->model_name() + " EVS");

    statistics_.reset();
    filter_chain_.set_geometry(sensor_info_.width, sensor_info_.height);
    conditioner_.init(sensor_info_.width, sensor_info_.height);
    conditioner_.set_filter_chain(&filter_chain_);
    conditioner_.reset_temporal();
    validate_roi_after_connect();

    const std::uint16_t fps = frame_pipeline_.fps();
    const Metavision::timestamp acc = frame_pipeline_.accumulation_time_us();
    if (!frame_pipeline_.start(sensor_info_.width, sensor_info_.height, fps, acc)) {
        teardown();
        emit disconnected();
        emit error(tr("Failed to start frame pipeline."));
        return false;
    }

    // Device removal mid-stream: the callback fires on the libusb thread —
    // only hop to the GUI thread here (no Device calls: locks are held).
    davis_device_->set_gone_callback([this]() {
        QMetaObject::invokeMethod(this, [this]() { on_davis_gone(); }, Qt::QueuedConnection);
    });
    davis_device_->set_imu_sink([this](const davis::ImuSample& s) { on_imu_sample(s); });
    davis_device_->set_imu_enabled(imu_enabled_);
    davis_device_->set_aps_sink([this](const davis::ApsFrame& f) { on_aps_frame(f); });
    davis_device_->set_aps_enabled(aps_enabled_);

    emit connected(sensor_info_);
    return true;
}

void CameraController::on_davis_gone() {
    if (!davis_device_) return;
    teardown();
    emit disconnected();
    emit error(tr("DAVIS camera disconnected."));
}

bool CameraController::connect_dvx(const davis::DeviceDescriptor& descriptor) {
    teardown();
    try {
        dvx_device_ = std::make_unique<davis::DvxplorerDevice>(descriptor);
        dvx_biases_ = std::make_unique<davis::DvxLLBiases>(*dvx_device_);
    } catch (const std::exception& e) {
        dvx_device_.reset();
        dvx_biases_.reset();
        emit disconnected();
        emit error(QString::fromUtf8(e.what()));
        return false;
    }

    is_file_ = false;
    sensor_info_ = SensorInfo{};
    sensor_info_.width = dvx_device_->width();
    sensor_info_.height = dvx_device_->height();
    sensor_info_.serial = QString::fromStdString(dvx_device_->serial());
    sensor_info_.integrator = QStringLiteral("inivation");
    sensor_info_.plugin_name = QStringLiteral("DVXplorer");
    sensor_info_.encoding_format = QString::fromStdString(dvx_device_->model_name() + " EVS");

    statistics_.reset();
    filter_chain_.set_geometry(sensor_info_.width, sensor_info_.height);
    conditioner_.init(sensor_info_.width, sensor_info_.height);
    conditioner_.set_filter_chain(&filter_chain_);
    conditioner_.reset_temporal();
    validate_roi_after_connect();

    const std::uint16_t fps = frame_pipeline_.fps();
    const Metavision::timestamp acc = frame_pipeline_.accumulation_time_us();
    if (!frame_pipeline_.start(sensor_info_.width, sensor_info_.height, fps, acc)) {
        teardown();
        emit disconnected();
        emit error(tr("Failed to start frame pipeline."));
        return false;
    }

    dvx_device_->set_gone_callback([this]() {
        QMetaObject::invokeMethod(this, [this]() { on_dvx_gone(); }, Qt::QueuedConnection);
    });
    dvx_device_->set_imu_sink([this](const davis::ImuSample& s) { on_imu_sample(s); });
    dvx_device_->set_imu_enabled(imu_enabled_);

    emit connected(sensor_info_);
    return true;
}

void CameraController::on_dvx_gone() {
    if (!dvx_device_) return;
    teardown();
    emit disconnected();
    emit error(tr("DVXplorer camera disconnected."));
}
#endif

void CameraController::disconnect() {
    teardown();
    emit disconnected();
}

bool CameraController::connect_external_file(std::unique_ptr<ExternalFileSource> source) {
    teardown();
    try {
        source->open();
    } catch (const std::exception& e) {
        emit disconnected();
        emit error(QString::fromUtf8(e.what()));
        return false;
    }
    is_file_ = true;
    external_source_ = std::move(source);
    external_started_ = false;
    const ExternalFileMeta& meta = external_source_->meta();

    // Recorded IMU/APS streams surface like the live device streams: the
    // checkboxes appear (capabilities) and default ON — a recording can
    // only be replayed once, so a manually-enabled-later checkbox would
    // never see data. set_imu_enabled/set_aps_enabled stay meaningful for
    // the session state; the file stream itself cannot be switched.
#if GUI_HAVE_DAVIS
    // Side streams appear by CONTENT: the first decoded packet flips the
    // flags, checks the Devices-panel boxes and (via the MainWindow hook on
    // file_side_stream_discovered) opens the matching windows.
    external_source_->set_imu_sink([this](const davis::ImuSample& s) {
        if (!imu_discovered_.exchange(true)) {
            imu_enabled_ = true;
            emit file_side_stream_discovered(true);
        }
        on_imu_sample(s);
    });
    external_source_->set_aps_sink([this](const davis::ApsFrame& f) {
        if (!aps_discovered_.exchange(true)) {
            aps_enabled_ = true;
            emit file_side_stream_discovered(false);
        }
        on_aps_frame(f);
    });
#endif

    sensor_info_ = SensorInfo{};
    sensor_info_.width = meta.width;
    sensor_info_.height = meta.height;
    sensor_info_.serial = meta.serial;
    sensor_info_.integrator = meta.integrator;
    sensor_info_.plugin_name = meta.plugin_name;
    sensor_info_.encoding_format = meta.encoding_format;
    sensor_info_.is_file = true;

    statistics_.reset();
    filter_chain_.set_geometry(sensor_info_.width, sensor_info_.height);
    // The file source conditions per-frame in FileFrameGenerator; keep the
    // live conditioner symmetric with setup_camera (init + chain + reset).
    conditioner_.init(sensor_info_.width, sensor_info_.height);
    conditioner_.set_filter_chain(&filter_chain_);
    conditioner_.reset_temporal();
    validate_roi_after_connect();

    const std::uint16_t fps = frame_pipeline_.fps();
    const Metavision::timestamp acc = frame_pipeline_.accumulation_time_us();
    frame_pipeline_.set_file_filter_chain(&filter_chain_);
    if (!frame_pipeline_.start_file(sensor_info_.width, sensor_info_.height, fps, acc)) {
        teardown();
        emit disconnected();
        emit error(tr("Failed to start file frame pipeline."));
        return false;
    }
    if (meta.accumulation_hint_us > 0) {
        // ALPDATA frames carry all their pixels on one timestamp — make each
        // displayed frame cover exactly one recorded frame (emit
        // accumulation_time_changed keeps the UI multiplier in sync).
        frame_pipeline_.set_accumulation_time_us(meta.accumulation_hint_us);
    }
    if (meta.worst_case_events > kWarnEventCount) {
        emit runtime_warning(
            tr("Very large file (up to %1M events): playback may use a lot "
               "of memory.")
                .arg(meta.worst_case_events / 1'000'000));
    }
    emit connected(sensor_info_);
    return true;
}

void CameraController::on_external_source_done(const QString& error) {
    if (!external_source_) return; // source replaced/removed meanwhile
    external_running_.store(false, std::memory_order_relaxed);
    // The whole file is now buffered: allow the FileFrameGenerator's EOF
    // handling (stop / loop wrap) to engage — same as the SDK camera's EOF
    // status/error path.
    frame_pipeline_.set_file_loading_complete(true);
    if (!error.isEmpty()) {
        emit runtime_warning(error);
    }
    emit stopped();
}

bool CameraController::start() {
#if GUI_HAVE_DAVIS
    if (davis_device_) {
        if (davis_streaming_started_) return true;
        // Events flow through the same live path as the SDK CD callback:
        // statistics → auto bias → conditioning → listener → pipeline.
        davis_device_->set_event_sink(
            [this](const Metavision::EventCD* b, const Metavision::EventCD* e) {
                on_live_events(b, e);
            });
        try {
            davis_device_->start();
        } catch (const std::exception& e) {
            emit error(QString::fromUtf8(e.what()));
            return false;
        }
        davis_streaming_started_ = true;
        emit started();
        return true;
    }
    if (dvx_device_) {
        if (dvx_streaming_started_) return true;
        dvx_device_->set_event_sink(
            [this](const Metavision::EventCD* b, const Metavision::EventCD* e) {
                on_live_events(b, e);
            });
        try {
            dvx_device_->start();
        } catch (const std::exception& e) {
            emit error(QString::fromUtf8(e.what()));
            return false;
        }
        dvx_streaming_started_ = true;
        emit started();
        return true;
    }
#endif
    if (external_source_) {
        if (external_running_.load(std::memory_order_relaxed) || external_started_) {
            return true; // already streaming (or fully buffered)
        }
        external_started_ = true;
        external_running_.store(true, std::memory_order_relaxed);
        ExternalFileSource* src = external_source_.get();
        external_thread_ = std::thread([this, src]() {
            // Reader thread — the same role as the SDK's streaming thread:
            // raw batches into statistics + pipeline, no conditioning
            // (FileFrameGenerator conditions per rendered window).
            auto sink = [this](const Metavision::EventCD* b,
                               const Metavision::EventCD* e) {
                statistics_.add_events(b, e);
                frame_pipeline_.add_events(b, e);
            };
            try {
                src->run(sink, [this](const std::string& err) {
                    QMetaObject::invokeMethod(
                        this, [this, err]() {
                            on_external_source_done(QString::fromUtf8(err.c_str()));
                        },
                        Qt::QueuedConnection);
                });
            } catch (const std::exception& e) {
                QMetaObject::invokeMethod(
                    this, [this, msg = std::string(e.what())]() {
                        on_external_source_done(QString::fromUtf8(msg.c_str()));
                    },
                    Qt::QueuedConnection);
            } catch (...) {
                QMetaObject::invokeMethod(
                    this, [this]() {
                        on_external_source_done(tr("Unknown reader error"));
                    },
                    Qt::QueuedConnection);
            }
        });
        emit started();
        return true;
    }
    if (!camera_) {
        return false;
    }
    try {
        if (!camera_->is_running()) {
            camera_->start();
        }
        // Don't emit started() here: the status-change callback fires it
        // exactly once when the SDK confirms the STARTED transition.
        return true;
    } catch (const Metavision::CameraException& e) {
        emit error(QString::fromUtf8(e.what()));
        return false;
    }
}

bool CameraController::stop() {
#if GUI_HAVE_DAVIS
    if (davis_device_) {
        if (davis_streaming_started_) {
            davis_device_->stop();
            davis_streaming_started_ = false;
        }
        return true;
    }
    if (dvx_device_) {
        if (dvx_streaming_started_) {
            dvx_device_->stop();
            dvx_streaming_started_ = false;
        }
        return true;
    }
#endif
    if (external_source_) {
        // Cooperative cancel: the reader thread finishes soon after and its
        // completion callback emits stopped() on the GUI thread. The buffer
        // prefix stays playable, exactly like a stopped SDK file camera.
        external_source_->request_stop();
        return true;
    }
    if (!camera_) {
        return false;
    }
    try {
        if (camera_->is_running()) {
            camera_->stop();
        }
        // Don't emit stopped() here: the status-change callback fires it
        // exactly once when the SDK confirms the STOPPED transition. For
        // runtime errors (file EOF, disconnect), the error callback also
        // emits stopped() so the UI is notified even if the status callback
        // never fires.
        return true;
    } catch (const Metavision::CameraException& e) {
        emit error(QString::fromUtf8(e.what()));
        return false;
    }
}

bool CameraController::is_running() const {
#if GUI_HAVE_DAVIS
    if (davis_device_) return davis_streaming_started_;
    if (dvx_device_) return dvx_streaming_started_;
#endif
    if (external_source_) {
        return external_running_.load(std::memory_order_relaxed);
    }
    return camera_ && camera_->is_running();
}

// ---------------------------------------------------------------------------
// Phase 2 facility accessors
// ---------------------------------------------------------------------------
// All go through Device::get_facility<T>() which returns a nullable pointer
// (vs Camera::get_facility<T>() which throws on unsupported features). This
// lets the GUI degrade gracefully by disabling the corresponding panel.
facility::Biases* CameraController::biases_facility() {
#if GUI_HAVE_DAVIS
    if (davis_biases_) return davis_biases_.get();
    if (dvx_biases_) return dvx_biases_.get();
#endif
    if (!camera_) return nullptr;
    return camera_->get_device().get_facility<facility::Biases>();
}
facility::Roi* CameraController::roi_facility() {
    if (!camera_) return nullptr;
    return camera_->get_device().get_facility<facility::Roi>();
}

bool CameraController::set_unified_roi(bool enabled, int x, int y, int w, int h,
                                       std::optional<bool> roni) {
    const bool roni_mode = roni.value_or(roi_roni_);
    // Snapshot the OLD output geometry for the transition ordering below.
    const int old_out_w = (roi_enabled_ && !roi_roni_) ? roi_x1_ - roi_x0_
                                                       : sensor_info_.width;
    const int old_out_h = (roi_enabled_ && !roi_roni_) ? roi_y1_ - roi_y0_
                                                       : sensor_info_.height;
    if (is_file_) {
        // File playback: no hardware — software crop with the same
        // "sensor outputs only ROI events" semantics (Phase 2.6). RONI
        // inverts the crop (Phase 2.6 debug D-5). No ordering constraints
        // here: rendering and the bridge resize both run on the GUI thread
        // and cannot interleave.
        frame_pipeline_.set_file_roi(enabled, x, y, w, h, roni_mode);
        roi_roni_ = roni_mode;
        bool en = false;
        int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
        frame_pipeline_.file_roi(en, x0, y0, x1, y1);
        emit roi_state_changed(en, x0, y0, x1, y1);
        return true;
    }
    // Compute the window (auto-center on -1, clamp to sensor), mirroring
    // ProcessRegion::compute so live and file paths agree.
    const int sw = sensor_info_.width > 0 ? sensor_info_.width : 1280;
    const int sh = sensor_info_.height > 0 ? sensor_info_.height : 720;
    const int rw = (w <= 0) ? sw : std::min(w, sw);
    const int rh = (h <= 0) ? sh : std::min(h, sh);
    const int rx = (x < 0) ? (sw - rw) / 2 : std::min(std::max(0, x), sw - rw);
    const int ry = (y < 0) ? (sh - rh) / 2 : std::min(std::max(0, y), sh - rh);
    if (rw <= 0 || rh <= 0) return false;

#if GUI_HAVE_DAVIS
    if (davis_device_ || dvx_device_) {
        // Phase 6: DAVIS accelerates keep-inside ROI with the DVS ROI
        // filter (best-effort — a failed register write only costs USB
        // bandwidth, the conditioner's software crop still applies).
        // RONI (drop-inside) and DVXplorer are software-only.
        if (davis_device_) {
            if (enabled && !roni_mode) {
                davis_device_->set_hw_roi(rx, ry, rw, rh);
            } else {
                davis_device_->clear_hw_roi();
            }
        }
        roi_enabled_ = enabled;
        roi_roni_ = roni_mode;
        roi_x0_ = rx; roi_y0_ = ry;
        roi_x1_ = rx + rw; roi_y1_ = ry + rh;
    } else
#endif
    {
        auto* roi = roi_facility();
        if (!roi) return false;
        try {
            // Phase 2.6 debug D-5: the mode is part of the unified state
            // (was hardcoded ROI, clobbering RONI set via the RoiPanel), and
            // the window/mode are configured even when disabling so callers
            // can pre-configure a rect while the ROI is off (mirrors the
            // file path, which stores the rect unconditionally).
            roi->set_mode(roni_mode ? Metavision::I_ROI::Mode::RONI
                                    : Metavision::I_ROI::Mode::ROI);
            roi->set_windows({Metavision::I_ROI::Window(rx, ry, rw, rh)});
            roi->enable(enabled);
            roi_enabled_ = enabled;
            roi_roni_ = roni_mode;
            roi_x0_ = rx; roi_y0_ = ry;
            roi_x1_ = rx + rw; roi_y1_ = ry + rh;
        } catch (const std::exception&) {
            return false;
        }
    }
    // Conditioner follows the ROI (highest priority): ROI mode → crop+shift
    // to ROI-relative (canonical for algorithms); RONI → drop-inside,
    // absolute coordinates. The DISPLAY keeps the full-sensor frame (content
    // shifted back to its absolute position by FramePipeline; the rest stays
    // the palette background) — the overlay / Zoom-to-ROI / Replace-composite
    // strategies are built around full frames.
    //
    // TRANSITION ORDERING (review 2026-08-21, race fix): the conditioner
    // decides the coordinate regime read by the SDK thread; roi_state_changed
    // synchronously resizes backends (direct connection). In-flight batches
    // must stay in-bounds in EVERY intermediate state, so order by geometry:
    //   output SHRINKS (new dims ≤ old) → conditioner first (new small
    //     coords into not-yet-shrunk big buffers — safe, no loss);
    //   output GROWS (new dims > old)   → resize first (old small coords
    //     into already-grown buffers — safe; the conditioner keeps dropping
    //     outside-the-old-rect events for the few ms until it flips, events
    //     hw I_ROI was discarding the instant before anyway).
    // No hot-path cost: this only reorders statements on a user action.
    const int new_out_w = (roi_enabled_ && !roi_roni_) ? roi_x1_ - roi_x0_
                                                       : sensor_info_.width;
    const int new_out_h = (roi_enabled_ && !roi_roni_) ? roi_y1_ - roi_y0_
                                                       : sensor_info_.height;
    const auto apply_conditioner = [this] {
        conditioner_.set_roi(roi_enabled_, roi_x0_, roi_y0_, roi_x1_, roi_y1_,
                             roi_roni_);
        frame_pipeline_.set_display_roi_origin(roi_enabled_ && !roi_roni_,
                                               roi_x0_, roi_y0_);
    };
    const bool shrink = new_out_w <= old_out_w && new_out_h <= old_out_h;
    const bool grow = new_out_w >= old_out_w && new_out_h >= old_out_h;
    if (shrink) {
        // Conditioner first: new small coords into not-yet-shrunk buffers.
        apply_conditioner();
        emit roi_state_changed(roi_enabled_, roi_x0_, roi_y0_, roi_x1_, roi_y1_);
    } else if (grow) {
        // Resize first: old small coords into already-grown buffers. The
        // conditioner keeps dropping outside-the-old-rect events for the
        // few ms until it flips — events hw I_ROI was discarding the
        // instant before anyway.
        emit roi_state_changed(roi_enabled_, roi_x0_, roi_y0_, roi_x1_, roi_y1_);
        apply_conditioner();
    } else {
        // MIXED rect change (one axis shrinks, the other grows — reachable
        // via the ROI settings dialog): NO single order is safe; both leave
        // one axis's coordinates out of bounds in one intermediate state.
        // Route through a min-dims intermediate instead:
        //   1. conditioner -> min rect at the NEW origin: coords ≤ min dims,
        //      which fit BOTH the old and the future backend buffers;
        //   2. resize to the final rect (min coords fit the new dims);
        //   3. conditioner -> final rect.
        // Costs a transient drop of events outside the min sub-rect (ms
        // scale) — the margin hw I_ROI is already delivering at the new
        // rect. Mixed only occurs rect→rect (rect-vs-sensor is always
        // shrink-or-grow since rects are clamped inside the sensor).
        const int min_w = std::min(old_out_w, new_out_w);
        const int min_h = std::min(old_out_h, new_out_h);
        conditioner_.set_roi(true, roi_x0_, roi_y0_,
                             roi_x0_ + min_w, roi_y0_ + min_h, false);
        frame_pipeline_.set_display_roi_origin(true, roi_x0_, roi_y0_);
        emit roi_state_changed(roi_enabled_, roi_x0_, roi_y0_, roi_x1_, roi_y1_);
        apply_conditioner();
    }
    return true;
}

void CameraController::unified_roi(bool& enabled, int& x0, int& y0,
                                   int& x1, int& y1) const {
    if (is_file_) {
        frame_pipeline_.file_roi(enabled, x0, y0, x1, y1);
        return;
    }
    enabled = roi_enabled_;
    x0 = roi_x0_; y0 = roi_y0_; x1 = roi_x1_; y1 = roi_y1_;
}

void CameraController::validate_roi_after_connect() {
    if (is_file_) {
        bool en = false;
        int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
        frame_pipeline_.file_roi(en, x0, y0, x1, y1);
        if (!en) return;
        // A previous file's software crop is meaningless for the newly
        // opened file: FileFrameGenerator::set_geometry would re-apply it
        // (clamped) while every checkbox shows OFF — reset instead.
        frame_pipeline_.set_file_roi(false, 0, 0, 0, 0, false);
        emit roi_state_changed(false, 0, 0, 0, 0);
        return;
    }
    const int sw = sensor_info_.width;
    const int sh = sensor_info_.height;
    if (!roi_enabled_ || sw <= 0 || sh <= 0) return;
    if (roi_roni_) {
        // RONI drops events INSIDE the rect: a stale rect can only change
        // how much is dropped, never zero the stream. Re-sync the UI only.
        emit roi_state_changed(roi_enabled_, roi_x0_, roi_y0_, roi_x1_, roi_y1_);
        return;
    }
    if (roi_x0_ >= sw || roi_y0_ >= sh) {
        // No overlap with the new sensor: an enabled keep-inside rect
        // entirely outside it crops every batch to zero events (silent
        // total data loss). Disable outright.
        roi_enabled_ = false;
        conditioner_.set_roi(false, 0, 0, sw, sh, false);
        frame_pipeline_.set_display_roi_origin(false, 0, 0);
        emit roi_state_changed(false, 0, 0, 0, 0);
        return;
    }
    if (roi_x1_ > sw || roi_y1_ > sh) {
        // Partial overlap: clamp to the intersection.
        roi_x1_ = std::min(roi_x1_, sw);
        roi_y1_ = std::min(roi_y1_, sh);
        conditioner_.set_roi(true, roi_x0_, roi_y0_, roi_x1_, roi_y1_, false);
        frame_pipeline_.set_display_roi_origin(true, roi_x0_, roi_y0_);
    }
    emit roi_state_changed(roi_enabled_, roi_x0_, roi_y0_, roi_x1_, roi_y1_);
}
facility::AntiFlicker* CameraController::anti_flicker_facility() {
    if (!camera_) return nullptr;
    return camera_->get_device().get_facility<facility::AntiFlicker>();
}
facility::TrailFilter* CameraController::trail_filter_facility() {
    if (!camera_) return nullptr;
    return camera_->get_device().get_facility<facility::TrailFilter>();
}
facility::Erc* CameraController::erc_facility() {
    if (!camera_) return nullptr;
    return camera_->get_device().get_facility<facility::Erc>();
}
facility::TriggerIn* CameraController::trigger_in_facility() {
    if (!camera_) return nullptr;
    return camera_->get_device().get_facility<facility::TriggerIn>();
}
facility::TriggerOut* CameraController::trigger_out_facility() {
    if (!camera_) return nullptr;
    return camera_->get_device().get_facility<facility::TriggerOut>();
}

CameraController::SourceCapabilities CameraController::source_capabilities() {
    SourceCapabilities caps;
#if GUI_HAVE_DAVIS
    // inivation device layer (DAVIS/DVXplorer): events + biases + the IMU
    // stream; the trigger/ESP facilities are not wired yet, so those
    // panels hide.
    if (davis_device_ || dvx_device_) {
        caps.imu = true;
        caps.aps = davis_device_ != nullptr;  // APS frames: DAVIS only.
        return caps;
    }
#endif
    if (external_source_) {
        // AEDAT4 replay: the IMU/APS streams recorded in the file surface
        // exactly like the live device streams.
#if GUI_HAVE_DAVIS
        caps.imu = external_source_->has_imu();
        caps.aps = external_source_->has_aps();
#endif
        return caps;
    }
    if (!camera_) return caps;  // nothing connected.
    caps.trigger = trigger_in_facility() != nullptr ||
                   trigger_out_facility() != nullptr;
    caps.esp = anti_flicker_facility() != nullptr ||
               trail_filter_facility() != nullptr || erc_facility() != nullptr;
    return caps;
}

// The IMU/APS stream API is source-agnostic from the callers' point of
// view (the windows and the devices panel compile in every configuration);
// without libusb the members below do not exist and the API degrades to
// "not available".
bool CameraController::set_imu_enabled(bool on) {
#if GUI_HAVE_DAVIS
    if (external_source_) {
        // Replay: the stream is fixed inside the file — the flag only gates
        // the consumption state (the reader-thread sink checks it).
        imu_enabled_ = on;
        return true;
    }
    if (!davis_device_ && !dvx_device_) return false;
    imu_enabled_ = on;
    if (davis_device_) davis_device_->set_imu_enabled(on);
    if (dvx_device_) dvx_device_->set_imu_enabled(on);
    if (on) {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        // Fresh session: reset the sequence counter AND drop the retained
        // ring. Leaving the old samples behind reuses sequence numbers, so
        // drain_imu() handed a new consumer the whole stale backlog as if it
        // were current data (the reopened IMU window integrated it at once —
        // the pose whipped around).
        imu_count_ = 0;
        imu_ring_.clear();
        imu_latest_ = davis::ImuSample{};
    }
    return true;
#else
    (void)on;
    return false;
#endif
}

bool CameraController::imu_enabled() const {
#if GUI_HAVE_DAVIS
    return imu_enabled_;
#else
    return false;
#endif
}

davis::ImuSample CameraController::latest_imu() const {
#if GUI_HAVE_DAVIS
    std::lock_guard<std::mutex> lock(imu_mutex_);
    return imu_latest_;
#else
    return {};
#endif
}

long CameraController::imu_sample_count() const {
#if GUI_HAVE_DAVIS
    std::lock_guard<std::mutex> lock(imu_mutex_);
    return imu_count_;
#else
    return 0;
#endif
}

void CameraController::on_imu_sample(const davis::ImuSample& sample) {
    // Recording tap FIRST (like the event raw_tap_): the AEDAT4 file gets
    // the device stream regardless of the GUI consumption state. Snapshot
    // under tap_mutex_ so a concurrent stop-recording assignment can never
    // destroy the functor mid-call.
    std::function<void(const davis::ImuSample&)> imu_tap;
    {
        std::lock_guard<std::mutex> lock(tap_mutex_);
        imu_tap = imu_tap_;
    }
    if (imu_tap) imu_tap(sample);
#if GUI_HAVE_DAVIS
    std::lock_guard<std::mutex> lock(imu_mutex_);
    imu_latest_ = sample;
    const auto seq = ++imu_count_;
    imu_ring_.emplace_back(seq, sample);
    while (imu_ring_.size() > kImuRingMax) imu_ring_.pop_front();
#else
    (void)sample;
#endif
}

std::vector<davis::ImuSample> CameraController::drain_imu(std::int64_t& cursor) {
#if GUI_HAVE_DAVIS
    std::lock_guard<std::mutex> lock(imu_mutex_);
    if (cursor == std::numeric_limits<std::int64_t>::min()) {
        if (!is_file_source()) {
            // Fresh LIVE consumer: skip the retained backlog, start at the
            // newest (old samples are stale history for a live stream).
            cursor = imu_count_;
            return {};
        }
        // File replay: the samples decoded ONCE when the file opened —
        // they will never "arrive" again, so a consumer attached later
        // (the IMU window opened mid-replay) must still receive the
        // whole retained recording.
        cursor = imu_count_ - static_cast<std::int64_t>(imu_ring_.size());
    }
    if (cursor >= imu_count_) {
        cursor = imu_count_;
        return {};
    }
    // The caller may be behind the ring (wrapped or fresh consumer) — give
    // whatever is retained from the oldest surviving sample on.
    const auto oldest = imu_count_ - static_cast<std::int64_t>(imu_ring_.size());
    if (cursor < oldest) cursor = oldest;
    // File replay: serve samples up to the playback position so the IMU
    // window animates in sync with the event playback (negative = no
    // position known, serve everything).
    const auto gate = is_file_source() ? file_playback_pos_.load(std::memory_order_relaxed) : -1;
    std::vector<davis::ImuSample> out;
    out.reserve(imu_ring_.size());
    std::int64_t delivered = cursor;
    for (const auto& [seq, sample] : imu_ring_) {
        if (seq <= delivered) continue;
        if (gate >= 0 && sample.t > gate) continue;  // not played yet
        out.push_back(sample);
        delivered = seq;
    }
    // The cursor only advances over DELIVERED samples: gated ones stay
    // available for the next drain as the playback position advances.
    if (!imu_ring_.empty()) cursor = std::max(delivered, cursor);
    return out;
#else
    (void)cursor;
    return {};
#endif
}

bool CameraController::set_aps_enabled(bool on) {
#if GUI_HAVE_DAVIS
    if (external_source_) {
        // Replay: same as the IMU stream — flag-only gating.
        aps_enabled_ = on;
        return true;
    }
    if (!davis_device_) return false;  // APS frames are DAVIS-only.
    aps_enabled_ = on;
    davis_device_->set_aps_enabled(on);
    if (on) {
        std::lock_guard<std::mutex> lock(aps_mutex_);
        aps_count_ = 0;  // fresh session for the rate display
    }
    return true;
#else
    (void)on;
    return false;
#endif
}

bool CameraController::aps_enabled() const {
#if GUI_HAVE_DAVIS
    return aps_enabled_;
#else
    return false;
#endif
}

davis::ApsFrame CameraController::latest_aps_frame() {
#if GUI_HAVE_DAVIS
    // File replay: serve the frame matching the playback position (the
    // recorded frames decode in one burst, so a plain "latest" would pin
    // the window to the recording's final frame).
    if (external_source_) {
        // Before the first frame's position: nothing to show (the window
        // keeps its previous content until a valid frame arrives).
        davis::ApsFrame f;
        if (external_source_->read_aps_frame_for_position(
                file_playback_position_us(), f)) {
            return f;
        }
        return {};
    }
    std::lock_guard<std::mutex> lock(aps_mutex_);
    return aps_latest_;
#else
    (void)this;
    return {};
#endif
}

long CameraController::aps_frame_count() const {
#if GUI_HAVE_DAVIS
    std::lock_guard<std::mutex> lock(aps_mutex_);
    return aps_count_;
#else
    return 0;
#endif
}

void CameraController::on_aps_frame(const davis::ApsFrame& frame) {
    std::function<void(const davis::ApsFrame&)> aps_tap;
    {
        std::lock_guard<std::mutex> lock(tap_mutex_);
        aps_tap = aps_tap_;
    }
    if (aps_tap) aps_tap(frame);
#if GUI_HAVE_DAVIS
    std::lock_guard<std::mutex> lock(aps_mutex_);
    aps_latest_ = frame;
    ++aps_count_;
#else
    (void)frame;
#endif
}
facility::CameraSync* CameraController::camera_sync_facility() {
    if (!camera_) return nullptr;
    return camera_->get_device().get_facility<facility::CameraSync>();
}

void CameraController::set_raw_tap(RawTap tap) {
    std::lock_guard<std::mutex> lock(tap_mutex_);
    raw_tap_ = std::move(tap);
#if GUI_HAVE_DAVIS
    // The recorder must see the batch on the USB decode thread, BEFORE the
    // bounded worker queue (drop-oldest under a flood) — recording data
    // never goes through the queue.
    if (davis_device_) {
        davis_device_->set_raw_consumer(
            [this](const Metavision::EventCD* b, const Metavision::EventCD* e) {
                RawTap tap;
                {
                    std::lock_guard<std::mutex> lock(tap_mutex_);
                    tap = raw_tap_;
                }
                if (tap) tap(b, e);
            });
    }
    if (dvx_device_) {
        dvx_device_->set_raw_consumer(
            [this](const Metavision::EventCD* b, const Metavision::EventCD* e) {
                RawTap tap;
                {
                    std::lock_guard<std::mutex> lock(tap_mutex_);
                    tap = raw_tap_;
                }
                if (tap) tap(b, e);
            });
    }
#endif
}

void CameraController::set_conditioned_listener(ConditionedListener cb) {
    std::lock_guard<std::mutex> lk(conditioned_mutex_);
    conditioned_listener_ = std::move(cb);
}

bool CameraController::set_conditioner_param(const std::string& key,
                                             const std::string& value) {
    // Live conditioner + the file generator's conditioner (only one is ever
    // active; the inactive one just stores the value).
    const bool ok = conditioner_.set_param(key, value);
    frame_pipeline_.set_file_conditioner_param(key, value);
    return ok;
}

void CameraController::reset_conditioner() {
    conditioner_.reset_temporal();
    frame_pipeline_.reset_file_conditioner();
}

bool CameraController::conditioner_active() const {
    return conditioner_.active();
}

void CameraController::set_cd_broadcast(bool enabled) {
    cd_broadcast_.store(enabled, std::memory_order_relaxed);
}

// --- Auto bias (§4.4.6) -----------------------------------------------------

bool CameraController::set_auto_bias_enabled(bool on) {
    if (on == auto_bias_enabled()) return on;
    if (on) {
#if GUI_HAVE_DAVIS
        if (is_file_ || (!camera_ && !davis_device_ && !dvx_device_)) return false;
#else
        if (is_file_ || !camera_) return false;
#endif
#if GUI_HAVE_DAVIS
        if (dvx_device_) {
            // Phase 5 — DVXplorer: no diff biases; the ON/OFF contrast
            // thresholds (0-17) are the two control axes. Bound by exact
            // name, homing toward the reference defaults (9/9). Delta
            // signs stay at the default +1: the controller's convention is
            // "positive delta = fewer events of that polarity", and a
            // higher contrast register means exactly that (hardware-
            // measured: contrast 0 floods at ~19 Mev/s, 12 is quiet).
            if (!bias_applier_.attach_axes(biases_facility(),
                                           "contrast_on", "contrast_off")) {
                return false;
            }
            bias_applier_.set_home_targets(9, 9);
        } else
#endif
        if (!bias_applier_.attach(biases_facility())) {
            return false;
        }
#if GUI_HAVE_DAVIS
        if (davis_device_) {
            // DAVIS diff biases are absolute operating points with non-zero
            // reference defaults — homing drifts toward those defaults, not
            // toward 0 (which is the Prophesee convention).
            int t_on = 0, t_off = 0;
            if (davis::davis_reference_default_for(davis_device_->chip_model(),
                                                   "diff_on", t_on) &&
                davis::davis_reference_default_for(davis_device_->chip_model(),
                                                   "diff_off", t_off)) {
                bias_applier_.set_home_targets(t_on, t_off);
            }
            // DAVIS OFF-axis polarity is inverted vs Prophesee (measured on
            // hardware: higher diff_off → MORE OFF events), so the OFF-axis
            // delta sign flips; the ON axis matches Prophesee.
            bias_applier_.set_off_delta_sign(-1);
        }
#endif
        {
            std::lock_guard<std::mutex> lk(auto_bias_mutex_);
            auto_bias_ctrl_.reset();
            // Phase 5: the default step (32) is tuned for the 0-2047 diff
            // range — on the DVXplorer 0-17 contrast range it would slam
            // into a range limit every correction (bang-bang control), so
            // cap the per-tick delta there.
#if GUI_HAVE_DAVIS
            const int max_step =
                dvx_device_ ? 2 : gui_algo::AutoBiasController::kDefaultMaxStep;
#else
            const int max_step = gui_algo::AutoBiasController::kDefaultMaxStep;
#endif
            auto_bias_ctrl_.set_max_step(max_step);
            auto_bias_last_tick_ = -1;
        }
        auto_bias_enabled_.store(true, std::memory_order_relaxed);
    } else {
        auto_bias_enabled_.store(false, std::memory_order_relaxed);
        if (bias_applier_.attached()) {
            bias_applier_.restore();
            bias_applier_.detach();
            emit auto_bias_applied();
        }
        std::lock_guard<std::mutex> lk(auto_bias_mutex_);
        auto_bias_ctrl_.reset();
    }
    return true;
}

bool CameraController::set_auto_bias_rate_bounds(float lo_mev, float hi_mev) {
    std::lock_guard<std::mutex> lk(auto_bias_mutex_);
    return auto_bias_ctrl_.set_rate_bounds(lo_mev, hi_mev);
}

void CameraController::auto_bias_rate_bounds(float& lo_mev, float& hi_mev) const {
    std::lock_guard<std::mutex> lk(auto_bias_mutex_);
    lo_mev = auto_bias_ctrl_.rate_min_mev();
    hi_mev = auto_bias_ctrl_.rate_max_mev();
}

void CameraController::auto_bias_tick(const Metavision::EventCD* b,
                                      const Metavision::EventCD* e) {
    if (b == e) return;
    std::uint32_t n_on = 0, n_off = 0;
    for (auto* it = b; it != e; ++it) {
        if (it->p != 0) ++n_on; else ++n_off;
    }
    const std::int64_t t = (e - 1)->t;
    gui_algo::BiasCommand cmd;
    {
        std::lock_guard<std::mutex> lk(auto_bias_mutex_);
        auto_bias_ctrl_.accumulate(t, n_on, n_off);
        if (auto_bias_last_tick_ < 0) auto_bias_last_tick_ = t;
        if (t - auto_bias_last_tick_ >= gui_algo::AutoBiasController::kTickUs) {
            cmd = auto_bias_ctrl_.update(t);
            auto_bias_last_tick_ = t;
        }
    }
    if (!cmd.active) return;
    // Apply on the GUI thread: the register writes are rare (each is
    // followed by a hold + measurement refill) and the Biases panel can
    // safely re-read the hardware afterwards.
    const bool home = cmd.home;
    QMetaObject::invokeMethod(this, [this, home, cmd]() {
        // The controller may have been disabled since the tick — the
        // snapshot restore already ran, don't fight it.
        if (!auto_bias_enabled_.load(std::memory_order_relaxed)) return;
        // Home commands move both biases half the remaining distance to 0
        // (factory default), clipped to [1, kDefaultMaxStep]; at 0 they
        // no-op and emit nothing.
        const bool changed =
            home ? bias_applier_.home(gui_algo::AutoBiasController::kDefaultMaxStep)
                 : bias_applier_.apply(cmd.delta_on, cmd.delta_off) !=
                       BiasApplier::Status::NoBias;
        if (changed) emit auto_bias_applied();
    }, Qt::QueuedConnection);
}

// ---------------------------------------------------------------------------
// Internals
// ---------------------------------------------------------------------------

void CameraController::on_live_events(const Metavision::EventCD* b, const Metavision::EventCD* e) {
    try {
        // (The recorder's raw tap runs on the USB decode thread — see
        // set_raw_tap — so this pipeline only serves the live consumers.)
        statistics_.add_events(b, e);
        if (is_file_) {
            // File mode: buffer RAW events — conditioning happens per-frame
            // in FileFrameGenerator::render_frame() so toggles take effect
            // immediately during playback.
            frame_pipeline_.add_events(b, e);
        } else {
            // Auto bias measures the RAW sensor output (biases act before any
            // software conditioning).
            if (auto_bias_enabled_.load(std::memory_order_relaxed)) {
                auto_bias_tick(b, e);
            }
            // Live mode: condition ONCE (unified ROI → polarity stages →
            // noise filter → thin → undistort → flips). Display, processed
            // recording and the algorithm listener all consume the SAME
            // output span.
            const auto [cb, cn] = conditioner_.apply(b, e);
            const auto* ce = cb + cn;
            // cn == 0 (a stage emptied the batch): skip the display push — an
            // empty span may carry a null data() on first use. The listener
            // still runs: it owns the raw-count profiler tick for empty
            // batches.
            ConditionedListener listener;
            {
                std::lock_guard<std::mutex> lk(conditioned_mutex_);
                listener = conditioned_listener_;
            }
            // P6-A: the listener runs BEFORE the display push and may
            // substitute the output span (stream-filter algorithm output —
            // hot_pixel_filter / EIS stabilization).
            const Metavision::EventCD* ob = cb;
            const Metavision::EventCD* oe = ce;
            if (listener) listener(b, e, cb, ce, ob, oe);
            if (ob != nullptr && oe != nullptr && ob < oe) {
                frame_pipeline_.add_events(ob, oe);
            }
        }
        // Optional CD broadcast for calibration tools — always the RAW span.
        // The buffer comes from the reusable pool (no allocation in steady
        // state) and only flows when a consumer opted in via
        // set_cd_broadcast(true); the emit crosses to the GUI thread through
        // Qt's queued-connection machinery.
        if (cd_broadcast_.load(std::memory_order_relaxed) && b != e) {
            auto batch = broadcast_pool_.acquire();
            batch->assign(b, e);
            emit cd_events_ready(batch);
        }
    } catch (const std::exception& ex) {
        QMetaObject::invokeMethod(this, [this, msg = std::string(ex.what())]() {
            emit runtime_warning(QString::fromUtf8(msg.c_str()));
        }, Qt::QueuedConnection);
    } catch (...) {
        // Swallow to keep the stream alive; the source thread must not
        // propagate exceptions out of the callback.
    }
}

void CameraController::setup_camera(Metavision::Camera&& cam, bool is_file) {
    is_file_ = is_file;
    camera_ = std::make_unique<Metavision::Camera>(std::move(cam));
    fetch_sensor_info();

    // Runtime error callback: file EOF, disconnects, firmware errors arrive here.
    // Capture the camera pointer: this callback's queued lambdas may execute
    // AFTER the user has connected a different source — without the identity
    // check, a stale error from source A would stop the freshly-connected
    // source B and emit a spurious stopped() (audit §六-C1).
    err_cb_id_ = camera_->add_runtime_error_callback(
        [this, cam = camera_.get()](const Metavision::CameraException& e) {
            // Reaching end-of-file is a normal stop condition for playback.
            const QString msg = QString::fromUtf8(e.what());

            // Evt3 "NonMonotonicTimeHigh" is a transient HAL-layer warning
            // that occurs ~50% of the time when starting Gen3.x cameras.
            // The timestamp high bits momentarily go backwards, but the
            // camera keeps streaming and the frame pipeline handles the
            // timestamp gap gracefully. Stopping the camera on this error
            // (the previous behavior) made the camera fail half the time.
            // Treat it as a non-fatal warning and keep the stream running.
            const bool is_evt3_time_glitch =
                msg.contains(QStringLiteral("NonMonotonicTimeHigh"), Qt::CaseInsensitive) ||
                msg.contains(QStringLiteral("Evt3 protocol violation"), Qt::CaseInsensitive);

            if (is_evt3_time_glitch) {
                // Transient Evt3 timestamp glitch — ignore for BOTH live and
                // file sources.  Gen3 raw files frequently contain
                // NonMonotonicTimeHigh warnings; treating them as EOF (the
                // previous behaviour for is_file_) stopped playback before
                // any frames were visible.
                emit runtime_warning(tr("Transient timestamp glitch (ignored): %1").arg(msg));
            } else if (is_file_) {
                // File source + non-glitch error → genuine EOF.
                emit runtime_warning(tr("Playback ended: %1").arg(msg));
                QMetaObject::invokeMethod(this, [this, cam]() {
                    if (camera_.get() != cam) return;  // stale callback, see above
                    // The whole file is now buffered: allow the
                    // FileFrameGenerator's EOF handling (stop / loop wrap)
                    // to engage (audit §六-P2).
                    frame_pipeline_.set_file_loading_complete(true);
                    if (camera_->is_running()) {
                        try { camera_->stop(); } catch (...) {}
                    }
                    emit stopped();
                }, Qt::QueuedConnection);
            } else {
                // Live camera + genuine error: stop and report.
                emit error(msg);
                QMetaObject::invokeMethod(this, [this, cam]() {
                    if (camera_.get() != cam) return;  // stale callback, see above
                    if (camera_->is_running()) {
                        try { camera_->stop(); } catch (...) {}
                    }
                    emit stopped();
                }, Qt::QueuedConnection);
            }
        });

    // Status change callback.
    status_cb_id_ = camera_->add_status_change_callback(
        [this](const Metavision::CameraStatus& status) {
            if (status == Metavision::CameraStatus::STARTED) {
                emit started();
            } else {
                // A file source stopping on its own means EOF: everything
                // the file will ever yield is now buffered. Signal
                // loading-complete so the FileFrameGenerator's EOF handling
                // (stop / loop wrap) can engage. The runtime-error EOF path
                // (above) also does this, but the SDK does not guarantee an
                // error callback at EOF — relying on it alone left
                // loading_complete_ unset and loop playback stalled at the
                // buffer top forever. Idempotent; on user-initiated stops
                // the pipeline is being torn down anyway.
                if (is_file_) {
                    frame_pipeline_.set_file_loading_complete(true);
                }
                emit stopped();
            }
        });

    // CD callback: forward events to the frame pipeline + statistics.
    // The SDK dispatches this callback on its streaming/decoding thread with
    // NO try/catch, so an exception escaping the lambda would call
    // std::terminate and crash the whole GUI with no diagnostic. The
    // conditioner may reallocate its buffers, so std::bad_alloc at high event
    // rates is plausible. Wrap the body and surface failures to the GUI
    // thread.
    cd_cb_id_ = camera_->cd().add_callback(
        [this](const Metavision::EventCD* b, const Metavision::EventCD* e) {
            on_live_events(b, e);
        });

    statistics_.reset();
    filter_chain_.set_geometry(sensor_info_.width, sensor_info_.height);
    // Conditioner: live batches only (the file source conditions per-frame
    // in FileFrameGenerator). reset() clears temporal state so a reconnect's
    // timestamps never see stale surfaces.
    conditioner_.init(sensor_info_.width, sensor_info_.height);
    conditioner_.set_filter_chain(&filter_chain_);
    conditioner_.reset_temporal();
    validate_roi_after_connect();

    // Start the frame pipeline for the new sensor geometry. File sources use
    // FileFrameGenerator (buffers events, controls playback rate via QTimer);
    // live sources use CDFrameGenerator (shows latest accumulation window).
    // fps_ / accumulation_us_ / fps_limit_ persist across stop/start cycles
    // so user settings survive camera reconnects and file reopens.
    const long w = sensor_info_.width;
    const long h = sensor_info_.height;
    const std::uint16_t fps = frame_pipeline_.fps();
    const Metavision::timestamp acc = frame_pipeline_.accumulation_time_us();
    if (is_file) {
        frame_pipeline_.set_file_filter_chain(&filter_chain_);
        if (!frame_pipeline_.start_file(w, h, fps, acc)) {
            // Without a running pipeline the display stays black forever —
            // abort the connection instead of reporting "Connected"
            // (audit §六-C4).
            teardown();
            emit disconnected();
            emit error(tr("Failed to start file frame pipeline."));
            return;
        }
    } else {
        if (!frame_pipeline_.start(w, h, fps, acc)) {
            teardown();
            emit disconnected();
            emit error(tr("Failed to start frame pipeline."));
            return;
        }
    }

    emit connected(sensor_info_);
}

void CameraController::teardown() {
#if GUI_HAVE_DAVIS
    // 0a. Stop the DAVIS stream and release the device first: its libusb
    //     thread feeds statistics_/frame_pipeline_ exactly like the SDK
    //     streaming thread.
    if (davis_device_) {
        try {
            davis_device_->stop();
        } catch (...) {
        }
    }
    davis_device_.reset();
    davis_biases_.reset();
    davis_streaming_started_ = false;
    if (dvx_device_) {
        try {
            dvx_device_->stop();
        } catch (...) {
        }
    }
    dvx_device_.reset();
    dvx_biases_.reset();
    dvx_streaming_started_ = false;
    // Phase 2: the IMU stream is session-scoped — a source switch or
    // disconnect clears the flag and the telemetry window.
    imu_enabled_ = false;
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        imu_latest_ = davis::ImuSample{};
        imu_count_ = 0;
    }
    aps_enabled_ = false;
    {
        std::lock_guard<std::mutex> lock(aps_mutex_);
        aps_latest_ = davis::ApsFrame{};
        aps_count_ = 0;
    }
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        imu_ring_.clear();
    }
#endif
    raw_tap_ = nullptr;
    imu_tap_ = nullptr;
    aps_tap_ = nullptr;
    file_playback_pos_.store(-1, std::memory_order_relaxed);
    imu_discovered_.store(false);
    aps_discovered_.store(false);
    // 0. Stop the external reader FIRST: it feeds statistics_ and
    //    frame_pipeline_ from its own thread, so it must be joined before
    //    the pipeline is stopped below.
    if (external_source_) {
        external_source_->request_stop();
    }
    if (external_thread_.joinable()) {
        external_thread_.join();
    }
    external_thread_ = {};
    external_source_.reset();
    external_started_ = false;
    external_running_.store(false, std::memory_order_relaxed);

    // 1. Remove the SDK callbacks FIRST so the SDK thread stops calling into
    //    FramePipeline / FilterChain / StatisticsController. Without this,
    //    stopping the pipeline (which resets generator_) races with the CD
    //    callback's frame_pipeline_.add_events() — a use-after-free.
    // Also disable CD broadcast so no in-flight emit references the camera.
    cd_broadcast_.store(false, std::memory_order_relaxed);
    // Auto bias: stop the control loop before the device goes away. The
    // register writes would fail — the sensor keeps its current biases.
    auto_bias_enabled_.store(false, std::memory_order_relaxed);
    bias_applier_.detach();
    {
        std::lock_guard<std::mutex> lk(auto_bias_mutex_);
        auto_bias_ctrl_.reset();
    }
    if (camera_) {
        if (cd_cb_id_) {
            camera_->cd().remove_callback(*cd_cb_id_);
            cd_cb_id_.reset();
        }
        if (err_cb_id_) {
            camera_->remove_runtime_error_callback(*err_cb_id_);
            err_cb_id_.reset();
        }
        if (status_cb_id_) {
            camera_->remove_status_change_callback(*status_cb_id_);
            status_cb_id_.reset();
        }
        if (camera_->is_running()) {
            try { camera_->stop(); } catch (...) {}
        }
        camera_.reset();
    }

    // 2. Now that no SDK thread can touch it, stop the frame pipeline.
    frame_pipeline_.stop();

    sensor_info_ = SensorInfo{};
    is_file_ = false;
}

void CameraController::fetch_sensor_info() {
    SensorInfo info;
    info.is_file = is_file_;
    if (!camera_) {
        sensor_info_ = info;
        return;
    }
    try {
        const auto& g = camera_->geometry();
        info.width = g.get_width();
        info.height = g.get_height();
    } catch (...) {}
    try {
        const auto& cfg = camera_->get_camera_configuration();
        info.serial = QString::fromStdString(cfg.serial_number);
        info.integrator = QString::fromStdString(cfg.integrator);
        info.plugin_name = QString::fromStdString(cfg.plugin_name);
        info.encoding_format = QString::fromStdString(cfg.data_encoding_format);
        info.firmware_version = QString::fromStdString(cfg.firmware_version);
    } catch (...) {}
    try {
        const auto& gen = camera_->generation();
        info.generation_name = QString::fromStdString(gen.name());
        info.generation_major = gen.version_major();
        info.generation_minor = gen.version_minor();
    } catch (...) {}
    sensor_info_ = info;
}

} // namespace gui
