// gui/exporter/exporter_controller.cpp

#include "exporter_controller.h"

#include <QFile>
#include <QMetaObject>

#include <algorithm>
#include <mutex>

#include <metavision/sdk/base/events/event_cd.h>
#include <metavision/sdk/core/algorithms/periodic_frame_generation_algorithm.h>
#include <metavision/sdk/core/utils/colors.h>
#include <metavision/sdk/core/utils/cv_video_recorder.h>
#include <metavision/sdk/stream/camera.h>
#include <metavision/sdk/stream/camera_exception.h>
#include <metavision/sdk/stream/file_config_hints.h>
#include <metavision/sdk/stream/hdf5_event_file_writer.h>

#include <opencv2/imgproc.hpp>

#include "app/duration_query.h"
#include "app/external_file_source.h"

#include <mutex>
#include <utility>

namespace gui {

ExporterController::ExporterController(QObject* parent) : QObject(parent) {}

ExporterController::~ExporterController() {
    cancel();
    if (worker_.joinable()) worker_.join();
}

bool ExporterController::start(const ExportParams& params) {
    if (running_) return false;
    if (worker_.joinable()) worker_.join();  // reap previous finished worker
    cancel_ = false;
    running_ = true;
    worker_ = std::thread([this, params]() {
        try {
            if (params.format == ExportParams::Format::HDF5) {
                run_hdf5(params);
            } else {
                run_avi(params);
            }
        } catch (const std::exception& e) {
            QMetaObject::invokeMethod(this, [this, msg = QString::fromUtf8(e.what())]() {
                emit failed(msg);
            }, Qt::QueuedConnection);
        } catch (...) {
            QMetaObject::invokeMethod(this, [this]() {
                emit failed(tr("Export failed with an unknown error."));
            }, Qt::QueuedConnection);
        }
        running_ = false;
    });
    return true;
}

void ExporterController::cancel() {
    cancel_ = true;
}

void ExporterController::run_hdf5(const ExportParams& p) {
    if (is_external_file_extension(p.source_path.toStdString())) {
        run_hdf5_external(p);
        return;
    }
    std::shared_ptr<Metavision::Camera> cam;
    try {
        Metavision::FileConfigHints hints;
        hints.real_time_playback(false); // as fast as possible
        cam = std::make_shared<Metavision::Camera>(
            Metavision::Camera::from_file(p.source_path.toStdString(), hints));
    } catch (const Metavision::CameraException& e) {
        QMetaObject::invokeMethod(this, [this, msg = QString::fromUtf8(e.what())]() {
            emit failed(msg);
        }, Qt::QueuedConnection);
        return;
    }

    // Total duration for progress reporting: use the caller-provided value
    // when known (exporting the currently-open file), otherwise query the
    // OSC duration ASYNCHRONOUSLY — get_duration() blocks while building
    // the raw index, which would freeze progress reporting (see
    // query_duration_async).
    auto dur_us = std::make_shared<std::atomic<Metavision::timestamp>>(p.duration_us);

    // callback_error captures the first exception message from the writer
    // callback (e.g. missing HDF5 ECF compression plugin). It is written
    // before cancel_ is set (release), so the polling loop observes it after
    // reading cancel_ (acquire). Without this, a genuine failure would be
    // silently reported as "Export cancelled" — hiding the real cause.
    std::string callback_error;
    Metavision::HDF5EventFileWriter writer(p.output_path.toStdString());
    std::atomic<Metavision::timestamp> last_ts{0};
    auto id = cam->cd().add_callback(
        [&writer, &last_ts, &callback_error, this](const Metavision::EventCD* b, const Metavision::EventCD* e) {
            try {
                if (cancel_) return;
                writer.add_events(b, e);
                if (b != e) last_ts.store((e - 1)->t, std::memory_order_relaxed);
            } catch (const std::exception& ex) {
                callback_error = ex.what();
                cancel_.store(true, std::memory_order_release);
            } catch (...) {
                callback_error = "Unknown error in HDF5 writer";
                cancel_.store(true, std::memory_order_release);
            }
        });
    cam->start();
    if (dur_us->load(std::memory_order_relaxed) == 0) {
        query_duration_async(cam, dur_us);
    }
    // Spin until EOF or cancel. Emit incremental progress so the progress bar
    // does not sit at 0% for the entire export (only jumping to 100% at the end).
    while (!cancel_) {
        try {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            if (!cam->is_running()) break;
            const Metavision::timestamp dur = dur_us->load(std::memory_order_relaxed);
            if (dur > 0) {
                const double r = std::min(1.0, static_cast<double>(
                    last_ts.load(std::memory_order_relaxed)) / dur);
                QMetaObject::invokeMethod(this, [this, r]() { emit progress(r); },
                                          Qt::QueuedConnection);
            }
        } catch (...) {
            break;
        }
    }
    try { cam->stop(); } catch (...) {}
    cam->cd().remove_callback(id);
    // writer.close() may throw if the ECF compression plugin is missing or
    // the file system is full. Capture the error so the user sees the cause
    // rather than a generic abort.
    try {
        writer.close();
    } catch (const std::exception& ex) {
        callback_error = ex.what();
        cancel_.store(true, std::memory_order_release);
    } catch (...) {
        callback_error = "Unknown error closing HDF5 file";
        cancel_.store(true, std::memory_order_release);
    }

    // Distinguish cancel from completion: a cancelled export must not emit
    // completed (the output file is partial/truncated). Delete the partial
    // file so the user can't mistake it for a valid recording (audit §六-E4).
    if (cancel_.load(std::memory_order_acquire)) {
        QFile::remove(p.output_path);
        QMetaObject::invokeMethod(this, [this, msg = callback_error]() {
            emit failed(msg.empty() ? tr("Export cancelled.")
                                    : QString::fromUtf8(msg.c_str()));
        }, Qt::QueuedConnection);
        return;
    }
    QMetaObject::invokeMethod(this, [this, out = p.output_path]() {
        emit progress(1.0);
        emit completed(out);
    }, Qt::QueuedConnection);
}

void ExporterController::run_avi(const ExportParams& p) {
    if (is_external_file_extension(p.source_path.toStdString())) {
        run_avi_external(p);
        return;
    }
    std::shared_ptr<Metavision::Camera> cam;
    try {
        Metavision::FileConfigHints hints;
        hints.real_time_playback(false);
        cam = std::make_shared<Metavision::Camera>(
            Metavision::Camera::from_file(p.source_path.toStdString(), hints));
    } catch (const Metavision::CameraException& e) {
        QMetaObject::invokeMethod(this, [this, msg = QString::fromUtf8(e.what())]() {
            emit failed(msg);
        }, Qt::QueuedConnection);
        return;
    }

    // Total duration for progress reporting: caller-provided when known,
    // otherwise queried asynchronously (see query_duration_async — a
    // synchronous get_duration() would freeze progress reporting).
    auto dur_us = std::make_shared<std::atomic<Metavision::timestamp>>(p.duration_us);

    // Geometry may be unavailable for certain file formats (e.g. DAT without
    // embedded geometry metadata). Querying it outside a try/catch would
    // crash via std::terminate on an uncaught exception. Likewise, w/h == 0
    // would make cv::VideoWriter segfault on cv::Size(0,0).
    int w = 0, h = 0;
    try {
        const auto& g = cam->geometry();
        w = g.get_width();
        h = g.get_height();
    } catch (const std::exception& e) {
        QMetaObject::invokeMethod(this, [this, msg = tr("Source file has no geometry: %1")
                                                              .arg(QString::fromUtf8(e.what()))]() {
            emit failed(msg);
        }, Qt::QueuedConnection);
        return;
    }
    if (w <= 0 || h <= 0) {
        QMetaObject::invokeMethod(this, [this, w, h]() {
            emit failed(tr("Source file has invalid geometry (%1x%2). Cannot export.")
                            .arg(w).arg(h));
        }, Qt::QueuedConnection);
        return;
    }

    // Direct cv::VideoWriter (synchronous), NOT Metavision::CvVideoRecorder:
    // the SDK recorder is THREADED with a small bounded object pool (~8
    // frames) and silently discards frames when production outruns encoding —
    // at slow-motion frame rates (gen fps = 1e6/acc, e.g. 3333 fps at
    // acc=300us) nearly every frame was dropped ("video does not contain all
    // of the raw content"). A synchronous writer blocks the frame callback
    // during encoding, which naturally propagates backpressure through the
    // generator thread -> last_frame_ts -> the event-callback throttle.
    cv::VideoWriter recorder;
    int fourcc = (p.quality >= 50) ? cv::VideoWriter::fourcc('H', '2', '6', '4')
                                   : cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
    if (!recorder.open(p.output_path.toStdString(), fourcc,
                       static_cast<double>(p.fps), cv::Size(w, h),
                       /*isColor=*/p.color) && p.quality >= 50) {
        // H264 encoders are often missing from Linux OpenCV builds (no
        // openh264/FFmpeg backend) — the default quality=90 would otherwise
        // always fail. Fall back to MJPG (widely available) and say so,
        // instead of dying with a misleading path/permission-style error.
        qWarning("AVI export: H264 encoder unavailable, falling back to MJPG");
        fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
        recorder.open(p.output_path.toStdString(), fourcc,
                      static_cast<double>(p.fps), cv::Size(w, h), p.color);
    }
    if (!recorder.isOpened()) {
        QMetaObject::invokeMethod(this, [this, fourcc]() {
            emit failed(tr("Failed to open AVI writer (codec %1 unavailable or path not writable).")
                            .arg(fourcc == cv::VideoWriter::fourcc('M','J','P','G')
                                     ? QStringLiteral("MJPG") : QStringLiteral("H264")));
        }, Qt::QueuedConnection);
        return;
    }

    // PeriodicFrameGenerationAlgorithm used DIRECTLY (synchronously): the cd
    // callback calls process_events(), which invokes the output callback —
    // and thus the encoder — on the SAME SDK read thread. Encoding throttles
    // reading naturally: no events_back_ queue, no backpressure sleeps, and
    // no drain phase (every frame is produced by the time the read finishes).
    //
    // Crucially, the algorithm takes the accumulation time directly, so small
    // windows are exact. CDFrameGenerator::start() takes uint16 fps (and the
    // previous code additionally capped it at 10000): any accumulation below
    // 100us silently collapsed to a 100us frame period, making 10us and 50us
    // windows produce identical 31s videos (user report).
    Metavision::PeriodicFrameGenerationAlgorithm gen(
        w, h,
        static_cast<std::uint32_t>(std::max<Metavision::timestamp>(p.accumulation_us, 1)),
        /*fps=*/0.,  // 0 = derive from the accumulation time (exact period)
        p.color ? Metavision::ColorPalette::Dark : Metavision::ColorPalette::Gray);
    const Metavision::timestamp frame_period_us =
        std::max<Metavision::timestamp>(p.accumulation_us, 1);

    // callback_error captures the first exception message from any callback.
    // It is written before cancel_ is set (release), so the polling loop
    // observes it after reading cancel_ (acquire). This lets us distinguish
    // a real error from a user-initiated cancel and show the actual cause.
    //
    // Mutex-guarded: the frame callback and the event callback can both catch
    // exceptions and write callback_error (the output callback fires inside
    // process_events on the read thread, but cancel paths race) — an
    // unsynchronized std::string write is a data race (UB).
    std::string callback_error;
    std::mutex callback_error_mtx;

    // Timestamp of the last frame actually written to the AVI — drives the
    // progress bar (tracks the real bottleneck: frame production + encoding).
    std::atomic<Metavision::timestamp> last_frame_ts{0};

    // Color handling: CvVideoRecorder was constructed with p.color as its
    // `colored` flag, so we must hand it a frame whose channel count matches:
    // 3-channel BGR when p.color, 1-channel gray otherwise. cv::VideoWriter
    // with isColor=false fed a 3-channel image (or vice versa) produces
    // corrupted output or an OpenCV assertion.
    // Expected timestamp of the NEXT frame, for gap filling: the generator
    // only produces frames when events arrive, so quiet stretches of the
    // recording produce NO frames and the video's time axis would compress
    // (user report: "video does not contain all of the raw content"). We
    // insert black frames for every missing period so 1 s of event time
    // always maps to exactly (1e6/acc) frames in the output.
    Metavision::timestamp next_frame_ts = -1;
    cv::Mat black_frame;
    gen.set_output_callback(
        [&recorder, &callback_error, &callback_error_mtx, this, color = p.color,
         &last_frame_ts, frame_period_us,
         &next_frame_ts, &black_frame](Metavision::timestamp ts, cv::Mat& frame) {
            try {
                // Publish frame timestamp for progress reporting.
                last_frame_ts.store(ts, std::memory_order_relaxed);
                // Only a real cancel skips writing.
                if (cancel_) return;
                auto write_frame = [&](const cv::Mat& f) {
                    cv::Mat out;
                    if (color) {
                        if (f.channels() == 1) {
                            cv::cvtColor(f, out, cv::COLOR_GRAY2BGR);
                        } else {
                            out = f;
                        }
                    } else {
                        if (f.channels() == 3) {
                            cv::cvtColor(f, out, cv::COLOR_BGR2GRAY);
                        } else {
                            out = f;
                        }
                    }
                    recorder.write(out);
                };
                if (black_frame.empty() && !frame.empty()) {
                    black_frame = cv::Mat::zeros(frame.rows, frame.cols,
                                                 frame.type());
                }
                // Fill quiet-gap periods with black frames so the
                // output time axis is complete.
                if (next_frame_ts < 0) next_frame_ts = ts;
                while (next_frame_ts < ts && !black_frame.empty()) {
                    write_frame(black_frame);
                    next_frame_ts += frame_period_us;
                }
                next_frame_ts = ts + frame_period_us;
                if (frame.empty()) return;
                write_frame(frame);
            } catch (const std::exception& e) {
                {
                    std::lock_guard<std::mutex> lk(callback_error_mtx);
                    callback_error = e.what();
                }
                cancel_.store(true, std::memory_order_release);
            } catch (...) {
                {
                    std::lock_guard<std::mutex> lk(callback_error_mtx);
                    callback_error = "Unknown error in frame writer";
                }
                cancel_.store(true, std::memory_order_release);
            }
        });

    auto id = cam->cd().add_callback(
        [&gen, &callback_error, &callback_error_mtx, this](const Metavision::EventCD* b, const Metavision::EventCD* e) {
            try {
                if (cancel_.load(std::memory_order_acquire)) return;
                // Synchronous: process_events() runs frame production AND
                // encoding on this read thread, so reading is throttled by
                // the encoder automatically — no queue to bound.
                gen.process_events(b, e);
            } catch (const std::exception& e2) {
                {
                    std::lock_guard<std::mutex> lk(callback_error_mtx);
                    callback_error = e2.what();
                }
                cancel_.store(true, std::memory_order_release);
            } catch (...) {
                {
                    std::lock_guard<std::mutex> lk(callback_error_mtx);
                    callback_error = "Unknown error in event callback";
                }
                cancel_.store(true, std::memory_order_release);
            }
        });

    // Wrap cam->start()..polling in a try/catch: if anything throws after
    // add_callback (above), remove_callback must run before gen is destructed
    // — otherwise the CD callback dereferences a dangling &gen (use-after-free).
    try {
    cam->start();
    // Query the duration asynchronously if the caller didn't provide it —
    // a synchronous get_duration() here would block the worker for the
    // whole index build (the entire export for short files), making the
    // progress bar jump 0 -> 100 (user report).
    if (dur_us->load(std::memory_order_relaxed) == 0) {
        query_duration_async(cam, dur_us);
    }
    while (!cancel_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        if (!cam->is_running()) break;
        const Metavision::timestamp dur = dur_us->load(std::memory_order_relaxed);
        if (dur > 0) {
            // Progress = timestamp of the last frame actually WRITTEN to the
            // AVI — tracks the real bottleneck (frame production + encoding).
            const double r = std::min(1.0, static_cast<double>(
                last_frame_ts.load(std::memory_order_relaxed)) / dur);
            QMetaObject::invokeMethod(this, [this, r]() { emit progress(r); },
                                      Qt::QueuedConnection);
        }
    }
    try { cam->stop(); } catch (...) {}
    } catch (...) {
        try { cam->stop(); } catch (...) {}
        try { cam->cd().remove_callback(id); } catch (...) {}
        throw;
    }
    try { cam->cd().remove_callback(id); } catch (...) {}
    // Everything read so far has already been framed and encoded
    // (process_events is synchronous). Flush the trailing partial
    // accumulation window as a final frame so the tail isn't cut short.
    if (!cancel_.load(std::memory_order_acquire)) {
        try { gen.force_generate(); } catch (...) {}
    }
    recorder.release();

    if (cancel_.load(std::memory_order_acquire)) {
        // Partial/truncated AVI — delete it (audit §六-E4).
        QFile::remove(p.output_path);
        std::string err_msg;
        {
            std::lock_guard<std::mutex> lk(callback_error_mtx);
            err_msg = callback_error;
        }
        QMetaObject::invokeMethod(this, [this, msg = std::move(err_msg)]() {
            emit failed(msg.empty() ? tr("Export cancelled.")
                                    : QString::fromUtf8(msg.c_str()));
        }, Qt::QueuedConnection);
        return;
    }
    QMetaObject::invokeMethod(this, [this, out = p.output_path]() {
        emit progress(1.0);
        emit completed(out);
    }, Qt::QueuedConnection);
}

void ExporterController::run_hdf5_external(const ExportParams& p) {
    auto source = try_open_external_file(p.source_path.toStdString());
    try {
        source->open();
    } catch (const std::exception& e) {
        QMetaObject::invokeMethod(this, [this, msg = QString::fromUtf8(e.what())]() {
            emit failed(msg);
        }, Qt::QueuedConnection);
        return;
    }

    Metavision::HDF5EventFileWriter writer(p.output_path.toStdString());
    std::string cb_error;
    std::mutex cb_mtx;
    std::atomic<bool> cb_error_flag{false};
    auto fail_cb = [&](const std::string& msg) {
        std::lock_guard<std::mutex> lk(cb_mtx);
        if (!cb_error_flag.load(std::memory_order_acquire)) {
            cb_error = msg;
            cb_error_flag.store(true, std::memory_order_release);
        }
        source->request_stop();
    };
    const Metavision::timestamp dur =
        p.duration_us > 0 ? p.duration_us : source->meta().duration_us;
    source->run(
        [&](const Metavision::EventCD* b, const Metavision::EventCD* e) {
            if (cancel_.load(std::memory_order_acquire)) {
                source->request_stop();
                return;
            }
            try {
                writer.add_events(b, e);
                if (dur > 0) {
                    const double r = std::min(
                        1.0, static_cast<double>((e - 1)->t) / static_cast<double>(dur));
                    QMetaObject::invokeMethod(this, [this, r]() { emit progress(r); },
                                              Qt::QueuedConnection);
                }
            } catch (const std::exception& ex) {
                fail_cb(ex.what());
            } catch (...) {
                fail_cb("Unknown error in HDF5 writer");
            }
        },
        [&](const std::string& err) {
            if (!err.empty()) fail_cb(err);
        });
    // writer.close() may throw if the ECF compression plugin is missing.
    try {
        writer.close();
    } catch (const std::exception& ex) {
        fail_cb(ex.what());
    } catch (...) {
        fail_cb("Unknown error closing HDF5 file");
    }

    // Distinguish cancel from completion — same contract as run_hdf5 /
    // run_avi_external: a cancelled export must not emit completed, and the
    // partial output must not survive on disk looking like a valid file.
    if (cancel_.load(std::memory_order_acquire)) {
        QFile::remove(p.output_path);
        std::string msg;
        {
            std::lock_guard<std::mutex> lk(cb_mtx);
            msg = cb_error;
        }
        QMetaObject::invokeMethod(this, [this, msg = std::move(msg)]() {
            emit failed(msg.empty() ? tr("Export cancelled.")
                                    : QString::fromUtf8(msg.c_str()));
        }, Qt::QueuedConnection);
        return;
    }
    if (cb_error_flag.load(std::memory_order_acquire)) {
        std::string msg;
        {
            std::lock_guard<std::mutex> lk(cb_mtx);
            msg = cb_error;
        }
        QFile::remove(p.output_path);
        QMetaObject::invokeMethod(this, [this, msg = std::move(msg)]() {
            emit failed(msg.empty() ? tr("Export cancelled.")
                                    : QString::fromUtf8(msg.c_str()));
        }, Qt::QueuedConnection);
        return;
    }
    QMetaObject::invokeMethod(this, [this, out = p.output_path]() {
        emit progress(1.0);
        emit completed(out);
    }, Qt::QueuedConnection);
}

void ExporterController::run_avi_external(const ExportParams& p) {
    auto source = try_open_external_file(p.source_path.toStdString());
    try {
        source->open();
    } catch (const std::exception& e) {
        QMetaObject::invokeMethod(this, [this, msg = QString::fromUtf8(e.what())]() {
            emit failed(msg);
        }, Qt::QueuedConnection);
        return;
    }
    const ExternalFileMeta& meta = source->meta();
    if (meta.width <= 0 || meta.height <= 0) {
        QMetaObject::invokeMethod(this, [this, w = meta.width, h = meta.height]() {
            emit failed(tr("Source file has invalid geometry (%1x%2). Cannot export.")
                            .arg(w).arg(h));
        }, Qt::QueuedConnection);
        return;
    }

    cv::VideoWriter recorder;
    int fourcc = (p.quality >= 50) ? cv::VideoWriter::fourcc('H', '2', '6', '4')
                                   : cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
    if (!recorder.open(p.output_path.toStdString(), fourcc,
                       static_cast<double>(p.fps), cv::Size(meta.width, meta.height),
                       /*isColor=*/p.color) && p.quality >= 50) {
        qWarning("AVI export: H264 encoder unavailable, falling back to MJPG");
        fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
        recorder.open(p.output_path.toStdString(), fourcc,
                      static_cast<double>(p.fps), cv::Size(meta.width, meta.height),
                      p.color);
    }
    if (!recorder.isOpened()) {
        QMetaObject::invokeMethod(this, [this, fourcc]() {
            emit failed(tr("Failed to open AVI writer (codec %1 unavailable or path not writable).")
                            .arg(fourcc == cv::VideoWriter::fourcc('M','J','P','G')
                                     ? QStringLiteral("MJPG") : QStringLiteral("H264")));
        }, Qt::QueuedConnection);
        return;
    }

    // Same synchronous PeriodicFrameGenerationAlgorithm setup as the SDK
    // path — process_events() runs frame production AND encoding on this
    // (reader) thread, so reading is throttled by the encoder automatically.
    Metavision::PeriodicFrameGenerationAlgorithm gen(
        meta.width, meta.height,
        static_cast<std::uint32_t>(std::max<Metavision::timestamp>(p.accumulation_us, 1)),
        /*fps=*/0.,
        p.color ? Metavision::ColorPalette::Dark : Metavision::ColorPalette::Gray);
    const Metavision::timestamp frame_period_us =
        std::max<Metavision::timestamp>(p.accumulation_us, 1);

    std::string cb_error;
    std::mutex cb_mtx;
    auto set_cb_error = [&](const std::string& msg) {
        std::lock_guard<std::mutex> lk(cb_mtx);
        if (cb_error.empty()) cb_error = msg;
        cancel_.store(true, std::memory_order_release);
        source->request_stop();
    };
    std::atomic<Metavision::timestamp> last_frame_ts{0};
    const auto dur = p.duration_us > 0 ? p.duration_us : source->meta().duration_us;

    Metavision::timestamp next_frame_ts = -1;
    cv::Mat black_frame;
    gen.set_output_callback(
        [&recorder, &cb_error, &cb_mtx, &set_cb_error, this, color = p.color,
         &last_frame_ts, frame_period_us, &next_frame_ts,
         &black_frame](Metavision::timestamp ts, cv::Mat& frame) {
            try {
                last_frame_ts.store(ts, std::memory_order_relaxed);
                if (cancel_.load(std::memory_order_acquire)) return;
                auto write_frame = [&](const cv::Mat& f) {
                    cv::Mat out;
                    if (color) {
                        if (f.channels() == 1) {
                            cv::cvtColor(f, out, cv::COLOR_GRAY2BGR);
                        } else {
                            out = f;
                        }
                    } else {
                        if (f.channels() == 3) {
                            cv::cvtColor(f, out, cv::COLOR_BGR2GRAY);
                        } else {
                            out = f;
                        }
                    }
                    recorder.write(out);
                };
                if (black_frame.empty() && !frame.empty()) {
                    black_frame = cv::Mat::zeros(frame.rows, frame.cols,
                                                 frame.type());
                }
                // Fill quiet-gap periods with black frames so the output
                // time axis stays complete (same as the SDK path).
                if (next_frame_ts < 0) next_frame_ts = ts;
                while (next_frame_ts < ts && !black_frame.empty()) {
                    write_frame(black_frame);
                    next_frame_ts += frame_period_us;
                }
                next_frame_ts = ts + frame_period_us;
                if (frame.empty()) return;
                write_frame(frame);
            } catch (const std::exception& e) {
                set_cb_error(e.what());
            } catch (...) {
                set_cb_error("Unknown error in frame writer");
            }
        });

    source->run(
        [&](const Metavision::EventCD* b, const Metavision::EventCD* e) {
            if (cancel_.load(std::memory_order_acquire)) {
                source->request_stop();
                return;
            }
            try {
                gen.process_events(b, e);
            } catch (const std::exception& e2) {
                set_cb_error(e2.what());
            } catch (...) {
                set_cb_error("Unknown error in event callback");
            }
        },
        [&](const std::string& err) {
            if (!err.empty()) set_cb_error(err);
        });
    // Flush the trailing partial accumulation window (same as the SDK path).
    if (!cancel_.load(std::memory_order_acquire)) {
        try { gen.force_generate(); } catch (...) {}
    }
    recorder.release();

    if (cancel_.load(std::memory_order_acquire)) {
        QFile::remove(p.output_path);
        std::string msg;
        {
            std::lock_guard<std::mutex> lk(cb_mtx);
            msg = cb_error;
        }
        QMetaObject::invokeMethod(this, [this, msg = std::move(msg)]() {
            emit failed(msg.empty() ? tr("Export cancelled.")
                                    : QString::fromUtf8(msg.c_str()));
        }, Qt::QueuedConnection);
        return;
    }
    QMetaObject::invokeMethod(this, [this, out = p.output_path]() {
        emit progress(1.0);
        emit completed(out);
    }, Qt::QueuedConnection);
}

} // namespace gui
