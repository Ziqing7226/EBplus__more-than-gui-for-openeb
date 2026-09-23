// gui/app/camera_controller.h — owns the Metavision::Camera lifecycle.
//
// Discovers, connects (live or file), wires the CD callback into the
// FramePipeline and StatisticsController, and exposes sensor metadata to the
// GUI. All cross-thread communication goes through queued Qt signals.

#ifndef GUI_APP_CAMERA_CONTROLLER_H
#define GUI_APP_CAMERA_CONTROLLER_H

#include <QObject>
#include <QString>
#include <atomic>
#include <deque>
#include <functional>
#include <mutex>
#include <utility>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <metavision/hal/facilities/i_antiflicker_module.h>
#include <metavision/hal/facilities/i_erc_module.h>
#include <metavision/hal/facilities/i_camera_synchronization.h>
#include <metavision/hal/facilities/i_event_trail_filter_module.h>
#include <metavision/hal/facilities/i_geometry.h>
#include <metavision/hal/facilities/i_ll_biases.h>
#include <metavision/hal/facilities/i_roi.h>
#include <metavision/hal/facilities/i_trigger_in.h>
#include <metavision/hal/facilities/i_trigger_out.h>
#include <metavision/sdk/base/utils/callback_id.h>
#include <metavision/sdk/base/utils/object_pool.h>
#include <metavision/sdk/stream/camera.h>

#include "frame_pipeline.h"
#include "stream_conditioner.h"
#include "statistics_controller.h"
#include "external_file_source.h"
#include "davis/aps_decoder.h"
#include "davis/imu_types.h"
#if GUI_HAVE_DAVIS
#include "davis/davis_ll_biases.h"
#include "davis/davis_device.h"
#include "davis/dvxplorer_ll_biases.h"
#include "davis/dvxplorer_device.h"
#endif
#include "algo_bridge/filter_chain.h"
#include "algo/analytics/auto_bias_controller.h"
#include "app/bias_applier.h"

namespace gui {

// HAL facility aliases used by Phase 2 panels. Each is obtained via
// Camera::get_device().get_facility<T>() which returns a nullable pointer;
// panels must nullptr-check before use (graceful degradation when the
// connected sensor doesn't support a feature).
namespace facility {
using Biases       = Metavision::I_LL_Biases;
using Roi          = Metavision::I_ROI;
using AntiFlicker  = Metavision::I_AntiFlickerModule;
using TrailFilter  = Metavision::I_EventTrailFilterModule;
using Erc          = Metavision::I_ErcModule;
using TriggerIn    = Metavision::I_TriggerIn;
using TriggerOut   = Metavision::I_TriggerOut;
using CameraSync   = Metavision::I_CameraSynchronization;
using Geometry     = Metavision::I_Geometry;
} // namespace facility

/// @brief Snapshot of sensor metadata shown in the Information panel.
struct SensorInfo {
    int width{0};
    int height{0};
    QString serial;
    QString integrator;
    QString plugin_name;
    QString encoding_format;
    QString firmware_version;
    QString generation_name;
    short generation_major{0};
    short generation_minor{0};
    bool is_file{false};
};

class CameraController : public QObject {
    Q_OBJECT
public:
    explicit CameraController(QObject* parent = nullptr);
    ~CameraController();

    /// @brief Lists online camera sources as (type_label, serial) pairs.
    std::vector<std::pair<QString, QString>> list_online_sources();

    /// @brief Connects to the first available live camera. Returns false on failure.
    bool connect_first_available();
    /// @brief Connects to a camera by serial number. Searches Metavision
    /// sources first, then live DAVIS cameras (when the build has libusb).
    bool connect_serial(const std::string& serial);
    /// @brief Opens an event file (RAW / HDF5 / DAT) for playback. Always
    /// uses real_time_playback=false so all events are read as fast as
    /// possible and buffered in the FileFrameGenerator. Playback rate is
    /// controlled by the FileFrameGenerator's QTimer (fps * window / 1e6).
    bool connect_file(const std::string& path);

    void disconnect();

    bool start();
    bool stop();
    bool is_running() const;
    bool is_connected() const {
#if GUI_HAVE_DAVIS
        if (davis_device_ || dvx_device_) return true;
#endif
        return static_cast<bool>(camera_) || external_source_ != nullptr;
    }
    bool is_file_source() const { return is_file_; }

    /// @brief Returns the underlying Metavision::Camera (nullptr if none).
    /// External file sources (AEDAT4/ALPDATA) have no SDK camera.
    Metavision::Camera* camera_handle() { return camera_.get(); }

    /// @brief True while an inivation device (DAVIS/DVXplorer) is connected
    /// (Phase 4: routes recording to the AEDAT4 writer).
    bool is_inivation_source() const {
#if GUI_HAVE_DAVIS
        return davis_device_ != nullptr || dvx_device_ != nullptr;
#else
        return false;
#endif
    }

    /// @brief Phase 4: single-consumer tap on the RAW device stream (the
    /// span as delivered by the device, before conditioning) — used by the
    /// recorder to write AEDAT4 files. Invoked from the device thread.
    using RawTap = std::function<void(const Metavision::EventCD*, const Metavision::EventCD*)>;
    void set_raw_tap(RawTap tap);
    /// Recording taps for the inivation side streams (IMU samples / APS
    /// frames) — invoked on the device thread like RawTap. Assignment is
    /// serialized through tap_mutex_ and the call sites invoke a COPY taken
    /// under that mutex: a plain member assign races the USB/reader thread's
    /// check-and-call when recording stops while the camera is still
    /// streaming (the functor can be destroyed mid-call).
    void set_imu_tap(std::function<void(const davis::ImuSample&)> tap) {
        std::lock_guard<std::mutex> lock(tap_mutex_);
        imu_tap_ = std::move(tap);
    }
    void set_aps_tap(std::function<void(const davis::ApsFrame&)> tap) {
        std::lock_guard<std::mutex> lock(tap_mutex_);
        aps_tap_ = std::move(tap);
    }

    /// @brief Duration reported by an external file source (0 when the
    /// current source is an SDK camera or unknown until fully streamed).
    /// PlaybackController uses it instead of the SDK's OSC query.
    Metavision::timestamp external_duration_hint() const {
        return external_source_ ? external_source_->meta().duration_us
                                : Metavision::timestamp{0};
    }

    const SensorInfo& sensor_info() const { return sensor_info_; }
    FramePipeline* frame_pipeline() { return &frame_pipeline_; }
    StatisticsController* statistics() { return &statistics_; }
    FilterChain* filter_chain() { return &filter_chain_; }

    /// @brief Phase 2 facility accessors. Each returns nullptr when no camera
    /// is connected or the connected sensor does not support that feature.
    /// Panels must nullptr-check before invoking any method on the returned
    /// pointer. The pointer is only valid until the next disconnect()/connect.
    facility::Biases*      biases_facility();
    facility::Roi*         roi_facility();
    facility::AntiFlicker* anti_flicker_facility();
    facility::TrailFilter* trail_filter_facility();
    facility::Erc*         erc_facility();
    facility::TriggerIn*   trigger_in_facility();
    facility::TriggerOut*  trigger_out_facility();
    facility::CameraSync*  camera_sync_facility();

    /// @brief Which hardware facility groups the CURRENT source actually
    /// provides (Phase 1 capability framework). Facility-backed panels that
    /// lack their capability auto-hide while this source is connected.
    /// SDK cameras report from their HAL facilities; inivation devices and
    /// file sources report the device layer's implemented subset (biases
    /// only — no trigger/ESP streams yet).
    struct SourceCapabilities {
        bool trigger{false};  ///< I_TriggerIn or I_TriggerOut present.
        bool esp{false};      ///< Anti-flicker, trail or ERC module present.
        bool imu{false};      ///< inivation IMU6 stream (enable + read out).
        bool aps{false};      ///< inivation APS frames (DAVIS only).
    };
    SourceCapabilities source_capabilities();

    // Phase 2/3: IMU/APS stream API. Declared unconditionally (the IMU/APS
    // windows and the devices panel compile in every configuration); builds
    // without libusb degrade to "not available" (set_* return false, the
    // samples/counters read back empty).
    /// @brief Enables/disables the inivation IMU stream. The three IMU RUN
    /// registers are written immediately while streaming and re-applied by
    /// the device on every start(); the flag is session-scoped (teardown
    /// clears it).
    bool set_imu_enabled(bool on);
    [[nodiscard]] bool imu_enabled() const;
    /// Latest completed IMU sample + a monotonic sample counter (the window
    /// derives the sample rate from the counter delta). Safe from any
    /// thread; the sample arrives on the libusb thread.
    davis::ImuSample latest_imu() const;
    [[nodiscard]] long imu_sample_count() const;
    /// @brief Enables/disables the DAVIS APS frame stream (MODULE_APS /
    /// APS_RUN; session-scoped like the IMU flag). Returns false for
    /// sources without APS frames (DVXplorer, SDK cameras, files).
    bool set_aps_enabled(bool on);
    [[nodiscard]] bool aps_enabled() const;
    /// Latest completed APS frame (cloned under the mutex).
    /// Live mode: the newest decoded frame. File replay: the newest frame
    /// at or before the playback position (decoded on demand from the
    /// file's APS packet index).
    davis::ApsFrame latest_aps_frame();
    [[nodiscard]] long aps_frame_count() const;
    /// @brief IMU plotting (Phase 2 visualization): drains the retained
    /// sample ring after @p cursor (sequence numbers from
    /// imu_sample_count()). Live sources: a fresh viewer (INT64_MIN) starts
    /// from the LATEST sample — integrating the stale backlog would fling
    /// the pose. File replays: a fresh viewer receives the WHOLE retained
    /// recording (samples decode once and never arrive again), served up to
    /// the current playback position. Returns the new samples in stream
    /// order and advances @p cursor. Without libusb returns an empty vector.
    std::vector<davis::ImuSample> drain_imu(std::int64_t& cursor);
    /// Current file-playback position (µs, normalized to file start) —
    /// gates IMU replay so the attitude animates with the playback.
    /// Negative when no file playback position is known (live sources).
    [[nodiscard]] Metavision::timestamp file_playback_position_us() const {
        return file_playback_pos_.load(std::memory_order_relaxed);
    }

    /// @brief Unified ROI entry point (Phase 2.6): the single ROI concept.
    /// Live camera: applies the hardware ROI (I_ROI) so the sensor itself
    /// only outputs ROI events. File playback: forwards to FramePipeline's
    /// software crop (same semantics). @p x/@p y = -1 means auto-center the
    /// window on the sensor. @p roni selects ROI (keep-inside, default) vs
    /// RONI (drop-inside) mode; std::nullopt keeps the current mode.
    /// Returns false on failure (facility missing / invalid rect) — caller
    /// should NOT treat the ROI as applied.
    /// On success emits roi_state_changed with the computed rect (the single
    /// driver for the overlay frame, zoom button and algorithm path,
    /// Phase 2.6 debug D-5).
    bool set_unified_roi(bool enabled, int x, int y, int w, int h,
                         std::optional<bool> roni = std::nullopt);

    /// @brief Connect-time sanity for the persistent unified ROI (called at
    /// the end of every connect path, after the conditioner init): the rect
    /// survives teardown by design (same-camera reconnects keep the crop),
    /// but a NEW source can be smaller than the saved rect — applied
    /// unclamped, the conditioner would crop every batch to zero events.
    /// Salvages the overlap, drops the ROI when nothing overlaps, resets a
    /// previous file's software crop, and re-emits roi_state_changed so the
    /// panels / bridge / display re-sync (disconnect unchecks the RoiPanel
    /// with signals blocked, so the UI would otherwise disagree).
    void validate_roi_after_connect();


    /// @brief Reads the current unified ROI state (computed rect
    /// [x0,x1) × [y0,y1]) for overlay rendering.
    void unified_roi(bool& enabled, int& x0, int& y0, int& x1, int& y1) const;

    /// @brief True when the unified ROI is in RONI (drop-inside) mode
    /// (Phase 2.6 debug D-5). In RONI mode events keep ABSOLUTE sensor
    /// coordinates (the source filters, nothing is translated) — consumers
    /// that shift coordinates (OverlayStrategy) or resize backends
    /// (AlgoBridge) must treat RONI as "no translation / no resize".
    bool unified_roi_roni() const { return roi_roni_; }

    /// @brief Enables/disables broadcasting of every CD batch via
    /// cd_events_ready(). When false (default), the CD callback takes the
    /// fast path with zero extra copies. Calibration tools flip this to true
    /// on start and back to false on stop so the SDK thread only pays the
    /// copy cost while a consumer is actively listening.
    void set_cd_broadcast(bool enabled);

    /// @brief Conditioned-listener signature: raw span [rb,re) as delivered
    /// by the SDK, plus the conditioned span [cb,ce) (ROI → polarity stages →
    /// noise filter → thin → undistort → flips — see StreamConditioner).
    /// The listener runs BEFORE the display push and may REWRITE the output
    /// references [ob,oe) (initially the conditioned span): a stream-filter
    /// algorithm's per-batch output (AlgoBackend::output_span, Phase 7 P6-A)
    /// replaces the display/recorded span, so hot_pixel_filter visibly
    /// removes events and EIS displays stabilized coordinates. Leaving
    /// ob/oe untouched = pass-through.
    using ConditionedListener = std::function<void(
        const Metavision::EventCD* rb, const Metavision::EventCD* re,
        const Metavision::EventCD* cb, const Metavision::EventCD* ce,
        const Metavision::EventCD*& ob, const Metavision::EventCD*& oe)>;

    /// @brief Registers the live-mode consumer of the conditioned stream
    /// (algorithm feeding, XYT display). Invoked on the SDK CD thread BEFORE
    /// the display pipeline receives the span — so a filtering algorithm can
    /// substitute the display/recorded span via the ob/oe references
    /// (one conditioning pass for everyone). File sources never invoke it (file playback feeds
    /// algorithms via FramePipeline::events_window_ready, synchronized with
    /// the displayed frame).
    void set_conditioned_listener(ConditionedListener cb);

    /// @brief Single entry for the panel's preproc_* parameters: forwards to
    /// the live conditioner AND the file generator's conditioner. Same keys
    /// as AlgorithmsPanel::apply_global_preproc emits.
    bool set_conditioner_param(const std::string& key, const std::string& value);

    /// @brief Clears the conditioners' temporal state (source restart, file
    /// seek/loop — event time jumps backward and stale timestamp surfaces
    /// would suppress events).
    void reset_conditioner();

    /// @brief True when any conditioning stage would modify the stream.
    /// Used by RecorderController to choose processed-stream recording.
    bool conditioner_active() const;

    // --- Auto bias (§4.4.6) ---------------------------------------------
    // Camera-level dual-loop bias control (total-rate band + ON/OFF
    // balance over bias_diff_on/off). Independent of the algorithm
    // instances — it coexists with every other feature. Live sources only;
    // enabling on a file source or a sensor without diff biases fails.

    /// @brief Enables/disables auto bias. On enable the diff biases are
    ///        snapshotted; disable restores them. Returns false when the
    ///        current source cannot be bias-driven (file / no diff biases).
    bool set_auto_bias_enabled(bool on);
    bool auto_bias_enabled() const {
        return auto_bias_enabled_.load(std::memory_order_relaxed);
    }
    /// @brief Sets the target rate band (cross-validated: rejected unless
    ///        it forms a valid lo < hi band; the max has no ceiling — the
    ///        hardware bias range is the real limit).
    bool set_auto_bias_rate_bounds(float lo_mev, float hi_mev);
    void auto_bias_rate_bounds(float& lo_mev, float& hi_mev) const;

signals:
    /// AEDAT4 replay discovered an actual IMU(true)/APS(false) packet —
    /// fired once per stream, on the reader thread (UI updates must queue).
    void file_side_stream_discovered(bool imu);
    void connected(const SensorInfo& info);
    void disconnected();
    void started();
    void stopped();
    void error(const QString& message);
    void runtime_warning(const QString& message);
    /// @brief Emitted after every successful set_unified_roi (Phase 2.6
    /// debug D-5): the single driver for the overlay frame, the Zoom-to-ROI
    /// button, the algorithm path (AlgoBridge::set_unified_roi_state) and
    /// GUI checkbox sync. Carries the COMPUTED rect [x0,x1) × [y0,y1).
    void roi_state_changed(bool enabled, int x0, int y0, int x1, int y1);
    /// @brief Emitted from the SDK CD callback (cross-thread, queued) when
    /// cd_broadcast_ is true. Carries a shared_ptr copy of the batch so
    /// listeners on the GUI thread can process it safely. Used by the
    /// calibration wizard's 1 ms event accumulator.
    void cd_events_ready(std::shared_ptr<std::vector<Metavision::EventCD>> events);
    /// @brief Auto bias wrote the diff bias registers out-of-band (or
    /// restored them on disable) — the Biases panel re-reads the hardware.
    void auto_bias_applied();

private:
    /// Auto bias measurement + control state, guarded by auto_bias_mutex_
    /// (accumulate/tick on the SDK thread, params on the GUI thread).
    void auto_bias_tick(const Metavision::EventCD* b, const Metavision::EventCD* e);
    mutable std::mutex auto_bias_mutex_;
    gui_algo::AutoBiasController auto_bias_ctrl_;
    std::int64_t auto_bias_last_tick_{-1};
    BiasApplier bias_applier_;
    std::atomic<bool> auto_bias_enabled_{false};

private:
    /// @brief Sets up callbacks + pipeline for a new camera. Calls
    /// frame_pipeline_.start_file() for file sources (FileFrameGenerator)
    /// or frame_pipeline_.start() for live sources (CDFrameGenerator).
    void setup_camera(Metavision::Camera&& cam, bool is_file);
    /// @brief Live-stream event handler shared by the SDK CD callback and the
    /// DAVIS source: statistics → auto bias → conditioning → listener →
    /// pipeline, plus the optional raw CD broadcast.
    void on_live_events(const Metavision::EventCD* b, const Metavision::EventCD* e);
#if GUI_HAVE_DAVIS
    /// @brief Connects to a live inivation DAVIS camera (events + biases).
    bool connect_davis(const davis::DeviceDescriptor& descriptor);
    /// @brief DAVIS device unplugged mid-stream (from the libusb thread).
    void on_davis_gone();
    /// @brief Connects to a live inivation DVXplorer camera (events + the
    /// two contrast thresholds; no Auto Bias — no diff biases).
    bool connect_dvx(const davis::DeviceDescriptor& descriptor);
    void on_dvx_gone();
#endif
    /// @brief Opens a non-SDK file format (AEDAT4 / ALPDATA): parses the
    /// header, populates sensor_info_ from the reader's metadata and starts
    /// the shared FileFrameGenerator — identical downstream behavior, only
    /// the event source differs (reader thread instead of the SDK).
    bool connect_external_file(std::unique_ptr<ExternalFileSource> source);
    /// @brief Completion of the external reader thread (queued to the GUI
    /// thread): enables the FileFrameGenerator's EOF handling, like the SDK
    /// camera's EOF status/error callback. Empty @p error = clean EOF.
    void on_external_source_done(const QString& error);
    /// @brief Tears down camera + callbacks + frame pipeline.
    void teardown();
    void fetch_sensor_info();

    std::unique_ptr<Metavision::Camera> camera_;
#if GUI_HAVE_DAVIS
    /// Live DAVIS source + its bias facility. Mutually exclusive with
    /// camera_ and with external_source_.
    std::unique_ptr<davis::Device> davis_device_;
    std::unique_ptr<davis::DavisLLBiases> davis_biases_;
    bool davis_streaming_started_{false};
    std::unique_ptr<davis::DvxplorerDevice> dvx_device_;
    std::unique_ptr<davis::DvxLLBiases> dvx_biases_;
    bool dvx_streaming_started_{false};

    /// IMU stream state (flag owned by the GUI thread; the sample is
    /// written from the libusb thread under imu_mutex_).
    bool imu_enabled_{false};
    mutable std::mutex imu_mutex_;
    davis::ImuSample imu_latest_{};
    long imu_count_{0};
    /// Rolling ring backing drain_imu() — capped (kImuRingMax) and cleared
    /// by teardown.
    std::deque<std::pair<std::int64_t, davis::ImuSample>> imu_ring_;
    static constexpr std::size_t kImuRingMax = 8192;  // ~10 s at 800 Hz

    /// APS frame stream state (DAVIS only; same session-scoped pattern).
    bool aps_enabled_{false};
    mutable std::mutex aps_mutex_;
    davis::ApsFrame aps_latest_{};
    long aps_count_{0};
#endif
    void on_imu_sample(const davis::ImuSample& sample);
    void on_aps_frame(const davis::ApsFrame& frame);

    /// Phase 4 recorder tap (raw device stream; nulled by teardown).
    /// Unconditional — the AEDAT4 recorder installs it for inivation
    /// sources only, but the accessor itself has no inivation dependency.
    RawTap raw_tap_;
    /// File-playback position (mirrored from the FramePipeline signal).
    std::atomic<Metavision::timestamp> file_playback_pos_{-1};
    std::function<void(const davis::ImuSample&)> imu_tap_;
    std::function<void(const davis::ApsFrame&)> aps_tap_;
    /// Serializes tap installation/teardown (GUI thread) against the
    /// device-thread snapshots taken at every tap call site.
    mutable std::mutex tap_mutex_;
    std::atomic<bool> imu_discovered_{false};
    std::atomic<bool> aps_discovered_{false};
    /// External (non-SDK) file source and its reader thread. Mutually
    /// exclusive with camera_: only one is ever set.
    std::unique_ptr<ExternalFileSource> external_source_;
    std::thread external_thread_;
    std::atomic<bool> external_running_{false};
    /// One-shot guard: start() must never re-read an already-buffered file.
    bool external_started_{false};
    std::optional<Metavision::CallbackId> cd_cb_id_;
    std::optional<Metavision::CallbackId> err_cb_id_;
    std::optional<Metavision::CallbackId> status_cb_id_;
    SensorInfo sensor_info_;
    bool is_file_{false};
    /// Unified ROI state (live path; file path keeps its rect in
    /// FileFrameGenerator, read via frame_pipeline_.file_roi()). roi_roni_
    /// is tracked here for BOTH paths (single writer: set_unified_roi).
    bool roi_enabled_{false};
    bool roi_roni_{false};
    int roi_x0_{0}, roi_y0_{0}, roi_x1_{0}, roi_y1_{0};

    FramePipeline frame_pipeline_;
    StatisticsController statistics_;
    FilterChain filter_chain_;
    /// Shared conditioning (live batches). The file source's conditioner
    /// lives inside FileFrameGenerator; only one is ever active.
    StreamConditioner conditioner_;
    /// Conditioned-stream listener (live mode), guarded by
    /// conditioned_mutex_. Copied into a local before invocation so a
    /// set_conditioned_listener() during a callback cannot race.
    ConditionedListener conditioned_listener_;
    std::mutex conditioned_mutex_;
    /// @brief When true, the CD callback emits cd_events_ready() with a copy
    /// of every batch. Off by default so non-calibration usage pays nothing.
    std::atomic<bool> cd_broadcast_{false};
    /// @brief Reusable event-buffer pool for the CD broadcast: buffers are
    /// returned to the pool when the last consumer (calibration tap / worker)
    /// drops its shared_ptr, so steady-state broadcasting allocates nothing.
    Metavision::ObjectPool<std::vector<Metavision::EventCD>, true> broadcast_pool_;
};

} // namespace gui

#endif // GUI_APP_CAMERA_CONTROLLER_H
