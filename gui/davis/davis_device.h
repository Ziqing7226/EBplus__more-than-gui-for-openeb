// gui/davis/davis_device.h — live inivation DAVIS346/640 camera over libusb.
//
// Ported from dv-processing 2.0.4 io/camera/{usb_device,davis}.hpp
// (Apache-2.0) with a reduced, events-only feature set: APS frames, IMU
// samples and trigger markers are parsed and discarded, the stream is
// rebased to start at t=0 after each device timestamp reset.
//
// Threading model mirrors the reference: a dedicated libusb event thread
// services N queued bulk transfers (data endpoint 0x82) and the control
// transfers used for configuration; the event sink is invoked from that
// thread — the consumer (CameraController) treats it exactly like a
// Metavision SDK CD callback.

#ifndef GUI_DAVIS_DAVIS_DEVICE_H
#define GUI_DAVIS_DAVIS_DEVICE_H

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <libusb.h>

#include "imu_types.h"

#include <metavision/sdk/base/events/event_cd.h>

#include "davis_biases.h"
#include "batch_worker.h"
#include "davis_parser.h"

namespace gui::davis {

struct DeviceDescriptor {
    std::uint16_t vid{0};
    std::uint16_t pid{0};
    std::uint8_t bus{0};
    std::uint8_t addr{0};
    std::uint8_t firmware{0};
    std::string serial;
};

/// Enumerates connected inivation DAVIS (FX2/FX3) cameras without opening them.
std::vector<DeviceDescriptor> find_devices();

class Device {
public:
    /// Sink invoked from the libusb thread with decoded, 0-based events.
    using EventSink = std::function<void(const Metavision::EventCD*, const Metavision::EventCD*)>;
    /// Invoked (libusb thread) when the device disappears unexpectedly.
    using GoneCallback = std::function<void()>;

    /// Opens and fully configures the camera; the stream stays idle until
    /// start(). Throws std::runtime_error on any failure (permissions,
    /// missing device, unsupported firmware/logic version, …).
    explicit Device(const DeviceDescriptor& descriptor);
    ~Device();

    Device(const Device&) = delete;
    Device& operator=(const Device&) = delete;

    void set_event_sink(EventSink sink);
    /// Recording tap (see raw_consumer_). Callable while streaming: the
    /// write races the USB-thread reads the same benign way the controller's
    /// raw_tap_ member always has (rare start/stop assignment).
    void set_raw_consumer(std::function<void(const Metavision::EventCD*, const Metavision::EventCD*)> cb) {
        raw_consumer_ = std::move(cb);
    }
    void set_gone_callback(GoneCallback callback);
    /// Completed IMU6 samples (accel/gyro/temp) — invoked from the USB
    /// thread when the IMU stream is enabled.
    void set_imu_sink(const ImuSink& sink);
    /// Enables/disables the IMU stream (three RUN registers; applied at
    /// once, and re-applied by start() while @p on).
    void set_imu_enabled(bool on);
    [[nodiscard]] bool imu_enabled() const { return imu_enabled_; }
    /// Completed APS frames — invoked from the USB thread when the APS
    /// stream is enabled.
    void set_aps_sink(const ApsFrameSink& sink);
    /// Enables/disables the APS stream (MODULE_APS / APS_RUN; applied at
    /// once, and re-applied by start() while @p on).
    void set_aps_enabled(bool on);
    [[nodiscard]] bool aps_enabled() const { return aps_enabled_; }
    /// @brief Phase 6: programs the DVS hardware ROI filter (keep-inside)
    /// with a USER-frame rect. Mirrors the reference sequence (stop DVS,
    /// four ROI registers, restore run). Returns false when the sensor
    /// lacks the filter, the rect is invalid, or the SPI write fails —
    /// the conditioner's software ROI still applies either way.
    bool set_hw_roi(int x, int y, int w, int h);
    /// Resets the filter to the full sensor (the disabled shape).
    bool clear_hw_roi();
    /// @brief Per-frame auto-exposure (Phase 6 APS): ported from the
    /// reference computeAutomaticExposure — histogram-based under/over
    /// detection plus mean-sample-value refinement, adjusting the
    /// APS_EXPOSURE register. Runs on the USB thread; the register write is
    /// submitted asynchronously (a synchronous wait inside the data
    /// callback would deadlock the event loop).
    void apply_auto_exposure(const davis::ApsFrame& frame);
    /// Fire-and-forget control OUT (self-freeing transfer).
    void usb_control_out_noblock(std::uint8_t request, std::uint16_t value,
                                 std::uint16_t index, const std::uint8_t* data,
                                 std::size_t size);

    /// Starts event streaming (data transfers + run switches + timestamp
    /// reset handshake; blocks up to ~1 s waiting for the reset marker).
    /// Auto-exposure control (diagnostics and the APS panel toggle).
    void set_auto_exposure(bool on) { auto_exposure_ = on; }
    [[nodiscard]] bool auto_exposure() const { return auto_exposure_; }
    /// The current AEC exposure estimate (µs) — diagnostics.
    [[nodiscard]] double aec_exposure_us() const { return aec_exposure_us_; }

    void start();
    /// Stops event streaming; the camera returns to the configured-idle state.
    void stop();

    [[nodiscard]] int width() const { return width_; }
    [[nodiscard]] int height() const { return height_; }
    [[nodiscard]] const std::string& serial() const { return serial_; }
    /// Sensor chip identifier (MODULE_SYSINFO / chip identifier) — selects
    /// the model-specific bias table and quirk handling.
    [[nodiscard]] int chip_model() const { return chip_model_; }
    /// Raw orientation registers (bit 0x04 invert axes, 0x02 flip
    /// horizontal, 0x01 flip vertical) — DVS and APS have their own.
    [[nodiscard]] int dvs_orientation() const { return dvs_orientation_; }
    [[nodiscard]] int aps_orientation() const { return aps_orientation_; }
    [[nodiscard]] const std::string& model_name() const { return model_name_; }

    /// Bias store (register state + device writes). Lives as long as the
    /// device; safe to call from the GUI thread while streaming (SPI control
    /// transfers run on the libusb thread).
    BiasStore& biases() { return biases_; }

    /// @brief Raw SPI read of a MODULE_BIAS (5) register — hardware readback
    /// for verifying that parameter writes actually landed on the camera.
    [[nodiscard]] std::uint16_t read_bias_register(std::uint16_t address) {
        return static_cast<std::uint16_t>(spi_config_receive(5, address) & 0xFFFF);
    }
    /// @brief Raw SPI read of an arbitrary module register — diagnostics
    /// (register write verification). Safe while streaming (control
    /// transfers run on the libusb thread).
    [[nodiscard]] std::uint32_t read_module_register(std::uint8_t module,
                                                     std::uint16_t address) {
        return spi_config_receive(module, address);
    }

private:
    // USB primitives (ported from usb_device.hpp).
    void usb_control_out(std::uint8_t request, std::uint16_t value, std::uint16_t index,
                         const std::uint8_t* data, std::size_t size);
    void usb_control_in(std::uint8_t request, std::uint16_t value, std::uint16_t index,
                        std::uint8_t* data, std::size_t size);
    void spi_config_send(std::uint8_t module, std::uint16_t param, std::uint32_t value);
    std::uint32_t spi_config_receive(std::uint8_t module, std::uint16_t param);
    void spi_config_send_multiple(const std::vector<std::array<std::uint8_t, 6>>& configs);
    void usb_cleanup_buffers();
    void usb_thread_start();
    void usb_thread_stop();
    void usb_data_transfers_start();
    void usb_data_transfers_stop();

    // DAVIS configuration (ported from davis.hpp).
    void configure_idle(); // full reference init sequence, RUN switches off
    void send_timestamp_reset();
    bool wait_for_timestamp_reset();

    static void LIBUSB_CALL usb_data_transfer_cb(libusb_transfer* transfer);

    // USB state.
    libusb_context* context_{nullptr};
    libusb_device_handle* handle_{nullptr};
    std::string serial_;
    std::uint8_t data_endpoint_{0x82};
    std::thread usb_thread_;
    std::atomic<bool> usb_thread_run_{false};
    std::mutex usb_ops_lock_;
    std::vector<libusb_transfer*> data_transfers_;
    std::uint32_t data_transfers_active_{0};
    std::mutex data_transfers_lock_;

    void teardown_usb();

    // DAVIS state.
    std::uint16_t pid_{0};
    std::uint8_t firmware_{0};
    int width_{0};
    int height_{0};
    std::string model_name_{"DAVIS346"};
    float logic_clock_{0};
    float usb_clock_{0};
    Parser parser_;

    // Decouples the per-batch event pipeline from the USB reaping thread
    // (IMU/APS stay inline in the parser — latency-critical). Started with
    // the USB thread, stopped after it (producer-first shutdown).
    BatchWorker batches_;
    BiasStore biases_;
    EventSink sink_;

    /// Synchronous consumer for the decoded event batch, invoked on the USB
    /// decode thread BEFORE the batch is queued for the processing worker —
    /// the AEDAT4 recorder records here so queue overflow (drop-oldest)
    /// can never cost recording data.
    std::function<void(const Metavision::EventCD*, const Metavision::EventCD*)> raw_consumer_;
    GoneCallback gone_callback_;
    std::atomic<bool> streaming_{false};
    bool imu_enabled_{false};
    bool aps_enabled_{false};
    bool has_roi_filter_{false};
    int chip_model_{5};
    int dvs_orientation_{0};
    int aps_orientation_{0};
    float adc_clock_{0};
    bool auto_exposure_{true};
    double aec_exposure_us_{0};
    ApsFrameSink user_aps_sink_;
};

} // namespace gui::davis

#endif // GUI_DAVIS_DAVIS_DEVICE_H
