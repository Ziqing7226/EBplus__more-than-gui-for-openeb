// gui/davis/dvxplorer_device.h — live inivation DVXplorer camera over libusb.
//
// Ported from dv-processing 2.0.4 io/camera/dvxplorer.hpp (Apache-2.0),
// events-only: the IMU and trigger streams are parsed and discarded. The
// DVXplorer uses the same inivation USB transport family as the DAVIS
// (vendor request 0xBF SPI config), but the sensor chip (Samsung) has its
// own register map, a ~170-write init sequence, mgroup-compressed event
// encoding (8-pixel groups), and only two user-adjustable "bias" parameters
// (the ON/OFF contrast thresholds).
//
// The event wire format (16-bit LE words):
//   bit 15 set → timestamp: wrapAdd + (word & 0x7FFF), 1 µs ticks
//   code (bits 14-12):
//     0 → special (data 1 = timestamp reset)
//     1 → X column address latch (10 bits; 1023 = reset marker)
//     2 → 8-pixel group from Y-group 2 (bit 8: 0 = ON, 1 = OFF)
//     3 → 8-pixel group from Y-group 1
//     4 → Y-group address latch (two groups, base ×8, ±offset)
//     5/6 → IMU/misc (consumed, ignored)
//     7 → timestamp wrap (wrapAdd += 0x8000 × data)
// The stream is rebased to start at 0 after each device timestamp reset.

#ifndef GUI_DAVIS_DVXPLORER_DEVICE_H
#define GUI_DAVIS_DVXPLORER_DEVICE_H

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

#include <metavision/sdk/base/events/event_cd.h>

#include "davis_device.h"
#include "batch_worker.h"
#include "dvxplorer_parser.h"

namespace gui::davis {

struct DeviceDescriptor; // fwd from davis side

/// Enumerates connected inivation DVXplorer cameras (without opening them).
std::vector<DeviceDescriptor> find_dvx_devices();

class DvxplorerDevice {
public:
    /// Sink invoked from the libusb thread with decoded, 0-based events.
    using EventSink = std::function<void(const Metavision::EventCD*, const Metavision::EventCD*)>;
    /// Invoked (libusb thread) when the device disappears unexpectedly.
    using GoneCallback = std::function<void()>;

    explicit DvxplorerDevice(const DeviceDescriptor& descriptor);
    ~DvxplorerDevice();

    DvxplorerDevice(const DvxplorerDevice&) = delete;
    DvxplorerDevice& operator=(const DvxplorerDevice&) = delete;

    void set_event_sink(EventSink sink);
    /// Recording tap (see davis_device.h raw_consumer_).
    void set_raw_consumer(std::function<void(const Metavision::EventCD*, const Metavision::EventCD*)> cb) {
        std::lock_guard<std::mutex> lock(raw_consumer_mutex_);
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
    void start();
    void stop();

    [[nodiscard]] int width() const { return width_; }
    [[nodiscard]] int height() const { return height_; }
    [[nodiscard]] const std::string& serial() const { return serial_; }
    [[nodiscard]] const std::string& model_name() const { return model_name_; }

    /// ON/OFF contrast thresholds (0–17 each) — the only user-adjustable
    /// DVXplorer "bias" parameters. Read/write through the local mirror +
    /// SPI (safe from the GUI thread while streaming).
    [[nodiscard]] int contrast_on() const { return contrast_on_; }
    [[nodiscard]] int contrast_off() const { return contrast_off_; }
    void set_contrast_on(int value);
    void set_contrast_off(int value);

private:
    // USB primitives (nextgen: SPI value is 8 bytes, no multiple-config).
    void usb_control_out(std::uint8_t request, std::uint16_t value, std::uint16_t index,
                         const std::uint8_t* data, std::size_t size);
    void usb_control_in(std::uint8_t request, std::uint16_t value, std::uint16_t index,
                        std::uint8_t* data, std::size_t size);
    void spi_config_send(std::uint8_t module, std::uint16_t param, std::uint64_t value);
    std::uint64_t spi_config_receive(std::uint8_t module, std::uint16_t param);
    void usb_cleanup_buffers();
    void usb_thread_start();
    void usb_thread_stop();
    void usb_data_transfers_start();
    void usb_data_transfers_stop();
    void teardown_usb();

    // DVXplorer configuration (ported from dvxplorer.hpp).
    void configure_idle();
    /// Full Samsung chip init: known-state shutdown, chip out of reset, the
    /// whole register sequence. Called from the constructor and replayed by
    /// start() (stop() parks the chip in reset).
    void chip_init();
    void send_timestamp_reset();
    bool wait_for_timestamp_reset();
    void set_contrast_threshold(int on_value, int off_value);
    void set_global_hold(bool on);
    void set_global_reset(bool on);
    void set_flatten_none();
    void set_subsample_every_pixel();
    void set_readout_fps_variable_5000();
    void set_crop_bypass();
    void parse_events(const std::uint8_t* data, std::size_t size);

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

    // DVXplorer state.
    std::uint8_t firmware_{0};
    int width_{0};
    int height_{0};
    std::string model_name_{"DVXplorer"};
    float logic_clock_{0};
    float usb_clock_{0};
    int contrast_on_{9};
    int contrast_off_{9};
    EventSink sink_;

    /// Synchronous pre-queue consumer — see davis_device.h (recording tap).
    std::function<void(const Metavision::EventCD*, const Metavision::EventCD*)> raw_consumer_;
    mutable std::mutex raw_consumer_mutex_;

    // Decouples the per-batch event pipeline from the USB reaping thread
    // (IMU stays inline in the parser — latency-critical). Started with
    // the USB thread, stopped after it (producer-first shutdown).
    BatchWorker batches_;
    GoneCallback gone_callback_;
    bool imu_enabled_{false};
    DvxParser parse_;
    std::atomic<bool> streaming_{false};
};

} // namespace gui::davis

#endif // GUI_DAVIS_DVXPLORER_DEVICE_H
