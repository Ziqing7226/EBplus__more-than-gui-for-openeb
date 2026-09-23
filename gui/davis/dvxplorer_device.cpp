// gui/davis/dvxplorer_device.cpp — see dvxplorer_device.h.
// Ported from dv-processing 2.0.4 io/camera/dvxplorer.hpp (Apache-2.0),
// events-only: the IMU and trigger streams are configured but not enabled,
// debug-endpoint transfers are skipped, and the event stream is rebased to
// start at 0 after each device timestamp reset.

#include "dvxplorer_device.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <thread>

namespace gui::davis {
namespace {

constexpr std::uint16_t VID_INIVATION = 0x152A;
constexpr std::uint16_t PID_DVXPLORER = 0x8419;
constexpr std::uint8_t USB_DEVICE_TYPE_FX3_RED = 3;
constexpr std::uint8_t FIRMWARE_REQUIRED = 9;
constexpr std::uint32_t LOGIC_REQUIRED = 18;
constexpr std::uint32_t LOGIC_MINIMUM_PATCH = 4;

constexpr std::uint8_t VENDOR_REQUEST_SPI_CONFIG = 0xBF;
constexpr std::uint8_t VENDOR_REQUEST_DATA_CLEANUP = 0xC6;
constexpr std::uint8_t DATA_ENDPOINT = 0x82;
constexpr std::uint32_t DATA_TRANSFERS_NUMBER = 32;
constexpr std::uint32_t DATA_TRANSFERS_SIZE = 8 * 1024;

// Register addresses (dv-processing dvxplorer.hpp constants).
constexpr std::uint8_t MODULE_MULTIPLEXER = 0;
constexpr std::uint8_t MODULE_DVS = 1;
constexpr std::uint8_t MODULE_IMU = 3;
constexpr std::uint8_t MODULE_EXTERNAL_INPUT = 4;
constexpr std::uint8_t MODULE_DEVICE = 5;
constexpr std::uint8_t MODULE_SYSINFO = 6;
constexpr std::uint8_t MODULE_USB = 9;
constexpr std::uint16_t MUX_RUN = 0;
constexpr std::uint16_t MUX_TIMESTAMP_RUN = 1;
constexpr std::uint16_t MUX_TIMESTAMP_RESET = 2;
constexpr std::uint16_t MUX_RUN_CHIP = 3;
constexpr std::uint16_t MUX_DROP_EXTINPUT_ON_TRANSFER_STALL = 4;
constexpr std::uint16_t MUX_DROP_DVS_ON_TRANSFER_STALL = 5;
constexpr std::uint16_t DVS_SIZE_COLUMNS = 0;
constexpr std::uint16_t DVS_SIZE_ROWS = 1;
constexpr std::uint16_t DVS_ORIENTATION_INFO = 2;
constexpr std::uint16_t DVS_RUN = 3;
constexpr std::uint16_t IMU_TYPE = 0;
constexpr std::uint16_t IMU_ORIENTATION_INFO = 1;
constexpr std::uint16_t IMU_RUN_ACCELEROMETER = 2;
constexpr std::uint16_t IMU_RUN_GYROSCOPE = 3;
constexpr std::uint16_t IMU_RUN_TEMPERATURE = 4;
// (No IMU_SAMPLE_RATE_DIVIDER on this chip — address 5 is ACCEL_DATA_RATE.)
constexpr std::uint16_t IMU_ACCEL_DATA_RATE = 5;
constexpr std::uint16_t IMU_ACCEL_FILTER = 6;
constexpr std::uint16_t IMU_ACCEL_RANGE = 7;
constexpr std::uint16_t IMU_GYRO_DATA_RATE = 8;
constexpr std::uint16_t IMU_GYRO_FILTER = 9;
constexpr std::uint16_t IMU_GYRO_RANGE = 10;
constexpr std::uint16_t EXTINPUT_DETECT_RISING_EDGES = 1;
constexpr std::uint16_t EXTINPUT_DETECT_FALLING_EDGES = 2;
constexpr std::uint16_t EXTINPUT_GENERATE_PULSE_INTERVAL = 13;
constexpr std::uint16_t EXTINPUT_GENERATE_PULSE_LENGTH = 14;
constexpr std::uint16_t EXTINPUT_GENERATE_INJECT_ON_RISING_EDGE = 15;
constexpr std::uint16_t EXTINPUT_GENERATE_INJECT_ON_FALLING_EDGE = 16;
constexpr std::uint16_t SYSINFO_LOGIC_VERSION = 0;
constexpr std::uint16_t SYSINFO_CHIP_IDENTIFIER = 1;
constexpr std::uint16_t SYSINFO_DEVICE_IS_MASTER = 2;
constexpr std::uint16_t SYSINFO_LOGIC_CLOCK = 3;
constexpr std::uint16_t SYSINFO_USB_CLOCK = 5;
constexpr std::uint16_t SYSINFO_CLOCK_DEVIATION = 6;
constexpr std::uint16_t SYSINFO_LOGIC_PATCH = 7;
constexpr std::uint16_t USB_RUN = 0;
constexpr std::uint16_t USB_EARLY_PACKET_DELAY = 1;
constexpr std::uint16_t REGISTER_BIAS_CURRENT_RANGE_SELECT_LOGSFONREST = 0x000B;
constexpr std::uint16_t REGISTER_BIAS_CURRENT_RANGE_SELECT_LOGALOGD_MONITOR = 0x000C;
constexpr std::uint16_t REGISTER_BIAS_OTP_TRIM = 0x000D;
constexpr std::uint16_t REGISTER_BIAS_CURRENT_LEVEL_SFOFF = 0x0012;
constexpr std::uint16_t REGISTER_BIAS_PINS_BUFP = 0x0013;
constexpr std::uint16_t REGISTER_BIAS_PINS_BUFN = 0x0014;
constexpr std::uint16_t REGISTER_BIAS_PINS_DOB = 0x0015;
constexpr std::uint16_t REGISTER_BIAS_CURRENT_AMP = 0x0018;
constexpr std::uint16_t REGISTER_BIAS_CURRENT_ON = 0x001C;
constexpr std::uint16_t REGISTER_BIAS_CURRENT_OFF = 0x001E;
constexpr std::uint16_t REGISTER_CONTROL_MODE = 0x3000;
constexpr std::uint16_t REGISTER_CONTROL_CLOCK_DIVIDER_SYS = 0x3011;
constexpr std::uint16_t REGISTER_CONTROL_PARALLEL_OUT_CONTROL = 0x3019;
constexpr std::uint16_t REGISTER_CONTROL_PARALLEL_OUT_ENABLE = 0x301E;
constexpr std::uint16_t REGISTER_CONTROL_PACKET_FORMAT = 0x3067;
constexpr std::uint16_t REGISTER_DIGITAL_ENABLE = 0x3200;
constexpr std::uint16_t REGISTER_DIGITAL_RESTART = 0x3201;
constexpr std::uint16_t REGISTER_DIGITAL_DUAL_BINNING = 0x3202;
constexpr std::uint16_t REGISTER_DIGITAL_SUBSAMPLE_RATIO = 0x3204;
constexpr std::uint16_t REGISTER_DIGITAL_TIMESTAMP_SUBUNIT = 0x3234;
constexpr std::uint16_t REGISTER_DIGITAL_TIMESTAMP_REFUNIT = 0x3235;
constexpr std::uint16_t REGISTER_DIGITAL_TIMESTAMP_RESET = 0x3238;
constexpr std::uint16_t REGISTER_DIGITAL_DTAG_REFERENCE = 0x323D;
constexpr std::uint16_t REGISTER_TIMING_FIRST_SELX_START = 0x323C;
constexpr std::uint16_t REGISTER_TIMING_GH_COUNT = 0x3240;
constexpr std::uint16_t REGISTER_TIMING_GH_COUNT_FINE = 0x3243;
constexpr std::uint16_t REGISTER_TIMING_GRS_COUNT = 0x3244;
constexpr std::uint16_t REGISTER_TIMING_GRS_COUNT_FINE = 0x3247;
constexpr std::uint16_t REGISTER_DIGITAL_GLOBAL_RESET_READOUT = 0x3248;
constexpr std::uint16_t REGISTER_TIMING_NEXT_GH_CNT = 0x324B;
constexpr std::uint16_t REGISTER_TIMING_SELX_WIDTH = 0x324C;
constexpr std::uint16_t REGISTER_TIMING_AY_START = 0x324E;
constexpr std::uint16_t REGISTER_TIMING_AY_END = 0x324F;
constexpr std::uint16_t REGISTER_TIMING_MAX_EVENT_NUM = 0x3251;
constexpr std::uint16_t REGISTER_TIMING_R_START = 0x3253;
constexpr std::uint16_t REGISTER_TIMING_R_END = 0x3254;
constexpr std::uint16_t REGISTER_DIGITAL_MODE_CONTROL = 0x3255;
constexpr std::uint16_t REGISTER_TIMING_GRS_END = 0x3256;
constexpr std::uint16_t REGISTER_TIMING_GRS_END_FINE = 0x3259;
constexpr std::uint16_t REGISTER_DIGITAL_FIXED_READ_TIME = 0x325C;
constexpr std::uint16_t REGISTER_TIMING_READ_TIME_INTERVAL = 0x325D;
constexpr std::uint16_t REGISTER_DIGITAL_EXTERNAL_TRIGGER = 0x3260;
constexpr std::uint16_t REGISTER_TIMING_NEXT_SELX_START = 0x3261;
constexpr std::uint16_t REGISTER_DIGITAL_BOOT_SEQUENCE = 0x3266;
constexpr std::uint16_t REGISTER_CROPPER_BYPASS = 0x3300;
constexpr std::uint16_t REGISTER_ACTIVITY_DECISION_BYPASS = 0x3500;
constexpr std::uint16_t REGISTER_SPATIAL_HISTOGRAM_OFF = 0x3600;
// Samsung DVS chip operating modes (REGISTER_CONTROL_MODE values).
constexpr std::uint16_t DVS_CHIP_MODE_STREAM = 2;
constexpr std::uint16_t DVS_CHIP_DTAG_CONTROL_RESTART = 2;
constexpr std::uint16_t SYSTEM_CLOCK_FREQUENCY = 50;

} // namespace

// ---------------------------------------------------------------------------

std::vector<DeviceDescriptor> find_dvx_devices() {
    std::vector<DeviceDescriptor> found;
    libusb_context* ctx = nullptr;
    if (libusb_init(&ctx) != LIBUSB_SUCCESS) return found;

    libusb_device** list = nullptr;
    const ssize_t count = libusb_get_device_list(ctx, &list);
    if (count >= 0) {
        for (ssize_t i = 0; i < count; ++i) {
            libusb_device_descriptor desc{};
            if (libusb_get_device_descriptor(list[i], &desc) != LIBUSB_SUCCESS) continue;
            if (desc.idVendor != VID_INIVATION || desc.idProduct != PID_DVXPLORER) continue;
            const auto device_type = static_cast<std::uint8_t>((desc.bcdDevice >> 8) & 0xFF);
            if (device_type != USB_DEVICE_TYPE_FX3_RED) continue; // DVXplorer (FX3 red)

            DeviceDescriptor info;
            info.vid = desc.idVendor;
            info.pid = desc.idProduct;
            info.bus = libusb_get_bus_number(list[i]);
            info.addr = libusb_get_device_address(list[i]);
            info.firmware = static_cast<std::uint8_t>(desc.bcdDevice & 0xFF);

            libusb_device_handle* handle = nullptr;
            if (libusb_open(list[i], &handle) == LIBUSB_SUCCESS) {
                std::string serial(9, '\0');
                auto got = libusb_get_string_descriptor_ascii(handle, desc.iSerialNumber,
                    reinterpret_cast<unsigned char*>(serial.data()), static_cast<int>(serial.size()));
                if (got > 8) got = 8;
                if (got > 0) info.serial = serial.substr(0, static_cast<std::size_t>(got));
                libusb_close(handle);
            }
            if (info.serial.empty()) {
                char tmp[16];
                std::snprintf(tmp, sizeof(tmp), "TMP%05d", (info.bus << 8) | info.addr);
                info.serial = tmp;
            }
            found.push_back(std::move(info));
        }
        libusb_free_device_list(list, 1);
    }

    libusb_exit(ctx);
    return found;
}

// ---------------------------------------------------------------------------

DvxplorerDevice::DvxplorerDevice(const DeviceDescriptor& descriptor)
    : parse_(0, 0) {
    libusb_context* ctx = nullptr;
    if (libusb_init(&ctx) != LIBUSB_SUCCESS) {
        throw std::runtime_error("DVXplorer: failed to initialize libusb.");
    }
    context_ = ctx;

    libusb_device** list = nullptr;
    const ssize_t count = libusb_get_device_list(context_, &list);
    if (count < 0) {
        libusb_exit(context_);
        context_ = nullptr;
        throw std::runtime_error("DVXplorer: failed to list USB devices.");
    }
    for (ssize_t i = 0; i < count && handle_ == nullptr; ++i) {
        libusb_device_descriptor desc{};
        if (libusb_get_device_descriptor(list[i], &desc) != LIBUSB_SUCCESS) continue;
        if (desc.idVendor != descriptor.vid || desc.idProduct != descriptor.pid) continue;
        if (libusb_get_bus_number(list[i]) != descriptor.bus) continue;
        if (libusb_get_device_address(list[i]) != descriptor.addr) continue;
        libusb_device_handle* handle = nullptr;
        if (libusb_open(list[i], &handle) != LIBUSB_SUCCESS) {
            libusb_free_device_list(list, 1);
            libusb_exit(context_);
            context_ = nullptr;
            throw std::runtime_error(
                "DVXplorer: failed to open USB device — check permissions (udev rules) "
                "and that no other program is using the camera.");
        }
        handle_ = handle;
        firmware_ = static_cast<std::uint8_t>(desc.bcdDevice & 0xFF);
        serial_ = descriptor.serial;
    }
    libusb_free_device_list(list, 1);

    if (handle_ == nullptr) {
        libusb_exit(context_);
        context_ = nullptr;
        throw std::runtime_error("DVXplorer: device disappeared during open.");
    }

    int active_config = 0;
    if (libusb_get_configuration(handle_, &active_config) != LIBUSB_SUCCESS) {
        teardown_usb();
        throw std::runtime_error("DVXplorer: failed to get USB configuration.");
    }
    if (active_config != 1 && libusb_set_configuration(handle_, 1) != LIBUSB_SUCCESS) {
        teardown_usb();
        throw std::runtime_error("DVXplorer: failed to set USB configuration 1.");
    }
    if (libusb_claim_interface(handle_, 0) != LIBUSB_SUCCESS) {
        teardown_usb();
        throw std::runtime_error("DVXplorer: failed to claim USB interface 0 — the camera may be in use.");
    }

    usb_thread_start();
    batches_.start([this]() -> BatchWorker::Sink { return sink_; });
    usb_cleanup_buffers();

    try {
        configure_idle();
    } catch (...) {
        batches_.stop();
        usb_thread_stop();
        libusb_release_interface(handle_, 0);
        teardown_usb();
        throw;
    }
}

DvxplorerDevice::~DvxplorerDevice() {
    try {
        stop();
    } catch (...) {
    }
    if (usb_thread_run_.load()) usb_thread_stop();
    // After the USB thread is gone no new batches can be submitted —
    // drain and join the processing worker.
    batches_.stop();
    if (handle_ != nullptr) libusb_release_interface(handle_, 0);
    teardown_usb();
}

void DvxplorerDevice::teardown_usb() {
    if (handle_ != nullptr) {
        libusb_close(handle_);
        handle_ = nullptr;
    }
    if (context_ != nullptr) {
        libusb_exit(context_);
        context_ = nullptr;
    }
}

void DvxplorerDevice::set_event_sink(EventSink sink) {
    sink_ = std::move(sink);
}

void DvxplorerDevice::set_gone_callback(GoneCallback callback) {
    gone_callback_ = std::move(callback);
}

// --- USB primitives ---------------------------------------------------------

void DvxplorerDevice::usb_control_out(std::uint8_t request, std::uint16_t value, std::uint16_t index,
                                      const std::uint8_t* data, std::size_t size) {
    std::atomic<int> completed{-1};
    libusb_transfer* transfer = libusb_alloc_transfer(0);
    if (transfer == nullptr) throw std::runtime_error("DVXplorer: out of memory (control transfer).");

    std::vector<std::uint8_t> buffer(sizeof(libusb_control_setup) + size, 0);
    libusb_fill_control_setup(buffer.data(), LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR |
                                                  LIBUSB_RECIPIENT_DEVICE,
        request, value, index, static_cast<std::uint16_t>(size));
    if (size > 0 && data != nullptr) {
        std::memcpy(buffer.data() + sizeof(libusb_control_setup), data, size);
    }

    libusb_fill_control_transfer(transfer, handle_, buffer.data(),
        [](libusb_transfer* t) {
            static_cast<std::atomic<int>*>(t->user_data)->store(t->status);
        },
        &completed, 0);

    if (libusb_submit_transfer(transfer) != LIBUSB_SUCCESS) {
        libusb_free_transfer(transfer);
        throw std::runtime_error("DVXplorer: failed to submit control transfer OUT.");
    }
    while (completed.load() < 0) {
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    libusb_free_transfer(transfer);
    if (completed.load() != LIBUSB_TRANSFER_COMPLETED) {
        throw std::runtime_error("DVXplorer: control transfer OUT failed (libusb: " +
                                 std::string(libusb_error_name(completed.load())) + ").");
    }
}

void DvxplorerDevice::usb_control_in(std::uint8_t request, std::uint16_t value, std::uint16_t index,
                                     std::uint8_t* data, std::size_t size) {
    std::atomic<int> completed{-1};
    libusb_transfer* transfer = libusb_alloc_transfer(0);
    if (transfer == nullptr) throw std::runtime_error("DVXplorer: out of memory (control transfer).");

    std::vector<std::uint8_t> buffer(sizeof(libusb_control_setup) + size, 0);
    libusb_fill_control_setup(buffer.data(), LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR |
                                                  LIBUSB_RECIPIENT_DEVICE,
        request, value, index, static_cast<std::uint16_t>(size));

    libusb_fill_control_transfer(transfer, handle_, buffer.data(),
        [](libusb_transfer* t) {
            static_cast<std::atomic<int>*>(t->user_data)->store(t->status);
        },
        &completed, 0);

    if (libusb_submit_transfer(transfer) != LIBUSB_SUCCESS) {
        libusb_free_transfer(transfer);
        throw std::runtime_error("DVXplorer: failed to submit control transfer IN.");
    }
    while (completed.load() < 0) {
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    const bool ok = (completed.load() == LIBUSB_TRANSFER_COMPLETED);
    if (ok && data != nullptr && size > 0) {
        std::memcpy(data, buffer.data() + sizeof(libusb_control_setup), size);
    }
    libusb_free_transfer(transfer);
    if (!ok) {
        throw std::runtime_error("DVXplorer: control transfer IN failed (libusb: " +
                                 std::string(libusb_error_name(completed.load())) + ").");
    }
}

// SPI values are 4-byte big-endian on this camera generation (verified on
// hardware: 8-byte reads STALL, libcaer-style 4-byte reads return valid data).
void DvxplorerDevice::spi_config_send(std::uint8_t module, std::uint16_t param, std::uint64_t value) {
    const auto v = static_cast<std::uint32_t>(value);
    const std::uint8_t be[4] = {static_cast<std::uint8_t>(v >> 24), static_cast<std::uint8_t>(v >> 16),
        static_cast<std::uint8_t>(v >> 8), static_cast<std::uint8_t>(v)};
    usb_control_out(VENDOR_REQUEST_SPI_CONFIG, module, param, be, 4);
}

std::uint64_t DvxplorerDevice::spi_config_receive(std::uint8_t module, std::uint16_t param) {
    std::uint8_t be[4] = {0, 0, 0, 0};
    usb_control_in(VENDOR_REQUEST_SPI_CONFIG, module, param, be, 4);
    return (static_cast<std::uint64_t>(be[0]) << 24) | (static_cast<std::uint64_t>(be[1]) << 16) |
           (static_cast<std::uint64_t>(be[2]) << 8) | static_cast<std::uint64_t>(be[3]);
}

void DvxplorerDevice::usb_cleanup_buffers() {
    try {
        usb_control_out(VENDOR_REQUEST_DATA_CLEANUP, 0, 0, nullptr, 0);
    } catch (const std::exception&) {
        libusb_clear_halt(handle_, data_endpoint_);
    }
}

// --- USB thread + data transfers ---------------------------------------------

void DvxplorerDevice::usb_thread_start() {
    std::scoped_lock lock(usb_ops_lock_);
    if (usb_thread_run_.load()) throw std::runtime_error("DVXplorer: USB thread already running.");
    usb_thread_ = std::thread([this]() {
        usb_thread_run_.store(true);
        timeval timeout = {0, 10000};
        while (usb_thread_run_.load()) {
            libusb_handle_events_timeout_completed(context_, &timeout, nullptr);
        }
    });
    while (!usb_thread_run_.load()) {
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
}

void DvxplorerDevice::usb_thread_stop() {
    std::scoped_lock lock(usb_ops_lock_);
    if (!usb_thread_run_.load()) return;
    usb_thread_run_.store(false);
    if (usb_thread_.joinable()) usb_thread_.join();
}

void LIBUSB_CALL DvxplorerDevice::usb_data_transfer_cb(libusb_transfer* transfer) {
    auto* self = static_cast<DvxplorerDevice*>(transfer->user_data);

    if ((transfer->status == LIBUSB_TRANSFER_COMPLETED ||
            transfer->status == LIBUSB_TRANSFER_CANCELLED) &&
        transfer->actual_length > 0) {
        self->parse_events(transfer->buffer, static_cast<std::size_t>(transfer->actual_length));
    }

    if (transfer->status == LIBUSB_TRANSFER_COMPLETED) {
        if (libusb_submit_transfer(transfer) == LIBUSB_SUCCESS) return;
    }

    {
        std::scoped_lock lock(self->data_transfers_lock_);
        self->data_transfers_active_--;
        const bool failed_not_cancelled = (transfer->status != LIBUSB_TRANSFER_CANCELLED);
        if (self->data_transfers_active_ == 0 && failed_not_cancelled) {
            if (self->gone_callback_) self->gone_callback_();
        }
    }
}

void DvxplorerDevice::usb_data_transfers_start() {
    std::scoped_lock lock(usb_ops_lock_);
    std::scoped_lock data_lock(data_transfers_lock_);
    if (data_transfers_active_ > 0) return;
    for (std::uint32_t i = 0; i < DATA_TRANSFERS_NUMBER; ++i) {
        libusb_transfer* transfer = libusb_alloc_transfer(0);
        if (transfer == nullptr) break;
        auto* buffer = static_cast<std::uint8_t*>(malloc(DATA_TRANSFERS_SIZE));
        if (buffer == nullptr) {
            libusb_free_transfer(transfer);
            break;
        }
        transfer->buffer = buffer;
        transfer->length = static_cast<int>(DATA_TRANSFERS_SIZE);
        transfer->dev_handle = handle_;
        transfer->endpoint = data_endpoint_;
        transfer->type = LIBUSB_TRANSFER_TYPE_BULK;
        transfer->callback = &usb_data_transfer_cb;
        transfer->user_data = this;
        transfer->timeout = 0;
        transfer->flags = LIBUSB_TRANSFER_FREE_BUFFER;
        if (libusb_submit_transfer(transfer) != LIBUSB_SUCCESS) {
            free(buffer);
            libusb_free_transfer(transfer);
            continue;
        }
        data_transfers_.push_back(transfer);
    }
    data_transfers_active_ = static_cast<std::uint32_t>(data_transfers_.size());
    if (data_transfers_.empty()) {
        throw std::runtime_error("DVXplorer: unable to allocate any USB data transfers.");
    }
}

void DvxplorerDevice::usb_data_transfers_stop() {
    // Keep cancelling until every transfer is gone — completed transfers are
    // re-submitted by their callbacks and would otherwise live forever.
    std::scoped_lock ops_lock(usb_ops_lock_);
    while (true) {
        {
            std::scoped_lock data_lock(data_transfers_lock_);
            if (data_transfers_active_ == 0 || data_transfers_.empty()) break;
            for (auto* transfer : data_transfers_) {
                libusb_cancel_transfer(transfer);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::scoped_lock data_lock(data_transfers_lock_);
    for (auto* transfer : data_transfers_) {
        libusb_free_transfer(transfer);
    }
    data_transfers_.clear();
    data_transfers_active_ = 0;
}

// --- DVXplorer configuration ----------------------------------------------------

void DvxplorerDevice::configure_idle() {
    // Firmware/logic version checks (reference hard-fails on mismatch).
    if (firmware_ != FIRMWARE_REQUIRED) {
        throw std::runtime_error("DVXplorer: unsupported USB firmware version " +
                                 std::to_string(firmware_) + " (expected " +
                                 std::to_string(FIRMWARE_REQUIRED) + "). Update the camera with Flashy.");
    }
    const auto logic_version = spi_config_receive(MODULE_SYSINFO, SYSINFO_LOGIC_VERSION);
    if (logic_version != LOGIC_REQUIRED) {
        throw std::runtime_error("DVXplorer: unsupported FPGA logic version " +
                                 std::to_string(logic_version) + " (expected " +
                                 std::to_string(LOGIC_REQUIRED) + "). Update the camera with Flashy.");
    }
    const auto logic_patch = spi_config_receive(MODULE_SYSINFO, SYSINFO_LOGIC_PATCH);
    if (logic_patch < LOGIC_MINIMUM_PATCH) {
        throw std::runtime_error("DVXplorer: FPGA logic patch " + std::to_string(logic_patch) +
                                 " is too old (minimum " + std::to_string(LOGIC_MINIMUM_PATCH) + ").");
    }

    // Resolution + orientation.
    const auto columns = spi_config_receive(MODULE_DVS, DVS_SIZE_COLUMNS);
    const auto rows = spi_config_receive(MODULE_DVS, DVS_SIZE_ROWS);
    const auto orientation = spi_config_receive(MODULE_DVS, DVS_ORIENTATION_INFO);
    width_ = static_cast<int>(columns);
    height_ = static_cast<int>(rows);
    if ((orientation & 0x04) != 0) std::swap(width_, height_);
    parse_ = DvxParser(columns, rows);

    // Clocks.
    const auto logic_clock = static_cast<double>(spi_config_receive(MODULE_SYSINFO, SYSINFO_LOGIC_CLOCK));
    const auto usb_clock = static_cast<double>(spi_config_receive(MODULE_SYSINFO, SYSINFO_USB_CLOCK));
    const auto deviation = static_cast<double>(spi_config_receive(MODULE_SYSINFO, SYSINFO_CLOCK_DEVIATION));
    logic_clock_ = static_cast<float>(logic_clock * (deviation / 1000.0));
    usb_clock_ = static_cast<float>(usb_clock * (deviation / 1000.0));

    chip_init();
}

void DvxplorerDevice::chip_init() {
    // Known-state shutdown before configuring (mirrors the reference
    // destructor): stop every stream and put the DVS chip back in reset.
    spi_config_send(MODULE_DVS, DVS_RUN, false);
    spi_config_send(MODULE_IMU, IMU_RUN_ACCELEROMETER, false);
    spi_config_send(MODULE_IMU, IMU_RUN_GYROSCOPE, false);
    spi_config_send(MODULE_IMU, IMU_RUN_TEMPERATURE, false);
    spi_config_send(MODULE_MULTIPLEXER, MUX_RUN, false);
    spi_config_send(MODULE_MULTIPLEXER, MUX_TIMESTAMP_RUN, false);
    spi_config_send(MODULE_USB, USB_RUN, false);
    spi_config_send(MODULE_MULTIPLEXER, MUX_RUN_CHIP, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    usb_cleanup_buffers();

    // Take the DVS out of reset, then run the Samsung chip init sequence
    // (ported verbatim from the reference constructor).
    spi_config_send(MODULE_MULTIPLEXER, MUX_RUN_CHIP, true);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    spi_config_send(MODULE_DEVICE, REGISTER_BIAS_OTP_TRIM, 0x24);
    spi_config_send(MODULE_DEVICE, REGISTER_BIAS_PINS_BUFP, 0x03);
    spi_config_send(MODULE_DEVICE, REGISTER_BIAS_PINS_BUFN, 0x7F);
    spi_config_send(MODULE_DEVICE, REGISTER_BIAS_PINS_DOB, false);
    spi_config_send(MODULE_DEVICE, REGISTER_BIAS_CURRENT_RANGE_SELECT_LOGSFONREST, 0x04);
    spi_config_send(MODULE_DEVICE, REGISTER_BIAS_CURRENT_RANGE_SELECT_LOGALOGD_MONITOR, 0x14);
    spi_config_send(MODULE_DEVICE, REGISTER_BIAS_CURRENT_LEVEL_SFOFF, 0x7D);
    spi_config_send(MODULE_DEVICE, REGISTER_BIAS_CURRENT_AMP, 4);
    // Contrast thresholds: write the mirror values (reference defaults 9/9
    // on first open; a restart re-applies whatever the user last set —
    // stop()/start() must not silently reset the panel's live settings).
    set_contrast_threshold(contrast_on_, contrast_off_);
    spi_config_send(MODULE_DEVICE, REGISTER_CONTROL_CLOCK_DIVIDER_SYS, 0xA0);
    spi_config_send(MODULE_DEVICE, REGISTER_CONTROL_PARALLEL_OUT_CONTROL, 0x00);
    spi_config_send(MODULE_DEVICE, REGISTER_CONTROL_PARALLEL_OUT_ENABLE, 0x01);
    spi_config_send(MODULE_DEVICE, REGISTER_CONTROL_PACKET_FORMAT, 0x80); // mgroup compression
    spi_config_send(MODULE_DEVICE, REGISTER_DIGITAL_MODE_CONTROL, 0x0C);
    spi_config_send(MODULE_DEVICE, REGISTER_DIGITAL_BOOT_SEQUENCE, 0x08);
    spi_config_send(MODULE_DEVICE, REGISTER_DIGITAL_TIMESTAMP_REFUNIT, 0x03);
    spi_config_send(MODULE_DEVICE, REGISTER_DIGITAL_TIMESTAMP_REFUNIT + 1, 0xE7);
    spi_config_send(MODULE_DEVICE, REGISTER_DIGITAL_TIMESTAMP_SUBUNIT, SYSTEM_CLOCK_FREQUENCY - 1);
    spi_config_send(MODULE_DEVICE, REGISTER_DIGITAL_DTAG_REFERENCE, SYSTEM_CLOCK_FREQUENCY);
    spi_config_send(MODULE_DEVICE, REGISTER_TIMING_GH_COUNT_FINE, SYSTEM_CLOCK_FREQUENCY);
    spi_config_send(MODULE_DEVICE, REGISTER_TIMING_GRS_COUNT_FINE, SYSTEM_CLOCK_FREQUENCY);
    spi_config_send(MODULE_DEVICE, REGISTER_TIMING_GRS_END_FINE, SYSTEM_CLOCK_FREQUENCY);
    spi_config_send(MODULE_DEVICE, REGISTER_TIMING_GRS_COUNT, 0);
    spi_config_send(MODULE_DEVICE, REGISTER_TIMING_GRS_COUNT + 1, 0);
    spi_config_send(MODULE_DEVICE, REGISTER_TIMING_GRS_COUNT + 2, 0);
    spi_config_send(MODULE_DEVICE, REGISTER_TIMING_GRS_END, 0);
    spi_config_send(MODULE_DEVICE, REGISTER_TIMING_GRS_END + 1, 0);
    spi_config_send(MODULE_DEVICE, REGISTER_TIMING_GRS_END + 2, 1);
    spi_config_send(MODULE_DEVICE, REGISTER_TIMING_FIRST_SELX_START, 4);
    spi_config_send(MODULE_DEVICE, REGISTER_TIMING_SELX_WIDTH, 6);
    spi_config_send(MODULE_DEVICE, REGISTER_TIMING_AY_START, 4);
    spi_config_send(MODULE_DEVICE, REGISTER_TIMING_AY_END, 6);
    spi_config_send(MODULE_DEVICE, REGISTER_TIMING_R_START, 8);
    spi_config_send(MODULE_DEVICE, REGISTER_TIMING_R_END, 10);
    spi_config_send(MODULE_DEVICE, REGISTER_TIMING_NEXT_GH_CNT, 4);
    spi_config_send(MODULE_DEVICE, REGISTER_ACTIVITY_DECISION_BYPASS, 0x01);
    spi_config_send(MODULE_DEVICE, REGISTER_SPATIAL_HISTOGRAM_OFF, 0x01);
    spi_config_send(MODULE_DEVICE, REGISTER_DIGITAL_EXTERNAL_TRIGGER, 0);
    spi_config_send(MODULE_DEVICE, REGISTER_DIGITAL_GLOBAL_RESET_READOUT, false);
    spi_config_send(MODULE_DEVICE, REGISTER_DIGITAL_ENABLE, 0x02);
    spi_config_send(MODULE_DEVICE, REGISTER_DIGITAL_DUAL_BINNING, false);
    set_global_reset(false);
    set_global_hold(true);
    set_flatten_none();
    set_subsample_every_pixel();
    set_readout_fps_variable_5000();
    set_crop_bypass();
    spi_config_send(MODULE_DEVICE, REGISTER_DIGITAL_RESTART, DVS_CHIP_DTAG_CONTROL_RESTART);

    // Multiplexer stall-drop flags.
    spi_config_send(MODULE_MULTIPLEXER, MUX_DROP_EXTINPUT_ON_TRANSFER_STALL, true);
    spi_config_send(MODULE_MULTIPLEXER, MUX_DROP_DVS_ON_TRANSFER_STALL, false);

    // IMU sample config (writes only — the IMU stream stays disabled).
    // BMI160 registers (dv dvxplorer.hpp / imu_support.hpp): there is NO
    // sample-rate divider here (that is the DAVIS/InvenSense layout, where
    // address 5 means DIVIDER) — on the DVXplorer address 5 is
    // ACCEL_DATA_RATE, so writing a "divider" there silently parked both
    // ODRs at the device default (measured: 25 Hz). The reference sets
    // 800 Hz output on both axes: register value = enum − 5 (accel) /
    // − 6 (gyro), RATE_800HZ = 11.
    spi_config_send(MODULE_IMU, IMU_ACCEL_DATA_RATE, 11 - 5);  // 800 Hz
    spi_config_send(MODULE_IMU, IMU_ACCEL_FILTER, 2);          // NORMAL
    spi_config_send(MODULE_IMU, IMU_ACCEL_RANGE, 1);           // ±4 g
    spi_config_send(MODULE_IMU, IMU_GYRO_DATA_RATE, 11 - 6);   // 800 Hz
    spi_config_send(MODULE_IMU, IMU_GYRO_FILTER, 2);           // NORMAL
    // ±2000 dps (widest): the DAVIS346 saturated its closed path at ±500.
    // Discriminator: the in-band Scale Config word should broadcast gyro
    // code 0 after this write — if the next EBPLUS_IMU_TRACE=1 connect
    // still shows code 2, the register write is not taking effect.
    spi_config_send(MODULE_IMU, IMU_GYRO_RANGE, 0);

    // External input detector/generator defaults.
    spi_config_send(MODULE_EXTERNAL_INPUT, EXTINPUT_DETECT_RISING_EDGES, false);
    spi_config_send(MODULE_EXTERNAL_INPUT, EXTINPUT_DETECT_FALLING_EDGES, false);
    const auto gen_high = static_cast<std::uint64_t>(std::llround(5.0F * logic_clock_));
    spi_config_send(MODULE_EXTERNAL_INPUT, EXTINPUT_GENERATE_PULSE_LENGTH, gen_high);
    spi_config_send(MODULE_EXTERNAL_INPUT, EXTINPUT_GENERATE_PULSE_INTERVAL, gen_high * 2);
    spi_config_send(MODULE_EXTERNAL_INPUT, EXTINPUT_GENERATE_INJECT_ON_RISING_EDGE, false);
    spi_config_send(MODULE_EXTERNAL_INPUT, EXTINPUT_GENERATE_INJECT_ON_FALLING_EDGE, false);

    // USB early-packet delay: 1 ms in USB-clock ticks.
    spi_config_send(MODULE_USB, USB_EARLY_PACKET_DELAY,
        static_cast<std::uint64_t>(std::llround(1000.0F * usb_clock_)));
}

void DvxplorerDevice::set_imu_sink(const ImuSink& sink) {
    parse_.set_imu_sink(sink);
}

void DvxplorerDevice::set_imu_enabled(bool on) {
    imu_enabled_ = on;
    if (streaming_.load()) {
        spi_config_send(MODULE_IMU, IMU_RUN_ACCELEROMETER, on);
        spi_config_send(MODULE_IMU, IMU_RUN_GYROSCOPE, on);
        spi_config_send(MODULE_IMU, IMU_RUN_TEMPERATURE, on);
    }
}

void DvxplorerDevice::set_contrast_on(int value) {
    if (value < 0) value = 0;
    if (value > 17) value = 17;
    contrast_on_ = value;
    set_contrast_threshold(contrast_on_, contrast_off_);
}

void DvxplorerDevice::set_contrast_off(int value) {
    if (value < 0) value = 0;
    if (value > 17) value = 17;
    contrast_off_ = value;
    set_contrast_threshold(contrast_on_, contrast_off_);
}

void DvxplorerDevice::parse_events(const std::uint8_t* data, std::size_t size) {
    // Decode here (cheap; the IMU sink fires inline so it stays
    // latency-critical), hand the event batch to the processing worker —
    // the heavy pipeline must not run on the USB reaping thread (it
    // saturates under an event flood and delays IMU by seconds).
    parse_.decode(data, size);
    auto slot = batches_.acquire();
    parse_.swap_batch(*slot);
    std::function<void(const Metavision::EventCD*, const Metavision::EventCD*)> consumer;
    {
        std::lock_guard<std::mutex> lock(raw_consumer_mutex_);
        consumer = raw_consumer_;
    }
    if (consumer) {
        consumer(slot->data(), slot->data() + slot->size());
    }
    batches_.submit(std::move(slot));
}

void DvxplorerDevice::set_contrast_threshold(int on_value, int off_value) {
    // ON: current register plus the high-range bit in the shared range-select
    // register (values 0-8 use the low range, 9-17 the high range).
    if (on_value < 9) {
        spi_config_send(MODULE_DEVICE, REGISTER_BIAS_CURRENT_ON, static_cast<std::uint64_t>(on_value));
        spi_config_send(MODULE_DEVICE, REGISTER_BIAS_CURRENT_RANGE_SELECT_LOGSFONREST, 0x04 | 0x00);
    }
    else {
        spi_config_send(MODULE_DEVICE, REGISTER_BIAS_CURRENT_ON, static_cast<std::uint64_t>(on_value - 9));
        spi_config_send(MODULE_DEVICE, REGISTER_BIAS_CURRENT_RANGE_SELECT_LOGSFONREST, 0x04 | 0x02);
    }
    // OFF: the current ladder is inverted (higher threshold = lower current)
    // and the range bit lives in the SF-OFF level register.
    if (off_value < 9) {
        spi_config_send(MODULE_DEVICE, REGISTER_BIAS_CURRENT_OFF, static_cast<std::uint64_t>(8 - off_value));
        spi_config_send(MODULE_DEVICE, REGISTER_BIAS_CURRENT_LEVEL_SFOFF, 0x7D | 0x02);
    }
    else {
        spi_config_send(MODULE_DEVICE, REGISTER_BIAS_CURRENT_OFF, static_cast<std::uint64_t>(8 - (off_value - 9)));
        spi_config_send(MODULE_DEVICE, REGISTER_BIAS_CURRENT_LEVEL_SFOFF, 0x7D | 0x00);
    }
}

void DvxplorerDevice::set_global_hold(bool on) {
    const auto reg = spi_config_receive(MODULE_DEVICE, REGISTER_DIGITAL_MODE_CONTROL);
    spi_config_send(MODULE_DEVICE, REGISTER_DIGITAL_MODE_CONTROL, on ? (reg | 0x01) : (reg & ~0x01));
}

void DvxplorerDevice::set_global_reset(bool on) {
    const auto reg = spi_config_receive(MODULE_DEVICE, REGISTER_DIGITAL_MODE_CONTROL);
    spi_config_send(MODULE_DEVICE, REGISTER_DIGITAL_MODE_CONTROL, on ? (reg | 0x02) : (reg & ~0x02));
}

void DvxplorerDevice::set_flatten_none() {
    const auto reg = spi_config_receive(MODULE_DEVICE, REGISTER_CONTROL_PACKET_FORMAT);
    spi_config_send(MODULE_DEVICE, REGISTER_CONTROL_PACKET_FORMAT, reg & ~0x70);
}

void DvxplorerDevice::set_subsample_every_pixel() {
    spi_config_send(MODULE_DEVICE, REGISTER_DIGITAL_SUBSAMPLE_RATIO, 0);
}

void DvxplorerDevice::set_readout_fps_variable_5000() {
    spi_config_send(MODULE_DEVICE, REGISTER_DIGITAL_RESTART, 0); // DTAG STOP
    spi_config_send(MODULE_DEVICE, REGISTER_DIGITAL_FIXED_READ_TIME, false);
    spi_config_send(MODULE_DEVICE, REGISTER_TIMING_READ_TIME_INTERVAL,
        static_cast<std::uint64_t>((900 * SYSTEM_CLOCK_FREQUENCY) >> 8));
    spi_config_send(MODULE_DEVICE, REGISTER_TIMING_READ_TIME_INTERVAL + 1,
        static_cast<std::uint64_t>((900 * SYSTEM_CLOCK_FREQUENCY) & 0xFF));
    spi_config_send(MODULE_DEVICE, REGISTER_TIMING_GH_COUNT, 0);
    spi_config_send(MODULE_DEVICE, REGISTER_TIMING_GH_COUNT + 1, 0);
    spi_config_send(MODULE_DEVICE, REGISTER_TIMING_GH_COUNT + 2, 7);
    spi_config_send(MODULE_DEVICE, REGISTER_TIMING_NEXT_SELX_START, 0);
    spi_config_send(MODULE_DEVICE, REGISTER_TIMING_NEXT_SELX_START + 1, 15);
    spi_config_send(MODULE_DEVICE, REGISTER_TIMING_MAX_EVENT_NUM, 10);
}

void DvxplorerDevice::set_crop_bypass() {
    spi_config_send(MODULE_DEVICE, REGISTER_CROPPER_BYPASS, true);
}

void DvxplorerDevice::send_timestamp_reset() {
    spi_config_send(MODULE_MULTIPLEXER, MUX_TIMESTAMP_RESET, true);
    spi_config_send(MODULE_MULTIPLEXER, MUX_TIMESTAMP_RESET, false);
}

bool DvxplorerDevice::wait_for_timestamp_reset() {
    for (int waited_ms = 0; waited_ms < 1000; waited_ms += 10) {
        if (parse_.time_initialized()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
}

void DvxplorerDevice::start() {
    if (streaming_.load()) return;
    // stop() put the chip back in reset — replay the full init sequence so a
    // restart behaves like a fresh open (same open == start contract as the
    // reference, whose constructor is the only start path).
    chip_init();
    usb_data_transfers_start();
    try {
        spi_config_send(MODULE_USB, USB_RUN, true);
        spi_config_send(MODULE_MULTIPLEXER, MUX_TIMESTAMP_RUN, true);
        spi_config_send(MODULE_MULTIPLEXER, MUX_RUN, true);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        // Events only by default — trigger streams stay disabled; the IMU
        // runs re-apply here when the user enabled the IMU panel.
        spi_config_send(MODULE_DVS, DVS_RUN, true);
        if (imu_enabled_) {
            spi_config_send(MODULE_IMU, IMU_RUN_ACCELEROMETER, true);
            spi_config_send(MODULE_IMU, IMU_RUN_GYROSCOPE, true);
            spi_config_send(MODULE_IMU, IMU_RUN_TEMPERATURE, true);
        }
        spi_config_send(MODULE_DEVICE, REGISTER_CONTROL_MODE, DVS_CHIP_MODE_STREAM);
        send_timestamp_reset();
        if (!wait_for_timestamp_reset()) {
            throw std::runtime_error("DVXplorer: no timestamp reset received — stream did not start.");
        }
    } catch (...) {
        try {
            spi_config_send(MODULE_DVS, DVS_RUN, false);
            spi_config_send(MODULE_MULTIPLEXER, MUX_RUN, false);
            spi_config_send(MODULE_MULTIPLEXER, MUX_TIMESTAMP_RUN, false);
            spi_config_send(MODULE_USB, USB_RUN, false);
            spi_config_send(MODULE_MULTIPLEXER, MUX_RUN_CHIP, false);
        } catch (...) {
        }
        usb_data_transfers_stop();
        throw;
    }
    streaming_.store(true);
}

void DvxplorerDevice::stop() {
    if (!streaming_.exchange(false)) return;
    try {
        spi_config_send(MODULE_DVS, DVS_RUN, false);
        if (imu_enabled_) {
            spi_config_send(MODULE_IMU, IMU_RUN_ACCELEROMETER, false);
            spi_config_send(MODULE_IMU, IMU_RUN_GYROSCOPE, false);
            spi_config_send(MODULE_IMU, IMU_RUN_TEMPERATURE, false);
        }
        spi_config_send(MODULE_MULTIPLEXER, MUX_RUN, false);
        spi_config_send(MODULE_MULTIPLEXER, MUX_TIMESTAMP_RUN, false);
        spi_config_send(MODULE_USB, USB_RUN, false);
        spi_config_send(MODULE_MULTIPLEXER, MUX_RUN_CHIP, false);
    } catch (...) {
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    usb_data_transfers_stop();
    parse_.reset();
}

} // namespace gui::davis
