// gui/app/external_file_source.h — playback source for file formats the
// Metavision SDK cannot open (AEDAT4 from inivation DV, ALPDATA from
// Alpsentek). Implementations parse the header on open() (GUI thread) and
// stream EventCD batches from run() on a worker thread owned by
// CameraController — the same contract as the SDK's streaming thread feeding
// FramePipeline in file mode.
//
// Events are the primary stream; the AEDAT4 source additionally decodes its
// IMU and APS (frame) streams when present (surfaced like the live-device
// streams). ALPDATA APS frames stay event-only.

#ifndef GUI_APP_EXTERNAL_FILE_SOURCE_H
#define GUI_APP_EXTERNAL_FILE_SOURCE_H

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include <QString>

#include <metavision/sdk/base/events/event_cd.h>
#include <metavision/sdk/base/utils/timestamp.h>

#include "davis/aps_decoder.h"
#include "davis/imu_types.h"

namespace gui {

struct ExternalFileMeta {
    int width{0};
    int height{0};
    /// Total duration in µs (0 = unknown until fully streamed).
    Metavision::timestamp duration_us{0};
    /// Upper bound on synthesized events (-1 = unknown); feeds the OOM warning.
    std::int64_t worst_case_events{-1};
    /// Suggested accumulation window (0 = no hint). ALPDATA frames carry all
    /// their pixels on one timestamp, so the window is set to the frame
    /// period — every displayed frame then shows exactly one recorded frame.
    Metavision::timestamp accumulation_hint_us{0};
    QString serial;
    QString integrator;
    /// Short format label shown in the Information panel ("AEDAT4"/"ALPDATA").
    QString plugin_name;
    QString encoding_format;
};

class ExternalFileSource {
public:
    /// Sink invoked on the reader thread with a batch of sorted events. The
    /// span is only valid for the duration of the call (the consumer copies).
    using EventSink =
        std::function<void(const Metavision::EventCD*, const Metavision::EventCD*)>;
    /// Completion callback: invoked exactly once at the end of run() with an
    /// empty string on success or a user-readable error message.
    using DoneFn = std::function<void(const std::string&)>;

    virtual ~ExternalFileSource();

    /// Parses the file header / metadata. Throws std::runtime_error with a
    /// user-readable message on failure. Called once, before run().
    virtual void open() = 0;

    /// Streams the whole file, invoking @p sink per batch, then @p done
    /// exactly once. Runs on the caller's thread; must observe request_stop()
    /// promptly. Must not throw past its first sink call — errors are
    /// reported through @p done.
    virtual void run(EventSink sink, DoneFn done) = 0;

    /// Cooperative cancellation (GUI thread); run() returns soon after.
    virtual void request_stop() { stop_.store(true, std::memory_order_relaxed); }

    /// Side streams (AEDAT4 only). has_*() is known after open(); the sinks
    /// are invoked on the reader thread while run() streams. Defaults: the
    /// format carries no such stream.
    using ImuSink = std::function<void(const davis::ImuSample&)>;
    using ApsSink = std::function<void(const davis::ApsFrame&)>;
    virtual bool has_imu() const { return false; }
    virtual bool has_aps() const { return false; }
    virtual void set_imu_sink(ImuSink) {}
    virtual void set_aps_sink(ApsSink) {}
    /// First actual IMU(true)/APS(false) packet decoded — presence by
    /// content (declarations in the file header can over-report).
    virtual void set_side_stream_discovered(std::function<void(bool)>) {}

    const ExternalFileMeta& meta() const { return meta_; }

protected:
    ExternalFileMeta meta_;
    std::string path_;
    std::atomic<bool> stop_{false};
};

/// Factory: returns a source for @p path when the extension is a supported
/// external format (.aedat4 / .alpdata), nullptr otherwise. Throws nothing —
/// header errors surface from open().
std::unique_ptr<ExternalFileSource> try_open_external_file(const std::string& path);

/// True when @p path has an external-format extension (used for user hints).
bool is_external_file_extension(const std::string& path);

} // namespace gui

#endif // GUI_APP_EXTERNAL_FILE_SOURCE_H
