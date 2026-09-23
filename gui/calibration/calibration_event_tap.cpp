// gui/calibration/calibration_event_tap.cpp — see header (Phase 4).

#include "calibration_event_tap.h"

#include <algorithm>
#include <utility>

#include "app/camera_controller.h"

namespace gui {

CalibrationEventTap::CalibrationEventTap(QObject* parent) : QObject(parent) {}

CalibrationEventTap::~CalibrationEventTap() {
    // Disconnect first so no new on_events_ready() calls start after this
    // returns. Then lock mutex_ to wait for any in-progress call (running on
    // the SDK thread via DirectConnection) to finish before batches_ is
    // destroyed — prevents use-after-free.
    if (camera_) {
        disconnect(camera_, &CameraController::cd_events_ready,
                   this, &CalibrationEventTap::on_events_ready);
    }
    std::lock_guard<std::mutex> lk(mutex_);
    batches_.clear();
}

void CalibrationEventTap::attach(CameraController* camera) {
    // Phase 4 bug-absorption: disconnect any existing connection FIRST so a
    // repeated attach() (the wizard calls set_camera → attach each time it
    // opens) never stacks duplicate connections that would deliver each batch
    // N times. disconnect() is a no-op if no connection exists.
    if (camera_) {
        disconnect(camera_, &CameraController::cd_events_ready,
                   this, &CalibrationEventTap::on_events_ready);
    }
    camera_ = camera;
    if (camera_) {
        // DirectConnection: on_events_ready() runs on the SDK streaming
        // thread (the emitter's thread), NOT the GUI thread. This avoids
        // posting batches to the GUI event queue, which would grow without
        // bound when the worker blocks and OOM-kill the process. The slot is
        // thread-safe (mutex_ only, no GUI access).
        connect(camera_, &CameraController::cd_events_ready,
                this, &CalibrationEventTap::on_events_ready,
                Qt::DirectConnection);
    }
}

void CalibrationEventTap::on_events_ready(
    std::shared_ptr<std::vector<Metavision::EventCD>> events) {
    // Perf: store the shared_ptr directly — no per-event copy. The SDK thread
    // cost is a refcount increment + deque push_back, both O(1). The actual
    // event collection happens on the GUI thread in drain_last_window(), which
    // runs only once per Space press.
    if (!events || events->empty()) return;
    std::lock_guard<std::mutex> lk(mutex_);
    if (events->front().t < last_back_t_) {
        // Timestamp epoch reset (camera stop/start/reconnect while the
        // wizard is open): drop the entire previous epoch, or it would
        // poison every capture until the wizard is reopened.
        batches_.clear();
        total_events_ = 0;
    }
    last_back_t_ = events->back().t;
    batches_.push_back(events);  // shared_ptr copy — refcount only
    total_events_ += events->size();

    // Trim by TIME: drop batches whose last event is older than the keep
    // window. Retention is deliberately not batch-count-capped — small SDK
    // batches under a blinking-LCD noise floor need tens of thousands of
    // batches to span kKeepWindowUs (see kMaxTotalEvents in the header).
    // Beyond the absolute event-count valve, drop oldest batches too.
    const Metavision::timestamp t_last = events->back().t;
    const Metavision::timestamp t_cutoff = t_last - kKeepWindowUs;
    while (!batches_.empty() &&
           (batches_.front()->back().t < t_cutoff ||
            total_events_ > kMaxTotalEvents)) {
        total_events_ -= batches_.front()->size();
        batches_.pop_front();
    }
}

std::size_t CalibrationEventTap::drain_last_window(
    Metavision::timestamp window_us,
    std::vector<Metavision::EventCD>& out) {
    out.clear();
    if (window_us <= 0) return 0;

    std::deque<BatchPtr> local;
    {
        std::lock_guard<std::mutex> lk(mutex_);
        local.swap(batches_);
        total_events_ = 0;
    }
    if (local.empty()) return 0;

    // t_last is the timestamp of the most recent event across all batches.
    // Batches are in chronological order, so the last batch's last event is
    // the global maximum.
    const Metavision::timestamp t_last = local.back()->back().t;
    const Metavision::timestamp t_start = t_last - window_us;

    // Collect events with t >= t_start. Batches are chronologically ordered;
    // skip entire batches whose last event is before t_start. Within a batch,
    // events are SDK-sorted by timestamp, so lower_bound finds the start.
    std::size_t total = 0;
    for (const auto& batch : local) {
        if (batch->empty()) continue;
        if (batch->back().t < t_start) continue;  // whole batch is stale
        auto it = std::lower_bound(batch->begin(), batch->end(), t_start,
            [](const Metavision::EventCD& e, Metavision::timestamp t) {
                return e.t < t;
            });
        out.insert(out.end(), it, batch->end());
        total += static_cast<std::size_t>(batch->end() - it);
    }
    return total;
}

void CalibrationEventTap::clear() {
    std::lock_guard<std::mutex> lk(mutex_);
    batches_.clear();
    total_events_ = 0;
}

} // namespace gui
