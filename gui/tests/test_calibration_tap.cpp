// gui/tests/test_calibration_tap.cpp — CalibrationEventTap retention tests.
//
// Regression guard for the wizard's "never detects" failure (2026-08-15): the
// tap used to cap the ring at 256 BATCHES. The SDK delivers small batches
// under a blinking-LCD noise floor (measured on rec_20260812: ~256
// events/batch at ~104 Mev/s), so the cap kept only ~0.64 ms of a 100 ms
// capture window — no blink cycle, detection could never succeed. Retention
// is now purely time-based (kKeepWindowUs) with an absolute event-count
// safety valve; these tests pin both properties.

#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <vector>

#include <QCoreApplication>

#include <metavision/sdk/base/events/event_cd.h>

#include "calibration/calibration_event_tap.h"

namespace {

using Batch = std::vector<Metavision::EventCD>;
using BatchPtr = std::shared_ptr<Batch>;

// Builds a batch of n events all stamped at time t.
BatchPtr make_batch(Metavision::timestamp t, int n) {
    auto b = std::make_shared<Batch>();
    b->reserve(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
        b->push_back(Metavision::EventCD(0, 0, 0, t));
    }
    return b;
}

// Feeds the tap small batches at the measured real-world cadence:
// 256-event batches every 2.5 µs = ~104 Mev/s, over [t0, t0 + span_us].
void feed_small_batches(gui::CalibrationEventTap& tap,
                        Metavision::timestamp t0,
                        Metavision::timestamp span_us) {
    constexpr int kEvPerBatch = 256;
    constexpr Metavision::timestamp kBatchPeriodUs = 2;
    for (Metavision::timestamp t = t0; t < t0 + span_us; t += kBatchPeriodUs) {
        tap.on_events_ready(make_batch(t, kEvPerBatch));
    }
}

class CalibrationTapTest : public ::testing::Test {
protected:
    gui::CalibrationEventTap tap_;
};

} // namespace

// The core regression: with 256-event batches streaming for well over
// kMaxBatches batches, drain_last_window(100 ms) must return a FULL window —
// events spanning [t_last - 100 ms, t_last], not a sub-millisecond sliver.
TEST_F(CalibrationTapTest, SmallBatchesKeepFullCaptureWindow) {
    // 300 ms of small batches = 150,000 batches (>> the old 256-batch cap).
    feed_small_batches(tap_, 1'000'000, 300'000);

    std::vector<Metavision::EventCD> out;
    const std::size_t n = tap_.drain_last_window(100'000, out);
    ASSERT_EQ(n, out.size());
    EXPECT_FALSE(out.empty());

    // Window span: first event must be within one batch period of the cut.
    const Metavision::timestamp t_last = out.back().t;
    const Metavision::timestamp t_first = out.front().t;
    EXPECT_GE(t_last - t_first, 100'000 - 2);
    // Event count consistent with the fed rate: ~104 Mev/s over 100 ms
    // (allow slack for the batch-quantized cut) — the old bug returned
    // ~65 K events (0.64 ms); anything below 5 M means the cap is back.
    EXPECT_GT(n, 5'000'000u) << "capture window truncated — batch-count cap regression";
}

// Retention bound: events older than kKeepWindowUs (210 ms) are dropped, so
// memory stays bounded by the keep window, not by the stream duration.
TEST_F(CalibrationTapTest, OldBatchesBeyondKeepWindowAreDropped) {
    // 400 ms of small batches — the first ~190 ms must be evicted by the
    // time-based trim (feed is much larger than the keep window).
    feed_small_batches(tap_, 0, 400'000);

    std::vector<Metavision::EventCD> out;
    tap_.drain_last_window(200'000, out);
    ASSERT_FALSE(out.empty());
    const Metavision::timestamp t_last = out.back().t;
    EXPECT_LE(out.front().t, t_last - 200'000 + 2)
        << "drain did not find a full 200 ms window inside the keep window";
    // And nothing older than the keep window survives a second drain... the
    // first drain empties the ring, so this asserts zero instead.
    std::vector<Metavision::EventCD> out2;
    EXPECT_EQ(tap_.drain_last_window(200'000, out2), 0u);
}

// The absolute safety valve: even with batches that never age out (all at the
// same timestamp — the time trim cannot evict them), the tap must stop
// retaining beyond kMaxTotalEvents worth of events.
TEST_F(CalibrationTapTest, EpochResetDropsStaleBatches) {
    // Camera stop/start rebases the timestamp epoch to 0 while the wizard
    // stays open. The stale previous-epoch batches have timestamps that
    // dwarf the new epoch, defeating both the time-based trim and the
    // drain's stale-batch skip: every capture would mix pre-restart scene
    // with the new window. The tap must drop the old epoch on detection.
    constexpr int kEv = 256;
    // Epoch 1: batches at t = 1 s .. 1.2 s ("old" scene).
    for (Metavision::timestamp t = 1'000'000; t <= 1'200'000; t += 10'000) {
        tap_.on_events_ready(make_batch(t, kEv));
    }
    // Epoch 2 (restart): timestamps restart near 0.
    for (Metavision::timestamp t = 0; t <= 100'000; t += 10'000) {
        tap_.on_events_ready(make_batch(t, kEv));
    }
    std::vector<Metavision::EventCD> out;
    const std::size_t n = tap_.drain_last_window(100'000, out);
    EXPECT_GT(n, 0u);
    EXPECT_EQ(n, 11u * kEv);  // only epoch-2 batches survive
    for (const auto& e : out) {
        EXPECT_LT(e.t, 1'000'000) << "stale epoch-1 event survived the reset";
    }
}

TEST_F(CalibrationTapTest, TotalEventSafetyValveBoundsRunaway) {
    constexpr Metavision::timestamp t = 42'000'000;
    // 40 M events in 256-event batches, all timestamped identically (the
    // pathological case: time trim is a no-op, only the event valve bites).
    for (int i = 0; i < 40'000'000 / 256; ++i) {
        tap_.on_events_ready(make_batch(t, 256));
    }
    std::vector<Metavision::EventCD> out;
    const std::size_t n = tap_.drain_last_window(100'000, out);
    EXPECT_LE(n, 30'000'000u + 255u) << "event-count safety valve did not bound retention";
    EXPECT_GT(n, 0u);
}

int main(int argc, char** argv) {
    QCoreApplication app(argc, argv);  // QObject needs a Qt application
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
