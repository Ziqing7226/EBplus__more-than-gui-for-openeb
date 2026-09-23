// gui/tests/test_algo_bridge.cpp — AlgoBridge unit tests (design §3.11.2).
//
// Covers: registry completeness, find_or_create idempotency, create()
// freshness, find_live() visibility, the rate-based flood guard (sustained
// event rate above threshold across consecutive 1s wall-clock windows
// auto-disables the instance), and set_param/get_param round-trip.
// All events are synthetic — no real camera or file I/O.

#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <metavision/sdk/base/events/event_cd.h>

#include "algo_bridge/algo_backend.h"
#include "algo_bridge/algo_bridge.h"

using gui::AlgoBridge;
using gui::AlgoInstance;
using Metavision::EventCD;

namespace {

// Builds a synthetic batch of @p n EventCD events with monotonically
// increasing timestamps and in-bounds coordinates for a 1280x720 sensor.
std::vector<EventCD> make_events(std::size_t n, int w = 1280, int h = 720) {
    std::vector<EventCD> evs(n);
    for (std::size_t i = 0; i < n; ++i) {
        evs[i].x = static_cast<uint16_t>(i % w);
        evs[i].y = static_cast<uint16_t>((i / w) % h);
        evs[i].p = (i % 2) ? 1 : 0;
        evs[i].t = static_cast<Metavision::timestamp>(i);
    }
    return evs;
}

} // namespace

// ---------------------------------------------------------------------------
// Registry completeness (§3.11.2: 28 self + 4 openEB). noise_filter was
// removed in v1.0.9 (now a stackable preprocessing stage); sensor_self_test
// was added in §4.4.8; intrinsic_calibration was removed (now a Tools-menu
// wizard, not a registered algo); perspective_undistort was removed (audit
// §三-B8, superseded by the preproc undistort stage); the 22 unreachable
// OpenEB frame/preproc/util/roi_mask/adaptive_rate_split registrations were
// removed (audit §三-B6/7), leaving the FilterChain event-transform stages
// (polarity_filter, polarity_invert, flip_x, flip_y — roi_filter was removed
// in Phase 2.6 debug D-6, superseded by the unified ROI; rotate/transpose/
// rescale were removed 2026-08-18 for coordinate-OOB heap corruption).
// ultra_slow_motion was removed in Phase 2.5 (broken
// Replace-display promise, no downstream consumer). particle_counter,
// flow_statistics and overlay were removed 2026-08-22 (user decision:
// particle_counter fully self-invented, flow_statistics needed unavailable
// ground truth, overlay was a pass-through placeholder; isi_analyzer /
// trigger_synced / active_marker were self-invented or input-less). sensor_self_test is
// intentionally NOT registered (it is a Devices-panel diagnostic, not an
// algorithm — its instance is created via create_with_info).
// Live registry: 20 self-developed + 4 OpenEB = 24.
// ---------------------------------------------------------------------------
TEST(AlgoBridgeRegistry, ListsAllRegisteredAlgos) {
    AlgoBridge bridge;
    const auto algos = bridge.list_algos();
    EXPECT_EQ(algos.size(), 25u);

    std::size_t self_count = 0, openeb_count = 0;
    for (const auto& a : algos) {
        if (a.source == "self") ++self_count;
        else if (a.source == "openeb") ++openeb_count;
    }
    EXPECT_EQ(self_count, 21u);
    EXPECT_EQ(openeb_count, 4u);
}

TEST(AlgoBridgeRegistry, KeyNamesPresent) {
    AlgoBridge bridge;
    // Self-developed key algorithms.
    EXPECT_NE(bridge.find("hot_pixel_filter"), nullptr);
    EXPECT_NE(bridge.find("event_to_video"), nullptr);
    EXPECT_NE(bridge.find("object_tracker"), nullptr);
    EXPECT_NE(bridge.find("hough_line"), nullptr);
    EXPECT_NE(bridge.find("time_surface"), nullptr);
    EXPECT_NE(bridge.find("blob_detector"), nullptr);
    // sensor_self_test is intentionally NOT registered (Devices-panel
    // diagnostic, not an algorithm) — but its instance must still be
    // creatable from an explicit AlgoInfo via create_with_info.
    EXPECT_EQ(bridge.find("sensor_self_test"), nullptr);
    // OpenEB event-transform stages (handled by FilterChain). roi_filter is
    // intentionally NOT registered (Phase 2.6 debug D-6 — superseded by the
    // unified ROI).
    EXPECT_EQ(bridge.find("roi_filter"), nullptr);
    EXPECT_NE(bridge.find("flip_x"), nullptr);
    EXPECT_NE(bridge.find("polarity_filter"), nullptr);
}

TEST(AlgoBridgeInstances, SensorSelfTestCreatableWithoutRegistration) {
    AlgoBridge bridge;
    gui::AlgoInfo a;
    a.name = "sensor_self_test";
    a.display_name = "Sensor Self-Test";
    a.category = "devices";
    a.source = "self";
    a.display_mode = gui::AlgoDisplayMode::Standalone;
    auto inst = bridge.find_or_create_with_info(a);
    ASSERT_NE(inst, nullptr);
    EXPECT_EQ(inst->info().name, "sensor_self_test");
    // find_or_create_with_info returns the same live instance on repeat.
    auto inst2 = bridge.find_or_create_with_info(a);
    EXPECT_EQ(inst.get(), inst2.get());
}

TEST(AlgoBridgeRegistry, RemovedRegistrationsAreGone) {
    AlgoBridge bridge;
    // Audit §三-B6/7: unreachable OpenEB backends removed.
    EXPECT_EQ(bridge.find("frame_integration"), nullptr);
    EXPECT_EQ(bridge.find("preproc_diff"), nullptr);
    EXPECT_EQ(bridge.find("preproc_hw_diff"), nullptr);
    EXPECT_EQ(bridge.find("util_rate_estimator"), nullptr);
    EXPECT_EQ(bridge.find("roi_mask"), nullptr);
    EXPECT_EQ(bridge.find("adaptive_rate_split"), nullptr);
    // Audit §三-B8: superseded by the preproc undistort stage.
    EXPECT_EQ(bridge.find("perspective_undistort"), nullptr);
    // 2026-08-18: coordinate-changing transforms removed (OOB heap
    // corruption — no frame-geometry propagation to downstream buffers).
    EXPECT_EQ(bridge.find("rotate"), nullptr);
    EXPECT_EQ(bridge.find("transpose"), nullptr);
    EXPECT_EQ(bridge.find("rescale"), nullptr);
}

TEST(AlgoBridgeRegistry, NoiseFilterRemovedInV1_0_9) {
    AlgoBridge bridge;
    // noise_filter is now a stackable preprocessing stage, not a standalone
    // algorithm, so it must not be in the registry.
    EXPECT_EQ(bridge.find("noise_filter"), nullptr);
    // Unknown name returns nullptr.
    EXPECT_EQ(bridge.find("does_not_exist_algo"), nullptr);
}

TEST(AlgoBridgeRegistry, IntrinsicCalibrationRemovedIsNowToolsWizard) {
    // intrinsic_calibration is now a Tools-menu wizard (CalibrationWizard),
    // not a registered algorithm, so it must not be in the registry.
    // Undistortion is available as a stackable preprocessing checkbox in the
    // Algorithms panel (preproc_undistort_enabled / preproc_undistort_path).
    AlgoBridge bridge;
    EXPECT_EQ(bridge.find("intrinsic_calibration"), nullptr);
}

TEST(AlgoBridgeRegistry, EventToVideoIsRegistered) {
    AlgoBridge bridge;
    const auto* info = bridge.find("event_to_video");
    ASSERT_NE(info, nullptr);
    EXPECT_EQ(info->source, "self");
    EXPECT_FALSE(info->params.empty());
}

// §五-H1: the "model_loaded" pseudo-param is not a registered key — it is
// served by the backend fallback in AlgoInstance::get_param that was added
// for it. In E2VID mode (the default) the value is "true"/"false" depending
// on whether the ONNX file at model_path could be loaded; it must never be
// empty.
TEST(AlgoBridgeInstances, EventToVideoModelLoadedPseudoParam) {
    AlgoBridge bridge;
    auto inst = bridge.find_or_create("event_to_video");
    ASSERT_NE(inst, nullptr);
    const std::string v = inst->get_param("model_loaded");
    EXPECT_TRUE(v == "true" || v == "false")
        << "model_loaded must be reachable via the backend fallback, got: '"
        << v << "'";
}

// ---------------------------------------------------------------------------
// find_or_create() idempotency / create() freshness / find_live() visibility.
// ---------------------------------------------------------------------------
TEST(AlgoBridgeInstances, FindOrCreateIsIdempotent) {
    AlgoBridge bridge;
    auto a = bridge.find_or_create("hot_pixel_filter");
    auto b = bridge.find_or_create("hot_pixel_filter");
    ASSERT_NE(a, nullptr);
    EXPECT_EQ(a.get(), b.get());  // same underlying instance
    EXPECT_EQ(a, b);              // shared_ptr equality
}

TEST(AlgoBridgeInstances, CreateReturnsFreshInstanceEachCall) {
    AlgoBridge bridge;
    auto a = bridge.create("hot_pixel_filter");
    auto b = bridge.create("hot_pixel_filter");
    ASSERT_NE(a, nullptr);
    ASSERT_NE(b, nullptr);
    EXPECT_NE(a.get(), b.get());  // distinct instances
}

TEST(AlgoBridgeInstances, FindLiveBeforeAndAfterCreate) {
    AlgoBridge bridge;
    EXPECT_EQ(bridge.find_live("hot_pixel_filter"), nullptr);
    auto inst = bridge.create("hot_pixel_filter");
    (void)inst;
    EXPECT_NE(bridge.find_live("hot_pixel_filter"), nullptr);
    EXPECT_EQ(bridge.find_live("object_tracker"), nullptr);
}

TEST(AlgoBridgeInstances, ListLiveReturnsAllLiveInstances) {
    AlgoBridge bridge;
    // Hold the shared_ptrs alive — AlgoBridge stores weak_ptrs, so the live
    // instances expire if the returned shared_ptrs are discarded.
    auto a = bridge.create("hot_pixel_filter");
    auto b = bridge.create("object_tracker");
    ASSERT_NE(a, nullptr);
    ASSERT_NE(b, nullptr);
    auto live = bridge.list_live();
    EXPECT_EQ(live.size(), 2u);
}

TEST(AlgoBridgeInstances, FindLivePrunesExpiredEntries) {
    AlgoBridge bridge;
    {
        auto inst = bridge.create("hot_pixel_filter");
        (void)inst;
        EXPECT_NE(bridge.find_live("hot_pixel_filter"), nullptr);
    }  // inst destroyed here
    // After the shared_ptr is released, find_live returns nullptr and prunes.
    EXPECT_EQ(bridge.find_live("hot_pixel_filter"), nullptr);
}

TEST(AlgoBridgeInstances, CreateUnknownReturnsNull) {
    AlgoBridge bridge;
    EXPECT_EQ(bridge.create("not_a_real_algo"), nullptr);
    EXPECT_EQ(bridge.find_or_create("not_a_real_algo"), nullptr);
}

// ---------------------------------------------------------------------------
// set_param / get_param round-trip.
// ---------------------------------------------------------------------------
TEST(AlgoBridgeInstances, ParamRoundTrip) {
    AlgoBridge bridge;
    auto inst = bridge.find_or_create("hot_pixel_filter");
    ASSERT_NE(inst, nullptr);
    inst->set_param("fpn_alpha", "0.5");
    inst->set_param("learning_window_s", "10.0");
    EXPECT_EQ(inst->get_param("fpn_alpha"), "0.5");
    EXPECT_EQ(inst->get_param("learning_window_s"), "10.0");
    // Unknown key returns an empty string.
    EXPECT_EQ(inst->get_param("no_such_key"), "");

    // sparse flow (P7): BlockMatch params now registered + round-trip; the
    // arrow scale param round-trips too.
    // blob_detector (P leftover 2): accumulation_ms / min_area now registered
    // and round-trip through the backend.
    auto bl = bridge.find_or_create("blob_detector");
    ASSERT_NE(bl, nullptr);
    bl->set_param("accumulation_ms", "50");
    bl->set_param("min_area", "25");
    EXPECT_EQ(bl->get_param("accumulation_ms"), "50");
    EXPECT_EQ(bl->get_param("min_area"), "25");

    auto sf = bridge.find_or_create("sparse_optical_flow");
    ASSERT_NE(sf, nullptr);
    sf->set_param("bm_time_window_us", "33000");
    sf->set_param("num_scales", "4");
    sf->set_param("pps_scale_lk", "0.5");  // per-mode arrow scale (LK mode)
    EXPECT_EQ(sf->get_param("bm_time_window_us"), "33000");
    EXPECT_EQ(sf->get_param("num_scales"), "4");
    EXPECT_EQ(sf->get_param("pps_scale_lk"), "0.5");

    // direction_selective: embedded orientation pre-stage params round-trip
    // (jAER enclosed SimpleOrientationFilter alignment).
    auto ds = bridge.find_or_create("direction_selective");
    ASSERT_NE(ds, nullptr);
    ds->set_param("ori_min_dt_threshold_us", "2000");
    ds->set_param("ori_dt_reject_threshold_us", "300000");
    EXPECT_EQ(ds->get_param("ori_min_dt_threshold_us"), "2000");
    EXPECT_EQ(ds->get_param("ori_dt_reject_threshold_us"), "300000");
}

// time_surface decay/tau_us/refresh_rate_hz must round-trip through the
// TimeSurfaceBackend into the algo (Phase 6-3).
TEST(AlgoBridgeInstances, TimeSurfaceDecayParamsRoundTrip) {
    AlgoBridge bridge;
    auto inst = bridge.find_or_create("time_surface");
    ASSERT_NE(inst, nullptr);
    // Registered defaults are applied at construction.
    EXPECT_EQ(inst->get_param("decay"), "0");
    EXPECT_EQ(inst->get_param("tau_us"), "100000");
    EXPECT_EQ(inst->get_param("refresh_rate_hz"), "30");
    inst->set_param("decay", "1");
    inst->set_param("tau_us", "200000");
    inst->set_param("refresh_rate_hz", "60");
    EXPECT_EQ(inst->get_param("decay"), "1");
    EXPECT_EQ(inst->get_param("tau_us"), "200000");
    EXPECT_EQ(inst->get_param("refresh_rate_hz"), "60");
}

TEST(AlgoBridgeInstances, DefaultsAppliedAtConstruction) {
    AlgoBridge bridge;
    auto inst = bridge.find_or_create("hot_pixel_filter");
    ASSERT_NE(inst, nullptr);
    // The default fpn_alpha declared in the registry is "0.9".
    EXPECT_EQ(inst->get_param("fpn_alpha"), "0.9");
    // n_sigma was removed from the registry (algo marks it unused, audit
    // §三-31) — it is now an unknown key and returns an empty string.
    EXPECT_EQ(inst->get_param("n_sigma"), "");
}

TEST(AlgoBridgeInstances, EnableDisableState) {
    AlgoBridge bridge;
    auto inst = bridge.find_or_create("hot_pixel_filter");
    ASSERT_NE(inst, nullptr);
    EXPECT_FALSE(inst->is_enabled());
    inst->set_enabled(true);
    EXPECT_TRUE(inst->is_enabled());
    inst->set_enabled(false);
    EXPECT_FALSE(inst->is_enabled());
}

// P6-A stream write-back: a filtering algorithm (hot_pixel_filter) exposes a
// per-batch output span after push_events; an annotating detector
// (blob_detector) exposes none (nullptr = pass-through, the display keeps
// the conditioned span). Disabled instances never expose a span.
TEST(AlgoBridgeInstances, OutputSpanFilterVsDetector) {
    AlgoBridge bridge;

    auto filt = bridge.find_or_create("hot_pixel_filter");
    ASSERT_NE(filt, nullptr);
    filt->set_enabled(true);
    {
        auto [fb, fn] = filt->output_span();
        EXPECT_EQ(fb, nullptr);
        EXPECT_EQ(fn, 0u);
    }
    auto evs = make_events(100);
    filt->push_events(evs.data(), evs.data() + evs.size());
    {
        auto [fb, fn] = filt->output_span();
        ASSERT_NE(fb, nullptr);
        EXPECT_LE(fn, evs.size());
        // Span points at the kept events: contents are in-bounds EventCDs.
        for (std::size_t i = 0; i < fn; ++i) {
            EXPECT_GE(fb[i].x, 0);
        }
    }
    filt->set_enabled(false);
    {
        auto [fb, fn] = filt->output_span();
        EXPECT_EQ(fb, nullptr);
    }

    auto det = bridge.find_or_create("blob_detector");
    ASSERT_NE(det, nullptr);
    det->set_enabled(true);
    det->push_events(evs.data(), evs.data() + evs.size());
    {
        auto [fb, fn] = det->output_span();
        EXPECT_EQ(fb, nullptr);  // detectors annotate, they don't filter
        EXPECT_EQ(fn, 0u);
    }
}

// ---------------------------------------------------------------------------
// Flood guard (design §5.6.7, rate-based): the instance measures the
// wall-clock event rate over a sliding 1s window; when the rate exceeds
// kMaxEventRateEvPerSec (30 Mev/s) for 4 consecutive windows the instance is
// auto-disabled. Batches are never truncated (the old batch-cap guard that
// silently dropped window-front events was removed, audit §五-E1).
// ---------------------------------------------------------------------------
TEST(AlgoBridgeFloodGuard, OverloadsAfterSustainedHighRate) {
    AlgoBridge bridge;
    // The blob_detector backend is among the cheapest (per-event count
    // accumulation), so a tight push loop sustains far more than the
    // 100 Mev/s threshold.
    auto inst = bridge.find_or_create("blob_detector");
    ASSERT_NE(inst, nullptr);
    inst->set_enabled(true);
    ASSERT_FALSE(inst->is_overloaded());

    auto batch = make_events(1000000);
    EventCD* data = batch.data();
    // 4 consecutive 1s windows above threshold are needed; allow up to 15s.
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(15);
    while (!inst->is_overloaded() &&
           std::chrono::steady_clock::now() < deadline) {
        inst->push_events(data, data + batch.size());
    }
    EXPECT_TRUE(inst->is_overloaded())
        << "sustained >30 Mev/s input should trip the flood guard";
    EXPECT_FALSE(inst->is_enabled());  // overloaded implies disabled

    // Pushing more events while overloaded is a no-op (counted as dropped).
    inst->push_events(data, data + batch.size());
    EXPECT_TRUE(inst->is_overloaded());

    // clear_overload() resets the state.
    inst->clear_overload();
    EXPECT_FALSE(inst->is_overloaded());
}

TEST(AlgoBridgeFloodGuard, ModerateRateDoesNotOverload) {
    AlgoBridge bridge;
    auto inst = bridge.find_or_create("hot_pixel_filter");
    ASSERT_NE(inst, nullptr);
    inst->set_enabled(true);

    // ~50k ev/s for >1s — the same order as slow file playback (the audit
    // requires 1.5 Mev/s playback to never trip; the threshold is 30 Mev/s).
    auto batch = make_events(1000);
    EventCD* data = batch.data();
    for (int i = 0; i < 60; ++i) {
        inst->push_events(data, data + batch.size());
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    EXPECT_FALSE(inst->is_overloaded());
    EXPECT_TRUE(inst->is_enabled());
    // Batches are never truncated: nothing was dropped.
    EXPECT_EQ(inst->total_pushed(), 60000u);
    EXPECT_EQ(inst->total_dropped(), 0u);
}

TEST(AlgoBridgeBackends, OpticalFlowPpsScaleKeysMapToDistinctSlots) {
    // Backend-level check (instance-level get_param reads the param cache,
    // which masked this): the ctor replays every registered default through
    // set_param while mode is still 0, so a backend that slots pps_scale_*
    // by the CURRENT mode ends up with the last default (co/tm) in slot 0
    // and the GUI-vs-backend values diverge. Slots must follow the key.
    const auto backend = gui::create_algo_backend("sparse_optical_flow", 64, 64);
    ASSERT_NE(backend, nullptr);
    backend->set_param("pps_scale_lp", "0.003");
    backend->set_param("pps_scale_lk", "0.7");
    backend->set_param("pps_scale_bm", "0.09");
    backend->set_param("pps_scale_co", "0.05");
    const auto expect_val = [](gui::AlgoBackend& b, const char* key, double want) {
        EXPECT_NEAR(std::stod(b.get_param(key)), want, 1e-9) << key;
    };
    backend->set_param("mode", "1");  // switch away from LocalPlanes
    expect_val(*backend, "pps_scale_lp", 0.003);
    expect_val(*backend, "pps_scale_lk", 0.7);
    backend->set_param("mode", "0");
    expect_val(*backend, "pps_scale_lp", 0.003);

    const auto dense = gui::create_algo_backend("dense_optical_flow", 64, 64);
    ASSERT_NE(dense, nullptr);
    dense->set_param("pps_scale_pf", "0.002");
    dense->set_param("pps_scale_tg", "0.007");
    dense->set_param("pps_scale_tm", "0.009");
    dense->set_param("mode", "2");
    expect_val(*dense, "pps_scale_pf", 0.002);
    expect_val(*dense, "pps_scale_tm", 0.009);
}
