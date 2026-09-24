// gui/algo_bridge/algo_bridge.cpp
//
// AlgoInstance 持有真实的 AlgoBackend 实例，真正调用 algo/cv 与 algo/analytics
// 的算法类。注册表列出 21 个自研算法 + 4 个 OpenEB 滤波阶段（2026-08-22
// 裁剪后的真值，test_algo_bridge 钉住 25）。

#include "algo_bridge.h"

#include <QImage>

#include <mutex>

#include "display/display_strategy.h"  // IDisplayStrategy + concrete strategies

namespace gui {

// ---------------------------------------------------------------------------
// AlgoInstance
// ---------------------------------------------------------------------------

AlgoInstance::AlgoInstance(const AlgoInfo& info, int width, int height)
    : info_(info), width_(width), height_(height),
      create_w_(width), create_h_(height) {
    for (const auto& p : info_.params) {
        param_values_[p.key] = p.default_value;
    }
    // 创建真实后端（自研算法）。OpenEB 包装算法返回 nullptr → 透传。
    backend_ = create_algo_backend(info_.name, width_, height_);
    if (backend_) {
        // 应用默认参数到后端。
        for (const auto& p : info_.params) {
            backend_->set_param(p.key, p.default_value);
        }
    }
    // Select the display strategy from the declared display mode
    // (design §3.5.3). The strategy is queried later via apply_strategy().
    switch (info_.display_mode) {
        case AlgoDisplayMode::Passive:    strategy_ = std::make_unique<PassiveStrategy>(); break;
        case AlgoDisplayMode::Overlay:    strategy_ = std::make_unique<OverlayStrategy>(); break;
        case AlgoDisplayMode::Replace:    strategy_ = std::make_unique<ReplaceStrategy>(); break;
        case AlgoDisplayMode::Standalone: strategy_ = std::make_unique<StandaloneStrategy>(); break;
    }
}

// Out-of-line so std::unique_ptr<IDisplayStrategy> destroys with a complete type.
AlgoInstance::~AlgoInstance() = default;

void AlgoInstance::apply_strategy(QImage& frame, AlgoResult& result,
                                  DisplayContext& ctx) {
    ctx.instance = this;
    strategy_->apply(frame, result, info_, ctx);
}

void AlgoInstance::set_param(const std::string& key, const std::string& value) {
    std::lock_guard<std::mutex> lk(mutex_);
    // Only store known parameter keys to avoid map pollution from
    // unknown/obsolete keys (BUG-G12). The backend still receives the
    // call so it can handle backward-compat forwards (e.g. "downsample"
    // -> "preproc_downsample" in EventToVideoBackend) and global
    // preproc_* keys (handled by Preprocessor, not in info_.params).
    bool known = false;
    for (const auto& p : info_.params) {
        if (p.key == key) { known = true; break; }
    }
    if (known) param_values_[key] = value;
    if (backend_) {
        backend_->set_param(key, value);
        // BUG-G2/N11: after setting model_path (or num_bins itself) on an
        // E2VID backend, the real num_bins is dictated by the loaded ONNX
        // model's input channel count — the algo ignores the caller's value
        // when a model is loaded (e2vid_inference.h set_num_bins). The
        // backend re-syncs e2vid_num_bins_ from the algo; sync that
        // authoritative value back into param_values_ so get_param("num_bins")
        // returns the model-determined value, not the stale registry default
        // ("5") or a stale cached value. We intentionally do NOT delegate
        // get_param to the backend in general — that would change the string
        // format of every param (backend uses std::to_string(double),
        // producing "5.500000" instead of the registry's "5.5") and break
        // ParamRoundTrip tests. This targeted sync keeps the fix minimal.
        if (key == "model_path" || key == "num_bins") {
            std::string nb = backend_->get_param("num_bins");
            if (!nb.empty()) param_values_["num_bins"] = nb;
        }
    }
}

std::string AlgoInstance::get_param(const std::string& key) const {
    std::lock_guard<std::mutex> lk(mutex_);
    auto it = param_values_.find(key);
    if (it != param_values_.end()) return it->second;
    // Not a registered key: fall back to the backend so runtime pseudo-params
    // (e.g. EventToVideo's "model_loaded", §五-H1) remain reachable. This
    // cannot change the string format of registered params — every key in
    // info_.params is initialised into param_values_ at construction, so the
    // fallback only ever fires for unregistered keys, for which the backend
    // is the only source (unknown keys still yield the backend's empty
    // string, preserving the previous contract).
    if (backend_) return backend_->get_param(key);
    return {};
}

void AlgoInstance::set_enabled(bool e) {
    std::lock_guard<std::mutex> lk(mutex_);
    enabled_ = e;
    if (e) {
        // Re-enabling clears any prior overload state and resets the strike
        // counter so the algo gets a fresh start. The backend's internal
        // state (buffers/accumulators) is intentionally NOT reset here —
        // this preserves the pause-resume workflow where the user temporarily
        // disables an algorithm and later resumes with its accumulated state
        // intact (e.g. E2VID log_intensity_, InteractingMaps I_map_). Full
        // reset is performed separately by the caller when starting a new
        // session (e.g. MainWindow::on_camera_connected calls inst->reset()
        // for every live instance). Drop-rate counters are also reset so
        // the InformationPanel shows fresh telemetry for the new session.
        // Exception: algorithms flagged reset_on_reenable (learning-window
        // algos like Background Mask) must start from a clean state when
        // re-enabled — their frozen result is stale after the disabled gap.
        if (info_.reset_on_reenable && backend_) {
            backend_->reset();
        }
        overloaded_ = false;
        flood_strikes_ = 0;
        rate_window_events_ = 0;
        rate_window_start_ = {};
        total_pushed_ = 0;
        total_dropped_ = 0;
    }
}

void AlgoInstance::set_overload_callback(std::function<void()> cb) {
    std::lock_guard<std::mutex> lk(mutex_);
    overload_callback_ = std::move(cb);
}

bool AlgoInstance::is_enabled() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return enabled_ && !overloaded_;
}

bool AlgoInstance::is_overloaded() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return overloaded_;
}

void AlgoInstance::clear_overload() {
    std::lock_guard<std::mutex> lk(mutex_);
    overloaded_ = false;
    flood_strikes_ = 0;
}

std::size_t AlgoInstance::total_pushed() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return total_pushed_;
}

std::size_t AlgoInstance::total_dropped() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return total_dropped_;
}

void AlgoInstance::push_events(const Metavision::EventCD* begin,
                               const Metavision::EventCD* end) {
    std::function<void()> tripped_cb;
    {
        std::lock_guard<std::mutex> lk(mutex_);
        const std::size_t n = static_cast<std::size_t>(end - begin);
        total_pushed_ += n;
        if (!enabled_ || overloaded_) {
            // Instance is disabled/overloaded — the entire batch is dropped.
            total_dropped_ += n;
            return;
        }
        if (backend_) {
            // Flood guard (design §5.6.7, rate-based, audit §五-E1): measure
            // the wall-clock event rate over a sliding 1s window. When a full
            // window completes, its rate is compared against
            // kMaxEventRateEvPerSec; kFloodStrikes consecutive over-threshold
            // windows auto-disable the instance to prevent memory blowup and
            // GUI stalls. Batches are never truncated — every event is
            // delivered until the guard trips. Using wall clock (not batch
            // size or event timestamps) makes file playback (~1-2 Mev/s) and
            // live streams subject to the same limit (previously the 50k/batch
            // cap mis-fired on file playback, where one push is a whole 33ms
            // window, and silently dropped window-front events).
            const auto now = std::chrono::steady_clock::now();
            if (rate_window_start_ == std::chrono::steady_clock::time_point{}) {
                rate_window_start_ = now;
            }
            rate_window_events_ += n;
            const double elapsed_s =
                std::chrono::duration<double>(now - rate_window_start_).count();
            if (elapsed_s >= 1.0) {
                const double rate =
                    static_cast<double>(rate_window_events_) / elapsed_s;
                if (rate > kMaxEventRateEvPerSec) {
                    if (++flood_strikes_ >= kFloodStrikes) {
                        overloaded_ = true;
                        enabled_ = false;
                        total_dropped_ += n;
                        tripped_cb = overload_callback_;
                    }
                } else {
                    flood_strikes_ = 0;
                }
                rate_window_events_ = 0;
                rate_window_start_ = now;
            }
            if (!overloaded_) {
                // Events arrive ROI-relative and conditioned from the shared
                // StreamConditioner (live: CameraController's CD callback;
                // file: FileFrameGenerator::render_frame) — the per-instance
                // crop+shift that used to live here computed the same
                // translation again for every backend. set_unified_roi still
                // resizes backends to the ROI dims.
                backend_->push_events(begin, end);
            }
        } else {
            // OpenEB 包装算法：透传（由 filter_chain 处理）。
        }
    }
    // Invoke the overload callback outside the instance lock so the receiver
    // (e.g. the AlgorithmsPanel, via a queued signal) can safely call back
    // into this instance (clear_overload, set_enabled) without deadlocking.
    if (tripped_cb) tripped_cb();
}

AlgoResult AlgoInstance::pull_result() {
    std::lock_guard<std::mutex> lk(mutex_);
    if (backend_) {
        return backend_->pull_result();
    }
    // 透传：返回空结果（OpenEB 算法由 filter_chain 处理）。
    AlgoResult r;
    r.status = "pass-through (openeb)";
    return r;
}

std::pair<const Metavision::EventCD*, std::size_t>
AlgoInstance::output_span() {
    std::lock_guard<std::mutex> lk(mutex_);
    if (backend_ && enabled_ && !overloaded_) {
        return backend_->output_span();
    }
    return {nullptr, 0};
}

void AlgoInstance::reset() {
    std::lock_guard<std::mutex> lk(mutex_);
    if (backend_) {
        backend_->reset();
    }
}

void AlgoInstance::set_sensor_dimensions(int width, int height) {
    std::lock_guard<std::mutex> lk(mutex_);
    // Track the source's full-sensor dims (source switch) so a later
    // set_unified_roi(false) restores the CURRENT sensor size, not the stale
    // creation size. While the unified ROI is active the effective dims stay
    // at the ROI window — the source switch must not silently resize the
    // backend out from under the crop (push_events still feeds
    // ROI-relative coordinates).
    create_w_ = width;
    create_h_ = height;
    if (uroi_enabled_) {
        width_ = uroi_x1_ - uroi_x0_;
        height_ = uroi_y1_ - uroi_y0_;
    } else {
        width_ = width;
        height_ = height;
    }
    if (backend_) {
        backend_->set_sensor_dimensions(width_, height_);
    }
}

void AlgoInstance::set_unified_roi(bool enabled, int x0, int y0, int x1, int y1) {
    std::lock_guard<std::mutex> lk(mutex_);
    uroi_enabled_ = enabled;
    uroi_x0_ = x0; uroi_y0_ = y0; uroi_x1_ = x1; uroi_y1_ = y1;
    if (!backend_) return;
    // The ROI window IS the effective sensor for the algorithm: resize the
    // backend to the ROI dims (§五-D1 rebuild path handles "rebuild only
    // when effective dims change"), restore creation dims when disabled.
    if (enabled) {
        width_ = x1 - x0;
        height_ = y1 - y0;
    } else {
        width_ = create_w_;
        height_ = create_h_;
    }
    backend_->set_sensor_dimensions(width_, height_);
}

// ---------------------------------------------------------------------------
// Small spec helpers
// ---------------------------------------------------------------------------

namespace {

AlgoParamSpec pint(const std::string& k, const std::string& disp,
                   const std::string& def, const std::string& lo,
                   const std::string& hi, const std::string& mf = "") {
    return {k, disp, "int", def, lo, hi, {}, mf};
}

AlgoParamSpec pfloat(const std::string& k, const std::string& disp,
                     const std::string& def, const std::string& lo,
                     const std::string& hi, const std::string& mf = "") {
    return {k, disp, "float", def, lo, hi, {}, mf};
}

AlgoParamSpec penum(const std::string& k, const std::string& disp,
                    const std::string& def, std::vector<std::string> vals,
                    const std::string& mf = "") {
    return {k, disp, "enum", def, "", "", std::move(vals), mf};
}

AlgoParamSpec pbool(const std::string& k, const std::string& disp,
                    const std::string& def, const std::string& mf = "") {
    return {k, disp, "bool", def, "", "", {}, mf};
}

AlgoParamSpec pstring(const std::string& k, const std::string& disp,
                      const std::string& def, const std::string& mf = "") {
    return {k, disp, "string", def, "", "", {}, mf};
}

/// Returns the shared preprocessing parameters (v1.0.9): a stackable noise
/// filter + 1/4 downsample applied AFTER the algorithm ROI, in the order
/// ROI → filter → downsample. These overlay on top of any main algorithm and
/// are NOT mutually exclusive with it. preproc_downsample defaults to "false"
/// (audit §五-F1): for most backends it only thins events (coordinates
/// unchanged), a silent 4× input loss for detection/tracking algorithms; the
///
/// NOTE (Phase 2.6): the shared roi_params() registration block was deleted
/// with the legacy per-backend ROI mechanism. The unified ROI (hardware /
/// software crop) is driven via AlgorithmsPanel::unified_roi_changed.
/// panel auto-enables it for coordinate-halving backends (§11.2-I).
/// preproc_filter_enabled defaults to "false" (opt-in). The preproc_filter_*
/// params mirror the standalone NoiseFilter params (§4.3.5) so the same 9
/// denoiser modes are available as a preprocessing stage. NOTE: the noise
/// defaults below (baf_dt 1000, stcf corr 0.005, dwf_wlen 2, agep_tau 3000,
/// harm Q 5, sbp_dt 10000, ...) are the intentional GUI working
/// points from design §4.3.5 — they deliberately differ from both jAER and
/// the algo member defaults. Do NOT "align" them. EXCEPTION (user
/// decision, 2.6 debug): rep_ratio_shorter/longer follow jAER (2/2).
std::vector<AlgoParamSpec> preproc_params() {
    return {
        pbool("preproc_filter_enabled", "Preproc: noise filter", "false"),
        pbool("preproc_downsample", "Preproc: 1/4 downsample", "false"),
        penum("preproc_filter_mode", "Preproc: filter mode", "8",
              {"0=BAF", "1=STCF", "2=Refractory", "3=DWF",
               "4=AgePolarity", "5=Harmonic", "6=Repetitious", "7=SpatialBP",
               "8=KNoise"}),
        // STCF (mode 1)
        pfloat("preproc_filter_correlation_time_s", "Preproc STCF corr (s)", "0.025", "0.001", "0.1"),
        pint("preproc_filter_min_neighbors", "Preproc STCF min nbr", "2", "1", "8"),
        pbool("preproc_filter_require_polarity_match", "Preproc STCF pol match", "true"),
        pbool("preproc_filter_allow_coincidence", "Preproc STCF coincide", "true"),
        // BAF (mode 0)
        pint("preproc_filter_baf_dt_us", "Preproc BAF dt (us)", "25000", "1000", "100000"),
        pint("preproc_filter_baf_subsample_by", "Preproc BAF subsample", "0", "0", "4"),
        // Refractory (mode 2)
        pint("preproc_filter_refractory_us", "Preproc Refractory (us)", "10000", "100", "100000"),
        // DWF (mode 3)
        // dwf_window_length upper bound raised to 1024 (audit §5-B3): the
        // jAER/design working point is 512, unreachable with the old max 100.
        // Defaults mirror the NoiseFilter member defaults (win len 64: an
        // O(wLen) scan per noise event makes 512 too slow at this sensor's
        // event rates — see noise_filter.h).
        pint("preproc_filter_dwf_window_length", "Preproc DWF win len", "64", "1", "1024"),
        pint("preproc_filter_dwf_dist_threshold", "Preproc DWF dist", "5", "1", "1024"),
        pint("preproc_filter_dwf_min_correlated", "Preproc DWF min corr", "1", "1", "8"),
        pbool("preproc_filter_dwf_double_mode", "Preproc DWF double", "true"),
        // AgePolarity (mode 4)
        pint("preproc_filter_agep_tau_us", "Preproc AgePol tau (us)", "10000", "1000", "100000"),
        pfloat("preproc_filter_age_threshold", "Preproc AgePol thresh", "5.0", "0.0", "8.0"),
        pint("preproc_filter_agep_radius", "Preproc AgePol radius", "1", "1", "5"),
        // Harmonic (mode 5)
        penum("preproc_filter_line_freq_hz", "Preproc Harmonic Hz", "50", {"50", "60"}),
        pfloat("preproc_filter_notch_q", "Preproc Harmonic Q", "3.0", "0.1", "100.0"),
        pfloat("preproc_filter_harmonic_threshold", "Preproc Harmonic thresh", "0.1", "0.0", "1.0"),
        // Repetitious (mode 6). rep_period_us/rep_tolerance_us are NOT
        // registered: the algo stores them but never uses them (audit §7.3).
        // ratio defaults aligned to jAER RepetitiousFilter (2/2, verified in
        // ref/jaer .../RepetitiousFilter.java:41,43) — the earlier 10/10 GUI
        // working point widened the "repetitious" ISI band 5× on both sides,
        // over-dropping quasi-periodic events (user decision, 2.6 debug).
        pint("preproc_filter_rep_ratio_shorter", "Preproc Rep ratio short", "2", "1", "100"),
        pint("preproc_filter_rep_ratio_longer", "Preproc Rep ratio long", "2", "1", "100"),
        pint("preproc_filter_rep_min_dt_to_store_us", "Preproc Rep min dt (us)", "1000", "0", "1000000"),
        pint("preproc_filter_rep_averaging_samples", "Preproc Rep avg samples", "3", "1", "100"),
        // SpatialBP (mode 7). Defaults are the cheap jAER working point
        // (center 0 / surround 1 → 8 splat writes per event); the previous
        // 2/10 splatted 416 cells per event and capped the filter at
        // ~2.4 Mev/s/core (see noise_filter.h).
        pint("preproc_filter_sbp_center_radius_px", "Preproc SBP center", "0", "0", "10"),
        pint("preproc_filter_sbp_surround_radius_px", "Preproc SBP surround", "1", "1", "30"),
        pint("preproc_filter_sbp_dt_surround_us", "Preproc SBP dt (us)", "8000", "100", "1000000"),
        // KNoise (mode 8) — dv-processing KNoiseFilter port; dt aligned to
        // the dv default 2000 us (keep-rate on sparklers.raw is flat in dt,
        // see algo/cv/noise_filter.h).
        pint("preproc_filter_knoise_dt_us", "Preproc KNoise dt (us)", "2000", "100", "100000"),
        // Cross-mode flags
        pbool("preproc_filter_filter_hot_pixels", "Preproc filter hot px", "false"),
        pbool("preproc_filter_adaptive_correlation_time", "Preproc adaptive corr", "false"),
    };
}

} // namespace

// ---------------------------------------------------------------------------
// AlgoBridge
// ---------------------------------------------------------------------------

AlgoBridge::AlgoBridge() {
    register_openeb_filters();
    register_self_cv();
    register_self_analytics();
}

std::vector<AlgoInfo> AlgoBridge::list_algos() const {
    std::vector<AlgoInfo> out;
    out.reserve(registry_.size());
    for (const auto& kv : registry_) {
        out.push_back(kv.second);
    }
    return out;
}

const AlgoInfo* AlgoBridge::find(const std::string& name) const {
    auto it = registry_.find(name);
    return it == registry_.end() ? nullptr : &it->second;
}

std::shared_ptr<AlgoInstance> AlgoBridge::create(const std::string& name) {
    auto it = registry_.find(name);
    if (it == registry_.end()) {
        return nullptr;
    }
    return create_with_info(it->second);
}

std::shared_ptr<AlgoInstance> AlgoBridge::create_with_info(const AlgoInfo& info) {
    const auto& name = info.name;
    auto inst = std::make_shared<AlgoInstance>(info, sensor_w_, sensor_h_);
    // Wire the flood-guard overload callback (§五-E2): when the guard trips,
    // the registered receiver (AlgorithmsPanel) unchecks the sidebar checkbox.
    if (overload_cb_) {
        auto cb = overload_cb_;
        inst->set_overload_callback([cb, name]() { cb(name); });
    }
    {
        std::lock_guard<std::mutex> lk(live_mutex_);
        live_instances_[name] = inst;
        // Replay per-algorithm cached params (N1) FIRST: values loaded from
        // a config file for an algorithm that had no live instance. These
        // include preproc_*/roi_* keys (because capture_algo_state saves
        // all info.params). Then replay the global preproc_cache_/roi_cache_
        // SECOND so they override the config's preproc_*/roi_* values —
        // the user's latest global modifications take precedence over stale
        // config values (BUG-1: N1+N6 interaction). Per-algo params like
        // model_path/mode are unaffected since the global caches don't
        // contain those keys.
        auto pit = algo_param_cache_.find(name);
        if (pit != algo_param_cache_.end()) {
            for (const auto& [k, v] : pit->second) {
                inst->set_param(k, v);
            }
            algo_param_cache_.erase(pit);
        }
        if (info.source == "self") {
            for (const auto& [k, v] : preproc_cache_) {
                inst->set_param(k, v);
            }
            // roi_cache_ replay intentionally stopped (Phase 2.6): the
            // unified ROI (hardware/software crop) is the single ROI
            // concept; per-backend roi_* replay would re-enable the legacy
            // per-backend ROI being removed in step 2.
        }
        // Replay the cached unified ROI state (Phase 2.6) so instances
        // created while a ROI window is active start out cropped/resized.
        // RONI (debug D-5): pass-through at full dims (source-filtered
        // absolute coordinates).
        if (info.category != "calibration") {
            inst->set_unified_roi(uroi_enabled_ && !uroi_roni_,
                                  uroi_x0_, uroi_y0_, uroi_x1_, uroi_y1_);
        }
    }
    return inst;
}

std::shared_ptr<AlgoInstance> AlgoBridge::find_or_create(const std::string& name) {
    if (auto existing = find_live(name)) {
        return existing;
    }
    return create(name);
}

std::shared_ptr<AlgoInstance> AlgoBridge::find_or_create_with_info(const AlgoInfo& info) {
    if (auto existing = find_live(info.name)) {
        return existing;
    }
    return create_with_info(info);
}

void AlgoBridge::set_sensor_dimensions(int width, int height) {
    sensor_w_ = (width > 0) ? width : 1280;
    sensor_h_ = (height > 0) ? height : 720;
}

void AlgoBridge::apply_global_preproc(const std::string& key,
                                      const std::string& value) {
    // Apply the preproc_* parameter to every live self-developed instance.
    // OpenEB event-transform stages (source=="openeb") have backend_==nullptr
    // (pass-through to FilterChain); preproc_* params have no effect on them
    // and would pollute their param_values_ map, so they are skipped.
    //
    // Threading (audit §五-C2): set_param can be heavyweight (e.g. E2VID
    // rebuilds reload the ONNX model). Take a snapshot under live_mutex_ and
    // apply the params OUTSIDE the lock so list_live/find_live callers are
    // not blocked for the duration.
    std::vector<std::shared_ptr<AlgoInstance>> snapshot;
    {
        std::lock_guard<std::mutex> lk(live_mutex_);
        // Cache the value so instances created later (by other code paths)
        // inherit the shared preprocessing state (BUG-R4).
        preproc_cache_[key] = value;
        for (auto it = live_instances_.begin(); it != live_instances_.end(); ) {
            if (auto inst = it->second.lock()) {
                // Skip calibration algorithms — they don't use preproc params
                // and would just accumulate pollution in param_values_ (BUG-R8).
                if (inst->info().source == "self" &&
                    inst->info().category != "calibration") {
                    snapshot.push_back(std::move(inst));
                }
                ++it;
            } else {
                it = live_instances_.erase(it);
            }
        }
    }
    for (auto& inst : snapshot) {
        inst->set_param(key, value);
    }
}

// Phase 2.6: AlgoBridge::apply_global_roi and roi_cache_ were deleted with
// the legacy per-backend ROI mechanism. The unified ROI is driven via
// AlgorithmsPanel::unified_roi_changed -> CameraController::set_unified_roi
// (display/record path) and AlgoBridge::set_unified_roi_state (algo path).

void AlgoBridge::set_unified_roi_state(bool enabled, int x0, int y0,
                                       int x1, int y1, bool roni) {
    // Cache the state so instances created later inherit it (same pattern as
    // apply_global_preproc). Snapshot under live_mutex_, apply outside the
    // lock — set_unified_roi may trigger a backend rebuild (e.g. E2VID
    // reloads the ONNX model when effective dims change).
    //
    // RONI (Phase 2.6 debug D-5): the source (hardware I_ROI RONI mode, or
    // the file software crop) already drops inside-rect events at ABSOLUTE
    // coordinates, so instances must NOT crop/translate/resize — they get
    // set_unified_roi(false) = pass-through at full dimensions.
    const bool inst_enabled = enabled && !roni;
    std::vector<std::shared_ptr<AlgoInstance>> snapshot;
    {
        std::lock_guard<std::mutex> lk(live_mutex_);
        uroi_enabled_ = enabled;
        uroi_roni_ = roni;
        uroi_x0_ = x0; uroi_y0_ = y0; uroi_x1_ = x1; uroi_y1_ = y1;
        for (auto it = live_instances_.begin(); it != live_instances_.end(); ) {
            if (auto inst = it->second.lock()) {
                // Calibration instances manage their own geometry and must
                // not be resized/cropped by the global ROI (same exemption
                // as apply_global_preproc).
                if (inst->info().category != "calibration") {
                    snapshot.push_back(std::move(inst));
                }
                ++it;
            } else {
                it = live_instances_.erase(it);
            }
        }
    }
    for (auto& inst : snapshot) {
        inst->set_unified_roi(inst_enabled, x0, y0, x1, y1);
    }
}

void AlgoBridge::cache_algo_params(
    const std::string& name,
    const std::map<std::string, std::string>& params) {
    // Store per-algorithm params for an instance that is not yet live (N1).
    // create() will replay these when the instance is eventually created.
    std::lock_guard<std::mutex> lk(live_mutex_);
    algo_param_cache_[name] = params;
}

std::optional<std::string> AlgoBridge::get_cached_algo_param(
    const std::string& name, const std::string& key) const {
    std::lock_guard<std::mutex> lk(live_mutex_);
    const auto ait = algo_param_cache_.find(name);
    if (ait == algo_param_cache_.end()) return std::nullopt;
    const auto pit = ait->second.find(key);
    if (pit == ait->second.end()) return std::nullopt;
    return pit->second;
}

std::shared_ptr<AlgoInstance> AlgoBridge::find_live(const std::string& name) {
    std::lock_guard<std::mutex> lk(live_mutex_);
    auto it = live_instances_.find(name);
    if (it == live_instances_.end()) return nullptr;
    auto inst = it->second.lock();
    if (!inst) {
        live_instances_.erase(it);
    }
    return inst;
}

std::vector<std::shared_ptr<AlgoInstance>> AlgoBridge::list_live() {
    std::vector<std::shared_ptr<AlgoInstance>> out;
    std::lock_guard<std::mutex> lk(live_mutex_);
    for (auto it = live_instances_.begin(); it != live_instances_.end(); ) {
        if (auto inst = it->second.lock()) {
            out.push_back(std::move(inst));
            ++it;
        } else {
            it = live_instances_.erase(it);
        }
    }
    return out;
}

void AlgoBridge::set_overload_callback(
    std::function<void(const std::string& name)> cb) {
    overload_cb_ = std::move(cb);
    // Retro-wire instances that already exist (created before the panel
    // registered the callback).
    for (auto& inst : list_live()) {
        if (overload_cb_) {
            auto cb = overload_cb_;
            const std::string name = inst->info().name;
            inst->set_overload_callback([cb, name]() { cb(name); });
        } else {
            inst->set_overload_callback(nullptr);
        }
    }
}

// ---------------------------------------------------------------------------
// OpenEB-wrapped event-transform stages (design §4.3.1). These stay
// registered so the FilterChain stages appear in config capture and keep a
// stable name space. The former openeb_frame / openeb_preproc / openeb_util
// registrations and their backends were removed (audit §三-B6/7, §五-A2):
// they had no GUI entry point and were unreachable dead code.
// ---------------------------------------------------------------------------

void AlgoBridge::register_openeb_filters() {
    auto add = [&](AlgoInfo a) {
        a.source = "openeb";
        a.category = "openeb_filter";
        registry_[a.name] = std::move(a);
    };

    add({"polarity_filter", "Polarity Filter", "openeb_filter", "openeb",
         AlgoDisplayMode::Passive,
         {penum("polarity", "Polarity", "1", {"0=OFF", "1=ON"})}});

    add({"polarity_invert", "Polarity Invert", "openeb_filter", "openeb",
         AlgoDisplayMode::Passive, {}});

    add({"flip_x", "Flip X", "openeb_filter", "openeb",
         AlgoDisplayMode::Passive, {}});

    add({"flip_y", "Flip Y", "openeb_filter", "openeb",
         AlgoDisplayMode::Passive, {}});

    // rotate / transpose / rescale were removed (2026-08-18): their
    // coordinate changes are not propagated to the downstream W×H buffers,
    // causing out-of-bounds writes (heap corruption / delayed segfault).
}

// ---------------------------------------------------------------------------
// Self-developed CV algorithms (design §4.3.5 - §4.3.27)
// 21 modules, all with real algo_backend wiring.
// ---------------------------------------------------------------------------

void AlgoBridge::register_self_cv() {
    auto add = [&](AlgoInfo a) {
        a.source = "self";
        a.category = "cv";
        // All self-developed CV algorithms support ROI (design §5.6.6) and the
        // shared preprocessing stage (v1.0.9: ROI → filter → downsample).
        // (Phase 2.6: shared roi_params() block removed — unified ROI.)
        for (auto& p : preproc_params()) a.params.push_back(std::move(p));
        registry_[a.name] = std::move(a);
    };

    // §4.3.5 Noise Filter — removed as a standalone algorithm in v1.0.9; the
    // noise filter is now a stackable preprocessing stage (see preproc_params)
    // shared by all self-developed algorithms (ROI → filter → downsample).

    // §4.3.6 Hot Pixel Filter. n_sigma is NOT registered: the algo marks it
    // "deprecated, unused" — exposing it would be a no-op control (§三-31).
    add({"hot_pixel_filter", "Hot Pixel Filter", "cv", "self",
         AlgoDisplayMode::Passive,
         {pfloat("learning_window_s", "Learning window (s)", "5.0", "0.1", "60.0"),
          pbool("enable_fpn_correction", "FPN correction", "false"),
          pfloat("fpn_alpha", "FPN alpha", "0.9", "0.01", "1.0"),
          pfloat("fpn_mixing_factor", "FPN mixing factor", "0.01", "0.0001", "1.0")}});

    // §4.3.7 Orientation Filter (jAER SimpleOrientationFilter min-dt WTA)
    add({"orientation_filter", "Orientation Filter", "cv", "self",
         AlgoDisplayMode::Overlay,
         // tau_us / min_neighbors / multi_ori_output / pass_all_events were
         // removed (audit §二-2.7): dead params in the algo, never effective.
         {pint("min_dt_threshold_us", "Min dt threshold (us)", "100000", "1", "1000000"),
          pbool("use_average_dt", "Use average dt", "true"),
          pbool("ori_history_enabled", "Ori history smoothing", "false"),
          pint("dt_reject_threshold_us", "Dt reject threshold (us)", "100000", "1", "10000000")}});

    // §4.3.8 Direction Selective Filter (jAER DirectionSelectiveFilter)
    add({"direction_selective", "Direction Selective Filter", "cv", "self",
         AlgoDisplayMode::Overlay,
         {pint("tau_us", "Time window (us)", "10000", "1000", "50000"),
          pint("min_dt_us", "Min dt (us)", "100", "0", "1000000"),
          pint("search_distance", "Search distance (px)", "3", "1", "12"),
          pint("tau_low_ms", "Tau low-pass (ms)", "100", "1", "100000"),
          pbool("enable_global_mode", "Global motion mode", "true"),
          // Embedded orientation pre-stage (jAER enclosed SimpleOrientationFilter)
          pint("ori_min_dt_threshold_us", "Ori min dt (us)", "1000", "0", "1000000"),
          pint("ori_dt_reject_threshold_us", "Ori dt reject (us)", "200000", "0", "1000000")}});

    // §4.3.9 Sparse Optical Flow (4 modes: LocalPlanes/LucasKanade/BlockMatch/ClusterOF)
    // mode_filter scopes each param to its mode(s) (Phase 7 P7): the panel
    // shows only what the selected mode consumes. pps_scale (arrow length =
    // speed * pps_scale, jAER rbodo drawVector) is common.
    add({"sparse_optical_flow", "Sparse Optical Flow", "cv", "self",
         AlgoDisplayMode::Overlay,
         {penum("mode", "Mode", "0", {"0=LocalPlanes", "1=LucasKanade",
           "2=BlockMatch", "3=ClusterOF"}),
          pfloat("pps_scale_lp", "Arrow scale (px per px/s)", "0.003", "0.0001", "1.0", "0"),
          pfloat("pps_scale_lk", "Arrow scale (px per px/s)", "0.7", "0.0001", "1.0", "1"),
          pfloat("pps_scale_bm", "Arrow scale (px per px/s)", "0.09", "0.0001", "1.0", "2"),
          pfloat("pps_scale_co", "Arrow scale (px per px/s)", "0.05", "0.0001", "1.0", "3"),
          pint("arrow_grid_px", "Arrow grid (px)", "24", "4", "64"),
          pint("time_window_us", "Time window (us)", "20000", "1000", "100000", "0,1,3"),
          pint("spatial_radius", "Spatial radius (px)", "8", "3", "30", "0,3"),
          pint("min_events_per_cluster", "Min events/cluster", "10", "1", "1000", "0,3"),
          pint("search_radius", "Search radius (px)", "4", "1", "16", "1,2"),
          pfloat("lk_thr", "LK lambda threshold", "1.0", "0", "1000", "1"),
          pfloat("cluster_ema_alpha", "Cluster EMA alpha", "0.05", "0.001", "1.0", "3"),
          pint("bm_time_window_us", "BM time window (us)", "20000", "1000", "100000", "2"),
          pint("downsample_factor", "BM downsample factor", "2", "1", "8", "2"),
          pint("num_scales", "BM num scales", "3", "1", "5", "2"),
          pint("max_slice_value", "BM max slice value", "15", "1", "255", "2"),
          pfloat("valid_pix_occupancy", "BM valid occupancy", "0.01", "0", "1", "2"),
          pfloat("weight_distance", "BM distance weight", "0.95", "0", "1", "2")}});

    // §4.3.9b Dense Optical Flow (3 modes, from published methods)
    add({"dense_optical_flow", "Dense Optical Flow", "cv", "self",
         AlgoDisplayMode::Overlay,
         {penum("mode", "Mode", "0", {"0=PlaneFitting", "1=TimeGradient",
           "2=TripletMatching"}),
          pfloat("pps_scale_pf", "Arrow scale (px per px/s)", "0.002", "0.0001", "1.0", "0"),
          pfloat("pps_scale_tg", "Arrow scale (px per px/s)", "0.007", "0.0001", "1.0", "1"),
          pfloat("pps_scale_tm", "Arrow scale (px per px/s)", "0.009", "0.0001", "1.0", "2"),
          pint("arrow_grid_px", "Arrow grid (px)", "24", "4", "64"),
          pint("time_window_us", "Time window (us)", "10000", "1000", "500000"),
          pint("spatial_radius", "Spatial radius (px)", "3", "1", "16"),
          pfloat("max_velocity_px_s", "Max velocity (px/s)", "20000", "100", "100000")}});

    // §4.3.x DL dense optical flow — EV-FlowNet architecture (Zhu et al.
    // 2018) retrained by Stoffregen et al. ECCV 2020 (event_cnn_minimal).
    // One 5-bin voxel per accumulation window → per-pixel (u, v) velocity,
    // HSV-coded frame (hue = direction, value = speed) in a Standalone
    // window. Inference reuses the E2VID runtime selection (OpenVINO iGPU →
    // ONNX Runtime CPU). Model conversion:
    // models/convert_event_cnn_minimal_to_onnx.py --model evflownet.
    add({"dl_optical_flow", "Dense Optical Flow (DL)", "cv", "self",
         AlgoDisplayMode::Standalone,
         {pstring("model_path", "Model path (ONNX)", "models/evflownet.onnx"),
          penum("device", "Inference device", "0",
                {"0=Auto (iGPU if available)", "1=CPU", "2=GPU"}),
          pint("output_fps", "Output fps", "30", "1", "120"),
          pfloat("max_velocity_px_s", "HSV speed scale (px/s, 0=auto)", "0",
                 "0", "100000")}});

    // §4.3.10 Blob Detector. Default threshold 5: the foreground is the
    // per-pixel event count above the EMA background over a 33 ms window, so
    // a default of 50 demanded ~50 events/pixel and small/moving blobs never
    // produced a box.
    add({"blob_detector", "Blob Detector", "cv", "self",
         AlgoDisplayMode::Overlay,
         {pfloat("threshold", "Threshold", "5", "1", "254"),
          pfloat("learning_rate", "Learning rate", "0.05", "0.001", "1.0"),
          pfloat("accumulation_ms", "Accumulation window (ms)", "33.3", "1", "1000"),
          pint("min_area", "Min blob area (px)", "10", "1", "100000")}});

    // §4.3.11 Object Tracker (4 modes, jAER RectangularClusterTracker)
    add({"object_tracker", "Object Tracker", "cv", "self",
         AlgoDisplayMode::Overlay,
         {pfloat("cluster_size_fraction", "Cluster size (fraction of sensor)", "0.036", "0.005", "0.5"),
          pint("cluster_time_us", "Cluster time (us)", "5000", "1000", "50000"),
          pint("min_cluster_events", "Min cluster events", "50", "10", "500"),
          pfloat("max_lost_age_s", "Max lost age (s)", "1.0", "0.1", "5.0"),
          pbool("enable_velocity_prediction", "Velocity prediction", "true"),
          pfloat("location_mixing_factor", "Location mixing factor", "0.05", "0.0", "1.0"),
          pfloat("predictive_velocity_factor", "Predictive velocity factor", "1.0", "0.0", "10.0"),
          pint("mass_decay_tau_us", "Mass decay tau (us)", "10000", "1", "1000000"),
          pfloat("threshold_mass_for_visible", "Visible mass threshold", "30", "0", "1000000"),
          pint("max_clusters", "Max clusters", "10", "1", "100")}});

    // §4.3.12 Corner Detector (4 modes). The enum labels must match
    // algo/cv/corner_detector.h's Mode enum order exactly (the backend maps
    // the index via static_cast) — previously mislabelled (§五-A1): the old
    // labels {"Harris","FAST","AGAST"} ran EndStopped/TypeCoincidence/Harris.
    // Mode 3 = Arc (dv-processing Arc* port, ring radii 3/4).
    add({"corner_detector", "Corner Detector", "cv", "self",
         AlgoDisplayMode::Overlay,
         {penum("mode", "Mode", "0", {"0=EndStopped", "1=TypeCoincidence", "2=Harris",
           "3=Arc"}),
          pfloat("min_score", "Min score", "0.1", "0", "1.0"),
          pint("arc_corner_range_us", "Arc corner range (us)", "5000", "100", "1000000", "3"),
          pfloat("arc_min_response_us", "Arc min response (us)", "1", "0", "1000000", "3")}});

    // §4.3.13 Line Segment Detector (ELiSeD)
    add({"line_segment", "Line Segment (ELiSeD)", "cv", "self",
         AlgoDisplayMode::Overlay,
         {pint("min_length", "Min length (px)", "20", "3", "100"),
          pint("gap", "Max gap (px)", "5", "1", "20")}});

    // §4.3.14 Hough Line Tracker (jAER HoughLineTracker). accumulator_decay_us
    // is NOT registered: the algo stores it but never uses it (§7.3).
    add({"hough_line", "Hough Line Tracker", "cv", "self",
         AlgoDisplayMode::Overlay,
         {pint("threshold", "Threshold", "30", "2", "500"),
          pint("num_theta_bins", "Theta bins", "90", "8", "360"),
          pint("num_rho_bins", "Rho bins (0=auto)", "0", "0", "4000"),
          pfloat("hough_decay_factor", "Per-packet decay factor", "0.6", "0.0", "1.0"),
          pfloat("output_tau_ms", "Output smoothing tau (ms)", "10", "1", "1000"),
          pfloat("favor_vertical_range_deg", "Favor vertical range (deg)", "90", "5", "90")}});

    // §4.3.15 Hough Circle Tracker (jAER HoughCircleTracker) — tightened
    // defaults to reduce lag: narrower radius range (8-30 → 23 radii vs
    // 5-50 → 46) and higher threshold (50 vs 30) so find_peaks scans fewer
    // candidates. min_radius and accumulator_decay_us are NOT registered:    // Hough Circle (jAER HoughCircleTracker port). NOTE: "radius" is the
    // SINGLE fixed circle radius (jAER `radius`) — there is no multi-radius
    // sweep; until set explicitly the backend derives it from the working
    // resolution (min(w,h)/8). min_radius and accumulator_decay_us are NOT
    // registered: the algo never reads them (§三-32).
    add({"hough_circle", "Hough Circle Tracker", "cv", "self",
         AlgoDisplayMode::Overlay,
         {pint("radius", "Radius (px)", "30", "5", "500"),
          pint("threshold", "Threshold", "15", "2", "500"),
          pfloat("decay", "Decay factor", "1.0", "0.0", "10.0"),
          pint("buffer_length", "Buffer length", "4000", "100", "100000"),
          pint("nr_max", "Max circles", "1", "1", "20"),
          pbool("decay_mode", "Decay mode", "true"),
          pbool("loc_depression", "Local depression", "true")}});

    // §4.3.17 Orientation Cluster. min_events is NOT registered: the algo
    // marks it "Stored, unused" (§五-A3) — exposing it was a no-op control.
    add({"orientation_cluster", "Orientation Cluster", "cv", "self",
         AlgoDisplayMode::Overlay,
         {pfloat("dt", "dt (us)", "10000", "100", "1000000"),
          pfloat("factor", "Factor", "1000", "1", "100000"),
          pint("rf_width", "RF width", "1", "1", "64"),
          pint("rf_height", "RF height", "1", "1", "64"),
          pfloat("tolerance", "Tolerance (deg)", "10", "0", "180"),
          pfloat("ori", "Ori (deg)", "45", "0", "180"),
          pfloat("neighbor_thr", "Neighbor thr", "10", "0", "1000"),
          pfloat("thr_gradient", "Thr gradient", "0", "0", "100"),
          pfloat("history_factor", "History factor", "1", "0", "10"),
          pbool("use_opposite_polarity", "Use opposite polarity", "true"),
          pbool("ori_history_enabled", "Ori history enabled", "false"),
          pint("display_length", "Display length", "10", "1", "500")}});

    // §4.3.18 Cluster LIF
    add({"cluster_lif", "Cluster LIF", "cv", "self",
         AlgoDisplayMode::Overlay,
         {pfloat("tau_ms", "Tau (ms)", "22", "1", "1000"),
          pfloat("threshold", "Threshold", "15", "0.1", "1000"),
          pint("receptive_field_size_pixels", "Receptive field (px)", "8", "2", "128"),
          pfloat("initial_potential_percent", "Initial potential (%)", "50", "0", "100"),
          pfloat("jump_after_firing_percent", "Jump after firing (%)", "10", "0", "100")}});

    // §4.3.19 Background Mask Filter. The "learning" control maps to the
    // algo's learning window in seconds (set_learning_window_s), not a rate —
    // named accordingly (§五-B2); default 5s matches the jAER working point.
    // The backend still accepts the old "learning_rate" key as an alias, and
    // ConfigManager migrates it on load (§11.2-G).
    add({"background_mask", "Background Mask Filter", "cv", "self",
         AlgoDisplayMode::Replace,
         {pfloat("learning_window_s", "Learning window (s)", "5.0", "0.1", "60.0"),
          pfloat("threshold", "Threshold", "10", "1", "100"),
          pint("erosion_size", "Erosion size", "0", "0", "20")},
         /*description=*/"", /*uses_algo_roi=*/true,
         /*reset_on_reenable=*/true});

    // §4.3.20 Perspective Undistort — removed (§三-B8): the backend never
    // wired calibration into the algo (always a no-op) and the shared
    // Preprocessor undistort stage (preproc_undistort_*) supersedes it.

    // §4.3.21 Trigger Synced Filter removed (2026-08-22, user decision):
    // faithful jAER FilterSyncedEvents port, but the GUI never wired an
    // external trigger source, so the output was always empty (§5-G3).

    // §4.3.22 Bandpass Filter removed (2026-08-22, user decision):
    // self-invented combination (jAER has no standalone bandpass algorithm —
    // LowpassFilter/HighpassFilter are internal tracker utilities there);
    // its only output was a single event-rate text label. The jAER-equivalent
    // building blocks remain in algo/common/filter/.

    // §4.3.23 Optical Gyro (EIS) — Overlay: draws translation/rotation
    // vectors on top of the stabilized event display (jAER OpticalGyro
    // annotates the GL canvas with motion vectors).
    add({"optical_gyro", "EIS (Optical Gyro)", "cv", "self",
         AlgoDisplayMode::Overlay,
         {pbool("stabilize", "Stabilize", "true"),
          pbool("rotation_enabled", "Rotation estimation", "false"),
          pfloat("smoothing_window_ms", "Smoothing window (ms)", "100", "10", "1000")}});

    // §4.3.24 Ultra Slow Motion — REMOVED (Phase 2.5): its Replace-display
    // promise was never implemented, and time dilation is meaningless as a
    // GUI display without a downstream slow-motion consumer.

    // §4.3.25 XYT Visualizer. max_points is NOT registered: the backend only
    // stored it and the 3D display uses SpaceTimeDisplay's own XYTVisualizer
    // instance, so the control had no effect on anything (§五-A3).
    add({"xyt_visualizer", "XYT 3D Visualizer", "cv", "self",
         AlgoDisplayMode::Standalone,
         {pint("time_window_us", "Time window (us)", "500000", "10000", "10000000")},
         /*description=*/"", /*uses_algo_roi=*/false});

    // §4.3.26 Overlay removed (2026-08-22, user decision): the backend was a
    // pure pass-through placeholder — event coloring is owned by the Display
    // panel's frame mode / color theme, so the checkbox did nothing.

    // §4.3.27 Time Surface
    // decay is the mode selector (first param): Linear reads decay_time_us,
    // Exponential reads tau_us (time_surface.h render() branches on decay_).
    // mode_filter ties each time param to its decay mode so the user can't
    // edit a parameter the renderer ignores — the root cause of "changing
    // decay time has no effect in Exp mode".
    add({"time_surface", "Time Surface", "cv", "self",
         AlgoDisplayMode::Standalone,
         {penum("decay", "Decay", "0", {"0=Linear", "1=Exponential"}),
          pint("decay_time_us", "Decay time (us)", "100000", "10000", "5000000", "0"),
          pint("tau_us", "Tau (us)", "100000", "10000", "5000000", "1"),
          penum("palette", "Palette", "1", {"0=Gray", "1=Hot", "2=Plasma", "3=Turbo"}),
          penum("channels", "Channels", "1", {"1=merged", "2=split"}),
          pint("refresh_rate_hz", "Refresh rate (Hz)", "30", "10", "120")}});
}

// ---------------------------------------------------------------------------
// Self-developed analytics (design §4.4) — 7 modules
// ---------------------------------------------------------------------------

void AlgoBridge::register_self_analytics() {
    auto add = [&](AlgoInfo a) {
        a.source = "self";
        a.category = "analytics";
        // Shared preprocessing stage (v1.0.9: ROI → filter → downsample).
        // (Phase 2.6: shared roi_params() block removed — unified ROI.)
        for (auto& p : preproc_params()) a.params.push_back(std::move(p));
        registry_[a.name] = std::move(a);
    };

    // §4.4.1 Active Marker removed (2026-08-22, user decision):
    // fully self-invented (only "inspired by" the Lighthouse scheme — it
    // borrowed the heatmap+CC stage that Lighthouse itself later replaced
    // with Goertzel frequency-identity detection; no tracking, no IDs).

    // §4.4.2 Event To Video (6 modes). Parameters are mode-scoped via
    // mode_filter: the UI shows only the params that apply to the currently
    // selected mode (BardowVariational=0, InteractingMaps=1, E2VID=2,
    // E2VID+=3, FireNet+=4, HyperE2VID=5). Common params (mode, output_fps)
    // have an empty mode_filter. Modes 2..5 share the same DL inference
    // pipeline (and the device/num_bins/postproc rows) and differ only in
    // the loaded ONNX weights — each mode has its own model path so several
    // weight sets can be installed side by side.
    add({"event_to_video", "Event -> Video (E2VID)", "analytics", "self",
         AlgoDisplayMode::Standalone,
         {penum("mode", "Mode", "2",
                {"0=BardowVariational", "1=InteractingMaps", "2=E2VID",
                 "3=E2VID+", "4=FireNet+", "5=HyperE2VID"}),
          pint("output_fps", "Output fps", "30", "1", "120"),
          // --- Shared non-DL params (mode 0,1) ---
          pfloat("window_ms", "Window (ms)", "50", "10", "500", "0,1"),
          // decay_tau_ms: exponential dimming of the reconstructed frame
          // between output frames (event_to_video.h get_frame); 0 = off.
          pfloat("decay_tau_ms", "Frame dimming tau (ms)", "500", "0", "5000", "0,1"),
          // --- BardowVariational (mode 0) ---
          pfloat("delta_t_ms", "Delta t (ms)", "15", "1", "50", "0"),
          pfloat("theta", "Theta", "0.22", "0.05", "0.5", "0"),
          pint("num_iterations", "TV iterations", "100", "10", "500", "0"),
          pfloat("lambda1", "Lambda1 (TV weight)", "0.02", "0.0", "1.0", "0"),
          pfloat("lambda2", "Lambda2", "0.05", "0.0", "1.0", "0"),
          pfloat("lambda3", "Lambda3 (TV weight)", "0.02", "0.0", "1.0", "0"),
          pfloat("lambda4", "Lambda4", "0.2", "0.0", "2.0", "0"),
          pfloat("lambda5", "Lambda5", "0.1", "0.0", "1.0", "0"),
          pfloat("lambda6", "Lambda6 (prior)", "1.0", "0.0", "2.0", "0"),
          // --- InteractingMaps (mode 1) ---
          pfloat("relaxation_step", "Relaxation step", "0.1", "0.001", "0.5", "1"),
          pint("im_iterations", "Relax iterations", "50", "10", "1000", "1"),
          pfloat("fov_deg", "Camera FOV (deg)", "60", "10", "170", "1"),
          // --- DL reconstruction modes (2..5) ---
          pstring("model_path", "Model path (ONNX)", "models/e2vid_lightweight.onnx", "2"),
          pstring("e2vid_plus_model_path", "Model path (ONNX)", "models/e2vid_plus.onnx", "3"),
          pstring("firenet_plus_model_path", "Model path (ONNX)", "models/firenet_plus.onnx", "4"),
          pstring("hypere2vid_model_path", "Model path (ONNX)", "models/hypere2vid.onnx", "5"),
          // Inference device: Auto runs the neural path on the Intel iGPU via
          // OpenVINO when the build has it and a GPU is present (measured
          // ~11× faster than CPU); CPU/GPU pin the respective runtime, with
          // silent CPU degradation when the GPU path is unavailable.
          penum("device", "Inference device", "0",
                {"0=Auto (iGPU if available)", "1=CPU", "2=GPU"}, "2,3,4,5"),
          pint("num_bins", "Num bins", "5", "1", "20", "2,3,4,5"),
          pbool("auto_hdr", "Auto HDR", "false", "2,3,4,5"),
          // 1/4 downsample is now a shared preprocessing stage
          // (preproc_downsample) — removed the per-algo downsample param.
          pfloat("unsharp_amount", "Unsharp amount", "0.3", "0.0", "2.0", "2,3,4,5"),
          pfloat("unsharp_sigma", "Unsharp sigma", "1.0", "0.1", "5.0", "2,3,4,5"),
          pfloat("bilateral_sigma", "Bilateral sigma", "0.0", "0.0", "10.0", "2,3,4,5")}});

    // §4.4.4 ISI Analyzer removed (2026-08-22, user decision):
    // fully self-invented per-pixel ISI histogram (loosely inspired by jAER
    // ISIHistogrammer); the frequency analytics role is now covered by
    // freq_detector / frequency_map.

    // §4.4.3 Flow Statistics removed (2026-08-22, user decision): the
    // evaluation core needed ground truth that a live stream cannot provide,
    // so the module had degenerated to an event-count status line.
    // §4.4.5 Particle Counter removed (2026-08-22, user decision):
    // fully self-invented (only the conveyor-counting scenario was inspired
    // by jAER particlecounter); superseded by blob_detector + tracking.

    // §4.4.6 Auto Bias Controller moved out of the algorithm pipeline
    // (2026-08-21): it is a camera-level feature now (CameraController +
    // Biases panel) so it no longer occupies the single algorithm slot and
    // coexists with every algorithm.

    // §4.4.7 Freq Detector — overlay boxes + frequency labels on the main
    // display (centered usage hint during the initialization phase). Full
    // reference parameter set. reset_on_reenable: the warm-up window,
    // heatmap and frozen result are all stale after a disabled gap — a new
    // measurement session must start from scratch.
    add({"freq_detector", "Frequency Detector", "analytics", "self",
         AlgoDisplayMode::Overlay,
         {pfloat("fmin", "Min event freq (Hz)", "100", "10", "1000"),
          pfloat("fmax", "Max event freq (Hz)", "10000", "1000", "50000"),
          pfloat("bin_dt_us", "Time bin (us)", "50", "10", "1000"),
          pint("heatmap_threshold", "Heatmap threshold", "50", "1", "1000"),
          pint("min_events", "Min cluster area (px)", "3", "1", "100"),
          pint("region_radius", "Region radius (px)", "1", "0", "5"),
          pfloat("peak_alpha", "Peak threshold alpha", "5.0", "1.0", "20.0"),
          pfloat("first_analysis", "Init phase (s)", "5.0", "0.5", "10.0"),
          pfloat("max_duration", "Max window (s)", "20.0", "5.0", "120.0"),
          pfloat("update_interval_s", "Update interval (s)", "2.0", "0.1", "10.0")},
         /*description=*/"", /*uses_algo_roi=*/false,
         /*reset_on_reenable=*/true});

    // §4.3.x Per-pixel frequency/period map + frequency clustering + modulated-
    // light source detection (vibration/flicker analysis).
    add({"frequency_map", "Frequency Map", "analytics", "self",
         AlgoDisplayMode::Standalone,
         {pint("update_interval_ms", "Update interval (ms)", "250", "50", "5000"),
          pint("filter_length", "Period confirm count", "7", "2", "8"),
          pint("period_diff_thresh_us", "Period jitter (us)", "1600", "100", "100000"),
          pfloat("min_freq_hz", "Min frequency (Hz)", "8", "0.5", "1000"),
          pfloat("max_freq_hz", "Max frequency (Hz)", "120", "10", "5000"),
          pfloat("max_freq_diff", "Cluster freq diff (Hz)", "6", "1", "500"),
          pint("min_cluster_size", "Min cluster size (px)", "20", "3", "1000")}});

    // §4.4.8 Sensor Self-Test — per-pixel refractory-period heatmap + bad-pixel
    // detection. Registered WITHOUT roi_params/preproc_params (the self-test
    // must cover the full sensor, not a 128×128 ROI). No user-tunable params.
    // Triggered from the Devices panel (hardware module) as a sensor diagnostic.
    // NOTE: sensor_self_test is intentionally NOT registered here. It is a
    // hardware diagnostic launched from the Devices panel button, not an
    // algorithm — registering it (as e0439b7 did, category="analytics")
    // made it appear as a checkable algorithm in the Algorithms panel,
    // which was never the intent. AlgoWindow creates its instance from a
    // built-in AlgoInfo via AlgoBridge::create_with_info instead.
}

} // namespace gui
