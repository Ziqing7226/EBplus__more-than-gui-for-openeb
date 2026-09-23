// gui/config/config_manager.cpp

#include "config_manager.h"

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonValue>
#include <QStringList>
#include <QtDebug>

#include <map>

#include <metavision/hal/facilities/i_antiflicker_module.h>
#include <metavision/hal/facilities/i_erc_module.h>
#include <metavision/hal/facilities/i_event_trail_filter_module.h>
#include <metavision/hal/facilities/i_ll_biases.h>
#include <metavision/hal/facilities/i_roi.h>
#include <metavision/hal/facilities/i_camera_synchronization.h>
#include <metavision/hal/facilities/i_trigger_in.h>
#include <metavision/hal/facilities/i_trigger_out.h>

#include "app/camera_controller.h"
#include "algo_bridge/algo_bridge.h"

namespace gui {

ConfigManager::ConfigManager(QObject* parent) : QObject(parent) {}

// ---------------------------------------------------------------------------
// Capture
// ---------------------------------------------------------------------------

QJsonObject ConfigManager::capture(CameraController* controller) const {
    QJsonObject root;
    if (!controller) return root;
    const auto& info = controller->sensor_info();
    root["format"] = QStringLiteral("GUI-for-openEB-config");
    root["version"] = 1;
    QJsonObject sensor;
    sensor["integrator"] = info.integrator;
    sensor["generation"] = info.generation_name;
    sensor["width"] = info.width;
    sensor["height"] = info.height;
    root["sensor"] = sensor;
    root["biases"] = capture_biases(controller);
    root["roi"] = capture_roi(controller);
    root["esp"] = capture_esp(controller);
    root["trigger"] = capture_trigger(controller);
    return root;
}

QJsonObject ConfigManager::capture_biases(CameraController* c) const {
    QJsonObject o;
    auto* b = c->biases_facility();
    if (!b) return o;
    try {
        for (const auto& kv : b->get_all_biases()) {
            QJsonObject entry;
            entry["value"] = kv.second;
            Metavision::LL_Bias_Info info;
            try {
                if (b->get_bias_info(kv.first, info)) {
                    entry["desc"] = QString::fromStdString(info.get_description());
                }
            } catch (...) {}
            o[QString::fromStdString(kv.first)] = entry;
        }
    } catch (...) {}
    return o;
}

QJsonObject ConfigManager::capture_roi(CameraController* c) const {
    QJsonObject o;
    // Phase 2.6 debug D-5: read the unified state cache (single source of
    // truth) instead of the facility — all writers go through
    // CameraController::set_unified_roi now, so the cache is always fresh.
    bool en = false;
    int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
    c->unified_roi(en, x0, y0, x1, y1);
    o["enabled"] = en;
    o["mode"] = c->unified_roi_roni()
                    ? static_cast<int>(Metavision::I_ROI::Mode::RONI)
                    : static_cast<int>(Metavision::I_ROI::Mode::ROI);
    QJsonObject geom;
    geom["x"] = x0;
    geom["y"] = y0;
    geom["width"] = x1 - x0;
    geom["height"] = y1 - y0;
    o["window"] = geom;
    return o;
}

QJsonObject ConfigManager::capture_esp(CameraController* c) const {
    QJsonObject o;
    if (auto* af = c->anti_flicker_facility()) {
        QJsonObject a;
        try {
            a["enabled"] = af->is_enabled();
            a["mode"] = static_cast<int>(af->get_filtering_mode());
            a["band_low"] = static_cast<int>(af->get_band_low_frequency());
            a["band_high"] = static_cast<int>(af->get_band_high_frequency());
            a["duty_cycle"] = static_cast<double>(af->get_duty_cycle());
            a["start_threshold"] = static_cast<int>(af->get_start_threshold());
            a["stop_threshold"] = static_cast<int>(af->get_stop_threshold());
        } catch (...) {}
        o["anti_flicker"] = a;
    }
    if (auto* tf = c->trail_filter_facility()) {
        QJsonObject t;
        try {
            t["enabled"] = tf->is_enabled();
            t["type"] = static_cast<int>(tf->get_type());
            t["threshold"] = static_cast<qint64>(tf->get_threshold());
        } catch (...) {}
        o["trail_filter"] = t;
    }
    if (auto* erc = c->erc_facility()) {
        QJsonObject e;
        try {
            e["enabled"] = erc->is_enabled();
            e["target_rate"] = static_cast<qint64>(erc->get_cd_event_rate());
        } catch (...) {}
        o["erc"] = e;
    }
    return o;
}

QJsonObject ConfigManager::capture_trigger(CameraController* c) const {
    QJsonObject o;
    if (auto* ti = c->trigger_in_facility()) {
        QJsonObject i;
        // I_TriggerIn exposes per-channel state (the panel renders one
        // checkbox per available channel) — persist every standard channel
        // that answers, not just Main.
        const std::pair<const char*, Metavision::I_TriggerIn::Channel> chans[] = {
            {"main", Metavision::I_TriggerIn::Channel::Main},
            {"aux", Metavision::I_TriggerIn::Channel::Aux},
            {"loopback", Metavision::I_TriggerIn::Channel::Loopback},
        };
        for (const auto& [name, ch] : chans) {
            try { i[name] = ti->is_enabled(ch); } catch (...) {}
        }
        o["in"] = i;
    }
    if (auto* to = c->trigger_out_facility()) {
        QJsonObject oo;
        try {
            oo["enabled"] = to->is_enabled();
            oo["period_us"] = static_cast<qint64>(to->get_period());
            oo["duty_cycle"] = to->get_duty_cycle();
        } catch (...) {}
        o["out"] = oo;
    }
    return o;
}

// ---------------------------------------------------------------------------
// Apply
// ---------------------------------------------------------------------------

bool ConfigManager::apply(CameraController* controller, const QJsonObject& obj, QString& err) const {
    if (!controller) {
        err = tr("No camera connected.");
        return false;
    }
    const QString validation = validate(obj, controller);
    if (!validation.isEmpty()) {
        // Surface the validation problem to the caller so the UI can warn
        // the user — previously this was written into `err` but then
        // overwritten by the (successful) apply steps, silently swallowing
        // sensor-mismatch warnings.
        err = validation;
        return false;
    }
    bool ok = true;
    QString sub_err;
    if (obj.contains("biases"))  ok &= apply_biases(controller, obj.value("biases").toObject(), sub_err);
    if (obj.contains("roi"))     ok &= apply_roi(controller, obj.value("roi").toObject(), sub_err);
    if (obj.contains("esp"))     ok &= apply_esp(controller, obj.value("esp").toObject(), sub_err);
    if (obj.contains("trigger")) ok &= apply_trigger(controller, obj.value("trigger").toObject(), sub_err);
    if (!ok && err.isEmpty()) err = sub_err;
    return ok;
}

bool ConfigManager::apply_biases(CameraController* c, const QJsonObject& o, QString& err) const {
    auto* b = c->biases_facility();
    if (!b) { err = tr("Biases not supported."); return false; }
    bool ok = true;
    for (auto it = o.begin(); it != o.end(); ++it) {
        const auto name = it.key().toStdString();
        // Value is persisted as a JSON number; accept both int and string forms.
        const auto val_obj = it.value().toObject().value("value");
        int v = 0;
        bool converted = false;
        if (val_obj.isString()) {
            v = val_obj.toString().toInt(&converted);
        } else if (val_obj.isDouble()) {
            v = val_obj.toInt();
            converted = true;
        }
        if (!converted) { ok = false; continue; }
        try {
            b->set(name, v);
        } catch (...) {
            ok = false;
        }
    }
    return ok;
}

bool ConfigManager::apply_roi(CameraController* c, const QJsonObject& o, QString& /*err*/) const {
    // Phase 2.6 debug D-5: route through the unified state source so the
    // controller cache (overlay/zoom/algorithm path) reflects the loaded
    // ROI — direct facility writes used to leave it stale.
    const bool enabled = o.value("enabled").toBool(false);
    const bool roni = o.contains("mode") &&
        static_cast<Metavision::I_ROI::Mode>(o.value("mode").toInt()) ==
            Metavision::I_ROI::Mode::RONI;
    int x = -1, y = -1, w = 0, h = 0;  // defaults: auto-center, full sensor
    if (o.contains("window")) {
        const auto geom = o.value("window").toObject();
        x = geom.value("x").toInt(-1);
        y = geom.value("y").toInt(-1);
        w = geom.value("width").toInt(0);
        h = geom.value("height").toInt(0);
    }
    return c->set_unified_roi(enabled, x, y, w, h, roni);
}

bool ConfigManager::apply_esp(CameraController* c, const QJsonObject& o, QString& /*err*/) const {
    bool ok = true;
    if (o.contains("anti_flicker")) {
        auto* af = c->anti_flicker_facility();
        const auto a = o.value("anti_flicker").toObject();
        if (af) {
            // Parameters first, enable last, so the filter starts with the
            // persisted configuration (same order as trail_filter below).
            if (a.contains("mode")) {
                try {
                    af->set_filtering_mode(
                        static_cast<Metavision::I_AntiFlickerModule::AntiFlickerMode>(
                            a.value("mode").toInt()));
                } catch (...) { ok = false; }
            }
            if (a.contains("band_low") && a.contains("band_high")) {
                try {
                    af->set_frequency_band(static_cast<uint32_t>(a.value("band_low").toInt()),
                                           static_cast<uint32_t>(a.value("band_high").toInt()));
                } catch (...) { ok = false; }
            }
            if (a.contains("duty_cycle")) {
                try { af->set_duty_cycle(
                          static_cast<float>(a.value("duty_cycle").toDouble())); }
                catch (...) { ok = false; }
            }
            if (a.contains("start_threshold")) {
                try { af->set_start_threshold(static_cast<uint32_t>(
                          a.value("start_threshold").toVariant().toLongLong())); }
                catch (...) { ok = false; }
            }
            if (a.contains("stop_threshold")) {
                try { af->set_stop_threshold(static_cast<uint32_t>(
                          a.value("stop_threshold").toVariant().toLongLong())); }
                catch (...) { ok = false; }
            }
            if (a.contains("enabled")) {
                // enable() is a register write: a failing device throws
                // HalException, which must not escape into the Qt event
                // loop (std::terminate). Same guard as the setters above
                // and as apply_trigger.
                try { af->enable(a.value("enabled").toBool()); }
                catch (...) { ok = false; }
            }
        }
    }
    if (o.contains("trail_filter")) {
        auto* tf = c->trail_filter_facility();
        const auto t = o.value("trail_filter").toObject();
        if (tf) {
            // Apply type/threshold before enabling so the filter is configured
            // with the persisted parameters when it starts processing.
            if (t.contains("type")) {
                try { tf->set_type(static_cast<Metavision::I_EventTrailFilterModule::Type>(
                                       t.value("type").toInt())); }
                catch (...) { ok = false; }
            }
            if (t.contains("threshold")) {
                try { tf->set_threshold(static_cast<uint32_t>(t.value("threshold").toVariant().toLongLong())); }
                catch (...) { ok = false; }
            }
            if (t.contains("enabled")) {
                try { tf->enable(t.value("enabled").toBool()); }
                catch (...) { ok = false; }
            }
        }
    }
    if (o.contains("erc")) {
        auto* erc = c->erc_facility();
        const auto e = o.value("erc").toObject();
        if (erc) {
            if (e.contains("target_rate")) {
                try { erc->set_cd_event_rate(static_cast<uint32_t>(e.value("target_rate").toVariant().toLongLong())); }
                catch (...) { ok = false; }
            }
            if (e.contains("enabled")) {
                try { erc->enable(e.value("enabled").toBool()); }
                catch (...) { ok = false; }
            }
        }
    }
    return ok;
}

bool ConfigManager::apply_trigger(CameraController* c, const QJsonObject& o, QString& /*err*/) const {
    bool ok = true;
    if (o.contains("in")) {
        auto* ti = c->trigger_in_facility();
        const auto i = o.value("in").toObject();
        if (ti) {
            const std::pair<const char*, Metavision::I_TriggerIn::Channel> chans[] = {
                {"main", Metavision::I_TriggerIn::Channel::Main},
                {"aux", Metavision::I_TriggerIn::Channel::Aux},
                {"loopback", Metavision::I_TriggerIn::Channel::Loopback},
            };
            for (const auto& [name, ch] : chans) {
                if (!i.contains(name)) continue;
                const bool want = i.value(name).toBool();
                try {
                    if (want) ti->enable(ch);
                    else      ti->disable(ch);
                } catch (...) { ok = false; }
            }
        }
    }
    if (o.contains("out")) {
        auto* to = c->trigger_out_facility();
        const auto oo = o.value("out").toObject();
        if (to) {
            if (oo.contains("period_us")) {
                try { to->set_period(static_cast<uint32_t>(oo.value("period_us").toVariant().toLongLong())); }
                catch (...) { ok = false; }
            }
            if (oo.contains("duty_cycle")) {
                try { to->set_duty_cycle(oo.value("duty_cycle").toDouble()); }
                catch (...) { ok = false; }
            }
            if (oo.contains("enabled")) {
                // HAL rejects trigger out in Master sync mode; switch to
                // Slave and retry so a saved "enabled" config restores cleanly.
                bool en_ok = false;
                try {
                    if (oo.value("enabled").toBool()) {
                        en_ok = to->enable();
                        if (!en_ok) {
                            auto* sync = c->camera_sync_facility();
                            if (sync && sync->get_mode() == Metavision::I_CameraSynchronization::SyncMode::MASTER &&
                                sync->set_mode_slave()) {
                                en_ok = to->enable();
                            }
                        }
                    } else {
                        en_ok = to->disable();
                    }
                } catch (...) {}
                ok &= en_ok;
            }
        }
    }
    return ok;
}

// ---------------------------------------------------------------------------
// File I/O
// ---------------------------------------------------------------------------

bool ConfigManager::save_to_file(CameraController* controller, const QString& path, QString& err) const {
    const QJsonObject obj = capture(controller);
    QJsonDocument doc(obj);
    QFile f(path);
    if (!f.open(QIODevice::WriteOnly)) {
        err = tr("Cannot open file for writing:\n%1").arg(path);
        return false;
    }
    const QByteArray data = doc.toJson(QJsonDocument::Indented);
    if (f.write(data) != data.size() || !f.flush()) {
        err = tr("Failed to write config file:\n%1").arg(path);
        return false;
    }
    return true;
}

bool ConfigManager::load_from_file(CameraController* controller, const QString& path, QString& err) const {
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) {
        err = tr("Cannot open file for reading:\n%1").arg(path);
        return false;
    }
    QJsonParseError pe;
    auto doc = QJsonDocument::fromJson(f.readAll(), &pe);
    if (pe.error != QJsonParseError::NoError) {
        err = pe.errorString();
        return false;
    }
    return apply(controller, doc.object(), err);
}

QString ConfigManager::validate(const QJsonObject& obj, CameraController* controller) const {
    if (obj.value("format").toString() != "GUI-for-openEB-config") {
        return tr("Not a GUI-for-openEB config file.");
    }
    const auto sensor = obj.value("sensor").toObject();
    const QString gen = sensor.value("generation").toString();
    const QString live = controller->sensor_info().generation_name;
    if (!gen.isEmpty() && !live.isEmpty() && gen != live) {
        return tr("Sensor mismatch: config for '%1', connected '%2'.").arg(gen, live);
    }
    return {};
}

// ---------------------------------------------------------------------------
// Phase 10: algorithm parameter save/load
// ---------------------------------------------------------------------------

QJsonObject ConfigManager::capture_algo_state(AlgoBridge* bridge) const {
    QJsonObject root;
    root["format"] = QStringLiteral("GUI-for-openEB-algo-params");
    root["version"] = 1;
    if (!bridge) return root;
    QJsonObject algos;
    for (const auto& info : bridge->list_algos()) {
        QJsonObject entry;
        entry["category"] = QString::fromStdString(info.category);
        QJsonObject params;
        // Query the live instance (if any) for current values; fall back to
        // the factory default when no instance exists yet.
        auto live = bridge->find_live(info.name);
        for (const auto& p : info.params) {
            std::string val = p.default_value;
            if (live) {
                const auto cur = live->get_param(p.key);
                if (!cur.empty()) val = cur;
            }
            params[QString::fromStdString(p.key)] = QString::fromStdString(val);
        }
        entry["params"] = params;
        if (live) entry["enabled"] = live->is_enabled();
        algos[QString::fromStdString(info.name)] = entry;
    }
    root["algorithms"] = algos;
    return root;
}

bool ConfigManager::apply_algo_state(AlgoBridge* bridge, const QJsonObject& obj, QString& err,
                                     std::map<std::string, std::string>* legacy_roi) const {
    if (!bridge) { err = tr("No algorithm bridge."); return false; }
    if (obj.value("format").toString() != "GUI-for-openEB-algo-params") {
        err = tr("Not an algorithm parameter file.");
        return false;
    }
    // Apply parameter values to live instances. Instances not yet created by
    // AlgorithmsPanel are cached via bridge->cache_algo_params() so they are
    // replayed when the instance is eventually created (N1). Without this,
    // loading a config before enabling an algorithm would silently discard
    // all saved parameters.
    //
    // §11.2-F+G: before forwarding each param to set_param/cache_algo_params,
    // we (1) rename obsolete keys per-algorithm via the migration table below
    // (without this, old configs silently lose the renamed param's value —
    // set_param only stores keys present in info->params), and (2) clamp
    // out-of-range float/int values to the registered [min, max] so that the
    // stored param_values_ matches the algo's runtime value (the algo setter
    // already clamps internally, but without this the displayed value would
    // diverge from the runtime value until the user touches the spinbox). The
    // migration table is scoped by algo name because the same key may
    // legitimately exist under different algos with different semantics —
    // e.g. blob_detector still uses "learning_rate" as a rate, while
    // background_mask renamed it to "learning_window_s" for a window-in-
    // seconds semantics (§五-B2).
    static const std::map<std::string, std::map<std::string, std::string>> kParamRenames = {
        {"background_mask", {{"learning_rate", "learning_window_s"}}},
    };

    const auto algos = obj.value("algorithms").toObject();
    bool ok = true;
    QStringList unknown_algos;
    for (auto it = algos.begin(); it != algos.end(); ++it) {
        const auto name = it.key().toStdString();
        const auto* info = bridge->find(name);
        if (!info) {
            // Unknown algorithm — skip but flag failure with a descriptive
            // message so the user knows which entries were rejected (BUG-R1).
            ok = false;
            unknown_algos << it.key();
            continue;
        }
        const auto entry = it.value().toObject();
        const auto params = entry.value("params").toObject();

        // Build the migrated + clamped param map once so the live and cached
        // paths see identical values.
        std::map<std::string, std::string> migrated;
        for (auto pit = params.begin(); pit != params.end(); ++pit) {
            std::string key = pit.key().toStdString();
            std::string val = pit.value().toString().toStdString();

            // §11.2-G: rename obsolete keys per-algorithm.
            auto rit = kParamRenames.find(name);
            if (rit != kParamRenames.end()) {
                auto mit = rit->second.find(key);
                if (mit != rit->second.end()) {
                    qWarning("ConfigManager: migrating obsolete param %s::%s -> %s",
                             name.c_str(), key.c_str(), mit->second.c_str());
                    key = mit->second;
                }
            }

            // Phase 2.6: legacy per-algorithm roi_* keys (the deleted
            // per-backend ROI). When the caller collects them, record the
            // FIRST algorithm's values for mapping onto the unified ROI and
            // skip silently (no warning, no forwarding — backends no longer
            // honour them anyway).
            if (legacy_roi && key.rfind("roi_", 0) == 0) {
                // First algorithm wins (the header documents exactly that;
                // the old else-branch was a last-writer-wins override).
                if (!legacy_roi->count(key)) (*legacy_roi)[key] = val;
                continue;
            }

            // Warn on parameter keys the algorithm no longer registers (e.g.
            // removed dead params such as n_sigma or accumulator_decay_us, or
            // hand-edited typos). The values are still forwarded: set_param()
            // drops unknown keys from param_values_ (BUG-G12) but passes them
            // to the backend, which handles backward-compat aliases (e.g. the
            // event_to_video "downsample" -> preproc_downsample forward).
            // Without this log a stale config entry would vanish silently
            // (§11.2-style silent failure).
            const AlgoParamSpec* spec = nullptr;
            for (const auto& p : info->params) {
                if (p.key == key) { spec = &p; break; }
            }
            if (!spec) {
                qWarning("ConfigManager: unrecognized param %s::%s in config",
                         name.c_str(), key.c_str());
                migrated[key] = val;
                continue;
            }

            // §11.2-F: clamp out-of-range numeric values to the registered
            // [min, max]. Only float/int params have a numeric range; enum/
            // bool/string are passed through unchanged. If the registered
            // min/max is empty (open bound), that side is skipped. If the
            // value string is not parseable as a number, it is passed through
            // unchanged (the backend will deal with it).
            if (spec->type == "float" || spec->type == "int") {
                bool okv = false, oklo = false, okhi = false;
                const double v  = QString::fromStdString(val).toDouble(&okv);
                const double lo = QString::fromStdString(spec->min_value).toDouble(&oklo);
                const double hi = QString::fromStdString(spec->max_value).toDouble(&okhi);
                if (okv) {
                    double clamped = v;
                    if (oklo && clamped < lo) clamped = lo;
                    if (okhi && clamped > hi) clamped = hi;
                    if (clamped != v) {
                        qWarning("ConfigManager: clamping param %s::%s from %g to %g (range [%s, %s])",
                                 name.c_str(), key.c_str(), v, clamped,
                                 oklo ? spec->min_value.c_str() : "-inf",
                                 okhi ? spec->max_value.c_str() : "+inf");
                        // Preserve the registry's string format: ints as
                        // ints, floats with 6 decimals (matches the
                        // QDoubleSpinBox decimals in algorithms_panel).
                        val = (spec->type == "int")
                                  ? std::to_string(static_cast<long long>(clamped))
                                  : QString::number(clamped, 'f', 6).toStdString();
                    }
                }
            }
            migrated[key] = val;
        }

        auto live = bridge->find_live(name);
        if (live) {
            for (const auto& kv : migrated) {
                live->set_param(kv.first, kv.second);
            }
            if (entry.contains("enabled")) {
                // Enforce single-algorithm mutual exclusion (design §5.6.6):
                // enabling one algorithm from a config must disable every
                // other live instance, otherwise two algorithms run at once.
                if (entry.value("enabled").toBool()) {
                    for (auto& other : bridge->list_live()) {
                        if (other != live) other->set_enabled(false);
                    }
                }
                live->set_enabled(entry.value("enabled").toBool());
            }
        } else {
            // No live instance — cache the params so create() can replay
            // them when the algorithm is later enabled (N1).
            bridge->cache_algo_params(name, migrated);
        }
    }
    if (!unknown_algos.isEmpty()) {
        err = tr("Unknown algorithm(s) in config: %1").arg(unknown_algos.join(", "));
    }
    return ok;
}

bool ConfigManager::save_algo_params_to_file(AlgoBridge* bridge, const QString& path, QString& err) const {
    const QJsonObject obj = capture_algo_state(bridge);
    QJsonDocument doc(obj);
    QFile f(path);
    if (!f.open(QIODevice::WriteOnly)) {
        err = tr("Cannot open file for writing:\n%1").arg(path);
        return false;
    }
    const QByteArray data = doc.toJson(QJsonDocument::Indented);
    if (f.write(data) != data.size() || !f.flush()) {
        err = tr("Failed to write algorithm parameter file:\n%1").arg(path);
        return false;
    }
    return true;
}

bool ConfigManager::load_algo_params_from_file(AlgoBridge* bridge, const QString& path, QString& err,
                                               std::map<std::string, std::string>* legacy_roi) const {
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) {
        err = tr("Cannot open file for reading:\n%1").arg(path);
        return false;
    }
    QJsonParseError pe;
    auto doc = QJsonDocument::fromJson(f.readAll(), &pe);
    if (pe.error != QJsonParseError::NoError) {
        err = pe.errorString();
        return false;
    }
    return apply_algo_state(bridge, doc.object(), err, legacy_roi);
}

} // namespace gui
