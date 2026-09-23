// gui/panels/biases_panel.cpp

#include "biases_panel.h"

#include <algorithm>

#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QSlider>
#include <QString>
#include <QStyle>
// QScrollArea no longer needed — the BiasesPanel is hosted directly inside the
// Basic tab's outer scroll area, so an inner scroll is redundant.

#include <metavision/hal/facilities/i_ll_biases.h>

#include "app/camera_controller.h"

namespace gui {

BiasesPanel::BiasesPanel(QWidget* parent) : AbstractPanel(parent) {
    auto* outer = new QVBoxLayout(this);
    outer->setContentsMargins(0, 0, 0, 0);
    outer->setSpacing(8);

    hint_label_ = new QLabel(tr("No live camera connected."), this);
    hint_label_->setWordWrap(true);
    hint_label_->setProperty("class", "hint");
    outer->addWidget(hint_label_);

    // The bias rows live in a titled group box (matching the ESP panel's
    // Anti-Flicker / Trail Filter / ERC sections). No inner QScrollArea —
    // the BiasesPanel is already hosted inside the Basic tab's outer scroll
    // area, so an inner scroll would just produce a tiny viewport with its
    // own scrollbar (the user explicitly asked to see all bias rows at
    // once). A plain layout lets the outer scroll handle overflow naturally
    // and gives every row its full height. Hidden until a camera with a
    // bias facility is connected (the hint label covers the empty states).
    group_ = new QGroupBox(tr("BIAS"), this);
    rows_layout_ = new QVBoxLayout(group_);
    // The group box border + title padding already inset the content; 4 px
    // keeps the rows close to their former position inside the frame.
    rows_layout_->setContentsMargins(4, 4, 4, 4);
    rows_layout_->setSpacing(8);
    rows_layout_->addStretch(1);
    outer->addWidget(group_, 1);
    group_->setVisible(false);
    group_->setEnabled(false);

    // Debounced apply for wheel/keyboard slider edits (see header, §六-U1).
    apply_debounce_.setSingleShot(true);
    apply_debounce_.setInterval(300);
    connect(&apply_debounce_, &QTimer::timeout, this, [this]() {
        // Apply every row edited in the window (in insertion order), then
        // clear — sweeping several biases with the wheel must reach the
        // hardware for each of them, not only the last.
        for (const auto& name : pending_applies_) {
            for (auto& r : rows_) {
                if (r.name == name) {
                    apply_value(r, r.slider->value());
                    break;
                }
            }
        }
        pending_applies_.clear();
    });

    build_auto_bias_section(outer);
}

void BiasesPanel::build_auto_bias_section(QVBoxLayout* outer) {
    auto_group_ = new QGroupBox(tr("AUTO BIAS"), this);
    auto* lay = new QVBoxLayout(auto_group_);
    lay->setContentsMargins(4, 4, 4, 4);
    lay->setSpacing(6);

    auto_bias_cb_ = new QCheckBox(tr("Enable"), auto_group_);
    lay->addWidget(auto_bias_cb_);

    // Rate band: one field per row (Min / Max, Mev/s). No artificial
    // ceiling on the max — the hardware bias range is the real limit
    // (spinbox max is set high enough to never bind).
    auto make_spin = [this](double lo, double hi, double def) {
        auto* sp = new QDoubleSpinBox(auto_group_);
        sp->setRange(lo, hi);
        sp->setDecimals(1);
        sp->setSingleStep(0.5);
        sp->setValue(def);
        sp->setSuffix(tr(" Mev/s"));
        return sp;
    };
    auto* min_row = new QWidget(auto_group_);
    auto* min_hl = new QHBoxLayout(min_row);
    min_hl->setContentsMargins(0, 0, 0, 0);
    min_hl->setSpacing(6);
    min_hl->addWidget(new QLabel(tr("Min rate"), min_row), 0);
    rate_min_sp_ = make_spin(0.05, 1000.0, 1.0);
    min_hl->addWidget(rate_min_sp_, 1);
    lay->addWidget(min_row);
    auto* max_row = new QWidget(auto_group_);
    auto* max_hl = new QHBoxLayout(max_row);
    max_hl->setContentsMargins(0, 0, 0, 0);
    max_hl->setSpacing(6);
    max_hl->addWidget(new QLabel(tr("Max rate"), max_row), 0);
    rate_max_sp_ = make_spin(0.1, 100000.0, 50.0);
    max_hl->addWidget(rate_max_sp_, 1);
    lay->addWidget(max_row);

    outer->addWidget(auto_group_);
    auto_group_->setVisible(true);
    auto_group_->setEnabled(false);

    connect(auto_bias_cb_, &QCheckBox::toggled, this, [this](bool on) {
        if (!camera_ || !camera_->set_auto_bias_enabled(on)) {
            // File playback / sensor without usable control axes (Prophesee
            // without diff biases): refuse and reflect the actual state.
            QSignalBlocker b(auto_bias_cb_);
            auto_bias_cb_->setChecked(false);
        }
        sync_auto_bias_ui();
    });
    connect(rate_min_sp_, &QDoubleSpinBox::valueChanged, this,
            [this](double) { apply_auto_bias_bounds(); });
    connect(rate_max_sp_, &QDoubleSpinBox::valueChanged, this,
            [this](double) { apply_auto_bias_bounds(); });
}

void BiasesPanel::apply_auto_bias_bounds() {
    if (!camera_) {
        return;
    }
    // The just-edited field wins; the other follows so the pair stays a
    // valid lo < hi band (mirrors the controller's own validation).
    const double lo = rate_min_sp_->value();
    const double hi = rate_max_sp_->value();
    if (sender() == rate_min_sp_ && lo >= hi) {
        QSignalBlocker b(rate_max_sp_);
        rate_max_sp_->setValue(lo + 0.5);
    } else if (sender() == rate_max_sp_ && hi <= lo) {
        QSignalBlocker b(rate_min_sp_);
        rate_min_sp_->setValue(std::max(0.05, hi - 0.5));
    }
    camera_->set_auto_bias_rate_bounds(
        static_cast<float>(rate_min_sp_->value()),
        static_cast<float>(rate_max_sp_->value()));
}

void BiasesPanel::sync_auto_bias_ui() {
    const bool usable = camera_ && populated_ && camera_->is_connected() &&
                        !camera_->is_file_source();
    auto_group_->setEnabled(usable);
    if (!camera_ || !usable) {
        QSignalBlocker b(auto_bias_cb_);
        auto_bias_cb_->setChecked(false);
        return;
    }
    QSignalBlocker b(auto_bias_cb_);
    auto_bias_cb_->setChecked(camera_->auto_bias_enabled());
    float lo = 0.F, hi = 0.F;
    camera_->auto_bias_rate_bounds(lo, hi);
    QSignalBlocker b1(rate_min_sp_);
    QSignalBlocker b2(rate_max_sp_);
    rate_min_sp_->setValue(static_cast<double>(lo));
    rate_max_sp_->setValue(static_cast<double>(hi));
}

void BiasesPanel::on_camera_connected(CameraController* controller) {
    camera_ = controller;
    // Re-connect the out-of-band write notification for THIS source
    // (the controller object outlives sources — untracked connections
    // would stack across reconnects).
    if (auto_bias_conn_) {
        disconnect(auto_bias_conn_);
    }
    if (controller) {
        auto_bias_conn_ = connect(controller, &CameraController::auto_bias_applied,
                                  this, &BiasesPanel::refresh_row_values,
                                  Qt::QueuedConnection);
    }
    clear_rows();
    populate();
    sync_auto_bias_ui();
}

void BiasesPanel::on_camera_disconnected() {
    camera_ = nullptr;
    if (auto_bias_conn_) {
        disconnect(auto_bias_conn_);
        auto_bias_conn_ = {};
    }
    clear_rows();
    hint_label_->setText(tr("No live camera connected."));
    hint_label_->setProperty("class", "hint");
    hint_label_->setVisible(true);
    restyle(hint_label_);
    group_->setEnabled(false);
    group_->setVisible(false);
    populated_ = false;
    sync_auto_bias_ui();
}

void BiasesPanel::save_to_file(const QString& path) {
    if (!camera_) {
        emit error_message(tr("No camera connected."));
        return;
    }
    auto* b = camera_->biases_facility();
    if (!b) {
        emit error_message(tr("Bias facility unavailable on this camera."));
        return;
    }
    try {
        b->save_to_file(std::filesystem::path(path.toStdString()));
        emit info_message(tr("Biases saved to %1").arg(path));
    } catch (const std::exception& e) {
        emit error_message(QString::fromUtf8(e.what()));
    }
}

void BiasesPanel::load_from_file(const QString& path) {
    if (!camera_) {
        emit error_message(tr("No camera connected."));
        return;
    }
    auto* b = camera_->biases_facility();
    if (!b) {
        emit error_message(tr("Bias facility unavailable on this camera."));
        return;
    }
    try {
        b->load_from_file(std::filesystem::path(path.toStdString()));
        refresh_row_values();
        emit info_message(tr("Biases loaded from %1").arg(path));
    } catch (const std::exception& e) {
        emit error_message(QString::fromUtf8(e.what()));
    }
}

// ---------------------------------------------------------------------------

void BiasesPanel::clear_rows() {
    for (auto& row : rows_) {
        // Remove from the layout first so synchronous repopulation below
        // doesn't see stale layout state. Deleting the row widget also
        // destroys its slider/spin/label/button children (Qt's parent-child
        // ownership).
        if (row.row_widget) {
            rows_layout_->removeWidget(row.row_widget);
            row.row_widget->deleteLater();
        }
    }
    rows_.clear();
}

void BiasesPanel::populate() {
    if (!camera_) return;
    auto* biases = camera_->biases_facility();
    if (!biases) {
        hint_label_->setText(tr("Biases not supported by this camera."));
        hint_label_->setProperty("class", "hint");
        hint_label_->setVisible(true);
        restyle(hint_label_);
        group_->setEnabled(false);
        group_->setVisible(false);
        populated_ = false;
        return;
    }

    std::map<std::string, int> all;
    try {
        all = biases->get_all_biases();
    } catch (const std::exception& e) {
        hint_label_->setText(tr("Failed to enumerate biases: %1").arg(QString::fromUtf8(e.what())));
        hint_label_->setVisible(true);
        group_->setEnabled(false);
        group_->setVisible(false);
        populated_ = false;
        return;
    }

    if (all.empty()) {
        hint_label_->setText(tr("Camera reports no configurable biases."));
        hint_label_->setVisible(true);
        group_->setEnabled(false);
        group_->setVisible(false);
        populated_ = false;
        return;
    }

    // The group box title carries the "BIAS" heading — hide the hint label
    // so the text does not appear twice.
    hint_label_->setVisible(false);
    group_->setVisible(true);

    // Insert rows before the trailing stretch.
    for (const auto& [name, value] : all) {
        BiasRow row;
        row.name = name;
        row.snapshot_value = value;

        auto* row_widget = new QWidget(group_);
        row.row_widget = row_widget;

        // Resolve range + metadata.
        int lo = 0, hi = 0;
        QString description;
        bool modifiable = true;
        try {
            Metavision::LL_Bias_Info info;
            if (biases->get_bias_info(name, info)) {
                const auto r = info.get_bias_range();
                lo = r.first;
                hi = r.second;
                description = QString::fromStdString(info.get_description());
                modifiable = info.is_modifiable();
            }
        } catch (...) {}
        if (hi <= lo) {
            // Fall back to a wide symmetric window around the current value.
            lo = value - 1000;
            hi = value + 1000;
        }

        auto* hl = new QHBoxLayout(row_widget);
        hl->setContentsMargins(0, 0, 0, 0);
        hl->setSpacing(6);

        auto* label = new QLabel(QString::fromStdString(name), row_widget);
        label->setMinimumWidth(70);
        label->setToolTip(description.isEmpty()
                              ? tr("No description available.")
                              : description);
        row.slider = new QSlider(Qt::Horizontal, row_widget);
        row.slider->setRange(lo, hi);
        row.slider->setValue(value);
        row.slider->setToolTip(description);

        row.spin = new QSpinBox(row_widget);
        row.spin->setRange(lo, hi);
        row.spin->setValue(value);
        row.spin->setToolTip(description);

        auto* btn_reset = new QPushButton(tr("Reset"), row_widget);
        // Compact padding (base.qss) trims the wide internal side border so
        // the rows fit inside the group box frame without widening the
        // sidebar — the width still follows the (smaller) size hint, so the
        // text is never clipped.
        btn_reset->setProperty("class", "compact");

        if (!modifiable) {
            row.slider->setEnabled(false);
            row.spin->setEnabled(false);
            btn_reset->setEnabled(false);
            label->setProperty("class", "muted");
        }

        hl->addWidget(label, 0);
        hl->addWidget(row.slider, 1);
        hl->addWidget(row.spin, 0);
        hl->addWidget(btn_reset, 0);

        rows_layout_->insertWidget(rows_layout_->count() - 1, row_widget);

        // Wire edits. Capture name (not the row pointer) to stay safe if the
        // vector reallocates — we look up by name when applying.
        const std::string bias_name = name;
        connect(row.slider, &QSlider::valueChanged, this,
                [this, bias_name, spin = row.spin](int v) {
                    QSignalBlocker b(spin);
                    spin->setValue(v);
                    // Don't apply per tick — valueChanged fires continuously
                    // during a drag and would flood USB writes. Drag edits
                    // are applied on sliderReleased; wheel/keyboard edits
                    // (which never emit sliderReleased) go through a 300 ms
                    // debounce so they still reach the hardware (§六-U1).
                    if (std::find(pending_applies_.begin(),
                                  pending_applies_.end(),
                                  bias_name) == pending_applies_.end()) {
                        pending_applies_.push_back(bias_name);
                    }
                    apply_debounce_.start();
                });
        connect(row.slider, &QSlider::sliderReleased, this,
                [this, bias_name]() {
                    for (auto& r : rows_) {
                        if (r.name == bias_name) {
                            apply_value(r, r.slider->value());
                            break;
                        }
                    }
                });
        connect(row.spin, QOverload<int>::of(&QSpinBox::valueChanged), this,
                [this, bias_name, slider = row.slider](int v) {
                    QSignalBlocker b(slider);
                    slider->setValue(v);
                    for (auto& r : rows_) {
                        if (r.name == bias_name) { apply_value(r, v); break; }
                    }
                });
        connect(btn_reset, &QPushButton::clicked, this,
                [this, bias_name, snap = value,
                 slider = row.slider, spin = row.spin]() {
                    QSignalBlocker bs(slider);
                    QSignalBlocker bp(spin);
                    slider->setValue(snap);
                    spin->setValue(snap);
                    for (auto& r : rows_) {
                        if (r.name == bias_name) { apply_value(r, snap); break; }
                    }
                });

        rows_.push_back(std::move(row));
    }

    group_->setEnabled(true);
    populated_ = true;
}

void BiasesPanel::apply_value(BiasRow& row, int value) {
    if (!camera_) return;
    auto* biases = camera_->biases_facility();
    if (!biases) return;
    try {
        biases->set(row.name, value);
    } catch (const std::exception& e) {
        emit error_message(tr("Failed to set %1: %2")
                               .arg(QString::fromStdString(row.name))
                               .arg(QString::fromUtf8(e.what())));
        // Roll the slider/spin back to the hardware's actual value so the
        // UI doesn't show a value that never took effect.
        refresh_row_values();
    }
}

void BiasesPanel::refresh_row_values() {
    if (!camera_ || !populated_) return;
    auto* biases = camera_->biases_facility();
    if (!biases) return;
    std::map<std::string, int> all;
    try { all = biases->get_all_biases(); } catch (...) { return; }
    for (auto& row : rows_) {
        auto it = all.find(row.name);
        if (it == all.end()) continue;
        QSignalBlocker bs(row.slider);
        QSignalBlocker bp(row.spin);
        row.slider->setValue(it->second);
        row.spin->setValue(it->second);
    }
}

} // namespace gui
