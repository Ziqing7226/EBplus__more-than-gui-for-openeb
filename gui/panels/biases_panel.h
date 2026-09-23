// gui/panels/biases_panel.h — sensor bias control (design §3.1.2).
//
// Iterates I_LL_Biases::get_all_biases() at connect time and builds a row
// per bias with: name label, slider, precise spinbox, reset button. The
// recommended range from LL_Bias_Info::get_bias_range() drives the slider;
// the spinbox allows typing arbitrary values inside the same range. Edits
// are applied immediately via I_LL_Biases::set(name, value). Reset restores
// the value snapshot taken when the panel was populated.

#ifndef GUI_PANELS_BIASES_PANEL_H
#define GUI_PANELS_BIASES_PANEL_H

#include <QMetaObject>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>
#include <memory>
#include <string>
#include <vector>

#include "abstract_panel.h"

class QSpinBox;
class QDoubleSpinBox;
class QSlider;
class QCheckBox;
class QLabel;
class QGroupBox;

namespace gui {

class CameraController;

class BiasesPanel : public AbstractPanel {
    Q_OBJECT
public:
    explicit BiasesPanel(QWidget* parent = nullptr);

    QString panel_id() const override { return QStringLiteral("biases"); }
    QString panel_title() const override { return tr("Biases"); }
    QString panel_group() const override { return QStringLiteral("Hardware"); }

public slots:
    /// @brief Populates the panel from the connected camera's bias facility.
    /// If the facility is unavailable (file playback or unsupported sensor),
    /// the panel is disabled with an explanatory hint.
    void on_camera_connected(CameraController* controller) override;

    /// @brief Clears all rows and disables the panel.
    void on_camera_disconnected() override;

    /// @brief Saves current biases to a file via I_LL_Biases::save_to_file.
    void save_to_file(const QString& path);
    /// @brief Loads biases from a file via I_LL_Biases::load_from_file and
    /// refreshes the UI to reflect the new values.
    void load_from_file(const QString& path);

    /// @brief Re-reads every bias from the hardware and updates the rows
    /// (signal-blocked — no apply loop). Called after out-of-band changes,
    /// e.g. the calibration wizard's Auto Bias override.
    void refresh_row_values();

    /// @brief Enables/disables + state sync for the current source. Public so
    /// out-of-band Auto Bias changes (the calibration wizard's override) can
    /// re-sync the checkbox + rate spinboxes with the controller.
    void sync_auto_bias_ui();

private:
    struct BiasRow {
        std::string name;
        int snapshot_value{0};
        QWidget* row_widget{nullptr};
        QSlider* slider{nullptr};
        QSpinBox* spin{nullptr};
    };

    void clear_rows();
    void populate();
    void apply_value(BiasRow& row, int value);

    /// Auto bias section (§4.4.6) at the bottom of the panel: enable
    /// checkbox + rate band spinboxes, driving the CameraController-level
    /// dual-loop bias control. Coexists with every algorithm (it is not an
    /// algorithm instance). Live cameras only.
    void build_auto_bias_section(QVBoxLayout* outer);
    /// Pushes the spinbox pair to the controller, keeping it a valid
    /// lo < hi band (the just-edited field wins, the other follows).
    void apply_auto_bias_bounds();

    QVBoxLayout* rows_layout_{nullptr};
    QGroupBox* group_{nullptr};
    QLabel* hint_label_{nullptr};
    QGroupBox* auto_group_{nullptr};
    QCheckBox* auto_bias_cb_{nullptr};
    QDoubleSpinBox* rate_min_sp_{nullptr};
    QDoubleSpinBox* rate_max_sp_{nullptr};
    /// CameraController::auto_bias_applied connection for the current
    /// source (re-connected on every on_camera_connected — without tracking
    /// it the connections would stack on the long-lived controller).
    QMetaObject::Connection auto_bias_conn_;

    std::vector<BiasRow> rows_;
    bool populated_{false};

    // Debounce for wheel/keyboard slider edits (audit §六-U1): those never
    // emit sliderReleased, so without this they updated the UI but never
    // wrote to the hardware. ~300 ms after the last change the pending bias
    // is applied once — no USB-write flooding during continuous adjustments.
    QTimer apply_debounce_;
    /// Rows edited via wheel/keyboard since the last debounce fire. A
    /// single-slot pending name silently dropped every earlier row when the
    /// user swept several biases within one 300 ms window.
    std::vector<std::string> pending_applies_;
};

} // namespace gui

#endif // GUI_PANELS_BIASES_PANEL_H
