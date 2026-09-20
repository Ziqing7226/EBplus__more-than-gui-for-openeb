// gui/recorder/record_dialog.h — recording setup dialog (ExportDialog-style).
//
// Collects the output path (auto .raw suffix, timestamped default) and the
// "save biases alongside" option, then asks MainWindow to start the actual
// recording (the recorder itself lives in RecorderController).

#ifndef GUI_RECORDER_RECORD_DIALOG_H
#define GUI_RECORDER_RECORD_DIALOG_H

#include <QDialog>

class QCheckBox;
class QLabel;
class QLineEdit;
class QPushButton;

namespace gui {

class RecordDialog : public QDialog {
    Q_OBJECT
public:
    explicit RecordDialog(QWidget* parent = nullptr);
    /// @brief Phase 4: inivation sources record to AEDAT4 — switches the
    /// default suffix, file filter and hint text accordingly.
    void set_aedat4_mode(bool on);
    /// @brief Side-stream availability (from the connected source's
    /// capabilities). Checkboxes appear only for available streams; both
    /// default to included.
    void set_side_stream_capabilities(bool imu, bool aps);

signals:
    /// @brief Emitted when the user confirms: record to @p path, optionally
    /// saving the current biases alongside as <base>.bias and including the
    /// IMU / APS side streams (AEDAT4 recordings only).
    void start_recording(const QString& path, bool save_biases,
                         bool include_imu, bool include_aps);

private slots:
    void on_browse();
    void on_start();

private:
    void update_side_stream_rows();
    bool cap_imu_{false};
    bool cap_aps_{false};
    QLineEdit* edt_output_{nullptr};
    QPushButton* btn_browse_{nullptr};
    QCheckBox* chk_biases_{nullptr};
    QCheckBox* chk_imu_{nullptr};
    QCheckBox* chk_aps_{nullptr};
    QLabel* lbl_status_{nullptr};
    QPushButton* btn_start_{nullptr};
    QPushButton* btn_close_{nullptr};
    bool aedat4_{false};
};

} // namespace gui

#endif // GUI_RECORDER_RECORD_DIALOG_H
