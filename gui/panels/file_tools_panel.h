// gui/panels/file_tools_panel.h — offline file conversion tools UI (design §1.5.7).
//
// Buttons for: Convert to HDF5, Convert to CSV, File Cutter, File Info.
// Delegates all work to FileConverter.

#ifndef GUI_PANELS_FILE_TOOLS_PANEL_H
#define GUI_PANELS_FILE_TOOLS_PANEL_H

#include <QWidget>

#include <functional>

#include "abstract_panel.h"
#include "file_op_dialog.h"
#include "file_info_dialog.h"

class QProgressBar;
class QLabel;
class QPushButton;

namespace gui {

class FileConverter;

class FileToolsPanel : public AbstractPanel {
    Q_OBJECT
public:
    explicit FileToolsPanel(FileConverter* converter, QWidget* parent = nullptr);
    /// Joins the file-info query worker (deletes the dialog) — must run
    /// while the shared FileConverter is still alive.
    void shutdown_info_dialog();

    QString panel_id() const override { return QStringLiteral("file_tools"); }
    QString panel_title() const override { return tr("File Tools"); }
    QString panel_group() const override { return QStringLiteral("Tools"); }

    /// @brief Enable/disable the Start Recording button (enabled when a
    /// camera is connected, disabled during recording).
    void set_record_enabled(bool enabled);
    /// @brief Enable/disable the Stop Recording button (enabled during
    /// recording, disabled otherwise).
    void set_stop_enabled(bool enabled);
    /// @brief Enable/disable the Export button (enabled when a camera is
    /// connected or a file is open).
    void set_export_enabled(bool enabled);

    /// @brief Provides the currently open file's path for dialog prefill
    /// (MainWindow wires this to the playback controller).
    void set_source_provider(std::function<QString()> provider) {
        source_provider_ = std::move(provider);
    }

signals:
    /// Emitted when the user clicks "Start Recording".
    void record_start_requested();
    /// Emitted when the user clicks "Stop Recording".
    void record_stop_requested();
    /// Emitted when the user clicks "Export...".
    void export_requested();

    private slots:
    void on_convert_hdf5();
    void on_convert_csv();
    void on_cutter();
    void on_info();
    void on_completed(const QString& out);
    void on_failed(const QString& msg);

private:
    void set_buttons_enabled(bool enabled);
    void open_op_dialog(FileOpDialog::Mode mode);

    FileConverter* converter_;
    std::function<QString()> source_provider_;
    FileOpDialog* op_dialog_{nullptr};
    FileInfoDialog* info_dialog_{nullptr};
    QPushButton* btn_hdf5_{nullptr};
    QPushButton* btn_csv_{nullptr};
    QPushButton* btn_cut_{nullptr};
    QPushButton* btn_info_{nullptr};
    QPushButton* btn_record_{nullptr};
    QPushButton* btn_stop_{nullptr};
    QPushButton* btn_export_{nullptr};
    QProgressBar* progress_{nullptr};
    QLabel* lbl_status_{nullptr};
};

} // namespace gui

#endif // GUI_PANELS_FILE_TOOLS_PANEL_H
