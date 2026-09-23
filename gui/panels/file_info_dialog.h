// gui/panels/file_info_dialog.h — asynchronous file-info dialog.
//
// Replaces the synchronous QMessageBox-based File Info flow, which ran
// FileConverter::info() on the GUI thread and could freeze the UI for
// seconds on large files (blocking OSC duration query / index build).
// The query runs on a worker thread; the dialog stays responsive and the
// user can close it anytime (the result is discarded via QPointer guard).

#ifndef GUI_PANELS_FILE_INFO_DIALOG_H
#define GUI_PANELS_FILE_INFO_DIALOG_H

#include <QDialog>
#include <QPointer>

#include <thread>

#include <metavision/sdk/base/utils/timestamp.h>

class QLabel;
class QLineEdit;
class QPushButton;
class QPlainTextEdit;

namespace gui {

class FileConverter;

class FileInfoDialog : public QDialog {
    Q_OBJECT
public:
    FileInfoDialog(FileConverter* converter, QWidget* parent = nullptr);
    ~FileInfoDialog() override;

    /// @brief Pre-fills the source path and starts the async query.
    void set_source(const QString& path);

private slots:
    void on_browse();
    void on_query();

private:
    void start_query(const QString& path);
    static QString format_info(const struct FileInfo& fi);

    FileConverter* converter_;
    bool querying_{false};
    /// Joinable handle for the info-query worker. The thread dereferences
    /// the MainWindow-owned FileConverter, which dies before this dialog
    /// (a deep Qt child) is destroyed — the join in our destructor (invoked
    /// from MainWindow::closeEvent via the panel) is what keeps the query
    /// from outliving the converter.
    std::thread worker_;

    QLineEdit* edt_source_{nullptr};
    QPushButton* btn_browse_{nullptr};
    QPushButton* btn_query_{nullptr};
    QPlainTextEdit* text_{nullptr};
    QLabel* lbl_status_{nullptr};
    QPushButton* btn_close_{nullptr};
};

} // namespace gui

#endif // GUI_PANELS_FILE_INFO_DIALOG_H
