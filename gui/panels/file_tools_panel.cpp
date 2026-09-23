// gui/panels/file_tools_panel.cpp

#include "file_tools_panel.h"

#include <QCursor>
#include <QDialogButtonBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGroupBox>
#include <QGuiApplication>
#include <QHBoxLayout>
#include <QKeySequence>
#include <QLabel>
#include <QMessageBox>
#include <QProgressBar>
#include <QPushButton>
#include <QSpinBox>
#include <QVBoxLayout>

#include "app/file_converter.h"

namespace gui {

FileToolsPanel::FileToolsPanel(FileConverter* converter, QWidget* parent)
    : AbstractPanel(parent), converter_(converter) {
    auto* outer = new QVBoxLayout(this);
    outer->setContentsMargins(0, 0, 0, 0);
    auto* gb = new QGroupBox(tr("File Tools"), this);
    auto* lay = new QVBoxLayout(gb);

    btn_hdf5_ = new QPushButton(tr("Convert to HDF5..."), gb);
    btn_csv_  = new QPushButton(tr("Convert to CSV..."), gb);
    btn_cut_  = new QPushButton(tr("File Cutter..."), gb);
    btn_info_ = new QPushButton(tr("File Info..."), gb);
    progress_ = new QProgressBar(gb);
    progress_->setRange(0, 100);
    // Hide the progress bar when idle so it doesn't look like an empty box
    // (§14.6). It's shown only during conversion/cutting operations.
    progress_->setVisible(false);
    lbl_status_ = new QLabel(tr("Ready."), gb);

    // Recording + Export controls (moved from File menu / toolbar — §14.5).
    btn_record_ = new QPushButton(tr("Start Recording..."), gb);
    btn_record_->setShortcut(QKeySequence("R"));
    btn_stop_   = new QPushButton(tr("Stop Recording"), gb);
    btn_export_ = new QPushButton(tr("Export..."), gb);
    btn_stop_->setEnabled(false);
    btn_record_->setEnabled(false);
    btn_export_->setEnabled(false);

    lay->addWidget(btn_hdf5_);
    lay->addWidget(btn_csv_);
    lay->addWidget(btn_cut_);
    lay->addWidget(btn_info_);
    lay->addWidget(progress_);
    lay->addWidget(lbl_status_);
    lay->addSpacing(8);
    lay->addWidget(btn_record_);
    lay->addWidget(btn_stop_);
    lay->addWidget(btn_export_);

    outer->addWidget(gb);

    connect(btn_hdf5_, &QPushButton::clicked, this, &FileToolsPanel::on_convert_hdf5);
    connect(btn_csv_,  &QPushButton::clicked, this, &FileToolsPanel::on_convert_csv);
    connect(btn_cut_,  &QPushButton::clicked, this, &FileToolsPanel::on_cutter);
    connect(btn_info_, &QPushButton::clicked, this, &FileToolsPanel::on_info);
    connect(btn_record_, &QPushButton::clicked, this, &FileToolsPanel::record_start_requested);
    connect(btn_stop_,   &QPushButton::clicked, this, &FileToolsPanel::record_stop_requested);
    connect(btn_export_, &QPushButton::clicked, this, &FileToolsPanel::export_requested);

    if (converter_) {
        connect(converter_, &FileConverter::completed, this, &FileToolsPanel::on_completed);
        connect(converter_, &FileConverter::failed, this, &FileToolsPanel::on_failed);
        connect(converter_, &FileConverter::progress, this, [this](double r) {
            progress_->setValue(static_cast<int>(r * 100));
        });
    }
}

void FileToolsPanel::set_buttons_enabled(bool enabled) {
    btn_hdf5_->setEnabled(enabled);
    btn_csv_->setEnabled(enabled);
    btn_cut_->setEnabled(enabled);
    btn_info_->setEnabled(enabled);
}

void FileToolsPanel::set_record_enabled(bool enabled) {
    btn_record_->setEnabled(enabled);
}

void FileToolsPanel::set_stop_enabled(bool enabled) {
    btn_stop_->setEnabled(enabled);
}

void FileToolsPanel::set_export_enabled(bool enabled) {
    btn_export_->setEnabled(enabled);
}

void FileToolsPanel::on_convert_hdf5() {
    open_op_dialog(FileOpDialog::Mode::ConvertHdf5);
}

void FileToolsPanel::on_convert_csv() {
    open_op_dialog(FileOpDialog::Mode::ConvertCsv);
}

void FileToolsPanel::on_cutter() {
    open_op_dialog(FileOpDialog::Mode::Cut);
}

void FileToolsPanel::open_op_dialog(FileOpDialog::Mode mode) {
    if (!converter_) return;
    if (!op_dialog_) {
        op_dialog_ = new FileOpDialog(mode, converter_, this);
        op_dialog_->setAttribute(Qt::WA_DeleteOnClose);
        connect(op_dialog_, &QObject::destroyed, this, [this]() {
            op_dialog_ = nullptr;
        });
    }
    op_dialog_->set_source(source_provider_ ? source_provider_() : QString());
    op_dialog_->show();
    op_dialog_->raise();
    op_dialog_->activateWindow();
}

void FileToolsPanel::on_info() {
    if (!converter_) return;
    if (!info_dialog_) {
        info_dialog_ = new FileInfoDialog(converter_, this);
        info_dialog_->setAttribute(Qt::WA_DeleteOnClose);
        connect(info_dialog_, &QObject::destroyed, this, [this]() {
            info_dialog_ = nullptr;
        });
    }
    info_dialog_->set_source(source_provider_ ? source_provider_() : QString());
    info_dialog_->show();
    info_dialog_->raise();
    info_dialog_->activateWindow();
}

void FileToolsPanel::on_completed(const QString& out) {
    progress_->setValue(100);
    progress_->setVisible(false);
    lbl_status_->setText(tr("Done: %1").arg(out));
    set_buttons_enabled(true);
}

void FileToolsPanel::on_failed(const QString& msg) {
    progress_->reset();
    progress_->setVisible(false);
    lbl_status_->setText(tr("Failed: %1").arg(msg));
    set_buttons_enabled(true);
}

void FileToolsPanel::shutdown_info_dialog() {
    // ~FileInfoDialog joins the query worker; called from MainWindow's
    // close event while converter_ (MainWindow's file_converter_ member)
    // is still alive.
    delete info_dialog_;
    info_dialog_ = nullptr;
}

} // namespace gui
