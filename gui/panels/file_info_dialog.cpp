// gui/panels/file_info_dialog.cpp

#include "file_info_dialog.h"

#include <QFileDialog>
#include <QCoreApplication>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMetaObject>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QThread>

#include "app/file_converter.h"

namespace gui {

FileInfoDialog::FileInfoDialog(FileConverter* converter, QWidget* parent)
    : QDialog(parent), converter_(converter) {
    setWindowTitle(tr("File Info"));
    resize(480, 320);

    auto* form = new QFormLayout(this);

    auto* row = new QHBoxLayout;
    edt_source_ = new QLineEdit(this);
    btn_browse_ = new QPushButton(tr("Browse..."), this);
    btn_query_ = new QPushButton(tr("Query"), this);
    row->addWidget(edt_source_);
    row->addWidget(btn_browse_);
    row->addWidget(btn_query_);
    form->addRow(tr("Source:"), row);

    text_ = new QPlainTextEdit(this);
    text_->setReadOnly(true);
    form->addRow(text_);

    lbl_status_ = new QLabel(tr("Ready."), this);
    lbl_status_->setWordWrap(true);
    form->addRow(lbl_status_);

    btn_close_ = new QPushButton(tr("Close"), this);
    form->addRow(btn_close_);

    connect(btn_browse_, &QPushButton::clicked, this, &FileInfoDialog::on_browse);
    connect(btn_query_, &QPushButton::clicked, this, &FileInfoDialog::on_query);
    connect(btn_close_, &QPushButton::clicked, this, &QDialog::reject);
}

void FileInfoDialog::set_source(const QString& path) {
    edt_source_->setText(path);
    if (!path.isEmpty()) start_query(path);
}

void FileInfoDialog::on_browse() {
    const QString src = QFileDialog::getOpenFileName(
        this, tr("Source file"), edt_source_->text(),
        tr("Event files (*.raw *.hdf5 *.h5 *.dat *.aedat4 *.alpdata);;All files (*)"));
    if (!src.isEmpty()) set_source(src);
}

void FileInfoDialog::on_query() {
    const QString src = edt_source_->text();
    if (src.isEmpty()) {
        lbl_status_->setText(tr("A source path is required."));
        return;
    }
    start_query(src);
}

QString FileInfoDialog::format_info(const FileInfo& fi) {
    QString text;
    text += QStringLiteral("Path: %1\n").arg(fi.path);
    text += QStringLiteral("Integrator: %1\n").arg(fi.integrator);
    text += QStringLiteral("Serial: %1\n").arg(fi.serial);
    text += QStringLiteral("Plugin: %1\n").arg(fi.plugin);
    text += QStringLiteral("Encoding: %1\n").arg(fi.encoding);
    text += QStringLiteral("Geometry: %1 x %2\n").arg(fi.width).arg(fi.height);
    if (fi.duration_us > 0) {
        text += QStringLiteral("Duration: %1 us (%2 s)\n")
                    .arg(fi.duration_us)
                    .arg(fi.duration_us / 1.0e6, 0, 'f', 3);
    } else {
        text += QStringLiteral("Duration: unknown\n");
    }
    return text;
}

void FileInfoDialog::start_query(const QString& path) {
    if (!converter_ || querying_) return;
    querying_ = true;
    btn_query_->setEnabled(false);
    lbl_status_->setText(tr("Querying (large files may take a while)…"));
    text_->setPlainText(QString());

    // Run the (potentially blocking) info query on a worker thread so the
    // GUI never freezes. QPointer guard: if the dialog is closed before the
    // query returns, the result is discarded safely.
    QPointer<FileInfoDialog> self(this);
    FileConverter* conv = converter_;
    if (worker_.joinable()) worker_.join();
    worker_ = std::thread([self, conv, path]() {
        FileInfo fi;
        QString err;
        try {
            fi = conv->info(path);
            // Prefer the provider's known duration (instant, non-blocking)
            // over whatever the (possibly slow) OSC query returned.
            const auto dur = conv->duration_of(path);
            if (dur > 0) fi.duration_us = dur;
        } catch (const std::exception& e) {
            err = QString::fromUtf8(e.what());
        } catch (...) {
            err = QStringLiteral("unknown error");
        }
        QMetaObject::invokeMethod(QCoreApplication::instance(), [self, fi, err]() {
            if (!self) return;
            self->querying_ = false;
            self->btn_query_->setEnabled(true);
            if (err.isEmpty()) {
                self->text_->setPlainText(format_info(fi));
                self->lbl_status_->setText(tr("Done."));
            } else {
                self->lbl_status_->setText(tr("Failed to read file info: %1").arg(err));
            }
        }, Qt::QueuedConnection);
    });
}

FileInfoDialog::~FileInfoDialog() {
    // Must not outlive the converter: MainWindow::closeEvent deletes this
    // dialog (before the converter member dies) and the join blocks until
    // the in-flight query returns.
    if (worker_.joinable()) worker_.join();
}

} // namespace gui
