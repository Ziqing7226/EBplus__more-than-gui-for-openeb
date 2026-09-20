// gui/recorder/record_dialog.cpp

#include "record_dialog.h"

#include <QCheckBox>
#include <QDateTime>
#include <QDir>
#include <QFileDialog>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QStandardPaths>

namespace gui {
namespace {
QString suffix_for(bool aedat4) { return aedat4 ? QStringLiteral(".aedat4") : QStringLiteral(".raw"); }
} // namespace

RecordDialog::RecordDialog(QWidget* parent) : QDialog(parent) {
    setWindowTitle(tr("Start Recording"));

    auto* form = new QFormLayout(this);

    auto* row = new QHBoxLayout;
    edt_output_ = new QLineEdit(this);
    // Timestamped default so consecutive recordings don't collide.
    const QString dir =
        QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation) +
        QStringLiteral("/EBplus/recordings");
    edt_output_->setText(
        dir + QStringLiteral("/rec_") +
        QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss")) +
        QStringLiteral(".raw"));
    btn_browse_ = new QPushButton(tr("Browse..."), this);
    row->addWidget(edt_output_);
    row->addWidget(btn_browse_);
    form->addRow(tr("Output:"), row);

    chk_biases_ = new QCheckBox(tr("Save biases alongside (.bias)"), this);
    chk_biases_->setChecked(true);
    chk_biases_->setToolTip(
        tr("Stores the camera's current bias configuration next to the RAW file "
           "so the recording is reproducible (best-effort, like Metavision Viewer)."));
    form->addRow(QString(), chk_biases_);

    // Side streams (AEDAT4 recordings only; visible only for sources that
    // actually have the stream — set_side_stream_capabilities).
    chk_imu_ = new QCheckBox(tr("Include IMU samples"), this);
    chk_imu_->setChecked(true);
    chk_imu_->setVisible(false);
    form->addRow(QString(), chk_imu_);
    chk_aps_ = new QCheckBox(tr("Include APS frames"), this);
    chk_aps_->setChecked(true);
    chk_aps_->setVisible(false);
    form->addRow(QString(), chk_aps_);

    lbl_status_ = new QLabel(tr("Recording starts the live camera's RAW event log."), this);
    lbl_status_->setWordWrap(true);
    lbl_status_->setProperty("class", "hint");
    form->addRow(lbl_status_);

    auto* btn_row = new QHBoxLayout;
    btn_start_ = new QPushButton(tr("Start"), this);
    btn_close_ = new QPushButton(tr("Close"), this);
    btn_row->addWidget(btn_start_);
    btn_row->addWidget(btn_close_);
    form->addRow(btn_row);

    set_aedat4_mode(false);  // default RAW mode (after lbl_status_ exists)

    connect(btn_browse_, &QPushButton::clicked, this, &RecordDialog::on_browse);
    connect(btn_start_, &QPushButton::clicked, this, &RecordDialog::on_start);
    connect(btn_close_, &QPushButton::clicked, this, &QDialog::reject);
}

void RecordDialog::set_aedat4_mode(bool on) {
    aedat4_ = on;
    if (chk_imu_) update_side_stream_rows();
    const QString suffix = suffix_for(on);
    QString path = edt_output_->text();
    if (!path.isEmpty()) {
        const int dot = path.lastIndexOf(QLatin1Char('.'));
        if (dot > 0) path = path.left(dot);
        path += suffix;
        edt_output_->setText(path);
    }
    if (!lbl_status_) return;  // constructor pre-phase (hint set there)
    lbl_status_->setText(on
        ? tr("Recording writes the inivation camera's raw event stream into an "
             "AEDAT4 (DV-format) file.")
        : tr("Recording starts the live camera's RAW event log."));
    if (chk_imu_) update_side_stream_rows();
}

void RecordDialog::set_side_stream_capabilities(bool imu, bool aps) {
    cap_imu_ = imu;
    cap_aps_ = aps;
    if (chk_imu_) update_side_stream_rows();
}

void RecordDialog::update_side_stream_rows() {
    // Side-stream rows exist only for AEDAT4 recordings of sources that
    // actually have the stream (checked by default = include).
    chk_imu_->setVisible(aedat4_ && cap_imu_);
    chk_aps_->setVisible(aedat4_ && cap_aps_);
}

void RecordDialog::on_browse() {
    const QString path = QFileDialog::getSaveFileName(
        this, tr("Record to file"), edt_output_->text(),
        aedat4_ ? tr("AEDAT4 files (*.aedat4);;All files (*)")
                : tr("RAW files (*.raw);;All files (*)"));
    if (!path.isEmpty()) edt_output_->setText(path);
}

void RecordDialog::on_start() {
    QString path = edt_output_->text();
    if (path.isEmpty()) {
        lbl_status_->setText(tr("An output path is required."));
        return;
    }
    // Ensure the expected extension is present so downstream tools can
    // identify the file format.
    const QString suffix = suffix_for(aedat4_);
    if (!path.endsWith(suffix, Qt::CaseInsensitive)) path += suffix;
    edt_output_->setText(path);
    // Create the output directory if needed (the timestamped default lives
    // in ~/Documents/EBplus/recordings which may not exist yet).
    QDir().mkpath(QFileInfo(path).absolutePath());
    emit start_recording(path, chk_biases_->isChecked(),
                         chk_imu_->isChecked(), chk_aps_->isChecked());
    accept();
}

} // namespace gui
