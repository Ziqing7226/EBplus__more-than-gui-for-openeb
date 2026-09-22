// gui/widgets/aps_window.cpp — see aps_window.h.

#include "aps_window.h"

#include <QImage>
#include <QPixmap>
#include <QTimer>
#include <QVBoxLayout>

#include "app/camera_controller.h"

namespace gui {

ApsWindow::ApsWindow(CameraController* controller, QWidget* parent)
    : QDockWidget(tr("APS Frames"), parent), controller_(controller) {
    setObjectName("ApsDock");
    setAttribute(Qt::WA_DeleteOnClose);
    setFeatures(QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetClosable |
                QDockWidget::DockWidgetFloatable);
    setAllowedAreas(Qt::RightDockWidgetArea | Qt::LeftDockWidgetArea);

    auto* content = new QWidget(this);
    auto* layout = new QVBoxLayout(content);
    layout->setContentsMargins(8, 8, 8, 8);  // the narrow gap around the preview
    setWidget(content);

    // Grayscale preview, scaled to fit while preserving aspect ratio.
    image_label_ = new QLabel(content);
    image_label_->setAlignment(Qt::AlignCenter);
    image_label_->setMinimumSize(340, 240);
    image_label_->setStyleSheet(QStringLiteral("background: black;"));
    layout->addWidget(image_label_, 1);

    status_label_ = new QLabel(content);
    layout->addWidget(status_label_);

    timer_ = new QTimer(this);
    timer_->setInterval(33);
    connect(timer_, &QTimer::timeout, this, &ApsWindow::refresh);
    timer_->start();
    rate_clock_.start();
    refresh();
}

void ApsWindow::refresh() {
    const auto frame = controller_->latest_aps_frame();
    const long count = controller_->aps_frame_count();

    if (frame.valid && !frame.image.empty()) {
        // Build the QImage with its OWN 32-bit-aligned scanlines and
        // convert directly into it. Wrapping the cv::Mat buffer exposes an
        // unaligned stride (346*3 = 1038 bytes) to Qt's optimized image
        // conversions, which intermittently renders the WHOLE frame white
        // (offline-reproduced: 84 white renders out of 156 via the wrap,
        // 0 out of 156 via this pattern — the flashing white frames on the
        // color 346, in replay and live alike).
        const int chn = frame.image.channels();
        QImage img(frame.image.cols, frame.image.rows,
                   chn == 3 ? QImage::Format_RGB888 : QImage::Format_Grayscale8);
        cv::Mat wrap(img.height(), img.width(), CV_8UC(chn == 3 ? 3 : 1),
                     img.bits(), static_cast<size_t>(img.bytesPerLine()));
        if (chn == 3) {
            cv::cvtColor(frame.image, wrap, cv::COLOR_BGR2RGB);
        } else {
            frame.image.copyTo(wrap);
        }
        image_label_->setPixmap(QPixmap::fromImage(img).scaled(
            image_label_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    }

    // Accumulate over >= 0.5 s windows: the 33 ms tick interval is below
    // any sane rate gate, and per-tick deltas of a ~20 fps stream would
    // quantize to 0/1 events (the rate display read 0.0 Hz forever).
    const double elapsed_s = rate_clock_.restart() / 1000.0;
    rate_accum_time_ += elapsed_s;
    rate_accum_events_ += count - last_count_;
    last_count_ = count;
    if (rate_accum_time_ >= 0.5) {
        const double inst =
            static_cast<double>(rate_accum_events_) / rate_accum_time_;
        smoothed_rate_ =
            smoothed_rate_ > 0 ? (0.7 * smoothed_rate_ + 0.3 * inst) : inst;
        rate_accum_events_ = 0;
        rate_accum_time_ = 0;
    }

    if (count == 0) {
        status_label_->setText(tr("Waiting for frames…\n(Stream runs only while the camera streams)"));
    } else {
        status_label_->setText(tr("Frames: %1   Rate: %2 Hz   %3×%4")
                                   .arg(count)
                                   .arg(smoothed_rate_, 5, 'f', 1)
                                   .arg(frame.image.cols)
                                   .arg(frame.image.rows));
    }
}

void ApsWindow::closeEvent(QCloseEvent* event) {
    emit window_closed();
    QDockWidget::closeEvent(event);
}

} // namespace gui
