// gui/widgets/imu_window.h — DV-style live IMU visualization: the camera is
// drawn as a cuboid whose 3D pose follows the gyro-integrated IMU output,
// with numeric accel/gyro/temperature readouts.
//
// IP note: fresh implementation. iniVation's dv-gui ships a custom
// (non-standard) license — none of its code is used or referenced; only the
// generic concept (a 3D box posed by the IMU) is shared, which is not
// protectable expression. The pose math below is written from scratch on
// standard quaternion kinematics.

#ifndef GUI_WIDGETS_IMU_WINDOW_H
#define GUI_WIDGETS_IMU_WINDOW_H

#include <QDockWidget>
#include <QElapsedTimer>
#include <QString>

#include <limits>

#include "davis/imu_pose.h"
#include "davis/imu_types.h"

class QTimer;

namespace gui {

class CameraController;

/// Dockable window (AlgoWindow style, right dock area): the camera drawn
/// as a cuboid whose pose follows the IMU, with numeric readouts.
class ImuWindow : public QDockWidget {
    Q_OBJECT
public:
    explicit ImuWindow(CameraController* controller, QWidget* parent = nullptr);

signals:
    /// Emitted when the user closes the window (the host unchecks the
    /// Devices-panel toggle and stops the stream).
    void window_closed();

protected:
    void closeEvent(QCloseEvent* event) override;

private:
    void refresh();
    /// Renders the status line, the camera cuboid and the body axes into
    /// the dock's content canvas (invoked by the canvas's paintEvent).
    void render(QPainter& p, const QRectF& r);

    class Canvas;
    Canvas* canvas_;
    CameraController* controller_;
    QTimer* timer_;
    /// Status line, drawn inside the pose canvas (no themed label strip).
    QString status_text_;

    /// Camera attitude: gravity-aligned initialisation + gyro integration
    /// with gravity correction and a stationary-gated gyro-bias estimate
    /// (see davis/imu_pose.h — unit-tested against synthetic motion).
    davis::ImuPose pose_;

    std::int64_t imu_cursor_{std::numeric_limits<std::int64_t>::min()};
    std::int64_t last_t_seen_{-1};
    long last_count_{0};
    long rate_accum_events_{0};
    double rate_accum_time_{0};
    double smoothed_rate_{0};
    QElapsedTimer rate_clock_;
};

} // namespace gui

#endif // GUI_WIDGETS_IMU_WINDOW_H
