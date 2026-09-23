// algo/common/kalman_filter.h — 2D position/velocity Kalman filter.
//
// Inspired by jAER KalmanFilter (labyrinthkalman). Constant-velocity model for
// 2D tracking: state = [x, y, vx, vy], measurement = [x, y]. Kept as a
// jAER-style building block (the event-level trackers that used it were
// removed in the 2026-08-22 provenance cull; currently test-only).
// Header-only; relies on small
// fixed-size Eigen-style arithmetic implemented with plain doubles to avoid a
// hard Eigen dependency in the common layer.

#ifndef GUI_ALGO_COMMON_KALMAN_FILTER_H
#define GUI_ALGO_COMMON_KALMAN_FILTER_H

#include <cmath>

namespace gui_algo {

/// @brief Constant-velocity Kalman filter for 2D position + velocity.
class KalmanFilter2D {
public:
    /// @brief Constructs the filter with process/measurement noise tuning.
    /// @param process_noise_std Per-step position process noise (px).
    /// @param measurement_noise_std Measurement noise std-dev (px).
    /// @param dt Assumed time step between updates (seconds).
    KalmanFilter2D(double process_noise_std = 1.0,
                   double measurement_noise_std = 2.0,
                   double dt = 0.033)
        : dt_(dt), q_(process_noise_std * process_noise_std),
          r_(measurement_noise_std * measurement_noise_std) {}

    /// @brief Initializes the state at a given position with zero velocity.
    void init(double x, double y) {
        x_ = x; y_ = y; vx_ = 0.0; vy_ = 0.0;
        p_xx_ = q_; p_yy_ = q_;
        p_vxvx_ = q_; p_vyvy_ = q_;
        p_xvx_ = 0.0; p_yvy_ = 0.0;
        initialized_ = true;
    }

    /// @brief Predict step: advance state by constant-velocity model.
    void predict() {
        if (!initialized_) return;
        x_ += vx_ * dt_;
        y_ += vy_ * dt_;
        // Covariance prediction: P' = F P F^T + Q, where F is the
        // constant-velocity transition matrix. The F P F^T propagation includes
        // the cross-term 2*dt*P_xvx. The process noise Q follows jAER's
        // constant-acceleration model per axis:
        //   Q = sigma2 * [[dt^4/4, dt^3/2], [dt^3/2, dt^2]],  sigma2 = q_.
        const double dt2 = dt_ * dt_;
        const double dt3 = dt2 * dt_;
        const double dt4 = dt2 * dt2;
        p_xx_ += 2.0 * dt_ * p_xvx_ + dt2 * p_vxvx_ + q_ * dt4 / 4.0;
        p_yy_ += 2.0 * dt_ * p_yvy_ + dt2 * p_vyvy_ + q_ * dt4 / 4.0;
        p_xvx_ += dt_ * p_vxvx_ + q_ * dt3 / 2.0;
        p_yvy_ += dt_ * p_vyvy_ + q_ * dt3 / 2.0;
        p_vxvx_ += q_ * dt2;
        p_vyvy_ += q_ * dt2;
    }

    /// @brief Update step with a position measurement (mx, my).
    void update(double mx, double my) {
        if (!initialized_) { init(mx, my); return; }
        // Innovation.
        const double ix = mx - x_;
        const double iy = my - y_;
        // Innovation covariance S = P + R.
        const double sx = p_xx_ + r_;
        const double sy = p_yy_ + r_;
        // Kalman gain K = P / S.
        const double kx = p_xx_ / sx;
        const double ky = p_yy_ / sy;
        const double k_vx = p_xvx_ / sx;
        const double k_vy = p_yvy_ / sy;
        // State update.
        x_ += kx * ix;
        y_ += ky * iy;
        vx_ += k_vx * ix;
        vy_ += k_vy * iy;
        // Covariance update P = (I - K H) P.
        // Must use pre-update P values for all entries.
        const double old_p_xx = p_xx_, old_p_yy = p_yy_;
        const double old_p_xvx = p_xvx_, old_p_yvy = p_yvy_;
        const double old_p_vxvx = p_vxvx_, old_p_vyvy = p_vyvy_;
        p_xx_ = (1.0 - kx) * old_p_xx;
        p_yy_ = (1.0 - ky) * old_p_yy;
        p_xvx_ = old_p_xvx - k_vx * old_p_xx;
        p_yvy_ = old_p_yvy - k_vy * old_p_yy;
        p_vxvx_ = old_p_vxvx - k_vx * old_p_xvx;
        p_vyvy_ = old_p_vyvy - k_vy * old_p_yvy;
    }

    double x() const { return x_; }
    double y() const { return y_; }
    double vx() const { return vx_; }
    double vy() const { return vy_; }
    bool initialized() const { return initialized_; }

    // Read-only covariance/noise accessors (kept for Mahalanobis-style
    // gating; the multi-hypothesis object_tracker mode is gone). The
    // position covariance is diagonal (no x-y cross term), so a 2D
    // Mahalanobis distance factorises as (dx^2/Sx + dy^2/Sy) with
    // Sx = p_xx + r, Sy = p_yy + r. These getters do not alter filter state.
    double p_xx() const { return p_xx_; }
    double p_yy() const { return p_yy_; }
    double measurement_noise_var() const { return r_; }

    void set_dt(double dt) { dt_ = dt; }

private:
    double dt_;
    double q_;   // process noise variance
    double r_;   // measurement noise variance
    double x_{0}, y_{0}, vx_{0}, vy_{0};
    double p_xx_{1}, p_yy_{1}, p_vxvx_{1}, p_vyvy_{1};
    double p_xvx_{0}, p_yvy_{0};
    bool initialized_{false};
};

} // namespace gui_algo

#endif // GUI_ALGO_COMMON_KALMAN_FILTER_H
