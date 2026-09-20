// gui/davis/imu_pose.h — IMU attitude estimation for the IMU window.
//
// Gyro-dominant design modeled on dv-processing's RotationIntegrator
// (the reference has NO accel-fused attitude estimator: it integrates
// the gyroscope alone and takes the gyro bias as a CONSTANT offset,
// measured offline by its imu-bias-estimation utility — a static
// capture averaged over ~1 s). We keep that constant-offset model but
// refine the constant online, ONLY while the chip is provably still:
// near 1 g, below 10 deg/s, AND with the attitude already agreeing
// with gravity (tilt error small). Under those conditions gyro − bias
// is pure bias, so the estimate converges in seconds and keeps
// tracking temperature drift over long sessions; any real motion
// freezes it, so it cannot absorb the specific-force error that
// corrupts online bias trackers (hardware-proven failure mode on the
// DAVIS346: a Mahony integral driven from the accel error produced a
// large closed-path residual).
//
// On top sits a weak roll/pitch correction from the accelerometer gravity
// reference (Mahony et al., IEEE TAC 2008 — proportional term only),
// applied ONLY at rest (near 1 g AND below 10 deg/s — the same rest gate
// as the bias leak). Between rest points the attitude is PURE
// bias-subtracted gyro integration: during motion the accelerometer
// measures specific force, not gravity, and letting it touch the attitude
// feeds path error into the integrator and breaks loop closure (observed
// on the DAVIS346; the reference RotationIntegrator is "perfect" at
// closed paths for exactly the reason that it never lets the accel near
// the attitude). At rest the gravity reference re-anchors roll/pitch, so
// the display stays meaningful over long sessions while every motion
// segment stays gyro-pure. At rest the attitude is additionally pulled
// back onto the DEFAULT pose (the unique alignment attitude: z out of
// the lens, y up) about the world vertical — the heading component no
// 6-axis sensor can absolute-reference; the pull is rate-capped like
// the tilt anchor, so the re-upright is a slow slew, never a snap.
//
// Initialization is instant: the first near-1 g sample aligns the
// attitude (bias starts at 0 and refines in the background; the accel
// correction already holds roll/pitch to ~bias/(2kp) ≈ 0.4 deg in the
// meantime).
//
// Chip-agnostic: consumes gyro (deg/s) and accel (g) samples, so it
// works unchanged for DAVIS346 and DVXplorer (the wire decoders
// already normalise units and axis order). Header-only and
// dependency-free so the math is unit-testable.

#ifndef GUI_DAVIS_IMU_POSE_H
#define GUI_DAVIS_IMU_POSE_H

#include <algorithm>
#include <cmath>

#include "imu_types.h"

namespace gui::davis {

class ImuPose {
public:
    explicit ImuPose() = default;

    /// Drops the attitude and the bias estimate.
    void reset() {
        w_ = 1; x_ = 0; y_ = 0; z_ = 0;
        bias_x_ = bias_y_ = bias_z_ = 0;
        sustain_s_ = 0;
        gy_slow_x_ = gy_slow_y_ = gy_slow_z_ = 0;
        aligned_t_us_ = -1;
        rest_run_s_ = 0;
        aligned_ = false;
        last_t_ = -1;
    }

    /// Feeds one IMU sample (dt taken from the sample timestamps).
    void update(const ImuSample& s) {
        const double ax = s.accel_x, ay = s.accel_y, az = s.accel_z;
        const double amag = std::sqrt(ax * ax + ay * ay + az * az);
        const bool gravity_ok = amag > 0.7 && amag < 1.3;

        // Initial alignment: orient the sensor frame so the estimated
        // body-frame up (= R^T · (0,0,1), what the correction compares
        // against the accel) equals the measured gravity direction. That
        // is the rotation taking the measurement TO world up: axis = m × z.
        if (!aligned_) {
            if (!gravity_ok) return;
            const double nx = ax / amag, ny = ay / amag, nz = az / amag;
            const double angle = std::acos(std::clamp(nz, -1.0, 1.0));
            double axis_x = ny, axis_y = -nx;
            const double axis_norm = std::sqrt(axis_x * axis_x + axis_y * axis_y);
            if (axis_norm < 1e-12) {
                w_ = 1; x_ = y_ = z_ = 0;
            } else {
                const double half = angle / 2.0;
                axis_x /= axis_norm;
                axis_y /= axis_norm;
                w_ = std::cos(half);
                x_ = axis_x * std::sin(half);
                y_ = axis_y * std::sin(half);
                z_ = 0;
            }
            aligned_ = true;
            aligned_t_us_ = s.t;
            last_t_ = s.t;
            // Pinning the world frame: the default pose is THE unique
            // canonical attitude (z out of the lens toward the viewer,
            // y up, x left). A 6-axis IMU cannot sense compass heading,
            // so the one free parameter — the world's yaw reference — is
            // pinned HERE, at alignment: the camera's attitude at this
            // instant becomes the default pose, and it never changes
            // afterward. Every window-open renders the same default pose.
            default_w_ = w_;
            default_x_ = x_;
            default_y_ = y_;
            default_z_ = z_;
            return;
        }

        const double dt = static_cast<double>(s.t - last_t_) / 1e6;
        last_t_ = s.t;
        if (dt <= 0 || dt > 0.2) return;

        // Convert gyro to rad/s with the current offset removed.
        double wx = (s.gyro_x - bias_x_) * kDeg2Rad;
        double wy = (s.gyro_y - bias_y_) * kDeg2Rad;
        double wz = (s.gyro_z - bias_z_) * kDeg2Rad;

        // Smoothed rate envelope: handheld tremor spikes (5-7 deg/s) must
        // not toggle the stillness gate — stillness is judged on the
        // SUSTAINED rate, not the instantaneous one.
        const double env_a = std::min(1.0, dt / kGyroEnvelopeTauS);
        gy_slow_x_ += (s.gyro_x - gy_slow_x_) * env_a;
        gy_slow_y_ += (s.gyro_y - gy_slow_y_) * env_a;
        gy_slow_z_ += (s.gyro_z - gy_slow_z_) * env_a;
        const double gm_slow =
            std::sqrt(gy_slow_x_ * gy_slow_x_ + gy_slow_y_ * gy_slow_y_ +
                      gy_slow_z_ * gy_slow_z_);
        const double cmx = gy_slow_x_ - bias_x_, cmy = gy_slow_y_ - bias_y_,
                     cmz = gy_slow_z_ - bias_z_;
        const double gm_corr =
            std::sqrt(cmx * cmx + cmy * cmy + cmz * cmz);
        const bool quiet = gravity_ok && gm_corr < kRestGyroDps;
        rest_run_s_ = quiet ? rest_run_s_ + dt : 0.0;
        const bool anchored = quiet && rest_run_s_ >= kAnchorSettleS;
        if (anchored) {
            // STILLNESS = 正. At a genuine stop the WHOLE attitude (tilt
            // and heading together) is pulled onto the default pose along
            // the shortest arc, and snapped once within ~1 deg — the
            // residual is zero by fiat. The accelerometer defines
            // stillness (|a| ~ 1 g) but NEVER touches the attitude after
            // alignment: with the display's yaw-sign convention the
            // integrated frame is mirrored w.r.t. the accel frame, so a
            // gravity anchor would wrench the pose toward a direction the
            // convention does not share (measured on
            // rec_20260920_221204: a saturated 40 deg/s wrench during
            // handheld rest, anchor error standing at ~50 deg).
            const Q4 qd{default_w_, default_x_, default_y_, default_z_};
            const Q4 e = qmul(Q4{w_, x_, y_, z_}, Q4{qd.w, -qd.x, -qd.y, -qd.z});
            const double err_angle =
                2.0 * std::acos(std::clamp(std::abs(e.w), -1.0, 1.0));
            double nx_ = e.x, ny_ = e.y, nz_ = e.z;
            const double vn = std::sqrt(nx_ * nx_ + ny_ * ny_ + nz_ * nz_);
            if (vn > 1e-12) {
                nx_ /= vn;
                ny_ /= vn;
                nz_ /= vn;
            } else {
                nx_ = 0;
                ny_ = 0;
                nz_ = 1;
            }
            if (e.w < 0) {  // shortest arc
                nx_ = -nx_;
                ny_ = -ny_;
                nz_ = -nz_;
            }
            if (err_angle < kReUprightSnapRad) {
                w_ = default_w_;
                x_ = default_x_;
                y_ = default_y_;
                z_ = default_z_;
            } else {
                // Capped slew along the error axis (world frame,
                // left-multiplied): a quarter turn returns in ~1.1 s at
                // the 40 deg/s cap; small residuals trim at gain 2/s.
                const double omega =
                    std::min(kYawPullGain * err_angle, kMaxAnchorRateRadS);
                const double half = -omega * dt * 0.5;
                const Q4 step{std::cos(half), nx_ * std::sin(half),
                              ny_ * std::sin(half), nz_ * std::sin(half)};
                const Q4 corrected = qmul(step, Q4{w_, x_, y_, z_});
                w_ = corrected.w;
                x_ = corrected.x;
                y_ = corrected.y;
                z_ = corrected.z;
            }
        }

        // Bias refinement, frozen like the reference's constant offset.
        // The leak runs ONLY while (a) the chip is quasi-still by every
        // gate AND (b) one of: the initial warm-up since alignment (so the
        // zero-init estimate converges), or an unbroken >= 3 s park at
        // < 3 deg/s (a deliberate "put it down", which re-opens the leak
        // for temperature re-calibration). Mid-motion pauses and slow
        // rotations never sustain 3 s below 3 deg/s, so between parks the
        // bias is FROZEN — closed paths then close with pure
        // bias-subtracted integration, exactly like the reference.
        // Bias refinement, per the field rule: a rate that holds SUSTAINED
        // (>= 1.5 s) and moderate (<= 15 deg/s — anything faster is real
        // motion) is BY DEFINITION bias. The estimate tracks the smoothed
        // gyro envelope while that holds and freezes the instant the rate
        // leaves the band (fast or erratic motion). Once the envelope is
        // absorbed the corrected rate vanishes, stillness opens, and the
        // re-upright takes over (静止 = 正); at a true stop the envelope
        // reads the residual noise and the bias re-adapts to it, so the
        // absorption is self-correcting.
        // The instantaneous-vs-envelope difference gates out real motion
        // ONSET: a hard twist jumps gm far above the envelope on the very
        // first sample, long before the envelope itself leaves the band.
        const double gm_inst = std::sqrt(
            static_cast<double>(s.gyro_x) * s.gyro_x +
            static_cast<double>(s.gyro_y) * s.gyro_y +
            static_cast<double>(s.gyro_z) * s.gyro_z);
        const bool rate_stable = gravity_ok && gm_slow < kAbsorbRateDps &&
            std::fabs(gm_inst - gm_slow) < kAbsorbStableDps;
        if (rate_stable) {
            sustain_s_ += dt;
        } else {
            sustain_s_ = 0;
        }
        if (sustain_s_ >= kAbsorbHoldS) {
            const double a = dt / kBiasTauS;
            bias_x_ += (gy_slow_x_ - bias_x_) * a;
            bias_y_ += (gy_slow_y_ - bias_y_) * a;
            bias_z_ += (gy_slow_z_ - bias_z_) * a;
        }

        // Quaternion integration: q' = q + 0.5 * q (x) omega * dt.
        const Q4 q{w_, x_, y_, z_};
        const Q4 om{0, wx, wy, wz};
        const Q4 dq = qmul(q, om);
        w_ += 0.5 * dq.w * dt;
        x_ += 0.5 * dq.x * dt;
        y_ += 0.5 * dq.y * dt;
        z_ += 0.5 * dq.z * dt;
        const double n = std::sqrt(w_ * w_ + x_ * x_ + y_ * y_ + z_ * z_);
        w_ /= n; x_ /= n; y_ /= n; z_ /= n;
    }

    [[nodiscard]] bool aligned() const { return aligned_; }
    [[nodiscard]] double w() const { return w_; }
    [[nodiscard]] double x() const { return x_; }
    [[nodiscard]] double y() const { return y_; }
    [[nodiscard]] double z() const { return z_; }
    /// The default pose — THE unique canonical attitude (z out of the lens
    /// toward the viewer, y up, x left) that the display shows at startup
    /// and that the re-upright returns to. It is pinned once at alignment
    /// (a 6-axis IMU cannot sense compass heading, so the world's yaw
    /// reference is fixed there) and never changes afterward.
    [[nodiscard]] double default_w() const { return default_w_; }
    [[nodiscard]] double default_x() const { return default_x_; }
    [[nodiscard]] double default_y() const { return default_y_; }
    [[nodiscard]] double default_z() const { return default_z_; }
    /// The current gyro-bias estimate (deg/s) — diagnostics; converges from
    /// 0 while the chip is still.
    [[nodiscard]] double bias_x_dps() const { return bias_x_; }
    [[nodiscard]] double bias_y_dps() const { return bias_y_; }
    [[nodiscard]] double bias_z_dps() const { return bias_z_; }

private:
    struct Q4 {
        double w, x, y, z;
    };
    static constexpr double kDeg2Rad = M_PI / 180.0;
    /// STILLNESS gate shared by the tilt anchor, the yaw pull and the bias
    /// leak: below this rotation rate the chip counts as stationary.
    /// Deliberate slow rotations (2-7 deg/s) sit ABOVE it — they display
    /// faithfully as pure gyro integration and no accel-driven term may
    /// touch the attitude during them (the recording rec_20260920_221204
    /// showed the old 10 deg/s gate letting the tilt anchor wrench the
    /// attitude at ~23 deg/s during a slow pan — specific force is not
    /// gravity while rotating). Handheld rest reads |bias| + tremor
    /// ~1.5-2 deg/s, comfortably below. 静止 = 正: stillness is the one
    /// and only re-upright trigger.
    static constexpr double kRestGyroDps = 3.0;
    /// Time constant of the smoothed gyro-rate envelope the stillness gate
    /// is judged on (tremor spikes average out; real rotation rises through
    /// the gate within ~a quarter second).
    static constexpr double kGyroEnvelopeTauS = 0.25;
    /// ...and the quiet period must last this long before the accel is
    /// trusted again: motion direction reversals dip below the rate gate
    /// for ~10-30 ms only — far shorter than the window. The large-deviation
    /// snap is prevented by the correction-rate CAP below, not by this
    /// window, so it is kept short on purpose (the re-upright must start
    /// promptly after the camera stops).
    static constexpr double kAnchorSettleS = 0.3;
    /// Cap on the anchor correction rate (rad/s): large deviations re-
    /// upright at this fixed slew. 0.698 rad/s = 40 deg/s (doubled again
    /// per user request; a quarter turn returns in ~1.1 s) and the final
    /// snap lands the attitude exactly on the default pose.
    static constexpr double kMaxAnchorRateRadS = 0.698;
    /// Proportional gain (1/s) of the yaw pull toward the default pose's
    /// heading — the time constant of the exponential tail once the error
    /// is below the rate cap.
    static constexpr double kYawPullGain = 2.0;
    /// At rest, once the attitude is within this angle of the default pose
    /// the re-upright SNAPS onto it: the residual is zero by fiat instead
    /// of an exponential tail.
    static constexpr double kReUprightSnapRad = 0.0175;  // ~1 deg
    /// Bias tracking time constant (s) while a sustained moderate rate
    /// holds.
    static constexpr double kBiasTauS = 1.0;
    /// A rate at or below this (smoothed) magnitude, held SUSTAINED, is
    /// bias by definition; faster is real motion and must integrate.
    static constexpr double kAbsorbRateDps = 15.0;
    /// Sustained duration before the rate counts as bias (the user's
    /// rule: 1-2 s of constant angular velocity is bias).
    static constexpr double kAbsorbHoldS = 1.5;
    /// The leak runs only while the instantaneous rate agrees with the
    /// envelope to within this — a real twist jumps the instantaneous rate
    /// far above the envelope on its first sample, freezing the bias at
    /// once (no motion-onset kick).
    static constexpr double kAbsorbStableDps = 4.0;

    static Q4 qmul(const Q4& a, const Q4& b) {
        return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
                a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
    }

    double w_{1}, x_{0}, y_{0}, z_{0};
    double default_w_{1}, default_x_{0}, default_y_{0}, default_z_{0};
    double bias_x_{0}, bias_y_{0}, bias_z_{0};
    bool aligned_{false};
    std::int64_t last_t_{-1};
    std::int64_t aligned_t_us_{-1};
    double rest_run_s_{0};
    double sustain_s_{0};
    double gy_slow_x_{0};
    double gy_slow_y_{0};
    double gy_slow_z_{0};
};

} // namespace gui::davis

#endif // GUI_DAVIS_IMU_POSE_H
