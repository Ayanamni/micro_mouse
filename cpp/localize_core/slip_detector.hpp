#pragma once
/// Slip detector: compares IMU-predicted motion with encoder measurements
/// to detect wheel slip and adaptively scale measurement noise R.
///
/// Longitudinal:  |v_enc - v_imu|           @ velocity layer (avoids 2nd derivative!)
/// Lateral:       |a_y_meas - v_fwd * w_z|  @ acceleration layer
///
/// Smooth deadzone mapping → slip_scale ∈ [1.0, 20.0]

namespace ms {

struct SlipDetectorParams {
    double thresh_lon = 0.05;   // m/s, longitudinal velocity residual deadzone
    double thresh_lat = 1.0;    // m/s^2, lateral acceleration residual deadzone
    double k_slip     = 5.0;    // gain: slip_scale = 1 + k * excess
    double max_scale  = 20.0;   // upper bound on slip_scale
};

class SlipDetector {
public:
    explicit SlipDetector(const SlipDetectorParams& params = {});

    /// Compute slip scaling factor.
    /// @param v_enc        Encoder-measured forward velocity (dist_enc / dt_enc)
    /// @param v_imu        IMU-predicted forward velocity (dist_accum / dt_enc)
    /// @param a_y_avg      Average lateral acceleration over encoder interval
    /// @param v_fwd        Current forward velocity estimate
    /// @param w_z          Current yaw rate (gyro_z - bias)
    /// @returns            slip_scale ∈ [1.0, max_scale], 1.0 = no slip
    double compute(double v_enc, double v_imu,
                   double a_y_avg, double v_fwd, double w_z);

    /// Get last computed values (debug)
    double last_lon_residual() const { return last_lon_res_; }
    double last_lat_residual() const { return last_lat_res_; }

    SlipDetectorParams& params() { return params_; }

private:
    SlipDetectorParams params_;
    double last_lon_res_ = 0.0;
    double last_lat_res_ = 0.0;

    static double map_residual(double residual, double threshold, double k);
};

} // namespace ms
