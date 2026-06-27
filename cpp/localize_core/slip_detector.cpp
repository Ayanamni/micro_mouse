#include "slip_detector.hpp"
#include "shared/math_utils.hpp"
#include <cmath>
#include <algorithm>

namespace ms {

SlipDetector::SlipDetector(const SlipDetectorParams& params)
    : params_(params) {}

double SlipDetector::compute(double v_enc, double v_imu,
                              double a_y_avg, double v_fwd, double w_z) {
    // ---- Longitudinal: velocity-layer comparison ----
    // v_enc = dist_enc / dt_enc  (encoder-measured speed)
    // v_imu = dist_accum / dt_enc (IMU-predicted speed from a_x integration)
    // This is a first-derivative comparison — no encoder position differentiation!
    last_lon_res_ = std::abs(v_enc - v_imu);

    // ---- Lateral: acceleration-layer comparison ----
    // Expected lateral acceleration: a_y = v_fwd * w_z (centripetal)
    double a_y_expected = v_fwd * w_z;
    last_lat_res_ = std::abs(a_y_avg - a_y_expected);

    // ---- Smooth deadzone mapping ----
    double lon_scale = map_residual(last_lon_res_, params_.thresh_lon, params_.k_slip);
    double lat_scale = map_residual(last_lat_res_, params_.thresh_lat, params_.k_slip);

    double scale = std::max(lon_scale, lat_scale);
    return math::clamp(scale, 1.0, params_.max_scale);
}

double SlipDetector::map_residual(double residual, double threshold, double k) {
    // Deadzone: residual <= threshold → scale = 1.0
    // Above threshold: scale = 1.0 + k * (residual - threshold)
    double excess = std::max(0.0, residual - threshold);
    return 1.0 + k * excess;
}

} // namespace ms
