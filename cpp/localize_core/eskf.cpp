#include "eskf.hpp"
#include "shared/math_utils.hpp"
#include <cmath>

namespace ms {

Eskf::Eskf() { reset(); }

void Eskf::reset() {
    pose_ = PoseEstimate{};
    pose_.cov = {0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01}; // ~0.1m std, 0.1rad std initially
}

PoseEstimate Eskf::predict(const ImuData& imu, const OdometryDelta& odom) {
    double dt = imu.dt;
    if (dt <= 0) return pose_;

    // ---- Dead reckoning from odometry ----
    // Average distance
    double dist_avg = (odom.dist_L + odom.dist_R) * 0.5;
    // Heading change from differential
    double d_yaw = (odom.dist_R - odom.dist_L) / 0.050; // track_width = 0.050m

    // ---- Gyro integration for yaw (blend with odometry yaw) ----
    double gyro_yaw_rate = imu.gyro[2]; // z-axis in body = yaw rate in world (small angles)
    double d_yaw_gyro = gyro_yaw_rate * dt;

    // Blend: trust odometry for low-frequency, gyro for high-frequency
    // Simple complementary filter: 80% gyro, 20% odometry for yaw
    double d_yaw_fused = d_yaw_gyro * 0.8 + d_yaw * 0.2;

    // Update pose
    double cos_yaw = std::cos(pose_.yaw);
    double sin_yaw = std::sin(pose_.yaw);

    pose_.x += dist_avg * cos_yaw;
    pose_.y += dist_avg * sin_yaw;
    pose_.yaw = math::wrap_angle(pose_.yaw + d_yaw_fused);

    // Velocity estimates (simple backward difference)
    pose_.v_fwd = dist_avg / dt;
    pose_.v_lat = 0.0; // can't observe lateral velocity from odometry alone
    // Could add lateral from IMU integration here

    // Covariance propagation (simplified — full ESKF will fill this)
    // Grow uncertainty proportional to distance traveled
    double dist_noise = 0.001 * std::abs(dist_avg); // 0.1% per distance
    double yaw_noise = 0.0001 * std::abs(d_yaw_fused);
    pose_.cov[0] += dist_noise * dist_noise; // xx
    pose_.cov[4] += dist_noise * dist_noise; // yy
    pose_.cov[8] += yaw_noise * yaw_noise;   // yawyaw

    return pose_;
}

} // namespace ms
