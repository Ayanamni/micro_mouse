#pragma once
/// ESKF (Error-State Kalman Filter) skeleton.
/// Current implementation: dead-reckoning integration only.
/// Full ESKF predict/update to be filled in Phase 3+.

#include <array>
#include "shared/types.hpp"

namespace ms {

class Eskf {
public:
    Eskf();

    /// Integrate IMU + odometry to produce pose estimate.
    /// Currently dead-reckoning only (no filter correction).
    /// @param imu    Noisy IMU data (body frame)
    /// @param odom   Wheel odometry delta this step
    /// @returns      Updated pose estimate with covariance
    PoseEstimate predict(const ImuData& imu, const OdometryDelta& odom);

    /// Reset state to origin
    void reset();

    /// Current pose estimate
    const PoseEstimate& pose() const { return pose_; }

private:
    PoseEstimate pose_;
};

} // namespace ms
