#pragma once
/// Odometry: encoder integration + tire deformation error injection.
/// Converts raw wheel joint data into distance traveled, accounting for
/// silicone-foam tire deformation effects on effective rolling radius.

#include <array>
#include <random>
#include "shared/types.hpp"

namespace ms {

struct OdometryParams {
    double wheel_radius   = 0.0105;    // m, nominal
    double track_width    = 0.050;     // m, lateral wheel separation
    double deformation_k  = 2.0e-5;    // m/N, radius reduction per newton
    double scale_error    = 0.002;     // 1-sigma wheel radius uncertainty
    double slip_noise     = 0.0015;    // 1-sigma per-step slip noise fraction
    double min_contact_N  = 0.01;      // N, minimum normal force for contact
};

class Odometry {
public:
    explicit Odometry(const OdometryParams& params = {});
    void reset(int seed = 42);

    /// Convert wheel angle deltas and normal forces into distance traveled.
    /// @param angle_L, angle_R   Wheel angle this step (rad)
    /// @param normal_L, normal_R Approximate normal force per wheel (N)
    OdometryDelta compute(double angle_L, double angle_R,
                          double normal_L, double normal_R);

private:
    OdometryParams p_;
    double scale_L_, scale_R_;  // wheel-specific scale factors
    std::mt19937 rng_;
    std::normal_distribution<double> normal_{0.0, 1.0};
};

} // namespace ms
