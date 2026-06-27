#include "odometry.hpp"
#include <cmath>

namespace ms {

Odometry::Odometry(const OdometryParams& params) : p_(params) {
    reset(42);
}

void Odometry::reset(int seed) {
    rng_.seed(seed);
    scale_L_ = 1.0 + normal_(rng_) * p_.scale_error;
    scale_R_ = 1.0 + normal_(rng_) * p_.scale_error;
}

OdometryDelta Odometry::compute(double angle_L, double angle_R,
                                 double normal_L, double normal_R) {
    OdometryDelta delta{};

    auto compute_one = [&](double angle, double normal, double scale) -> double {
        // Effective radius under load (foam compression)
        double r_eff = p_.wheel_radius;
        if (normal > p_.min_contact_N) {
            r_eff *= (1.0 - p_.deformation_k * normal);
        }
        r_eff *= scale;

        double dist = r_eff * angle;
        // Slip noise proportional to distance
        dist += normal_(rng_) * p_.slip_noise * std::abs(dist);
        return dist;
    };

    delta.dist_L = compute_one(angle_L, normal_L, scale_L_);
    delta.dist_R = compute_one(angle_R, normal_R, scale_R_);
    return delta;
}

} // namespace ms
