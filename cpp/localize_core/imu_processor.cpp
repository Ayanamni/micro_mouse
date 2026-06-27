#include "imu_processor.hpp"
#include <cmath>

namespace ms {

ImuProcessor::ImuProcessor() {
    reset(42);
}

void ImuProcessor::reset(int seed) {
    rng_.seed(seed);
    bias_ = ImuBiasState{};

    // Per-axis scale errors (fixed for a run)
    for (int i = 0; i < 3; ++i) {
        gyro_scale_[i]  = 1.0 + gyro_scale_error * (normal_(rng_) * 0.33);
        accel_scale_[i] = 1.0 + accel_scale_error * (normal_(rng_) * 0.33);
    }

    // Misalignment: identity + small off-diagonal terms
    for (int i = 0; i < 9; ++i) {
        int row = i / 3, col = i % 3;
        gyro_misalign_[i]  = (row == col) ? 1.0 : gyro_cross_axis * (normal_(rng_) * 0.33);
        accel_misalign_[i] = (row == col) ? 1.0 : accel_cross_axis * (normal_(rng_) * 0.33);
    }
}

ImuData ImuProcessor::process(const double* gyro_raw, const double* accel_raw, double dt) {
    ImuData out;
    out.dt = dt;

    // Process each axis
    for (int axis = 0; axis < 3; ++axis) {
        // --- Gyro ---
        double g = gyro_raw[axis];
        // Scale + cross-axis
        double g_scaled = 0.0;
        for (int j = 0; j < 3; ++j)
            g_scaled += gyro_misalign_[axis * 3 + j] * gyro_raw[j] * gyro_scale_[j];
        // Bias drift (Gauss-Markov)
        update_bias(bias_.gyro_bias[axis], gyro_bias_sigma, gyro_bias_tau, dt);
        g_scaled += bias_.gyro_bias[axis];
        // White noise (power scales with 1/√dt for discrete samples)
        double noise_std = gyro_noise_density * std::sqrt(0.5 / dt);
        g_scaled += normal_(rng_) * noise_std;
        // Quantize
        out.gyro[axis] = quantize(g_scaled, gyro_lsb);

        // --- Accel ---
        double a = accel_raw[axis];
        double a_scaled = 0.0;
        for (int j = 0; j < 3; ++j)
            a_scaled += accel_misalign_[axis * 3 + j] * accel_raw[j] * accel_scale_[j];
        update_bias(bias_.accel_bias[axis], accel_bias_sigma, accel_bias_tau, dt);
        a_scaled += bias_.accel_bias[axis];
        double anoise_std = accel_noise_density * std::sqrt(0.5 / dt);
        a_scaled += normal_(rng_) * anoise_std;
        out.accel[axis] = quantize(a_scaled, accel_lsb);
    }

    return out;
}

double ImuProcessor::quantize(double value, double lsb) const {
    return std::round(value / lsb) * lsb;
}

double ImuProcessor::update_bias(double& bias, double sigma, double tau, double dt) {
    // Gauss-Markov: db/dt = -bias/tau + w, w ~ N(0, 2*sigma²/tau)
    double beta = dt / tau;
    double w_std = sigma * std::sqrt(2.0 * beta);
    bias = bias * (1.0 - beta) + normal_(rng_) * w_std;
    return bias;
}

} // namespace ms
