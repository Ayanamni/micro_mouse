#pragma once
/// IMU noise chain: ground truth → scale/cross-axis → bias drift → white noise → quantization
/// Based on ICM-42688-P parameters (slightly degraded).

#include <array>
#include <random>
#include "shared/types.hpp"

namespace ms {

struct ImuBiasState {
    std::array<double, 3> gyro_bias{0, 0, 0};   // rad/s
    std::array<double, 3> accel_bias{0, 0, 0};  // m/s²
};

class ImuProcessor {
public:
    ImuProcessor();

    /// Inject noise into raw IMU data, update internal bias state.
    /// @param gyro_raw  Ground truth gyro [3] in body frame (rad/s)
    /// @param accel_raw Ground truth accel [3] in body frame (m/s²)
    /// @param dt        Time since last call (s) — should match 4kHz ODR
    /// @returns         Noisy IMU data
    ImuData process(const double* gyro_raw, const double* accel_raw, double dt);

    /// Reset bias and random state
    void reset(int seed = 42);

    // Accessors for debugging
    const ImuBiasState& bias_state() const { return bias_; }

private:
    // ---- Noise parameters (ICM-42688-P, slightly degraded) ----
    // Gyro
    static constexpr double gyro_noise_density  = 2.0e-5;   // rad/s/√Hz
    static constexpr double gyro_bias_sigma     = 3.0e-5;   // rad/s, steady-state bias instability
    static constexpr double gyro_bias_tau       = 100.0;    // s, Gauss-Markov time constant
    static constexpr double gyro_scale_error    = 0.01;     // 1% scale factor error
    static constexpr double gyro_cross_axis     = 0.02;     // 2% cross-axis sensitivity
    static constexpr double gyro_range_dps      = 2000.0;
    static constexpr double gyro_lsb = (gyro_range_dps * 3.141592653589793 / 180.0) / 32768.0;

    // Accel
    static constexpr double accel_noise_density = 1.0e-3;   // m/s²/√Hz
    static constexpr double accel_bias_sigma    = 1.5e-4;   // m/s²
    static constexpr double accel_bias_tau      = 100.0;    // s
    static constexpr double accel_scale_error   = 0.01;
    static constexpr double accel_cross_axis    = 0.02;
    static constexpr double accel_range_g       = 16.0;
    static constexpr double accel_lsb = (accel_range_g * 9.81) / 32768.0;

    // ---- State ----
    ImuBiasState bias_;
    std::mt19937 rng_;
    std::normal_distribution<double> normal_{0.0, 1.0};

    double gyro_scale_[3];   // per-axis scale (1 + error)
    double gyro_misalign_[9]; // 3x3 misalignment matrix
    double accel_scale_[3];
    double accel_misalign_[9];

    double quantize(double value, double lsb) const;
    double update_bias(double& bias, double sigma, double tau, double dt);
};

} // namespace ms
