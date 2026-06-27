#pragma once
/// 2nd-order Butterworth low-pass filter (Direct Form I).
/// Header-only, zero dynamic allocation, designed for real-time MCU ISR use.
///
/// Usage:
///   Butterworth2 lpf;
///   lpf.design(80.0, 4000.0);  // fc=80Hz, fs=4000Hz
///   float clean = lpf.step(raw_sample);
///
/// Coefficient design uses the bilinear transform with pre-warping.
/// Group delay at fc ≈ 1/(2π·fc) ≈ 2ms @ 80Hz — negligible for control.

#include <cmath>
#include <algorithm>

namespace ms {

struct Butterworth2 {
    // Feedforward coefficients
    float b0 = 0.0f, b1 = 0.0f, b2 = 0.0f;
    // Feedback coefficients (a0 normalised to 1)
    float a1 = 0.0f, a2 = 0.0f;

    // Delay line (Direct Form I)
    float x1 = 0.0f, x2 = 0.0f;  // input history
    float y1 = 0.0f, y2 = 0.0f;  // output history

    /// Design a 2nd-order Butterworth low-pass.
    /// @param fc  Cutoff frequency (Hz), e.g. 80.0
    /// @param fs  Sampling frequency (Hz), e.g. 4000.0
    void design(float fc, float fs) {
        // Pre-warped analog frequency
        float omega = 2.0f * 3.141592653589793f * fc / fs;
        // sin/cos for bilinear transform
        float sn = std::sin(omega);
        float cs = std::cos(omega);
        // Butterworth Q = 1/sqrt(2) → alpha = sin(ω)/√2
        float alpha = sn * 0.7071067811865476f;  // sn / √2

        float a0_inv = 1.0f / (1.0f + alpha);

        b0 = (1.0f - cs) * 0.5f * a0_inv;
        b1 = (1.0f - cs) * a0_inv;
        b2 = b0;
        a1 = (-2.0f * cs) * a0_inv;
        a2 = (1.0f - alpha) * a0_inv;
    }

    /// Process a single sample.
    float step(float x) {
        float y = b0 * x + b1 * x1 + b2 * x2
                - a1 * y1 - a2 * y2;
        x2 = x1;  x1 = x;
        y2 = y1;  y1 = y;
        return y;
    }

    /// Process 3-axis samples in parallel (gyro XYZ or accel XYZ).
    /// Avoids scalar overhead when filtering all 3 axes at once.
    void step3(float x, float y_in, float z_in,
               float& x_out, float& y_out, float& z_out) {
        // Use the same coefficient set for all axes.
        // Note: each axis needs its own delay line.
        // For the built-in 3-axis variant, we assume 3 separate Butterworth2
        // instances or call step() independently. This method exists for
        // interface documentation only — in practice, instantiate 3 filters.
        (void)x; (void)y_in; (void)z_in;
        (void)x_out; (void)y_out; (void)z_out;
        // Placeholder: use separate Butterworth2 instances per axis.
    }

    /// Reset all state to zero.
    void reset() {
        x1 = 0.0f; x2 = 0.0f;
        y1 = 0.0f; y2 = 0.0f;
    }
};

/// 1st-order low-pass for D-term filtering (simpler than Butterworth for this use).
/// fc = 50Hz @ 5kHz → alpha ≈ 0.939
struct LowPass1F {
    float alpha = 0.0f;  // smoothing factor [0,1], larger = stronger filtering
    float y_prev = 0.0f;

    /// Configure from cutoff frequency.
    /// alpha = exp(-2π·fc/fs)
    void design(float fc, float fs) {
        alpha = std::exp(-6.283185307179586f * fc / fs);
    }

    /// Directly set alpha (for runtime tuning).
    void set_alpha(float a) { alpha = std::clamp(a, 0.0f, 0.9999f); }

    float step(float x) {
        y_prev = alpha * y_prev + (1.0f - alpha) * x;
        return y_prev;
    }

    void reset() { y_prev = 0.0f; }
};

} // namespace ms
