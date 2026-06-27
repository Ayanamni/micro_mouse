/// vw_controller.cpp — v-ω 解耦控制实现
///
/// ω 环（5kHz，前馈主导 >80% + 滤波 PID）：
///   τ_ff = Jz * ω̇_ref + Bω * ω_ref
///   e = ω_ref - ω_meas
///   ė_raw = (e - e_prev) / dt
///   ė_filt = α * ė_raw + (1-α) * ė_prev   (fc=50Hz 一阶低通)
///   τ_fb = Kp*e + Ki*∫e + Kd*ė_filt
///   τ_ω = clamp(τ_ff + τ_fb, ±w_max)
///
/// v 环（1kHz，前馈 + 增量式 PI + 抗饱和）：
///   τ_ff = K_v_acc * v̇_ref + K_v_vel * v_ref + C_friction
///   e = v_ref - v_meas
///   Δτ = Kp*(e_k - e_{k-1}) + Ki*e_k     ← 增量式，天然抗饱和
///   τ_fb = τ_fb_prev + Δτ
///   抗饱和门控：若 |τ_v| 达上限且 sign(e) == sign(τ_v)，冻结 Ki 项
///   τ_v = clamp(τ_ff + τ_fb, ±v_max)
///
/// 巡线控制器（lateral_error → ω_ref）：
///   ω_ff = v_fwd * curvature * Kff     ← 纯几何前馈
///   ω_fb = Kp * lat_err + Kd * d(lat_err)/dt
///   ω_ref = ω_ff + ω_fb
///
/// 运动学混控：
///   τ_L = τ_v - τ_ω,  τ_R = τ_v + τ_ω
///   u_L = clamp(τ_L / τ_max, -1, 1)
///   u_R = clamp(τ_R / τ_max, -1, 1)

#include "vw_controller.hpp"
#include <cmath>
#include <algorithm>

namespace ms {

// ═══════════════════════════════════════════════════════════════
// OmegaLoop
// ═══════════════════════════════════════════════════════════════

float OmegaLoop::step(float gyro_z_raw, float dt) {
    if (dt <= 0.0f) return tau_omega_;

    // ── 直接使用原始陀螺仪（无滤波，零延迟）──
    // Butterworth 的群延迟 ~2ms 在 5kHz 回路中引起过大相位滞后
    // 高频噪声由 ω_loop Kd 抑制，不需要硬件低通
    omega_meas_ = gyro_z_raw;

    // ── 前馈：τ_ff = (Jz * r/B) * ω̇_ref + (Dw * r/B) * ω_ref ──
    float K_alpha = Jz_ * r_ / B_;   // Nm/(rad/s²)
    float K_omega = Dw_ * r_ / B_;   // Nm/(rad/s)
    float omega_dot_raw = (omega_ref_ - omega_ref_prev_) / dt;
    static constexpr float OMEGA_DOT_MAX = 500.0f;  // rad/s²
    // 阶跃检测：若 ω̇_raw 超过合理范围（>1500 rad/s²），说明是跳变而非连续轨迹
    // 直接同步 omega_ref_prev_ 避免前馈冲击
    if (std::abs(omega_dot_raw) > 1500.0f) {
        omega_ref_prev_ = omega_ref_;
        omega_dot_raw = 0.0f;
    }
    float omega_dot_ref = std::clamp(omega_dot_raw, -OMEGA_DOT_MAX, OMEGA_DOT_MAX);
    // 前馈 = (Jz·r/B)·ω̇_ref + (Dw·r/B)·ω_ref + Cw·sign(ω_ref)
    float Cw_sign = (omega_ref_ > 0.0f ? 1.0f : (omega_ref_ < 0.0f ? -1.0f : 0.0f));
    tau_ff_ = K_alpha * omega_dot_ref + K_omega * omega_ref_ + Cw_ * Cw_sign;
    omega_ref_prev_ += omega_dot_ref * dt;

    // ── 误差 ──
    omega_error_ = omega_ref_ - omega_meas_;

    // ── 滤波 D 项：一阶低通 ė，fc=50Hz ──
    float deriv_raw = (omega_error_ - error_prev_) / dt;
    deriv_filt_ = deriv_lpf_.step(deriv_raw);

    // ── 2DOF PI + 滤波 D ──
    // τ_fb = Kp*(β*ω_ref - ω_meas) + Ki*∫e + Kd*ė_filt
    integrator_ += omega_error_ * dt;
    float tau_fb_raw = Kp_ * (beta_ * omega_ref_ - omega_meas_)
                     + Ki_ * integrator_
                     + Kd_ * deriv_filt_;

    // ── 合成 + 限幅 ──
    float tau_omega_raw = tau_ff_ + tau_fb_raw;
    tau_omega_ = std::clamp(tau_omega_raw, -max_, max_);

    // ── 回算抗饱和 ──
    integrator_ += kaw_ * (tau_omega_ - tau_omega_raw) * dt;

    error_prev_ = omega_error_;
    return tau_omega_;
}

void OmegaLoop::set_params(const VWParams& p) {
    Jz_   = p.Jz;
    Dw_   = p.Dw;
    r_    = p.wheel_r;
    B_    = p.track_B;
    Kp_   = p.w_Kp;
    Ki_   = p.w_Ki;
    Kd_   = p.w_Kd;
    max_  = p.w_max;
    beta_ = p.beta_w;
    kaw_  = p.kaw_w;
    Cw_   = p.w_Cfrict;
    gyro_lpf_.design(p.gyro_lpf_fc, p.gyro_fs);
    deriv_lpf_.design(50.0f, 1.0f / 2.0e-4f); // fc=50Hz @ 5kHz
}

void OmegaLoop::reset() {
    gyro_lpf_.reset();
    deriv_lpf_.reset();
    omega_ref_      = 0.0f;
    omega_ref_prev_ = 0.0f;
    omega_meas_     = 0.0f;
    omega_error_    = 0.0f;
    error_prev_     = 0.0f;
    integrator_     = 0.0f;
    deriv_filt_     = 0.0f;
    tau_ff_         = 0.0f;
    tau_fb_         = 0.0f;
    tau_omega_      = 0.0f;
}

// ═══════════════════════════════════════════════════════════════
// VelocityLoop
// ═══════════════════════════════════════════════════════════════

float VelocityLoop::step(float v_fwd, float dt) {
    if (dt <= 0.0f) return tau_v_;

    // ── 前馈：τ_ff = (m_eq * r/2) * v̇_ref + (D_v * r/2) * v_ref + C_friction ──
    float K_v_acc = m_eq_ * r_ * 0.5f;   // kg·m = effective inertia * wheel radius / 2
    float K_v_vel = D_v_ * r_ * 0.5f;   // Nm/(m/s) = damping * radius / 2
    float v_dot_raw = (v_ref_ - v_ref_prev_) / dt;
    // 限制加速度参考，防止启动跳变导致 τ_ff 爆炸
    static constexpr float V_DOT_MAX = 5.0f;  // m/s²
    float v_dot_ref = std::clamp(v_dot_raw, -V_DOT_MAX, V_DOT_MAX);
    tau_ff_ = K_v_acc * v_dot_ref + K_v_vel * v_ref_ + C_frict_;
    // v_ref_prev 追踪 clamped 轨迹，而非原始跳变
    v_ref_prev_ += v_dot_ref * dt;

    // ── 2DOF 误差（P 项用加权设定值，I 项用原始误差）──
    v_error_ = v_ref_ - v_fwd;                 // full error for integral
    float error_fb = beta_ * v_ref_ - v_fwd;    // weighted error for proportional

    // ── 增量式 PI：Δτ = Kp*(e_fb_k - e_fb_{k-1}) + Ki*e_int_k ──
    float delta_tau = Kp_ * (error_fb - v_error_fb_prev_) + Ki_ * v_error_ * dt;
    float tau_fb_raw = tau_fb_ + delta_tau;

    // ── 抗饱和（Anti-Windup）门控 ──
    float tau_v_raw = tau_ff_ + tau_fb_raw;
    bool saturated = (tau_v_raw >= max_ || tau_v_raw <= -max_);
    bool pushing_into_sat = (v_ref_ - v_fwd > 0.0f && tau_v_raw > 0.0f)
                         || (v_ref_ - v_fwd < 0.0f && tau_v_raw < 0.0f);
    if (saturated && pushing_into_sat) {
        // 冻结积分增量：移除 Ki 贡献
        tau_fb_raw = tau_fb_ + Kp_ * (error_fb - v_error_fb_prev_);
        tau_v_raw = tau_ff_ + tau_fb_raw;
    }
    tau_fb_ = tau_fb_raw;

    // ── 合成 + 限幅 + 回算抗饱和 ──
    float tv_limited = std::clamp(tau_v_raw, -max_, max_);
    // Back-calculation anti-windup on integrator (tau_fb_ IS the accumulator)
    tau_fb_ += kaw_ * (tv_limited - tau_v_raw) * dt;
    tau_v_ = std::clamp(tau_ff_ + tau_fb_, -max_, max_);

    // ── Store 2DOF feedback error for next step ──
    v_error_fb_prev_ = error_fb;
    return tau_v_;
}

void VelocityLoop::set_params(const VWParams& p) {
    // Physical feedforward coefficients
    m_eq_    = p.m_eq;
    D_v_     = p.D_v;
    r_       = p.wheel_r;
    C_frict_ = p.C_frict;
    Kp_      = p.v_Kp;
    Ki_      = p.v_Ki;
    max_     = p.v_max;
    beta_    = p.beta_v;
    kaw_     = p.kaw_v;
}

void VelocityLoop::reset() {
    v_ref_      = 0.0f;
    v_ref_prev_ = 0.0f;
    v_error_    = 0.0f;
    v_error_fb_prev_ = 0.0f;
    integrator_ = 0.0f;
    tau_ff_     = 0.0f;
    tau_fb_     = 0.0f;
    tau_v_      = 0.0f;
}

// ═══════════════════════════════════════════════════════════════
// TrackingController
// ═══════════════════════════════════════════════════════════════

float TrackingController::compute(float lateral_error, float dt) {
    if (dt <= 0.0f) return 0.0f;

    // ── 高带宽低通滤波（fc=200Hz，用户指定）──
    lat_raw_ = lateral_error;
    lat_filt_ = lat_lpf_.step(lateral_error);

    // ── 纯光电管反馈：横向误差 → 角速度参考 ──
    // error = desired(0) - actual = -lateral_error
    // lateral_error > 0（车偏左）→ error < 0 → ω_ref < 0（右转修正）✓
    float tracking_err = -lat_filt_;
    lat_rate_ = (tracking_err - lat_prev_) / dt;
    lat_integ_ += tracking_err * dt;

    // 积分限幅（抗饱和，较小的钳位值防止过度累积）
    static constexpr float INTEG_MAX = 5.0f;  // rad
    lat_integ_ = std::clamp(lat_integ_, -INTEG_MAX, INTEG_MAX);

    float omega_ref = Kp_ * tracking_err + Ki_ * lat_integ_ + Kd_ * lat_rate_;

    lat_prev_ = tracking_err;

    // ── 限幅（物理可达偏航率）──
    static constexpr float OMEGA_REF_MAX = 20.0f;  // rad/s
    float omega_clamped = std::clamp(omega_ref, -OMEGA_REF_MAX, OMEGA_REF_MAX);
    omega_fb_ = omega_clamped;
    return omega_clamped;
}

void TrackingController::set_params(const VWParams& p) {
    Kp_  = p.lat_Kp;
    Ki_  = p.lat_Ki;
    Kd_  = p.lat_Kd;
}

void TrackingController::reset() {
    lat_lpf_.reset();
    lat_raw_  = 0.0f;
    lat_filt_ = 0.0f;
    lat_prev_ = 0.0f;
    lat_rate_ = 0.0f;
    lat_integ_ = 0.0f;
    omega_fb_ = 0.0f;
}

// ═══════════════════════════════════════════════════════════════
// VWController — 编排器
// ═══════════════════════════════════════════════════════════════

void VWController::set_params(const VWParams& p) {
    params_ = p;
    omega_loop_.set_params(p);
    vel_loop_.set_params(p);
    track_ctrl_.set_params(p);
}

void VWController::omega_step(float gyro_z_raw, float dt_imu) {
    omega_loop_.step(gyro_z_raw, dt_imu);
}

VWControlResult VWController::control_tick(
        float lateral_error, float /*curvature*/,
        float v_fwd, float v_ref, float dt_ctrl)
{
    // ── 计算 ω_ref（纯光电管反馈，无曲率前馈）──
    if (std::isnan(omega_override_)) {
        omega_ref_used_ = track_ctrl_.compute(lateral_error, dt_ctrl);
    } else {
        omega_ref_used_ = omega_override_;
    }
    omega_loop_.set_omega_ref(omega_ref_used_);
    vel_loop_.set_v_ref(v_ref);

    // ── 速度环 → τ_v ──
    float tv_raw = vel_loop_.step(v_fwd, dt_ctrl);

    // ── 取最新的 τ_ω（由 omega_step 持续更新）──
    float tw_raw = omega_loop_.tau_omega();

    // ── 动态力矩上限（电压/反电势/电流约束）──
    // U = R/(Kt*G*eta)*τ + Ke*G*ω, |U|<=Vbus, |I|<=I_peak
    struct WheelLimit {
        float neg;
        float pos;
    };
    auto wheel_tau_limit = [&](float w) -> WheelLimit {
        float v_bemf = params_.motor_Ke * params_.motor_G * w;
        float i_pos = (params_.motor_V_bus - v_bemf) / params_.motor_R;
        float i_neg = (-params_.motor_V_bus - v_bemf) / params_.motor_R;
        i_pos = std::clamp(i_pos, 0.0f, params_.motor_I_peak);
        i_neg = std::clamp(i_neg, -params_.motor_I_peak, 0.0f);
        float k = params_.motor_Kt * params_.motor_G * params_.motor_eta;
        return WheelLimit{i_neg * k, i_pos * k};
    };
    WheelLimit lim_L = wheel_tau_limit(omega_wheel_L_);
    WheelLimit lim_R = wheel_tau_limit(omega_wheel_R_);

    // ── Yaw-priority allocation ──
    // 约束：τ_L = τ_v - τ_ω ∈ [Lneg,Lpos], τ_R = τ_v + τ_ω ∈ [Rneg,Rpos]
    // 先保 yaw：只要存在某个 τ_v 能满足左右轮约束，τ_ω 就是可行的。
    float tw_min = 0.5f * (lim_R.neg - lim_L.pos);
    float tw_max = 0.5f * (lim_R.pos - lim_L.neg);
    float tw_lim = std::clamp(tw_raw, tw_min, tw_max);

    // 在固定 τ_ω 后，τ_v 的可行交集。
    float tv_min = std::max(lim_L.neg + tw_lim, lim_R.neg - tw_lim);
    float tv_max = std::min(lim_L.pos + tw_lim, lim_R.pos - tw_lim);
    float tv_lim = std::clamp(tv_raw, tv_min, tv_max);

    // ── 运动学混控（使用限幅后的力矩）──
    float tau_L = tv_lim - tw_lim;
    float tau_R = tv_lim + tw_lim;

    // ── 逆电机电压前馈：τ → duty ──
    // U = (R / (Kt * G * eta)) * tau_wheel + Ke * G * omega_wheel
    // u = clamp(U / V_bus, -1, 1)
    float K_tau2V = params_.motor_R
                  / (params_.motor_Kt * params_.motor_G * params_.motor_eta);
    float K_bemf  = params_.motor_Ke * params_.motor_G;

    float U_L = K_tau2V * tau_L + K_bemf * omega_wheel_L_;
    float U_R = K_tau2V * tau_R + K_bemf * omega_wheel_R_;

    float u_L = std::clamp(U_L / params_.motor_V_bus, -1.0f, 1.0f);
    float u_R = std::clamp(U_R / params_.motor_V_bus, -1.0f, 1.0f);

    VWControlResult r;
    r.tau_v     = tv_lim;
    r.tau_omega = tw_lim;
    r.tau_v_raw = tv_raw;
    r.tau_omega_raw = tw_raw;
    r.tau_L_cmd = tau_L;
    r.tau_R_cmd = tau_R;
    r.tau_limit_L_pos = lim_L.pos;
    r.tau_limit_L_neg = lim_L.neg;
    r.tau_limit_R_pos = lim_R.pos;
    r.tau_limit_R_neg = lim_R.neg;
    r.sat_v = std::abs(tv_lim - tv_raw) > 1.0e-6f;
    r.sat_w = std::abs(tw_lim - tw_raw) > 1.0e-6f;
    r.u_L       = u_L;
    r.u_R       = u_R;
    r.throttle  = (u_L + u_R) * 0.5f;  // common-mode duty
    r.steer     = (u_R - u_L) * 0.5f;  // differential duty
    last_result_ = r;
    return r;
}

void VWController::reset() {
    omega_loop_.reset();
    vel_loop_.reset();
    track_ctrl_.reset();
    omega_ref_used_ = 0.0f;
    omega_ref_set_  = false;
    last_result_ = VWControlResult{};
}

} // namespace ms
