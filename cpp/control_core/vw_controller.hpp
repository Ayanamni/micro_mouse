#pragma once
/// v-ω（线速度-角速度）解耦控制器
///
/// 核心理念：完全摒弃左右轮独立 PID。
/// 将电子鼠视为刚性整体，控制量收敛为整体线速度 v 和整体角速度 ω。
/// 左右轮仅通过运动学混控（τ_L = τ_v - τ_ω, τ_R = τ_v + τ_ω）作为最终执行器。
///
/// 两层架构：
///   上层（1kHz）：TrackingController — lateral_error + curvature → ω_ref
///                 SpeedPlanner — curvature → v_ref（外部提供）
///   下层（5kHz）：OmegaLoop — gyro_z + ω_ref → τ_ω（前馈 >80% + 滤波 PID）
///         （1kHz）：VelocityLoop — v_fwd + v_ref → τ_v（前馈 + 增量 PI + 抗饱和）
///
/// 接口设计：
///   - omega_step(gyro_z, dt)         → 5kHz 角速度环（更新 τ_ω 内部状态）
///   - control_tick(lat_err, curv,    → 1kHz 巡线+速度环+混控
///                   v_fwd, v_ref, dt)  → {u_L, u_R, throttle, steer, tau_v, tau_ω}
///
/// 拓展点：
///   - omega_ref 可通过 set_omega_ref() 外部覆盖（供未来纯追踪等上层控制器）
///   - v_ref 直接由 control_tick 参数传入
///   - 所有参数通过 VWParams 结构体统一管理，支持在线调参

#include <cstdint>
#include "shared/types.hpp"
#include "shared/math_utils.hpp"
#include "shared/butterworth2.hpp"

namespace ms {

// ═══════════════════════════════════════════════════════════════
// 参数结构体 — 统一调参入口
// ═══════════════════════════════════════════════════════════════

struct VWParams {
    // ── 车辆几何（物理常数）──
    float wheel_r = 0.0105f;  // m，轮半径
    float track_B = 0.090f;   // m，轮距

    // ── ω 环（角速度内环，5kHz）──
    // 前馈：τ_ω,ff = (Jz * r/B) * α_ref + (Dw * r/B) * ω_ref
    float Jz   = 7.9e-5f;   // kg·m²，Z轴转动惯量（base.xml: chassis Izz + wheel translation）
    float Dw   = 2.1e-3f;   // Nm/(rad/s)，偏航粘性阻尼系数
    float w_Kp = 0.05f;     // (Nm)/(rad/s)，比例增益
    float w_Ki = 0.03f;     // (Nm)/(rad·s)，积分增益
    float w_Kd = 0.0f;      // (Nm)/(rad/s²)，微分增益
    float w_d_alpha = 0.939f; // D 项一阶低通系数（fc=50Hz @ 5kHz）
    float w_max = 0.05f;    // Nm，ω 环力矩限幅
    // ── 2DOF + 抗饱和（默认 1.0=禁用，待 S 曲线规划后再调 β < 1）──
    float beta_w  = 1.0f;   // ω 环设定值权重
    float kaw_w   = 0.5f;   // ω 环抗饱和回算增益
    float beta_v  = 1.0f;   // v 环设定值权重
    float kaw_v   = 0.5f;   // v 环抗饱和回算增益

    // ── v 环（线速度外环，1kHz）──
    // 前馈：τ_v,ff = (m_eq * r/2) * a_ref + (D_v * r/2) * v_ref + C_friction
    float m_eq  = 0.11f;    // kg，等效平移质量（车身+轮转动惯量折算）
    float D_v   = 0.2f;     // N/(m/s)，平移粘性阻尼余项
    float C_frict = 0.0016f; // Nm，基础静摩擦补偿
    float v_Kp     = 1.0f;  // (Nm)/(m/s)，比例增益
    float v_Ki     = 0.3f;  // (Nm)/m，积分增益（增量式，天然抗饱和）
    float v_max    = 0.05f; // Nm，v 环力矩限幅

    // ── 巡线控制器（lateral_error → ω_ref，1kHz）──
    float lat_Kp   = 3000.0f; // (rad/s)/m，横向误差→角速度比例
    float lat_Ki   = 0.0f;    // (rad/s)/m²，横向积分（消除稳态漂移）
    float lat_Kd   = 0.0f;    // (rad/s)/(m/s)，横向变化率→角速度阻尼
    float lat_Kff  = 0.0f;    // 保留给未来曲率/横向前馈；当前纯光电管路径不用

    // ── 陀螺仪滤波 ──
    float gyro_lpf_fc = 80.0f; // Hz，Butterworth 截止频率（0=禁用滤波）
    float gyro_fs     = 5000.0f; // Hz，IMU 采样率

    // ── 偏航库仑摩擦前馈 ──
    float w_Cfrict = 0.006f;   // Nm，μ·downforce·skirt_R 理论约 0.009，保守补偿

    // ── 电机电气参数（电压前馈 + 动态力矩上限）──
    float motor_R    = 0.344f;   // Ohm，相间电阻
    float motor_Kt   = 0.00241f; // Nm/A，力矩常数
    float motor_Ke   = 0.00241f; // V/(rad/s)，反电势常数 (=Kt in SI)
    float motor_G    = 4.0f;     // 减速比
    float motor_eta  = 0.88f;    // 传动效率
    float motor_V_bus= 11.1f;    // V，母线电压（3S LiPo 标称；活动路径由 vw_set_params 覆盖）
    float motor_I_peak=10.0f;    // A，峰值电流
};

// ═══════════════════════════════════════════════════════════════
// 子控制器
// ═══════════════════════════════════════════════════════════════

/// 角速度 ω 环（5kHz 内环）
/// 前馈主导（>80%）+ 滤波 PID 反馈
class OmegaLoop {
public:
    OmegaLoop() { gyro_lpf_.design(80.0f, 5000.0f); }

    /// @param gyro_z_raw  陀螺仪 Z 轴原始值 (rad/s)
    /// @param dt          时间步长 (s)，典型 200us
    /// @returns τ_ω (Nm)，正=左转力矩
    float step(float gyro_z_raw, float dt);

    void set_params(const VWParams& p);
    void set_omega_ref(float w_ref) { omega_ref_ = w_ref; }
    void reset();

    // 调试访问
    float tau_omega()     const { return tau_omega_; }
    float omega_meas()    const { return omega_meas_; }
    float omega_ref()     const { return omega_ref_; }
    float omega_error()   const { return omega_error_; }
    float tau_ff()        const { return tau_ff_; }
    float tau_fb()        const { return tau_fb_; }
    float deriv_filt()    const { return deriv_filt_; }
    float integrator()    const { return integrator_; }

private:
    Butterworth2 gyro_lpf_;
    LowPass1F    deriv_lpf_;

    // 前馈参数（物理）
    float Jz_   = 7.9e-5f;
    float Dw_   = 2.1e-3f;   // Nm/(rad/s)，偏航阻尼
    float r_    = 0.0105f;    // m，轮半径
    float B_    = 0.090f;     // m，轮距

    // 反馈参数
    float Kp_    = 3.0e-3f;
    float Ki_    = 5.0e-4f;
    float Kd_    = 5.0e-5f;
    float max_   = 0.05f;
    float beta_  = 0.3f;   // 2DOF setpoint weight
    float kaw_   = 0.5f;   // anti-windup back-calculation gain
    float Cw_    = 0.009f; // Nm，偏航库仑摩擦前馈（μ·downforce·skirt_R）

    // 状态
    float omega_ref_     = 0.0f;
    float omega_ref_prev_ = 0.0f;
    float omega_meas_    = 0.0f;
    float omega_error_   = 0.0f;
    float error_prev_    = 0.0f;
    float integrator_    = 0.0f;
    float deriv_filt_    = 0.0f;
    float tau_ff_        = 0.0f;
    float tau_fb_        = 0.0f;
    float tau_omega_     = 0.0f;
};

/// 线速度 v 环（1kHz 外环）
/// 前馈 + 增量式 PI（天然抗积分饱和）+ 显式 Anti-Windup
class VelocityLoop {
public:
    /// @param v_fwd   定位融合前向速度 (m/s)
    /// @param dt      时间步长 (s)，典型 1ms
    /// @returns τ_v (Nm)
    float step(float v_fwd, float dt);

    void set_params(const VWParams& p);
    void set_v_ref(float v_ref) { v_ref_ = v_ref; }
    void reset();

    // 调试访问
    float tau_v()        const { return tau_v_; }
    float tau_ff()       const { return tau_ff_; }
    float tau_fb()       const { return tau_fb_; }
    float v_ref()        const { return v_ref_; }
    float v_error()      const { return v_error_; }
    float integrator()   const { return integrator_; }

private:
    // 前馈参数（物理）
    float m_eq_     = 0.11f;
    float D_v_      = 9.5f;
    float r_         = 0.0105f;
    float C_frict_  = 0.005f;

    // 反馈参数
    float Kp_   = 0.5f;
    float Ki_   = 0.15f;
    float max_  = 0.05f;
    float beta_ = 0.5f;   // 2DOF setpoint weight
    float kaw_  = 0.5f;   // anti-windup back-calculation gain

    // 状态
    float v_ref_      = 0.0f;
    float v_ref_prev_ = 0.0f;
    float v_error_    = 0.0f;
    float v_error_fb_prev_ = 0.0f; // 2DOF: β*ref - y from previous step
    float integrator_ = 0.0f;      // τ_fb accumulator (delta-PI)
    float tau_ff_     = 0.0f;
    float tau_fb_     = 0.0f;
    float tau_v_      = 0.0f;
};

/// 巡线控制器（lateral_error → ω_ref，1kHz）
/// 桥接"线传感器"和"v-ω 底控制"
/// 内置横向误差低通滤波，抗光电管噪声
/// 未来可替换为纯追踪（Pure Pursuit）等更高级的横向控制器
class TrackingController {
public:
    TrackingController() { lat_lpf_.design(200.0f, 1000.0f); } // fc=200Hz high-bandwidth

    /// @param lateral_error  原始横向误差 (m)，正=偏左
    /// @return ω_ref (rad/s)，正=左转
    float compute(float lateral_error, float dt);

    void set_params(const VWParams& p);
    void reset();

    // 调试
    float omega_fb()        const { return omega_fb_; }
    float lat_error_raw()   const { return lat_raw_; }
    float lat_error_filt()  const { return lat_filt_; }
    float lat_error_rate()  const { return lat_rate_; }

private:
    float Kp_  = 200.0f;
    float Ki_  = 20.0f;   // 横向积分 (rad/s)/m
    float Kd_  = 5.0f;

    LowPass1F lat_lpf_;     // 横向误差低通 (fc=200Hz)
    float lat_raw_   = 0.0f;
    float lat_filt_  = 0.0f;
    float lat_prev_  = 0.0f;  // tracking_err 上一拍
    float lat_rate_  = 0.0f;
    float lat_integ_ = 0.0f;  // ∫tracking_err dt
    float omega_fb_  = 0.0f;
};

// ═══════════════════════════════════════════════════════════════
// VWController — 顶层编排器
// ═══════════════════════════════════════════════════════════════

struct VWControlResult {
    float u_L;        // 左轮归一化指令 [-1, 1]
    float u_R;        // 右轮归一化指令 [-1, 1]
    float throttle;   // 共模推力 τ_v / τ_max
    float steer;      // 差速转向 τ_ω / τ_max
    float tau_v;      // Nm，实际分配后的总推进力矩
    float tau_omega;  // Nm，实际分配后的总转向力矩
    float tau_v_raw;      // Nm，v 环原始输出
    float tau_omega_raw;  // Nm，ω 环原始输出
    float tau_L_cmd;      // Nm，左轮实际请求力矩
    float tau_R_cmd;      // Nm，右轮实际请求力矩
    float tau_limit_L_pos; // Nm，左轮正向可用上限
    float tau_limit_L_neg; // Nm，左轮反向可用下限
    float tau_limit_R_pos; // Nm，右轮正向可用上限
    float tau_limit_R_neg; // Nm，右轮反向可用下限
    bool sat_v;            // yaw 优先分配是否削减了 τ_v
    bool sat_w;            // yaw 优先分配是否削减了 τ_ω
};

class VWController {
public:
    VWController() { set_params(VWParams{}); }

    // ══ 调参接口 ══
    void set_params(const VWParams& p);
    const VWParams& params() const { return params_; }

    // ══ 5kHz 高频接口：角速度环 ══
    /// 每次 IMU 采样调用一次。内部更新 τ_ω 状态。
    /// @param gyro_z_raw  陀螺仪 Z 轴原始值 (rad/s)，未滤波
    /// @param dt_imu      时间步长 (s)，典型 200us
    void omega_step(float gyro_z_raw, float dt_imu);

    // ══ 1kHz 控制节拍：巡线 + 速度环 + 混控 ══
    /// @param lateral_error  预处理后的横向误差 (m)，正=偏左
    /// @param curvature      当前位置曲率 (1/m)
    /// @param v_fwd          定位融合前向速度 (m/s)
    /// @param v_ref          目标线速度 (m/s)
    /// @param dt_ctrl        控制周期 (s)，典型 1ms
    VWControlResult control_tick(float lateral_error, float curvature,
                                 float v_fwd, float v_ref, float dt_ctrl);

    // ══ 轮速更新（电压前馈需要）══
    /// 每控制周期调用一次，设置左右轮角速度供电压前馈使用
    void set_wheel_omega(float wL, float wR) {
        omega_wheel_L_ = wL; omega_wheel_R_ = wR;
    }

    // ══ 外部 ω_ref 覆盖（给未来纯追踪等上层控制器）══
    void override_omega_ref(float omega_ref) { omega_override_ = omega_ref; }
    void clear_omega_override() { omega_override_ = NAN; }

    // ══ 重置 ══
    void reset();

    // ══ 调试访问 ══
    const OmegaLoop&          omega()    const { return omega_loop_; }
    const VelocityLoop&       velocity() const { return vel_loop_; }
    const TrackingController& tracking() const { return track_ctrl_; }

    float omega_ref_used()  const { return omega_ref_used_; }
    float v_ref_last()      const { return vel_loop_.v_ref(); }
    float tau_v()           const { return last_result_.tau_v; }
    float tau_omega()       const { return last_result_.tau_omega; }
    const VWControlResult& last_result() const { return last_result_; }

private:
    VWParams params_;

    OmegaLoop          omega_loop_;
    VelocityLoop       vel_loop_;
    TrackingController track_ctrl_;

    // ω_ref 状态
    float omega_ref_used_   = 0.0f;
    float omega_override_   = NAN;  // NAN = 不覆盖，使用 TrackingController
    bool  omega_ref_set_    = false; // 是否已有有效 ω_ref

    // 轮速（电压前馈）
    float omega_wheel_L_ = 0.0f;
    float omega_wheel_R_ = 0.0f;
    VWControlResult last_result_{};

    static float tau_to_norm(float tau, float max_tau) {
        if (max_tau <= 0.0f) return 0.0f;
        return ms::math::clamp(tau / max_tau, -1.0f, 1.0f);
    }
};

} // namespace ms
