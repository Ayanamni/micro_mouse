

---

# 2026-06-27 控制器完整复查报告：设计偏离、前馈模型、仿真模型与参数问题

## 0. 复查结论

本轮结论不是“单纯某个 Kp/Ki 没调好”。当前控制器已经实现了 v/ω 解耦、动态力矩限幅、yaw 优先分配、统一参数入口等关键工程框架，但它仍然没有完全实现 `控制器方案报告.md` 的核心控制假设：

1. 原方案要求轨迹层输出连续的 `v_ref, a_ref, j_ref, ω_ref, α_ref`，前馈直接使用显式 `α_ref`。当前代码没有显式 `α_ref` API，而是在 `OmegaLoop` 内部对 `ω_ref` 做差分。
2. 原方案禁止直接速度/角速度阶跃。当前 `eval_vw.py omega-step` 和 `bench_vw.py omega-step` 仍然给 `ω_ref` 直接阶跃；该测试只能作为压力测试，不能作为“前馈调完美”的主量具。
3. 当前直接阶跃触发 `OmegaLoop` 的跳变保护，`α` 前馈被清零；因此用 `omega-step` 改 `Jz` 或调角加速度前馈，本质上测不到真正的 `Jz * α_ref` 前馈。
4. `vw_omega_step()` 在 5kHz 更新 `τ_ω`，但电机混控和 `u_L/u_R` 只在 1kHz `vw_control_tick()` 内刷新。平滑参考下还能采样到一部分低频前馈，但它不是方案报告中的“5-10kHz 控制/DOB/力矩输出”结构；短脉冲或高频 `α_ref` 会被采样路径削弱或错过。
5. `w_Cfrict` 的量纲和注释存在严重误导。代码把它作为轮端差动力矩 `τ_ω`，而注释按车体偏航力矩 `M_z=μF_NR` 写。按当前 `downforce=6.0, skirt_mu=0.05, skirt_R=0.03`，裙边偏航力矩是 `0.009 Nm`，换算到轮端差动力矩只有 `0.009*r/B = 0.00105 Nm`；默认 `w_Cfrict=0.006 Nm` 等价于车体偏航力矩约 `0.051 Nm`，约为裙边项 5.7 倍。
6. 但是，把 `w_Cfrict` 直接改成 `0.00105` 并不能解决问题。纯前馈实验显示当前 MuJoCo 偏航对象存在明显库伦/接触死区断崖：`w_Cfrict=0.009` 时几乎不转，`0.010` 时直接冲到约 `98 rad/s`。这说明仿真对象不是方案前馈/DOB假设的一阶线性 `Jz/Dω` 对象。
7. 因此目前最危险的做法是继续拿直接阶跃去盲调 `β/Kp/Ki/DOB`。反馈可以把阶跃稳态“拉对”，但这会掩盖前馈不可辨识、仿真非线性过强、控制输出采样层级不对的问题。

一句话判断：当前控制器方向没有错，但已经偏离原方案中“连续轨迹 + 显式加速度前馈 + DOB + 残差反馈”的实现方式；前馈参数也有单位/注释错误；仿真偏航模型使纯前馈阶跃目标本身不成立。下一步必须先修参考生成和 5kHz 输出链路，再谈把阶跃响应调到接近完美。

## 1. 原方案的不可违背假设

从 `控制器方案报告.md` 可直接抽出这些约束：

1. 控制输入必须连续。报告第 493-506 行明确说明速度和角速度不能直接阶跃，轨迹层必须输出连续 `v_ref, a_ref, j_ref, ω_ref, α_ref`。
2. 角速度前馈公式是轮端差动力矩：

   `τ_ω,ff = (J_z*r/B)*α_ref + (D_ω*r/B)*ω_ref`

   见报告第 748-760 行。
3. 反馈只修正残差，不应该承担把不可执行阶跃变成真实运动的任务。报告第 76-82 行、第 770-786 行说明 2DOF PI/LQI 的定位是残差反馈。
4. DOB 是原方案核心部件，不是后续装饰。报告第 885-1014 行给出 DOB 结构，最终控制律为 `u = u_ff + u_fb - d_hat`。
5. 电机/底层应尽量被上层看成力矩源；如果不是力矩源，必须做好电压补偿和动态力矩限幅。报告第 1178-1233 行、第 1242-1278 行与当前动态限幅方向一致。

当前实现已经接近第 5 点，但第 1-4 点仍未完整实现。

## 2. 当前代码与原方案的偏离点

### 2.1 没有显式 `α_ref`

当前 `cpp/control_core/vw_controller.cpp:41-52` 在 `OmegaLoop::step()` 内部用

`omega_dot_raw = (omega_ref_ - omega_ref_prev_) / dt`

推导角加速度，然后再算

`tau_ff_ = K_alpha*omega_dot_ref + K_omega*omega_ref_ + Cw*sign(omega_ref_)`

这与原方案不同。原方案里 `α_ref` 应由轨迹层或参考生成层明确给出，不应该由底层角速度环猜测。

后果：

1. 角加速度前馈取决于 `ω_ref` 更新时刻，而不是取决于真实轨迹曲率、速度、加速度。
2. 直接阶跃会触发跳变保护，导致 `α` 前馈被清零。
3. 以后上巡线时，侧向控制器直接改 `ω_ref`，会把线误差噪声/采样抖动变成伪 `α_ref`，必须靠 clamp 掩盖。

### 2.2 `ω` 环 5kHz 计算，但执行输出 1kHz 刷新

当前主循环结构：

1. `scripts/workbench.py:465-479`：5kHz 调 `control_core.vw_omega_step()`。
2. `scripts/workbench.py:482-535`：1kHz 调 `control_core.vw_control_tick()`，在这里才混控、限幅、输出 `u_L/u_R`。
3. `scripts/bench_vw.py:262-289` 和 `scripts/eval_vw.py:578-604` 也是同样结构。

这意味着 5kHz `τ_ω` 并没有以 5kHz 直接进入电机命令。它只是被 1kHz tick 采样成最新值再输出。对于平滑低频参考，问题不一定致命；但对于 `α_ref` 前馈、DOB、扰动拒绝和高带宽角速度环，这是结构性偏差。

### 2.3 当前 `omega-step` 不是前馈调参量具

直接阶跃与原方案冲突。更关键的是，当前代码在 `vw_controller.cpp:41-48` 检测到 `ω_ref` 大跳变后把 `omega_dot_raw` 置零。也就是说，直接阶跃不会测试 `Jz` 角加速度前馈。

本轮实验也验证了这一点：

```text
python scripts\bench_vw.py omega-step --mag 5 --v-hold 0 --vw-w-Kp 0 --vw-w-Ki 0 --vw-w-Kd 0 --w-Cfrict 0.00105 --vw-Dw 0.0021 --vw-Jz 0.000079
结果：ss = 0.029 rad/s, err = -99.4%

python scripts\bench_vw.py omega-step --mag 5 --v-hold 0 --vw-w-Kp 0 --vw-w-Ki 0 --vw-w-Kd 0 --w-Cfrict 0.00105 --vw-Dw 0.0021 --vw-Jz 0.01
结果：ss = 0.029 rad/s, err = -99.4%
```

`Jz` 从 `7.9e-5` 改到 `0.01`，结果完全不变。这不是“Jz 不重要”，而是当前测试路径没有测到 `Jz*α_ref`。

### 2.4 DOB 尚未实现

原方案第 9 章把 DOB 放在前馈和反馈之间，控制律是：

`τ_cmd = τ_ff + τ_fb - d_hat`

当前代码没有 `d_hat_v/d_hat_ω`，也没有 `Q(s)`、延迟对齐后的 `u_delay`、或 DOB 输出记录。因此现在的控制器本质是：

`τ_cmd = feedforward + 2DOF PI`

这不是最终方案，只是中间形态。

### 2.5 巡线层直接输出 `ω_ref`，不是轨迹层

当前 `TrackingController` 根据光电管横向误差直接生成 `ω_ref`，并限制在 `OMEGA_REF_MAX=20`。它没有输出连续 `κ(s), v_ref, a_ref, α_ref`，也没有摩擦圆约束。所以上巡线前应先把低层 `ω` 控制做成“能跟连续参考”的结构，而不是先把直接阶跃调得漂亮。

## 3. 前馈模型和参数核查

### 3.1 角速度前馈公式本身没有错

按报告的定义：

`J_z*ω_dot = (B/r)*τ_ω - D_ω*ω + M_d`

忽略扰动并令 `ω_dot = α_ref`：

`τ_ω,ff = (J_z*r/B)*α_ref + (D_ω*r/B)*ω_ref`

代码中的 `K_alpha = Jz*r/B`、`K_omega = Dw*r/B` 公式是对的。

### 3.2 错在输入来源、单位注释和仿真对象

当前默认值：

```text
r = 0.0105 m
B = 0.090 m
Jz = 7.9e-5 kg*m^2
Dw = 2.1e-3 Nm/(rad/s)
w_Cfrict = 0.006 Nm
```

换算：

```text
K_alpha = Jz*r/B = 9.22e-6 Nm/(rad/s^2)
K_omega = Dw*r/B = 2.45e-4 Nm/(rad/s)
```

如果 `ω_ref=5 rad/s` 且无角加速度，`Dw` 项只有 `0.00123 Nm`。默认 `w_Cfrict=0.006 Nm` 是主导项。

当前仿真裙边偏航库伦力矩：

```text
M_skirt = skirt_mu * downforce * skirt_R
        = 0.05 * 6.0 * 0.03
        = 0.009 Nm   # 车体偏航力矩
```

换算成控制器的轮端差动力矩：

```text
τ_ω,skirt = M_skirt * r / B
          = 0.009 * 0.0105 / 0.090
          = 0.00105 Nm
```

所以 `w_Cfrict=0.006` 如果解释为裙边摩擦补偿，单位是错的；它实际等价于车体偏航力矩：

```text
M_equiv = w_Cfrict * B / r
        = 0.006 * 0.090 / 0.0105
        = 0.0514 Nm
```

约为裙边偏航力矩 `0.009 Nm` 的 5.7 倍。

### 3.3 但不能简单把 `w_Cfrict` 改成 0.00105

纯前馈实验：

```text
# 只开前馈，关反馈，按裙边理论值
--vw-w-Kp 0 --vw-w-Ki 0 --vw-w-Kd 0 --w-Cfrict 0.00105
结果：ss = 0.029 rad/s, 目标 5 rad/s

# 只开前馈，使用当前默认 Cw
--vw-w-Kp 0 --vw-w-Ki 0 --vw-w-Kd 0 --w-Cfrict 0.006
结果：ss = 0.109 rad/s, 目标 5 rad/s

# 只开前馈，接近断崖
--w-Cfrict 0.009 --vw-Dw 0
结果：ss = 0.159 rad/s

# 只开前馈，略高一点
--w-Cfrict 0.010 --vw-Dw 0
结果：ss = 98.301 rad/s
```

这说明当前 MuJoCo 偏航对象有明显接触/库伦死区和断崖，不是一个可以靠 `D_ω*ω` 速度前馈调准的线性对象。在这个仿真对象上要求“纯前馈阶跃完美”是不合理目标。

合理目标应改为：

1. 纯前馈只负责跟踪连续 `α_ref/ω_ref` 的名义低频动态。
2. 接触死区、静摩擦、地面/轮胎非线性由 DOB 和残差反馈处理。
3. 如果希望 DOB 的一阶名义模型成立，仿真里必须加入可辨识的线性/准线性偏航阻尼项，或者明确把当前库伦接触模型视为扰动。

## 4. 仿真模型核查

### 4.1 当前偏航阻尼不是 `D_ω*ω`

`micromouse_sim/physics/engine.py:99-100` 计算：

`self._tau_skirt = skirt_mu * downforce * skirt_R`

`engine.py:204-211` 实际施加：

```python
if abs(yaw_rate) > omega_thresh:
    qfrc_applied[5] = -tau_skirt * sign(yaw_rate)
else:
    qfrc_applied[5] = -tau_skirt * yaw_rate / omega_thresh
```

这是带小速度线性平滑的库伦偏航摩擦，不是报告中 `D_ω*ω` 的粘性阻尼。再叠加轮胎接触、MuJoCo 约束和滑板几何，就会出现前馈不可辨识的死区断崖。

### 4.2 当前闭环阶跃能收敛，主要靠反馈

默认闭环实验：

```text
python scripts\bench_vw.py omega-step --mag 5 --v-hold 0 --vw-w-Kp 0.05 --vw-w-Ki 0.03 --vw-w-Kd 0 --w-Cfrict 0.006
结果：ss = 4.988 rad/s, err = -0.2%, overshoot = 38.5%, rise = 2.0ms
```

这证明闭环能把目标拉住，但不能证明前馈正确。当前的 38.5% 超调也不应该直接用降 `β_w` 去硬压，因为测试输入本身是违反方案的直接阶跃，且 `α` 前馈没有参与。

### 4.3 电池规格也有方案不一致

`控制器方案报告.md:50` 写的是 `2S LiPo`；当前 `micromouse_sim/actuation/motor_model.py:23` 是 `V_bus=11.1`，即 3S LiPo。项目 AGENTS 里也写了当前仿真使用 3S。

这不一定是 bug，可能是硬件设定更新；但必须在报告/参数源里明确。否则动态力矩上限、反电势前馈、最高速度、饱和统计都会按不同硬件解释。

## 5. Claude 计划复核

`claude计划.md` 中“先修量具、再调控制器”的总体方向是对的，尤其是以下点已经被验证或已修：

1. `w_z` 输出 bug。
2. `eval_vw omega-step` 参数污染。
3. `tau_v/tau_omega` 返回限幅前值导致饱和统计失真。
4. `beta/kaw/I_peak/w_Cfrict` 需要暴露。
5. 参数入口需要统一。
6. 执行延迟缓冲需要统一。

但 Claude 计划仍有三处需要修正：

1. 它仍把直接 `omega-step` 当成主要调参量具。现在应降级为压力测试，新增连续参考 `omega-profile` 作为前馈量具。
2. 它没有指出 `α_ref` 缺失和 5kHz/1kHz 输出链路不一致，这是比 `β_w` 更根本的结构问题。
3. 它没有指出 `w_Cfrict` 的量纲混淆，也没有把仿真偏航模型和前馈一阶名义模型的冲突单独列为阻塞项。

所以不能继续照 Claude 后半段“扫 β、加 DOB、上高速”直接做。必须先插入一轮结构修复。

## 6. 一步一步修改方案

### Phase 1：先让参考和前馈可验证

目标：别再用直接阶跃标定前馈。

1. 新增 `OmegaReference` 或等价结构：

   ```cpp
   struct OmegaReference {
       float omega_ref;
       float alpha_ref;
   };
   ```

2. 修改 C++ API：

   ```cpp
   vw_omega_step(gyro_z, omega_ref, alpha_ref, dt_imu)
   ```

   保留旧 `vw_override_omega_ref()` 只作为兼容入口，但新 bench/eval/workbench 不再依赖底层差分猜 `alpha_ref`。

3. `OmegaLoop::step()` 不再内部对 `omega_ref` 差分作为主路径。可以保留一个 legacy fallback，但 debug 中要明确 `alpha_ref_source = explicit|derived`。

4. 新增 `scripts/bench_vw.py omega-profile`：

   - 输入 `omega_target`、`profile_time`、`shape=s_curve|second_order`。
   - 输出连续 `ω_ref(t)` 和 `α_ref(t)`。
   - 验收 `α_ref` 峰值、`τ_ff` 峰值、`ω_meas` 跟踪误差。

5. `omega-step` 保留，但报告里标记为“不可执行阶跃压力测试”，不再用于前馈参数判定。

验收：

```bash
python scripts/bench_vw.py omega-profile --omega 5 --profile-time 0.20 --vw-w-Kp 0 --vw-w-Ki 0
```

要求：调 `Jz` 会显著改变加速段 `tau_ff` 和响应，而不是像当前直接阶跃一样完全不变。

### Phase 2：让 5kHz `τ_ω` 真正进入电机输出

目标：`ω` 环/DOB/前馈不是只被 1kHz 采样。

1. 在 C++ 中拆出“混控与电压前馈”函数：

   ```cpp
   vw_mix_step(tau_v_hold, tau_omega_now, wheel_L_omega, wheel_R_omega)
   ```

2. 1kHz `control_tick()` 只更新：

   - `tau_v_hold`
   - tracking 产生的下一段 `omega/alpha` 参考
   - 低频状态/日志

3. 5kHz `omega_step()` 后立即调用 mix，用最新 `tau_omega_now` 和保持的 `tau_v_hold` 刷新 `u_L/u_R`。

4. Python 主循环中，物理步进前使用最新 `u_L/u_R` 经延迟和电机模型转成 `tau_L/tau_R`。

验收：

1. 5kHz 日志里 `tau_omega` 和 `u_L/u_R` 同步变化。
2. `omega-profile` 下 `Jz`、`alpha_ref`、`tau_ff` 有可观测因果关系。
3. 1kHz/5kHz 输出对同一低频 profile 的结果差异可量化，而不是混在一起。

### Phase 3：修正 `w_Cfrict` 定义和仿真偏航模型

目标：让参数有物理意义，避免以后再次把车体力矩和轮端力矩混用。

1. 重命名参数：

   - `vw_w_Cfrict` 改为 `vw_tauw_coulomb`。
   - 注释写清楚：单位是轮端差动力矩 `τ_ω=(τ_R-τ_L)/2`。

2. 新增只读计算项或日志：

   ```text
   body_yaw_coulomb = skirt_mu * downforce * skirt_R
   wheel_tau_equiv = body_yaw_coulomb * wheel_r / track_B
   ```

3. 默认值不要继续写“理论 0.009”。应写：

   ```text
   wheel_tau_equiv = 0.00105 Nm for downforce=6, mu=0.05, R=0.03
   ```

4. 仿真模型二选一：

   - 如果目标是验证 DOB/前馈的一阶名义模型：在 `engine.py` 中加入可配置 `yaw_viscous_damping`，即 `qfrc_applied[5] += -D_yaw_sim * yaw_rate`，库伦项保留为扰动。
   - 如果目标是极真实接触模型：保留库伦/接触死区，但不要要求纯前馈阶跃完美；改用 DOB 和反馈处理。

建议选择第一种作为控制器开发默认，第二种作为 robustness 测试。

验收：

1. 纯前馈 `omega-profile` 不再出现 `0.009` 不动、`0.010` 飞掉的断崖。
2. 由 `D_yaw_sim` 识别出的 `Dw` 与控制器 `vw_Dw` 可以互相解释。
3. DOB 关闭时的误差可解释，DOB 打开后低频扰动明显下降。

### Phase 4：再调前馈，不调反馈

目标：响应用户提出的“先关反馈纯调前馈”，但测试对象必须是连续 profile，不是直接阶跃。

步骤：

1. 设置：

   ```text
   vw_w_Kp = 0
   vw_w_Ki = 0
   vw_w_Kd = 0
   DOB = off
   tracking = off
   v_hold = 0 或 1 m/s 分别测试
   ```

2. 用 `omega-profile` 而非 `omega-step`。

3. 先调 `Jz`：

   - 看加速段 `ω_meas` 是否超前/滞后。
   - 只用 profile 的加速段，不看稳态死区。

4. 再调 `Dw`：

   - 看匀速 `ω_ref` 段所需保持力矩。
   - 如果仿真仍是库伦主导，`Dw` 不要强行拟合库伦死区。

5. 最后调 `tauw_coulomb`：

   - 起点用 `0.00105 Nm`。
   - 如果加入了仿真粘性阻尼，库伦项只补低速死区。
   - 不允许再用 `0.009 Nm` 这种车体力矩直接填轮端参数。

验收：

1. 连续 `ω` profile 的纯前馈响应方向正确，无反馈时不要求零误差，但不能断崖失控。
2. 改 `Jz/Dw/tauw_coulomb` 分别只主要影响加速段/匀速段/低速死区。
3. 每个参数有可解释单位和可重复实验。

### Phase 5：再上 DOB

目标：处理库伦、接触、模型误差，而不是让 PI 积分硬扛。

1. 实现角速度 DOB：

   ```text
   d_hat = Q(s) * ((omega_dot_hat + a*omega_hat)/K - tau_omega_delay)
   K = B/(Jz*r)
   a = Dw/Jz
   tau_cmd = tau_ff + tau_fb - d_hat
   ```

2. `tau_omega_delay` 必须用执行延迟后的实际命令，不是 raw 命令。
3. `omega_dot_hat` 必须滤波，不能直接差分噪声陀螺。
4. `Q` 从低带宽开始，例如 30Hz，再到 60/90/120Hz。

验收：

```bash
python scripts/bench_vw.py disturb --loop omega
```

目标：扰动峰值和恢复时间显著改善，同时 `omega-profile` 不恶化。

### Phase 6：最后上反馈

目标：反馈只修残差。

1. 保持 `omega-profile`，先 `Kp`，再 `Ki`，最后考虑 `Kd`。
2. `beta_w` 不先作为遮羞布使用。只有在连续参考下仍有 setpoint kick 时再降。
3. 禁止用“直接阶跃超调 < 某数值”作为唯一目标。
4. 同时看：

   - profile RMS error
   - steady error
   - tau saturation
   - disturbance rejection
   - noise-to-torque

验收目标建议：

```text
omega-profile 5 rad/s：无明显超调，稳态误差 < 3%
omega-profile 10 rad/s：稳态误差 < 5%
disturb omega：峰值扰动下降 > 50%
tau_w_sat：常规 profile 下 < 10%，极限 profile 下可短时饱和
```

### Phase 7：低层稳定后再上巡线

1. tracking 层不能直接给不可连续 `ω_ref`。
2. 根据曲率、速度、横向误差生成连续 `ω_ref/α_ref`。
3. 加摩擦圆限速：

   `v_max = sqrt(a_y_max / |κ|)`

4. 再跑：

```bash
python scripts/eval_vw.py single --track 2019kansai --speed 3.5 --duration 12 --seed 42
python scripts/eval_vw.py sweep --track 2019kansai --speed-min 1.0 --speed-max 5.0 --step 0.25 --duration 12
```

## 7. 立即执行清单

最优先顺序如下：

1. 新增连续 `omega-profile` 量具。
2. 修改 `vw_omega_step` 支持显式 `omega_ref/alpha_ref`。
3. 把 5kHz `tau_omega` 变成 5kHz `u_L/u_R` 输出，而不是只在 1kHz 采样。
4. 重命名并修正 `w_Cfrict` 的单位注释，按轮端差动力矩记录。
5. 给仿真加入可配置 `yaw_viscous_damping`，把库伦项作为扰动，不再让前馈去拟合断崖死区。
6. 关反馈，用连续 profile 重辨识 `Jz/Dw/tauw_coulomb`。
7. 上 DOB。
8. 上残差反馈。
9. 最后上巡线和高速 sweep。

不建议现在继续做的事：

1. 不建议继续在直接 `omega-step` 上追求“完美前馈”。
2. 不建议直接把 `w_Cfrict` 改成 `0.00105` 后认为问题解决。
3. 不建议现在就调巡线 `lat_Kp/Kd`。
4. 不建议把默认闭环阶跃稳态误差接近 0 解释为前馈正确。

## 8. 最终判断

当前问题是四类问题叠加：

1. 设计偏离：缺显式 `α_ref`、缺连续参考生成、缺 DOB、5kHz 输出链路不完整。
2. 前馈参数问题：`w_Cfrict` 量纲/注释错误，默认值不是裙边理论轮端等效值。
3. 仿真模型问题：偏航对象是库伦/接触死区主导，不是 `D_ω*ω` 线性阻尼主导。
4. 测试方法问题：直接 `omega-step` 违反原方案，并且触发 `α` 前馈清零，不能用来调 `Jz` 或评价纯前馈。

所以答案是：控制器确实部分偏离了原本设计；前馈公式本身没错，但输入来源和 `w_Cfrict` 参数解释错了；仿真环境的偏航模型也让纯前馈阶跃不可辨识；参数当然也有问题，但参数不是第一根因。正确路线是先修结构和量具，再调前馈、DOB、反馈，最后再上巡线。

---

# 2026-06-28 修订计划：下层控制器必须保护硬件并逼近极限

## 0. 新目标

原控制方案还不够硬。只要求轨迹层输出连续 `v_ref/ω_ref/α_ref` 过于理想，实际工程里上层可能输出阶跃、折线、噪声、临时避障命令、巡线误差突变，甚至错误的过大指令。下层控制器不能因此炸掉。

新的控制契约：

1. 上层可以输出粗糙命令：`v_cmd, ω_cmd`。
2. 下层必须把命令整形成硬件可执行参考：`v_ref, a_ref, j_ref, ω_ref, α_ref, j_ω_ref`。
3. 前馈、DOB、反馈一律使用整形后的参考，不直接使用上层命令。
4. 下层必须做动态力矩可行域、反电势/电流限幅、yaw 优先、抗饱和和状态降额。
5. 任何上层命令都不允许让电机命令、积分器、DOB 或仿真状态发散。
6. 在不炸的前提下，下层应尽量逼近当前硬件理论极限，而不是靠保守限幅把性能浪费掉。

## 1. 架构修订

旧命名容易混淆，必须拆成三层：

```text
上层命令层:
  v_cmd, omega_cmd
  来源可以是巡线、轨迹、键盘、测试脚本或未来规划器

参考整形层:
  v_cmd     -> TD_v -> v_ref, a_ref, j_ref
  omega_cmd -> TD_w -> omega_ref, alpha_ref, j_omega_ref

控制执行层:
  feedforward(ref) + DOB(ref, measured, delayed_u) + residual_feedback(ref, measured)
  -> yaw-priority allocation
  -> voltage/feedforward duty
  -> motor/MuJoCo
```

这里的 TD 可以是 ADRC 风格跟踪微分器，也可以先实现成“二阶/三阶限幅跟踪器”。第一版建议用确定性受限二阶 TD，原因是参数物理意义清楚，容易和硬件极限对应：

```text
v:     |v_ref| <= v_max, |a_ref| <= a_max, |j_ref| <= j_max
omega: |omega_ref| <= omega_max, |alpha_ref| <= alpha_max, |j_omega_ref| <= j_omega_max
```

后续如果需要再替换为更标准的 ADRC `fhan` TD，但接口不变。

## 2. 硬件极限必须进入参考整形

参考整形不能拍脑袋写固定 `a_max/alpha_max`。它至少需要考虑：

1. 电池电压和反电势：

   `I_max(Ω) = (V_bus - K_e G |Ω|) / R`

2. 电机峰值电流：

   `|I| <= I_peak`

3. 左右轮可行力矩：

   `τ_L ∈ [τ_L_min, τ_L_max]`

   `τ_R ∈ [τ_R_min, τ_R_max]`

4. v/ω 混控：

   `τ_L = τ_v - τ_ω`

   `τ_R = τ_v + τ_ω`

5. 地面摩擦圆：

   `a_y = v * ω = v^2 κ`

6. 负压和轮胎摩擦：

   `F_max ≈ μ (m g + downforce)`

第一版可以先用保守常数：

```text
v_cmd_max = 5.0 m/s
v_accel_max = 8-12 m/s^2
v_jerk_max = 400-800 m/s^3
omega_cmd_max = 20 rad/s
omega_alpha_max = 400-1000 rad/s^2
omega_jerk_max = 30000-100000 rad/s^3
```

第二版再把 `a_max/alpha_max` 动态化，由当前轮速、电压、电流和 yaw 优先分配实时计算。

## 3. 前馈定义修订

前馈不再从 `cmd` 或内部差分阶跃得来，只能用 TD 输出：

```text
τ_v_ff = (m_eq*r/2)*a_ref + (D_v*r/2)*v_ref + τ_v_coulomb
τ_w_ff = (J_z*r/B)*alpha_ref + (D_w*r/B)*omega_ref + τ_w_coulomb
```

重要修正：

1. `τ_w_coulomb` 的单位是轮端差动力矩 `τ_ω=(τ_R-τ_L)/2`，不是车体偏航力矩 `M_z`。
2. 如果仿真或实车存在明显静摩擦死区，不能把它全部塞进 `D_w*ω`，应由库伦项、DOB 和反馈共同处理。
3. 直接 `omega-step` 只能测试保护和抗炸能力，不能作为前馈辨识量具。

## 4. DOB 顺序修订

用户建议的顺序是正确的，但必须加一个前置条件：

```text
先 TD/参考整形
再纯前馈
再 DOB
再反馈
最后巡线
```

详细顺序：

1. 实现 TD，确保任意 `v_cmd/ω_cmd` 输入都不会产生无限 `a/α`。
2. 关闭反馈和 DOB，用连续 TD profile 验证前馈方向和单位。
3. 实现 DOB，但只让它处理整形参考下的模型误差，不让它替代 TD。
4. 打开反馈，反馈只修残差。
5. 上巡线，让巡线只产生 `ω_cmd` 或曲率命令，不能直接写底层 `ω_ref`。

## 5. 代码实施计划

### P1. 文档与接口冻结

1. 更新 `控制器方案报告.md`，加入“下层命令保护契约”和“TD 参考整形层”。
2. 更新 `gpt计划.md`，明确先文档后代码。
3. 冻结术语：

   ```text
   cmd = 上层命令，允许跳变
   ref = 下层整形参考，连续且有导数限制
   tau_cmd = 控制器请求力矩
   tau_alloc = 动态可行域分配后的实际力矩
   duty = 电机电压命令
   ```

### P2. C++ 控制核心

1. 新增 `ReferenceShaper` 或 `TrackingDifferentiator` 类。
2. `VWParams` 新增：

   ```text
   v_cmd_min/v_cmd_max
   v_accel_max/v_jerk_max
   w_cmd_max
   w_alpha_max/w_jerk_max
   ```

3. `VelocityLoop` 内部保存：

   ```text
   v_cmd, v_ref, a_ref
   ```

4. `OmegaLoop` 内部保存：

   ```text
   omega_cmd, omega_ref, alpha_ref
   ```

5. `vw_control_tick()` 接口语义改为接收 `v_cmd`，不是已经可执行的 `v_ref`。
6. `override_omega_ref()` 改名或新增 `override_omega_cmd()`，旧 API 保留但标记 legacy。
7. debug 输出增加：

   ```text
   v_cmd, v_ref, a_ref
   omega_cmd, omega_ref, alpha_ref
   td_v_sat, td_w_sat
   ```

### P3. 量具

1. `bench_vw.py omega-step` 改名语义：输入阶跃 `omega_cmd`，观察 TD 后的 `omega_ref/alpha_ref` 和真实 `omega`。
2. 新增 `omega-profile` 或 `td-profile`，专门测试 TD 输出是否满足极限。
3. `eval_vw.py omega-step` 输出必须同时列：

   ```text
   omega_cmd
   omega_ref
   alpha_ref
   omega_meas
   tau_ff/tau_fb/tau_alloc
   ```

### P4. 调参顺序

1. 关反馈：`w_Kp=w_Ki=w_Kd=0, v_Kp=v_Ki=0`。
2. 关 DOB。
3. 只测 TD + 前馈。
4. 确认前馈不会把 TD profile 带炸。
5. 加 DOB。
6. 加反馈。
7. 上巡线。

## 6. 验收标准

### 安全验收

1. 输入 `v_cmd` 从 0 跳到 5m/s，不产生无限加速度，不出现 NaN，不出现持续电机饱和积分风up。
2. 输入 `omega_cmd` 从 0 跳到 20rad/s，不出现电机命令振荡发散。
3. 输入上层错误命令，例如 `v_cmd=100m/s`、`omega_cmd=1000rad/s`，下层 clamp 到硬件极限，系统仍可恢复。

### 性能验收

1. 在不丢线前，下层尽量吃满电压/电流/摩擦能力，而不是固定保守限幅。
2. `tau_w_sat/tau_v_sat` 短时可高，长期不能被积分器继续推爆。
3. 速度越高，动态力矩上限应随反电势下降，评估报告必须显示这一点。
4. 上层巡线输出粗糙时，底层也不炸；只是根据硬件极限平滑执行。

## 7. 当前暂停点

按用户要求，代码实现应在文档更新后再继续。下一步开始写代码前，需要先确认：

1. `控制器方案报告.md` 已追加修订章节。
2. 本计划中的 `cmd/ref/tau/duty` 术语已固定。
3. TD 第一版采用确定性受限二阶跟踪器，而不是直接上复杂 ADRC `fhan`。

## 8. 维护协议暂停点

2026-06-28 用户追加要求：先不要实现，做完完整计划就停下来。

本轮只允许完成：

1. 项目级维护协议。
2. Codex/Claude 共用入口规则。
3. 控制器后续改造计划。

本轮禁止继续：

1. 实现 TD/参考整形层。
2. 修改 C++ 控制器逻辑。
3. 调参。
4. 跑巡线优化。
5. 添加 DOB。

下一轮继续前必须先确认：

1. `项目维护协议.md` 是否被用户接受。
2. `控制器方案报告.md` 第 24 节是否作为新控制架构基准。
3. 是否按 `cmd -> ref -> feedforward/DOB/feedback -> allocation -> duty` 的顺序开始实现。
