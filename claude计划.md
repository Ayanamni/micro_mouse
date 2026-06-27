1. > 
---

# 第三轮：GPT审核修正 + 量具/一致性优先修复（2026-06-27）

> GPT对第二轮计划进行了逐条审核（`gpt计划.md`），本论已逐项交叉验证源码。
> **核心发现**：第二轮在量具污染（`eval_vw omega-step` Cw错误、`w_z`输出恒为0、`tau`返回限幅前值）和代码理解偏差（β_w实际是1.0而非0.3）的情况下继续调参。本轮**严格先修量具、再调控制器**。

## GPT审核验证结果（已逐条溯源）

| GPT主张 | 源码证据 | 判决 |
|---------|----------|------|
| `eval_vw omega-step` Cw错传1.0 | `eval_vw.py:557` 写死 `1.0, 80.0` | ✅ 确凿bug，FF=20x w_max |
| `w_z` 输出恒为0 | `pipeline_localizer.cpp:109` 在清零 `slip_sample_cnt_(:101)` 后读它 | ✅ 确凿bug（指标级，不污染控制） |
| `tau_v/tau_omega` 返回限幅前值 | `vw_controller.cpp:314-315` 返回 `tv/tw` 而非 `tv_lim/tw_lim` | ✅ 确凿，饱和统计失真 |
| β_w实际是1.0不是0.3 | `VWParams::beta_w=1.0`(hpp:51), `set_params()`覆盖OmegaLoop的`0.3`(cpp:98) | ✅ **Claude第二轮#4错误** |
| `act_delay`仅workbench有 | eval_vw/bench_vw无任何延迟 | ✅ 确凿 |
| 线传感器忽略航向角 | `line_sensor.py:92` `sensor_lat + self._led_y` | ✅ 确凿（急弯失真） |
| 轮半径三套值 | engine.py=0.0104 vs base.xml/controller=0.0105 | ✅ 确凿（~1%偏差） |
| sensor_fwd_offset不一致 | workbench/eval=0.060 vs base.xml LED=0.040 | ✅ 确凿 |
| motor_I_peak/β/kaw未暴露 | `vw_set_params`签名止于`motor_V_bus` | ✅ 第二轮未做完 |
| workbench delay off-by-one | buffer长度N+1, append-then-pop → 实际N+1步延迟 | ✅ 小幅偏差(~20us) |
| `bench_vw omega-step`无Cw污染 | `bench_vw.py:207` 正确传 `p.vw_w_Cfrict` | ✅ bench路径干净，用bench做量具 |

## 第三轮修复步骤（严格单变量，先修量具再调控制器）

> 规则：每步：改文件 → 编译（如涉及C++）→ `bench_vw` 验证 → `eval_vw single` 回归 → 通过标准。

### Phase A — 量具与一致性修复（8步，优先）

**A1. 修 `localize_core` 的 `w_z` 输出bug**
- 文件：`cpp/localize_core/pipeline_localizer.cpp`
- 改动：在 `slip_sample_cnt_=0` 之前保存 `w_z_avg`，输出时用保存值
```cpp
// 在第101行(slip_sample_cnt_=0)之前插入:
double w_z_out = w_z_avg;
// 第109行改为:
output_.w_z = w_z_out;
```
- 编译：`cmake --build cpp/build --config Release && copy cpp\build\localize_core\Release\localize_core.cp312-win_amd64.pyd micromouse_sim\`
- 验证：`eval_vw.py single --speed 3.5 --duration 12 --seed 42` 输出 `max_w_z` 不再为0
- 通过标准：`max_w_z` 与 `vw_dbg[omega_meas]` 同量级

**A2. 修 `eval_vw.py omega-step` Cw错误**
- 文件：`scripts/eval_vw.py`
- 改动：L557 `1.0, 80.0` → `params.vw_w_Cfrict, 80.0`
- 无需编译（纯Python）
- 验证：`eval_vw.py omega-step --speed 1.0 --omega-step-value 5.0` 结果与 `bench_vw.py omega-step --mag 5` 可比
- 通过标准：两工具同参数下omega阶跃指标接近（超调/整定/SS误差差<10%）

**A3. 返回实际执行力矩 + 真实饱和标志**
- 文件：`cpp/control_core/vw_controller.hpp`、`vw_controller.cpp`、`control_core.cpp`
- 改动：
  - `VWControlResult` 增加：`tau_v_cmd, tau_omega_cmd, tau_L_cmd, tau_R_cmd, sat_v, sat_w`
  - `control_tick()` 返回 `tv_lim/tw_lim` 作为 cmd，`tv/tw` 作为 raw
  - pybind debug dict 增加 `tau_v_cmd, tau_omega_cmd, tau_v_raw, tau_omega_raw`
- 编译：`cmake --build cpp/build --config Release && copy cpp\build\control_core\Release\control_core.cp312-win_amd64.pyd micromouse_sim\`
- 验证：`eval_vw.py single` 饱和统计用 cmd 值重算，与仪表盘一致
- 通过标准：`tau_w_sat%` 与实际 `tw_lim` 逼近上限一致

**A4. 暴露 β_w, β_v, kaw_w, kaw_v, motor_I_peak, w_Cfrict 独立参数**
- 文件：`VWParams`（hpp）、`control_core.cpp`（pybind签名）、`workbench.py`（SimParams）、`eval_vw.py`（EvalParams）、`bench_vw.py`（BenchParams）
- 改动：
  - hpp：`w_Cfrict` 不再是 `lat_Kff` 别名，新增独立字段
  - pybind：`vw_set_params` 增加 `beta_w, beta_v, kaw_w, kaw_v, motor_I_peak, w_Cfrict` 参数
  - Python三文件：对应 dataclass 增加字段，调用 `vw_set_params` 时传入
- 编译：control_core .pyd
- 验证：`bench_vw.py omega-step --mag 5 --beta-w 0.3` 可扫 β
- 通过标准：β_w 可调且 cls override 生效；A5的β扫荡可执行

**A5. 验证并修正 Claude 第二轮 β_w 归因**
- 前提：A4 完成
- `bench_vw.py omega-step --mag 5 10 15` 分别扫 `beta_w = 1.0, 0.7, 0.5, 0.3, 0.1`
- 目标：确认当前 `beta_w=1.0`（非 Claude 归因的 0.3）下的实际超调，以及降 β 对超调/整定的影响
- 通过标准：5 rad/s 超调 **<15%**（从38.5%）；若β降低导致SS误差，由Ki补

**A6. 统一轮半径**
- 文件：`engine.py`
- 改动：`WHEEL_RADIUS = 0.0105`（与 base.xml 和控制器一致）
- 无需编译
- 验证：`eval_vw.py single --speed 3.5` 对比改前后 `pose[v_fwd]` 与 `st.forward_velocity` 平均偏差
- 通过标准：偏差减小约1%（~0.03 m/s @ 3m/s）

**A7. 统一 sensor_fwd_offset**
- 文件：`workbench.py`、`eval_vw.py`、`bench_vw.py`
- 改动：`sensor_fwd_offset: 0.060 → 0.040`（与 base.xml LED 位置一致）——除非确认实车是60mm
- 无需编译
- 验证：`eval_vw.py single --speed 3.5` 对比
- 通过标准：巡线 `max|lat|` 不显著退化（若退化说明旧模型过乐观，需接受）

**A8. 抽取共享参数源**
- 文件：新增 `scripts/vw_params.py` 或 `micromouse_sim/config/sim_params.py`
- 改动：`SimParams/EvalParams/BenchParams` 共用同一默认值字典/基类；CLI override 在创建后覆盖
- 无需编译
- 验证：`rg "vw_D_v|vw_C_frict|vw_w_Cfrict|vw_w_Ki"` 只出现一个默认定义源
- 通过标准：三入口打印同一参数摘要

### Phase B — 延迟与传感器同步（4步）

**B1. 将执行延迟同步到 eval_vw 和 bench_vw**
- 文件：`eval_vw.py`、`bench_vw.py`
- 改动：封装延迟缓冲为 `ActuationDelayBuffer` 工具类（三处共用）；修正 off-by-one：buffer 长度 = `act_delay_steps`（非 +1），先pop再append
- 无需编译
- 验证：`bench_vw.py omega-step --mag 10 --act-delay-us 0/200/400` 量化超调/相位变化
- 通过标准：`act_delay_us=0` 与当前结果一致；`act_delay_us=300` 下指标量化

**B2. 增加传感延迟**
- 文件：`eval_vw.py`、`bench_vw.py`
- 改动：统一 `sense_delay_us` 参数，对 IMU/encoder/line sensor 读数做采样保持延迟队列。初始统一延迟，后续可拆为 `imu_delay_us/enc_delay_us/line_delay_us`
- 验证：`sense_delay_us=0/300/800` 下生成 `omega-step` 对比表
- 通过标准：延迟增加时带宽下降、相位裕度减少，符合 `phi = -360 * f_c * T_d`

**B3. 增加电机力矩一阶滞后**
- 文件：`motor_model.py`
- 改动：`MotorModel` 增加 `torque_lag_fc` 参数（默认1000Hz或关闭），`compute_torque()` 输出经一阶低通
- 验证：`bench_vw.py bw --loop omega` 开/关力矩滞后对比
- 通过标准：滞后开启时高频增益合理下降

**B4. 修线传感器航向角修正**
- 文件：`line_sensor.py`
- 改动：每个LED的世界坐标由车体yaw旋转后投影：
```python
# 替换 L92: led_lat_offsets = sensor_lat + self._led_y
for i, led_y in enumerate(self._led_y):
    led_world = sensor_origin + R_yaw @ [0, led_y]
    # 投影到track或Frenet坐标
```
- 至少加入 `cos(yaw - track_heading)` 修正
- 验证：直线时新旧读数一致；固定半径弯中读数随yaw误差变化符合几何预期
- 通过标准：`eval_vw.py single 3.5m/s` 不因修正突然虚假变好；若变差，记录旧模型过乐观程度

### Phase C — 控制器精修（4步，在干净量具上重新做）

**C1. 重跑可信基线**

所有 Phase A/B 完成后：
```bash
python scripts/bench_vw.py all --json bench_baseline_r3.json
python scripts/eval_vw.py single --track 2019kansai --speed 3.5 --duration 12 --seed 42
python scripts/eval_vw.py sweep --track 2019kansai --speed-min 1.0 --speed-max 5.0 --step 0.25 --duration 12
```
目标：用修好的量具重新定标，不在旧数据上继续。

**C2. 扫 β_w 降小信号超调**

前提：A4/A5暴露了可调β_w
- 用 `bench_vw.py omega-step --mag 5 10 15 --beta-w <val>` 扫 β_w
- 起点：`Kp=0.05, Ki=0.03, Kd=0, Cw=0.006, beta_w=1.0`
- 扫 `beta_w = 1.0, 0.7, 0.5, 0.3, 0.1`
- 通过标准：5 rad/s 超调从38.5%降到 <15%；10/15 rad/s 不显著变慢；若SS误差增大，Ki补偿

**C3. 修 yaw 优先分配的最终抗饱和**

前提：A3已返回真实cmd值
- 文件：`vw_controller.cpp`
- 改动：用左右轮各自可行域 `[tau_max_L, tau_max_R]` 而非保守取min；将 `tv_lim/tw_lim` 回传给两环做最终 back-calculation
- 编译：control_core .pyd
- 验证：`bench_vw.py decouple` 对比改前后
- 通过标准：高速段 τ_w_sat 更准确；ω阶跃大幅值不因抢占而恶化

**C4. ω 环 DOB（若 C2+C3 后扰动仍 >8%）**

前提：量具可信、C2/C3完成
- 一阶名义对象 `P = Kω/(s+aω)`，`Q` 从30Hz起逐级加
- 验证：`bench_vw.py disturb --loop omega` 峰值从18.8% → <8%，恢复 <150ms
- 通过标准：disturb改善且omega-step超调不恶化；若恶化则减Q

### Phase D — 高速验收（3步）

**D1. 计算赛道摩擦圆理论上界**
- 文件：`eval_vw.py`（sweep输出增加）
- 输出：`κ_max`、最紧弯位置s、理论 `v_max_corner = sqrt(μ*(m*g+downforce)/(m_eq*κ_max))`
- 通过标准：sweep报告同时显示「达到理论上界百分比」

**D2. 重新 sweep**
```bash
python scripts/eval_vw.py sweep --track 2019kansai --speed-min 1.0 --speed-max 5.0 --step 0.25 --duration 12 --seed 42
```
- 报告最大不丢线速度、首次丢线速度/位置/时间/当时lat/ω/tau_sat

**D3. 多 seed + 真实度开关验收**
```bash
# 3 seed可重复性
for seed in 1 2 3; do
    python scripts/eval_vw.py single --track 2019kansai --speed <best> --duration 20 --seed $seed
done
# 逐步开真实度开关
--act-delay-us 300 --sense-delay-us 300
```
- 通过标准（建议）：不丢线、`max|lat| < 35mm`（留传感器余量）、`tau_w_sat% < 30%`、≥2圈多seed可重复

## 第三轮量化目标（修后量具上的干净基线）

| 指标 | 第二轮声称值 | 第二轮实际可靠性 | 第三轮目标 |
|------|-------------|-----------------|-----------|
| w_z 输出 | max_w_z=0.0（bug） | ❌ 不可信 | 有效值，与gyro同量级 |
| omega-step评估 | 用bench_vw（干净） | ⚠️ eval分支污染 | 两工具一致 |
| tau饱和统计 | 基于限幅前值 | ❌ 失真 | 基于实际cmd值 |
| β_w 实际值 | 归因为0.3 | ❌ 实际1.0 | 可调、可扫、可验证 |
| 轮半径一致性 | 三套值（0.0104/0.0105） | ❌ 系统偏差 | 统一0.0105 |
| sensor_fwd_offset | 0.060 vs 0.040 | ❌ 不一致 | 统一为实车值 |
| 3.50 m/s最大可生存 | max|lat|=48.4mm | ⚠️ 余量不足+量具污染 | 修量具后重定 |
| ω超调@5 rad/s | 38.5% | ⚠️ β_w归因错误 | <15%（通过β_w扫荡） |

## 风险与对策

| 风险 | 对策 |
|------|------|
| 修线传感器后巡线变差 | 说明旧模型过乐观，记录退化量；可能需要重新调lat_Kp应对更真实的传感器 |
| 统一轮半径后速度估计漂移 | 1%偏差很小，Kalman应吸收；若v误差增大则检查Kalman sigma |
| C++ 改动多、编译链易出错 | 每步只改一个.pyd（control_core或localize_core），不改则跳过编译 |
| 三轮修改量大、中途上下文过长 | 严格按Phase顺序，每Phase完成后跑一次`bench_vw all`快照；Phase A优先完成即可产出可信量具 |
| GPT自身判断可能有误 | 本轮已逐条溯源验证源码后才写入计划；A5的β_w扫荡是实验验证而非盲信GPT