# GPT 执行记录（滚动摘要）

本文件不是每轮必读。只有 compact 后恢复、跨聊天栏交接、连续失败诊断，或需要最近验证结果时才读。

维护规则：每完成一个工作块就清理本文件，删除已完成/废弃/重复流水账；保留当前状态、仍有效约束、最近关键结果、下一步和验收命令。目标是 compact-safe，不是完整历史档案。

## 当前状态

- 当前执行主题：底层 omega 环动态差模余量输入整形。
- 当前阶段禁止：巡线 `single/sweep/workbench`，除非用户明确解除；先完成底层角速度阶跃/极限量具。
- 主动路径：`control_core.vw_omega_step` / `vw_control_tick`，不要回到 legacy `control_core.step()`。
- 调参优先级：闭环带宽/响应时间 > 小幅超调 > 速度/角速度极限值。
- 最新计划修正：不要只靠固定提高 `w_cmd_max/w_max`；激进程度应前移到输入整形层，根据左右电机剩余差模力矩余量动态限制 `omega_ref/alpha_ref`。

## 当前 6 行状态

目标：实现并验证 omega 轴动态差模余量输入整形，让 `alpha_ref` 随当前电机可用差模力矩余量变化。
允许改：`cpp/control_core` 的 omega ReferenceShaper/OmegaLoop/VWController debug；`scripts/bench_vw.py` 的 trace/omega-limit 字段；必要文档记录。
禁止改：DOB 参数、Kp/Ki/Kd、线速度轴整形、巡线策略、物理模型、电机模型；不跑巡线。
必跑验证：`cmake --build cpp\build --config Release`；`python -m py_compile scripts\bench_vw.py`；`omega-limit`；必要时 `omega-step` 和 `bw --loop omega`。
通过判据：trace/json 记录 `tau_w_margin_pos/neg`、`alpha_limit_pos/neg`；高速反电势/电压余量下降时 alpha 上限自动收敛；后端 `sat_w` 不长期发生。
失败动作：若动态整形发散或长期 allocation 饱和，停止硬推并记录瓶颈，不调 DOB/Kp/Ki/Kd、不跑巡线。

## 当前模块契约

- 输入：左右轮角速度 `omega_wheel_L/R`、电机参数 `V_bus/R/Kt/Ke/G/eta/I_peak`、保持的 `tau_v_hold`、当前 `tau_w_now`、当前 `omega_ref`。
- 输出：`tau_w_feasible_min/max`、`tau_w_margin_pos/neg`、`alpha_limit_pos/neg`，并由 ReferenceShaper 生成连续的 `omega_ref/alpha_ref/j_omega_ref`。
- 限幅职责：固定 `w_cmd_max` 和 `w_alpha_max` 只作为绝对安全上限；常态激进程度由电机余量动态决定；yaw-priority allocation 仍作为最后保护和诊断。
- 调用频率：`omega_step()` 5kHz 每拍先用最新轮速和保持 `tau_v` 计算余量，再更新 omega shaper；速度轴、DOB、Kp/Ki/Kd 不改。

## 最近关键结果

- 安全量具已通过：极端 `v_cmd=100m/s`、`w_cmd=1000rad/s` 被 clamp 到 `v_ref=5m/s`、`omega_ref=20rad/s`；`a_ref=10m/s^2`、`alpha_ref=600rad/s^2`；最大 duty 约 `0.542`。
- 底层总 bench 曾通过：omega step@10 rise `23ms`、overshoot `~2%`、ss error `~0%`；v step@2 rise `159ms`、overshoot `0%`、ss error `-0.5%`；omega/v bandwidth 约 `40Hz`；解耦误差小。
- 宽范围纯阶跃：默认范围内 `omega <= 20rad/s`、`v <= 5m/s` 稳定；超过默认上限会安全 clamp。
- 机械极限评估：当前默认不是硬件极限，主要受软件整形、命令 clamp、上层策略限制。
- omega 极限探索：`w_cmd_max=100`、`w_max=0.09` 时 100rad/s 可跟踪但 `max|u|≈0.981`，已接近电机电压/反电势极限；120rad/s 进入 duty=1 区域。
- 因用户追加设计，固定默认候选 `w_cmd_max=100`、`w_max=0.09` 暂不作为最终方案；先实现动态余量整形。

## 下一步

1. 检查当前其它聊天栏是否已部分实现动态差模余量字段，避免重复改。
2. 若未完成，实现最小闭环：余量计算 -> dynamic alpha limit -> trace/debug -> `omega-limit` 输出。
3. 验证只跑底层命令，不跑巡线。
4. 完成后再次压缩本文件，只留下新结论和下一步。
