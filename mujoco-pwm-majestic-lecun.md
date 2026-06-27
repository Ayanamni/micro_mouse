# 计划书：电子鼠仿真工作台 (Micromouse Simulation Workbench)

## 一、愿景

**一个让你只关心算法和调参，其他什么都不用管的地方。**

当前状态：`run_sim.py` 是一个命令行脚本。调参靠改代码里的常量，看结果靠终端日志。每次改一个参数要：改代码 → 重新跑 → 看日志 → 改代码 → 重新跑……这个过程是算法工程师最大的敌人。

目标状态：**一个图形化工作台**。仿真跑着，你拖动滑块，波形立刻反映变化。发现一组好参数，点保存。下次从参数库加载，一键对比两组参数的跑圈效果。标定流程内建，点一个按钮自动走完。

## 二、核心设计原则

### 2.1 可扩展 (extensible)

每个监控面板是独立模块，有统一接口。要加新的监控面板（比如"电机电流 vs 转速"、"IMU 原始值 vs 滤波值"），只需写一个 30 行的类，注册到面板列表，不需要改动任何现有代码。

```python
class MonitorPanel(ABC):
    """所有监控面板的基类"""
    name: str                          # 面板名称，用于图窗标题和选择列表
    update_rate: float = 10            # Hz, 面板刷新率（可低于控制频率）

    def build(self, fig, gs): ...      # 在给定的 GridSpec 区域创建子图
    def update(self, state, pose): ... # 每次刷新时调用，state=SimulationState, pose=LocalizeOutput
    def reset(self): ...               # 复位按钮按下时清空本面板的缓冲区
```

### 2.2 高兼容 (compatible)

工作台不绑定特定的赛道、车模、控制器。所有可切换的部分都通过配置注入：

- **赛道**：`--track robotena` → 自动找到对应数据文件
- **控制器**：`--controller pid` / `--controller mpc` / `--controller ladrc`（未来扩展）
- **定位器**：`--localizer pipeline` / `--localizer eskf`（未来扩展）
- **可视化后端**：默认 matplotlib，未来可加 Dear ImGui / web-based

兼容 MuJoCo viewer：可以同时开启（`--with-viewer`），也可以在 headless 模式下只靠工作台。

### 2.3 易用 (usable)

- `python scripts/workbench.py` — 零参数启动，跑默认赛道 2m/s
- `python scripts/workbench.py --track 2019kansai --speed 3.0` — 只指定你关心的
- 滑块调节 PID 和 Kalman 参数，实时生效（调用 `control_core.set_lateral_gains(...)` 等 C++ 接口）
- 发现好参数 → 点"保存配置" → 存为 `presets/my-perfect-tune.yaml`
- 下次 `python scripts/workbench.py --preset my-perfect-tune` 直接加载
- 标定模式：点"标定"按钮 → 自动直走 1m → 记录脉冲数 → 更新校准参数

## 三、图窗布局

```
┌──────────────────────────────────────────────────────────────────┐
│  Micromouse Simulation Workbench                    [⏸暂停] [🔄复位] │
├────────────────────────────┬─────────────────────────────────────┤
│  Panel A: 跟踪误差        │  Panel B: 速度 & 加速度              │
│  横向偏差 (mm), 线传感器   │  v_fwd vs target, v_enc vs v_imu,    │
│  16ch ADC 热力图 (可选)    │  accel_x/y 原始值, slip_scale       │
├────────────────────────────┴─────────────────────────────────────┤
│  Panel C: 定位地图 (俯视图)                                       │
│  红色 = 赛道中心线, 蓝色 = 定位估计, 灰色虚线 = 真实轨迹          │
│  当前位姿: 大圆点, 朝向箭头                                      │
│  等比例坐标, 自动缩放跟随                                         │
├────────────────────────────┬─────────────────────────────────────┤
│  Panel D: Kalman 诊断      │  Panel E: 电机指令                   │
│  innovation (mm),          │  u_L/u_R 波形, throttle vs steer     │
│  P[0] (v_fwd方差),         │  齿槽转矩, 反电动势电压              │
│  P[3] (accel_bias方差),    │  扭矩输出 vs 电流限制                │
│  accel_bias 估计值          │                                     │
├────────────────────────────┴─────────────────────────────────────┤
│  Panel F: 参数调谐 (右侧固定面板)                                  │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │ 横向PID:  Kp [===3.0===]  Kd [===1.0===]  Ki [==0.05==]  │   │
│  │ 速度PI:   Kp [===1.0===]  Ki [===0.3===]                 │   │
│  │ Kalman:   σ_accel [=0.005=]  σ_enc [=1e-4=]             │   │
│  │ 滑移:     thresh_lon [=0.03=]  k_slip [===5.0===]        │   │
│  │ 目标速度: [========1.5 m/s========]                       │   │
│  │ [💾保存配置] [📂加载配置] [📊导出数据]                     │   │
│  └──────────────────────────────────────────────────────────┘   │
├──────────────────────────────────────────────────────────────────┤
│  状态栏: t=3.4s  v=1.52m/s  lat=12mm  lap=2/5  loc_err=0.07m    │
│  Kalman: innov=0.15mm  bias=0.47m/s²  slip=1.0                  │
└──────────────────────────────────────────────────────────────────┘
```

### 面板优先级（MVP 必须有的）

| 优先级 | 面板 | 理由 |
|--------|------|------|
| P0 | A: 跟踪误差 | 巡线质量最直接的指标 |
| P0 | C: 定位地图 | 空间理解，定位质量可视化 |
| P0 | F: 参数调谐 | **核心价值**——实时调参 |
| P0 | 状态栏 | 关键数字一目了然 |
| P1 | E: 电机指令 | 控制量是否饱和、是否存在对抗 |
| P1 | D: Kalman 诊断 | 滤波器是否正常工作 |
| P2 | B: 速度&加速度 | 速度跟踪、滑移检测 |

## 四、配置管理

### 4.1 预设文件格式 (`presets/*.yaml`)

```yaml
# presets/my-2ms-tune.yaml
name: "2m/s 稳定巡线"
description: "robotena 赛道 2m/s, 侧偏<3cm"
created: "2026-06-19"
track: robotena
target_speed: 2.0

lateral_pid:
  Kp: 3.0
  Kd: 1.0
  Ki: 0.05
  Kff: 0.03

speed_pi:
  Kp: 1.0
  Ki: 0.3

kalman:
  sigma_accel: 0.005
  sigma_bias_walk: 0.0003
  sigma_enc_dist: 0.0001

slip:
  thresh_lon: 0.03
  thresh_lat: 0.5
  k_slip: 5.0

# 未来扩展
# controller: pid
# localizer: pipeline
# motor: real  # real | ideal
```

### 4.2 运行时操作

| 操作 | 快捷键 | 说明 |
|------|--------|------|
| 暂停/继续 | Space | 冻结仿真，检查波形细节 |
| 复位 | R | 清空波形 + 重置物理 → 自动跑一圈 |
| 保存配置 | Ctrl+S | 弹出文件名对话框，存为预设 |
| 加载配置 | Ctrl+L | 从预设列表选择 |
| 切换面板 | 1-6 | 放大指定面板到全屏 |
| 截图 | F12 | 保存当前图窗为 PNG |
| 导出数据 | Ctrl+E | 当前缓冲区存为 CSV/HDF5 |

## 五、标定模式

工作台内建标定工作流，菜单栏可触发：

### 5.1 陀螺零偏标定（上电自动）
```
1. 车静止
2. 采集 gyro_z 2 秒 (10000 samples @ 5kHz)
3. 计算 mean → gyro_bias_init
4. 计算 std  → gyro_noise_std
5. 显示结果，用户确认
```

### 5.2 编码器标定（直走 1m）
```
1. 用户将车放置于直线段起点
2. 点击"开始标定"
3. 车以低速 (0.5 m/s) 直线行驶 1m
4. 记录编码器脉冲数
5. 计算 pulses_per_m_L, pulses_per_m_R
6. 显示左右轮差异（诊断轮胎磨损/轮径不匹配）
```

### 5.3 轮距标定（原地旋转）
```
1. 车原地旋转 10 圈
2. 对比编码器推算的转角 vs 陀螺推算的转角
3. 修正 track_width
```

## 六、架构

```
workbench.py (入口, ~80 行)
  │
  ├── SimRunner (仿真线程, ~200 行)
  │   ├── 创建 PhysicsEngine, MotorModel, Track
  │   ├── 初始化 C++ cores (localize_core, control_core)
  │   ├── 主循环: 物理 50kHz → IMU 5kHz → 编码器 1kHz
  │   └── 提供最新 state, pose, debug → 给主线程
  │
  ├── Dashboard (GUI 主控, ~400 行)
  │   ├── _init_figure()        → 创建 matplotlib 图窗
  │   ├── _build_panels()       → 按布局实例化所有面板
  │   ├── _build_sliders()      → 创建参数滑块
  │   ├── _build_statusbar()    → 状态栏文本
  │   ├── _update_loop()        → 定时器回调 (30Hz)
  │   │   ├── 从 SimRunner 拉取最新数据
  │   │   ├── 逐面板调用 panel.update(state, pose)
  │   │   └── 更新状态栏
  │   └── _on_slider_change()   → 实时推送到 C++ cores
  │
  ├── Panel 插件系统 (每个 ~40-80 行)
  │   ├── panels/base.py            → MonitorPanel 基类
  │   ├── panels/tracking.py        → Panel A: 侧偏波形
  │   ├── panels/map.py             → Panel C: 定位地图
  │   ├── panels/speed.py           → Panel B: 速度跟踪
  │   ├── panels/kalman.py          → Panel D: Kalman 诊断
  │   ├── panels/motor.py           → Panel E: 电机指令
  │   └── panels/<your_panel>.py    → 用户自定义面板 (drop in!)
  │
  ├── PresetManager (~100 行)
  │   ├── save(path) / load(path)  → YAML 读写
  │   ├── apply_to_sim()           → 推送到 C++ cores
  │   └── list_presets()           → 列出所有可用预设
  │
  └── CalibrationWizard (~150 行)
      ├── calibrate_gyro()         → 静止采集
      ├── calibrate_encoder()      → 直走 1m
      └── calibrate_track_width()  → 原地旋转
```

## 七、SimRunner：解耦仿真与 GUI

关键设计：仿真在后台线程运行，GUI 在主线程。两者通过线程安全的数据缓冲区通信：

```python
class SimRunner:
    """仿真后台线程。独立于 GUI，可以 headless 运行。"""

    def __init__(self, config: SimConfig):
        self.config = config
        self._running = False
        self._paused = False

        # 线程安全的最新数据（GUI 端只读）
        self._lock = threading.Lock()
        self._latest_state: SimulationState | None = None
        self._latest_pose: dict | None = None
        self._latest_debug: dict | None = None
        self._latest_lateral_error: float = 0.0
        self._latest_curvature: float = 0.0
        self._lap_count: int = 0
        self._max_lat: float = 0.0

    def start(self):
        """启动仿真线程"""
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)

    def get_snapshot(self) -> dict:
        """GUI 线程安全读取最新数据"""
        with self._lock:
            return { ... }  # 拷贝所有最新值

    def push_param_change(self, param_name: str, value: float):
        """从 GUI 滑块推送参数变更"""
        # 直接调用 C++ set_*_gains / set_calibration / set_slip_params
        ...

    def _run(self):
        """仿真主循环（在后台线程中）"""
        # 与 run_sim.py 相同的主循环
        # 每 1kHz 周期:
        #   - push_encoder → read_pose
        #   - line sensor → control_core.step
        #   - 更新 _latest_* 字段（持锁）
        ...
```

这样 `SimRunner` 可以独立于 GUI 使用：
```python
# 无 GUI 的批量仿真
runner = SimRunner(config)
runner.start()
runner.wait_for_completion()
```

也可以被 GUI 驱动——这正是兼容性的体现。

## 八、文件变更清单

### 新建

| 文件 | 行数 | 说明 |
|------|------|------|
| `scripts/workbench.py` | ~80 | 入口：解析参数，组装 SimRunner + Dashboard，启动 |
| `micromouse_sim/workbench/__init__.py` | ~10 | 包声明 |
| `micromouse_sim/workbench/sim_runner.py` | ~250 | 仿真后台线程，数据缓冲区 |
| `micromouse_sim/workbench/dashboard.py` | ~450 | matplotlib GUI：布局、面板管理、滑块、状态栏 |
| `micromouse_sim/workbench/panels/__init__.py` | ~20 | 面板注册表 |
| `micromouse_sim/workbench/panels/base.py` | ~40 | MonitorPanel 基类 |
| `micromouse_sim/workbench/panels/tracking.py` | ~80 | 跟踪误差面板 |
| `micromouse_sim/workbench/panels/map.py` | ~100 | 定位地图面板 |
| `micromouse_sim/workbench/panels/speed.py` | ~80 | 速度面板 |
| `micromouse_sim/workbench/panels/kalman.py` | ~80 | Kalman 诊断面板 |
| `micromouse_sim/workbench/panels/motor.py` | ~60 | 电机指令面板 |
| `micromouse_sim/workbench/preset_manager.py` | ~100 | 预设保存/加载 |
| `micromouse_sim/workbench/calibration.py` | ~150 | 标定向导 |
| `presets/` | — | 预设目录 |
| `presets/default.yaml` | ~20 | 默认预设 |

### 修改

| 文件 | 改动 | 原因 |
|------|------|------|
| `micromouse_sim/__init__.py` | 添加 `from .workbench import SimRunner` | 包级导出，方便外部使用 |
| `启动仿真.bat` | 改为启动 `scripts/workbench.py` | 新的默认入口 |

### 不变

`run_sim.py`、`interactive.py`、所有 C++ 代码、所有物理模型 —— 完全不动。工作台是基于它们之上的新层。

## 九、与现有代码的关系

```
run_sim.py (现状: 命令行脚本)
    │
    │  提取核心循环逻辑
    ▼
SimRunner (新: 可被 GUI 或命令行复用)
    │
    │  被 Dashboard 驱动
    ▼
workbench.py (新: GUI 工作台)
```

`run_sim.py` 保留，但标记为 "legacy batch mode"。核心循环逻辑迁移到 `SimRunner`，避免代码重复。

## 十、实施步骤

### Step 1: SimRunner 提取
- [ ] 从 `run_sim.py` 提取核心循环到 `SimRunner` 类
- [ ] 线程安全的数据缓冲区（`get_snapshot()`）
- [ ] 参数推送接口（`push_param_change()`）
- [ ] 验证：无 GUI 的批量运行与 `run_sim.py` 行为一致

### Step 2: Panel 基类 + 跟踪误差面板
- [ ] `MonitorPanel` 抽象基类
- [ ] `TrackingPanel` — 侧偏波形 + 线传感器 ADC 热力图
- [ ] 验证：独立测试面板更新逻辑

### Step 3: Dashboard 骨架
- [ ] matplotlib 图窗创建 + 布局引擎
- [ ] 面板注册与实例化
- [ ] 定时器回调 → 数据拉取 → 面板更新
- [ ] 验证：图窗显示，面板实时刷新

### Step 4: 定位地图面板
- [ ] 赛道中心线预计算
- [ ] 定位轨迹 vs 真实轨迹叠加
- [ ] 等比例坐标，自动缩放

### Step 5: 参数滑块系统
- [ ] 滑块控件创建
- [ ] 滑块值 → `SimRunner.push_param_change()` → C++ 接口
- [ ] 验证：拖动 Kp 滑块，车的行为立刻改变

### Step 6: Kalman 诊断 + 电机指令面板
### Step 7: PresetManager（保存/加载）
### Step 8: 标定向导
### Step 9: 快捷键、状态栏、收尾打磨
### Step 10: 文档（`presets/README.md` 说明如何创建/管理预设）

## 十一、验证

| 测试 | 方法 | 通过标准 |
|------|------|---------|
| 零参数启动 | `python scripts/workbench.py` | 3 秒内显示图窗，仿真开始运行 |
| 滑块实时生效 | 拖动 Kp 滑块 | 车的行为在 1 秒内反映变化 |
| 保存/加载预设 | 调好一组参数 → 保存 → 复位 → 加载 | 恢复到保存时的参数 |
| 标定模式 | 点击陀螺标定 | 显示静止采集结果（bias + noise） |
| 复位按钮 | 点击复位 | 波形清空 → 车重新跑一圈 → 自动停车 |
| 跑完 5 圈 | 设 target_speed=2.0 | 5 圈后定位误差 < 1% 总里程 |
| 面板可扩展 | 新建一个面板文件，放到 panels/ | 自动出现在图窗中 |
| 关闭窗口 | 点击 X | 仿真线程正常退出，进程结束 |
