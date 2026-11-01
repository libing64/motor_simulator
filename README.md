# 电机仿真器（PMSM / BLDC）+ SimpleFOC 算法库闭环测试

两级仿真环境：

| 层级 | 内容 | 入口 |
|------|------|------|
| **本体级（开环）** | 只模拟电机：给三相电压 → 出转速/电流/转矩 | `motor_sim.py`、`motor_simulator.html` |
| **算法级（闭环）** | 电机模型 + **SimpleFOC 算法库**闭环：测 FOC/六步/调制/PID | `run_foc_test.py`、`tests/` |

算法库以 **git submodule** 引入 `third_party/Arduino-FOC`（SimpleFOC **v2.4.0**），
在 PC 上原生编译，通过 ctypes 每个控制周期调用一次 `loopFOC()` + `move()`，
形成「仿真电机 ↔ 真实库代码」的硬件在环式测试台。

## 目录

```
motor_simulator/
├── motor_sim.py                 # 原开环仿真（dq 模型，出 6 宫格波形）
├── motor_simulator.html         # 网页交互版（开环）
├── run_foc_test.py              # ★ 一键：编译 + 全部测试 + 出图/报告
├── third_party/
│   └── Arduino-FOC/             # git submodule，SimpleFOC v2.4.0（未修改）
├── sim/
│   ├── plant.py                 # 三相相域电机模型（正弦/梯形反电动势，RK4）
│   ├── foc.py                   # SimpleFOC 的 ctypes 封装
│   ├── bench.py                 # 闭环测试台（含测功机模式、开环 V/f 驱动）
│   ├── build.py                 # 编译桥接动态库
│   ├── plotstyle.py             # 中文字体/样式
│   ├── native/
│   │   ├── foc_sim.h/.cpp       # ★ C ABI 桥接层
│   │   └── libfocsim.so         # 编译产物（不入库）
│   └── platform/                # ★ Arduino 原生 shim（Arduino.h + 虚拟时钟）
└── tests/
    ├── test_suite.py            # 22 个用例
    ├── harness.py               # 极简测试框架
    └── run_tests.py             # 只跑测试
```

## 快速开始

```bash
pip install numpy matplotlib

python3 run_foc_test.py     # 编译 + 22 项测试 + foc_test_report.png + 报告
python3 tests/run_tests.py  # 只跑测试
python3 motor_sim.py        # 原开环仿真
```

输出物：`foc_test_report.png`（9 宫格结果图）、`foc_test_compare.png`（指标柱状图）、
`foc_test_summary.md`（PASS/FAIL 明细表）。

## 算法库是怎么"接进来"的

### 1. Arduino shim（`sim/platform/`）

SimpleFOC 的控制器代码只依赖极少数 Arduino API。shim 提供 `Arduino.h`（`Print`、
`micros`、`constrain`、`pinMode` 等）并把时间源换成**虚拟时钟**：

- `micros()/millis()` 由测试台在每个控制周期显式推进 → PID 的 `Ts`、速度估计完全确定、可复现；
- `delay()` 不阻塞，只推进虚拟时间（`motor.init()` 里两次 500 ms 启动延时瞬间完成）；
- GPIO/ADC 是空桩，因为桥接层绕过了所有 MCU 相关驱动。

只编译算法相关源文件（`BLDCMotor`、`FOCMotor`、`PID`、`LowPassFilter`、`foc_utils`、
`Sensor`、`CurrentSense`），不含任何 MCU 驱动/通信前端。

### 2. 桥接层（`sim/native/foc_sim.cpp`）

用**子类化**而非修改库的方式替换硬件：

| SimpleFOC 抽象类 | 桥接实现 | 作用 |
|---|---|---|
| `BLDCDriver` | `SimDriver` | 接收 `Ua/Ub/Uc`，按 `BLDCDriver6PWM` 的规则换算成占空比 |
| `Sensor` | `SimSensor` | 注入转子机械角与转速（可加量化/噪声） |
| `CurrentSense` | `SimCurrentSense` | 注入三相电流（可加噪声） |

于是 `setPhaseVoltage()`（含正弦/SVPWM 中点钳位/六步换相）、dq 电流 PI、速度/位置级联、
电压电流限幅、抗积分饱和，全部是**库里的原版代码**。

传感器对齐（`initFOC()`）跳过：桥接层直接写入 `zero_electric_angle` 与 `sensor_direction`，
相当于一台已标定编码器的实验台。

> 踩坑记录：`FOCMotor` 有若干 POD 成员（`feed_forward_voltage` 等）没有默认初始化器，
> 直接 `new` 会读到堆上的垃圾值（表现为 uq = 7e31）。桥接层改为在 `calloc` 的零内存上
> placement-new，保证了确定性。

### 3. 每个控制周期的时序（`sim/bench.py`）

```
采样 ia/ib/ic、θm、ωm
      ↓
motor.move(target)      # 速度/位置环、目标限幅
motor.loopFOC()         # 电流环 → 调制 → 占空比
      ↓
Uabc = duty × Vbus  →  电机模型 RK4 积分 1 个控制周期
```

## 22 个测试用例

**变换与调制**（与 numpy 参考实现/解析公式逐点比对）

- Clarke/Park 往返、正弦 PWM 相电压、SVPWM 相电压（中点钳位）→ 误差 < 2 mV
- 占空比 = 相电压/母线；SVPWM 线性区比正弦高 √3/2（11.99 V vs 13.85 V，+15.5%）
- 零序注入：同为 12 V 时正弦贴住 0/1，SVPWM 两端各留 6.7% 裕量
- 六步换相：6 个扇区、两相导通差值 = 2·Uq/Vbus、悬空相依次覆盖 ABC

**闭环控制**

- 转矩模式 `Te = 1.5·p·ψf·iq`：实测 kt = 0.1800 N·m/A（理论 0.180）
- 正反向电流跟踪误差 2.3%；电流限幅（目标 8 A / 限幅 3 A → 峰值 3.19 A）
- 速度环 30 rad/s 阶跃：无静差（0.01%）、超调 15%、0.17 s 进入 ±5%
- 速度反转 ±30 rad/s、位置环定位 π rad（误差 1.1 mrad）
- 负载阶跃 0.03→0.12 N·m：转速跌落 1.4 rad/s 后 PI 拉回

**调制方式对比（转速锁定测功机）**

| 调制 | 电流 THD | 转矩纹波 | 可调最高转速 |
|------|---------|---------|-------------|
| 正弦 PWM | 6.4% | ~0% | 96 rad/s |
| SVPWM | 6.4% | ~0% | 112 rad/s（+16.7%）|
| 六步 120° | 51.8% | 13.8% | — |

**鲁棒性与物理一致性**

- 12 bit 编码器 + 20 mA 电流噪声 + 2 mrad 角度噪声：速度仍无静差
- **开环 V/f（0.02 s 斜坡，无阻尼绕组）失步**，闭环 FOC 稳定 449 rpm（同步转速）
- 梯形反电动势电机 + 六步换相（BLDC 模式）闭环正常
- 积分步长 10 µs vs 5 µs：结果差 < 0.01%
- 相域模型稳态 dq 电流与解析解一致

## 模型与假设（重要）

- **相域 PMSM/BLDC**：`L·dik/dt = vk − vn − R·ik − ψf·ωe·e(θk)`，`Te = p·ψf·Σ ik·e(θk)`；
  `e(θ)` 可在正弦（`-sinθ`）与梯形（平顶 120°，磁链峰值归一为 1）之间切换。
- **PWM 取平均电压**：一个控制周期内施加恒定相电压，因此**不包含开关纹波**——
  正弦 FOC 的转矩纹波因此接近 0，这属于模型边界，不是算法结论。
- 表贴式（Ld = Lq），无磁饱和、无死区、无逆变器压降、无齿槽转矩；中性点按
  `vn = (va+vb+vc)/3` 处理。
- 控制周期 20 kHz、电机积分步长 10 µs（收敛性已测）。

## 下一步可以做的

1. 加入 PWM 开关级模型（三角载波 + 死区 + 管压降），观察开关纹波与死区畸变
2. 无感 FOC：把反电动势观测器/滑模观测器接进同一测试台
3. MTPA / 弱磁（`id < 0`）在 SVPWM 电压饱和下的表现
4. 把 FOC 模式搬进 `motor_simulator.html`，网页上直接切 V/f / 六步 / FOC

## 原开环仿真器（保留）

`motor_sim.py`：dq 坐标系 PMSM 方程 + Clarke/Park + V/f 软启动 + 阻尼绕组，
`python3 motor_sim.py` 输出 `motor_sim_result.png`。
`motor_simulator.html`：浏览器打开即用，滑条实时调幅值/频率/负载，带转子可视化。
