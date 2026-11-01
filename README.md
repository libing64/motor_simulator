# 电机本体仿真器（PMSM / BLDC · 开环）

模拟电机的「输入 → 输出」：**输入三相电压（幅值/频率）与负载转矩 → 输出转速、电磁转矩、三相电流、dq 电流、转子角度**。

## 文件

| 文件 | 用途 |
|------|------|
| `motor_simulator.html` | 交互式网页版：浏览器打开即用，滑条实时调输入、转子可视化、滚动波形 |
| `motor_sim.py` | Python 版：`python3 motor_sim.py` 运行，输出 6 宫格波形图 `motor_sim_result.png` |
| `motor_sim_result.png` | Python 默认参数下的仿真结果示例 |

## 模型（两者完全一致）

- dq 旋转坐标系 PMSM 方程（表贴式，Ld=Lq）
- Clarke / Park 变换实现 三相 ⇄ dq
- V/f 软启动（频率/电压从 0 斜坡上升，防止失步）
- 阻尼绕组（异步阻尼转矩，帮助开环牵入同步；设 `D_damp=0` 并加快斜坡可观察失步现象）
- Euler 数值积分（Python dt=2μs，网页 dt=50μs，均已验证与对方结果一致）

## 建议的实验

1. **启动过程**：点「启动」，观察转速沿同步转速加速、牵入同步时的转矩振荡
2. **负载响应**：稳定后拖大「负载转矩」，看转速微降、Te 与 iq 增大（Te = 1.5·p·ψf·iq 的直观验证）
3. **调速**：改「频率 f」，同步转速 = 60·f/p 随之改变
4. **dq 直给**：Python 版把 `INPUT['mode']` 改为 `'dq'`，直接给 vq 恒定电压，观察转矩与 iq 的正比关系
5. **失步现象**：Python 版设 `D_damp=0`、缩短 `start_ramp_time`，观察同步电机开环直启的失步振荡

## 依赖

- 网页版：任何现代浏览器，无依赖
- Python 版：`pip install numpy matplotlib`
