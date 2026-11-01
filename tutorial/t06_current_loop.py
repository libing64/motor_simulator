# -*- coding: utf-8 -*-
"""实验 06 · 电流环：FOC 的内核

前面几课都在讲"应该把电流矢量放在哪"，这一课讲"怎么把它放过去"。
你会看到：

    1. 一个 120 行的手写 FOC（minifoc.py）就能跑闭环，和 SimpleFOC 库几乎重合；
    2. 电流环的带宽由 Kp/L 决定，不是靠试出来的；
    3. 只有 P 会留下稳态误差，必须补 I —— 因为反电动势就是一个持续的"扰动"。
"""

import numpy as np

import tutlib as T
from minifoc import MiniFOC
from sim.bench import simulate
from sim.foc import (CTRL_TORQUE, FocController, MOD_SVPWM, TQ_FOC_CURRENT,
                     ControllerConfig)
from sim.plant import MotorParams, MotorPlant

T.banner('06', '电流环', '—— 把 iq 钉在给定值上')

M = MotorParams()
VBUS = 24.0
W_LOCK = 20.0
DT = 1.0 / 20000.0
LOAD = 0.04
T_WARM = 0.06          # 先让电流稳定在 0.5 A，再开始记录


def step_ref(t, t0=0.05, lo=0.5, hi=2.0):
    return hi if t >= t0 else lo


def run_minifoc(motor, foc, ref_fn, t_end, t_warm=T_WARM):
    """手写 FOC 闭环（转速锁定，专心看电流环；先预热再记录）。"""
    plant = MotorPlant(motor, dt=1.0e-5)

    def tick(ref):
        plant.wm = W_LOCK
        st = foc.step(plant.phase_currents(), plant.theta_e, ref, dt=DT)
        plant.advance(st['ua'], st['ub'], st['uc'], DT, tl=LOAD)
        return st

    for _ in range(int(t_warm / DT)):
        tick(ref_fn(0.0))

    n = int(round(t_end / DT))
    rec = {k: np.zeros(n) for k in ('t', 'ia', 'ib', 'ic', 'iq', 'id', 'ref', 'uq', 'ud')}
    for k in range(n):
        t = k * DT
        st = tick(ref_fn(t))
        rec['t'][k] = t
        rec['ia'][k], rec['ib'][k], rec['ic'][k] = plant.ia, plant.ib, plant.ic
        rec['iq'][k], rec['id'][k] = st['iq'], st['id']
        rec['uq'][k], rec['ud'][k] = st['uq'], st['ud']
        rec['ref'][k] = ref_fn(t)
    return rec


def run_library(motor, cfg, ref_fn, t_end, t_warm=T_WARM):
    """同一台电机跑 SimpleFOC 库（同样先预热，保证起点一致）。"""
    plant = MotorPlant(motor, dt=1.0e-5)
    ctrl = FocController(motor, cfg)
    simulate(motor, cfg, t_warm, target=ref_fn(0.0), load=LOAD, dt_plant=1.0e-5,
             plant=plant, controller=ctrl, warm_start=True, speed_lock=W_LOCK)
    return simulate(motor, cfg, t_end, target=ref_fn, load=LOAD, dt_plant=1.0e-5,
                    plant=plant, controller=ctrl, warm_start=True, speed_lock=W_LOCK)


T.h('先把 FOC 拆成四步')
T.p('  采样 iabc -> Clarke -> Park -> 两个 PI（id 给定 0、iq 给定转矩）')
T.p('  -> 反 Park -> SVPWM 占空比。每一步都只是一次矩阵乘法或一个 PI。')
T.p('  完整实现见 tutorial/minifoc.py，120 行，没有依赖。')

foc = MiniFOC(vbus=VBUS, kp=3.0, ki=300.0)
r_mine = run_minifoc(M, foc, step_ref, t_end=0.12)

cfg = ControllerConfig(controller=CTRL_TORQUE, torque_mode=TQ_FOC_CURRENT,
                       modulation=MOD_SVPWM, current_limit=4.0,
                       pid_current_q=(3.0, 300.0, 0.0), pid_current_d=(3.0, 300.0, 0.0),
                       voltage_limit=VBUS)
r_lib = run_library(M, cfg, step_ref, t_end=0.12)

T.h('两种实现对比（同一台电机、同一组增益 Kp=3, Ki=300）')
err = r_mine['iq'] - r_lib['iq']
print('  iq 轨迹最大偏差 %.3f A，RMS 偏差 %.3f A（阶跃 0.5 -> 2.0 A）'
      % (np.abs(err).max(), np.sqrt(np.mean(err ** 2))))
print('  手写：稳态 iq = %.3f A' % np.mean(r_mine['iq'][-400:]))
print('  库  ：稳态 iq = %.3f A' % np.mean(r_lib['iq'][-400:]))
i_pk = float(np.max(r_mine['iq']))
t_63 = r_mine['t'][np.argmax(r_mine['iq'] > 0.5 + 0.632 * 1.5)]
t_90 = r_mine['t'][np.argmax(r_mine['iq'] > 1.85)]
print('  63%% 上升 %.2f ms（P 主导），90%% 上升 %.1f ms（后段靠积分慢慢补）'
      % ((t_63 - 0.05) * 1e3, (t_90 - 0.05) * 1e3))
print('  稳态无超调（峰值 %.3f A）——Kp/Ki 不够大时就是这种"前快后慢"。' % i_pk)

T.h('带宽不是试出来的：Kp 决定它')
print('  电流环近似一阶惯性：L·di/dt = Kp·(i* − i)  ->  时间常数 τ = L/Kp')
taus = {}
for kp in (1.0, 3.0, 10.0):
    f = MiniFOC(vbus=VBUS, kp=kp, ki=300.0)
    r = run_minifoc(M, f, step_ref, t_end=0.09)
    taus[kp] = (r['t'][np.argmax(r['iq'] > 0.5 + 0.632 * 1.5)] - 0.05) * 1e3
    print('  Kp=%4.1f  ->  理论 τ=L/Kp=%.2f ms，实测 63%% 上升 %.2f ms'
          % (kp, M.L / kp * 1e3, taus[kp]))
T.note('Kp 再大就撞上 PWM 频率和采样延迟：控制周期 50 µs 时，'
       '电流环带宽做到 1~2 kHz 就到头了。')

T.h('为什么必须有 I')
f_p = MiniFOC(vbus=VBUS, kp=3.0, ki=0.0)
r_p = run_minifoc(M, f_p, lambda t: 2.0, t_end=0.12, t_warm=0.0)
print('  只有 P（Ki=0）：稳态 iq = %.3f A，给定 2.000 A，差 %.3f A'
      % (np.mean(r_p['iq'][-400:]), 2.0 - np.mean(r_p['iq'][-400:])))
print('  加上 I（Ki=300）：稳态 iq = %.3f A，误差 %.4f A'
      % (np.mean(r_mine['iq'][-400:]), 2.0 - np.mean(r_mine['iq'][-400:])))
print('  原因：反电动势 ωe·ψf 是个持续存在的"反向电压"，P 必须留一点误差才能顶住它。')

# ---------------------------------------------------------------- 画图
f, ax = T.fig(2, 2, suptitle='实验 06：电流环')

ax[0].plot(r_mine['t'] * 1e3, r_mine['ref'], 'k--', lw=1.0, label='iq 给定')
ax[0].plot(r_mine['t'] * 1e3, r_mine['iq'], color=T.C['red'], lw=1.8, label='手写 FOC')
ax[0].plot(r_lib['t'] * 1e3, r_lib['iq'], color=T.C['blue'], lw=1.2, ls=':',
           label='SimpleFOC 库')
T.spec(ax[0], '(a) iq 阶跃 0.5→2 A（转速锁定、已预热）', '时间 [ms]', '电流 [A]')

ax[1].plot(r_mine['t'] * 1e3, r_mine['id'], color=T.C['green'], lw=1.4,
           label='id（应保持 0）')
ax[1].plot(r_mine['t'] * 1e3, r_mine['iq'] - r_mine['ref'], color=T.C['gray'], lw=1.2,
           label='iq 跟踪误差')
ax[1].axhline(0, color='k', lw=0.5)
ax[1].set_ylim(-1.8, 0.35)
T.spec(ax[1], '(b) d 轴几乎无串扰；误差在 1 ms 内收回', '时间 [ms]', '电流 [A]')

for kp, col in zip((1.0, 3.0, 10.0), (T.C['gray'], T.C['orange'], T.C['red'])):
    fm = MiniFOC(vbus=VBUS, kp=kp, ki=300.0)
    r = run_minifoc(M, fm, step_ref, t_end=0.09)
    ax[2].plot(r['t'] * 1e3, r['iq'], color=col, lw=1.5,
               label='Kp=%.0f（实测 τ≈%.2f ms）' % (kp, taus[kp]))
ax[2].plot([50, 50], [0.45, 2.05], 'k:', lw=0.8)
ax[2].set_xlim(45, 90)
T.spec(ax[2], '(c) Kp 越大响应越快（τ = L/Kp）', '时间 [ms]', '电流 [A]')

ax[3].plot(r_p['t'] * 1e3, r_p['iq'], color=T.C['orange'], lw=1.5, label='只有 P（Ki=0）')
ax[3].plot(r_mine['t'] * 1e3, r_mine['iq'], color=T.C['red'], lw=1.5, label='PI（Ki=300）')
ax[3].axhline(2.0, color='k', ls='--', lw=1.0, label='给定 2 A')
ax[3].annotate('稳态误差 %.2f A' % (2.0 - np.mean(r_p['iq'][-400:])),
               (75, float(np.mean(r_p['iq'][-400:]))), xytext=(78, 1.45),
               fontsize=8, color=T.C['orange'],
               arrowprops=dict(arrowstyle='->', color=T.C['orange'], lw=0.8))
T.spec(ax[3], '(d) 只有 P 会留下稳态误差：反电动势是"常驻扰动"',
       '时间 [ms]', '电流 [A]')

path = T.save(f, 't06_current_loop.png')

T.h('小结')
T.bullet('电流环 = Clarke + Park + 2×PI + 反 Park + SVPWM，一页纸就能写完。',
         '它唯一的外部依赖是转子电角度 θe；θe 错了，id/iq 就分不开。',
         '带宽 ≈ Kp/L，受采样频率和 PWM 频率限制；I 用来抵消反电动势这个常驻扰动。',
         '电流环跑通之后，转矩就变成了"给 iq 就行"——速度环、位置环都建在它上面。')

T.questions('把 Kp 加到 50 会怎样？（提示：采样周期 50 µs，相位滞后 180° 附近就发散）',
            '如果电感 L 实际比标称值大 50%，Kp 不变，闭环会变成过阻尼还是欠阻尼？',
            '为什么 id 的 PI 和 iq 的 PI 可以独立设计？（提示：表贴式 Ld=Lq，'
            '交叉耦合项在低速时很小）')

T.done(path)
