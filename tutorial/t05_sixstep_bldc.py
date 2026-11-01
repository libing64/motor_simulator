# -*- coding: utf-8 -*-
"""实验 05 · 六步换相（BLDC 的经典驱动）

在 FOC 之前，BLDC 就是这么驱动的：每个电周期分 6 个扇区，每个扇区只让两相导通，
第三相悬空。逻辑极其简单——但代价是转矩脉动。

本课手写一段换相表（不调用任何库），直接作用在相域电机模型上，
量化"方波驱动"到底差在哪。
"""

import math

import numpy as np

import tutlib as T
from sim.bench import simulate, thd
from sim.foc import (CTRL_TORQUE, MOD_SINE, TQ_FOC_CURRENT, ControllerConfig)
from sim.plant import (MotorParams, MotorPlant, emf_sine, emf_trapezoid)

T.banner('05', '六步换相（BLDC）', '—— 一次只通两相，简单但抖')

# 六个电压矢量的角度（αβ 平面）与导通相
_VEC = [('A', 'B'), ('A', 'C'), ('B', 'C'), ('B', 'A'), ('C', 'A'), ('C', 'B')]
_VEC_ANGLE = [-30.0, 30.0, 90.0, 150.0, 210.0, 270.0]
_PHASE = ('A', 'B', 'C')
_THIRD = 2.0 * math.pi / 3.0


def sixstep_sector(theta_e: float):
    """转子电角度 -> (扇区号, 导通相高/低)。"""
    idx = int(round((math.degrees(theta_e) + 120.0) / 60.0)) % 6
    return idx, _VEC[idx]


def run_sixstep(motor: MotorParams, vbus: float, duty: float, t_end: float,
                dt_ctrl: float = 5.0e-5, dt_plant: float = 1.0e-5,
                speed_lock=None, load: float = 0.0):
    """手写六步换相，只用转子角度查表，没有任何电流/速度环。"""
    emf = emf_sine if motor.waveform == 'sine' else emf_trapezoid
    plant = MotorPlant(motor, dt=dt_plant)
    n = int(round(t_end / dt_ctrl))
    rec = {k: np.zeros(n) for k in ('t', 'ia', 'ib', 'ic', 'te', 'wm', 'sector',
                                    'ua', 'ub', 'uc', 'duty', 'theta_e')}
    v = vbus * duty / 2.0
    for k in range(n):
        if speed_lock is not None:
            plant.wm = speed_lock
        th = plant.theta_e
        idx, (hi, lo) = sixstep_sector(th)
        ih, il = _PHASE.index(hi), _PHASE.index(lo)
        out = [0.0, 0.0, 0.0]
        out[ih] = +v
        out[il] = -v

        # 第三相是**悬空**的：它不导通，端电压被反电动势"顶"到 1.5·e（这是无感
        # BLDC 靠过零点换相的理论依据）。若直接给它 0 V，等于把该相短路，模型会
        # 凭空多出一条环流。
        f = 3 - ih - il
        out[f] = 1.5 * motor.psi_f * plant.omega_e * emf(th - f * _THIRD)

        plant.advance(out[0], out[1], out[2], dt_ctrl, tl=load)
        if speed_lock is not None:
            plant.wm = speed_lock

        rec['t'][k] = k * dt_ctrl
        rec['ia'][k], rec['ib'][k], rec['ic'][k] = plant.ia, plant.ib, plant.ic
        rec['te'][k] = plant.t_e
        rec['wm'][k] = plant.wm
        rec['sector'][k] = idx
        rec['ua'][k], rec['ub'][k], rec['uc'][k] = out
        rec['duty'][k] = duty
        rec['theta_e'][k] = th
    return rec


T.h('换相表就这六行')
for i, (hi, lo) in enumerate(_VEC):
    print('  扇区 %d（矢量 %+4.0f°）：%s 相接 +Vbus/2，%s 相接 −Vbus/2，第三相悬空'
          % (i, _VEC_ANGLE[i], hi, lo))
T.p('')
T.p('  为什么矢量要超前 90°？转矩 ∝ iq，而 q 轴比 d 轴（转子磁极方向）超前 90°。')
T.p('  六步换相的本质：用 6 个**离散**的电压矢量去逼近一个连续旋转的 q 轴矢量。')
T.p('  角度误差最大 30°，所以转矩只能大致恒定——这是原理性的，不是实现不好。')

# ---------------------------------------------------------------- 运行
VBUS = 24.0
DUTY = 0.60
W_LOCK = 60.0
MOTOR = MotorParams()                  # 正弦机，与后面 FOC 对照保持同一台
r6 = run_sixstep(MOTOR, VBUS, DUTY, t_end=0.4, speed_lock=W_LOCK, load=0.05)

fe = W_LOCK * MOTOR.p / (2.0 * math.pi)
m = r6['t'] > 0.2
te6 = r6['te'][m]
ripple6 = (te6.max() - te6.min()) / te6.mean()
thd6 = thd(r6['ia'][m], fe, 1.0 / 5.0e-5)

T.h('六步换相实测（转速锁定 %.0f rad/s，PWM 占空比 %.0f%%）' % (W_LOCK, DUTY * 100))
print('  平均转矩 %.3f N·m，脉动 %.1f%%，相电流 THD %.0f%%'
      % (te6.mean(), ripple6 * 100, thd6 * 100))
# 悬空相没有外加电压，换相后残留的电流只能靠电阻衰减
frac = ((np.degrees(r6['theta_e'][m]) + 120.0) % 60.0) / 60.0
abs_i = np.abs(np.vstack([r6['ia'][m], r6['ib'][m], r6['ic'][m]]))
tau = MOTOR.L / MOTOR.Rs
print('  相电流峰值 %.2f A；悬空相在换相后仍有残流（时间常数 L/R = %.1f ms）'
      % (abs_i.max(), tau * 1e3))
print('  悬空相的端电压被反电动势"顶"到 1.5·e，所以它不输出电流；')
print('  但换相瞬间被甩开的那一相，电流要按 L/R 慢慢衰减——真实驱动器用二极管')
print('  把它钳到母线上，衰减快得多。这段拖尾就是转矩凹口的来源。')

# ---------------------------------------------------------------- 对照组
iq_eq = te6.mean() / MOTOR.kt
cfg = ControllerConfig(controller=CTRL_TORQUE, torque_mode=TQ_FOC_CURRENT,
                       modulation=MOD_SINE, current_limit=3.0)
r_foc = simulate(MOTOR, cfg, t_end=0.4, target=float(iq_eq), load=0.05,
                 dt_plant=1.0e-5, speed_lock=W_LOCK)
mf = r_foc['t'] > 0.2
tef = r_foc['te'][mf]
ripple_f = (tef.max() - tef.min()) / tef.mean()
thd_f = thd(r_foc['ia'][mf], fe, 1.0 / 5.0e-5)

T.h('同一台电机、同样平均转矩，换成 FOC')
print('  平均转矩 %.3f N·m，脉动 %.2f%%，相电流 THD %.1f%%'
      % (tef.mean(), ripple_f * 100, thd_f * 100))
ratio_r = ripple6 / max(ripple_f, 1e-9)
print('  -> 转矩脉动降到 1/%.0f%s，电流谐波降到 1/%.0f'
      % (min(ratio_r, 999), ' 以上' if ratio_r > 999 else '',
         thd6 / max(thd_f, 1e-9)))

# ---------------------------------------------------------------- 画图
f, ax = T.fig(2, 2, suptitle='实验 05：六步换相 vs FOC')

k0 = int(0.3 / 5.0e-5)
n_win = int(round(1.0 / fe / 5.0e-5))          # 一个电周期
n_win = min(n_win, len(r6['t']) - k0)
sl = slice(k0, k0 + n_win)
deg = np.linspace(0, 360, n_win)

for idx, (col, name) in enumerate(zip(('red', 'green', 'blue'), ('A', 'B', 'C'))):
    ax[0].plot(deg, r6[('ua', 'ub', 'uc')[idx]][sl], color=T.C[col], lw=1.6,
               label='%s 相端电压' % name)
for s in range(6):
    ax[0].axvspan(s * 60, (s + 1) * 60, color=T.C['gray'], alpha=0.10 if s % 2 else 0.03)
    ax[0].text(s * 60 + 30, 8.2, 'S%d' % s, ha='center', fontsize=7, color=T.C['gray'])
ax[0].set_xlim(0, 360)
ax[0].set_ylim(-9.5, 9.5)
T.spec(ax[0], '(a) 一个电周期：两相导通、一相悬空（电压被 BEMF 顶到 1.5e）',
       '电角度 [°]', '相电压 [V]')

for idx, (col, name) in enumerate(zip(('red', 'green', 'blue'), ('A', 'B', 'C'))):
    ax[1].plot(deg, r6[('ia', 'ib', 'ic')[idx]][sl], color=T.C[col], lw=1.2,
               label='%s 相' % name)
ax[1].axhline(0, color='k', lw=0.5)
ax[1].set_xlim(0, 360)
T.spec(ax[1], '(b) 相电流：方波，每 60° 换相一次', '电角度 [°]', '电流 [A]')

tef_win = tef[:n_win]
ax[2].plot(deg, te6[:n_win], color=T.C['red'], lw=1.2,
           label='六步：%.1f%% 脉动' % (ripple6 * 100))
ax[2].plot(deg, tef_win, color=T.C['blue'], lw=1.4,
           label='FOC：%.2f%% 脉动' % (ripple_f * 100))
ax[2].set_xlim(0, 360)
T.spec(ax[2], '(c) 电磁转矩（转速锁定，只看纯转矩纹波）', '电角度 [°]', '转矩 [N·m]')

th = np.linspace(0.0, 2.0 * math.pi, 721)
e_s = np.array([emf_sine(x) for x in th])
e_t = np.array([emf_trapezoid(x) for x in th])
ax[3].plot(np.degrees(th), e_s - np.array([emf_sine(x - _THIRD) for x in th]),
           color=T.C['blue'], lw=1.6, label='正弦机 线反电动势 eA−eB')
ax[3].plot(np.degrees(th), e_t - np.array([emf_trapezoid(x - _THIRD) for x in th]),
           color=T.C['orange'], lw=1.6, label='梯形机 线反电动势 eA−eB')
ax[3].axhline(0, color='k', lw=0.5)
ax[3].set_xlim(0, 360)
T.spec(ax[3], '(d) 梯形机的线反电动势有 60° 平顶 —— 这才是方波驱动的"原配"',
       '电角度 [°]', '归一化')

path = T.save(f, 't05_sixstep_bldc.png')

T.h('六步换相的功与过')
T.bullet('优点：只要 3 个霍尔开关（甚至无感），逻辑几十行，MCU 占用极低。',
         '缺点一：电压矢量只有 6 个方向，角度误差最大 30°，转矩必须脉动。',
         '缺点二：方波电流谐波大（本例 10%%，满占空比时更高），铁耗、噪声、振动更差。',
         '缺点三：换相瞬间电流突变，需要斩波限流，否则冲击电流很大。')
T.p('  FOC 用连续旋转的矢量代替 6 个离散矢量，角度误差趋近 0——')
T.p('  这就是它值得多花一个编码器的原因。')

T.note('模型里 PWM 按平均电压处理，所以这里的纹波只来自"矢量离散化"和换相过程；'
       '真实系统还有开关纹波、死区、采样延迟，纹波只会更大。')

T.questions('六步换相的角度误差最大 30°，理论转矩脉动是多少？（提示：cos30°≈0.866）',
            '为什么 BLDC 用"梯形反电动势 + 方波电流"，而 PMSM 用"正弦 + 正弦"？'
            '两者在单位铜耗出多少转矩上谁更划算？',
            '悬空相端电压 = 1.5×反电动势。如果只有这一路电压可以测，'
            '能推断出转子角度吗？精度受什么限制？')

T.done(path)
