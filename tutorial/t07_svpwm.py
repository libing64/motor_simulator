# -*- coding: utf-8 -*-
"""实验 07 · 调制：正弦 PWM vs SVPWM

电流环算出的是 uα/uβ（一个旋转的电压矢量），最后一步是把它变成三个占空比。
这一步的选择直接决定"母线电压能用掉多少"——也就是同样 24 V 电源能跑多快。

本课先用公式对比两种调制，再用闭环实测最高转速验证。
"""

import math

import numpy as np

import tutlib as T
from sim.bench import simulate, steady_state
from sim.foc import (CTRL_VELOCITY, MOD_SINE, MOD_SVPWM, TQ_FOC_CURRENT,
                     ControllerConfig)
from sim.plant import MotorParams

T.banner('07', '调制方式', '—— 同样的母线电压，怎么多用 15%')

M = MotorParams()
VBUS = 24.0
SQRT3 = math.sqrt(3.0)


def phase_duty(ualpha, ubeta, vbus, svpwm: bool):
    """电压矢量 -> 三相占空比（svpwm=True 时注入零序）。"""
    va = ualpha
    vb = -0.5 * ualpha + (SQRT3 / 2.0) * ubeta
    vc = -0.5 * ualpha - (SQRT3 / 2.0) * ubeta
    off = -0.5 * (max(va, vb, vc) + min(va, vb, vc)) if svpwm else 0.0
    return np.array([0.5 + (v + off) / vbus for v in (va, vb, vc)])


T.h('问题：一个旋转的电压矢量，怎么变成占空比')
T.p('  三相桥臂只能输出 0 或 Vbus，PWM 取平均后等效为 0~Vbus 的任意电压。')
T.p('  所以"矢量的长度"上限就是母线能给的最大电压——问题是怎么给得最划算。')

# ---------------------------------------------------------------- 一个电周期
th = np.linspace(0.0, 2.0 * math.pi, 721)
m_index = 1.0                                  # 调制比：|u| / (Vbus/2)
u_mag = m_index * VBUS / 2.0
ua_v = -u_mag * np.sin(th)                     # q 轴方向上的矢量
ub_v = u_mag * np.cos(th)

d_sine = np.array([phase_duty(a, b, VBUS, False) for a, b in zip(ua_v, ub_v)])
d_svpwm = np.array([phase_duty(a, b, VBUS, True) for a, b in zip(ua_v, ub_v)])

T.h('调制比 m = 1.0（矢量长度 = Vbus/2 = %.1f V）' % (VBUS / 2))
print('  正弦 PWM：占空比范围 %.3f ~ %.3f（刚好贴住 0 和 1，再大就削顶）'
      % (d_sine.min(), d_sine.max()))
print('  SVPWM  ：占空比范围 %.3f ~ %.3f（两端各留 %.1f%% 裕量）'
      % (d_svpwm.min(), d_svpwm.max(), d_svpwm.min() * 100))

# ---------------------------------------------------------------- 调制极限
m_sweep = np.linspace(0.0, 1.25, 251)
lim_sine, lim_sv = [], []
for m in m_sweep:
    mag = m * VBUS / 2.0
    ua = -mag * np.sin(th)
    ub = mag * np.cos(th)
    lim_sine.append(np.array([phase_duty(a, b, VBUS, False) for a, b in zip(ua, ub)]).max())
    lim_sv.append(np.array([phase_duty(a, b, VBUS, True) for a, b in zip(ua, ub)]).max())

m_sine_max = 1.0
m_sv_max = 2.0 / SQRT3
T.h('两种调制的线性区边界')
print('  正弦 PWM：占空比在 m = %.3f 时贴顶 -> 最大 |u| = %.2f V'
      % (m_sine_max, m_sine_max * VBUS / 2))
print('  SVPWM  ：占空比在 m = %.3f 时贴顶 -> 最大 |u| = %.2f V'
      % (m_sv_max, m_sv_max * VBUS / 2))
print('  电压利用率提升 %.1f%%（= 2/√3 − 1），转速上限同理提高 %.1f%%'
      % ((2 / SQRT3 - 1) * 100, (2 / SQRT3 - 1) * 100))

# ---------------------------------------------------------------- 实测
def max_clean_speed(module: int, lo=84.0, hi=124.0, step=4.0) -> float:
    """转速指令逐步抬高，找到"末段占空比仍未削顶"的最高稳态转速。"""
    best = 0.0
    for tgt in np.arange(lo, hi + 1e-9, step):
        cfg = ControllerConfig(controller=CTRL_VELOCITY, torque_mode=TQ_FOC_CURRENT,
                               modulation=module, velocity_limit=400.0, current_limit=3.0)
        rec = simulate(M, cfg, 1.2, target=float(tgt), load=0.02, dt_plant=5.0e-5)
        tail = slice(-300, None)                       # 只末段 15 ms，避开启动瞬态
        clipped = max(float(np.max(rec[k][tail])) for k in ('dc_a', 'dc_b', 'dc_c')) > 0.995
        if clipped:
            break
        best = steady_state(rec, 'wm', -0.02)
    return best


T.h('闭环实测：把转速往上顶，看谁先削顶')
w_sine = max_clean_speed(MOD_SINE)
w_svpwm = max_clean_speed(MOD_SVPWM)
print('  正弦 PWM：%.1f rad/s（%.0f rpm）' % (w_sine, w_sine * 60 / (2 * math.pi)))
print('  SVPWM  ：%.1f rad/s（%.0f rpm）' % (w_svpwm, w_svpwm * 60 / (2 * math.pi)))
print('  提升 %.1f%%（理论 %.1f%%）'
      % (100 * (w_svpwm / max(w_sine, 1e-9) - 1), 100 * (2 / SQRT3 - 1)))
print('  理论值 Vbus/2/ke = %.1f rad/s，Vbus/√3/ke = %.1f rad/s'
      % (VBUS / 2 / M.ke, VBUS / SQRT3 / M.ke))

# ---------------------------------------------------------------- 画图
f, ax = T.fig(2, 2, suptitle='实验 07：正弦 PWM vs SVPWM')

ax[0].plot(np.degrees(th), d_sine[:, 0], color=T.C['blue'], lw=1.8, label='正弦 PWM')
ax[0].plot(np.degrees(th), d_svpwm[:, 0], color=T.C['red'], lw=1.8, label='SVPWM')
ax[0].axhline(0, color='k', lw=0.6)
ax[0].axhline(1, color='k', lw=0.6)
ax[0].set_xlim(0, 360)
ax[0].set_ylim(-0.08, 1.08)
T.spec(ax[0], '(a) A 相占空比：SVPWM 是"马鞍形"（注入了三次谐波）',
       '电角度 [°]', '占空比')

ax[1].plot(m_sweep, lim_sine, color=T.C['blue'], lw=1.8, label='正弦 PWM')
ax[1].plot(m_sweep, lim_sv, color=T.C['red'], lw=1.8, label='SVPWM')
ax[1].axhline(1.0, color='k', ls=':', lw=1.0)
ax[1].axvline(m_sine_max, color=T.C['blue'], ls=':', lw=1.0)
ax[1].axvline(m_sv_max, color=T.C['red'], ls=':', lw=1.0)
ax[1].set_xlim(0, 1.25)
ax[1].set_ylim(0.4, 1.15)
ax[1].annotate('正弦上限 m=1.00', (m_sine_max, 0.45), fontsize=8, color=T.C['blue'],
               rotation=90, va='bottom', ha='right')
ax[1].annotate('SVPWM 上限 m=1.155', (m_sv_max, 0.45), fontsize=8, color=T.C['red'],
               rotation=90, va='bottom', ha='right')
T.spec(ax[1], '(b) 调制比超过线性区就削顶（波形失真、电流畸变）',
       '调制比 m = |u| / (Vbus/2)', '一个周期内最大占空比')

labels = ['正弦 PWM', 'SVPWM']
vals = [VBUS / 2, VBUS / SQRT3]
bars = ax[2].bar(labels, vals, color=[T.C['blue'], T.C['red']], width=0.55)
for b, v in zip(bars, vals):
    ax[2].text(b.get_x() + b.get_width() / 2, v + 0.2, '%.2f V' % v,
               ha='center', fontsize=9)
ax[2].set_ylim(0, VBUS / SQRT3 * 1.25)
ax[2].set_title('(c) 线性区可用相电压峰值（母线 %.0f V）' % VBUS, fontsize=10)
ax[2].set_ylabel('电压 [V]')
ax[2].text(0.5, VBUS / SQRT3 * 1.1, '+%.1f%%' % ((2 / SQRT3 - 1) * 100),
           ha='center', color=T.C['green'], fontsize=11)

bars = ax[3].bar(labels, [w_sine, w_svpwm], color=[T.C['blue'], T.C['red']], width=0.55)
for b, v in zip(bars, [w_sine, w_svpwm]):
    ax[3].text(b.get_x() + b.get_width() / 2, v + 1.5, '%.1f rad/s' % v,
               ha='center', fontsize=9)
ax[3].set_ylim(0, max(w_sine, w_svpwm) * 1.25)
ax[3].set_title('(d) 实测最高转速（占空比不削顶）', fontsize=10)
ax[3].set_ylabel('转速 [rad/s]')

path = T.save(f, 't07_svpwm.png')

T.h('为什么多出来的 15% 是"白送"的')
T.bullet('三相三线系统里，三个相电压同时加同一个常数，线电压不变、电流不变。',
         'SVPWM 就利用这个自由度：把 (max+min)/2 抬到 0，让三相占空比整体平移。',
         '平移后峰值被"压扁"，同样的矢量长度就不容易撞到 0/1，于是线性区扩到 2/√3。',
         '代价：相电压对地出现三次谐波——但三相星形没有中性线，它不产生电流。')

T.note('市面上很多电调默认用 SVPWM，不只是为了 15% 的电压利用率，'
       '还因为它开关次数少约 1/3，开关损耗更低。')

T.questions('如果电机是三角形接法，SVPWM 注入的零序还会"看不见"吗？',
            '过调制（m > 1.155）能再挤出一点电压，代价是什么？什么场合值得用？',
            '为什么正弦 PWM 在 m=1 时占空比刚好是 0 和 1？如果母线是 48 V，这个结论变吗？')

T.done(path)
