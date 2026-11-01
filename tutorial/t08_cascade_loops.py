# -*- coding: utf-8 -*-
"""实验 08 · 速度环与位置环：级联控制

电流环只是"内环"。真正的伺服系统是三层：

    位置 P  ->  速度 PI  ->  电流 PI  ->  SVPWM

本课看三件事：速度环怎么调、负载来了掉多少、位置环怎么做到"停在哪就是哪"。
"""

import math

import numpy as np

import tutlib as T
from sim.bench import simulate
from sim.foc import (CTRL_ANGLE, CTRL_VELOCITY, MOD_SVPWM, TQ_FOC_CURRENT,
                     ControllerConfig)
from sim.plant import MotorParams

T.banner('08', '速度环与位置环', '—— 一层套一层，带宽一级比一级低')

M = MotorParams()


def run(cfg, t_end, target, load=0.02, dt_plant=2.0e-5):
    return simulate(M, cfg, t_end, target=target, load=load, dt_plant=dt_plant)


T.h('级联的三层')
T.p('  位置环（P）：我要转到 θ*，差多少就让速度环跑多快。')
T.p('  速度环（PI）：我要 ω*，差多少就让电流环给多大 iq。')
T.p('  电流环（PI）：我要 iq*，差多少就给多大电压。')
T.p('  带宽必须一级比一级高（通常 5~10 倍），否则外环会"催"坏内环，表现为振荡。')

# ---------------------------------------------------------------- 速度阶跃
cfg_v = ControllerConfig(controller=CTRL_VELOCITY, torque_mode=TQ_FOC_CURRENT,
                         modulation=MOD_SVPWM, current_limit=3.0, velocity_limit=100.0)
rv = run(cfg_v, 1.2, 30.0)
m = rv['t'] > 0.9
w_ss = float(np.mean(rv['wm'][m]))
t_settle = rv['t'][np.argmax(np.abs(rv['wm'] - 30.0) < 1.5)]
overshoot = 100.0 * (rv['wm'].max() - 30.0) / 30.0

T.h('速度环：0 → 30 rad/s 阶跃')
print('  稳态 %.2f rad/s，静差 %.2f%%，超调 %.1f%%，%.2f s 进入 ±5%%'
      % (w_ss, 100 * (30.0 - w_ss) / 30.0, overshoot, t_settle))

# ---------------------------------------------------------------- 负载扰动
rv2 = run(cfg_v, 1.4, 40.0, load=lambda t: 0.02 if t < 0.7 else 0.15, dt_plant=3.0e-5)
m2 = rv2['t'] > 0.6
w_before = float(np.mean(rv2['wm'][(rv2['t'] > 0.6) & (rv2['t'] < 0.7)]))
w_dip = float(np.min(rv2['wm'][rv2['t'] >= 0.7]))
w_after = float(np.mean(rv2['wm'][rv2['t'] > 1.1]))

T.h('负载扰动：0.02 → 0.15 N·m 突变（速度环 PI 兜底）')
print('  扰动前 %.2f -> 最低 %.2f（跌 %.1f%%）-> 恢复 %.2f rad/s'
      % (w_before, w_dip, 100 * (w_before - w_dip) / w_before, w_after))
k_dip = int(np.argmin(np.where(rv2['t'] >= 0.7, rv2['wm'], 1e9)))
k_rec = k_dip + int(np.argmax(rv2['wm'][k_dip:] >= w_before * 0.995))
print('  恢复时间约 %.3f s（速度环积分项把 iq 顶上去，抵消多出来的负载）'
      % (rv2['t'][k_rec] - rv2['t'][k_dip]))

# ---------------------------------------------------------------- 位置环
cfg_p = ControllerConfig(controller=CTRL_ANGLE, torque_mode=TQ_FOC_CURRENT,
                         modulation=MOD_SVPWM, current_limit=3.0, velocity_limit=40.0,
                         p_angle=(8.0, 0.0, 0.0))
rp = run(cfg_p, 1.6, math.pi, dt_plant=3.0e-5)
theta_end = float(rp['shaft_angle'][-1])
T.h('位置环：转到 π rad')
print('  最终角度 %.4f rad（目标 %.4f），误差 %.2f mrad'
      % (theta_end, math.pi, abs(theta_end - math.pi) * 1e3))
print('  峰值转速 %.1f rad/s（位置环 P 输出 = %.0f×角度误差），期间电流峰值 %.2f A'
      % (rp['wm'].max(), 8.0, np.abs(rp['iq_true']).max()))
print('  位置环只有 P：没有"积分"就没有超调，稳态误差靠速度环积分消化掉。')

# ---------------------------------------------------------------- 带宽不匹配
T.h('真正限制速度响应的是"电流限幅"，不是 Kp')
T.p('  加速度上限 a_max = kt·Ilim / J，0→30 rad/s 至少需要 30 / a_max 秒。')
lims = (0.5, 1.5, 3.0)
variants = {}
for ilim in lims:
    cfg = ControllerConfig(controller=CTRL_VELOCITY, torque_mode=TQ_FOC_CURRENT,
                           modulation=MOD_SVPWM, current_limit=ilim, velocity_limit=100.0)
    variants[ilim] = run(cfg, 1.8 if ilim < 1.0 else 1.2, 30.0)
    r = variants[ilim]
    a_max = M.kt * ilim / M.J
    t_reach = r['t'][np.argmax(r['wm'] >= 30.0)]
    print('  Ilim=%.1f A -> a_max=%6.1f rad/s²，理论上限 %.2f s，实测到 30 rad/s 用 %.2f s'
          % (ilim, a_max, 30.0 / a_max, t_reach))

# ---------------------------------------------------------------- 画图
f, ax = T.fig(2, 2, suptitle='实验 08：速度环与位置环')

ax[0].plot(rv['t'], rv['wm'], color=T.C['blue'], lw=1.6, label='转速')
ax[0].axhline(30, color='k', ls='--', lw=1.0, label='给定 30 rad/s')
ax[0].axhspan(28.5, 31.5, color=T.C['green'], alpha=0.12)
T.spec(ax[0], '(a) 速度阶跃：%.0f%% 超调，%.2f s 进 ±5%%' % (overshoot, t_settle),
       '时间 [s]', '转速 [rad/s]')

ax[1].plot(rv2['t'], rv2['wm'], color=T.C['blue'], lw=1.8, label='转速')
ax[1].axvline(0.7, color=T.C['red'], ls=':', lw=1.2)
ax[1].annotate('负载 0.02→0.15 N·m', (0.7, w_dip), xytext=(0.78, 38.6),
               color=T.C['red'], fontsize=8,
               arrowprops=dict(arrowstyle='->', color=T.C['red'], lw=0.8))
ax[1].set_xlim(0.55, 1.4)
ax[1].set_ylim(37.6, 41.2)
ax2 = ax[1].twinx()
ax2.plot(rv2['t'], rv2['tl'], color=T.C['orange'], lw=1.0, ls='--', label='负载转矩')
ax2.set_ylabel('负载 [N·m]', color=T.C['orange'])
ax2.grid(False)
T.spec(ax[1], '(b) 负载扰动：跌 %.1f%%，速度环自动补回来' % (100 * (w_before - w_dip) / w_before),
       '时间 [s]', '转速 [rad/s]')

ax[2].plot(rp['t'], rp['shaft_angle'], color=T.C['red'], lw=1.6, label='转子角度')
ax[2].axhline(math.pi, color='k', ls='--', lw=1.0, label='目标 π')
ax[2].set_xlim(0, 1.2)
T.spec(ax[2], '(c) 位置定位：误差 %.1f mrad' % (abs(theta_end - math.pi) * 1e3),
       '时间 [s]', '角度 [rad]')

for ilim, col in zip(lims, (T.C['gray'], T.C['green'], T.C['red'])):
    r = variants[ilim]
    ax[3].plot(r['t'], r['wm'], color=col, lw=1.5,
               label='Ilim=%.1f A（a≤%.0f rad/s²）' % (ilim, M.kt * ilim / M.J))
    a_max = M.kt * ilim / M.J
    ax[3].plot([30.0 / a_max, 30.0 / a_max], [0, 30.0], ls=':', lw=0.8, color=col, alpha=0.6)
ax[3].axhline(30, color='k', ls='--', lw=1.0)
ax[3].set_xlim(0, 1.2)
T.spec(ax[3], '(d) 电流限幅 = 加速度上限，决定"多快能到"', '时间 [s]', '转速 [rad/s]')

path = T.save(f, 't08_cascade_loops.png')

T.h('小结')
T.bullet('三层环的带宽从内到外递减：电流环 ~1 kHz，速度环 ~50 Hz，位置环 ~5 Hz。',
         '速度环的 PI 负责"抗负载扰动"，位置环的 P 负责"到位"，各有分工。',
         '限幅无处不在：velocity_limit 限制位置环的输出，current_limit 限制速度环的输出。',
         '调试顺序永远是：先让电流环稳，再加速度环，最后加位置环。')

T.questions('如果速度环带宽接近电流环会怎样？为什么工程上要求至少差 5 倍？',
            '位置环加一点 D 会更好吗？在"有齿槽转矩"的真实电机上呢？',
            '负载从 0.02 突加到 0.15 时，如果没有速度环只有电流环（转矩模式），'
            '会发生什么？')

T.done(path)
