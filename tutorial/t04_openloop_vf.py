# -*- coding: utf-8 -*-
"""实验 04 · 开环 V/f：最简单，也最脆

不用任何位置传感器，只按"电压跟着频率走"给三相正弦——就是 V/f 变频器做的事。
本课会看到它能正常工作，也能看到它失步时**驱动器完全不知道**。

这一课的意义不在 V/f 本身，而在反衬：没有转子角度反馈，就没有转矩控制。
"""

import math

import numpy as np

import tutlib as T
from sim.bench import simulate_vf
from sim.plant import MotorParams

T.banner('04', '开环 V/f 驱动', '—— 没有反馈，失步了也没人告诉你')

M = MotorParams()
F_HZ = 40.0
W_SYNC = 2.0 * math.pi * F_HZ / M.p        # 同步机械角速度
RPM_SYNC = W_SYNC * 60.0 / (2.0 * math.pi)

T.h('V/f 的基本想法')
T.p('  反电动势 ∝ 频率，所以电压也按同样比例给：V/f = 常数')
print('  本课设定：f = %.0f Hz -> 同步转速 %.1f rad/s（%.0f rpm）'
      % (F_HZ, W_SYNC, RPM_SYNC))
print('  理论所需电压：vpk ≈ ψf·ωe = %.2f V；这里取 7 V（略欠励磁，电流最小）'
      % (M.psi_f * 2 * math.pi * F_HZ))

cases = {
    '匹配（7 V，斜坡 0.6 s）': dict(vpk=7.0, ramp_time=0.6, damp=0.05),
    '欠励磁（3 V）': dict(vpk=3.0, ramp_time=0.6, damp=0.05),
    '快斜坡（9 V，斜坡 0.02 s）': dict(vpk=9.0, ramp_time=0.02, damp=0.01),
}

T.h('三种情况')
runs = {}
for name, kw in cases.items():
    rec = simulate_vf(M, f=F_HZ, t_end=1.6, dt_plant=5e-5, **kw)
    runs[name] = rec
    m = rec['t'] > 1.4
    w_end = float(np.mean(rec['wm'][m]))
    i_pk = float(np.abs(rec['ia'][m]).max())
    print('  %-26s 稳态转速 %6.1f rad/s（%.0f rpm），相电流峰值 %5.2f A  %s'
          % (name, w_end, w_end * 60 / (2 * math.pi), i_pk,
             '同步' if abs(w_end - W_SYNC) < 1.0 else '失步'))

T.p('')
T.p('  欠励磁和快斜坡都失步了。注意它们的电流：一个 10 A、一个十几安——')
T.p('  失步后转子跟不上旋转磁场，定子绕组相当于"变压器短路"，电流只受电阻限制。')

# ---------------------------------------------------------------- 边界扫描
ratios = np.array([3.0, 5.0, 7.0, 9.0, 11.0, 13.0, 16.0])
sweep_w, sweep_i = [], []
for v in ratios:
    rec = simulate_vf(M, vpk=float(v), f=F_HZ, ramp_time=0.6, t_end=1.6,
                      dt_plant=5e-5, damp=0.05)
    m = rec['t'] > 1.4
    sweep_w.append(float(np.mean(rec['wm'][m])))
    sweep_i.append(float(np.abs(rec['ia'][m]).max()))
sweep_w, sweep_i = np.array(sweep_w), np.array(sweep_i)

loads = np.arange(0.0, 0.32, 0.02)
load_w, load_i = [], []
for ld in loads:
    rec = simulate_vf(M, vpk=7.0, f=F_HZ, ramp_time=0.6, t_end=1.6,
                      dt_plant=5e-5, damp=0.05, load=float(ld))
    m = rec['t'] > 1.4
    load_w.append(float(np.mean(rec['wm'][m])))
    load_i.append(float(np.abs(rec['ia'][m]).max()))
load_w, load_i = np.array(load_w), np.array(load_i)

T.h('它的两个边界')
ok = np.abs(sweep_w - W_SYNC) < 1.0
print('  ① 电压必须配得上频率：V/f 太小 -> 失步；V/f 太大 -> 电流白烧')
print('     ' + '  '.join('%.0fV:%s' % (v, '同步' if o else '失步')
                         for v, o in zip(ratios, ok)))
print('  ② 同步转矩有上限：负载到 %.2f N·m 就开始失步（极限约 0.12），电流冲到 %.0f A'
      % (loads[np.argmax(load_i > 8)], load_i.max()))
print('     而变频器仍然"以为"自己在 40 Hz 输出——这种故障只能靠外部保护发现。')

# ---------------------------------------------------------------- 画图
f, ax = T.fig(2, 2, suptitle='实验 04：开环 V/f 驱动')

for (name, rec), col in zip(runs.items(), (T.C['green'], T.C['red'], T.C['orange'])):
    ax[0].plot(rec['t'], rec['wm'], color=col, lw=1.5, label=name)
ax[0].axhline(W_SYNC, color='k', ls=':', lw=1.2, label='同步转速 %.1f' % W_SYNC)
ax[0].set_xlim(0, 1.6)
ax[0].set_ylim(0, 72)
ax[0].annotate('失步：围绕一个较低的转速来回"荡"，\n转差始终存在',
               (0.95, 43), xytext=(0.06, 9), fontsize=8, color=T.C['orange'],
               arrowprops=dict(arrowstyle='->', color=T.C['orange'], lw=0.8))
T.spec(ax[0], '(a) 转子能不能跟上旋转磁场', '时间 [s]', '机械角速度 [rad/s]')

rec = runs['快斜坡（9 V，斜坡 0.02 s）']
m = rec['t'] > 1.3
ax[1].plot(rec['t'][m], rec['ia'][m], color=T.C['red'], lw=0.9, label='ia')
ax[1].plot(rec['t'][m], rec['ib'][m], color=T.C['green'], lw=0.9, label='ib')
ax[1].plot(rec['t'][m], rec['ic'][m], color=T.C['blue'], lw=0.9, label='ic')
T.spec(ax[1], '(b) 失步后的相电流：幅值大、频率和电压指令对不上',
       '时间 [s]', '电流 [A]')

ax2 = ax[2]
ax2.plot(ratios, sweep_w, 'o-', color=T.C['blue'], lw=1.6, label='稳态转速')
ax2.axhline(W_SYNC, color='k', ls=':', lw=1.0)
ax2.set_xlabel('相电压峰值 [V]（f = 40 Hz 固定）')
ax2.set_ylabel('转速 [rad/s]', color=T.C['blue'])
ax2.set_ylim(40, 70)
ax2b = ax2.twinx()
ax2b.plot(ratios, sweep_i, 's--', color=T.C['red'], lw=1.3, label='相电流峰值')
ax2b.set_ylabel('电流 [A]', color=T.C['red'])
ax2b.grid(False)
ax2.set_title('(c) 同一频率下扫电压：欠励磁失步，过励磁过流', fontsize=10)
ax2.legend(fontsize=8, loc='lower left')
ax2b.legend(fontsize=8, loc='upper left')

ax3 = ax[3]
ax3.plot(loads, load_w, 'o-', color=T.C['blue'], lw=1.6, label='稳态转速')
ax3.axhline(W_SYNC, color='k', ls=':', lw=1.0)
ax3.set_xlabel('负载转矩 [N·m]')
ax3.set_ylabel('转速 [rad/s]', color=T.C['blue'])
ax3.set_ylim(45, 70)
ax3b = ax3.twinx()
ax3b.plot(loads, load_i, 's--', color=T.C['red'], lw=1.3, label='相电流峰值')
ax3b.set_ylabel('电流 [A]', color=T.C['red'])
ax3b.grid(False)
ax3.set_title('(d) 加负载：超过同步转矩极限就失步', fontsize=10)
ax3.legend(fontsize=8, loc='lower left')
ax3b.legend(fontsize=8, loc='upper left')

path = T.save(f, 't04_openloop_vf.png')

T.h('本课真正想说的')
T.bullet('开环 V/f 能用，但它控制的只是"我输出什么"，不是"电机发生了什么"。',
         '失步、过载、堵转——在它眼里都是"正常输出"，直到绕组烧掉。',
         '同步电机要输出稳定转矩，必须让定子磁场和转子磁场保持固定夹角。',
         '要做到这一点，必须知道转子的实时电角度 -> 这就是闭环 FOC 的入口。')

T.note('真实变频器靠"电流检测 + 失步保护"兜底；模型里我们给了一点阻尼绕组'
       '（damp 参数）来模拟异步启动转矩，否则同步电机连启动都做不到。')

T.questions('如果负载突然从 0 加到 0.3 N·m，V/f 驱动器会怎么反应？FOC 会怎么反应？',
            'V/f 控制下"转差"是被动的。为什么异步电机可以靠转差自然产生转矩，而同步电机不行？',
            '有没有办法在不开环的前提下做到"有点反馈"？（提示：反电动势观测器、电流相位）')

T.done(path)
