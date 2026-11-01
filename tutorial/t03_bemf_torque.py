# -*- coding: utf-8 -*-
"""实验 03 · 反电动势、转矩常数与机械特性

三个最基本的问题：
    1. 电机为什么会"自己发电"？反电动势长什么样？
    2. 给 1 A 电流能出多少转矩？（kt）
    3. 转速升上去以后，转矩为什么掉下来了？

本课用项目里的**相域电机模型**（sim/plant.py）+ SimpleFOC 电流环，
在"测功机"模式下扫点实测转矩常数，再和理论值对账。
"""

import math

import numpy as np

import tutlib as T
from sim.bench import simulate
from sim.foc import (CTRL_TORQUE, MOD_SVPWM, TQ_FOC_CURRENT, ControllerConfig)
from sim.plant import MotorParams, emf_sine, emf_trapezoid, flux_sine, flux_trapezoid

T.banner('03', '反电动势、转矩常数与机械特性',
         '—— 1 A 电流能出多少转矩，为什么高速没劲')

M = MotorParams()
VBUS = 24.0

T.h('模型里的电机')
print('  极对数 p=%d，相电阻 Rs=%.2f Ω，相电感 L=%.1f mH'
      % (M.p, M.Rs, M.L * 1e3))
print('  永磁磁链 ψf=%.3f Wb   ->  反电动势常数 ke = p·ψf = %.3f V·s/rad'
      % (M.psi_f, M.ke))
print('  转矩常数 kt = 1.5·p·ψf = %.4f N·m/A（国际单位制下 kt 与 ke 数值相等）'
      % M.kt)
print('  极对数为 4：转子转 1 圈，电角度转 4 圈 -> 机械 300 rpm 对应电频率 %d Hz'
      % (300 / 60.0 * M.p))

# ---------------------------------------------------------------- 反电动势波形
th = np.linspace(0.0, 2.0 * math.pi, 721)
e_sin = np.array([emf_sine(x) for x in th])
e_trp = np.array([emf_trapezoid(x) for x in th])
fl_sin = np.array([flux_sine(x) for x in th])
fl_trp = np.array([flux_trapezoid(x) for x in th])

T.h('反电动势从哪来')
T.p('  转子磁链 λ(θ) 随角度变化，绕组"切割"它，于是 e = ωe·dλ/dθ。')
T.p('  · 正弦机（表贴式 PMSM）：λ = ψf·cosθ，e = −ψf·ωe·sinθ，波形是纯正弦；')
T.p('  · 梯形机（BLDC）：磁钢做成平顶，λ 是梯形，e 是"平顶 120°"的方波。')
print('  两种波形的峰值比：梯形/正弦 = %.3f（同样 ψf 定义下梯形机的 EMF 反而更低）'
      % (np.max(np.abs(e_trp)) / np.max(np.abs(e_sin))))

# ---------------------------------------------------------------- 实测 kt
T.h('实测转矩常数（测功机模式：转速锁定 30 rad/s）')
iq_cmd = np.arange(0.2, 2.81, 0.2)
te_meas = []
for iq in iq_cmd:
    cfg = ControllerConfig(controller=CTRL_TORQUE, torque_mode=TQ_FOC_CURRENT,
                           modulation=MOD_SVPWM, current_limit=4.0)
    rec = simulate(M, cfg, t_end=0.09, target=float(iq), load=0.02,
                   dt_plant=2.0e-5, speed_lock=30.0)
    te_meas.append(float(np.mean(rec['te'][int(0.6 * len(rec['te'])):])))
te_meas = np.array(te_meas)
kt_fit = float(np.polyfit(iq_cmd, te_meas, 1)[0])

print('  iq 给定 [A]: ' + ' '.join('%5.2f' % v for v in iq_cmd))
print('  Te 实测[N·m]: ' + ' '.join('%5.3f' % v for v in te_meas[:8]))
print('  ...（其余 %d 点略）' % (len(iq_cmd) - 8))
print('  线性拟合 kt = %.4f N·m/A，理论 %.4f，偏差 %.2f%%'
      % (kt_fit, M.kt, 100 * (kt_fit - M.kt) / M.kt))
T.note('测出来的点几乎严格落在直线上，是因为模型里没有磁饱和、没有齿槽转矩。'
       '真实电机的 kt 会随电流饱和下降 10~20%，这也是"大电流不一定出大转矩"的原因之一。')

# ---------------------------------------------------------------- 机械特性
V_LIM = VBUS / math.sqrt(3.0)        # SVPWM 线性调制下可输出相电压峰值
CL = 3.0
wm = np.linspace(0.0, 140.0, 400)
curves = {}
for vbus in (12.0, 24.0, 36.0):
    vlim = vbus / math.sqrt(3.0)
    iq_avail = np.minimum(CL, np.maximum(0.0, (vlim - M.ke * wm) / M.Rs))
    curves[vbus] = M.kt * iq_avail

w_max = V_LIM / M.ke
T.h('机械特性：为什么高速"没劲"')
print('  电压方程（稳态、id=0）：vq = R·iq + ωe·ψf')
print('  母线 %0.0f V、SVPWM 线性区 -> 可输出相电压峰值 %.1f V' % (VBUS, V_LIM))
print('  转速升高，反电动势 ke·ωm 吃掉几乎全部电压，留给 R·iq 的余量越来越小')
print('  -> 理论上限转速 ωmax = %.1f rad/s（%.0f rpm），实测约 112 rad/s（电阻压降所致）'
      % (w_max, w_max * 60 / (2 * math.pi)))

# ---------------------------------------------------------------- 画图
f, ax = T.fig(2, 2, suptitle='实验 03：反电动势与转矩')

ax[0].plot(np.degrees(th), e_sin, color=T.C['blue'], lw=1.8, label='正弦机 e(θ)')
ax[0].plot(np.degrees(th), e_trp, color=T.C['orange'], lw=1.8, label='梯形机 e(θ)')
ax[0].plot(np.degrees(th), fl_sin, color=T.C['blue'], lw=1.0, ls='--', alpha=0.7,
           label='正弦机磁链 λ/ψf')
ax[0].plot(np.degrees(th), fl_trp, color=T.C['orange'], lw=1.0, ls='--', alpha=0.7,
           label='梯形机磁链 λ/ψf')
ax[0].set_xlim(0, 360)
ax[0].axhline(0, color='k', lw=0.5)
T.spec(ax[0], '(a) 单位反电动势与磁链（三相之一）', '电角度 [°]', '归一化')

ax[1].plot(iq_cmd, te_meas, 'o', color=T.C['red'], ms=6, label='仿真实测')
ax[1].plot(iq_cmd, M.kt * iq_cmd, '-', color=T.C['gray'], lw=1.4,
           label='理论 Te = %.3f·iq' % M.kt)
ax[1].set_xlim(0, 3.0)
T.spec(ax[1], '(b) 转矩与 q 轴电流严格成正比（转速锁定）',
       'iq [A]', 'Te [N·m]')

wm_axis = np.linspace(0.0, 140.0, 200)
ax[2].plot(wm_axis, M.ke * wm_axis, color=T.C['blue'], lw=1.8,
           label='正弦机 EMF 峰值 = ke·ωm')
ax[2].plot(wm_axis, 0.764 * M.ke * wm_axis, color=T.C['orange'], lw=1.8,
           label='梯形机 EMF 峰值 = 0.764·ke·ωm')
ax[2].axhline(V_LIM, color=T.C['red'], ls=':', lw=1.4,
              label='24 V 母线可用 %.1f V' % V_LIM)
ax[2].axvline(w_max, color='k', ls=':', lw=1.0)
ax[2].annotate('交点 = %.0f rad/s，再快就没有电压余量了' % w_max,
               (w_max, V_LIM), xytext=(w_max - 70, V_LIM + 3.2), fontsize=8,
               arrowprops=dict(arrowstyle='->', color='k', lw=0.8))
ax[2].set_xlim(0, 140)
ax[2].set_ylim(0, 20)
T.spec(ax[2], '(c) 反电动势随转速线性上升，迟早顶到母线',
       '机械转速 [rad/s]', '反电动势峰值 [V]')

ax[3].plot(wm, curves[12.0], color=T.C['green'], lw=1.8, label='12 V 母线')
ax[3].plot(wm, curves[24.0], color=T.C['blue'], lw=1.8, label='24 V 母线')
ax[3].plot(wm, curves[36.0], color=T.C['red'], lw=1.8, label='36 V 母线')
ax[3].axvline(w_max, color='k', ls=':', lw=1.0)
ax[3].annotate('24 V 上限 %.0f rad/s' % w_max, (w_max, 0.02),
               xytext=(w_max - 55, 0.13), fontsize=8,
               arrowprops=dict(arrowstyle='->', color='k', lw=0.8))
ax[3].set_xlim(0, 140)
ax[3].set_ylim(0, 0.6)
T.spec(ax[3], '(d) 机械特性：电流限幅 3 A、SVPWM 线性区',
       '机械转速 [rad/s]', '最大转矩 [N·m]')

path = T.save(f, 't03_bemf_torque.png')

T.h('小结')
T.bullet('反电动势是"电机自己发的电"，与转速成正比，方向与驱动电压相反。',
         '转矩只认 iq：Te = kt·iq，kt = 1.5·p·ψf。id 对表贴式电机不出力。',
         '转速越高，反电动势越大，留在电阻上的电压越少 -> 转矩自然衰减。',
         '想要更高的转速，只能提高母线电压，或者用 id<0 弱磁（把磁链"顶掉"一部分）。')

T.questions('同样体积的电机，把 p 从 4 改成 8，kt 和最高转速各变多少？实际中为什么不做成 p=100？',
            '弱磁（id<0）能提高转速上限，代价是什么？在什么工况下才值得用？',
            '如果把 ψf 减小一半，kt 减半、但最高转速翻倍。对于"需要低速大转矩"的场合，'
            '这个取舍意味着什么？')

T.done(path)
