# -*- coding: utf-8 -*-
"""实验 02 · Clarke / Park 变换：把交流变成直流

实验 01 里电流矢量在**静止**坐标系里画圆。可如果坐标系跟着转子一起转呢？
矢量就"停住"了——分量变成直流，于是可以用最普通的 PI 控制器去调它。
这就是 FOC（Field Oriented Control，磁场定向控制）的全部秘密。
"""

import math

import numpy as np

import tutlib as T

T.banner('02', 'Clarke / Park 变换', '—— 跟着转子转，交流就变成了直流')

# ---------------------------------------------------------------- 场景
th = np.linspace(0.0, 4.0 * math.pi, 4001)      # 电角度（两周）
A = 1.0                                         # 电流矢量幅值

# 假设电流矢量始终超前转子 90°：这正是"只产生转矩、不产生磁化"的 q 轴方向
phi = th + math.pi / 2.0
i_alpha = A * np.cos(phi)
i_beta = A * np.sin(phi)

# 反 Clarke（等幅值）得到三相电流
i_a = i_alpha
i_b = -0.5 * i_alpha + (math.sqrt(3.0) / 2.0) * i_beta
i_c = -0.5 * i_alpha - (math.sqrt(3.0) / 2.0) * i_beta

# Park：把 alpha-beta 投影到随转子旋转的 d-q 轴
i_d = i_alpha * np.cos(th) + i_beta * np.sin(th)
i_q = -i_alpha * np.sin(th) + i_beta * np.cos(th)

T.h('同一组电流，换两个参考系')
print('  在 αβ 系（定子静止）：iα、iβ 是 50 Hz 正弦，一直在动')
print('                         幅值 %.3f，最大值出现在 α 轴方向' % A)
print('  在 dq 系（随转子转）：id = %+.3f ~ %+.3f，iq = %+.3f ~ %+.3f'
      % (i_d.min(), i_d.max(), i_q.min(), i_q.max()))
print('                         -> 两条水平线，完全直流，PI 控制器可以直接用')

T.h('为什么要刻意让矢量超前 90°')
T.p('  d 轴 = 转子磁极方向（励磁），q 轴 = 与它垂直（转矩）。')
T.p('  · id 的作用是"充磁/弱磁"，对表贴式电机不产生转矩，纯属白烧铜耗；')
T.p('  · iq 与转子磁场正交，转矩 Te = 1.5·p·ψf·iq。')
T.p('  所以正常工作点取 id = 0、iq = 需求转矩 / kt。')

# ---------------------------------------------------------------- 角度误差
dth = np.radians(np.linspace(-90.0, 90.0, 721))
id_err = -A * np.sin(dth)          # 转过的矢量在 d 轴的投影
iq_err = A * np.cos(dth)           # 转过的矢量在 q 轴的投影
gain = 1.0 / np.maximum(np.cos(dth), 1e-6)   # 保持同样转矩所需的电流倍数

T.h('如果角度估计有误差 Δ 会怎样')
print('  id = −|i|·sinΔ,   iq = |i|·cosΔ')
for d in (10, 20, 30, 90):
    c = math.cos(math.radians(d))
    if c > 1e-6:
        tail = '要保持同样转矩电流需放大 %.2f 倍' % (1.0 / c)
    else:
        tail = '转矩彻底为零，加多大电流都不转'
    print('  Δ=%2d° -> id=%+.3f, iq=%+.3f（转矩只剩 %.1f%%），%s'
          % (d, -math.sin(math.radians(d)), c, 100 * c, tail))
T.p('  铜耗 ∝ |i|²，多出来的 −id 电流全部变成热。Δ=30° 时铜耗要多 33%。')
T.note('Δ=90° 是最常见的调试事故：电角度零点（zero_electric_angle）没对齐，'
       '矢量整个指在 d 轴上——电流很大、电机不转、几秒钟就烫。')

# ---------------------------------------------------------------- 画图
f, ax = T.fig(2, 2, suptitle='实验 02：Clarke / Park 变换')

n = len(th)
ax[0].plot(np.degrees(th), i_a, color=T.C['red'], lw=1.4, label='ia')
ax[0].plot(np.degrees(th), i_b, color=T.C['green'], lw=1.4, label='ib')
ax[0].plot(np.degrees(th), i_c, color=T.C['blue'], lw=1.4, label='ic')
ax[0].set_xlim(0, 720)
ax[0].axhline(0, color='k', lw=0.5)
T.spec(ax[0], '(a) 定子看到的三相电流：交流，两个电周期',
       '电角度 [°]', '电流 [A]')

ax[1].plot(i_alpha, i_beta, color=T.C['purple'], lw=1.0, alpha=0.7,
           label='αβ 轨迹（圆）')
for k, col in ((n // 8, T.C['red']), (3 * n // 8, T.C['orange'])):
    thk = th[k]
    ax[1].arrow(0, 0, math.cos(thk) * 1.2, math.sin(thk) * 1.2,
                width=0.008, color=col, alpha=0.85)
    ax[1].arrow(0, 0, math.cos(thk + math.pi / 2) * 0.8,
                math.sin(thk + math.pi / 2) * 0.8, width=0.006, color=col, alpha=0.45)
    ax[1].text(math.cos(thk) * 1.3, math.sin(thk) * 1.3, 'd', color=col, fontsize=11)
    ax[1].text(math.cos(thk + math.pi / 2) * 0.9,
               math.sin(thk + math.pi / 2) * 0.9, 'q', color=col, fontsize=11)

ax[1].set_aspect('equal')
ax[1].set_xlim(-1.45, 1.45)
ax[1].set_ylim(-1.45, 1.45)
T.spec(ax[1], '(b) 同一个圆，换到随转子转的 dq 轴上看', 'iα', 'iβ', legend=False)
ax[1].legend(fontsize=8, loc='lower left')

ax[2].plot(np.degrees(th), i_d, color=T.C['blue'], lw=2.0, label='id（励磁分量）')
ax[2].plot(np.degrees(th), i_q, color=T.C['red'], lw=2.0, label='iq（转矩分量）')
ax[2].set_xlim(0, 720)
ax[2].set_ylim(-0.15, 1.25)
T.spec(ax[2], '(c) Park 变换之后：两条水平线 = 直流', '电角度 [°]', 'dq 电流 [A]')

ax3 = ax[3]
ax3.plot(np.degrees(dth), id_err, color=T.C['blue'], lw=1.6, label='id / |i|')
ax3.plot(np.degrees(dth), iq_err, color=T.C['red'], lw=1.6, label='iq / |i|')
for d in (10, 20, 30):
    ax3.axvline(d, color='k', ls=':', lw=0.7)
    ax3.annotate('%d°' % d, (d, 1.18), fontsize=8, ha='center')
ax3.set_ylim(-1.1, 1.15)
ax3.text(-86, -0.62, 'Δ=30°：iq 只剩 86.6%\n同转矩电流 ×1.15，铜耗 +33%',
         fontsize=8, color=T.C['gray'], va='bottom')
T.spec(ax3, '(d) 角度误差 Δ 把电流从 q 轴"漏"到 d 轴', 'Δ [°]', '归一化')

path = T.save(f, 't02_clarke_park.png')

T.formula('Clarke:  iα = (2/3)(ia − ib/2 − ic/2),   iβ = (ib − ic)/√3',
          'Park:    id =  iα·cosθ + iβ·sinθ,      iq = −iα·sinθ + iβ·cosθ',
          '反 Park: iα = id·cosθ − iq·sinθ,       iβ = id·sinθ + iq·cosθ')

T.h('小结')
T.bullet('Clarke：3 根线 -> 2 个正交分量，只是换个写法，没有改变"交流"这个事实。',
         'Park：把坐标轴钉在转子上，交流就变成直流，PI 才能做到零稳态误差。',
         'FOC 的完整闭环 = 采样三相电流 -> Clarke -> Park -> 两个 PI -> 反 Park -> PWM。',
         '整条链唯一依赖的外部信息是**转子电角度 θe**，它错了，一切归零。')

T.questions('如果转子在转但角度传感器坏了，能否用"电流矢量固定"的方式硬拖？会有什么后果？',
            'Δ 恒为 30° 时，电机仍能输出转矩吗？此时 kt 变成多少？',
            '为什么表贴式电机取 id=0，而内置式（Ld≠Lq）电机要取 id<0 才更省电流（MTPA）？')

T.done(path)
