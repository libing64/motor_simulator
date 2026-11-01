# -*- coding: utf-8 -*-
"""实验 01 · 三相电流与旋转磁场

从零理解：三根线里的交流电流，为什么等效于一个**匀速旋转的矢量**。
这一步是 FOC 的物理起点——后面所有变换都在描述这个矢量。
"""

import math

import numpy as np

import tutlib as T

T.banner('01', '三相电流与旋转磁场', '—— 三根线里的交流，等效于一个旋转矢量')

# ---------------------------------------------------------------- 构造波形
# 三相电流：幅值相同、相位依次相差 120°，这就是"对称三相"
th = np.linspace(0.0, 2.0 * math.pi, 1441)
ia = np.cos(th)
ib = np.cos(th - 2.0 * math.pi / 3.0)
ic = np.cos(th + 2.0 * math.pi / 3.0)

# Clarke 变换（等幅值）：三相 -> 静止两相 alpha-beta
alpha = (2.0 / 3.0) * (ia - 0.5 * ib - 0.5 * ic)
beta = (ib - ic) / math.sqrt(3.0)

mag = np.hypot(alpha, beta)
ang = np.unwrap(np.arctan2(beta, alpha))

T.h('先看三个数')
for deg in (0, 30, 90, 180):
    k = int(deg / 360.0 * 1440)
    print('  θe=%3d°   ia=%+0.3f  ib=%+0.3f  ic=%+0.3f   和=%+0.1e'
          % (deg, ia[k], ib[k], ic[k], ia[k] + ib[k] + ic[k]))

T.h('这组电流对应什么')
T.p('  三相之和恒为 0  ->  只有 2 个独立量，所以能用两相 (α, β) 完整描述')
print('  矢量幅值 |iαβ|: 最小 %.6f / 最大 %.6f   （恒为 1，不脉动）'
      % (mag.min(), mag.max()))
print('  矢量角度 θi   : 与电角度 θe 的偏差最大 %.2e rad  （同步旋转）'
      % np.max(np.abs(np.diff(ang) / np.diff(th) - 1.0)))
T.p('  结论：三相交流 = 一个幅值恒定、以 ωe 匀速旋转的空间矢量。')
T.p('  （只有相位差严格 120° 时轨迹才是圆；90° 会变成椭圆，180° 直接退化成来回直线。）')

# ---------------------------------------------------------------- 画图
f, ax = T.fig(2, 2, suptitle='实验 01：三相对称电流与它的旋转矢量')

ax[0].plot(np.degrees(th), ia, color=T.C['red'], lw=1.6, label='ia')
ax[0].plot(np.degrees(th), ib, color=T.C['green'], lw=1.6, label='ib')
ax[0].plot(np.degrees(th), ic, color=T.C['blue'], lw=1.6, label='ic')
ax[0].axhline(0, color='k', lw=0.6)
for deg in (0, 120, 240):
    ax[0].axvline(deg, color='k', ls=':', lw=0.7)
ax[0].set_xlim(0, 360)
T.spec(ax[0], '(a) 三相电流：幅值相同，相位差 120°', '电角度 [°]', '电流 [A]')

# 圆轨迹 + 箭头
ax[1].plot(alpha, beta, color=T.C['purple'], lw=1.2, label='αβ 轨迹')
for deg, col in ((0, T.C['red']), (120, T.C['orange']), (240, T.C['teal'])):
    k = int(deg / 360.0 * 1440)
    ax[1].plot([0, alpha[k]], [0, beta[k]], color=col, lw=1.8)
    ax[1].annotate('θe=%d°' % deg, (alpha[k], beta[k]), fontsize=8,
                   xytext=(alpha[k] * 1.12, beta[k] * 1.12), color=col)
ax[1].plot([-1.25, 1.25], [0, 0], 'k-', lw=0.5)
ax[1].plot([0, 0], [-1.25, 1.25], 'k-', lw=0.5)
ax[1].set_aspect('equal')
ax[1].set_xlim(-1.3, 1.3)
ax[1].set_ylim(-1.3, 1.3)
T.spec(ax[1], '(b) αβ 平面：轨迹是一个标准圆', 'iα [A]', 'iβ [A]')

ax[2].plot(np.degrees(th), mag, color=T.C['blue'], lw=1.6, label='|iαβ|')
ax[2].plot(np.degrees(th), ang / (2 * math.pi), color=T.C['red'], lw=1.2,
           label='θi / 2π')
ax[2].set_xlim(0, 360)
ax[2].set_ylim(0, 1.25)
T.spec(ax[2], '(c) 幅值恒定；角度与 θe 同步增长', '电角度 [°]', '幅值 / 圈数')

# 相位差决定轨迹：只有 120° 才是标准圆
for shift, col, label in ((120, T.C['blue'], '相差 120°：圆（正常）'),
                          (90, T.C['orange'], '相差 90°：椭圆（转矩脉动）'),
                          (180, T.C['gray'], '相差 180°：来回直线（不转）')):
    a2 = np.cos(th)
    b2 = np.cos(th - math.radians(shift))
    c2 = np.cos(th + math.radians(shift))
    al2 = (2.0 / 3.0) * (a2 - 0.5 * b2 - 0.5 * c2)
    be2 = (b2 - c2) / math.sqrt(3.0)
    ax[3].plot(al2, be2, color=col, lw=1.4, label=label)
ax[3].set_aspect('equal')
ax[3].set_xlim(-1.25, 1.25)
ax[3].set_ylim(-1.25, 1.25)
T.spec(ax[3], '(d) 为什么必须是 120°：相位差决定矢量轨迹', 'iα [A]', 'iβ [A]')

path = T.save(f, 't01_three_phase.png')

T.h('为什么要做这件事')
T.bullet('转矩来自"定子磁场"和"转子磁场"的夹角——需要把三相电流合成一个矢量才好谈角度。',
         '这个矢量如果一直钉在转子的 q 轴上，转矩就恒定；这就是 FOC 要干的事。',
         '把三相压成两相（Clarke）只是第一步，第二步是跟着转子转（Park，实验 02）。')

T.formula('iα = (2/3)(ia − ib/2 − ic/2)        iβ = (ib − ic)/√3',
          '|iαβ| = 1（等幅值变换，不引入 1.5 系数）')

T.note('本课用的是"等幅值"Clarke 变换：矢量长度等于相电流峰值。'
       '教科书里另一种"等功率"变换会多一个 √(2/3)，两者只差比例，不影响物理结论。')

T.questions('三相电流中，如果某相断线（ia=0），剩下的电流矢量还匀速旋转吗？',
            '把三相幅值改成 1.0 / 0.8 / 1.0，矢量轨迹变成什么形状？转矩会出现什么频率的脉动？',
            '电角度 θe 与机械角度 θm 差 p 倍。p=4 时，转子转一圈，电流矢量转几圈？')

T.done(path)
