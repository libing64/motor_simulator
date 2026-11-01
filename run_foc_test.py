#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SimpleFOC × 电机仿真：一键测试 + 出图
=====================================

    python3 run_foc_test.py

会做三件事：
    1. 编译 sim/native/libfocsim.so（若源码有更新）
    2. 运行 tests/test_suite.py 里的全部用例，打印 PASS/FAIL
    3. 生成 foc_test_report.png 与 foc_test_summary.md
"""

import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, ROOT)

from sim import build                                    # noqa: E402
from sim import foc as F                                 # noqa: E402
from sim.bench import simulate, simulate_vf, steady_state, ripple_ptp, thd  # noqa: E402
from sim.plant import MotorParams                        # noqa: E402
from sim.plotstyle import setup_cjk                      # noqa: E402
from tests import test_suite as TS                       # noqa: E402
from tests.harness import run_all, summary               # noqa: E402

MOTOR = MotorParams()
VBUS = 24.0


def _cfg(**kw):
    return TS.cfg(**kw)


# ------------------------------------------------------------------ 数据采集
def collect():
    d = {}
    # 速度阶跃
    d['vel'] = simulate(MOTOR, _cfg(controller=F.CTRL_VELOCITY, torque_mode=F.TQ_FOC_CURRENT),
                        1.0, target=30.0, load=0.03)
    # 位置定位
    d['pos'] = simulate(MOTOR, _cfg(controller=F.CTRL_ANGLE, torque_mode=F.TQ_FOC_CURRENT,
                                    velocity_limit=40.0), 1.2, target=math.pi, load=0.02)
    # 电流限幅
    d['lim'] = simulate(MOTOR, _cfg(controller=F.CTRL_TORQUE, torque_mode=F.TQ_FOC_CURRENT,
                                    current_limit=3.0), 0.25, target=8.0)
    # 测功机对比（转速锁定 30 rad/s）
    d['dyno'] = {name: simulate(MOTOR, _cfg(controller=F.CTRL_TORQUE, torque_mode=F.TQ_FOC_CURRENT,
                                            modulation=mod, current_limit=3.0),
                                0.35, target=1.5, load=0.05, speed_lock=30.0)
                 for mod, name in ((F.MOD_SINE, 'sine'), (F.MOD_SVPWM, 'svpwm'),
                                   (F.MOD_TRAP120, 'trap'))}
    # 高转速下的调制波形（马鞍形更明显）
    d['dyno_hi'] = {name: simulate(MOTOR, _cfg(controller=F.CTRL_TORQUE,
                                               torque_mode=F.TQ_FOC_CURRENT,
                                               modulation=mod, current_limit=3.0),
                                   0.5, target=1.5, load=0.05, speed_lock=80.0)
                    for mod, name in ((F.MOD_SINE, 'sine'), (F.MOD_SVPWM, 'svpwm'))}
    # 电流环阶跃
    d['istep'] = simulate(MOTOR, _cfg(controller=F.CTRL_TORQUE, torque_mode=F.TQ_FOC_CURRENT,
                                      current_limit=3.0),
                          0.2, target=lambda t: 0.5 if t < 0.08 else 2.0,
                          load=0.05, speed_lock=20.0)
    # 开环 V/f 与闭环 FOC
    d['vf'] = simulate_vf(MOTOR, vpk=12.0, f=30.0, ramp_time=0.02, t_end=0.6,
                          load=0.05, dt_plant=2e-5, damp=0.0)
    d['foc'] = simulate(MOTOR, _cfg(controller=F.CTRL_VELOCITY, torque_mode=F.TQ_FOC_CURRENT),
                        0.6, target=47.12, load=0.05, dt_plant=2e-5)
    return d


# ------------------------------------------------------------------ 绘图
def make_figure(d, results, path):
    setup_cjk()
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(3, 3, figsize=(15.5, 11))
    fig.suptitle('SimpleFOC 算法库 × 相域电机模型：闭环联合仿真结果', fontsize=15)

    # 1. 速度阶跃
    ax = axes[0, 0]
    r = d['vel']
    ax.plot(r['t'], r['target'], 'k--', lw=1, label='目标')
    ax.plot(r['t'], r['wm'], 'b-', lw=1.4, label='实测转速')
    ax.set_title('速度环阶跃 30 rad/s')
    ax.set_xlabel('时间 [s]'); ax.set_ylabel('转速 [rad/s]')
    ax2 = ax.twinx()
    ax2.plot(r['t'], r['iq_true'], 'r-', lw=0.9, alpha=0.7)
    ax2.set_ylabel('iq [A]', color='r'); ax2.grid(False)
    ax.legend(loc='lower right', fontsize=8)

    # 2. 位置定位
    ax = axes[0, 1]
    r = d['pos']
    ax.plot(r['t'], r['target'], 'k--', lw=1, label='目标 θ')
    ax.plot(r['t'], r['shaft_angle'], 'g-', lw=1.4, label='转子角')
    ax.set_title('位置环：定位到 π rad')
    ax.set_xlabel('时间 [s]'); ax.set_ylabel('角度 [rad]'); ax.legend(fontsize=8)

    # 3. 电流限幅
    ax = axes[0, 2]
    r = d['lim']
    ax.plot(r['t'], r['current_sp'], 'k--', lw=1, label='电流给定（已限幅）')
    ax.plot(r['t'], r['iq_true'], 'r-', lw=1.2, label='实测 iq')
    ax.axhline(3.0, color='gray', ls=':', lw=1)
    ax.set_title('电流限幅：目标 8 A，限幅 3 A')
    ax.set_xlabel('时间 [s]'); ax.set_ylabel('电流 [A]'); ax.legend(fontsize=8)

    # 4. FOC 稳态波形（相电流 + dq）
    ax = axes[1, 0]
    r = d['vel']
    m = r['t'] > 0.8
    ax.plot(r['t'][m], r['ia'][m], lw=0.9, label='ia')
    ax.plot(r['t'][m], r['ib'][m], lw=0.9, label='ib')
    ax.plot(r['t'][m], r['ic'][m], lw=0.9, label='ic')
    ax.set_title('FOC 稳态：三相电流')
    ax.set_xlabel('时间 [s]'); ax.set_ylabel('电流 [A]'); ax.legend(fontsize=8, ncol=3)

    ax = axes[1, 1]
    s2 = d['istep']
    ax.plot(s2['t'] * 1e3, s2['current_sp'], 'k--', lw=1.0, label='iq 给定')
    ax.plot(s2['t'] * 1e3, s2['iq_true'], 'r-', lw=1.2, label='iq 响应')
    ax.plot(s2['t'] * 1e3, s2['id_true'], 'b-', lw=0.9, alpha=0.8, label='id（耦合）')
    ax.set_title('电流环阶跃 0.5 A → 2 A（转速锁定）')
    ax.set_xlabel('时间 [ms]'); ax.set_ylabel('电流 [A]'); ax.legend(fontsize=8)

    # 5. 调制对比：两个电周期内的 A 相占空比
    ax = axes[1, 2]
    fe = 80.0 * MOTOR.p / (2 * math.pi)
    k0 = int(0.42 / (1.0 / 20000.0))
    n_win = int(round(2.0 / fe * 20000.0))
    for name, color, label in (('sine', '#1f77b4', '正弦 PWM'),
                               ('svpwm', '#d62728', 'SVPWM（中点钳位）')):
        r = d['dyno_hi'][name]
        dca = r['dc_a'][k0:k0 + n_win]
        deg = np.linspace(0, 720, len(dca))
        ax.plot(deg, dca, color=color, lw=1.3, label=label)
    ax.axhline(0, color='k', lw=0.6); ax.axhline(1, color='k', lw=0.6)
    ax.set_ylim(-0.05, 1.05)
    ax.set_xlim(0, 720)
    ax.set_title('调制方式对比：A 相占空比（零序注入后呈马鞍形）')
    ax.set_xlabel('电角度 [°]'); ax.set_ylabel('占空比'); ax.legend(fontsize=8)

    # 6. 测功机：转矩与电流波形
    ax = axes[2, 0]
    for name, color, label in (('sine', '#1f77b4', 'FOC 正弦'),
                               ('trap', '#ff7f0e', '六步方波 120°')):
        r = d['dyno'][name]
        m = r['t'] > 0.24
        ax.plot(r['t'][m] * 1e3, r['ia'][m], color=color, lw=1.0, label=label + ' ia')
    ax.set_title('相电流波形：FOC vs 六步换相（同转速同负载）')
    ax.set_xlabel('时间 [ms]'); ax.set_ylabel('电流 [A]'); ax.legend(fontsize=8)

    ax = axes[2, 1]
    for name, color, label in (('sine', '#1f77b4', 'FOC 正弦'),
                               ('trap', '#ff7f0e', '六步方波 120°')):
        r = d['dyno'][name]
        m = r['t'] > 0.24
        ax.plot(r['t'][m] * 1e3, r['te'][m], color=color, lw=1.0, label=label)
    ax.set_title('电磁转矩纹波（转速锁定）')
    ax.set_xlabel('时间 [ms]'); ax.set_ylabel('转矩 [N·m]'); ax.legend(fontsize=8)

    # 7. 开环 V/f vs 闭环 FOC
    ax = axes[2, 2]
    ax.plot(d['vf']['t'], d['vf']['rpm'], color='#888', lw=1.0, label='开环 V/f（0.02 s 斜坡）')
    ax.plot(d['foc']['t'], d['foc']['rpm'], color='#2ca02c', lw=1.4, label='闭环 FOC')
    ax.axhline(450, color='k', ls=':', lw=1, label='同步转速 450 rpm')
    ax.set_title('失步对比：开环 V/f vs 闭环 FOC')
    ax.set_xlabel('时间 [s]'); ax.set_ylabel('转速 [rpm]'); ax.legend(fontsize=8)

    fig.tight_layout(rect=[0, 0, 1, 0.965])
    fig.savefig(path, dpi=110)
    print('[OK] 图已保存: %s' % path)
    return path


def make_bars(results, path):
    """把几个关键指标做成柱状图。"""
    setup_cjk()
    import matplotlib.pyplot as plt

    ripple = getattr(TS.t_ripple_compare, 'cache', None)
    speed = getattr(TS.t_max_speed, 'cache', None)

    fig, axes = plt.subplots(1, 3, figsize=(14, 3.8))
    fig.suptitle('调制方式量化对比（转速锁定测功机，iq=1.5 A）', fontsize=13)

    if ripple:
        names = ['正弦 PWM', 'SVPWM', '六步 120°']
        thds = [ripple[k]['thd'] for k in ('sine', 'svpwm', 'trap')]
        rips = [ripple[k]['ripple'] for k in ('sine', 'svpwm', 'trap')]
        ax = axes[0]
        ax.bar(names, thds, color=['#1f77b4', '#d62728', '#ff7f0e'])
        ax.set_ylabel('相电流 THD [%]'); ax.set_title('电流畸变')
        for i, v in enumerate(thds):
            ax.text(i, v, '%.1f%%' % v, ha='center', va='bottom', fontsize=9)
        ax = axes[1]
        ax.bar(names, rips, color=['#1f77b4', '#d62728', '#ff7f0e'])
        ax.set_ylabel('转矩纹波 [%]'); ax.set_title('电磁转矩纹波（峰峰值/均值）')
        for i, v in enumerate(rips):
            ax.text(i, v, '%.1f%%' % v, ha='center', va='bottom', fontsize=9)

    if speed:
        ax = axes[2]
        names = ['正弦 PWM', 'SVPWM']
        vals = [speed['sine'], speed['svpwm']]
        ax.bar(names, vals, color=['#1f77b4', '#d62728'])
        ax.axhline(speed['theory'], color='k', ls=':', lw=1)
        ax.set_ylabel('最高可调转速 [rad/s]')
        ax.set_title('母线利用率（24 V，理论极限 %.0f）' % speed['theory'])
        for i, v in enumerate(vals):
            ax.text(i, v, '%.0f' % v, ha='center', va='bottom', fontsize=9)

    fig.tight_layout(rect=[0, 0, 1, 0.92])
    fig.savefig(path, dpi=110)
    print('[OK] 图已保存: %s' % path)
    return path


def write_summary(results, path, extra_lines):
    n = len(results)
    ok = sum(1 for r in results if r.passed)
    lines = ['# SimpleFOC × 电机仿真 联合测试报告', '',
             '- 算法库: `third_party/Arduino-FOC` (SimpleFOC v2.4.0, git submodule)',
             '- 电机模型: `sim/plant.py`（三相相域 PMSM/BLDC，RK4）',
             '- 控制周期: 20 kHz，电机积分步长: 10 µs',
             '- 结果: **%d/%d 通过**' % (ok, n), '',
             '| 用例 | 结果 | 说明 |', '|------|------|------|']
    for r in results:
        lines.append('| %s | %s | %s |' % (r.name, 'PASS' if r.passed else 'FAIL',
                                           r.detail.replace('|', '/')))
    lines += ['', '## 场景对比', ''] + extra_lines
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write('\n'.join(lines) + '\n')
    print('[OK] 报告已保存: %s' % path)


if __name__ == '__main__':
    print('=' * 78)
    print('Step 1/3  编译 SimpleFOC 原生桥接库')
    print('=' * 78)
    print(build.build())

    print('=' * 78)
    print('Step 2/3  运行测试用例')
    print('=' * 78)
    results = run_all()
    print(summary(results))

    print('=' * 78)
    print('Step 3/3  生成图表')
    print('=' * 78)
    data = collect()
    make_figure(data, results, os.path.join(ROOT, 'foc_test_report.png'))
    make_bars(results, os.path.join(ROOT, 'foc_test_compare.png'))

    ripple = getattr(TS.t_ripple_compare, 'cache', None) or {}
    speed = getattr(TS.t_max_speed, 'cache', None) or {}
    extra = []
    if ripple:
        extra.append('| 调制方式 | 电流 THD | 转矩纹波 |')
        extra.append('|----------|----------|----------|')
        for key, label in (('sine', '正弦 PWM'), ('svpwm', 'SVPWM'), ('trap', '六步 120°')):
            extra.append('| %s | %.1f%% | %.2f%% |' % (
                label, ripple[key]['thd'], ripple[key]['ripple']))
    if speed:
        extra += ['', '| 调制方式 | 最高可调转速 |', '|----------|--------------|',
                  '| 正弦 PWM | %.0f rad/s |' % speed['sine'],
                  '| SVPWM | %.0f rad/s (+%.1f%%) |' % (
                      speed['svpwm'], (speed['svpwm'] / speed['sine'] - 1) * 100)]
    write_summary(results, os.path.join(ROOT, 'foc_test_summary.md'), extra)
