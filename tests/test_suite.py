# -*- coding: utf-8 -*-
"""
SimpleFOC × 电机仿真 联合测试套件
================================

每个用例都是「跑一次闭环仿真 / 直接调用库函数 → 与理论值或 numpy 参考实现比对」。
用 `python3 tests/run_tests.py` 运行，或由 `run_foc_test.py` 调用并出图。
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from sim import foc as F                      # noqa: E402
from sim.bench import simulate, simulate_vf, steady_state, ripple_ptp, thd  # noqa: E402
from sim.plant import MotorParams             # noqa: E402
from tests.harness import case                # noqa: E402

MOTOR = MotorParams()          # Rs=0.8Ω, L=2mH, ψf=0.03Wb, pp=4, J=3e-3
VBUS = 24.0
FS = 20000.0                   # 控制频率 [Hz]
DT = 1.0e-5                    # 电机模型积分步长 [s]


def cfg(**kw) -> F.ControllerConfig:
    base = dict(vbus=VBUS, voltage_limit=VBUS, current_limit=3.0, velocity_limit=60.0,
                control_freq=FS)
    base.update(kw)
    return F.ControllerConfig(**base)


def _step_at(ctrl: F.FocController, theta_e: float, uq: float) -> F.FocState:
    """在指定电角度施加 uq（电压模式），补偿库内一个控制周期的输出延迟。"""
    thm = theta_e / MOTOR.p
    ctrl.step((0.0, 0.0, 0.0), thm, 0.0, uq)
    return ctrl.step((0.0, 0.0, 0.0), thm, 0.0, uq)


# ===================================================================== 1. 变换/调制
@case('Clarke/Park 参考实现自检（numpy）', ['math'])
def t_transform_selftest():
    rng = np.random.default_rng(7)
    a = rng.normal(size=200)
    alpha = 2.0 / 3.0 * (a[0:64] - 0.5 * a[64:128] - 0.5 * a[128:192])
    th = rng.uniform(0, 2 * np.pi, size=64)
    beta = (a[64:128] - a[128:192]) / np.sqrt(3.0)
    d = alpha * np.cos(th) + beta * np.sin(th)
    q = -alpha * np.sin(th) + beta * np.cos(th)
    a2 = d * np.cos(th) - q * np.sin(th)
    b2 = d * np.sin(th) + q * np.cos(th)
    err = max(np.max(np.abs(a2 - alpha)), np.max(np.abs(b2 - beta)))
    assert err < 1e-12, 'round-trip error %.3e' % err
    return 'round-trip 误差 %.1e' % err


@case('正弦 PWM 相电压 = 参考公式', ['modulation'])
def t_sine_pwm_reference():
    ctrl = F.FocController(MOTOR, cfg(torque_mode=F.TQ_VOLTAGE, modulation=F.MOD_SINE))
    uq, worst = 10.0, 0.0
    for th in np.linspace(0.05, 2 * np.pi, 24):
        st = _step_at(ctrl, th, uq)
        al, be = -math.sin(th) * uq, math.cos(th) * uq
        ua, ub, uc = al, -0.5 * al + math.sqrt(3) / 2 * be, -0.5 * al - math.sqrt(3) / 2 * be
        ref = (ua + VBUS / 2, ub + VBUS / 2, uc + VBUS / 2)
        worst = max(worst, abs(st.ua - ref[0]), abs(st.ub - ref[1]), abs(st.uc - ref[2]))
    assert worst < 5e-3, '最大相电压偏差 %.4f V' % worst
    return '最大偏差 %.2f mV' % (worst * 1e3)


@case('SVPWM 相电压 = 参考公式（中点钳位）', ['modulation'])
def t_svpwm_reference():
    ctrl = F.FocController(MOTOR, cfg(torque_mode=F.TQ_VOLTAGE, modulation=F.MOD_SVPWM))
    uq, worst = 10.0, 0.0
    for th in np.linspace(0.05, 2 * np.pi, 24):
        st = _step_at(ctrl, th, uq)
        al, be = -math.sin(th) * uq, math.cos(th) * uq
        ua, ub, uc = al, -0.5 * al + math.sqrt(3) / 2 * be, -0.5 * al - math.sqrt(3) / 2 * be
        ctr = VBUS / 2 - (max(ua, ub, uc) + min(ua, ub, uc)) / 2
        ref = (ua + ctr, ub + ctr, uc + ctr)
        worst = max(worst, abs(st.ua - ref[0]), abs(st.ub - ref[1]), abs(st.uc - ref[2]))
    assert worst < 5e-3, '最大相电压偏差 %.4f V' % worst
    return '最大偏差 %.2f mV' % (worst * 1e3)


@case('占空比 = 相电压 / 母线电压', ['modulation'])
def t_duty_relation():
    ctrl = F.FocController(MOTOR, cfg(torque_mode=F.TQ_VOLTAGE, modulation=F.MOD_SINE))
    worst = 0.0
    for th in np.linspace(0.1, 6.0, 12):
        st = _step_at(ctrl, th, 8.0)
        worst = max(worst, abs(st.dc_a - st.ua / VBUS), abs(st.dc_b - st.ub / VBUS),
                    abs(st.dc_c - st.uc / VBUS))
    assert worst < 1e-3, 'duty 偏差 %.5f' % worst
    return 'duty 偏差 %.1e' % worst


def _max_linear_uq(module: int, tol: float = 3e-4) -> float:
    """在 duty 不触顶/触底的前提下，能施加的最大 q 轴电压。"""
    ctrl = F.FocController(MOTOR, cfg(torque_mode=F.TQ_VOLTAGE, modulation=module))
    thetas = np.linspace(0.0, 2 * np.pi, 49)[:-1]

    def feasible(uq: float) -> bool:
        for th in thetas:
            st = _step_at(ctrl, th, uq)
            dc = (st.dc_a, st.dc_b, st.dc_c)
            if min(dc) <= tol or max(dc) >= 1.0 - tol:
                return False
        return True

    lo, hi = 0.0, VBUS
    for _ in range(22):
        mid = 0.5 * (lo + hi)
        if feasible(mid):
            lo = mid
        else:
            hi = mid
    return lo


@case('SVPWM 线性区比正弦 PWM 高 √3/2', ['modulation'])
def t_svpwm_utilization():
    u_sine = _max_linear_uq(F.MOD_SINE)
    u_svpwm = _max_linear_uq(F.MOD_SVPWM)
    ratio = u_svpwm / u_sine
    assert abs(u_sine - VBUS / 2) / (VBUS / 2) < 0.03, '正弦线性上限 %.2f V' % u_sine
    assert abs(u_svpwm - VBUS / math.sqrt(3)) / (VBUS / math.sqrt(3)) < 0.03, \
        'SVPWM 线性上限 %.2f V' % u_svpwm
    assert abs(ratio - 2 / math.sqrt(3)) < 0.02, '利用率比 %.4f' % ratio
    return '正弦 %.2f V / SVPWM %.2f V（+%.1f%%）' % (u_sine, u_svpwm, (ratio - 1) * 100)


@case('零序注入：同电压下 SVPWM 保留调制裕量', ['modulation'])
def t_svpwm_zero_sequence():
    uq = 12.0                       # 恰好是正弦调制的线性上限（Vbus/2）
    out = {}
    for mod, name in ((F.MOD_SINE, 'sine'), (F.MOD_SVPWM, 'svpwm')):
        ctrl = F.FocController(MOTOR, cfg(torque_mode=F.TQ_VOLTAGE, modulation=mod))
        mins, maxs = [], []
        for th in np.linspace(0.02, 2 * np.pi, 36):
            st = _step_at(ctrl, th, uq)
            mins.append(min(st.dc_a, st.dc_b, st.dc_c))
            maxs.append(max(st.dc_a, st.dc_b, st.dc_c))
        out[name] = (min(mins), max(maxs))
    # 正弦：恰好贴住 0/1 两个边界；SVPWM：中点钳位后两端各留出 (1-√3/2) 的裕量
    assert out['sine'][0] < 0.01, '正弦最低占空比 %.4f 应贴 0' % out['sine'][0]
    assert out['sine'][1] > 0.99, '正弦最高占空比 %.4f 应贴 1' % out['sine'][1]
    # 中点钳位后占空比范围收紧到 0.5 ∓ √3/4（即 0.067~0.933），两端各留 6.7% 裕量
    margin = math.sqrt(3.0) / 4.0
    assert abs(out['svpwm'][0] - (0.5 - margin)) < 0.02, 'SVPWM 下限 %.4f' % out['svpwm'][0]
    assert abs(out['svpwm'][1] - (0.5 + margin)) < 0.02, 'SVPWM 上限 %.4f' % out['svpwm'][1]
    return '同 12 V：正弦 dc=[%.3f,%.3f]，SVPWM dc=[%.3f,%.3f]（两端各留 %.1f%% 裕量）' % (
        out['sine'][0], out['sine'][1], out['svpwm'][0], out['svpwm'][1],
        (0.5 - margin) * 100)


@case('六步换相（Trapezoid_120）扇区与电平', ['modulation'])
def t_six_step():
    ctrl = F.FocController(MOTOR, cfg(torque_mode=F.TQ_VOLTAGE, modulation=F.MOD_TRAP120))
    uq = 6.0
    off_phase = set()
    for th in np.linspace(0.05, 2 * np.pi, 36):
        st = _step_at(ctrl, th, uq)
        dc = (st.dc_a, st.dc_b, st.dc_c)
        assert abs(max(dc) - min(dc)) > 1e-6, '三相占空比应分成两组'
        mean = sum(dc) / 3.0
        driven = [i for i, v in enumerate(dc) if abs(v - mean) > 1e-6]
        assert len(driven) == 2, '六步换相应有两相导通'
        off_phase.add(3 - sum(driven))      # 悬空相（占空比 = 中心值）
        spread = max(dc) - min(dc)
        assert abs(spread - 2 * uq / VBUS) < 2e-3, '导通相差值 %.4f' % spread
    assert len(off_phase) == 3, '扇区未覆盖三相'
    return '6 扇区、导通相差值 2·Uq/Vbus，悬空相覆盖 ABC'


# ===================================================================== 2. 闭环控制
@case('转矩模式：Te = 1.5·p·ψf·iq', ['closed-loop'])
def t_torque_identity():
    rec = simulate(MOTOR, cfg(controller=F.CTRL_TORQUE, torque_mode=F.TQ_FOC_CURRENT,
                             current_limit=2.0), 0.4, target=1.0, load=0.05, dt_plant=DT)
    t_from = rec['t'][-1] - 0.1
    iq = steady_state(rec, 'iq_true', t_from)
    te = steady_state(rec, 'te', t_from)
    kt_meas = te / iq
    err = abs(kt_meas - MOTOR.kt) / MOTOR.kt
    assert abs(iq - 1.0) < 0.05, 'iq 稳态 %.3f A（目标 1.0）' % iq
    assert err < 0.05, '实测 kt=%.4f，理论 %.4f' % (kt_meas, MOTOR.kt)
    return 'iq=%.3f A, kt=%.4f N·m/A (理论 %.3f)' % (iq, kt_meas, MOTOR.kt)


@case('转矩模式：正反向电流跟踪', ['closed-loop'])
def t_torque_tracking():
    errs = []
    for tgt in (0.5, 1.5, -0.8):
        rec = simulate(MOTOR, cfg(controller=F.CTRL_TORQUE, torque_mode=F.TQ_FOC_CURRENT,
                                 current_limit=2.5), 0.3, target=tgt, load=0.0, dt_plant=DT)
        iq = steady_state(rec, 'iq_true', rec['t'][-1] - 0.08)
        errs.append(abs(iq - tgt) / abs(tgt))
    assert max(errs) < 0.08, '最大相对误差 %.1f%%' % (max(errs) * 100)
    return '误差 ' + ', '.join('%.1f%%' % (e * 100) for e in errs)


@case('电流限幅：目标超限时被钳位', ['closed-loop'])
def t_current_limit():
    rec = simulate(MOTOR, cfg(controller=F.CTRL_TORQUE, torque_mode=F.TQ_FOC_CURRENT,
                             current_limit=3.0), 0.25, target=8.0, load=0.0, dt_plant=DT)
    iq_peak = float(np.max(np.abs(rec['iq_true'])))
    sp = float(np.max(rec['current_sp']))
    assert sp <= 3.02, 'current_sp=%.3f 未被钳位' % sp
    assert iq_peak < 3.25, '实际电流超限 %.3f A' % iq_peak
    return 'current_sp=%.2f A（限幅 3 A），实际峰值 %.2f A' % (sp, iq_peak)


@case('速度模式：30 rad/s 阶跃（无静差、超调）', ['closed-loop'])
def t_velocity_step():
    tgt = 30.0
    rec = simulate(MOTOR, cfg(controller=F.CTRL_VELOCITY, torque_mode=F.TQ_FOC_CURRENT),
                   0.8, target=tgt, load=0.03, dt_plant=DT)
    w = rec['wm']
    peak = float(np.max(w))
    over = (peak - tgt) / tgt
    ss = steady_state(rec, 'wm', rec['t'][-1] - 0.15)
    err = abs(ss - tgt) / tgt
    band = np.abs(w - tgt) < 0.05 * tgt
    t_rise = rec['t'][np.argmax(band)] if band.any() else 9e9
    assert err < 0.005, '稳态误差 %.2f%%' % (err * 100)
    assert over < 0.30, '超调 %.1f%%' % (over * 100)
    assert t_rise < 0.25, '调节时间 %.3f s' % t_rise
    return '稳态 %.2f rad/s (误差 %.2f%%)，超调 %.0f%%，%.2f s 进入 ±5%%' % (
        ss, err * 100, over * 100, t_rise)


@case('速度模式：反转 30 → −30 rad/s', ['closed-loop'])
def t_velocity_reverse():
    rec = simulate(MOTOR, cfg(controller=F.CTRL_VELOCITY, torque_mode=F.TQ_FOC_CURRENT),
                   1.2, target=lambda t: 30.0 if t < 0.4 else -30.0, load=0.02, dt_plant=DT)
    ss = steady_state(rec, 'wm', rec['t'][-1] - 0.15)
    assert abs(ss + 30.0) < 0.5, '反转稳态 %.2f rad/s' % ss
    dip = float(np.min(rec['iq_true']))
    assert dip < -0.5, '未见反向电流'
    return '稳态 %.2f rad/s, 峰值反向电流 %.2f A' % (ss, dip)


@case('位置模式：定位 π rad（无静差）', ['closed-loop'])
def t_position_step():
    rec = simulate(MOTOR, cfg(controller=F.CTRL_ANGLE, torque_mode=F.TQ_FOC_CURRENT,
                             velocity_limit=40.0), 1.2, target=math.pi, load=0.02, dt_plant=DT)
    th = rec['shaft_angle'][-1]
    err = abs(th - math.pi)
    assert err < 0.03, '定位误差 %.4f rad' % err
    return 'θ=%.4f rad（目标 %.4f），误差 %.1f mrad' % (th, math.pi, err * 1e3)


@case('负载扰动：速度环 PI 恢复', ['closed-loop'])
def t_load_disturbance():
    rec = simulate(MOTOR, cfg(controller=F.CTRL_VELOCITY, torque_mode=F.TQ_FOC_CURRENT,
                             current_limit=3.0), 1.2, target=40.0,
                   load=lambda t: 0.03 if t < 0.5 else 0.12, dt_plant=DT)
    pre = steady_state(rec, 'wm', 0.3, 0.45)
    dip = float(np.min(rec['wm'][rec['t'] > 0.5]))
    post = steady_state(rec, 'wm', rec['t'][-1] - 0.2)
    assert abs(post - 40.0) / 40.0 < 0.01, '恢复后误差 %.2f%%' % (abs(post - 40) / 0.4)
    assert dip < pre - 0.5, '未见速度跌落'
    return '扰动前 %.2f → 最低 %.2f → 恢复 %.2f rad/s' % (pre, dip, post)


# ===================================================================== 3. 调制方式对比
def _run_ripple(module: int, iq: float = 1.5):
    # 测功机模式：转速锁定 30 rad/s，只看调制方式对电流/转矩波形的影响
    rec = simulate(MOTOR, cfg(controller=F.CTRL_TORQUE, torque_mode=F.TQ_FOC_CURRENT,
                             modulation=module, current_limit=3.0),
                   0.35, target=iq, load=0.05, dt_plant=DT, speed_lock=30.0)
    t_from = rec['t'][-1] - 0.2
    mask = rec['t'] >= t_from
    te = rec['te'][mask]
    mean = float(np.mean(te))
    rip = (float(np.max(te)) - float(np.min(te))) / mean * 100.0
    f0 = abs(steady_state(rec, 'wm', t_from)) * MOTOR.p / (2 * math.pi)
    ia = rec['ia'][mask]
    return dict(rec=rec, ripple=rip, thd=thd(ia, max(f0, 1.0), FS) * 100, f0=f0)


@case('转矩纹波：FOC 正弦/SVPWM < 六步方波', ['compare'])
def t_ripple_compare():
    res = {name: _run_ripple(mod) for mod, name in
           ((F.MOD_SINE, 'sine'), (F.MOD_SVPWM, 'svpwm'), (F.MOD_TRAP120, 'trap'))}
    assert res['trap']['ripple'] > 1.3 * res['sine']['ripple'], \
        '六步纹波 %.1f%% 未显著高于正弦 %.1f%%' % (res['trap']['ripple'], res['sine']['ripple'])
    assert res['sine']['ripple'] < 20.0, '正弦纹波过大 %.1f%%' % res['sine']['ripple']
    t_ripple_compare.cache = res          # 供出图复用
    return '纹波: 正弦 %.1f%% / SVPWM %.1f%% / 六步 %.1f%%' % (
        res['sine']['ripple'], res['svpwm']['ripple'], res['trap']['ripple'])


@case('电流 THD：FOC 优于六步', ['compare'])
def t_thd_compare():
    res = getattr(t_ripple_compare, 'cache', None) or {
        name: _run_ripple(mod) for mod, name in
        ((F.MOD_SINE, 'sine'), (F.MOD_SVPWM, 'svpwm'), (F.MOD_TRAP120, 'trap'))}
    assert res['trap']['thd'] > res['sine']['thd'], \
        '六步 THD %.1f%% 未高于正弦 %.1f%%' % (res['trap']['thd'], res['sine']['thd'])
    return 'THD: 正弦 %.1f%% / SVPWM %.1f%% / 六步 %.1f%%' % (
        res['sine']['thd'], res['svpwm']['thd'], res['trap']['thd'])


def _max_regulated_speed(module: int) -> float:
    """电流环仍能调节（占空比未触顶）时的最高转速。"""
    best = 0.0
    for tgt in np.arange(84.0, 150.0, 4.0):
        rec = simulate(MOTOR, cfg(controller=F.CTRL_VELOCITY, torque_mode=F.TQ_FOC_CURRENT,
                                  modulation=module, velocity_limit=400.0, current_limit=3.0),
                       1.2, target=tgt, load=0.02, dt_plant=5e-5)
        tail = slice(-200, None)
        clipped = max(float(np.max(rec[k][tail])) for k in ('dc_a', 'dc_b', 'dc_c')) > 0.995
        if clipped:
            break
        best = steady_state(rec, 'wm', -0.05)
    return best


@case('母线利用率：SVPWM 把可用转速提高 √3/2', ['compare'])
def t_max_speed():
    w_sine = _max_regulated_speed(F.MOD_SINE)
    w_svpwm = _max_regulated_speed(F.MOD_SVPWM)
    w_theory = VBUS / 2.0 / MOTOR.psi_f / MOTOR.p       # 正弦调制的电压上限对应转速
    ratio = w_svpwm / max(w_sine, 1e-9)
    assert abs(w_sine - w_theory) / w_theory < 0.05, '正弦极限 %.1f rad/s（理论 %.1f）' % (
        w_sine, w_theory)
    assert abs(ratio - 2 / math.sqrt(3)) < 0.05, '最高转速比 %.3f' % ratio
    t_max_speed.cache = dict(sine=w_sine, svpwm=w_svpwm, theory=w_theory)
    return '正弦 %.1f rad/s / SVPWM %.1f rad/s（+%.1f%%）' % (
        w_sine, w_svpwm, (ratio - 1) * 100)


# ===================================================================== 4. 鲁棒性与对比
@case('量化+噪声：12 bit 编码器、20 mA 电流噪声', ['robust'])
def t_noise():
    c = cfg(controller=F.CTRL_VELOCITY, torque_mode=F.TQ_FOC_CURRENT, encoder_bits=12,
            current_noise=0.02, angle_noise=2e-3)
    rec = simulate(MOTOR, c, 1.0, target=30.0, load=0.03, dt_plant=DT)
    ss = steady_state(rec, 'wm', rec['t'][-1] - 0.2)
    err = abs(ss - 30.0) / 30.0
    ia = rec['ia'][rec['t'] > 0.7]
    assert err < 0.02, '速度误差 %.2f%%' % (err * 100)
    assert np.std(ia) > 0, '电流噪声未生效'
    return '速度误差 %.2f%%（%.2f rad/s），相电流 RMS %.2f A' % (
        err * 100, ss, float(np.sqrt(np.mean(ia ** 2))))


@case('开环 V/f 失步 vs 闭环 FOC 跟踪', ['compare'])
def t_vf_vs_foc():
    f, vpk = 30.0, 12.0
    rec_vf = simulate_vf(MOTOR, vpk=vpk, f=f, ramp_time=0.02, t_end=0.6,
                         load=0.05, dt_plant=2e-5, damp=0.0)
    sync_rpm = 60.0 * f / MOTOR.p
    vf_rpm = float(np.mean(rec_vf['rpm'][-2000:]))
    assert vf_rpm < 0.6 * sync_rpm, 'V/f 意外同步（%.0f rpm）' % vf_rpm

    w_tgt = sync_rpm * 2 * math.pi / 60.0
    rec_foc = simulate(MOTOR, cfg(controller=F.CTRL_VELOCITY, torque_mode=F.TQ_FOC_CURRENT),
                       0.6, target=w_tgt, load=0.05, dt_plant=2e-5)
    foc_rpm = steady_state(rec_foc, 'rpm', rec_foc['t'][-1] - 0.1)
    assert abs(foc_rpm - sync_rpm) / sync_rpm < 0.02, 'FOC 未跟踪（%.0f rpm）' % foc_rpm
    t_vf_vs_foc.cache = (rec_vf, rec_foc, sync_rpm)
    return 'V/f 0.02 s 斜坡失步（%.0f rpm），FOC 稳定 %.0f rpm' % (vf_rpm, foc_rpm)


@case('梯形反电动势 + 六步换相（BLDC 模式）', ['compare'])
def t_bldc_trapezoid():
    m = MotorParams(waveform='trapezoid')
    rec = simulate(m, cfg(controller=F.CTRL_TORQUE, torque_mode=F.TQ_FOC_CURRENT,
                          modulation=F.MOD_TRAP120, current_limit=2.5),
                   0.4, target=1.2, load=0.05, dt_plant=DT)
    t_from = rec['t'][-1] - 0.12
    iq = steady_state(rec, 'iq_true', t_from)
    te = steady_state(rec, 'te', t_from)
    assert abs(iq - 1.2) < 0.15, 'iq 稳态 %.3f' % iq
    assert te > 0.15, '转矩偏小 %.3f' % te
    return 'iq=%.2f A, Te=%.3f N·m（梯形机 + 120° 方波）' % (iq, te)


@case('数值收敛：积分步长减半结果一致', ['plant'])
def t_convergence():
    c = cfg(controller=F.CTRL_VELOCITY, torque_mode=F.TQ_FOC_CURRENT)
    a = simulate(MOTOR, c, 0.5, target=25.0, load=0.05, dt_plant=1e-5)
    b = simulate(MOTOR, c, 0.5, target=25.0, load=0.05, dt_plant=5e-6)
    wa, wb = a['wm'][-1], b['wm'][-1]
    assert abs(wa - wb) / wb < 0.005, '步长敏感 %.3f%%' % (abs(wa - wb) / wb * 100)
    return 'dt=10 µs vs 5 µs：%.3f vs %.3f rad/s（差 %.2f%%）' % (
        wa, wb, abs(wa - wb) / wb * 100)


@case('电机模型：dq 稳态电流与理论解一致', ['plant'])
def t_plant_steady_state():
    """给定 dq 电压，稳态 id/iq 应满足 vd=Rs·id−ωe·L·iq、vq=Rs·iq+ωe·(L·id+ψf)。"""
    from sim.plant import MotorPlant
    plt = MotorPlant(MOTOR, dt=1e-5)
    we, vd, vq = 200.0, 0.0, 6.0
    plt.wm = we / MOTOR.p
    th = 0.0
    for _ in range(100000):          # 固定转速，直接给 dq 电压
        th += we * 1e-5
        thm = th / MOTOR.p
        plt.thm = thm
        plt.wm = we / MOTOR.p
        c, s = math.cos(th), math.sin(th)
        al, be = vd * c - vq * s, vd * s + vq * c
        va, vb, vc = al, -0.5 * al + math.sqrt(3) / 2 * be, -0.5 * al - math.sqrt(3) / 2 * be
        plt.step(va, vb, vc, 0.0)
        if _ > 50000 and _ % 20000 == 0:
            break
    id_m, iq_m = plt.dq()
    L, R, psi = MOTOR.L, MOTOR.Rs, MOTOR.psi_f
    # 解稳态方程：vd = R·id − we·L·iq ; vq = R·iq + we·L·id + we·ψf
    A = np.array([[R, -we * L], [we * L, R]])
    b = np.array([vd, vq - we * psi])
    id_r, iq_r = np.linalg.solve(A, b)
    assert abs(id_m - id_r) < 0.05 * abs(id_r) + 0.02, 'id %.3f vs 理论 %.3f' % (id_m, id_r)
    assert abs(iq_m - iq_r) < 0.05 * abs(iq_r) + 0.02, 'iq %.3f vs 理论 %.3f' % (iq_m, iq_r)
    return 'id=%.3f/%.3f, iq=%.3f/%.3f A（仿真/理论）' % (id_m, id_r, iq_m, iq_r)
