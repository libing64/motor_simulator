# -*- coding: utf-8 -*-
"""
仿真测试台（plant + controller 闭环）
====================================

每个控制周期：

    1. 采样：三相电流、转子机械角、机械角速度
    2. SimpleFOC：loopFOC() + move()  ->  三相电压指令
    3. 占空比 -> 相电压 (dc * Vbus)，推入电机模型积分一个控制周期

也提供传统开环 V/f 驱动，用于和闭环 FOC 做对比实验。
"""

from __future__ import annotations

import math
from typing import Callable, Dict, Optional, Union

import numpy as np

from .foc import FocController, ControllerConfig, pwm_to_volts
from .plant import MotorParams, MotorPlant

Signal = Union[float, Callable[[float], float]]


def _sig(x: Signal, t: float) -> float:
    return x(t) if callable(x) else float(x)


def simulate(motor: MotorParams,
             cfg: ControllerConfig,
             t_end: float,
             target: Signal = 0.0,
             load: Signal = 0.0,
             dt_plant: float = 1.0e-5,
             plant: Optional[MotorPlant] = None,
             controller: Optional[FocController] = None,
             warm_start: bool = False,
             speed_lock: Optional[float] = None) -> Dict[str, np.ndarray]:
    """闭环 FOC 仿真。

    Args:
        target: 控制器目标（转矩模式的 q 电流 / 速度模式 rad/s / 位置模式 rad）
        load:   负载转矩 [N·m]
        dt_plant: 电机模型积分步长（控制周期内自动细分为整数步）
        speed_lock: 给定转速时等效为测功机（转速被强制锁定），
                    用于测量**电磁转矩纹波**和稳态电流波形
    """
    ctrl = controller or FocController(motor, cfg)
    plt = plant or MotorPlant(motor, dt=dt_plant)
    if not warm_start:
        plt.reset()

    Tp = ctrl.cfg.period
    n = int(round(t_end / Tp))

    rec = {k: np.zeros(n) for k in (
        't', 'ia', 'ib', 'ic', 'id', 'iq', 'id_true', 'iq_true', 'wm', 'rpm',
        'te', 'tl', 'target', 'current_sp', 'uq', 'ud', 'theta_e',
        'dc_a', 'dc_b', 'dc_c', 'ua', 'ub', 'uc', 'vq_ref', 'shaft_angle')}

    for k in range(n):
        t = k * Tp
        if speed_lock is not None:
            plt.wm = speed_lock
        iabc = plt.phase_currents()
        st = ctrl.step(iabc, plt.thm, plt.wm, _sig(target, t))
        va, vb, vc = pwm_to_volts(st, ctrl.cfg.vbus)
        tl = _sig(load, t)
        plt.advance(va, vb, vc, Tp, tl=tl)
        if speed_lock is not None:
            plt.wm = speed_lock

        idt, iqt = plt.dq()
        rec['t'][k] = t
        rec['ia'][k], rec['ib'][k], rec['ic'][k] = plt.ia, plt.ib, plt.ic
        rec['id'][k], rec['iq'][k] = st.id, st.iq
        rec['id_true'][k], rec['iq_true'][k] = idt, iqt
        rec['wm'][k] = plt.wm
        rec['rpm'][k] = plt.rpm
        rec['te'][k] = plt.t_e
        rec['tl'][k] = tl
        rec['target'][k] = _sig(target, t)
        rec['current_sp'][k] = st.current_sp
        rec['uq'][k], rec['ud'][k] = st.uq, st.ud
        rec['theta_e'][k] = plt.theta_e
        rec['dc_a'][k], rec['dc_b'][k], rec['dc_c'][k] = st.dc_a, st.dc_b, st.dc_c
        rec['ua'][k], rec['ub'][k], rec['uc'][k] = va, vb, vc
        rec['vq_ref'][k] = st.uq
        rec['shaft_angle'][k] = plt.thm

    rec['params'] = motor  # type: ignore[assignment]
    rec['cfg'] = cfg       # type: ignore[assignment]
    return rec


def simulate_vf(motor: MotorParams,
                vpk: float,
                f: float,
                ramp_time: float,
                t_end: float,
                load: Signal = 0.0,
                dt_plant: float = 1.0e-5,
                damp: float = 0.01) -> Dict[str, np.ndarray]:
    """传统开环 V/f 驱动：频率/幅值从 0 斜坡上升，无位置反馈。

    `damp` 模拟阻尼绕组在转差下提供的异步转矩，帮助牵入同步
    （设为 0 时，斜坡过快会失步）。
    """
    plt = MotorPlant(motor, dt=dt_plant)
    n = int(round(t_end / dt_plant))
    rec = {k: np.zeros(n) for k in ('t', 'ia', 'ib', 'ic', 'wm', 'rpm', 'te',
                                    'tl', 'theta_v', 'id', 'iq', 'sync_rpm')}
    theta_v = 0.0
    for k in range(n):
        t = k * dt_plant
        ramp = min(1.0, t / ramp_time) if ramp_time > 0 else 1.0
        f_i = f * ramp
        v_i = vpk * ramp
        wv = 2.0 * math.pi * f_i
        theta_v += wv * dt_plant
        va = v_i * math.cos(theta_v)
        vb = v_i * math.cos(theta_v - 2.0 * math.pi / 3.0)
        vc = v_i * math.cos(theta_v + 2.0 * math.pi / 3.0)
        tl = _sig(load, t)
        t_damp = damp * (wv - plt.omega_e)
        plt.advance(va, vb, vc, dt_plant, tl=tl, t_extra=t_damp)

        idt, iqt = plt.dq()
        rec['t'][k] = t
        rec['ia'][k], rec['ib'][k], rec['ic'][k] = plt.ia, plt.ib, plt.ic
        rec['wm'][k] = plt.wm
        rec['rpm'][k] = plt.rpm
        rec['te'][k] = plt.t_e
        rec['tl'][k] = tl
        rec['theta_v'][k] = theta_v
        rec['id'][k], rec['iq'][k] = idt, iqt
        rec['sync_rpm'][k] = 60.0 * f_i / motor.p
    return rec


# ---------------------------------------------------------------- 指标
def steady_state(rec: Dict[str, np.ndarray], key: str, t_from: float = -1.0,
                 t_to: float = 1e9) -> float:
    t = rec['t']
    mask = (t >= (t[-1] + t_from if t_from < 0 else t_from)) & (t <= t_to)
    return float(np.mean(rec[key][mask]))


def ripple_ptp(rec: Dict[str, np.ndarray], key: str, t_from: float = -1.0) -> float:
    t = rec['t']
    mask = t >= (t[-1] + t_from if t_from < 0 else t_from)
    v = rec[key][mask]
    return float(np.max(v) - np.min(v))


def thd(signal: np.ndarray, f0: float, fs: float, n_harm: int = 10) -> float:
    """总谐波畸变率：最小二乘拟合基波后，残差 RMS / 基波 RMS。

    不依赖 FFT 频率分辨率，短窗口下也稳定（仿真里一个窗口常只有几个电周期）。
    """
    x = np.asarray(signal, dtype=float)
    x = x - x.mean()
    n = len(x)
    t = np.arange(n) / fs
    w = 2.0 * math.pi * f0
    A = np.column_stack([np.cos(w * t), np.sin(w * t)])
    coef, *_ = np.linalg.lstsq(A, x, rcond=None)
    fund = A @ coef
    resid = x - fund
    rms_f = float(np.sqrt(np.mean(fund ** 2)))
    if rms_f <= 1e-12:
        return float('inf')
    return float(np.sqrt(np.mean(resid ** 2)) / rms_f)
