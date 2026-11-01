# -*- coding: utf-8 -*-
"""
相域 PMSM / BLDC 电机模型（被控对象）
===================================

与 `motor_sim.py` 的 dq 模型不同，这里是**三相相域**模型，因此可以直接施加
任意波形（正弦、SVPWM、六步方波）并观察真实的反电动势形状对转矩纹波的影响。

状态方程（每相 k，θk = θe - k·2π/3）：

    L·dik/dt = vk - vn - R·ik - ψf·ωe·e(θk)
    J·dωm/dt = Te + Tdamp - TL - B·ωm
    dθm/dt   = ωm,   θe = p·θm
    Te       = p·ψf·Σ ik·e(θk)           （能量法，与波形无关）

其中 e(θ) 是单位化的反电动势形状函数：
    - 'sine'      : e = -sin(θ)，对应 λ = ψf·cos(θ)（表贴式 PMSM）
    - 'trapezoid' : 平顶 120° 的梯形波，对应方波驱动的 BLDC

中性点 vn = (va+vb+vc)/3（三相平衡，反电动势三次谐波为零）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Tuple

_TWO_PI = 2.0 * math.pi
_THIRD = _TWO_PI / 3.0


def _wrap(x: float) -> float:
    x = math.fmod(x, _TWO_PI)
    return x + _TWO_PI if x < 0.0 else x


def emf_sine(th: float) -> float:
    """单位反电动势形状（正弦机）：e = -sin(θ)。"""
    return -math.sin(th)


def flux_sine(th: float) -> float:
    """单位磁链形状（正弦机）：g = cos(θ)，g' = e。"""
    return math.cos(th)


def _trap(u: float) -> float:
    """周期 2π 的单位梯形波：平顶 ±1（各 120°），过渡段 60° 线性。"""
    u = _wrap(u)
    p3 = math.pi / 3.0
    if u < p3:
        return 1.0
    if u < 2.0 * p3:
        return 1.0 - 6.0 * (u - p3) / math.pi
    if u < 4.0 * p3:
        return -1.0
    if u < 5.0 * p3:
        return -1.0 + 6.0 * (u - 4.0 * p3) / math.pi
    return 1.0


def flux_trapezoid(th: float) -> float:
    """单位磁链形状（梯形机）：_trap 的积分并归一化到峰值 1。

    与正弦机保持一致：ψf 表示**磁链峰值**，因此反电动势平顶幅值为
    (12/5π) ≈ 0.764 倍的 ψf·ωe。
    """
    u = _wrap(th + math.pi / 2.0)      # 与 -sin 对齐：θ=0 处过零下降
    p3 = math.pi / 3.0
    if u < p3:
        g = u
    elif u < 2.0 * p3:
        d = u - p3
        g = p3 + d - 3.0 * d * d / math.pi
    elif u < 4.0 * p3:
        g = p3 - (u - 2.0 * p3)
    elif u < 5.0 * p3:
        d = u - 4.0 * p3
        g = -p3 - d + 3.0 * d * d / math.pi
    else:
        g = -p3 + (u - 5.0 * p3)
    return g * 12.0 / (5.0 * math.pi)


def emf_trapezoid(th: float) -> float:
    """单位反电动势形状（梯形机，平顶 120°）：flux_trapezoid 的导数。"""
    return (12.0 / (5.0 * math.pi)) * _trap(th + math.pi / 2.0)


_EMF = {'sine': emf_sine, 'trapezoid': emf_trapezoid}
_FLUX = {'sine': flux_sine, 'trapezoid': flux_trapezoid}


@dataclass
class MotorParams:
    Rs: float = 0.8e0        # 定子相电阻 [Ω]
    L: float = 2.0e-3        # 相电感（表贴式 Ld=Lq）[H]
    psi_f: float = 0.03      # 永磁磁链峰值 [Wb]
    p: int = 4               # 极对数
    J: float = 3.0e-3        # 转动惯量 [kg·m²]
    B: float = 3.0e-4        # 粘滞摩擦 [N·m·s/rad]
    D_damp: float = 0.0      # 阻尼绕组系数（开环 V/f 启动时用）
    waveform: str = 'sine'   # 'sine' | 'trapezoid'

    @property
    def ke(self) -> float:
        """反电动势常数：相电压峰值 / 机械角速度 [V·s/rad]。"""
        return self.psi_f * self.p

    @property
    def kt(self) -> float:
        """转矩常数 [N·m/A]（正弦机，按 q 轴电流定义）。"""
        return 1.5 * self.p * self.psi_f

    @property
    def kv_rpm_per_v(self) -> float:
        """SimpleFOC 使用的 KV 定义（rpm/V，按机械转速与相电压峰值）。"""
        return 60.0 / (_TWO_PI * self.p * self.psi_f)


class MotorPlant:
    """三相电机 + 机械负载的数值模型（RK4 积分）。"""

    def __init__(self, params: MotorParams, dt: float = 1.0e-5):
        self.prm = params
        self.dt = dt
        self._emf = _EMF[params.waveform]

        self.ia = 0.0
        self.ib = 0.0
        self.ic = 0.0
        self.wm = 0.0       # 机械角速度 [rad/s]
        self.thm = 0.0      # 机械角 [rad]
        self.tl = 0.0       # 当前负载转矩 [N·m]
        self.t_e = 0.0      # 上一拍电磁转矩 [N·m]
        self.time = 0.0

    # ------------------------------------------------------------ 输出
    @property
    def theta_e(self) -> float:
        return self.prm.p * self.thm

    @property
    def omega_e(self) -> float:
        return self.prm.p * self.wm

    @property
    def rpm(self) -> float:
        return self.wm * 60.0 / _TWO_PI

    def phase_currents(self) -> Tuple[float, float, float]:
        return self.ia, self.ib, self.ic

    def alpha_beta(self) -> Tuple[float, float]:
        a, b, c = self.ia, self.ib, self.ic
        return (2.0 / 3.0) * (a - 0.5 * b - 0.5 * c), (b - c) / math.sqrt(3.0)

    def dq(self) -> Tuple[float, float]:
        """测量用 dq 电流（与 SimpleFOC 相同的等幅值 Clarke/Park）。"""
        al, be = self.alpha_beta()
        th = self.theta_e
        c, s = math.cos(th), math.sin(th)
        return al * c + be * s, -al * s + be * c

    # ------------------------------------------------------------ 内部
    def _deriv(self, ia, ib, ic, wm, thm, va, vb, vc, t_extra=0.0):
        prm = self.prm
        R, L, psif, p = prm.Rs, prm.L, prm.psi_f, prm.p
        we = p * wm
        th = p * thm
        vn = (va + vb + vc) / 3.0
        e = self._emf
        dia = (va - vn - R * ia - psif * we * e(th)) / L
        dib = (vb - vn - R * ib - psif * we * e(th - _THIRD)) / L
        dic = (vc - vn - R * ic - psif * we * e(th + _THIRD)) / L
        te = p * psif * (ia * e(th) + ib * e(th - _THIRD) + ic * e(th + _THIRD))
        dwm = (te + t_extra - self.tl - prm.B * wm) / prm.J
        return dia, dib, dic, dwm, wm, te

    def step(self, va: float, vb: float, vc: float, tl: float = 0.0,
             t_extra: float = 0.0) -> None:
        """推进一个积分步（RK4）。t_extra 为附加转矩（如阻尼绕组）。"""
        self.tl = tl
        dt = self.dt
        ia, ib, ic, wm, thm = self.ia, self.ib, self.ic, self.wm, self.thm

        k1 = self._deriv(ia, ib, ic, wm, thm, va, vb, vc, t_extra)
        k2 = self._deriv(ia + 0.5 * dt * k1[0], ib + 0.5 * dt * k1[1], ic + 0.5 * dt * k1[2],
                         wm + 0.5 * dt * k1[3], thm + 0.5 * dt * k1[4], va, vb, vc, t_extra)
        k3 = self._deriv(ia + 0.5 * dt * k2[0], ib + 0.5 * dt * k2[1], ic + 0.5 * dt * k2[2],
                         wm + 0.5 * dt * k2[3], thm + 0.5 * dt * k2[4], va, vb, vc, t_extra)
        k4 = self._deriv(ia + dt * k3[0], ib + dt * k3[1], ic + dt * k3[2],
                         wm + dt * k3[3], thm + dt * k3[4], va, vb, vc, t_extra)

        ia += dt / 6.0 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0])
        ib += dt / 6.0 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1])
        ic += dt / 6.0 * (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2])
        wm += dt / 6.0 * (k1[3] + 2 * k2[3] + 2 * k3[3] + k4[3])
        thm += dt / 6.0 * (k1[4] + 2 * k2[4] + 2 * k3[4] + k4[4])

        # 星形连接：三相电流和必须为零，用去均值抑制数值漂移
        mean = (ia + ib + ic) / 3.0
        self.ia, self.ib, self.ic = ia - mean, ib - mean, ic - mean
        self.wm, self.thm = wm, thm
        self.t_e = k1[5]
        self.time += dt

    def advance(self, va: float, vb: float, vc: float, duration: float,
                tl: float = 0.0, t_extra: float = 0.0) -> None:
        """在 duration 内保持三相电压不变，按 dt 细分积分。"""
        n = int(round(duration / self.dt))
        if n < 1:
            n = 1
        h = duration / n
        if abs(h - self.dt) > 1e-15:
            old, self.dt = self.dt, h
            for _ in range(n):
                self.step(va, vb, vc, tl, t_extra)
            self.dt = old
        else:
            for _ in range(n):
                self.step(va, vb, vc, tl, t_extra)

    def reset(self) -> None:
        self.ia = self.ib = self.ic = 0.0
        self.wm = self.thm = 0.0
        self.t_e = 0.0
        self.time = 0.0
        self.tl = 0.0
