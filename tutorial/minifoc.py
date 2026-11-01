# -*- coding: utf-8 -*-
"""教学用的最小 FOC 实现（约 120 行）。

故意写得"笨"一点：每一步都对应课本上的一条公式，方便对着注释读。
生产代码请用 `sim/foc.py`（SimpleFOC 库），这里只是让你看清原理。

    iabc --Clarke--> iαβ --Park--> id/iq --PI--> ud/uq --反Park--> uαβ --SVPWM--> 占空比
"""

from __future__ import annotations

import math

_SQRT3 = math.sqrt(3.0)


def clarke(ia: float, ib: float, ic: float):
    """等幅值 Clarke：三相 -> 静止两相。"""
    return (2.0 / 3.0) * (ia - 0.5 * ib - 0.5 * ic), (ib - ic) / _SQRT3


def park(ialpha: float, ibeta: float, theta: float):
    """Park：静止两相 -> 随转子旋转的 dq。"""
    c, s = math.cos(theta), math.sin(theta)
    return ialpha * c + ibeta * s, -ialpha * s + ibeta * c


def inv_park(ud: float, uq: float, theta: float):
    """反 Park：dq -> 静止两相。"""
    c, s = math.cos(theta), math.sin(theta)
    return ud * c - uq * s, ud * s + uq * c


def svpwm(ualpha: float, ubeta: float, vbus: float, vmax: float = None):
    """空间矢量调制：注入零序，把三相占空比压进 [0, 1]。

    返回 (da, db, dc)。vmax 给定时先把矢量幅值按比例缩到 vmax 以内，
    相当于"电压圆限幅"（防止过调制）。
    """
    mag = math.hypot(ualpha, ubeta)
    if vmax is not None and mag > vmax > 0.0:
        k = vmax / mag
        ualpha, ubeta = ualpha * k, ubeta * k

    # 反 Clarke 得到三相电压
    va = ualpha
    vb = -0.5 * ualpha + (_SQRT3 / 2.0) * ubeta
    vc = -0.5 * ualpha - (_SQRT3 / 2.0) * ubeta

    # 零序注入：把 (max+min)/2 拉到 0，占空比就不会超出母线
    voff = -0.5 * (max(va, vb, vc) + min(va, vb, vc))
    da = 0.5 + (va + voff) / vbus
    db = 0.5 + (vb + voff) / vbus
    dc = 0.5 + (vc + voff) / vbus
    return min(max(da, 0.0), 1.0), min(max(db, 0.0), 1.0), min(max(dc, 0.0), 1.0)


class MiniFOC:
    """两轴 PI + SVPWM 的电流环（id 给定 0，iq 给定转矩）。"""

    def __init__(self, vbus: float = 24.0, kp: float = 3.0, ki: float = 300.0,
                 voltage_limit: float = None):
        self.vbus = vbus
        self.kp = kp
        self.ki = ki
        # 电压圆半径：SVPWM 线性区上限 = Vbus/√3
        self.vlim = voltage_limit if voltage_limit else vbus / _SQRT3
        self.reset()

    def reset(self) -> None:
        self.int_d = 0.0
        self.int_q = 0.0

    def _pi(self, err: float, integ: float, dt: float):
        """PI + 抗饱和：输出撞到限幅时停止积分。"""
        out = self.kp * err + integ
        if -self.vlim < out < self.vlim:
            integ += self.ki * err * dt
        return out, integ

    def step(self, iabc, theta_e: float, iq_ref: float, id_ref: float = 0.0,
             dt: float = 1.0 / 20000.0):
        ia, ib, ic = iabc
        ialpha, ibeta = clarke(ia, ib, ic)
        i_d, i_q = park(ialpha, ibeta, theta_e)

        ud, self.int_d = self._pi(id_ref - i_d, self.int_d, dt)
        uq, self.int_q = self._pi(iq_ref - i_q, self.int_q, dt)

        # 电压矢量整体限幅（保持方向）
        mag = math.hypot(ud, uq)
        if mag > self.vlim:
            ud, uq = ud * self.vlim / mag, uq * self.vlim / mag

        ualpha, ubeta = inv_park(ud, uq, theta_e)
        da, db, dc = svpwm(ualpha, ubeta, self.vbus, self.vlim)
        return dict(id=i_d, iq=i_q, ud=ud, uq=uq, da=da, db=db, dc=dc,
                    ua=da * self.vbus, ub=db * self.vbus, uc=dc * self.vbus)
