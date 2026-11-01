# -*- coding: utf-8 -*-
"""
SimpleFOC 控制器接口（ctypes 绑定）
==================================

把 `sim/native/libfocsim.so` 里的 C ABI 包装成 Python 类。仿真器每一步：

    采样电流/角度/转速  ->  SimpleFOC loopFOC() + move()  ->  三相电压

SimpleFOC 自身的硬件抽象（BLCMMotor 的驱动接口、传感器、电流采样）由桥接层用
仿真信号替代，因此库代码本身（FOC 变换、PID、SVPWM、六步换相、限幅）完全未经修改。
"""

from __future__ import annotations

import ctypes
import os
import sys
from dataclasses import dataclass
from typing import Optional, Tuple

from .plant import MotorParams

_HERE = os.path.dirname(os.path.abspath(__file__))
_LIB_NAME = 'libfocsim.so' if sys.platform != 'darwin' else 'libfocsim.dylib'
_LIB_PATH = os.path.join(_HERE, 'native', _LIB_NAME)

# ---- 与 foc_sim.h 保持一致 ----
CTRL_TORQUE = 0
CTRL_VELOCITY = 1
CTRL_ANGLE = 2
CTRL_VELOCITY_OPENLOOP = 3
CTRL_ANGLE_OPENLOOP = 4

TQ_VOLTAGE = 0
TQ_DC_CURRENT = 1
TQ_FOC_CURRENT = 2
TQ_ESTIMATED_CURRENT = 3

MOD_SINE = 0
MOD_SVPWM = 1
MOD_TRAP120 = 2
MOD_TRAP150 = 3

PID_VELOCITY = 0
PID_CURRENT_Q = 1
PID_CURRENT_D = 2
PID_ANGLE = 3


class FocParams(ctypes.Structure):
    _fields_ = [
        ('pole_pairs', ctypes.c_int),
        ('phase_resistance', ctypes.c_double),
        ('phase_inductance', ctypes.c_double),
        ('kv_rating', ctypes.c_double),
        ('vbus', ctypes.c_double),
        ('voltage_limit', ctypes.c_double),
        ('current_limit', ctypes.c_double),
        ('velocity_limit', ctypes.c_double),
        ('control_period_us', ctypes.c_double),
        ('encoder_bits', ctypes.c_int),
        ('current_noise_std', ctypes.c_double),
        ('angle_noise_std', ctypes.c_double),
        ('noise_seed', ctypes.c_uint),
        ('zero_electric_angle', ctypes.c_double),
    ]


class FocState(ctypes.Structure):
    _fields_ = [
        ('ua', ctypes.c_double), ('ub', ctypes.c_double), ('uc', ctypes.c_double),
        ('dc_a', ctypes.c_double), ('dc_b', ctypes.c_double), ('dc_c', ctypes.c_double),
        ('id', ctypes.c_double), ('iq', ctypes.c_double),
        ('current_sp', ctypes.c_double),
        ('ud', ctypes.c_double), ('uq', ctypes.c_double),
        ('shaft_angle', ctypes.c_double), ('shaft_velocity', ctypes.c_double),
        ('electrical_angle', ctypes.c_double),
        ('voltage_bemf', ctypes.c_double),
        ('enabled', ctypes.c_int),
    ]


_lib = None


def load_library(rebuild: bool = False):
    """加载（必要时先编译）桥接动态库。"""
    global _lib
    if _lib is not None and not rebuild:
        return _lib
    if rebuild or not os.path.exists(_LIB_PATH):
        from . import build
        build.build(force=True)

    lib = ctypes.CDLL(_LIB_PATH)
    lib.foc_create.restype = ctypes.c_void_p
    lib.foc_create.argtypes = []
    lib.foc_init.restype = ctypes.c_int
    lib.foc_init.argtypes = [ctypes.c_void_p, ctypes.POINTER(FocParams)]
    lib.foc_destroy.argtypes = [ctypes.c_void_p]
    lib.foc_set_controller.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.foc_set_torque_mode.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.foc_set_modulation.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int]
    lib.foc_set_target.argtypes = [ctypes.c_void_p, ctypes.c_double]
    lib.foc_set_pid.argtypes = [ctypes.c_void_p, ctypes.c_int] + [ctypes.c_double] * 5
    lib.foc_set_limits.argtypes = [ctypes.c_void_p] + [ctypes.c_double] * 3
    lib.foc_set_lpf_velocity.argtypes = [ctypes.c_void_p, ctypes.c_double]
    lib.foc_enable.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.foc_step.restype = ctypes.c_int
    lib.foc_step.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_double),
                             ctypes.c_double, ctypes.c_double, ctypes.c_double,
                             ctypes.POINTER(FocState)]
    lib.foc_step_keep_target.restype = ctypes.c_int
    lib.foc_step_keep_target.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_double),
                                         ctypes.c_double, ctypes.c_double,
                                         ctypes.POINTER(FocState)]
    lib.foc_get_status.restype = ctypes.c_int
    lib.foc_get_status.argtypes = [ctypes.c_void_p]
    _lib = lib
    return lib


@dataclass
class ControllerConfig:
    """控制回路配置（默认值对应 24 V / 4 极对 / 2 mH 的小功率电机）。"""
    vbus: float = 24.0
    voltage_limit: float = 24.0
    current_limit: float = 3.0
    velocity_limit: float = 100.0
    control_freq: float = 20000.0     # 控制/PWM 频率 [Hz]
    controller: int = CTRL_TORQUE
    torque_mode: int = TQ_FOC_CURRENT
    modulation: int = MOD_SVPWM
    centered: bool = True
    pid_current_q: Tuple[float, float, float] = (3.0, 300.0, 0.0)
    pid_current_d: Tuple[float, float, float] = (3.0, 300.0, 0.0)
    pid_velocity: Tuple[float, float, float] = (0.5, 10.0, 0.0)
    p_angle: Tuple[float, float, float] = (8.0, 0.0, 0.0)
    velocity_ramp: float = 1e4         # PI 输出斜坡 [V/s]
    lpf_velocity_Tf: float = 0.01      # 速度低通时间常数 [s]
    encoder_bits: int = 0
    current_noise: float = 0.0
    angle_noise: float = 0.0
    noise_seed: int = 12345

    @property
    def period(self) -> float:
        return 1.0 / self.control_freq


class FocController:
    """SimpleFOC BLDCMotor 的仿真封装（一步 = 一个控制周期）。"""

    def __init__(self, motor: MotorParams, cfg: Optional[ControllerConfig] = None):
        self.motor = motor
        self.cfg = cfg or ControllerConfig()
        self._lib = load_library()
        self._handle = self._lib.foc_create()
        if not self._handle:
            raise RuntimeError('foc_create failed')

        p = FocParams()
        p.pole_pairs = motor.p
        p.phase_resistance = motor.Rs
        p.phase_inductance = motor.L
        p.kv_rating = motor.kv_rpm_per_v
        p.vbus = self.cfg.vbus
        p.voltage_limit = self.cfg.voltage_limit
        p.current_limit = self.cfg.current_limit
        p.velocity_limit = self.cfg.velocity_limit
        p.control_period_us = 1e6 / self.cfg.control_freq
        p.encoder_bits = self.cfg.encoder_bits
        p.current_noise_std = self.cfg.current_noise
        p.angle_noise_std = self.cfg.angle_noise
        p.noise_seed = self.cfg.noise_seed
        p.zero_electric_angle = 0.0

        if not self._lib.foc_init(self._handle, ctypes.byref(p)):
            raise RuntimeError('SimpleFOC motor init failed')

        self.set_controller(self.cfg.controller)
        self.set_torque_mode(self.cfg.torque_mode)
        self.set_modulation(self.cfg.modulation, self.cfg.centered)
        pids = [
            (PID_CURRENT_Q, self.cfg.pid_current_q, 0.0, self.cfg.voltage_limit),
            (PID_CURRENT_D, self.cfg.pid_current_d, 0.0, self.cfg.voltage_limit),
            (PID_VELOCITY, self.cfg.pid_velocity, self.cfg.velocity_ramp, self.cfg.current_limit),
            (PID_ANGLE, self.cfg.p_angle, 0.0, self.cfg.velocity_limit),
        ]
        for which, (kp, ki, kd), ramp, limit in pids:
            self._lib.foc_set_pid(self._handle, which, kp, ki, kd, ramp, limit)
        self._lib.foc_set_lpf_velocity(self._handle, self.cfg.lpf_velocity_Tf)

        self._iabc = (ctypes.c_double * 3)()
        self._state = FocState()

    # ------------------------------------------------------------ 配置
    def set_controller(self, mode: int) -> None:
        self._lib.foc_set_controller(self._handle, mode)

    def set_torque_mode(self, mode: int) -> None:
        self._lib.foc_set_torque_mode(self._handle, mode)

    def set_modulation(self, mod: int, centered: bool = True) -> None:
        self._lib.foc_set_modulation(self._handle, mod, 1 if centered else 0)

    def set_pid(self, which: int, kp: float, ki: float, kd: float = 0.0,
                ramp: float = 1e4, limit: float = 24.0) -> None:
        self._lib.foc_set_pid(self._handle, which, kp, ki, kd, ramp, limit)

    def set_limits(self, voltage: float = 0.0, current: float = 0.0, velocity: float = 0.0) -> None:
        self._lib.foc_set_limits(self._handle, voltage, current, velocity)

    def enable(self, on: bool = True) -> None:
        self._lib.foc_enable(self._handle, 1 if on else 0)

    @property
    def status(self) -> int:
        return self._lib.foc_get_status(self._handle)

    # ------------------------------------------------------------ 单步
    def step(self, iabc: Tuple[float, float, float], shaft_angle: float,
             shaft_velocity: float, target: float) -> FocState:
        self._iabc[0], self._iabc[1], self._iabc[2] = iabc
        ok = self._lib.foc_step(self._handle, self._iabc, shaft_angle,
                                shaft_velocity, target, ctypes.byref(self._state))
        if not ok:
            raise RuntimeError('foc_step failed')
        return self._state

    # ------------------------------------------------------------ 便捷
    def period_us(self) -> float:
        return 1e6 / self.cfg.control_freq

    def close(self) -> None:
        if getattr(self, '_handle', None):
            self._lib.foc_destroy(self._handle)
            self._handle = None

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass


def pwm_to_volts(state: FocState, vbus: float) -> Tuple[float, float, float]:
    """占空比 -> 实际施加的相电压（桥臂输出，0..vbus）。"""
    return state.dc_a * vbus, state.dc_b * vbus, state.dc_c * vbus
