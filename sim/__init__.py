# -*- coding: utf-8 -*-
"""Simulation package: motor plant + SimpleFOC-in-the-loop testbench."""

from .plant import MotorParams, MotorPlant  # noqa: F401
from .foc import FocController               # noqa: F401

__all__ = ['MotorParams', 'MotorPlant', 'FocController']
