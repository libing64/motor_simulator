#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Build the native bridge that puts SimpleFOC inside the simulator.

Compiles
    sim/native/foc_sim.cpp              (the bridge: driver/sensor/current-sense adapters)
    sim/platform/src/arduino_native.cpp  (virtual clock + Arduino stubs)
    third_party/Arduino-FOC/src/**   (unmodified library sources)
into
    sim/native/libfocsim.so

Usage:
    python3 -m sim.build          # build once
    python3 -m sim.build --clean  # force a rebuild
"""

import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SFOC = os.path.join(ROOT, 'third_party', 'Arduino-FOC', 'src')
OUT = os.path.join(ROOT, 'sim', 'native', 'libfocsim.so')

# Sources of the library that are algorithm-only (no MCU drivers, no current
# sense hardware back-ends, no communication front-ends).
LIB_SOURCES = [
    'BLDCMotor.cpp',
    'common/base_classes/FOCMotor.cpp',
    'common/base_classes/Sensor.cpp',
    'common/base_classes/CurrentSense.cpp',
    'common/pid.cpp',
    'common/lowpass_filter.cpp',
    'common/foc_utils.cpp',
    'common/time_utils.cpp',
    'communication/SimpleFOCDebug.cpp',
]

BRIDGE_SOURCES = [
    os.path.join(ROOT, 'sim', 'native', 'foc_sim.cpp'),
    os.path.join(ROOT, 'sim', 'platform', 'src', 'arduino_native.cpp'),
]


def build(force: bool = False) -> str:
    if force and os.path.exists(OUT):
        os.remove(OUT)
    if os.path.exists(OUT) and not force:
        srcs = BRIDGE_SOURCES + [os.path.join(SFOC, s) for s in LIB_SOURCES]
        newest = max(os.path.getmtime(p) for p in srcs)
        if os.path.getmtime(OUT) >= newest:
            return OUT

    cmd = [
        os.environ.get('CXX', 'g++'), '-O2', '-std=c++17', '-fPIC', '-shared',
        '-Wno-unused-parameter', '-Wno-delete-non-virtual-dtor',
        '-I', os.path.join(ROOT, 'sim', 'platform', 'include'),
        '-I', os.path.join(ROOT, 'sim', 'native'),
        '-I', SFOC,
    ]
    cmd += BRIDGE_SOURCES
    cmd += [os.path.join(SFOC, s) for s in LIB_SOURCES]
    cmd += ['-o', OUT, '-lm']

    print('[build] ' + ' '.join(cmd[:6]) + ' ...')
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode != 0:
        sys.stderr.write(res.stdout + res.stderr)
        raise RuntimeError('build failed')
    payload = res.stderr.strip()
    if payload:
        # only surface warnings coming from our own code
        for line in payload.splitlines():
            if line.startswith('sim/') or line.startswith('In file'):
                print('[build] ' + line)
    return OUT


if __name__ == '__main__':
    path = build(force='--clean' in sys.argv)
    print('[build] OK -> %s' % path)
