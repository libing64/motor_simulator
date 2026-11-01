# -*- coding: utf-8 -*-
"""教程公共工具：路径、中文绘图、版式化输出。

所有实验脚本都以它开头：

    import tutlib as T
    T.banner('01', '三相电流与旋转磁场')

设计原则：每个脚本独立可运行、只产出一张图、把结论打印在终端里，
方便边读代码边看现象。
"""

from __future__ import annotations

import os
import sys

TUT = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TUT)
for _p in (ROOT, TUT):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from sim.plotstyle import setup_cjk          # noqa: E402

setup_cjk()

import matplotlib.pyplot as plt               # noqa: E402
import numpy as np                            # noqa: E402

OUT = os.path.join(TUT, 'out')
os.makedirs(OUT, exist_ok=True)

# 统一配色（与主报告一致）
C = dict(blue='#1f77b4', red='#d62728', green='#2ca02c',
         orange='#ff7f0e', purple='#9467bd', gray='#7f7f7f',
         teal='#17becf', olive='#bcbd22')

_RULE = '=' * 70


# ------------------------------------------------------------------ 输出
def banner(no: str, title: str, subtitle: str = '') -> None:
    print('\n' + _RULE)
    print(' 实验 %s · %s' % (no, title))
    if subtitle:
        print(' ' + subtitle)
    print(_RULE)


def h(text: str) -> None:
    print('\n【%s】' % text)


def p(*lines: str) -> None:
    for line in lines:
        print(line)


def bullet(*lines: str) -> None:
    for line in lines:
        print('  · ' + line)


def formula(*lines: str) -> None:
    print()
    for line in lines:
        print('      ' + line)


def note(text: str) -> None:
    print('\n  [注意] ' + text)


def questions(*lines: str) -> None:
    h('思考题')
    for i, line in enumerate(lines, 1):
        print('  %d. %s' % (i, line))


def done(figure_paths) -> None:
    print('\n' + '-' * 70)
    for path in np.atleast_1d(figure_paths):
        print('图已保存: %s' % os.path.relpath(str(path), ROOT))
    print('-' * 70 + '\n')


# ------------------------------------------------------------------ 绘图
def fig(rows: int = 2, cols: int = 2, size=None, suptitle: str = ''):
    """建图并返回 (fig, axes 一维数组)。"""
    if size is None:
        size = (11.0, 2.9 * rows + 0.7)
    f, axs = plt.subplots(rows, cols, figsize=size)
    if suptitle:
        f.suptitle(suptitle, fontsize=12.5, fontweight='bold')
    return f, np.atleast_1d(axs).ravel()


def save(f, name: str) -> str:
    path = os.path.join(OUT, name)
    f.tight_layout()
    f.savefig(path, dpi=130, bbox_inches='tight')
    plt.close(f)
    return path


def mark(ax, x, y, text, color=None, dx=0.0, dy=0.0, fs=8):
    """在曲线上标注一个点。"""
    color = color or C['red']
    ax.plot([x], [y], 'o', ms=4, color=color, zorder=5)
    ax.annotate(text, (x, y), xytext=(x + dx, y + dy), fontsize=fs,
                color=color, arrowprops=None)


def spec(ax, title: str, xlabel: str = '', ylabel: str = '', legend: bool = True):
    ax.set_title(title, fontsize=10)
    if xlabel:
        ax.set_xlabel(xlabel)
    if ylabel:
        ax.set_ylabel(ylabel)
    if legend:
        ax.legend(fontsize=8, loc='best')
