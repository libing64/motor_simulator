# -*- coding: utf-8 -*-
"""matplotlib 中文字体与统一样式。"""

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm

_CJK_CANDIDATES = [
    '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
    '/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc',
    '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc',
]


def setup_cjk() -> None:
    for path in _CJK_CANDIDATES:
        try:
            fm.fontManager.addfont(path)
        except Exception:
            pass
    plt.rcParams['font.sans-serif'] = ['Noto Sans CJK JP', 'Noto Sans CJK SC', 'Microsoft YaHei',
                                       'SimHei', 'PingFang SC', 'WenQuanYi Zen Hei',
                                       'Arial Unicode MS', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False
    plt.rcParams['figure.dpi'] = 110
    plt.rcParams['axes.grid'] = True
    plt.rcParams['grid.alpha'] = 0.3
    plt.rcParams['font.size'] = 9
