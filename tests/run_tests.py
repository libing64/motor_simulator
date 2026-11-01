#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""运行全部测试用例：python3 tests/run_tests.py"""

import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from tests.harness import run_all, summary  # noqa: E402
from tests import test_suite  # noqa: F401,E402  (导入即注册用例)

if __name__ == '__main__':
    print('SimpleFOC × 电机仿真 联合测试')
    print('-' * 78)
    results = run_all()
    print(summary(results))
    sys.exit(0 if all(r.passed for r in results) else 1)
