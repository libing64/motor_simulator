# -*- coding: utf-8 -*-
"""极简测试框架：注册用例、运行、汇总（无需 pytest）。"""

from __future__ import annotations

import time
import traceback
from dataclasses import dataclass, field
from typing import Callable, List, Optional


@dataclass
class Result:
    name: str
    passed: bool
    detail: str = ''
    elapsed: float = 0.0
    error: Optional[str] = None


@dataclass
class Case:
    name: str
    fn: Callable[[], str]
    tags: List[str] = field(default_factory=list)


CASES: List[Case] = []


def case(name: str, tags: Optional[List[str]] = None):
    """注册一个测试用例；函数返回字符串作为附加说明，断言失败即 FAIL。"""
    def deco(fn):
        CASES.append(Case(name=name, fn=fn, tags=tags or []))
        return fn
    return deco


def run_all(verbose: bool = True) -> List[Result]:
    results: List[Result] = []
    for c in CASES:
        t0 = time.time()
        try:
            detail = c.fn() or ''
            r = Result(c.name, True, detail, time.time() - t0)
        except AssertionError as e:
            r = Result(c.name, False, str(e) or 'assertion failed', time.time() - t0)
        except Exception as e:  # noqa: BLE001
            r = Result(c.name, False, 'ERROR: %s' % e, time.time() - t0,
                       error=traceback.format_exc())
        results.append(r)
        if verbose:
            flag = 'PASS' if r.passed else 'FAIL'
            print('  [%s] %-46s %6.2fs  %s' % (flag, r.name, r.elapsed, r.detail))
            if r.error:
                print(r.error)
    return results


def summary(results: List[Result]) -> str:
    n = len(results)
    ok = sum(1 for r in results if r.passed)
    lines = ['=' * 78,
             '测试汇总: %d/%d 通过%s' % (ok, n, '' if ok == n else '  <<< 有失败用例'),
             '=' * 78]
    for r in results:
        lines.append('  %s  %-46s %s' % ('PASS' if r.passed else 'FAIL', r.name, r.detail))
    return '\n'.join(lines)
