# -*- coding: utf-8 -*-
"""一键跑完全部实验：python3 tutorial/run_all.py [编号...]

不带参数时按 01..08 顺序全跑；带参数时只跑指定课，例如：

    python3 tutorial/run_all.py 6 7
"""

import glob
import os
import subprocess
import sys
import time

TUT = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TUT)
sys.path.insert(0, ROOT)

LESSONS = sorted(glob.glob(os.path.join(TUT, 't[0-9][0-9]_*.py')))


def main(argv):
    picked = LESSONS
    if argv:
        wanted = {a.zfill(2) for a in argv}
        picked = [p for p in LESSONS if os.path.basename(p)[1:3] in wanted]
        if not picked:
            print('没有匹配的课号：%s' % ' '.join(argv))
            return 2

    print('=' * 70)
    print(' 电机与 FOC 入门实验 · 共 %d 课' % len(picked))
    print('=' * 70)

    failed = []
    total = 0.0
    for path in picked:
        name = os.path.basename(path)
        print('\n>>> %s' % name)
        t0 = time.time()
        done = subprocess.run([sys.executable, path], cwd=TUT)
        cost = time.time() - t0
        total += cost
        print('<<< %s  %s  %.1fs' % (name, 'OK' if done.returncode == 0 else 'FAILED', cost))
        if done.returncode != 0:
            failed.append(name)

    print('\n' + '=' * 70)
    print(' 完成：%d/%d 课，总耗时 %.1f s' % (len(picked) - len(failed), len(picked), total))
    if failed:
        print(' 失败：%s' % ', '.join(failed))
    print(' 图在 %s' % os.path.join(TUT, 'out'))
    print('=' * 70)
    return 0 if not failed else 1


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
