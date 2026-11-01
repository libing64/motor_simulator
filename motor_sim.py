#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PMSM / BLDC 电机本体仿真器（开环）
====================================
模拟电机的「输入 → 输出」关系：

    输入：三相正弦电压（幅值 Vpk、频率 f，带 V/f 软启动） 或 直接 dq 电压（vd, vq）
          + 负载转矩 TL(t)
    输出：转速 ωm、电磁转矩 Te、三相电流 ia/ib/ic、dq 电流 id/iq、转子电角度 θe

模型说明：
    采用 dq 旋转坐标系下的永磁同步电机方程（表贴式，Ld = Lq = L），
    用 Clarke / Park 变换把三相电压转到转子 dq 系，再对状态方程做
    Euler 数值积分，最后用反变换还原出三相电流。

    电机带「阻尼绕组」（异步阻尼笼）：开环 V/f 启动时，转子与旋转磁场
    之间的转差会感应出阻尼转矩，帮助电机牵入同步。把 PARAMS 里的
    D_damp 设为 0 并加快斜坡，就能看到同步电机开环直启的失步现象。

状态方程（转子 dq 系）：
    d(id)/dt = (vd - Rs*id + ωe*L*iq) / L
    d(iq)/dt = (vq - Rs*iq - ωe*(L*id + ψf)) / L
    d(ωm)/dt = (Te + T_damp - TL - B*ωm) / J
    d(θm)/dt = ωm
    其中：ωe = p*ωm，θe = p*θm，Te = 1.5*p*ψf*iq，
          T_damp = D_damp*(ωv - ωe)  （ωv 为电压矢量电角速度，仅三相模式非零）

用法：
    python motor_sim.py            # 默认三相电压软启动开环驱动，输出波形图
    依赖：numpy, matplotlib
"""

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm

# ---- 中文字体配置（跨平台）----
for _f in ['/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
           '/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc']:
    try:
        fm.fontManager.addfont(_f)
    except Exception:
        pass
plt.rcParams['font.sans-serif'] = ['Noto Sans CJK JP', 'Noto Sans CJK SC', 'Microsoft YaHei',
                                   'SimHei', 'PingFang SC', 'WenQuanYi Zen Hei', 'Arial Unicode MS']
plt.rcParams['axes.unicode_minus'] = False

# ================= 电机参数（带阻尼绕组的表贴式 PMSM / BLDC）=================
PARAMS = {
    'Rs': 0.8,       # 定子电阻 [Ω]
    'L': 2.0e-3,     # 定子电感 Ld = Lq = L [H]
    'psi_f': 0.03,   # 永磁体磁链 [Wb]
    'p': 4,          # 极对数
    'J': 3.0e-3,     # 转子 + 负载转动惯量 [kg·m²]
    'B': 3.0e-4,     # 粘滞摩擦系数 [N·m·s/rad]
    'D_damp': 0.01,  # 阻尼绕组系数 [N·m·s/rad]（设为 0 可观察失步）
}

# ================= 输入设置（开环）=================
INPUT = {
    'mode': 'three_phase',   # 'three_phase' 三相电压 / 'dq' 直接给 dq 电压
    'Vpk': 12.0,             # 三相模式：相电压幅值 [V]
    'f': 30.0,               # 三相模式：电压频率 [Hz]（同步转速 = 60*f/p rpm）
    'start_ramp_time': 0.30, # 三相模式：软启动斜坡时间 [s]（f、V 从 0 线性升到设定值）
    'vd': 0.0,               # dq 模式：d 轴电压 [V]
    'vq': 6.0,               # dq 模式：q 轴电压 [V]
    'TL_initial': 0.02,      # 初始负载转矩 [N·m]
    'TL_step': 0.12,         # 阶跃后的负载转矩 [N·m]
    'TL_step_time': 0.50,    # 负载阶跃发生时刻 [s]
}

SIM = {
    'dt': 2e-6,              # 积分步长 [s]
    't_end': 0.70,           # 仿真总时长 [s]
}


def clarke(a, b, c):
    """Clarke 变换（等幅值）：三相 -> αβ。"""
    alpha = (2.0 / 3.0) * (a - 0.5 * b - 0.5 * c)
    beta = (b - c) / np.sqrt(3.0)
    return alpha, beta


def park(alpha, beta, theta):
    """Park 变换：αβ（静止）-> dq（随转子旋转）。theta 为转子电角度。"""
    d = alpha * np.cos(theta) + beta * np.sin(theta)
    q = -alpha * np.sin(theta) + beta * np.cos(theta)
    return d, q


def inverse_park(d, q, theta):
    """反 Park 变换：dq -> αβ。"""
    alpha = d * np.cos(theta) - q * np.sin(theta)
    beta = d * np.sin(theta) + q * np.cos(theta)
    return alpha, beta


def inverse_clarke(alpha, beta):
    """反 Clarke 变换：αβ -> 三相。"""
    a = alpha
    b = -0.5 * alpha + (np.sqrt(3.0) / 2.0) * beta
    c = -0.5 * alpha - (np.sqrt(3.0) / 2.0) * beta
    return a, b, c


def load_torque(t):
    """负载转矩：t < TL_step_time 为 TL_initial，之后阶跃到 TL_step。"""
    return INPUT['TL_initial'] if t < INPUT['TL_step_time'] else INPUT['TL_step']


def run_simulation():
    """运行仿真，返回结果字典（含时间序列）。"""
    Rs, L, psi_f, p, J, B, D_damp = (PARAMS[k] for k in ('Rs', 'L', 'psi_f', 'p', 'J', 'B', 'D_damp'))
    dt, t_end = SIM['dt'], SIM['t_end']

    n_steps = int(t_end / dt)
    t = np.zeros(n_steps)
    id_arr = np.zeros(n_steps)
    iq_arr = np.zeros(n_steps)
    wm_arr = np.zeros(n_steps)      # 机械角速度 [rad/s]
    te_arr = np.zeros(n_steps)      # 电磁转矩 [N·m]
    ia_arr = np.zeros(n_steps)
    ib_arr = np.zeros(n_steps)
    ic_arr = np.zeros(n_steps)
    tl_arr = np.zeros(n_steps)
    theta_e_arr = np.zeros(n_steps)

    # 状态变量初值（电机静止、无电流）
    id_s, iq_s, wm, theta_m = 0.0, 0.0, 0.0, 0.0
    theta_v = 0.0                  # 电压矢量相位（需积分，因为频率可时变）

    ramp_time = INPUT.get('start_ramp_time', 0.0)

    for k in range(n_steps):
        tk = k * dt
        we = p * wm                     # 电角速度
        theta_e = p * theta_m           # 转子电角度

        # ---- 生成输入电压（vd, vq，转子 dq 系）----
        T_damp = 0.0
        if INPUT['mode'] == 'three_phase':
            # V/f 软启动：f、V 从 0 线性升到设定值
            ramp = min(1.0, tk / ramp_time) if ramp_time > 0 else 1.0
            f_inst = INPUT['f'] * ramp
            V_inst = INPUT['Vpk'] * ramp
            wv = 2.0 * np.pi * f_inst          # 电压矢量电角速度
            theta_v += wv * dt                 # 相位积分
            va = V_inst * np.cos(theta_v)
            vb = V_inst * np.cos(theta_v - 2.0 * np.pi / 3.0)
            vc = V_inst * np.cos(theta_v + 2.0 * np.pi / 3.0)
            valpha, vbeta = clarke(va, vb, vc)
            vd, vq = park(valpha, vbeta, theta_e)       # 投影到转子 dq 系
            T_damp = D_damp * (wv - we)         # 异步阻尼转矩（阻尼笼）
        else:  # 'dq'
            vd, vq = INPUT['vd'], INPUT['vq']

        # ---- 电机状态方程（Euler 积分）----
        did = (vd - Rs * id_s + we * L * iq_s) / L
        diq = (vq - Rs * iq_s - we * (L * id_s + psi_f)) / L
        Te = 1.5 * p * psi_f * iq_s
        TL = load_torque(tk)
        dwm = (Te + T_damp - TL - B * wm) / J

        id_s += did * dt
        iq_s += diq * dt
        wm += dwm * dt
        theta_m += wm * dt

        # ---- 反变换得到三相电流 ----
        ialpha, ibeta = inverse_park(id_s, iq_s, theta_e)
        ia, ib, ic = inverse_clarke(ialpha, ibeta)

        # ---- 记录 ----
        t[k] = tk
        id_arr[k], iq_arr[k] = id_s, iq_s
        wm_arr[k] = wm
        te_arr[k] = Te
        ia_arr[k], ib_arr[k], ic_arr[k] = ia, ib, ic
        tl_arr[k] = TL
        theta_e_arr[k] = theta_e

    return {
        't': t, 'id': id_arr, 'iq': iq_arr, 'wm': wm_arr, 'te': te_arr,
        'ia': ia_arr, 'ib': ib_arr, 'ic': ic_arr, 'tl': tl_arr, 'theta_e': theta_e_arr,
        'p': p,
    }


def plot_results(res):
    """画出输入/输出波形。"""
    t = res['t']
    rpm = res['wm'] * 60.0 / (2.0 * np.pi)          # 转成 rpm
    sync_rpm = 60.0 * INPUT['f'] / res['p'] if INPUT['mode'] == 'three_phase' else None

    fig, axes = plt.subplots(2, 3, figsize=(13, 7))
    fig.suptitle('PMSM/BLDC 电机本体开环仿真（输入 → 输出）', fontsize=14)

    # 1. 转速
    ax = axes[0, 0]
    ax.plot(t, rpm, 'b-', lw=1.5)
    if sync_rpm is not None:
        ax.axhline(sync_rpm, color='r', ls='--', lw=1, label=f'同步转速 {sync_rpm:.0f} rpm')
    ax.set_ylabel('转速 [rpm]'); ax.set_title('输出：转速 ωm')
    ax.legend(fontsize=8); ax.grid(alpha=0.3)

    # 2. 转矩（含负载）
    ax = axes[0, 1]
    ax.plot(t, res['te'], 'b-', lw=1.5, label='电磁转矩 Te')
    ax.plot(t, res['tl'], 'r--', lw=1.2, label='负载转矩 TL')
    ax.set_ylabel('转矩 [N·m]'); ax.set_title('输出：转矩')
    ax.legend(fontsize=8); ax.grid(alpha=0.3)

    # 3. 三相电流
    ax = axes[0, 2]
    ax.plot(t, res['ia'], lw=1, label='ia')
    ax.plot(t, res['ib'], lw=1, label='ib')
    ax.plot(t, res['ic'], lw=1, label='ic')
    ax.set_ylabel('电流 [A]'); ax.set_title('输出：三相电流')
    ax.legend(fontsize=8, ncol=3); ax.grid(alpha=0.3)

    # 4. dq 电流
    ax = axes[1, 0]
    ax.plot(t, res['id'], lw=1.3, label='id（励磁）')
    ax.plot(t, res['iq'], lw=1.3, label='iq（转矩）')
    ax.set_xlabel('时间 [s]'); ax.set_ylabel('电流 [A]')
    ax.set_title('输出：dq 电流'); ax.legend(fontsize=8); ax.grid(alpha=0.3)

    # 5. 输入：三相电压
    ax = axes[1, 1]
    if INPUT['mode'] == 'three_phase':
        theta_v = 2.0 * np.pi * INPUT['f'] * t
        ax.plot(t, INPUT['Vpk'] * np.cos(theta_v), lw=0.8, label='va')
        ax.plot(t, INPUT['Vpk'] * np.cos(theta_v - 2 * np.pi / 3), lw=0.8, label='vb')
        ax.plot(t, INPUT['Vpk'] * np.cos(theta_v + 2 * np.pi / 3), lw=0.8, label='vc')
        ax.set_title('输入：三相电压')
    else:
        ax.plot(t, np.full_like(t, INPUT['vd']), lw=1.2, label='vd')
        ax.plot(t, np.full_like(t, INPUT['vq']), lw=1.2, label='vq')
        ax.set_title('输入：dq 电压')
    ax.set_xlabel('时间 [s]'); ax.set_ylabel('电压 [V]')
    ax.legend(fontsize=8, ncol=3); ax.grid(alpha=0.3)

    # 6. 转子电角度
    ax = axes[1, 2]
    ax.plot(t, res['theta_e'], 'g-', lw=1)
    ax.set_xlabel('时间 [s]'); ax.set_ylabel('电角度 [rad]')
    ax.set_title('输出：转子电角度 θe'); ax.grid(alpha=0.3)

    fig.tight_layout(rect=[0, 0, 1, 0.96])
    out = 'motor_sim_result.png'
    fig.savefig(out, dpi=120)
    print(f'[OK] 波形已保存到 {out}')
    # 本地有图形环境时可取消下行注释，直接弹窗显示：
    # plt.show()


if __name__ == '__main__':
    res = run_simulation()

    # 打印关键结果摘要
    wm_ss = res['wm'][-1]
    print('=' * 60)
    print(f"输入模式: {INPUT['mode']}")
    if INPUT['mode'] == 'three_phase':
        print(f"三相电压: 幅值 {INPUT['Vpk']} V, 频率 {INPUT['f']} Hz "
              f"(同步转速 {60*INPUT['f']/PARAMS['p']:.0f} rpm), 软启动 {INPUT['start_ramp_time']} s")
    else:
        print(f"dq 电压: vd={INPUT['vd']} V, vq={INPUT['vq']} V")
    print(f"负载: {INPUT['TL_initial']} -> {INPUT['TL_step']} N·m "
          f"(t={INPUT['TL_step_time']} s 阶跃)")
    print(f"稳态转速: {wm_ss:.1f} rad/s = {wm_ss*60/(2*np.pi):.0f} rpm")
    print(f"稳态电磁转矩: {res['te'][-1]:.3f} N·m")
    print(f"稳态 dq 电流: id={res['id'][-1]:.2f} A, iq={res['iq'][-1]:.2f} A")
    print('=' * 60)

    plot_results(res)
