"""梯形速度规划（无 ROS 依赖的纯函数）。

对关节空间的一段或多段直线，在单关节速度/加速度约束下，以「位移最大的
关节」归一化，生成带时间戳的离散采样点。三角形速度曲线用于短行程，
梯形速度曲线（加速—匀速—减速）用于长行程。
"""

import math


def trap_displacement(t, T, dmax, v_max, a_max):
    """梯形/三角速度曲线在时刻 t 的位移 s(t)，范围 [0, dmax]。

    参数
    ----
    t     当前时刻（秒）
    T     该段总时长（秒）
    dmax  目标关节的最大位移（rad）
    v_max 单关节最大速度（rad/s）
    a_max 单关节最大加速度（rad/s²）
    """
    if dmax <= 0.0 or T <= 0.0:
        return 0.0
    d_acc = v_max * v_max / (2.0 * a_max)  # 加速到 v_max 所需位移

    if dmax > 2.0 * d_acc:
        # 梯形：加速 → 匀速 → 减速
        t_acc = v_max / a_max
        t_const = (dmax - 2.0 * d_acc) / v_max
        if t < t_acc:
            return 0.5 * a_max * t * t
        if t < t_acc + t_const:
            return d_acc + v_max * (t - t_acc)
        return dmax - 0.5 * a_max * (T - t) * (T - t)

    # 三角形：行程太短，达不到 v_max
    if t < T / 2.0:
        return 0.5 * a_max * t * t
    return dmax - 0.5 * a_max * (T - t) * (T - t)


def trap_segment(start, goal, v_max, a_max, dt):
    """规划一段关节空间直线。

    返回 (points, duration)：
      points   [(positions, t), ...]，t 为相对该段起点的时间（秒），不含起点；
      duration 该段总时长（秒）。
    start / goal 为等长列表，长度即关节数 N。
    """
    n = len(start)
    delta = [goal[i] - start[i] for i in range(n)]
    dmax = max(abs(d) for d in delta)
    if dmax < 1e-12:
        return [], 0.0

    d_acc = v_max * v_max / (2.0 * a_max)
    if dmax > 2.0 * d_acc:
        t_acc = v_max / a_max
        T = 2.0 * t_acc + (dmax - 2.0 * d_acc) / v_max
    else:
        T = 2.0 * math.sqrt(dmax / a_max)

    n_samples = max(1, int(round(T / dt)))
    points = []
    for i in range(1, n_samples + 1):
        t = T * i / n_samples
        s = trap_displacement(t, T, dmax, v_max, a_max)
        frac = s / dmax
        pos = [start[j] + delta[j] * frac for j in range(n)]
        points.append((pos, t))
    return points, T


def build_waypoints(waypoints, v_max, a_max, dt):
    """多个关节角路点 → 带时间戳的连续采样。

    返回列表 timed：[(positions, t_abs), ...]，首点为 t_abs = 0。
    waypoints 为二维列表 [[j0..jN], ...]。
    """
    if not waypoints:
        return []
    timed = [(list(waypoints[0]), 0.0)]
    t_cum = 0.0
    for i in range(len(waypoints) - 1):
        pts, T = trap_segment(waypoints[i], waypoints[i + 1], v_max, a_max, dt)
        for pos, t in pts:
            timed.append((pos, t_cum + t))
        t_cum += T
    return timed


def verify_limits(timed, n_joints):
    """反算采样的最大速度 / 加速度，用于验证约束是否生效。

    参数 timed 为 [(positions, t), ...]，返回 (max_v, max_a)。
    """
    max_v = 0.0
    max_a = 0.0
    prev_v = [0.0] * n_joints
    prev_t = None
    prev_pos = None
    for pos, t in timed:
        if prev_t is not None:
            dv = t - prev_t
            if dv > 0:
                v = [abs(pos[j] - prev_pos[j]) / dv for j in range(n_joints)]
                a = [abs(v[j] - prev_v[j]) / dv for j in range(n_joints)]
                max_v = max(max_v, max(v))
                max_a = max(max_a, max(a))
                prev_v = v
        prev_t = t
        prev_pos = pos
    return max_v, max_a