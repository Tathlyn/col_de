#!/usr/bin/env python3
"""通用示例：长方体障碍 + 一条直线路点。

演示 col_de 框架的完整用法：
  1. 发布一个长方体障碍物（碰撞体）；
  2. 定义若干笛卡尔目标位姿 + 一条直线路径；
  3. 用「IK 多种子选解 + 碰撞检测 + 梯形速度规划」生成并执行轨迹；
  4. 记录 TF 末端轨迹 / 关节角并导出指标与图表。

用法：
    ros2 run col_de demo_waypoints [--config <path/to/params.yaml>]

机械臂关节名 / group / tcp / base 全部来自配置文件，本文件不写死任何
具体机械臂型号或作业场景；下方坐标仅为占位示例，请按你的工作空间修改。
"""

import argparse
import json
import math
import os
import sys
import time

import rclpy
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from src.planner_node import PlannerNode, make_pose
from src.demo import simple_scene


# ----------------------------------------------------------------------------
# 示例参数（占位，请按实际工作空间修改）
# ----------------------------------------------------------------------------
EULER_LINE = [math.pi, math.radians(45.0), 0.0]   # 直线段末端姿态

SAFE = (0.60, -0.30, 0.50)        # 安全点
APPROACH = (0.60, 0.00, 0.30)     # 直线起点上方
LINE_START = (0.60, 0.00, 0.10)   # 直线起点
LINE_END = (0.60, 0.20, 0.10)     # 直线终点
RETREAT = (0.60, 0.20, 0.40)      # 退避点

N_LINE = 18                        # 直线段采样点数
OUTPUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "demo_results")


def _resolve_config(argv):
    if "--config" in argv:
        return argv[argv.index("--config") + 1]
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory("col_de"), "config", "params.yaml")
    except Exception:
        return None


def _move_to(node, xyz, euler, label):
    """点到点：IK 求逆解 → 梯形规划 → 沿途碰撞校验 → 执行。"""
    pose = make_pose(*xyz, euler_rad=euler, frame_id=node.cfg.base_frame)
    joints = node.solve_ik(pose, label=label)
    if joints is None:
        node.get_logger().error(f"  [{label}] IK 失败")
        return False
    timed = node.timed_from_joints(joints)
    ok, bad = node.check_timed(timed)
    if not ok:
        node.get_logger().warn(f"  [{label}] 轨迹第 {bad} 点碰撞")
        return False
    max_v, max_a = node.verify_limits(timed)
    node.get_logger().info(f"  [{label}] 轨迹 {len(timed)} 点，max_v={max_v:.2f}, max_a={max_a:.2f}")
    exec_ok, exec_t = node.execute(timed, label)
    node.get_logger().info(f"  [{label}] {'OK' if exec_ok else 'FAIL'} ({exec_t:.2f}s)")
    return exec_ok


def _plan_line(node, start, end, n, euler, label):
    """沿直线逐点 IK，种子锁定臂形保证连续，返回路点列表或 None。"""
    joints_list = []
    prev_seed = None
    for i in range(n):
        t = i / (n - 1) if n > 1 else 0.0
        x = start[0] + t * (end[0] - start[0])
        y = start[1] + t * (end[1] - start[1])
        z = start[2] + t * (end[2] - start[2])
        pose = make_pose(x, y, z, euler_rad=euler, frame_id=node.cfg.base_frame)
        joints = node.solve_ik(pose, seed_joints=prev_seed, label=f"{label}_{i}")
        if joints is None:
            node.get_logger().error(f"  [{label}] 路点 {i}/{n - 1} IK 失败")
            return None
        joints_list.append(joints)
        prev_seed = joints
    return joints_list


def _move_line(node, start, end, n, euler, label):
    joints_list = _plan_line(node, start, end, n, euler, label)
    if joints_list is None:
        return False
    timed = node.timed_from_waypoints(joints_list)
    ok, bad = node.check_timed(timed)
    if not ok:
        node.get_logger().warn(f"  [{label}] 轨迹第 {bad} 点碰撞")
        return False
    exec_ok, exec_t = node.execute(timed, label, pre_buffer_s=0.2, post_buffer_s=0.2)
    node.get_logger().info(f"  [{label}] {'OK' if exec_ok else 'FAIL'} ({exec_t:.2f}s)")
    return exec_ok


def _save_results(node, steps):
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    out = {
        "steps": steps,
        "total_path_length_m": sum(s["path_length_m"] for s in steps),
        "total_joint_displacement_rad": sum(s["joint_displacement_rad"] for s in steps),
        "success_count": sum(1 for s in steps if s["success"]),
    }
    fpath = os.path.join(OUTPUT_DIR, "metrics.json")
    with open(fpath, "w", encoding="utf-8") as f:
        json.dump(out, f, indent=2, ensure_ascii=False)
    print(f"[Metrics] JSON -> {fpath}")


def _plot(node, steps):
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    # 末端轨迹（XY 与 XZ）
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
    fig.suptitle("末端轨迹（长方体障碍 + 直线路点）")
    for rec, s in zip(node._all_tf, steps):
        xs = [r["x"] for r in rec]
        ys = [r["y"] for r in rec]
        zs = [r["z"] for r in rec]
        ax1.plot(xs, ys, "-", linewidth=1.2, label=s["step"])
        ax2.plot(xs, zs, "-", linewidth=1.2, label=s["step"])
    ax1.plot([LINE_START[0], LINE_END[0]], [LINE_START[1], LINE_END[1]],
             "g--", linewidth=2, label="line ideal")
    ax1.set_xlabel("X (m)"); ax1.set_ylabel("Y (m)"); ax1.axis("equal")
    ax1.grid(True, alpha=0.3); ax1.legend(fontsize=7)
    ax2.set_xlabel("X (m)"); ax2.set_ylabel("Z (m)")
    ax2.grid(True, alpha=0.3); ax2.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(os.path.join(OUTPUT_DIR, "trajectory.png"), dpi=150)
    plt.close(fig)
    print("[Plot] 末端轨迹 -> trajectory.png")


def main(argv=None):
    argv = argv or sys.argv
    parser = argparse.ArgumentParser(add_help=True)
    parser.add_argument("--config", default=None, help="配置文件路径")
    args, _ = parser.parse_known_args(argv[1:])
    config_path = args.config or _resolve_config(argv)

    rclpy.init(args=argv)
    node = PlannerNode(config_path=config_path)
    if not node.wait_for_services():
        rclpy.shutdown()
        return 1

    node._all_tf = []

    # 示例障碍物：一个放置在路径附近的长方体
    simple_scene.add_box(node, "demo_obstacle", (0.10, 0.10, 0.10),
                         (0.60, 0.10, 0.10))
    time.sleep(0.5)

    steps = []

    # 顺序流程：点到点 + 直线段
    plan = [
        ("safe", SAFE, EULER_LINE, "move"),
        ("approach", APPROACH, EULER_LINE, "move"),
        ("line", None, EULER_LINE, "line"),
        ("retreat", RETREAT, EULER_LINE, "move"),
    ]

    node.get_logger().info("=" * 50)
    node.get_logger().info("col_de 通用示例：长方体障碍 + 直线路点")
    node.get_logger().info(f"速度限制 <= {node.cfg.v_max}rad/s，加速度限制 <= {node.cfg.a_max}rad/s²")

    for name, xyz, euler, kind in plan:
        node.get_logger().info(f"[{name}] 开始")
        ok = False
        if kind == "line":
            ok = _move_line(node, LINE_START, LINE_END, N_LINE, euler, name)
        else:
            ok = _move_to(node, xyz, euler, name)
        steps.append({
            "step": name,
            "type": kind,
            "success": ok,
            "path_length_m": round(node.compute_tf_path_length(), 4),
            "joint_displacement_rad": round(node.compute_joint_displacement(), 4),
        })
        node._all_tf.append(list(node.tf_records))

    _save_results(node, steps)
    _plot(node, steps)
    node.get_logger().info(f"完成 -> {OUTPUT_DIR}")
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())