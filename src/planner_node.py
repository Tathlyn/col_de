"""规划节点：编排 IK 求解、碰撞检测、梯形速度规划、执行与指标记录。

这是通用框架层——它只负责把「目标位姿 / 关节路点」翻译成可执行轨迹，
不包含任何具体场景 / 工件坐标，也不依赖具体机械臂型号。示意见 demo/。

内部数据约定：
  timed_points  梯形规划输出的带时间戳采样 [(positions, t_abs), ...]，
                首点 t_abs = 0，positions 为按 joint_names 顺序的关节角列表。
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from scipy.spatial.transform import Rotation as R
import tf2_ros

from src.config import PlannerConfig
from src.ik_solver import IKSolver
from src.collision_checker import CollisionChecker
from src import trapezoid


def make_pose(x, y, z, euler_rad, frame_id):
    """用 euler 角（xyz 顺序，弧度）构造一个 PoseStamped。"""
    p = PoseStamped()
    p.header.frame_id = frame_id
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.position.z = float(z)
    q = R.from_euler("xyz", euler_rad).as_quat()
    p.pose.orientation.x = q[0]
    p.pose.orientation.y = q[1]
    p.pose.orientation.z = q[2]
    p.pose.orientation.w = q[3]
    return p


class PlannerNode(Node):
    def __init__(self, config_path=None, node_name="ik_collision_planner"):
        super().__init__(node_name)
        self.cfg = PlannerConfig.load(config_path)
        self.n_joints = self.cfg.n_joints

        self.current_joints = [0.0] * self.n_joints
        self.js_sub = self.create_subscription(
            JointState, "/joint_states", self._js_cb, 10)
        self.traj_cli = ActionClient(self, FollowJointTrajectory, self.cfg.controller_action)
        self.ik_cli = self.create_client(GetPositionIK, "/compute_ik")

        self.ik_solver = IKSolver(self, self.cfg, self.ik_cli)
        self.collision_checker = CollisionChecker(self, self.cfg)

        # TF 末端轨迹 + 关节角记录（供指标计算与可视化）
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_records = []
        self.joint_records = []
        self.recording = False
        self.record_timer = self.create_timer(0.1, self._record_cb)

    # ------------------------------------------------------------------ 回调
    def _js_cb(self, msg):
        for name in self.cfg.joint_names:
            if name in msg.name:
                idx = msg.name.index(name)
                self.current_joints[self.cfg.joint_names.index(name)] = msg.position[idx]

    def _record_cb(self):
        if not self.recording:
            return
        now = time.time()
        try:
            t = self.tf_buffer.lookup_transform(
                self.cfg.base_frame, self.cfg.tcp_frame, rclpy.time.Time())
            self.tf_records.append({
                "time": now,
                "x": float(t.transform.translation.x),
                "y": float(t.transform.translation.y),
                "z": float(t.transform.translation.z),
            })
        except Exception:
            pass
        self.joint_records.append({"time": now, "joints": list(self.current_joints)})

    def start_recording(self):
        self.tf_records.clear()
        self.joint_records.clear()
        self.recording = True

    def stop_recording(self):
        self.recording = False

    # ------------------------------------------------------------ 服务就绪
    def wait_for_services(self):
        self.get_logger().info("等待服务...")
        if not self.ik_cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("compute_ik 不可用")
            return False
        if not self.collision_checker.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("check_state_validity 不可用，碰撞校验将降级为不校验")
        self.get_logger().info("等待控制器...")
        t0 = time.time()
        while time.time() - t0 < 15.0:
            if self.traj_cli.wait_for_server(timeout_sec=1.0):
                self.get_logger().info("全部就绪")
                return True
        self.get_logger().error("控制器不可用")
        return False

    # ------------------------------------------------------------ 求解/校验
    def solve_ik(self, pose_stamped, seed_joints=None, avoid_collisions=True, label=""):
        return self.ik_solver.solve(
            pose_stamped, self.current_joints, seed_joints=seed_joints,
            avoid_collisions=avoid_collisions, label=label)

    def check_timed(self, timed_points):
        return self.collision_checker.check_trajectory(timed_points)

    # ------------------------------------------------------------ 轨迹生成
    def timed_from_joints(self, target_joints):
        """从当前关节角到目标关节角的单段梯形规划。"""
        return trapezoid.build_waypoints(
            [list(self.current_joints), list(target_joints)],
            self.cfg.v_max, self.cfg.a_max, self.cfg.dt)

    def timed_from_waypoints(self, joints_list):
        """多个关节角路点 → 连续梯形规划（直线插补段用）。"""
        return trapezoid.build_waypoints(
            [list(j) for j in joints_list],
            self.cfg.v_max, self.cfg.a_max, self.cfg.dt)

    def to_joint_trajectory(self, timed_points):
        jt = JointTrajectory()
        jt.joint_names = list(self.cfg.joint_names)
        for pos, t in timed_points:
            p = JointTrajectoryPoint()
            p.positions = [float(v) for v in pos]
            sec = int(t)
            p.time_from_start.sec = sec
            p.time_from_start.nanosec = int((t - sec) * 1e9)
            jt.points.append(p)
        return jt

    def verify_limits(self, timed_points):
        return trapezoid.verify_limits(timed_points, self.n_joints)

    # ------------------------------------------------------------ 执行
    def execute(self, timed_points, label="", pre_buffer_s=0.0, post_buffer_s=0.0):
        self.start_recording()
        if pre_buffer_s > 0:
            t_end = time.time() + pre_buffer_s
            while time.time() < t_end and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = self.to_joint_trajectory(timed_points)
        goal.trajectory.header.stamp = self.get_clock().now().to_msg()
        goal.trajectory.header.frame_id = "world"

        t0 = time.perf_counter()
        sf = self.traj_cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, sf, timeout_sec=5.0)
        gh = sf.result()
        if not gh or not gh.accepted:
            exec_time = time.perf_counter() - t0
            self.stop_recording()
            self.get_logger().error(f"  [{label}] 控制器拒绝 ({exec_time:.2f}s)")
            return False, exec_time

        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=120.0)
        exec_time = time.perf_counter() - t0
        res = rf.result()
        if res is None:
            self.stop_recording()
            self.get_logger().error(f"  [{label}] 无结果 ({exec_time:.2f}s)")
            return False, exec_time

        if post_buffer_s > 0:
            t_end = time.time() + post_buffer_s
            while time.time() < t_end and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)
        self.stop_recording()

        ok = res.result.error_code == 0
        self.get_logger().info(
            f"  [{label}] 执行{'成功' if ok else '失败'} code={res.result.error_code} "
            f"({exec_time:.2f}s, TF记录 {len(self.tf_records)} 帧)")
        return ok, exec_time

    # ------------------------------------------------------------ 指标
    def compute_tf_path_length(self):
        if len(self.tf_records) < 2:
            return 0.0
        length = 0.0
        for i in range(1, len(self.tf_records)):
            dx = self.tf_records[i]["x"] - self.tf_records[i - 1]["x"]
            dy = self.tf_records[i]["y"] - self.tf_records[i - 1]["y"]
            dz = self.tf_records[i]["z"] - self.tf_records[i - 1]["z"]
            length += math.sqrt(dx * dx + dy * dy + dz * dz)
        return length

    def compute_joint_displacement(self):
        if len(self.joint_records) < 2:
            return 0.0
        total = 0.0
        for i in range(1, len(self.joint_records)):
            prev = self.joint_records[i - 1]["joints"]
            cur = self.joint_records[i]["joints"]
            for j in range(self.n_joints):
                total += abs(cur[j] - prev[j])
        return total