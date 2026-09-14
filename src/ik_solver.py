"""IK 逆解：多种子枚举 + 关节限位过滤 + 最小关节变化量选解。

只依赖 MoveIt 的 /compute_ik（GetPositionIK）服务，机械臂无关。
翻臂/翻腕等冗余自由度的候选种子由配置声明（默认索引 3 / 5），
不写死任何具体关节名。
"""

import itertools
import math

import rclpy
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import MoveItErrorCodes


class IKSolver:
    def __init__(self, node, config, ik_client):
        self._node = node
        self._cfg = config
        self._cli = ik_client
        self._n = config.n_joints

    @property
    def ready(self):
        return self._cli.service_is_ready()

    def solve(self, pose_stamped, current_joints, seed_joints=None,
              avoid_collisions=True, label=""):
        """给定目标位姿，返回关节角列表或 None。

        流程：先生成候选种子（指定种子优先，否则枚举翻臂/翻腕组合），
        再按「先在避碰下求解、失败则关闭避碰重试」的两轮策略，从所有
        满足关节限位的解中选出离 current_joints 最近的一个。
        """
        if not self.ready:
            self._node.get_logger().warn(f"  [{label}] IK 服务不可用")
            return None

        base = list(current_joints)
        seeds = self._build_seeds(base, seed_joints)

        best = None
        best_dist = float("inf")
        for use_avoid in (avoid_collisions, False):
            for seed in seeds:
                joints = self._solve_once(pose_stamped, use_avoid, seed)
                if joints is None:
                    continue
                if not self._in_limits(joints):
                    continue
                d = math.sqrt(sum((joints[j] - base[j]) ** 2 for j in range(self._n)))
                if d < best_dist:
                    best_dist = d
                    best = joints
            if best is not None and use_avoid == avoid_collisions:
                break

        if best is not None:
            self._node.get_logger().info(
                f"  [{label}] IK 成功 (Δ关节={best_dist:.3f}): "
                f"[{', '.join(f'{v:.3f}' for v in best)}]")
        return best

    def _build_seeds(self, base, seed_joints):
        if seed_joints is not None:
            return [list(seed_joints)]

        seeds = [list(base)]
        flip = self._cfg.seed_flip or []
        if not flip:
            return seeds

        indices = [f["index"] for f in flip]
        value_lists = [list(f["values"]) for f in flip]
        for vals in itertools.product(*value_lists):
            s = list(base)
            for idx, val in zip(indices, vals):
                if 0 <= idx < self._n:
                    s[idx] = val
            seeds.append(s)
        return seeds

    def _solve_once(self, pose_stamped, avoid, seed):
        req = GetPositionIK.Request()
        req.ik_request.group_name = self._cfg.group_name
        req.ik_request.ik_link_name = self._cfg.tcp_frame
        req.ik_request.pose_stamped = pose_stamped
        req.ik_request.pose_stamped.header.frame_id = self._cfg.base_frame
        req.ik_request.avoid_collisions = avoid
        timeout = self._cfg.ik_timeout
        req.ik_request.timeout.sec = int(timeout)
        req.ik_request.timeout.nanosec = int((timeout - int(timeout)) * 1e9)
        req.ik_request.robot_state.joint_state.name = list(self._cfg.joint_names)
        req.ik_request.robot_state.joint_state.position = [float(j) for j in seed]

        fut = self._cli.call_async(req)
        rclpy.spin_until_future_complete(self._node, fut, timeout_sec=timeout + 2.0)
        resp = fut.result()
        if resp is None or resp.error_code.val != MoveItErrorCodes.SUCCESS:
            return None

        js = resp.solution.joint_state
        if self._cfg.joint_names[0] not in js.name:
            return None
        return [float(js.position[js.name.index(n)]) for n in self._cfg.joint_names]

    def _in_limits(self, joints):
        for i, name in enumerate(self._cfg.joint_names):
            lim = self._cfg.joint_limits.get(name)
            if lim is None:
                continue
            lo, hi = lim
            if joints[i] < lo or joints[i] > hi:
                return False
        return True