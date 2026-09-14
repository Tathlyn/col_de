"""碰撞检测：基于 MoveIt /check_state_validity（GetStateValidity）服务。

底层的连续碰撞检测由 MoveIt/FCL 完成；这里只做单状态查询和整条轨迹的
沿途稠密校验，并提供「服务不可用时降级为不校验」的容错策略。
"""

import rclpy
from moveit_msgs.srv import GetStateValidity


class CollisionChecker:
    def __init__(self, node, config):
        self._node = node
        self._cfg = config
        self._cli = node.create_client(GetStateValidity, "/check_state_validity")

    def wait_for_service(self, timeout_sec=5.0):
        return self._cli.wait_for_service(timeout_sec=timeout_sec)

    @property
    def ready(self):
        return self._cli.service_is_ready()

    def is_state_valid(self, joints):
        """给定关节角，返回是否无碰撞且无超限。服务不可用时降级返回 True。"""
        if not self.ready:
            return True
        req = GetStateValidity.Request()
        req.group_name = self._cfg.group_name
        req.robot_state.joint_state.name = list(self._cfg.joint_names)
        req.robot_state.joint_state.position = [float(j) for j in joints]
        fut = self._cli.call_async(req)
        rclpy.spin_until_future_complete(self._node, fut, timeout_sec=5.0)
        resp = fut.result()
        return bool(resp is not None and resp.valid)

    def check_trajectory(self, timed_points):
        """校验一条轨迹沿途每个采样点是否无碰。

        参数 timed_points 为 [(positions, t), ...]，返回 (ok, first_bad_index)。
        """
        for i, (pos, _t) in enumerate(timed_points):
            if not self.is_state_valid(list(pos)):
                return False, i
        return True, -1