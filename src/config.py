"""加载与校验规划配置。

所有机械臂相关命名（关节名 / group / tcp / base / 控制器 action）、关节限位、
速度/加速度约束都从这里读取。代码中不写死任何具体机械臂或作业场景。

配置来源：内置默认值 + 可选 YAML 覆盖。YAML 形如：:

    robot:
      joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
      group_name: manipulator
      tcp_frame: tool0
      base_frame: base_link
      controller_action: /follow_joint_trajectory
    ik:
      timeout: 1.0
      joint_limits_deg:          # 值可为标量（对称 ±N°）或 [lo, hi]
        joint_1: 90.0
        joint_6: [-360.0, 360.0]
      seed_flip:                 # 翻臂/翻腕枚举（关节索引 + 候选值）
        - index: 3
          values: [-2.0, 0.0, 2.0]
        - index: 5
          values: [0.0, -3.141592653589793, 3.141592653589793]
    motion:
      v_max: 3.0
      a_max: 10.0
      dt: 0.05
"""

import copy
import math

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None


DEFAULT_CONFIG = {
    "robot": {
        "joint_names": ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
        "group_name": "manipulator",
        "tcp_frame": "tool0",
        "base_frame": "base_link",
        "controller_action": "/follow_joint_trajectory",
    },
    "ik": {
        "timeout": 1.0,
        "joint_limits_deg": {},
        "seed_flip": [
            {"index": 3, "values": [-2.0, 0.0, 2.0]},
            {"index": 5, "values": [0.0, -math.pi, math.pi]},
        ],
    },
    "motion": {
        "v_max": 3.0,
        "a_max": 10.0,
        "dt": 0.05,
    },
}


def _deep_merge(base, extra):
    """递归合并 extra 到 base（就地修改 base）。"""
    for k, v in extra.items():
        if k in base and isinstance(base[k], dict) and isinstance(v, dict):
            _deep_merge(base[k], v)
        else:
            base[k] = v


class PlannerConfig:
    """规划配置对象，提供便捷的属性访问。"""

    def __init__(self, raw=None):
        cfg = copy.deepcopy(DEFAULT_CONFIG)
        if raw:
            _deep_merge(cfg, raw)

        robot = cfg["robot"]
        ik = cfg["ik"]
        motion = cfg["motion"]

        self.joint_names = list(robot["joint_names"])
        self.n_joints = len(self.joint_names)
        self.group_name = robot.get("group_name", "manipulator")
        self.tcp_frame = robot.get("tcp_frame", "tool0")
        self.base_frame = robot.get("base_frame", "base_link")
        self.controller_action = robot.get(
            "controller_action", "/follow_joint_trajectory")

        self.ik_timeout = float(ik.get("timeout", 1.0))
        self.seed_flip = ik.get("seed_flip", DEFAULT_CONFIG["ik"]["seed_flip"])

        self.v_max = float(motion.get("v_max", 3.0))
        self.a_max = float(motion.get("a_max", 10.0))
        self.dt = float(motion.get("dt", 0.05))

        # 关节限位（转成弧度）：标量 → 对称 ±N°，列表 → [lo, hi]
        self.joint_limits = {}
        for name, val in (ik.get("joint_limits_deg", {}) or {}).items():
            if isinstance(val, (list, tuple)):
                lo, hi = val
            else:
                lo, hi = -float(val), float(val)
            self.joint_limits[name] = (math.radians(float(lo)), math.radians(float(hi)))

    @classmethod
    def load(cls, path=None):
        """从 YAML 文件加载配置；path 为 None 时使用内置默认值。"""
        raw = None
        if path:
            if yaml is None:
                raise RuntimeError("需要 PyYAML 才能读取配置，请安装 python3-yaml")
            with open(path, "r", encoding="utf-8") as f:
                raw = yaml.safe_load(f)
        return cls(raw)