# col_de

基于 **ROS2 + MoveIt2**的机械臂关节空间轨迹避障规划框架。

核心思路： **「IK 逆解 + 沿途碰撞检测 + 梯形速度规划」**，
在给定若干笛卡尔目标位姿（或一条直线/曲线路点）时，稳定生成满足
关节限位、碰撞约束、速度与加速度约束的连续轨迹。

## 核心能力

| 模块 | 文件 | 说明 |
|---|---|---|
| 配置加载 | `src/config.py` | 关节名 / group / tcp / base / 限位 / 速度加速度，全部配置化 |
| IK 逆解选解 | `src/ik_solver.py` | 多种子枚举（翻臂/翻腕）+ 关节限位过滤 + 最小关节变化量选解 |
| 碰撞检测 | `src/collision_checker.py` | 基于 `/check_state_validity`(FCL)，单状态 + 轨迹沿途稠密校验 |
| 梯形速度规划 | `src/trapezoid.py` | 纯函数，无 ROS 依赖，三角形/梯形速度曲线 + 反算验证 |
| 规划节点 | `src/planner_node.py` | 编排 IK/碰撞/速度规划、执行、TF 记录与指标计算 |

## 依赖

- ROS2（Humble 或更新）
- MoveIt2（提供 `/compute_ik`、`/check_state_validity` 服务）
- 机器人控制器（提供 `FollowJointTrajectory` action）
- Python：`scipy`、`matplotlib`、`PyYAML`

## 快速开始

1. 启动你的 MoveIt 与机器人控制器（需提供 IK、碰撞检测服务与轨迹控制 action）。
2. 按你的机器人修改 [`config/params.yaml`](config/params.yaml) 中的关节名、
   group、tcp/base 框架与关节限位。
3. 运行示例：

```bash
colcon build
source install/setup.bash
ros2 run col_de demo_waypoints --config <path/to/params.yaml>
# 或用 launch（自动定位已安装的 params.yaml）
ros2 launch col_de demo.launch.py
```

示例 [`src/demo/demo_waypoints.py`](src/demo/demo_waypoints.py) 演示了
「发布一个长方体障碍物 + 走一条直线路点」的完整流程，坐标均为占位值，
请按你的工作空间修改。

## 使用方法（作为库）

```python
from src.planner_node import PlannerNode, make_pose

node = PlannerNode(config_path="config/params.yaml")
node.wait_for_services()

pose = make_pose(0.5, 0.0, 0.3, euler_rad=[3.1416, 0.7854, 0.0],
                 frame_id=node.cfg.base_frame)
joints = node.solve_ik(pose, label="target")      # IK 选解
if joints is not None:
    timed = node.timed_from_joints(joints)        # 梯形速度规划
    ok, _ = node.execute(timed, label="target")   # 执行
```

直线路点可参考 `_plan_line` / `_move_line` 的写法：沿直线采样，逐点 IK，
并把上一次的解作为下一次的种子（种子锁定）以保证臂形连续。

## 目录结构

```
col_de/
├── src/
│   ├── config.py
│   ├── trapezoid.py
│   ├── ik_solver.py
│   ├── collision_checker.py
│   ├── planner_node.py
│   └── demo/
│       ├── simple_scene.py
│       └── demo_waypoints.py
├── config/params.yaml
├── launch/demo.launch.py
└── LICENSE
```

## 许可

[MIT](LICENSE)