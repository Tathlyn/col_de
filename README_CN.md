> [English](README.md) | 中文

# col_de

基于 **ROS2 + MoveIt2**的机械臂关节空间轨迹避障规划框架。

>本仓库包含 ROS2 + MoveIt2 环境下避障规划生成代码，用于演示六轴机械臂避障、RViz 可视化

核心思路： **「IK 逆解 + 沿途碰撞检测 + 梯形速度规划」**，在给定若干笛卡尔目标位姿（或一条直线/曲线路点）时，稳定生成满足关节限位、碰撞约束、速度与加速度约束的连续轨迹。

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

## 效果展示

### RRT规划轨迹

![RRT末端轨迹](docs/images/RRT_trajectory.png)

### 规划轨迹
六轴机械臂在ROS2环境中避障规划演示
![避障轨迹](docs/images/trajectory.png)

![规划耗时](docs/images/planning_time_comparison.png)

在该六轴复杂窄腔焊接任务中，受窄通道与随机性影响，RRT 容易出现规划超时、路径迂回，甚至无法完成直线焊缝；而确定性数值 IK 结合碰撞检测与工艺约束引导，更适合这类受约束的焊接任务，可实现毫秒级规划，并生成直接贴合焊缝的轨迹。

## C 库版测试效果
> 说明：以下效果图与指标来自同一算法封装为纯 C99 算法库后的测试，
> 并非 ROS Python 代码直接的运行输出。

成功率约 98%（200 组随机测试），焊接段 RMSE ≈ 0.02mm，
单条规划毫秒级完成，重复运行关节角偏差 < 1e-9 rad

### 三维轨迹（单组）
焊枪 TCP 在三维空间中的单组运动轨迹。
![三维轨迹](docs/images/tcp_3d.png)

### 多组规划轨迹叠加
多组随机位姿下规划结果叠加，对比不同工件位姿下的轨迹差异。
![多组轨迹叠加](docs/images/multi_3d.png)

### 覆盖率与失败样本
200 组随机测试覆盖的工件位姿空间。 5个失败样本集中在工件极端边界位姿，正常工况下全部通过。
![覆盖率与失败样本](docs/images/coverage_and_failures.png)

### 规划耗时分布
200 组随机测试的单次求解耗时箱线图。
![规划耗时箱线图](docs/images/planning_time_boxplot.png)

### 跟踪误差分布

![误差分布](docs/images/rmse_distribution.png)

焊接段 TCP 相对理想焊缝直线的跟踪误差 RMSE 分布。全部成功样本集中在 0.019~0.027 mm，均值约 0.023 mm，P95/P99 分布极窄，说明跟踪精度稳定、无离群漂移。

###  TCP位置误差
TCP 位置误差分量随焊接轨迹时间的变化（样本 sample_001）。
![位置误差](docs/images/tcp_position_error_components.png)

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
├── docs/images/
├── config/params.yaml
├── launch/demo.launch.py
└── LICENSE
```

## 许可

[MIT](LICENSE)
