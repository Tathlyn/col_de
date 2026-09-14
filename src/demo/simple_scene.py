"""小工具：往 /planning_scene 发布简单几何碰撞体（box / mesh）。

只用于示例场景的搭建，与规划算法本身无关，也不绑定任何具体工件。
"""

import struct

from geometry_msgs.msg import Pose, Point
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive, Mesh, MeshTriangle
from scipy.spatial.transform import Rotation as R


def add_box(node, name, size, xyz, euler=None, frame_id="base_link"):
    """发布一个长方体（BOX）碰撞体。

    参数
    ----
    size   (sx, sy, sz) 尺寸
    xyz    (x, y, z)    中心位置
    euler  (rx, ry, rz) 欧拉角（弧度），可选
    frame_id             参考坐标系
    """
    pub = _scene_publisher(node)
    co = CollisionObject()
    co.id = name
    co.header.frame_id = frame_id
    co.operation = CollisionObject.ADD

    box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(s) for s in size])
    pose = _make_pose(xyz, euler)
    co.primitives.append(box)
    co.primitive_poses.append(pose)

    _publish(pub, co)
    node.get_logger().info(f"碰撞体 '{name}' (box) 已添加于 {tuple(xyz)}")


def add_mesh(node, name, mesh_path, xyz, euler=None, frame_id="base_link", scale=0.001):
    """发布一个二进制 STL 网格碰撞体（默认 mm → m）。"""
    pub = _scene_publisher(node)
    vertices, triangles = _load_binary_stl(mesh_path, scale=scale)

    co = CollisionObject()
    co.id = name
    co.header.frame_id = frame_id
    co.operation = CollisionObject.ADD

    mesh = Mesh()
    mesh.vertices = [Point(x=v[0], y=v[1], z=v[2]) for v in vertices]
    mesh.triangles = [MeshTriangle(vertex_indices=list(t)) for t in triangles]
    co.meshes.append(mesh)
    co.mesh_poses.append(_make_pose(xyz, euler))

    _publish(pub, co)
    node.get_logger().info(f"碰撞体 '{name}' (mesh, {len(triangles)} 面) 已添加于 {tuple(xyz)}")


def remove_object(node, name, frame_id="base_link"):
    pub = _scene_publisher(node)
    co = CollisionObject()
    co.id = name
    co.header.frame_id = frame_id
    co.operation = CollisionObject.REMOVE
    _publish(pub, co)


def _scene_publisher(node):
    return node.create_publisher(PlanningScene, "/planning_scene", 10)


def _publish(pub, co):
    ps = PlanningScene()
    ps.is_diff = True
    ps.world.collision_objects.append(co)
    pub.publish(ps)


def _make_pose(xyz, euler):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = [float(v) for v in xyz]
    if euler:
        q = R.from_euler("xyz", euler).as_quat()
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
    else:
        pose.orientation.w = 1.0
    return pose


def _load_binary_stl(filepath, scale=1.0):
    with open(filepath, "rb") as f:
        f.read(80)  # 跳过 80 字节 header
        num_triangles = struct.unpack("<I", f.read(4))[0]
        vertices = []
        vertex_map = {}
        triangles = []
        for _ in range(num_triangles):
            f.read(12)  # 法向量
            tri_verts = []
            for _ in range(3):
                x, y, z = struct.unpack("<fff", f.read(12))
                v = (round(x * scale, 6), round(y * scale, 6), round(z * scale, 6))
                if v not in vertex_map:
                    vertex_map[v] = len(vertices)
                    vertices.append(v)
                tri_verts.append(vertex_map[v])
            triangles.append(tuple(tri_verts))
            f.read(2)  # attribute byte count
    return vertices, triangles