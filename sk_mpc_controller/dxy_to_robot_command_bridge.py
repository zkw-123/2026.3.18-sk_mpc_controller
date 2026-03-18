#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import Bool, String
from geometry_msgs.msg import Vector3, PoseStamped


def quat_to_rot_matrix(x: float, y: float, z: float, w: float):
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n <= 1e-12:
        return [[1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0]]
    x, y, z, w = x/n, y/n, z/n, w/n
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z
    return [
        [1.0 - 2.0*(yy + zz), 2.0*(xy - wz),       2.0*(xz + wy)],
        [2.0*(xy + wz),       1.0 - 2.0*(xx + zz), 2.0*(yz - wx)],
        [2.0*(xz - wy),       2.0*(yz + wx),       1.0 - 2.0*(xx + yy)],
    ]


def mat_vec(m, v):
    return [
        m[0][0]*v[0] + m[0][1]*v[1] + m[0][2]*v[2],
        m[1][0]*v[0] + m[1][1]*v[1] + m[1][2]*v[2],
        m[2][0]*v[0] + m[2][1]*v[1] + m[2][2]*v[2],
    ]


class DxyToRobotCommandBridge(Node):
    def __init__(self):
        super().__init__("dxy_to_robot_command_bridge")
        qos = QoSProfile(depth=10)

        self.dxy_topic = self.declare_parameter("dxy_topic", "/mpc/dxy_cmd").value
        self.safety_topic = self.declare_parameter("safety_topic", "/mpc/safety_mode").value
        self.robot_pose_topic = self.declare_parameter("robot_pose_topic", "/robot_current_pose").value
        self.cmd_topic = self.declare_parameter("cmd_topic", "/robot_command").value

        self.cmd_rate_hz = float(self.declare_parameter("cmd_rate_hz", 0.5).value)  # 建议默认降到 0.2~0.5
        self.max_step = float(self.declare_parameter("max_step", 0.0005).value)
        self.deadband = float(self.declare_parameter("deadband", 1e-4).value)
        self.scale = float(self.declare_parameter("scale", 1.0).value)

        # 核心开关
        self.use_tool_frame = bool(self.declare_parameter("use_tool_frame", False).value)
        self.lock_z = bool(self.declare_parameter("lock_z", True).value)

        # 注意：你当前代码里 safety=True 就 return（阻止输出）
        # 这等价于 safety_topic 表示“UNSAFE/阻止运动”。
        self.block_motion = False

        self.latest_pose: Optional[PoseStamped] = None
        self.latest_u = [0.0, 0.0, 0.0]

        self.create_subscription(Vector3, self.dxy_topic, self.cb_u, qos)
        self.create_subscription(Bool, self.safety_topic, self.cb_safe, qos)
        self.create_subscription(PoseStamped, self.robot_pose_topic, self.cb_pose, qos)

        self.pub_cmd = self.create_publisher(String, self.cmd_topic, qos)

        period = 1.0 / max(1e-6, self.cmd_rate_hz)
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(
            f"DXY bridge started. dxy_topic={self.dxy_topic}, pose={self.robot_pose_topic}, "
            f"rate={self.cmd_rate_hz}Hz, use_tool_frame={self.use_tool_frame}, lock_z={self.lock_z}"
        )

    def cb_u(self, msg: Vector3):
        self.latest_u = [float(msg.x), float(msg.y), float(msg.z)]

    def cb_safe(self, msg: Bool):
        # safety_mode=True => 阻止输出（按你现在写法的语义）
        self.block_motion = bool(msg.data)

    def cb_pose(self, msg: PoseStamped):
        self.latest_pose = msg

    def on_timer(self):
        if self.block_motion:
            return
        if self.latest_pose is None:
            self.get_logger().warn("No /robot_current_pose yet; cannot generate move command.")
            return

        ux = self.scale * self.latest_u[0]
        uy = -self.scale * self.latest_u[1]
        # 关键：MPC 的 z 本来就是 0；这里直接强制为 0，避免未来有人改 MPC 引入 z
        uz = 0.0

        # deadband（只看 xy）
        if abs(ux) < self.deadband and abs(uy) < self.deadband:
            return

        # clamp per-axis（只 clamp xy）
        ux = max(-self.max_step, min(self.max_step, ux))
        uy = max(-self.max_step, min(self.max_step, uy))

        p = self.latest_pose.pose.position
        q = self.latest_pose.pose.orientation

        if self.use_tool_frame:
            # tool-frame lateral -> base-frame increment
            R = quat_to_rot_matrix(float(q.x), float(q.y), float(q.z), float(q.w))
            delta_base = mat_vec(R, [ux, uy, uz])
            dx, dy = float(delta_base[0]), float(delta_base[1])
        else:
            # treat MPC output as base-frame lateral increment
            dx, dy = float(ux), float(uy)

        x = float(p.x) + dx
        y = float(p.y) + dy
        z = float(p.z) if self.lock_z else float(p.z)  # 目前 lock_z 只允许固定；如你未来要放开再扩展

        msg = String()
        msg.data = f"down {x:.6f} {y:.6f} {z:.6f} 0"
        self.pub_cmd.publish(msg)


def main():
    rclpy.init()
    node = DxyToRobotCommandBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()

