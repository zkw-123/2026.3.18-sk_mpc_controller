#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import Float64, Bool, String
from geometry_msgs.msg import PoseStamped


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


class MpcToRobotCommandBridge(Node):
    """
    Subscribe:
      - /mpc/dk_cmd (Float64)
      - /mpc/safety_mode (Bool)
      - /robot_current_pose (PoseStamped) [required for move modes]

    Publish:
      - /robot_command (String), using control_command package format

    Modes:
      - insert:          "insert <dk>"  (relative along tool Z in control_command)
      - move_base_x:     absolute move, x := x + dk
      - move_base_y:     absolute move, y := y + dk
      - move_tool_x:     absolute move, along EE(tool) +X axis by dk
      - move_tool_y:     absolute move, along EE(tool) +Y axis by dk
      - move_tool_z:     absolute move, along EE(tool) +Z axis by dk
    """

    def __init__(self):
        super().__init__("mpc_to_robot_command_bridge")

        qos = QoSProfile(depth=10)

        self.dk_topic = self.declare_parameter("dk_topic", "/mpc/dk_cmd").value
        self.safety_topic = self.declare_parameter("safety_topic", "/mpc/safety_mode").value
        self.cmd_topic = self.declare_parameter("cmd_topic", "/robot_command").value
        self.robot_pose_topic = self.declare_parameter("robot_pose_topic", "/robot_current_pose").value

        self.cmd_rate_hz = float(self.declare_parameter("cmd_rate_hz", 1.0).value)
        self.max_step = float(self.declare_parameter("max_step", 0.002).value)
        self.deadband = float(self.declare_parameter("deadband", 0.0001).value)
        self.scale = float(self.declare_parameter("scale", 1.0).value)
        self.mode = str(self.declare_parameter("mode", "insert").value).lower()

        self.dk_latest = 0.0
        self.safety = False
        self.latest_pose: Optional[PoseStamped] = None

        self.sub_dk = self.create_subscription(Float64, self.dk_topic, self.cb_dk, qos)
        self.sub_safe = self.create_subscription(Bool, self.safety_topic, self.cb_safe, qos)
        self.sub_pose = self.create_subscription(PoseStamped, self.robot_pose_topic, self.cb_pose, qos)

        self.pub_cmd = self.create_publisher(String, self.cmd_topic, qos)

        period = 1.0 / max(1e-3, self.cmd_rate_hz)
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(
            f"Bridge started. mode={self.mode}, dk_topic={self.dk_topic}, cmd_topic={self.cmd_topic}, "
            f"robot_pose_topic={self.robot_pose_topic}, cmd_rate_hz={self.cmd_rate_hz}, max_step={self.max_step}"
        )

    def cb_dk(self, msg: Float64):
        self.dk_latest = float(msg.data)

    def cb_safe(self, msg: Bool):
        self.safety = bool(msg.data)

    def cb_pose(self, msg: PoseStamped):
        self.latest_pose = msg

    def _build_move_command_from_current(self, delta_base_xyz) -> Optional[str]:
        """Generate absolute 'move x y z' command using current pose + delta in base frame."""
        if self.latest_pose is None:
            self.get_logger().warn("No /robot_current_pose received yet; cannot build move command.")
            return None

        p = self.latest_pose.pose.position
        x = float(p.x) + float(delta_base_xyz[0])
        y = float(p.y) + float(delta_base_xyz[1])
        z = float(p.z) + float(delta_base_xyz[2])
        return f"move {x:.6f} {y:.6f} {z:.6f}"

    def on_timer(self):
        # Safety gate
        if self.safety:
            return

        dk = self.scale * self.dk_latest

        # deadband
        if abs(dk) < self.deadband:
            return

        # clamp step size
        if self.max_step > 0:
            dk = max(-self.max_step, min(self.max_step, dk))

        cmd_str = None

        if self.mode == "insert":
            cmd_str = f"insert {dk:.6f}"

        elif self.mode == "move_base_x":
            cmd_str = self._build_move_command_from_current([dk, 0.0, 0.0])

        elif self.mode == "move_base_y":
            cmd_str = self._build_move_command_from_current([0.0, dk, 0.0])

        elif self.mode in ("move_tool_x", "move_tool_y", "move_tool_z"):
            if self.latest_pose is None:
                self.get_logger().warn("No /robot_current_pose received yet; cannot build tool-axis move command.")
                return

            q = self.latest_pose.pose.orientation
            R = quat_to_rot_matrix(float(q.x), float(q.y), float(q.z), float(q.w))  # tool->base

            if self.mode == "move_tool_x":
                delta_base = mat_vec(R, [dk, 0.0, 0.0])
            elif self.mode == "move_tool_y":
                delta_base = mat_vec(R, [0.0, dk, 0.0])
            else:  # move_tool_z
                delta_base = mat_vec(R, [0.0, 0.0, dk])

            cmd_str = self._build_move_command_from_current(delta_base)

        else:
            self.get_logger().warn(
                f"Unsupported mode='{self.mode}'. Use one of: "
                "insert, move_base_x, move_base_y, move_tool_x, move_tool_y, move_tool_z."
            )
            return

        if cmd_str is None:
            return

        msg = String()
        msg.data = cmd_str
        self.pub_cmd.publish(msg)


def main():
    rclpy.init()
    node = MpcToRobotCommandBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
