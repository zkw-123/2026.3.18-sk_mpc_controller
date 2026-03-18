#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Float32
from std_msgs.msg import Float64
from geometry_msgs.msg import PointStamped, PoseStamped, Vector3


def quat_to_rot_matrix(x: float, y: float, z: float, w: float):
    """Quaternion (x,y,z,w) -> 3x3 rotation matrix (tool->base)."""
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


def mat_t_vec(m, v):
    """m^T * v for 3x3 matrix m, 3-vector v."""
    return [
        m[0][0]*v[0] + m[1][0]*v[1] + m[2][0]*v[2],
        m[0][1]*v[0] + m[1][1]*v[1] + m[2][1]*v[2],
        m[0][2]*v[0] + m[1][2]*v[1] + m[2][2]*v[2],
    ]


class PerceptionToMpcInputsNode(Node):
    """
    Perception -> MPC Inputs

    Subscribes:
      - /perception/target_point (geometry_msgs/PointStamped)  [recommended: base_link frame, meters]
      - /perception/sk           (std_msgs/Float32)
      - /robot_current_pose      (geometry_msgs/PoseStamped)

    Publishes:
      - /mpc/lateral_error_vec   (geometry_msgs/Vector3)  (tool-frame error: ex, ey, ez)
      - /mpc/lateral_error       (std_msgs/Float64)       (compat scalar; default = sqrt(ex^2+ey^2))
      - /mpc/normal_step         (std_msgs/Float64)
      - /mpc/stability_index     (std_msgs/Float64)
    """

    def __init__(self):
        super().__init__("perception_to_mpc_inputs")

        # topics
        self.target_topic = self.declare_parameter("target_topic", "/perception/target_point").value
        self.sk_topic = self.declare_parameter("sk_topic", "/perception/sk").value
        self.robot_pose_topic = self.declare_parameter("robot_pose_topic", "/robot_current_pose").value

        self.lat_vec_topic = self.declare_parameter("lat_vec_topic", "/mpc/lateral_error_vec").value
        self.lat_scalar_topic = self.declare_parameter("lat_scalar_topic", "/mpc/lateral_error").value
        self.sn_topic = self.declare_parameter("sn_topic", "/mpc/normal_step").value
        self.stab_topic = self.declare_parameter("stab_topic", "/mpc/stability_index").value

        self.publish_rate_hz = float(self.declare_parameter("publish_rate_hz", 20.0).value)
        self.normal_step = float(self.declare_parameter("normal_step", 0.001).value)

        # scalar definition: "xy_norm" | "x" | "y"
        self.scalar_mode = str(self.declare_parameter("scalar_mode", "xy_norm").value).lower()

        # optional frame match check
        self.require_frame_match = bool(self.declare_parameter("require_frame_match", False).value)

        qos = qos_profile_sensor_data
        self.latest_target: Optional[PointStamped] = None
        self.latest_sk: Optional[float] = None
        self.latest_pose: Optional[PoseStamped] = None

        self.create_subscription(PointStamped, self.target_topic, self._on_target, qos)
        self.create_subscription(Float32, self.sk_topic, self._on_sk, qos)
        self.create_subscription(PoseStamped, self.robot_pose_topic, self._on_pose, qos)

        self.pub_lat_vec = self.create_publisher(Vector3, self.lat_vec_topic, 10)
        self.pub_lat_scalar = self.create_publisher(Float64, self.lat_scalar_topic, 10)
        self.pub_sn = self.create_publisher(Float64, self.sn_topic, 10)
        self.pub_stab = self.create_publisher(Float64, self.stab_topic, 10)

        period = 1.0 / max(1e-6, self.publish_rate_hz)
        self.timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            f"Started. target={self.target_topic}, sk={self.sk_topic}, pose={self.robot_pose_topic}, "
            f"pub_rate={self.publish_rate_hz}Hz, scalar_mode={self.scalar_mode}"
        )

    def _on_target(self, msg: PointStamped):
        self.latest_target = msg

    def _on_sk(self, msg: Float32):
        self.latest_sk = float(msg.data)

    def _on_pose(self, msg: PoseStamped):
        self.latest_pose = msg

    def _compute_error_tool(self):
        if self.latest_target is None or self.latest_pose is None:
            return None

        t_frame = (self.latest_target.header.frame_id or "").strip()
        p_frame = (self.latest_pose.header.frame_id or "").strip()
        if self.require_frame_match and t_frame and p_frame and t_frame != p_frame:
            self.get_logger().warn(
                f"Frame mismatch: target='{t_frame}', pose='{p_frame}'. "
                "Set require_frame_match:=false or publish in same frame."
            )
            return None

        pt = self.latest_target.point
        pr = self.latest_pose.pose.position
        qr = self.latest_pose.pose.orientation

        dx = float(pt.x) - float(pr.x)
        dy = float(pt.y) - float(pr.y)
        dz = float(pt.z) - float(pr.z)

        R = quat_to_rot_matrix(float(qr.x), float(qr.y), float(qr.z), float(qr.w))  # tool->base
        d_tool = mat_t_vec(R, [dx, dy, dz])  # base->tool

        return d_tool  # [ex,ey,ez]

    def _tick(self):
        # publish normal step & stability
        self.pub_sn.publish(Float64(data=float(self.normal_step)))
        if self.latest_sk is not None:
            self.pub_stab.publish(Float64(data=float(self.latest_sk)))

        d_tool = self._compute_error_tool()
        if d_tool is None:
            return

        ex, ey, ez = d_tool

        v = Vector3()
        v.x = float(ex)
        v.y = float(ey)
        v.z = float(ez)
        self.pub_lat_vec.publish(v)

        if self.scalar_mode == "x":
            e = ex
        elif self.scalar_mode == "y":
            e = ey
        else:
            e = math.sqrt(ex*ex + ey*ey)
        self.pub_lat_scalar.publish(Float64(data=float(e)))


def main():
    rclpy.init()
    node = PerceptionToMpcInputsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
