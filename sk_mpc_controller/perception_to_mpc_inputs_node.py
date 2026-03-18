#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Float32, Float64
from geometry_msgs.msg import PointStamped, PoseStamped


def quat_to_rot_matrix(x: float, y: float, z: float, w: float):
    """Quaternion (x,y,z,w) to 3x3 rotation matrix (right-handed)."""
    # Normalize to be safe
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
    """Return m^T * v for 3x3 matrix m and 3-vector v."""
    return [
        m[0][0]*v[0] + m[1][0]*v[1] + m[2][0]*v[2],
        m[0][1]*v[0] + m[1][1]*v[1] + m[2][1]*v[2],
        m[0][2]*v[0] + m[1][2]*v[1] + m[2][2]*v[2],
    ]


class PerceptionToMpcInputsNode(Node):
    """
    Bridge (Perception -> MPC inputs)

    Subscribes:
      - /perception/target_point (geometry_msgs/PointStamped)
      - /perception/sk           (std_msgs/Float32)
      - /robot_current_pose      (geometry_msgs/PoseStamped)  [optional, from robot_status_publisher]

    Publishes:
      - /mpc/lateral_error   (std_msgs/Float64)
      - /mpc/normal_step     (std_msgs/Float64)
      - /mpc/stability_index (std_msgs/Float64)

    Lateral error computation modes:
      A) Robot-aware (recommended when target_point is in robot base frame):
         e = (p_target - p_ee) expressed in EE(tool) frame; pick axis x/y or xy_norm.
      B) Fallback (legacy):
         e = || (target - ref_target) || in target_point coordinate (no robot info).
    """

    def __init__(self):
        super().__init__("perception_to_mpc_inputs")

        # Topics
        self.target_topic = self.declare_parameter("target_topic", "/perception/target_point").value
        self.sk_topic = self.declare_parameter("sk_topic", "/perception/sk").value
        self.robot_pose_topic = self.declare_parameter("robot_pose_topic", "/robot_current_pose").value

        # Publish topics
        self.lat_topic = self.declare_parameter("lat_topic", "/mpc/lateral_error").value
        self.sn_topic = self.declare_parameter("sn_topic", "/mpc/normal_step").value
        self.stab_topic = self.declare_parameter("stab_topic", "/mpc/stability_index").value

        # Rates / constants
        self.publish_rate_hz = float(self.declare_parameter("publish_rate_hz", 10.0).value)
        self.normal_step = float(self.declare_parameter("normal_step", 0.001).value)

        # Reference target fallback
        self.use_first_target_as_ref = bool(self.declare_parameter("use_first_target_as_ref", True).value)
        self.x_ref = self.declare_parameter("x_ref", None).value
        self.y_ref = self.declare_parameter("y_ref", None).value

        # Robot-aware lateral error
        self.use_robot_pose_for_lateral_error = bool(
            self.declare_parameter("use_robot_pose_for_lateral_error", True).value
        )
        # "x" | "y" | "xy_norm"
        self.lateral_axis = str(self.declare_parameter("lateral_axis", "x").value).lower()
        self.require_frame_match = bool(self.declare_parameter("require_frame_match", False).value)
        self.fallback_to_target_drift = bool(self.declare_parameter("fallback_to_target_drift", True).value)

        qos = qos_profile_sensor_data

        # Subscriptions
        self.latest_target: Optional[PointStamped] = None
        self.latest_sk: Optional[float] = None
        self.latest_robot_pose: Optional[PoseStamped] = None

        self.create_subscription(PointStamped, self.target_topic, self._on_target, qos)
        self.create_subscription(Float32, self.sk_topic, self._on_sk, qos)
        self.create_subscription(PoseStamped, self.robot_pose_topic, self._on_robot_pose, qos)

        # Publishers
        self.pub_lat = self.create_publisher(Float64, self.lat_topic, 10)
        self.pub_sn = self.create_publisher(Float64, self.sn_topic, 10)
        self.pub_stab = self.create_publisher(Float64, self.stab_topic, 10)

        # Timer
        period = 1.0 / self.publish_rate_hz if self.publish_rate_hz > 0 else 0.1
        self.timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            "Perception->MPC bridge started: "
            f"target_topic={self.target_topic}, sk_topic={self.sk_topic}, robot_pose_topic={self.robot_pose_topic}, "
            f"publish_rate_hz={self.publish_rate_hz}, normal_step={self.normal_step}, "
            f"use_robot_pose_for_lateral_error={self.use_robot_pose_for_lateral_error}, lateral_axis={self.lateral_axis}"
        )

    def _on_target(self, msg: PointStamped):
        self.latest_target = msg
        # Lock reference on first target if requested (fallback mode)
        if self.use_first_target_as_ref and self.x_ref is None and self.y_ref is None:
            self.x_ref = float(msg.point.x)
            self.y_ref = float(msg.point.y)
            self.get_logger().info(
                f"[fallback ref] Reference locked to first target: x_ref={self.x_ref:.6f}, y_ref={self.y_ref:.6f}"
            )

    def _on_sk(self, msg: Float32):
        self.latest_sk = float(msg.data)

    def _on_robot_pose(self, msg: PoseStamped):
        self.latest_robot_pose = msg

    def _compute_lateral_error_robot_aware(self) -> Optional[float]:
        """Compute signed lateral error using robot EE pose (tool frame)."""
        if self.latest_target is None or self.latest_robot_pose is None:
            return None

        # Frame check (optional)
        t_frame = (self.latest_target.header.frame_id or "").strip()
        p_frame = (self.latest_robot_pose.header.frame_id or "").strip()
        if self.require_frame_match and t_frame and p_frame and (t_frame != p_frame):
            self.get_logger().warn(
                f"Frame mismatch: target_frame='{t_frame}' vs robot_pose_frame='{p_frame}'. "
                "Set require_frame_match:=false to ignore, or publish in same frame."
            )
            return None

        pt = self.latest_target.point
        pr = self.latest_robot_pose.pose.position
        qr = self.latest_robot_pose.pose.orientation

        # delta in base frame
        dx = float(pt.x) - float(pr.x)
        dy = float(pt.y) - float(pr.y)
        dz = float(pt.z) - float(pr.z)

        # rotation: tool->base. We need base->tool, so apply transpose via mat_t_vec helper
        R = quat_to_rot_matrix(float(qr.x), float(qr.y), float(qr.z), float(qr.w))
        # delta_tool = R^T * delta_base
        delta_tool = mat_t_vec(R, [dx, dy, dz])

        if self.lateral_axis == "x":
            return float(delta_tool[0])
        if self.lateral_axis == "y":
            return float(delta_tool[1])
        if self.lateral_axis in ("xy", "xy_norm", "norm"):
            return float(math.sqrt(delta_tool[0]**2 + delta_tool[1]**2))

        self.get_logger().warn(f"Unknown lateral_axis='{self.lateral_axis}'. Use 'x', 'y', or 'xy_norm'.")
        return None

    def _compute_lateral_error_fallback(self) -> Optional[float]:
        """Legacy fallback: magnitude of target drift relative to reference target."""
        if self.latest_target is None:
            return None
        if self.x_ref is None or self.y_ref is None:
            return None

        x = float(self.latest_target.point.x)
        y = float(self.latest_target.point.y)
        dx = x - float(self.x_ref)
        dy = y - float(self.y_ref)
        return float(math.sqrt(dx * dx + dy * dy))

    def _tick(self):
        # Always publish normal_step (constant)
        self.pub_sn.publish(Float64(data=float(self.normal_step)))

        # Publish stability_index if available
        if self.latest_sk is not None:
            self.pub_stab.publish(Float64(data=float(self.latest_sk)))

        # Publish lateral_error
        e_lat = None
        if self.use_robot_pose_for_lateral_error:
            e_lat = self._compute_lateral_error_robot_aware()

        if e_lat is None and self.fallback_to_target_drift:
            e_lat = self._compute_lateral_error_fallback()

        if e_lat is not None:
            self.pub_lat.publish(Float64(data=float(e_lat)))


def main():
    rclpy.init()
    node = PerceptionToMpcInputsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
