#!/usr/bin/env python3
import math
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import WrenchStamped

import casadi as ca


class MpcNode2D(Node):
    """
    2D MPC for lateral correction in tool frame.

    Inputs:
      - /mpc/lateral_error_vec   (Vector3): tool-frame error ex,ey,ez
      - /mpc/stability_index     (Float64): S in [0,1]
      - /mpc/normal_step         (Float64): sn (kept for compatibility; not required by 2D lateral MPC)
      - /ft_sensor               (WrenchStamped): safety gate (optional)

    Outputs:
      - /mpc/dxy_cmd             (Vector3): tool-frame incremental command (ux, uy, 0)
      - /mpc/safety_mode         (Bool)
      - /mpc/dk_cmd              (Float64)  (compat; always 0 unless you extend)
    """

    def __init__(self):
        super().__init__("mpc_node_2d")

        # horizon and weights
        self.N = int(self.declare_parameter("N", 10).value)
        self.q = float(self.declare_parameter("q", 10.0).value)          # state weight
        self.qf = float(self.declare_parameter("qf", 30.0).value)        # terminal weight
        self.r = float(self.declare_parameter("r", 1.0).value)           # smoothness weight
        self.lambda0 = float(self.declare_parameter("lambda0", 1.0).value)
        self.kappa = float(self.declare_parameter("kappa", 2.0).value)

        # step constraint (meters); scaled by stability
        self.dmax_base = float(self.declare_parameter("dmax_base", 0.002).value)

        # safety (F/T)
        self.use_ft_safety = bool(self.declare_parameter("use_ft_safety", True).value)
        self.force_threshold = float(self.declare_parameter("force_threshold", 25.0).value)
        self.torque_threshold = float(self.declare_parameter("torque_threshold", 2.0).value)
        self.stability_min = float(self.declare_parameter("stability_min", 0.2).value)

        # update rate
        self.ctrl_rate_hz = float(self.declare_parameter("ctrl_rate_hz", 20.0).value)

        # state cache
        self.e_vec = np.zeros(2)       # [ex, ey] in tool frame
        self.stability = 1.0
        self.u_prev = np.zeros(2)      # previous control (for smoothness)
        self.F_latest = np.zeros(3)
        self.M_latest = np.zeros(3)

        # build solver
        self.solver = self._build_qp_solver(self.N)

        qos = QoSProfile(depth=10)

        self.create_subscription(Vector3, "/mpc/lateral_error_vec", self.cb_e, qos)
        self.create_subscription(Float64, "/mpc/stability_index", self.cb_s, qos)
        self.create_subscription(Float64, "/mpc/normal_step", self.cb_sn, qos)  # kept for compat; unused
        self.create_subscription(WrenchStamped, "/ft_sensor", self.cb_ft, qos)

        self.pub_u = self.create_publisher(Vector3, "/mpc/dxy_cmd", qos)
        self.pub_safe = self.create_publisher(Bool, "/mpc/safety_mode", qos)
        self.pub_dk = self.create_publisher(Float64, "/mpc/dk_cmd", qos)  # compat

        period = 1.0 / max(1e-6, self.ctrl_rate_hz)
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(
            f"2D MPC started. N={self.N}, rate={self.ctrl_rate_hz}Hz, dmax_base={self.dmax_base}"
        )

    def cb_e(self, msg: Vector3):
        self.e_vec[0] = float(msg.x)
        self.e_vec[1] = float(msg.y)

    def cb_s(self, msg: Float64):
        self.stability = float(msg.data)

    def cb_sn(self, msg: Float64):
        # compatibility: not used in 2D lateral MPC, but keep subscription to avoid breaking pipeline
        pass

    def cb_ft(self, msg: WrenchStamped):
        self.F_latest = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z], dtype=float)
        self.M_latest = np.array([msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z], dtype=float)

    def _check_safety(self) -> bool:
        if self.stability < self.stability_min:
            return True
        if not self.use_ft_safety:
            return False
        if np.linalg.norm(self.F_latest) > self.force_threshold:
            return True
        if np.linalg.norm(self.M_latest) > self.torque_threshold:
            return True
        return False

    def _build_qp_solver(self, N: int):
        """
        QP:
          minimize sum_{k=0}^{N-1} (e_k^T Q e_k) + lam(S) * sum ( (u_k - u_{k-1})^T R (u_k - u_{k-1}) )
                  + e_N^T Qf e_N
          s.t. e_{k+1} = e_k - u_k
               |u_k|_inf <= dmax(S)

        Decision variables: U (2N)
        Parameters: e0(2), u_prev(2), S
        """
        U = ca.SX.sym("U", 2*N)         # [ux0, uy0, ux1, uy1, ...]
        e0 = ca.SX.sym("e0", 2)
        u_prev = ca.SX.sym("u_prev", 2)
        S = ca.SX.sym("S")

        q = self.q
        qf = self.qf
        r = self.r
        lam = self.lambda0 * ca.exp(self.kappa * (1.0 - S))
        dmax = self.dmax_base * S

        Q = ca.DM([[q, 0.0],[0.0, q]])
        Qf = ca.DM([[qf, 0.0],[0.0, qf]])
        R = ca.DM([[r, 0.0],[0.0, r]])

        # rollout and cost
        e = e0
        J = 0
        up = u_prev
        g = []  # inequality constraints (box on U)
        for k in range(N):
            uk = ca.vertcat(U[2*k], U[2*k+1])
            # state cost
            J += ca.mtimes([e.T, Q, e])
            # smoothness
            du = uk - up
            J += lam * ca.mtimes([du.T, R, du])

            # dynamics
            e = e - uk
            up = uk

            # box constraints via g: -dmax <= ux <= dmax, -dmax <= uy <= dmax
            g += [uk[0], uk[1]]

        # terminal
        J += ca.mtimes([e.T, Qf, e])

        g = ca.vertcat(*g)  # length 2N
        lbg = [-dmax]*(2*N)
        ubg = [ dmax]*(2*N)

        qp = {"x": U, "f": J, "g": g, "p": ca.vertcat(e0, u_prev, S)}
        solver = ca.qpsol("solver", "osqp", qp)

        return {"solver": solver, "N": N}

    def _solve(self, e_vec, u_prev, S):
        N = self.solver["N"]
        solver = self.solver["solver"]
        p = ca.vertcat(float(e_vec[0]), float(e_vec[1]), float(u_prev[0]), float(u_prev[1]), float(S))
        sol = solver(p=p, lbg=-np.ones(2*N)*self.dmax_base*float(S), ubg=np.ones(2*N)*self.dmax_base*float(S))
        Uopt = np.array(sol["x"]).reshape(-1)
        return np.array([Uopt[0], Uopt[1]])

    def on_timer(self):
        safety = self._check_safety()
        self.pub_safe.publish(Bool(data=bool(safety)))

        if safety:
            # publish zero command when unsafe
            v = Vector3()
            v.x = 0.0; v.y = 0.0; v.z = 0.0
            self.pub_u.publish(v)
            self.pub_dk.publish(Float64(data=0.0))
            return

        # solve
        u = self._solve(self.e_vec, self.u_prev, max(0.0, min(1.0, self.stability)))
        self.u_prev = u.copy()

        v = Vector3()
        v.x = float(u[0])
        v.y = float(u[1])
        v.z = 0.0
        self.pub_u.publish(v)

        # compat scalar dk
        self.pub_dk.publish(Float64(data=0.0))


def main():
    rclpy.init()
    node = MpcNode2D()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
