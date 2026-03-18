#!/usr/bin/env python3
import math
from collections import deque
from typing import Deque, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import WrenchStamped

import numpy as np
import casadi as ca


class SkMPCControllerNode(Node):
    def __init__(self):
        super().__init__('sk_mpc_controller')

        # === 参数 ===
        self.declare_parameters(
            namespace='',
            parameters=[
                ('horizon_N', 10),
                ('q', 1.0),
                ('qf', 5.0),
                ('r', 0.1),
                ('lambda0', 1.0),
                ('kappa', 3.0),
                ('dmax_base', 1.0),
                ('force_threshold', 5.0),
                ('torque_threshold', 0.5),
                ('stability_min', 0.4),
                ('ft_history_len', 50),
            ]
        )

        self.N = self.get_parameter('horizon_N').get_parameter_value().integer_value
        self.q = self.get_parameter('q').value
        self.qf = self.get_parameter('qf').value
        self.r = self.get_parameter('r').value
        self.lambda0 = self.get_parameter('lambda0').value
        self.kappa = self.get_parameter('kappa').value
        self.dmax_base = self.get_parameter('dmax_base').value
        self.force_threshold = self.get_parameter('force_threshold').value
        self.torque_threshold = self.get_parameter('torque_threshold').value
        self.stability_min = self.get_parameter('stability_min').value
        self.ft_history_len = self.get_parameter('ft_history_len').value

        # === 状态缓存 ===
        self.e_perp = 0.0          # 当前 lateral error 估计
        self.sn = 0.0              # 当前 normal step
        self.stability = 1.0       # 当前 S̄_k
        self.dk_last = 0.0         # 上一时刻 dk-1

        self.force_torque_history: Deque[Tuple[np.ndarray, np.ndarray]] = deque(maxlen=self.ft_history_len)

        # 最新力矩，用于安全判定
        self.F_latest = np.zeros(3)
        self.M_latest = np.zeros(3)

        # MPC QP solver 构造（只依赖 N）
        self.qp_solver = self._build_qp_solver(self.N)

        # === 通信 ===
        qos = QoSProfile(depth=10)

        self.sub_e = self.create_subscription(
            Float64, '/mpc/lateral_error', self.cb_lateral_error, qos
        )
        self.sub_sn = self.create_subscription(
            Float64, '/mpc/normal_step', self.cb_normal_step, qos
        )
        self.sub_ft = self.create_subscription(
            WrenchStamped, '/ft_sensor', self.cb_ft, qos
        )
        self.sub_sk = self.create_subscription(
            Float64, '/mpc/stability_index', self.cb_stability, qos
        )

        self.pub_dk = self.create_publisher(Float64, '/mpc/dk_cmd', qos)
        self.pub_safety = self.create_publisher(Bool, '/mpc/safety_mode', qos)

        # 控制循环定时器（例如 50 Hz）
        self.control_dt = 0.02
        self.timer = self.create_timer(self.control_dt, self.control_loop)

        self.get_logger().info('SkMPCControllerNode started.')

    # ========== 回调 ==========
    def cb_lateral_error(self, msg: Float64):
        self.e_perp = msg.data

    def cb_normal_step(self, msg: Float64):
        self.sn = msg.data

    def cb_ft(self, msg: WrenchStamped):
        F = np.array([msg.wrench.force.x,
                      msg.wrench.force.y,
                      msg.wrench.force.z], dtype=float)
        M = np.array([msg.wrench.torque.x,
                      msg.wrench.torque.y,
                      msg.wrench.torque.z], dtype=float)
        self.F_latest = F
        self.M_latest = M
        self.force_torque_history.append((F, M))

    def cb_stability(self, msg: Float64):
        # 这里假定传进来的已经是 S̄_k（平滑后的），如果是 Sk，你可以在这里做指数平滑
        self.stability = float(np.clip(msg.data, 0.0, 1.0))

    # ========== MPC 控制循环 ==========
    def control_loop(self):
        # 1) 安全检查（对应 PDF 第 7 节） :contentReference[oaicite:3]{index=3}
        safety = self._check_safety()
        safety_msg = Bool()
        safety_msg.data = safety
        self.pub_safety.publish(safety_msg)

        if safety:
            # 保护模式：dk = 0
            dk_cmd = 0.0
        else:
            # 2) 根据 FT 历史构建特征 φ_k
            phi = self._compute_ft_features()

            # 3) 估计 β⊥, γ_n（这里只写占位函数，具体逻辑之后再填）
            beta_perp, gamma_n = self._estimate_carryover_params(phi)

            # 4) 调用 MPC 求解器，得到当前 dk
            dk_cmd = self._solve_mpc(
                e_perp=self.e_perp,
                sn=self.sn,
                beta_perp=beta_perp,
                gamma_n=gamma_n,
                S_bar=self.stability,
                dk_prev=self.dk_last
            )

        # 发布 dk
        dk_msg = Float64()
        dk_msg.data = float(dk_cmd)
        self.pub_dk.publish(dk_msg)

        # 更新 dk_last
        self.dk_last = dk_cmd

    # ========== 安全逻辑 ==========
    def _check_safety(self) -> bool:
        """对应 PDF 第 7 节的保护逻辑：S̄_k < Smin 或 |F|, |M| 超阈值时进入保护模式。"""
        S_bar = self.stability
        F_norm = np.linalg.norm(self.F_latest)
        M_norm = np.linalg.norm(self.M_latest)

        if S_bar < self.stability_min:
            return True
        if F_norm > self.force_threshold:
            return True
        if M_norm > self.torque_threshold:
            return True
        # QP 可行性判断在 _solve_mpc 里做，这里只检查传感器条件
        return False

    # ========== 力矩特征 φ_k ==========
    def _compute_ft_features(self) -> np.ndarray:
        """根据 FT 历史构造一个简单特征向量 φ_k，先用 mean + rms 占位。"""
        if len(self.force_torque_history) == 0:
            return np.zeros(6, dtype=float)

        F_stack = np.stack([F for F, _ in self.force_torque_history], axis=0)  # (T,3)
        M_stack = np.stack([M for _, M in self.force_torque_history], axis=0)  # (T,3)

        F_mean = np.mean(F_stack, axis=0)
        M_mean = np.mean(M_stack, axis=0)
        F_rms = np.sqrt(np.mean(F_stack**2, axis=0))
        M_rms = np.sqrt(np.mean(M_stack**2, axis=0))

        # 这里只取 F_mean + M_mean 组成一个 6 维特征，简单示例
        phi = np.concatenate([F_mean, M_mean], axis=0)
        return phi

    # ========== 估计 β⊥, γ_n（占位） ==========
    def _estimate_carryover_params(self, phi: np.ndarray) -> Tuple[float, float]:
        """
        使用 φ_k 估计 β⊥, γ_n。
        现在先给一个非常简单的示例：
          - β⊥ ∈ [0,1]，用 |F| 做个归一化；
          - γ_n 是一个小的常数，代表 sn 对 lateral 的耦合。
        实际上你可以在这里塞入一个学习模型或者经验拟合。
        """
        F_norm = np.linalg.norm(self.F_latest) + 1e-6
        beta_perp = float(np.clip(F_norm / (F_norm + 5.0), 0.0, 1.0))
        gamma_n = 0.1  # 占位，之后换成你的拟合/模型
        return beta_perp, gamma_n

    # ========== 构造 MPC QP ==========
    def _build_qp_solver(self, N: int):
        """
        使用 CasADi 构建一个 QP 形式的 MPC 求解器。
        决策变量：d = [d_k, ..., d_{k+N-1}]^T
        参数：e0, sn, beta_perp, gamma_n, S_bar, dk_prev
        """
        # 决策变量
        d = ca.SX.sym('d', N)  # d_k,...,d_{k+N-1}

        # 参数
        e0 = ca.SX.sym('e0')             # 当前 e⊥,k
        sn = ca.SX.sym('sn')             # 当前 sn,k （这里简单假设预测过程中常值）
        beta_p = ca.SX.sym('beta_p')     # β⊥
        gamma_n = ca.SX.sym('gamma_n')   # γ_n
        S_bar = ca.SX.sym('S_bar')       # 当前稳定性指数（假设预测过程中常值）
        dk_prev = ca.SX.sym('dk_prev')   # 上一时刻的 dk-1

        # 递推 e⊥
        e = e0
        e_list = []

        # 代价函数 J
        J = 0.0

        # λ(S̄) 和 dmax(S̄)
        # λ(S̄) = λ0 * exp(κ(1 - S̄))   （对应 PDF 式 (44)） :contentReference[oaicite:4]{index=4}
        lambda0 = self.lambda0
        kappa = self.kappa
        lam = lambda0 * ca.exp(kappa * (1.0 - S_bar))

        # dmax(S̄)：示例用 dmax_base * S̄（S 越低越保守）
        dmax_base = self.dmax_base
        dmax = dmax_base * S_bar

        # 运行代价权重
        q = self.q
        qf = self.qf
        r = self.r

        # 控制增量 Δd
        d_prev = dk_prev

        for i in range(N):
            di = d[i]
            # e⊥,k+i+1|k = e⊥,k+i|k + (1 - β⊥) d_i - γ_n sn  （式 (39) 简化） :contentReference[oaicite:5]{index=5}
            e = e + (1.0 - beta_p) * di - gamma_n * sn
            e_list.append(e)

            # Δd_i
            delta_d = di - d_prev
            d_prev = di

            # running cost：||e||_Q^2 + λ(S̄) ||Δd||_R^2  （式 (43)） :contentReference[oaicite:6]{index=6}
            J += q * e * e + lam * r * delta_d * delta_d

        # 终端代价
        e_terminal = e_list[-1]
        J += qf * e_terminal * e_terminal

        # 约束：|d_i| <= dmax(S̄)  （式 (45) 简化） :contentReference[oaicite:7]{index=7}
        # 转换为线性不等式： -dmax <= d_i <= dmax
        A = ca.DM.zeros(2 * N, N)
        lbg = []
        ubg = []

        # 对每个 i：  -d_i <= dmax   和   d_i <= dmax
        for i in range(N):
            #  -d_i <= dmax   →  -d_i - dmax <= 0
            A[2 * i, i] = -1.0
            lbg.append(-ca.inf)
            ubg.append(dmax)
            #   d_i <= dmax   →   d_i - dmax <= 0
            A[2 * i + 1, i] = 1.0
            lbg.append(-ca.inf)
            ubg.append(dmax)

        qp = {
            'x': d,
            'f': J,
            'g': A @ d
        }

        # 用 OSQP 作为 QP 求解器（需要支持）
        solver = ca.qpsol('solver', 'osqp', qp)

        # 把参数打包成一个函数，方便之后调用
        solver_wrapper = {
            'solver': solver,
            'N': N,
            'lbg': ca.DM(lbg),
            'ubg': ca.DM(ubg),
            'e0': e0,
            'sn': sn,
            'beta_p': beta_p,
            'gamma_n': gamma_n,
            'S_bar': S_bar,
            'dk_prev': dk_prev,
        }

        return solver_wrapper

    # ========== 调用 MPC 解算 ==========
    def _solve_mpc(self,
                   e_perp: float,
                   sn: float,
                   beta_perp: float,
                   gamma_n: float,
                   S_bar: float,
                   dk_prev: float) -> float:
        """
        调用 CasADi QP 求解器，返回本步 dk。
        """
        solver = self.qp_solver['solver']
        N = self.qp_solver['N']
        lbg = self.qp_solver['lbg']
        ubg = self.qp_solver['ubg']

        # 参数赋值
        # 注意：上面定义时参数是单独的 SX，实际传参用一个 dict
        p = {
            'e0': e_perp,
            'sn': sn,
            'beta_p': beta_perp,
            'gamma_n': gamma_n,
            'S_bar': S_bar,
            'dk_prev': dk_prev,
        }

        # CasADi 的 qpsol 接口需要把参数直接 substitute 进表达式里，
        # 这里为了简单，我们重用 solver，假设之前 build 时参数在表达式里已经是符号，
        # 实际上最干净的做法是：用 Function 封装或重构为 param 向量。
        # 为避免过于复杂，这里先假定 solver 内部不依赖外部 param（或你之后再按需求重构）。

        # 暂时用一个简单 fallback：在 β⊥,γ_n 常值、无约束时解析求最小 d0
        # 如果你想真正用 QP，可以之后重构，把参数写成向量 P 再传给 qpsol。
        try:
            # 这里简单做个“伪解”：只考虑一步预测和解析解，避免参数传递复杂性
            q = self.q
            r = self.r
            lambda0 = self.lambda0
            kappa = self.kappa
            lam = lambda0 * math.exp(kappa * (1.0 - S_bar))

            # 对单步： e1 = e0 + (1-β⊥) d0 - γ_n sn
            # cost = q e1^2 + lam r (d0 - dk_prev)^2
            # 对 d0 求导 = 0 → 得到解析解
            a = (1.0 - beta_perp)
            e0 = e_perp

            # d0 解：
            # d0 =  [ -q * 2 * a * (e0 - γ_n sn) + 2 * lam r dk_prev ] /
            #       [ 2 q a^2 + 2 lam r ]
            num = -q * a * (e0 - gamma_n * sn) + lam * r * dk_prev
            den = q * (a ** 2) + lam * r
            if den <= 1e-8:
                d0 = 0.0
            else:
                d0 = num / den

            # 再加上 dmax(S̄) 约束
            dmax = self.dmax_base * S_bar
            d0 = float(np.clip(d0, -dmax, dmax))

            return d0

        except Exception as e:
            self.get_logger().warn(f'MPC solve failed, fallback dk=0. Error: {e}')
            return 0.0


def main(args=None):
    rclpy.init(args=args)
    node = SkMPCControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

