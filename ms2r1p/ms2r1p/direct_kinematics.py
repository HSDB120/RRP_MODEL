#!/usr/bin/env python3
import math
from typing import Tuple, List

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PointStamped, Twist


def dh_homogeneous(a: float, alpha: float, d: float, theta: float) -> np.ndarray:
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    # Keep layout compatible with the original implementation
    T = np.array([
        [ct, -st, 0.0, a],
        [st * ca, ct * ca, -sa, -d * sa],
        [st * sa, ct * sa,  ca,  d * ca],
        [0.0, 0.0, 0.0, 1.0],
    ], dtype=float)
    return T


def chain_product(transforms: List[np.ndarray]) -> np.ndarray:
    """Left-to-right homogeneous chain product."""
    out = np.eye(4, dtype=float)
    for Ti in transforms:
        out = out @ Ti
    return out


class FKNodeCustom(Node):
    def __init__(self) -> None:
        super().__init__('fk_solver_cx')

        # --- Publishers/Subscribers ---
        self._pub_ee = self.create_publisher(PointStamped, 'end_effector_pose', 10)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # Keep the same subscribed topic (leading slash preserved)
        self._sub_cmd = self.create_subscription(Twist, '/scara_conf', self._on_cmd, qos)

        # --- Parameters (declared to keep dynamic-tweak capability) ---
        self.declare_parameter('scale', 100.0)
        self.declare_parameter('a_12', 160.0 / 100.0)
        self.declare_parameter('a_23', 120.0 / 100.0)
        self.declare_parameter('alpha_23', math.pi)
        self.declare_parameter('s_1', 82.0 / 100.0)
        self.declare_parameter('s_3', 18.0 / 100.0)

        # Internal joint state (θ1, θ2, s4)
        self._q1 = 0.0
        self._q2 = 0.0
        self._s4 = 0.0

        # Timer for periodic FK
        self._timer = self.create_timer(0.10, self._on_tick)

        # Light log pacing
        self._log_every = 10  # log 1/second if timer is 10 Hz
        self._tick = 0

    # ---------- Callbacks ----------

    def _on_cmd(self, msg: Twist) -> None:
        self._q1 = float(msg.linear.x)
        self._q2 = float(msg.linear.y)
        self._s4 = float(msg.linear.z)

    def _on_tick(self) -> None:
        # Fetch params fresh (allows live tuning without restart)
        a12, a23, a23_alpha, s1, s3 = self._get_geo_params()

        # Build DH rows [a, alpha, d, theta]
        dh_rows = [
            (0.0, 0.0, s1, self._q1),
            (a12, 0.0, 0.0, self._q2),
            (a23, a23_alpha, s3, 0.0),
            (0.0, 0.0, self._s4, 0.0),
        ]

        transforms = [dh_homogeneous(*row) for row in dh_rows]
        T = chain_product(transforms)

        x, y, z = float(T[0, 3]), float(T[1, 3]), float(T[2, 3])

        if (self._tick % self._log_every) == 0:
            self.get_logger().info(f'EE (x,y,z)=({x:.3f}, {y:.3f}, {z:.3f})')

        # Publish point for visualization
        pt = PointStamped()
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.header.frame_id = 'base_link'
        pt.point.x, pt.point.y, pt.point.z = x, y, z
        self._pub_ee.publish(pt)

        self._tick += 1

    # ---------- Helpers ----------

    def _get_geo_params(self) -> Tuple[float, float, float, float, float]:
        # Read parameters and ensure they are floats
        scale = float(self.get_parameter('scale').value)
        # The stored defaults already consider scale; keep the same behavior
        a12 = float(self.get_parameter('a_12').value)
        a23 = float(self.get_parameter('a_23').value)
        alpha23 = float(self.get_parameter('alpha_23').value)
        s1 = float(self.get_parameter('s_1').value)
        s3 = float(self.get_parameter('s_3').value)
        # Basic sanity (non-fatal)
        if scale <= 0.0:
            self.get_logger().warn('Parameter "scale" <= 0, using 1.0 for safety.')
            scale = 1.0
        return a12, a23, alpha23, s1, s3


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FKNodeCustom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
