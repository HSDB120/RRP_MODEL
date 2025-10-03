#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Inverse kinematics (refactor) — same topics and math, different structure/naming.
#
# I/O preserved:
#   - Subscribes PointStamped on 'goal_pose_trajectory'
#   - Publishes Twist on 'scara_conf'
#
# Changes:
#   - Class/identifiers renamed, comments/docstrings rewritten.
#   - Parameter handling wrapped in helper; reachability check kept.
#   - Linear joint s_4 computed as in original (relative to 0.072 offset).
#   - Robust clipping for acos() domain.

import math
from typing import Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist


class IKNodeCustom(Node):
    def __init__(self) -> None:
        super().__init__('inverse_kinematics_cx')

        # --- Pub/Sub ---
        self._pub_twist = self.create_publisher(Twist, 'scara_conf', 10)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._sub_goal = self.create_subscription(
            PointStamped, 'goal_pose_trajectory', self._on_goal, qos
        )

        # --- Parameters (geometry) ---
        self.declare_parameter('scale', 100.0)
        self.declare_parameter('a_12', 160.0 / 100.0)
        self.declare_parameter('a_23', 120.0 / 100.0)
        self.declare_parameter('s_1', 82.0 / 100.0)
        self.declare_parameter('s_3', 18.0 / 100.0)

    # ---------- Helpers ----------

    def _get_geo(self) -> Tuple[float, float, float, float, float]:
        scale = float(self.get_parameter('scale').value)
        a12 = float(self.get_parameter('a_12').value)
        a23 = float(self.get_parameter('a_23').value)
        s1 = float(self.get_parameter('s_1').value)
        s3 = float(self.get_parameter('s_3').value)
        if scale <= 0.0:
            self.get_logger().warn('Parameter "scale" <= 0, forcing 1.0.')
            scale = 1.0
        return scale, a12, a23, s1, s3

    # ---------- Callback ----------

    def _on_goal(self, msg: PointStamped) -> None:
        x_d = float(msg.point.x)
        y_d = float(msg.point.y)
        z_d = float(msg.point.z)

        _, a12, a23, _s1, _s3 = self._get_geo()

        # Prismatic joint relative to 0.072 as in the original code
        s_4 = -(0.072 - z_d)

        # Workspace check
        r2 = x_d * x_d + y_d * y_d
        rmax = (a12 + a23) * (a12 + a23)
        if r2 > rmax:
            self.get_logger().warn(
                f'Goal outside reach: sqrt(r2)={math.sqrt(r2):.3f} > a12+a23={a12 + a23:.3f}'
            )
            return

        # theta2 from cosine law
        cos_t2 = (r2 - a12 * a12 - a23 * a23) / (2.0 * a12 * a23)
        cos_t2 = float(np.clip(cos_t2, -1.0, 1.0))
        theta_2 = math.acos(cos_t2)

        # Solve theta1 via linear system (as in original approach)
        k1 = a12 + a23 * math.cos(theta_2)
        k2 = a23 * math.sin(theta_2)

        # [cos(theta1)] = inv([[k1,-k2],[k2,k1]]) @ [x; y]
        det = (k1 * k1 + k2 * k2)
        if det == 0.0:
            self.get_logger().error('Singular configuration: k1^2 + k2^2 == 0.')
            return
        cos_t1 = ( k1 * x_d + k2 * y_d) / det
        sin_t1 = (-k2 * x_d + k1 * y_d) / det
        theta_1 = math.atan2(sin_t1, cos_t1)

        # Publish solution
        cmd = Twist()
        cmd.linear.x = theta_1
        cmd.linear.y = theta_2
        cmd.linear.z = s_4
        self._pub_twist.publish(cmd)

        self.get_logger().info(
            f'IK -> th1={theta_1:.3f}, th2={theta_2:.3f}, s4={s_4:.3f}'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IKNodeCustom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
