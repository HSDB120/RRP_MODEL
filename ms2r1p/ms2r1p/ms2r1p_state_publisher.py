#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class StateRelayCx(Node):
    def __init__(self) -> None:
        super().__init__('ms2r1p_state_publisher_cx')

        # ---- Parameters (names kept for drop-in compatibility) ----
        self.declare_parameter('arm_1', 'base_to_arm1')
        self.declare_parameter('arm_2', 'arm_1_arm_2_joint')
        self.declare_parameter('prismatic', 'arm_2_prismatic_joint')

        self.declare_parameter('upper_limit_angular_1', math.pi/2)
        self.declare_parameter('lower_limit_angular_1', -math.pi/2)

        self.declare_parameter('upper_limit_angular_2', 2*math.pi/3)
        self.declare_parameter('lower_limit_angular_2', -2*math.pi/3)

        self.declare_parameter('upper_limit_linear', 0.0)
        self.declare_parameter('lower_limit_linear', -6.085/10)

        self.declare_parameter('publish_rate', 10.0)  # Hz

        # Resolve parameters
        self._name_j1 = str(self.get_parameter('arm_1').value)
        self._name_j2 = str(self.get_parameter('arm_2').value)
        self._name_j3 = str(self.get_parameter('prismatic').value)

        self._lim_th1 = (
            float(self.get_parameter('lower_limit_angular_1').value),
            float(self.get_parameter('upper_limit_angular_1').value),
        )
        self._lim_th2 = (
            float(self.get_parameter('lower_limit_angular_2').value),
            float(self.get_parameter('upper_limit_angular_2').value),
        )
        self._lim_lin = (
            float(self.get_parameter('lower_limit_linear').value),
            float(self.get_parameter('upper_limit_linear').value),
        )

        self._rate_hz = float(self.get_parameter('publish_rate').value)

        # ---- Commanded setpoints (updated via /scara_conf) ----
        self._cmd_th1 = 0.0
        self._cmd_th2 = 0.0
        self._cmd_z = 0.0

        # ---- Pub/Sub ----
        qos = QoSProfile(depth=10)
        self.create_subscription(Twist, 'scara_conf', self._on_cmd, qos)
        self._pub_js = self.create_publisher(JointState, '/joint_states', qos)

        # ---- Timer ----
        dt = 1.0 / max(1e-3, self._rate_hz)
        self._timer = self.create_timer(dt, self._tick)

        self.get_logger().info(
            f'Joints: {self._name_j1}, {self._name_j2}, {self._name_j3} | rate={self._rate_hz:.1f} Hz\n'
            f'Limits th1[{self._lim_th1[0]:.3f},{self._lim_th1[1]:.3f}] th2[{self._lim_th2[0]:.3f},{self._lim_th2[1]:.3f}] '
            f'lin[{self._lim_lin[0]:.3f},{self._lim_lin[1]:.3f}]'
        )

    # ---------- Callbacks ----------

    def _on_cmd(self, msg: Twist) -> None:
        self._cmd_th1 = float(msg.linear.x)
        self._cmd_th2 = float(msg.linear.y)
        self._cmd_z = float(msg.linear.z)

    def _tick(self) -> None:
        th1 = clamp(self._cmd_th1, *self._lim_th1)
        th2 = clamp(self._cmd_th2, *self._lim_th2)
        z = clamp(self._cmd_z,  *self._lim_lin)

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [self._name_j1, self._name_j2, self._name_j3]
        js.position = [th1, th2, z]
        js.velocity = [0.0, 0.0, 0.0]

        self._pub_js.publish(js)


def main() -> None:
    rclpy.init()
    node = StateRelayCx()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
