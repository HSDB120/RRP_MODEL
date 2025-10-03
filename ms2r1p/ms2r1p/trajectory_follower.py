#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PointStamped


class TrajectoryStepperCx(Node):
    def __init__(self) -> None:
        super().__init__('trajectory_follower_cx')

        # Parameters for pacing and looping
        self.declare_parameter('publish_period', 0.05)  # seconds between points
        self.declare_parameter('autoloop', False)      # restart when finished

        self._dt = float(self.get_parameter('publish_period').value)
        self._loop = bool(self.get_parameter('autoloop').value)

        # Pub/Sub
        self._pub_goal = self.create_publisher(PointStamped, 'goal_pose_trajectory', 10)
        self._sub_path = self.create_subscription(Path, 'planned_trajectory', self._on_path, 10)

        # Internal state
        self._poses = []
        self._idx = 0
        self._timer = None
        self._running = False

        self.get_logger().info(f'Follower ready: period={self._dt:.3f}s, loop={self._loop}')

    # --------- Callbacks ---------

    def _on_path(self, msg: Path) -> None:
        self._poses = list(msg.poses) if msg.poses else []
        self._idx = 0

        if not self._poses:
            self.get_logger().warn('Received empty trajectory')
            self._stop_timer()
            self._running = False
            return

        if not self._running:
            self._start_timer()
            self._running = True
        else:
            # Already running; continue with updated buffer
            self.get_logger().info(f'Updated trajectory: {len(self._poses)} poses')

    def _tick(self) -> None:
        if self._idx >= len(self._poses):
            self.get_logger().info('Finished trajectory')
            if self._loop and self._poses:
                self._idx = 0
                self.get_logger().info('Restarting (autoloop)')
            else:
                self._stop_timer()
                self._running = False
                return

        pose = self._poses[self._idx]

        # PoseStamped → PointStamped
        msg = PointStamped()
        msg.header = pose.header
        msg.point.x = pose.pose.position.x
        msg.point.y = pose.pose.position.y
        msg.point.z = pose.pose.position.z

        self._pub_goal.publish(msg)
        self._idx += 1

    # --------- Timer helpers ---------

    def _start_timer(self) -> None:
        self._stop_timer()
        self._timer = self.create_timer(self._dt, self._tick)
        self.get_logger().info(f'Starting follower: {len(self._poses)} poses')

    def _stop_timer(self) -> None:
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None


def main() -> None:
    rclpy.init()
    node = TrajectoryStepperCx()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
