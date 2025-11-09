#!/usr/bin/env python3

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


@dataclass
class Waypoint:
    x: float
    y: float


class PathFollower(Node):
    def __init__(self) -> None:
        super().__init__('tb3_path_follower')

        self.declare_parameter('path_topic', 'planned_path')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('max_linear_speed', 0.3)
        self.declare_parameter('max_angular_speed', 1.2)
        self.declare_parameter('goal_tolerance', 0.4)
        self.declare_parameter('waypoint_tolerance', 0.35)
        self.declare_parameter('linear_gain', 0.8)
        self.declare_parameter('angular_gain', 2.0)
        self.declare_parameter('lookahead', 0)

        self._path_topic = str(self.get_parameter('path_topic').value or 'planned_path')
        self._odom_topic = str(self.get_parameter('odom_topic').value or '/odom')
        self._cmd_topic = str(self.get_parameter('cmd_vel_topic').value or '/cmd_vel')

        self._max_linear = max(0.05, float(self.get_parameter('max_linear_speed').value))
        self._max_angular = max(0.1, float(self.get_parameter('max_angular_speed').value))
        self._goal_tol = max(0.05, float(self.get_parameter('goal_tolerance').value))
        self._waypoint_tol = max(0.05, float(self.get_parameter('waypoint_tolerance').value))
        self._linear_gain = max(0.1, float(self.get_parameter('linear_gain').value))
        self._angular_gain = max(0.5, float(self.get_parameter('angular_gain').value))
        self._lookahead = max(0, int(self.get_parameter('lookahead').value))

        self._path: List[Waypoint] = []
        self._current_idx = 0
        self._goal_reached = False
        self._offset_x: Optional[float] = None
        self._offset_y: Optional[float] = None
        self._goal_odom: Optional[Waypoint] = None

        path_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(Path, self._path_topic, self._on_path, path_qos)
        self.create_subscription(Odometry, self._odom_topic, self._on_odom, 20)
        self._cmd_pub = self.create_publisher(Twist, self._cmd_topic, 10)
        self._status_pub = self.create_publisher(String, 'navigation_status', 10)
        self._pose_pub = self.create_publisher(PoseStamped, 'robot_pose', 10)
        self._status = ''

        self.get_logger().info(f'Path follower waiting for path on {self._path_topic}')
        self._publish_status('waiting_for_path')

    def _on_path(self, msg: Path) -> None:
        self._path = [Waypoint(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self._current_idx = 0
        self._goal_reached = False
        self._offset_x = None
        self._offset_y = None
        if self._path:
            self._goal_odom = None  # recomputed when odom arrives
        self.get_logger().info(f'Received path with {len(self._path)} waypoints')
        if self._path:
            self._publish_status('path_received')
        else:
            self._publish_status('waiting_for_path')

    def _on_odom(self, msg: Odometry) -> None:
        if not self._path or self._goal_reached:
            if self._goal_reached:
                self._publish_stop(status='goal_reached')
            return

        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        yaw = self._yaw_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w)

        if self._offset_x is None or self._offset_y is None:
            self._offset_x = self._path[0].x - position.x
            self._offset_y = self._path[0].y - position.y
            self.get_logger().info(f'Offset map->odom: ({self._offset_x:.3f}, {self._offset_y:.3f})')

        if self._goal_odom is None:
            self._goal_odom = Waypoint(
                self._path[-1].x - self._offset_x,
                self._path[-1].y - self._offset_y,
            )
            self._publish_status('navigating')

        goal_dx = self._goal_odom.x - position.x
        goal_dy = self._goal_odom.y - position.y
        dist_to_goal = math.hypot(goal_dx, goal_dy)
        if dist_to_goal < self._goal_tol:
            if not self._goal_reached:
                self.get_logger().info(
                    f'Goal reached (distance {dist_to_goal:.2f} < {self._goal_tol:.2f})'
                )
            self._goal_reached = True
            self._publish_stop(status='goal_reached')
            return

        while self._current_idx < len(self._path) - 1:
            target_map = self._path[self._current_idx]
            target_odom = Waypoint(target_map.x - self._offset_x, target_map.y - self._offset_y)
            if math.hypot(target_odom.x - position.x, target_odom.y - position.y) > self._waypoint_tol:
                break
            self._current_idx += 1

        target_index = min(self._current_idx + self._lookahead, len(self._path) - 1)
        target_map = self._path[target_index]
        target = Waypoint(target_map.x - self._offset_x, target_map.y - self._offset_y)

        dx = target.x - position.x
        dy = target.y - position.y
        distance = math.hypot(dx, dy)
        desired_yaw = math.atan2(dy, dx)
        heading_error = self._normalize_angle(desired_yaw - yaw)

        cmd = Twist()
        cmd.angular.z = max(-self._max_angular, min(self._max_angular, self._angular_gain * heading_error))
        if abs(heading_error) < math.radians(70):
            cmd.linear.x = min(self._max_linear, self._linear_gain * distance)
        else:
            cmd.linear.x = 0.0

        self._cmd_pub.publish(cmd)
        self._publish_status('navigating')
        self._publish_pose(msg)

    def _publish_stop(self, *, status: str = 'stopped') -> None:
        self._cmd_pub.publish(Twist())
        self._publish_status(status)

    def _publish_status(self, text: str) -> None:
        if text == self._status:
            return
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)
        self._status = text

    @staticmethod
    def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _publish_pose(self, odom: Odometry) -> None:
        if self._offset_x is None or self._offset_y is None:
            return
        pose = PoseStamped()
        pose.header = odom.header
        pose.header.frame_id = 'map'
        pose.pose = odom.pose.pose
        pose.pose.position.x += self._offset_x
        pose.pose.position.y += self._offset_y
        self._pose_pub.publish(pose)


def main() -> None:
    rclpy.init()
    node = PathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':  
    main()
