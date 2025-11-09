#!/usr/bin/env python3
"""RRT global planner that reads a YAML map and publishes a nav_msgs/Path."""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as NavPath
from numpy.lib.stride_tricks import sliding_window_view
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

try:
    import yaml  
except ImportError as exc: 
    raise RuntimeError("pyyaml is required for tb3_map_rrt") from exc


State = Tuple[float, float]


@dataclass
class MapInfo:
    grid: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float

    @property
    def width(self) -> int:
        return int(self.grid.shape[1])

    @property
    def height(self) -> int:
        return int(self.grid.shape[0])


@dataclass
class TreeNode:
    x: float
    y: float
    parent: Optional[int]


class RRTPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__('tb3_rrt_planner')

        self.declare_parameter('map_yaml', '')
        self.declare_parameter('map_pgm', '')
        self.declare_parameter('start_x', -2.76177)
        self.declare_parameter('start_y', -6.63469)
        self.declare_parameter('goal_x', -2.37407)
        self.declare_parameter('goal_y', 13.9167)
        self.declare_parameter('occupied_threshold', 200)
        self.declare_parameter('safety_radius', 0.35)
        self.declare_parameter('max_iterations', 9000)
        self.declare_parameter('step_size', 0.8)
        self.declare_parameter('goal_radius', 0.7)
        self.declare_parameter('goal_bias', 0.12)
        self.declare_parameter('smoothing_iterations', 120)
        self.declare_parameter('random_seed', 0)
        self.declare_parameter('path_frame', 'map')
        self.declare_parameter('path_topic', 'planned_path')
        self.declare_parameter('simplify_distance', 0.4)

        seed = int(self.get_parameter('random_seed').value)
        if seed != 0:
            random.seed(seed)
            np.random.seed(seed)

        self._path_frame = str(self.get_parameter('path_frame').value or 'map')
        self._path_topic = str(self.get_parameter('path_topic').value or 'planned_path')

        self._map_info = self._load_map()
        self._passable = self._build_passable_grid()
        free_pixels = np.argwhere(self._passable)
        if free_pixels.size == 0:
            raise RuntimeError('Map contains no free cells after inflation')
        self._free_pixels = free_pixels

        self._start = (
            float(self.get_parameter('start_x').value),
            float(self.get_parameter('start_y').value),
        )
        self._goal = (
            float(self.get_parameter('goal_x').value),
            float(self.get_parameter('goal_y').value),
        )

        self._assert_free('start', self._start)
        self._assert_free('goal', self._goal)

        self.get_logger().info(
            f'Planning RRT from ({self._start[0]:.2f}, {self._start[1]:.2f}) '
            f'to ({self._goal[0]:.2f}, {self._goal[1]:.2f})'
        )

        raw_path = self._plan_rrt()
        if not raw_path:
            self.get_logger().fatal('RRT failed to connect start and goal')
            raise RuntimeError('RRT planning failed')

        simplified = self._shortcut_path(raw_path)
        simplified = self._simplify_colinear(simplified)
        self._publish_path(simplified)
        self.get_logger().info(f'RRT path ready with {len(simplified)} waypoints')

    # ------------------------------------------------------------------
    # Map helpers
    # ------------------------------------------------------------------
    def _load_map(self) -> MapInfo:
        yaml_param = str(self.get_parameter('map_yaml').get_parameter_value().string_value or '')
        pgm_override = str(self.get_parameter('map_pgm').get_parameter_value().string_value or '')

        map_yaml = Path(yaml_param) if yaml_param else None
        if not map_yaml:
            raise RuntimeError('Parameter `map_yaml` must be provided')
        if not map_yaml.is_absolute():
            map_yaml = (Path.cwd() / map_yaml).resolve()
        if not map_yaml.exists():
            raise FileNotFoundError(f'Map YAML not found: {map_yaml}')

        with map_yaml.open('r') as stream:
            data = yaml.safe_load(stream) or {}

        resolution = float(data['resolution'])
        origin = data.get('origin', [0.0, 0.0, 0.0])
        origin_x = float(origin[0])
        origin_y = float(origin[1])

        if pgm_override:
            pgm_path = Path(pgm_override)
            if not pgm_path.is_absolute():
                pgm_path = (map_yaml.parent / pgm_path).resolve()
        else:
            image_rel = data.get('image')
            if not image_rel:
                raise KeyError('`image` key missing in YAML')
            pgm_path = (map_yaml.parent / image_rel).resolve()

        if not pgm_path.exists():
            raise FileNotFoundError(f'Map image not found: {pgm_path}')

        grid = self._read_pgm(pgm_path)
        return MapInfo(grid=grid, resolution=resolution, origin_x=origin_x, origin_y=origin_y)

    @staticmethod
    def _read_pgm(path: Path) -> np.ndarray:
        with path.open('rb') as f:
            header = f.readline().strip()

            def next_line() -> bytes:
                line = f.readline()
                while line.startswith(b'#'):
                    line = f.readline()
                if not line:
                    raise ValueError('Unexpected EOF in PGM header')
                return line

            dims = next_line()
            width, height = map(int, dims.split())
            maxval = int(next_line())
            dtype = np.uint8 if maxval < 256 else np.uint16

            if header == b'P5':
                raw = np.frombuffer(f.read(), dtype=dtype)
            elif header == b'P2':
                raw = np.loadtxt(f, dtype=dtype)
            else:
                raise ValueError(f'Unsupported PGM header {header!r}')

        img = raw.reshape((height, width))
        if dtype != np.uint8:
            img = (img / img.max() * 255).astype(np.uint8)
        return img

    def _build_passable_grid(self) -> np.ndarray:
        threshold = int(self.get_parameter('occupied_threshold').value)
        passable = self._map_info.grid >= threshold
        return self._inflate_passable(passable)

    def _inflate_passable(self, passable: np.ndarray) -> np.ndarray:
        radius = float(self.get_parameter('safety_radius').value)
        if radius <= 0.0:
            return passable
        cells = int(math.ceil(radius / self._map_info.resolution))
        if cells <= 0:
            return passable
        occupied = ~passable
        padded = np.pad(occupied, cells, mode='constant', constant_values=False)
        window = sliding_window_view(padded, (2 * cells + 1, 2 * cells + 1))
        dilated = window.any(axis=(-1, -2))
        return ~dilated

    def _world_to_pixel(self, point: State) -> Tuple[int, int]:
        x, y = point
        col = int(round((x - self._map_info.origin_x) / self._map_info.resolution))
        row = int(round((y - self._map_info.origin_y) / self._map_info.resolution))
        return row, col

    def _pixel_to_world(self, row: int, col: int) -> State:
        x = self._map_info.origin_x + col * self._map_info.resolution
        y = self._map_info.origin_y + row * self._map_info.resolution
        return x, y

    def _assert_free(self, name: str, point: State) -> None:
        row, col = self._world_to_pixel(point)
        if row < 0 or row >= self._map_info.height or col < 0 or col >= self._map_info.width:
            raise RuntimeError(f'{name} {point} outside map bounds')
        if not self._passable[row, col]:
            raise RuntimeError(f'{name} {point} lies inside an obstacle (inflated)')

    # ------------------------------------------------------------------
    # RRT core
    # ------------------------------------------------------------------
    def _plan_rrt(self) -> List[State]:
        max_iters = int(self.get_parameter('max_iterations').value)
        step = float(self.get_parameter('step_size').value)
        goal_radius = float(self.get_parameter('goal_radius').value)
        goal_bias = float(self.get_parameter('goal_bias').value)

        nodes: List[TreeNode] = [TreeNode(self._start[0], self._start[1], parent=None)]
        goal_idx: Optional[int] = None

        for itr in range(max_iters):
            sample = self._sample_state(goal_bias)
            nearest_idx = self._nearest_neighbor(sample, nodes)
            new_point = self._steer(nodes[nearest_idx], sample, step)
            if not self._segment_free((nodes[nearest_idx].x, nodes[nearest_idx].y), new_point):
                continue
            nodes.append(TreeNode(new_point[0], new_point[1], parent=nearest_idx))
            new_idx = len(nodes) - 1

            if self._distance(new_point, self._goal) <= goal_radius and self._segment_free(new_point, self._goal):
                nodes.append(TreeNode(self._goal[0], self._goal[1], parent=new_idx))
                goal_idx = len(nodes) - 1
                self.get_logger().info(f'Goal reached after {itr + 1} iterations with {len(nodes)} nodes')
                break
        else:
            self.get_logger().warn('RRT iteration limit reached, connecting best node to goal')
            dists = [self._distance((node.x, node.y), self._goal) for node in nodes]
            best_idx = int(np.argmin(dists))
            if self._segment_free((nodes[best_idx].x, nodes[best_idx].y), self._goal):
                nodes.append(TreeNode(self._goal[0], self._goal[1], parent=best_idx))
                goal_idx = len(nodes) - 1

        if goal_idx is None:
            return []

        path: List[State] = []
        idx = goal_idx
        while idx is not None:
            node = nodes[idx]
            path.append((node.x, node.y))
            idx = node.parent
        path.reverse()
        return path

    def _sample_state(self, goal_bias: float) -> State:
        if random.random() < goal_bias:
            return self._goal
        row, col = self._free_pixels[np.random.randint(len(self._free_pixels))]
        jitter_x = (random.random() - 0.5) * self._map_info.resolution
        jitter_y = (random.random() - 0.5) * self._map_info.resolution
        x, y = self._pixel_to_world(int(row), int(col))
        return x + jitter_x, y + jitter_y

    def _nearest_neighbor(self, state: State, tree: Sequence[TreeNode]) -> int:
        distances = [self._distance((node.x, node.y), state) for node in tree]
        return int(np.argmin(distances))

    @staticmethod
    def _steer(node: TreeNode, target: State, step: float) -> State:
        dx = target[0] - node.x
        dy = target[1] - node.y
        dist = math.hypot(dx, dy)
        if dist <= step:
            return target
        scale = step / dist
        return node.x + dx * scale, node.y + dy * scale

    def _segment_free(self, a: State, b: State) -> bool:
        r0, c0 = self._world_to_pixel(a)
        r1, c1 = self._world_to_pixel(b)
        for r, c in self._bresenham(r0, c0, r1, c1):
            if r < 0 or c < 0 or r >= self._map_info.height or c >= self._map_info.width:
                return False
            if not self._passable[r, c]:
                return False
        return True

    @staticmethod
    def _distance(a: State, b: State) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    @staticmethod
    def _bresenham(r0: int, c0: int, r1: int, c1: int) -> Iterable[Tuple[int, int]]:
        dr = abs(r1 - r0)
        dc = abs(c1 - c0)
        sr = 1 if r1 >= r0 else -1
        sc = 1 if c1 >= c0 else -1
        err = dr - dc
        r, c = r0, c0
        while True:
            yield r, c
            if r == r1 and c == c1:
                break
            e2 = 2 * err
            if e2 > -dc:
                err -= dc
                r += sr
            if e2 < dr:
                err += dr
                c += sc

    # ------------------------------------------------------------------
    # Path post-processing
    # ------------------------------------------------------------------
    def _shortcut_path(self, path: List[State]) -> List[State]:
        iterations = int(self.get_parameter('smoothing_iterations').value)
        if len(path) <= 2:
            return path
        points = path[:]
        for _ in range(iterations):
            if len(points) <= 2:
                break
            i = random.randint(0, len(points) - 2)
            j = random.randint(i + 1, len(points) - 1)
            if j - i <= 1:
                continue
            if self._segment_free(points[i], points[j]):
                points = points[: i + 1] + points[j:]
        return points

    def _simplify_colinear(self, points: List[State]) -> List[State]:
        min_dist = float(self.get_parameter('simplify_distance').value)
        if len(points) <= 2 or min_dist <= 0.0:
            return points
        simplified = [points[0]]
        for idx in range(1, len(points) - 1):
            prev_pt = simplified[-1]
            curr = points[idx]
            nxt = points[idx + 1]
            if (
                self._are_colinear(prev_pt, curr, nxt)
                and self._distance(prev_pt, curr) < min_dist
                and self._segment_free(prev_pt, nxt)
            ):
                continue
            simplified.append(curr)
        if simplified[-1] != points[-1]:
            simplified.append(points[-1])
        return simplified

    @staticmethod
    def _are_colinear(a: State, b: State, c: State) -> bool:
        abx, aby = b[0] - a[0], b[1] - a[1]
        bcx, bcy = c[0] - b[0], c[1] - b[1]
        dot = abx * bcx + aby * bcy
        mag_ab = math.hypot(abx, aby)
        mag_bc = math.hypot(bcx, bcy)
        if mag_ab < 1e-6 or mag_bc < 1e-6:
            return False
        cos_angle = dot / (mag_ab * mag_bc)
        cos_angle = max(-1.0, min(1.0, cos_angle))
        angle = math.acos(cos_angle)
        return abs(angle) < math.radians(8.0)

    # ------------------------------------------------------------------
    # ROS publishing
    # ------------------------------------------------------------------
    def _publish_path(self, points: Sequence[State]) -> None:
        path_msg = NavPath()
        path_msg.header.frame_id = self._path_frame
        now = self.get_clock().now().to_msg()
        path_msg.header.stamp = now

        for x, y in points:
            pose = PoseStamped()
            pose.header.frame_id = self._path_frame
            pose.header.stamp = now
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        pub = self.create_publisher(NavPath, self._path_topic, qos)
        pub.publish(path_msg)
        self._path_pub = pub  


def main() -> None:
    rclpy.init()
    node = RRTPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':  
    main()
