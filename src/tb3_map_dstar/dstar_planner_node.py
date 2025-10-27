#!/usr/bin/env python3

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as NavPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from numpy.lib.stride_tricks import sliding_window_view

try:
    import yaml  # type: ignore
except ImportError as exc:  # pragma: no cover
    raise RuntimeError("pyyaml is required for tb3_map_dstar") from exc


State = Tuple[int, int]


@dataclass
class MapInfo:
    grid: np.ndarray  # uint8 occupancy, 0 occupied, 255 free
    resolution: float
    origin_x: float
    origin_y: float

    @property
    def width(self) -> int:
        return int(self.grid.shape[1])

    @property
    def height(self) -> int:
        return int(self.grid.shape[0])


class DStarLite:

    def __init__(
        self,
        passable: np.ndarray,
        start: State,
        goal: State,
        allow_diagonal: bool = True,
    ) -> None:
        self.passable = passable
        self.start = start
        self.goal = goal
        self.allow_diagonal = allow_diagonal
        self.height, self.width = passable.shape

        self.g = np.full(passable.shape, np.inf, dtype=np.float64)
        self.rhs = np.full(passable.shape, np.inf, dtype=np.float64)
        self.rhs[goal] = 0.0

        self.k_m = 0.0
        self._open: List[Tuple[float, float, int, int]] = []
        self._open_lookup: Dict[State, Tuple[float, float]] = {}
        self._push(goal)

    # Priority queue helpers
    def _calculate_key(self, state: State) -> Tuple[float, float]:
        g_val = self.g[state]
        rhs_val = self.rhs[state]
        min_val = min(g_val, rhs_val)
        h = self._heuristic(self.start, state)
        return (min_val + h + self.k_m, min_val)

    def _push(self, state: State) -> None:
        key = self._calculate_key(state)
        heapq.heappush(self._open, (key[0], key[1], state[0], state[1]))
        self._open_lookup[state] = key

    def _pop(self) -> Optional[Tuple[Tuple[float, float], State]]:
        while self._open:
            k1, k2, r, c = heapq.heappop(self._open)
            state = (r, c)
            if self._open_lookup.get(state) == (k1, k2):
                del self._open_lookup[state]
                return (k1, k2), state
        return None

    def _top_key(self) -> Tuple[float, float]:
        while self._open:
            k1, k2, r, c = self._open[0]
            state = (r, c)
            if self._open_lookup.get(state) == (k1, k2):
                return k1, k2
            heapq.heappop(self._open)
        return (math.inf, math.inf)

    # Core D* Lite operations
    def compute_shortest_path(self) -> None:
        while True:
            top_key = self._top_key()
            start_key = self._calculate_key(self.start)
            if (top_key >= start_key) and self._is_consistent(self.start):
                break
            popped = self._pop()
            if popped is None:
                break
            _, state = popped
            if self.g[state] > self.rhs[state]:
                self.g[state] = self.rhs[state]
                for pred in self._predecessors(state):
                    self._update_vertex(pred)
            else:
                self.g[state] = math.inf
                self._update_vertex(state)
                for pred in self._predecessors(state):
                    self._update_vertex(pred)

    def extract_path(self) -> List[State]:
        if not self.passable[self.start]:
            raise RuntimeError('Start is in an occupied cell')
        if not self.passable[self.goal]:
            raise RuntimeError('Goal is in an occupied cell')

        path: List[State] = [self.start]
        current = self.start
        visited = set([current])
        while current != self.goal:
            best_state = None
            best_cost = math.inf
            for succ in self._successors(current):
                cost = self._cost(current, succ)
                if math.isinf(cost):
                    continue
                total = cost + self.g[succ]
                if total < best_cost:
                    best_cost = total
                    best_state = succ
            if best_state is None or math.isinf(best_cost):
                raise RuntimeError('Failed to extract valid path')
            if best_state in visited:
                raise RuntimeError('Detected loop while extracting path')
            visited.add(best_state)
            path.append(best_state)
            current = best_state
        return path

    def _update_vertex(self, state: State) -> None:
        if state != self.goal:
            min_rhs = math.inf
            for succ in self._successors(state):
                c = self._cost(state, succ)
                if math.isinf(c):
                    continue
                min_rhs = min(min_rhs, c + self.g[succ])
            self.rhs[state] = min_rhs

        if state in self._open_lookup:
            del self._open_lookup[state]

        if not self._is_consistent(state):
            self._push(state)

    def _is_consistent(self, state: State) -> bool:
        return math.isclose(self.g[state], self.rhs[state]) or (
            math.isinf(self.g[state]) and math.isinf(self.rhs[state])
        )

    def _heuristic(self, a: State, b: State) -> float:
        dr = a[0] - b[0]
        dc = a[1] - b[1]
        return math.hypot(dr, dc)

    def _successors(self, state: State) -> Iterable[State]:
        r, c = state
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                if not self.allow_diagonal and abs(dr) + abs(dc) > 1:
                    continue
                nr = r + dr
                nc = c + dc
                if 0 <= nr < self.height and 0 <= nc < self.width:
                    if self.passable[nr, nc]:
                        yield (nr, nc)

    def _predecessors(self, state: State) -> Iterable[State]:
        return self._successors(state)

    def _cost(self, a: State, b: State) -> float:
        if not self.passable[b]:
            return math.inf
        dr = abs(a[0] - b[0])
        dc = abs(a[1] - b[1])
        if dr == 1 and dc == 1:
            return math.sqrt(2.0)
        if dr + dc == 1:
            return 1.0
        return math.hypot(dr, dc)


class DStarPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__('tb3_dstar_planner')

        self._passable_grid: Optional[np.ndarray] = None

        self.declare_parameter('map_yaml', '')
        self.declare_parameter('map_pgm', '')
        self.declare_parameter('start_x', -2.76177)
        self.declare_parameter('start_y', -6.63469)
        self.declare_parameter('goal_x', -2.37407)
        self.declare_parameter('goal_y', 13.9167)
        self.declare_parameter('occupied_threshold', 200)
        self.declare_parameter('allow_diagonal', True)
        self.declare_parameter('path_frame', 'map')
        self.declare_parameter('safety_radius', 0.35)
        self.declare_parameter('simplify_distance', 0.35)

        self._map_info = self._load_map()
        self._publish_planned_path()

    # Map handling
    def _load_map(self) -> MapInfo:
        yaml_param = str(self.get_parameter('map_yaml').get_parameter_value().string_value or '')
        pgm_override = str(self.get_parameter('map_pgm').get_parameter_value().string_value or '')

        map_yaml_path = Path(yaml_param).expanduser()
        if yaml_param and not map_yaml_path.is_absolute():
            map_yaml_path = (Path.cwd() / map_yaml_path).resolve()

        if yaml_param and map_yaml_path.exists():
            yaml_to_read = map_yaml_path
        else:
            raise FileNotFoundError(f'Map YAML not found: {map_yaml_path}')

        with yaml_to_read.open('r') as stream:
            yaml_data = yaml.safe_load(stream) or {}

        resolution = float(yaml_data['resolution'])
        origin = yaml_data.get('origin', [0.0, 0.0, 0.0])
        origin_x = float(origin[0])
        origin_y = float(origin[1])

        if pgm_override:
            pgm_path = Path(pgm_override)
            if not pgm_path.is_absolute():
                pgm_path = (yaml_to_read.parent / pgm_path).resolve()
        else:
            image_rel = yaml_data.get('image')
            if not image_rel:
                raise KeyError('`image` key not found in map YAML')
            pgm_path = (yaml_to_read.parent / image_rel).resolve()

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

    # Coordinate helpers
    def _world_to_pixel(self, x: float, y: float) -> State:
        col = int(round((x - self._map_info.origin_x) / self._map_info.resolution))
        row = int(round((y - self._map_info.origin_y) / self._map_info.resolution))
        return (row, col)

    def _pixel_to_world(self, row: int, col: int) -> Tuple[float, float]:
        x = self._map_info.origin_x + col * self._map_info.resolution
        y = self._map_info.origin_y + row * self._map_info.resolution
        return x, y

    # Planning
    def _publish_planned_path(self) -> None:
        start = (
            float(self.get_parameter('start_x').value),
            float(self.get_parameter('start_y').value),
        )
        goal = (
            float(self.get_parameter('goal_x').value),
            float(self.get_parameter('goal_y').value),
        )

        start_rc = self._world_to_pixel(*start)
        goal_rc = self._world_to_pixel(*goal)
        free_threshold = int(self.get_parameter('occupied_threshold').value)
        passable = self._map_info.grid >= free_threshold
        passable = self._inflate_free_space(passable)
        self._passable_grid = passable

        for name, state in [('start', start_rc), ('goal', goal_rc)]:
            r, c = state
            if r < 0 or r >= self._map_info.height or c < 0 or c >= self._map_info.width:
                raise RuntimeError(f'{name} cell {state} out of bounds')
            if not passable[r, c]:
                raise RuntimeError(f'{name} cell {state} lies in an occupied pixel')

        planner = DStarLite(
            passable=passable,
            start=start_rc,
            goal=goal_rc,
            allow_diagonal=bool(self.get_parameter('allow_diagonal').value),
        )
        planner.compute_shortest_path()

        try:
            path_grid = planner.extract_path()
        except RuntimeError as exc:
            self.get_logger().fatal(f'D* Lite failed: {exc}')
            raise

        path_points = [self._pixel_to_world(r, c) for r, c in path_grid]
        simplify_distance = max(0.0, float(self.get_parameter('simplify_distance').value))
        path_points = self._simplify_path(path_points, simplify_distance)

        frame_id = str(self.get_parameter('path_frame').value or 'map')
        path_msg = NavPath()
        path_msg.header.frame_id = frame_id
        now = self.get_clock().now().to_msg()
        path_msg.header.stamp = now
        max_idx = len(path_points) - 1

        for idx, (x, y) in enumerate(path_points):
            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.header.stamp = now
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
            if idx in (0, max_idx):
                self.get_logger().info(
                    f'Path {"start" if idx == 0 else "goal"} at ({x:.2f}, {y:.2f})'
                )

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        publisher = self.create_publisher(NavPath, 'planned_path', qos)
        publisher.publish(path_msg)
        self.get_logger().info(f'D* Lite path published with {len(path_msg.poses)} waypoints')
        self._path_pub = publisher  

    def _inflate_free_space(self, passable: np.ndarray) -> np.ndarray:
        radius_m = float(self.get_parameter('safety_radius').value)
        if radius_m <= 0.0:
            return passable
        cells = int(math.ceil(radius_m / self._map_info.resolution))
        if cells <= 0:
            return passable
        occupied = ~passable
        padded = np.pad(occupied, cells, mode='constant', constant_values=False)
        window = sliding_window_view(padded, (2 * cells + 1, 2 * cells + 1))
        dilated = window.any(axis=(-1, -2))
        return ~dilated

    def _simplify_path(self, points: Sequence[Tuple[float, float]], min_distance: float) -> List[Tuple[float, float]]:
        if not points:
            return []
        if min_distance <= 0.0 or len(points) <= 2:
            return list(points)

        simplified = [points[0]]
        for idx in range(1, len(points) - 1):
            prev_pt = simplified[-1]
            curr_pt = points[idx]
            next_pt = points[idx + 1]

            v_prev = (curr_pt[0] - prev_pt[0], curr_pt[1] - prev_pt[1])
            v_next = (next_pt[0] - curr_pt[0], next_pt[1] - curr_pt[1])
            prev_len = math.hypot(*v_prev)
            next_len = math.hypot(*v_next)

            colinear = False
            if prev_len > 1e-6 and next_len > 1e-6:
                dot = v_prev[0] * v_next[0] + v_prev[1] * v_next[1]
                cos_angle = dot / (prev_len * next_len)
                cos_angle = max(-1.0, min(1.0, cos_angle))
                angle = math.acos(cos_angle)
                colinear = abs(angle) < math.radians(8.0)

            if colinear and prev_len < min_distance and self._segment_is_free(prev_pt, next_pt):
                continue

            simplified.append(curr_pt)

        if simplified[-1] != points[-1]:
            simplified.append(points[-1])
        return simplified

    def _segment_is_free(self, p0: Tuple[float, float], p1: Tuple[float, float]) -> bool:
        if self._passable_grid is None:
            return True
        r0, c0 = self._world_to_pixel(*p0)
        r1, c1 = self._world_to_pixel(*p1)
        for r, c in self._bresenham(r0, c0, r1, c1):
            if r < 0 or c < 0 or r >= self._map_info.height or c >= self._map_info.width:
                return False
            if not self._passable_grid[r, c]:
                return False
        return True

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


def main() -> None:
    rclpy.init()
    node = DStarPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':  
    main()
