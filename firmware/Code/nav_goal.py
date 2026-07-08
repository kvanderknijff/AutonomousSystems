"""Local goal navigation with APF collision avoidance (MicroPython)."""

import json
import math
import time

GOAL_ACTION_SET = "set"
GOAL_ACTION_CLEAR = "clear"

DEFAULT_GOAL_TOLERANCE = 12.0
DEFAULT_HEADING_TOLERANCE = 12.0
DEFAULT_ROTATE_HEADING_THRESHOLD = 12.0
DEFAULT_POSITION_TIMEOUT = 20.0
DEFAULT_APF_INFLUENCE_RADIUS = 50.0
DEFAULT_APF_K_ATTRACT = 1.0
DEFAULT_APF_K_REPEL = 100.0
DEFAULT_MIN_SEPARATION = 28.0
DEFAULT_FORWARD_BLOCK_DISTANCE = 45.0
DEFAULT_FORWARD_BLOCK_ANGLE = 65.0
DEFAULT_NEIGHBOR_STALE_SEC = 2.0
DEFAULT_FIELD_MARGIN = 20.0
CORNER_ARUCO_FIRST = 5
CORNER_ARUCO_LAST = 8
TURN_COMMANDS = frozenset(("TL", "TR", "RL", "RR"))
# Pulse steering: brief turn, stop, wait for fresh camera frame (0 = continuous turns).
# DEFAULT_TURN_PULSE_SEC = 0.15
# DEFAULT_TURN_SETTLE_SEC = 0.35
DEFAULT_TURN_PULSE_SEC = 1.0
DEFAULT_TURN_SETTLE_SEC = 1.0


class FieldBounds:
    def __init__(self):
        self.corners = {}

    def update_corner(self, aruco_id, x, y):
        self.corners[int(aruco_id)] = (float(x), float(y))

    @property
    def is_ready(self):
        if len(self.corners) < 2:
            return False
        xs = {point[0] for point in self.corners.values()}
        ys = {point[1] for point in self.corners.values()}
        return len(xs) >= 2 and len(ys) >= 2

    def bounds(self):
        if not self.corners:
            return None
        xs = [point[0] for point in self.corners.values()]
        ys = [point[1] for point in self.corners.values()]
        return min(xs), min(ys), max(xs), max(ys)

    def center(self):
        rect = self.bounds()
        if rect is None:
            return None
        return (rect[0] + rect[2]) / 2, (rect[1] + rect[3]) / 2

    def clamp_point(self, x, y, margin):
        rect = self.bounds()
        if rect is None:
            return float(x), float(y)
        min_x, min_y, max_x, max_y = rect
        return (
            max(min_x + margin, min(max_x - margin, float(x))),
            max(min_y + margin, min(max_y - margin, float(y))),
        )

    def inside(self, x, y, margin=0.0):
        rect = self.bounds()
        if rect is None:
            return True
        min_x, min_y, max_x, max_y = rect
        return (
            min_x + margin <= x <= max_x - margin
            and min_y + margin <= y <= max_y - margin
        )

    def escape_heading(self, x, y, margin):
        if not self.is_ready or self.inside(x, y, margin):
            return None
        center = self.center()
        if center is None:
            return None
        return bearing_degrees(center[0] - x, center[1] - y)


def is_corner_aruco(aruco_id):
    return CORNER_ARUCO_FIRST <= int(aruco_id) <= CORNER_ARUCO_LAST


def normalize_angle(degrees):
    return (degrees + 180) % 360 - 180


def _hypot(x, y):
    # MicroPython's math module has no hypot().
    return math.sqrt(x * x + y * y)


# def bearing_degrees(dx, dy):
#     return math.degrees(math.atan2(dy, dx))

def bearing_degrees(dx, dy):
    return math.degrees(math.atan2(dx, -dy))


def steer_command_code(
    direct_error,
    heading_tolerance=DEFAULT_HEADING_TOLERANCE,
    rotate_threshold=DEFAULT_ROTATE_HEADING_THRESHOLD,
):
    abs_error = abs(direct_error)
    if abs_error < heading_tolerance:
        return "FW"
    if abs_error >= rotate_threshold:
        return "RL" if direct_error > 0 else "RR"
    if direct_error > 0:
        return "TL"
    return "TR"


def calculate_apf_heading(
    current_x,
    current_y,
    target_x,
    target_y,
    other_robots_positions,
    influence_radius=DEFAULT_APF_INFLUENCE_RADIUS,
    k_attract=DEFAULT_APF_K_ATTRACT,
    k_repel=DEFAULT_APF_K_REPEL,
):
    dx_target = target_x - current_x
    dy_target = target_y - current_y
    dist_target = _hypot(dx_target, dy_target)

    if dist_target == 0:
        return 0.0

    f_attr_x = k_attract * (dx_target / dist_target)
    f_attr_y = k_attract * (dy_target / dist_target)

    f_repel_x = 0.0
    f_repel_y = 0.0

    for ox, oy in other_robots_positions:
        dx_obstacle = current_x - ox
        dy_obstacle = current_y - oy
        dist_obstacle = _hypot(dx_obstacle, dy_obstacle)

        if dist_obstacle == 0 or dist_obstacle > influence_radius:
            continue

        force_magnitude = (
            k_repel * (1.0 / dist_obstacle - 1.0 / influence_radius) / (dist_obstacle ** 2)
        )
        f_repel_x += force_magnitude * (dx_obstacle / dist_obstacle)
        f_repel_y += force_magnitude * (dy_obstacle / dist_obstacle)

    f_total_x = f_attr_x + f_repel_x
    f_total_y = f_attr_y + f_repel_y

    if f_total_x == 0 and f_total_y == 0:
        return bearing_degrees(dx_target, dy_target)

    return math.degrees(math.atan2(f_total_y, f_total_x))


def parse_goal_payload(payload):
    text = payload.strip()
    if not text or not text.startswith("{"):
        return None
    try:
        data = json.loads(text)
    except (TypeError, ValueError):
        return None
    if not isinstance(data, dict):
        return None
    action = str(data.get("action", GOAL_ACTION_SET)).lower()
    if action == GOAL_ACTION_CLEAR:
        return {"action": GOAL_ACTION_CLEAR, "seq": int(data.get("seq", 0))}
    if data.get("target_x") is None or data.get("target_y") is None:
        return None
    return {
        "action": GOAL_ACTION_SET,
        "target_x": float(data["target_x"]),
        "target_y": float(data["target_y"]),
        "tolerance": float(data.get("tolerance", DEFAULT_GOAL_TOLERANCE)),
        "seq": int(data.get("seq", 0)),
    }


def parse_config_payload(payload):
    text = payload.strip()
    if not text.startswith("{"):
        return None
    try:
        data = json.loads(text)
    except (TypeError, ValueError):
        return None
    if not isinstance(data, dict) or data.get("aruco_id") is None:
        return None
    return {"aruco_id": int(data["aruco_id"])}


def parse_position_payload(payload):
    text = payload.strip()
    if not text.startswith("{"):
        return None
    try:
        data = json.loads(text)
    except (TypeError, ValueError):
        return None
    if not isinstance(data, dict):
        return None

    aruco_id = data.get("ArUco_ID", data.get("aruco_id"))
    x = data.get("x_position", data.get("x"))
    y = data.get("y_position", data.get("y"))
    if aruco_id is None or x is None or y is None:
        return None

    marker_type = str(data.get("marker_type", "")).lower()
    if marker_type == "corner" or is_corner_aruco(aruco_id):
        return {
            "kind": "corner",
            "aruco_id": int(aruco_id),
            "x": int(x),
            "y": int(y),
        }

    orientation = data.get("orientation")
    led_raw = data.get("led_status")
    if aruco_id is None or x is None or y is None or orientation is None or led_raw is None:
        return None

    led_key = str(led_raw).strip().lower()
    if led_key == "connecting":
        led_status = "connecting"
    elif led_key == "connected":
        led_status = "connected"
    else:
        led_status = "off"

    return {
        "kind": "robot",
        "aruco_id": int(aruco_id),
        "x": int(x),
        "y": int(y),
        "orientation": float(orientation),
        "led_status": led_status,
    }


def format_report_payload(status, seq=0, x=None, y=None):
    payload = {"status": status, "seq": int(seq)}
    if x is not None:
        payload["x"] = round(float(x), 1)
    if y is not None:
        payload["y"] = round(float(y), 1)
    return json.dumps(payload)


class GoalNavigator:
    def __init__(
        self,
        arrival_distance=DEFAULT_GOAL_TOLERANCE,
        heading_tolerance=DEFAULT_HEADING_TOLERANCE,
        position_timeout=DEFAULT_POSITION_TIMEOUT,
        apf_influence_radius=DEFAULT_APF_INFLUENCE_RADIUS,
        apf_k_attract=DEFAULT_APF_K_ATTRACT,
        apf_k_repel=DEFAULT_APF_K_REPEL,
        min_separation=DEFAULT_MIN_SEPARATION,
        forward_block_distance=DEFAULT_FORWARD_BLOCK_DISTANCE,
        forward_block_angle=DEFAULT_FORWARD_BLOCK_ANGLE,
        neighbor_stale_sec=DEFAULT_NEIGHBOR_STALE_SEC,
        field_margin=DEFAULT_FIELD_MARGIN,
        turn_pulse_sec=DEFAULT_TURN_PULSE_SEC,
        turn_settle_sec=DEFAULT_TURN_SETTLE_SEC,
    ):
        self.arrival_distance = arrival_distance
        self.heading_tolerance = heading_tolerance
        self.position_timeout = position_timeout
        self.apf_influence_radius = apf_influence_radius
        self.apf_k_attract = apf_k_attract
        self.apf_k_repel = apf_k_repel
        self.min_separation = min_separation
        self.forward_block_distance = forward_block_distance
        self.forward_block_angle = forward_block_angle
        self.neighbor_stale_sec = neighbor_stale_sec
        self.field_margin = field_margin
        self.turn_pulse_sec = float(turn_pulse_sec)
        self.turn_settle_sec = float(turn_settle_sec)
        self.own_aruco_id = None
        self.target_x = None
        self.target_y = None
        self.tolerance = arrival_distance
        self.seq = 0
        self.x = None
        self.y = None
        self.orientation = None
        self.last_position_time = 0
        self.autonomous = True
        self._reported_arrival = False
        self._fleet = {}
        self._field_bounds = FieldBounds()
        self._pulse_command = None
        self._pulse_phase = None
        self._pulse_run_until = 0.0
        self._pulse_settle_until = 0.0
        self._pulse_position_stamp = 0.0

    @property
    def field_bounds(self):
        return self._field_bounds

    @property
    def has_goal(self):
        return self.target_x is not None and self.target_y is not None

    @property
    def has_position(self):
        return self.x is not None and self.y is not None and self.orientation is not None

    def set_aruco_id(self, aruco_id):
        self.own_aruco_id = int(aruco_id) if aruco_id is not None else None

    def set_goal(self, target_x, target_y, tolerance=None, seq=0):
        tx = float(target_x)
        ty = float(target_y)
        if self._field_bounds.is_ready:
            tx, ty = self._field_bounds.clamp_point(tx, ty, self.field_margin)
        self.target_x = tx
        self.target_y = ty
        self.tolerance = float(tolerance if tolerance is not None else self.arrival_distance)
        self.seq = int(seq)
        self._reported_arrival = False
        self.autonomous = True
        self._clear_turn_pulse()

    def clear_goal(self):
        self.target_x = None
        self.target_y = None
        self._reported_arrival = False
        self._clear_turn_pulse()

    def update_position(self, x, y, orientation, now=None):
        self.x = float(x)
        self.y = float(y)
        self.orientation = float(orientation)
        self.last_position_time = now if now is not None else time.time()

    def update_fleet_position(self, aruco_id, x, y, now=None):
        if is_corner_aruco(aruco_id):
            self.update_field_corner(aruco_id, x, y)
            return
        stamp = now if now is not None else time.time()
        self._fleet[int(aruco_id)] = {
            "x": float(x),
            "y": float(y),
            "t": stamp,
        }

    def update_field_corner(self, aruco_id, x, y):
        self._field_bounds.update_corner(aruco_id, x, y)
        self._reclamp_goal()

    def _reclamp_goal(self):
        if not self.has_goal or not self._field_bounds.is_ready:
            return
        assert self.target_x is not None
        assert self.target_y is not None
        self.target_x, self.target_y = self._field_bounds.clamp_point(
            self.target_x,
            self.target_y,
            self.field_margin,
        )

    def _effective_goal(self):
        if not self.has_goal:
            return None, None
        assert self.target_x is not None
        assert self.target_y is not None
        if self._field_bounds.is_ready:
            return self._field_bounds.clamp_point(
                self.target_x,
                self.target_y,
                self.field_margin,
            )
        return self.target_x, self.target_y

    def _neighbor_positions(self, now):
        neighbors = []
        closest = None
        nearest = None

        for aruco_id, info in self._fleet.items():
            if self.own_aruco_id is not None and aruco_id == self.own_aruco_id:
                continue
            if now - info["t"] > self.neighbor_stale_sec:
                continue
            neighbors.append((info["x"], info["y"]))
            if self.has_position:
                dist = _hypot(info["x"] - self.x, info["y"] - self.y)
                if closest is None or dist < closest:
                    closest = dist
                    nearest = (info["x"], info["y"])

        return neighbors, closest, nearest

    def _heading_unit_vector(self):
        heading_rad = math.radians(self.orientation)
        return math.cos(heading_rad), math.sin(heading_rad)

    def _neighbor_blocking_forward(self, neighbors):
        if not self.has_position or not neighbors:
            return None

        hx, hy = self._heading_unit_vector()
        cos_limit = math.cos(math.radians(self.forward_block_angle))
        blocker = None
        blocker_dist = None

        for ox, oy in neighbors:
            dx = ox - self.x
            dy = oy - self.y
            dist = _hypot(dx, dy)
            if dist == 0 or dist > self.forward_block_distance:
                continue
            cos_angle = (dx * hx + dy * hy) / dist
            if cos_angle >= cos_limit and (blocker_dist is None or dist < blocker_dist):
                blocker_dist = dist
                blocker = (ox, oy)

        return blocker

    def _turn_away_command(self, obstacle_x, obstacle_y):
        escape_bearing = bearing_degrees(self.x - obstacle_x, self.y - obstacle_y)
        heading_error = normalize_angle(escape_bearing - self.orientation)
        return steer_command_code(heading_error, self.heading_tolerance)

    def enter_manual_mode(self):
        self.autonomous = False
        self.clear_goal()

    def _clear_turn_pulse(self):
        self._pulse_command = None
        self._pulse_phase = None
        self._pulse_run_until = 0.0
        self._pulse_settle_until = 0.0
        self._pulse_position_stamp = 0.0

    def _apply_turn_pulse(self, command, now):
        if self.turn_pulse_sec <= 0 or command not in TURN_COMMANDS:
            self._clear_turn_pulse()
            return command

        if self._pulse_phase == "run":
            if command != self._pulse_command:
                self._clear_turn_pulse()
            elif now < self._pulse_run_until:
                return self._pulse_command
            else:
                self._pulse_phase = "settle"
                self._pulse_settle_until = now + self.turn_settle_sec
                print("4444444444")
                return "SS"

        if self._pulse_phase == "settle":
            fresh_position = self.last_position_time > self._pulse_position_stamp
            if not fresh_position and now < self._pulse_settle_until:
                print("33333333333")
                return "SS"
            self._clear_turn_pulse()

        self._pulse_phase = "run"
        self._pulse_command = command
        self._pulse_run_until = now + self.turn_pulse_sec
        self._pulse_position_stamp = self.last_position_time
        return command

    def _movement_command(self, command, now):
        return ("command", command, now)
        # return ("command", self._apply_turn_pulse(command, now))

    def tick(self, now=None):
        if not self.autonomous or not self.has_goal:
            return None

        now = now if now is not None else time.time()
        if not self.has_position:
            return None

        if now - self.last_position_time > self.position_timeout:
            self._clear_turn_pulse()
            return ("command", "SS")

        goal_x, goal_y = self._effective_goal()
        if goal_x is None or goal_y is None:
            return None

        dx = goal_x - self.x
        dy = goal_y - self.y
        distance = _hypot(dx, dy)

        if distance < self.tolerance:
            if not self._reported_arrival:
                self._reported_arrival = True
                seq = self.seq
                self.clear_goal()
                return ("report", "arrived", seq)
            return ("command", "SS")

        neighbors, closest, nearest = self._neighbor_positions(now)

        # if nearest is not None and closest is not None and closest < self.min_separation:
        #     return self._movement_command(
        #         self._turn_away_command(nearest[0], nearest[1]), now
        #     )

        escape_heading = self._field_bounds.escape_heading(
            self.x, self.y, self.field_margin
        )
        if escape_heading is not None:
            target_heading = escape_heading
        # elif neighbors:
        #     print("BUUUUUUREN")
        #     target_heading = calculate_apf_heading(
        #         self.x,
        #         self.y,
        #         goal_x,
        #         goal_y,
        #         neighbors,
        #         influence_radius=self.apf_influence_radius,
        #         k_attract=self.apf_k_attract,
        #         k_repel=self.apf_k_repel,
        #     )
        else:
            target_heading = bearing_degrees(dx, dy)

        #flip target heading 180 degrees
        # target_heading += 180

        # if target_heading > 180:
        #     target_heading -= 360
        # elif target_heading <= -180:
        #     target_heading += 360

        heading_error = normalize_angle(self.orientation - target_heading)
        command = steer_command_code(heading_error, self.heading_tolerance)

        print("----------------------------")
        print("target_heading:", target_heading)
        print("orientation:", self.orientation)
        print("Heading_error:", heading_error)
        print("command:", command)


        blocker = self._neighbor_blocking_forward(neighbors)
        if command == "FW" and blocker is not None:
            command = self._turn_away_command(blocker[0], blocker[1])
        elif command == "FW" and closest is not None and closest < self.forward_block_distance:
            if nearest is not None:
                command = self._turn_away_command(nearest[0], nearest[1])
            else:
                command = "SS"

        return self._movement_command(command, now)
