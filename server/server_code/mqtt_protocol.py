"""MQTT protocol constants and payload helpers (shared contract with Webots & robots)."""

import json
import re

from config import (
    CORNER_ARUCO_FIRST,
    CORNER_ARUCO_LAST,
    PHYSICAL_GOAL_TOLERANCE,
    WEBOTS_CORNER_ARUCO_FIRST,
    WEBOTS_CORNER_ARUCO_LAST,
    WEBOTS_GOAL_TOLERANCE,
    WEBOTS_ROBOT_ARUCO_FIRST,
    WEBOTS_ROBOT_ARUCO_LAST,
)

TOPIC_CONTROL_CONNECTING = "Robots/Control/Connecting"
TOPIC_CONTROL_STATUS = "Robots/Control/{mac}/Status"
TOPIC_DATA_POSITIONS = "Robots/Data/Positions"
TOPIC_DATA_COMMANDS = "Robots/Data/{mac}/Commands"
TOPIC_DATA_GOALS = "Robots/Data/{mac}/Goals"
TOPIC_DATA_CONFIG = "Robots/Data/{mac}/Config"
TOPIC_DATA_REPORT = "Robots/Data/{mac}/Report"
TOPIC_DATA_REPORT_WILD = "Robots/Data/+/Report"

GOAL_ACTION_SET = "set"
GOAL_ACTION_CLEAR = "clear"

ROBOT_REPORT_MOVING = "moving"
ROBOT_REPORT_ARRIVED = "arrived"
ROBOT_REPORT_IDLE = "idle"
ROBOT_REPORT_BLOCKED = "blocked"

DEFAULT_GOAL_TOLERANCE = 12.0
DEFAULT_HEADING_TOLERANCE = 12.0

MARKER_TYPE_CORNER = "corner"
MARKER_TYPE_ROBOT = "robot"

STATUS_CHECKING = "checking"
STATUS_CONNECTED = "connected"
STATUS_DISCONNECTED = "disconnected"

LED_CONNECTING = "connecting"
LED_CONNECTED = "connected"
LED_OFF = "off"

COMMANDS = {
    "FW": "forward",
    "BW": "backward",
    "TL": "turn_left",
    "TR": "turn_right",
    "RL": "rotate_left",
    "RR": "rotate_right",
    "SS": "stop",
}


def bracket_payload(value: str) -> str:
    return f"[{value}]"


def parse_bracket_payload(payload: str) -> str:
    text = payload.strip()
    if text.startswith("[") and text.endswith("]"):
        return text[1:-1]
    return text


def parse_connecting_payload(payload: str) -> tuple[str, int | None]:
    """Parse [mac] or [mac, aruco_id] from robot handshake messages."""
    inner = parse_bracket_payload(payload)
    parts = [part.strip() for part in inner.split(",", 1)]
    mac = parts[0]
    if not mac:
        return "", None
    if len(parts) == 1 or not parts[1]:
        return mac, None
    try:
        return mac, int(parts[1])
    except ValueError:
        return mac, None


def control_status_topic(mac: str) -> str:
    return TOPIC_CONTROL_STATUS.format(mac=mac)


def data_commands_topic(mac: str) -> str:
    return TOPIC_DATA_COMMANDS.format(mac=mac)


def data_goals_topic(mac: str) -> str:
    return TOPIC_DATA_GOALS.format(mac=mac)


def data_config_topic(mac: str) -> str:
    return TOPIC_DATA_CONFIG.format(mac=mac)


def data_report_topic(mac: str) -> str:
    return TOPIC_DATA_REPORT.format(mac=mac)


def format_goal_payload(
    target_x: float,
    target_y: float,
    *,
    tolerance: float = DEFAULT_GOAL_TOLERANCE,
    seq: int = 0,
    action: str = GOAL_ACTION_SET,
) -> str:
    if action == GOAL_ACTION_CLEAR:
        return json.dumps(
            {"action": GOAL_ACTION_CLEAR, "seq": int(seq)},
            separators=(",", ":"),
        )
    return json.dumps(
        {
            "action": GOAL_ACTION_SET,
            "target_x": round(float(target_x), 1),
            "target_y": round(float(target_y), 1),
            "tolerance": round(float(tolerance), 1),
            "seq": int(seq),
        },
        separators=(",", ":"),
    )


def parse_goal_payload(payload: str) -> dict | None:
    text = payload.strip()
    if not text:
        return None
    if text.startswith("{"):
        try:
            data = json.loads(text)
        except json.JSONDecodeError:
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

    inner = parse_bracket_payload(text)
    parts = [part.strip() for part in inner.split(",")]
    if len(parts) == 1 and parts[0].upper() == "CLEAR":
        return {"action": GOAL_ACTION_CLEAR, "seq": 0}
    if len(parts) < 2:
        return None
    try:
        target_x = float(parts[0])
        target_y = float(parts[1])
        tolerance = float(parts[2]) if len(parts) > 2 else DEFAULT_GOAL_TOLERANCE
        seq = int(parts[3]) if len(parts) > 3 else 0
    except ValueError:
        return None
    return {
        "action": GOAL_ACTION_SET,
        "target_x": target_x,
        "target_y": target_y,
        "tolerance": tolerance,
        "seq": seq,
    }


def format_config_payload(aruco_id: int) -> str:
    return json.dumps({"aruco_id": int(aruco_id)}, separators=(",", ":"))


def parse_config_payload(payload: str) -> dict | None:
    text = payload.strip()
    if not text.startswith("{"):
        return None
    try:
        data = json.loads(text)
    except json.JSONDecodeError:
        return None
    if not isinstance(data, dict) or data.get("aruco_id") is None:
        return None
    return {"aruco_id": int(data["aruco_id"])}


def format_report_payload(
    status: str,
    *,
    seq: int = 0,
    x: float | None = None,
    y: float | None = None,
) -> str:
    payload: dict[str, object] = {"status": status, "seq": int(seq)}
    if x is not None:
        payload["x"] = round(float(x), 1)
    if y is not None:
        payload["y"] = round(float(y), 1)
    return json.dumps(payload, separators=(",", ":"))


def parse_report_payload(payload: str) -> dict | None:
    text = payload.strip()
    if not text.startswith("{"):
        return None
    try:
        data = json.loads(text)
    except json.JSONDecodeError:
        return None
    if not isinstance(data, dict) or not data.get("status"):
        return None
    report = {
        "status": str(data["status"]).lower(),
        "seq": int(data.get("seq", 0)),
    }
    if data.get("x") is not None:
        report["x"] = float(data["x"])
    if data.get("y") is not None:
        report["y"] = float(data["y"])
    return report


def format_corner_payload(aruco_id: int, x: int, y: int) -> str:
    return json.dumps(
        {
            "ArUco_ID": int(aruco_id),
            "x_position": int(x),
            "y_position": int(y),
            "marker_type": MARKER_TYPE_CORNER,
        },
        separators=(",", ":"),
    )


def is_corner_aruco(aruco_id: int) -> bool:
    aid = int(aruco_id)
    if CORNER_ARUCO_FIRST <= aid <= CORNER_ARUCO_LAST:
        return True
    return WEBOTS_CORNER_ARUCO_FIRST <= aid <= WEBOTS_CORNER_ARUCO_LAST


def is_webots_robot_aruco(aruco_id: int) -> bool:
    aid = int(aruco_id)
    return WEBOTS_ROBOT_ARUCO_FIRST <= aid <= WEBOTS_ROBOT_ARUCO_LAST


def goal_tolerance_for_aruco(aruco_id: int | None) -> float:
    """Pick goal tolerance from ArUco ID; physical robots get a looser threshold."""
    if aruco_id is not None and is_webots_robot_aruco(aruco_id):
        return WEBOTS_GOAL_TOLERANCE
    return PHYSICAL_GOAL_TOLERANCE


def format_position_payload(aruco_id: int, x: int, y: int, orientation: int, led_status: str) -> str:
    return bracket_payload(f'{aruco_id}, {x}, {y}, {orientation}, "{led_status}"')


def normalize_led_status(value: str) -> str | None:
    """Map Webots and physical-camera LED labels to protocol values."""
    key = value.strip().lower()
    aliases = {
        "connecting": LED_CONNECTING,
        "connected": LED_CONNECTED,
        "off": LED_OFF,
    }
    return aliases.get(key)


def _parse_position_json(payload: str) -> dict | None:
    """Parse JSON payloads on Robots/Data/Positions (robots or corner markers)."""
    try:
        data = json.loads(payload)
    except json.JSONDecodeError:
        return None
    if not isinstance(data, dict):
        return None

    aruco_id = data.get("ArUco_ID", data.get("aruco_id"))
    x = data.get("x_position", data.get("x"))
    y = data.get("y_position", data.get("y"))
    if aruco_id is None or x is None or y is None:
        return None

    if is_corner_aruco(int(aruco_id)):
        return {
            "kind": MARKER_TYPE_CORNER,
            "aruco_id": int(aruco_id),
            "x": int(x),
            "y": int(y),
        }

    orientation = data.get("orientation")
    led_raw = data.get("led_status")
    if orientation is None or led_raw is None:
        return None

    led_status = normalize_led_status(str(led_raw))
    if led_status is None:
        return None

    return {
        "kind": MARKER_TYPE_ROBOT,
        "aruco_id": int(aruco_id),
        "x": int(x),
        "y": int(y),
        "orientation": float(orientation),
        "led_status": led_status,
    }


def parse_position_payload(payload: str) -> dict | None:
    text = payload.strip()
    if text.startswith("{"):
        return _parse_position_json(text)

    inner = parse_bracket_payload(text)
    corner_match = re.match(
        r"(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*\"corner\"",
        inner,
        re.IGNORECASE,
    )
    if corner_match:
        return {
            "kind": MARKER_TYPE_CORNER,
            "aruco_id": int(corner_match.group(1)),
            "x": int(corner_match.group(2)),
            "y": int(corner_match.group(3)),
        }

    match = re.match(
        r'(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*(-?\d+(?:\.\d+)?)\s*,\s*"(connecting|connected|off)"',
        inner,
        re.IGNORECASE,
    )
    if not match:
        return None

    led_status = normalize_led_status(match.group(5))
    if led_status is None:
        return None

    return {
        "kind": MARKER_TYPE_ROBOT,
        "aruco_id": int(match.group(1)),
        "x": int(match.group(2)),
        "y": int(match.group(3)),
        "orientation": float(match.group(4)),
        "led_status": led_status,
    }
