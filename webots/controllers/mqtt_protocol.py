"""
MQTT protocol constants for Webots controllers (robots + camera).

Broker and run.py run on the remote server; Webots on your local PC connects
to that server via its public IP below.
"""

import json
import os
import re

BROKER = os.getenv("MQTT_BROKER", "145.24.237.88")
PORT = int(os.getenv("MQTT_PORT", "8883"))
MQTT_USER = os.getenv("MQTT_USER", "myuser")
MQTT_PASS = os.getenv("MQTT_PASS", "FormingFormsAS")

TOPIC_CONTROL_CONNECTING = "Robots/Control/Connecting"
TOPIC_CONTROL_STATUS = "Robots/Control/{mac}/Status"
TOPIC_DATA_POSITIONS = "Robots/Data/Positions"
TOPIC_DATA_POSITIONS_PHYSICAL = "Robots/Data/Positions/Physical"
TOPIC_DATA_POSITIONS_SIMULATION = "Robots/Data/Positions/Simulation"
TOPIC_DATA_POSITIONS_WILD = "Robots/Data/Positions/#"

POSITION_SOURCE_PHYSICAL = "physical"
POSITION_SOURCE_SIMULATION = "simulation"
TOPIC_DATA_COMMANDS = "Robots/Data/{mac}/Commands"
TOPIC_DATA_GOALS = "Robots/Data/{mac}/Goals"
TOPIC_DATA_CONFIG = "Robots/Data/{mac}/Config"
TOPIC_DATA_REPORT = "Robots/Data/{mac}/Report"

STATUS_CHECKING = "checking"
STATUS_CONNECTED = "connected"
STATUS_DISCONNECTED = "disconnected"

LED_CONNECTING = "connecting"
LED_CONNECTED = "connected"
LED_OFF = "off"

GOAL_ACTION_SET = "set"
GOAL_ACTION_CLEAR = "clear"

DEFAULT_GOAL_TOLERANCE = 12.0

# Physical field (overhead camera): robots 1-4, corners 5-8
PHYSICAL_ROBOT_ARUCO_FIRST = int(os.getenv("PHYSICAL_ROBOT_ARUCO_FIRST", "1"))
PHYSICAL_ROBOT_ARUCO_LAST = int(os.getenv("PHYSICAL_ROBOT_ARUCO_LAST", "4"))
PHYSICAL_CORNER_ARUCO_FIRST = int(os.getenv("PHYSICAL_CORNER_ARUCO_FIRST", "5"))
PHYSICAL_CORNER_ARUCO_LAST = int(os.getenv("PHYSICAL_CORNER_ARUCO_LAST", "8"))
# Webots simulation: robots 11-14, corners 15-18
WEBOTS_ROBOT_ARUCO_FIRST = int(os.getenv("WEBOTS_ROBOT_ARUCO_FIRST", "11"))
WEBOTS_ROBOT_ARUCO_LAST = int(os.getenv("WEBOTS_ROBOT_ARUCO_LAST", "14"))
CORNER_ARUCO_FIRST = int(os.getenv("WEBOTS_CORNER_ARUCO_FIRST", "15"))
CORNER_ARUCO_LAST = int(os.getenv("WEBOTS_CORNER_ARUCO_LAST", "18"))
MARKER_TYPE_CORNER = "corner"
MARKER_TYPE_ROBOT = "robot"

COMMANDS = {
    "FW": "forward",
    "BW": "backward",
    "TL": "turn_left",
    "TR": "turn_right",
    "RL": "rotate_left",
    "RR": "rotate_right",
    "SS": "stop",
}


def bracket_payload(value):
    return f"[{value}]"


def parse_bracket_payload(payload):
    text = payload.strip()
    if text.startswith("[") and text.endswith("]"):
        return text[1:-1]
    return text


def control_status_topic(mac):
    return TOPIC_CONTROL_STATUS.format(mac=mac)


def data_commands_topic(mac):
    return TOPIC_DATA_COMMANDS.format(mac=mac)


def data_goals_topic(mac):
    return TOPIC_DATA_GOALS.format(mac=mac)


def data_config_topic(mac):
    return TOPIC_DATA_CONFIG.format(mac=mac)


def data_report_topic(mac):
    return TOPIC_DATA_REPORT.format(mac=mac)


def is_physical_robot_aruco(aruco_id):
    aid = int(aruco_id)
    return PHYSICAL_ROBOT_ARUCO_FIRST <= aid <= PHYSICAL_ROBOT_ARUCO_LAST


def is_physical_corner_aruco(aruco_id):
    aid = int(aruco_id)
    return PHYSICAL_CORNER_ARUCO_FIRST <= aid <= PHYSICAL_CORNER_ARUCO_LAST


def is_webots_robot_aruco(aruco_id):
    aid = int(aruco_id)
    return WEBOTS_ROBOT_ARUCO_FIRST <= aid <= WEBOTS_ROBOT_ARUCO_LAST


def is_corner_aruco(aruco_id, first=CORNER_ARUCO_FIRST, last=CORNER_ARUCO_LAST):
    return first <= int(aruco_id) <= last


def format_corner_payload(aruco_id, x, y):
    return json.dumps(
        {
            "ArUco_ID": int(aruco_id),
            "x_position": int(x),
            "y_position": int(y),
            "marker_type": MARKER_TYPE_CORNER,
        },
        separators=(",", ":"),
    )


def format_position_payload(aruco_id, x, y, orientation, led_status):
    return bracket_payload(f'{aruco_id}, {x}, {y}, {orientation}, "{led_status}"')


def normalize_led_status(value):
    key = value.strip().lower()
    aliases = {
        "connecting": LED_CONNECTING,
        "connected": LED_CONNECTED,
        "off": LED_OFF,
    }
    return aliases.get(key)


def parse_goal_payload(payload):
    text = payload.strip()
    if not text:
        return None
    if text.startswith("{"):
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
    return None


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


def format_report_payload(status, seq=0, x=None, y=None):
    payload = {"status": status, "seq": int(seq)}
    if x is not None:
        payload["x"] = round(float(x), 1)
    if y is not None:
        payload["y"] = round(float(y), 1)
    return json.dumps(payload, separators=(",", ":"))


def _parse_position_json(payload):
    try:
        data = json.loads(payload)
    except (TypeError, ValueError):
        return None
    if not isinstance(data, dict):
        return None

    aruco_id = data.get("ArUco_ID", data.get("aruco_id"))
    x = data.get("x_position", data.get("x"))
    y = data.get("y_position", data.get("y"))
    orientation = data.get("orientation")
    led_raw = data.get("led_status")

    if aruco_id is None or x is None or y is None:
        return None

    marker_type = str(data.get("marker_type", "")).lower()
    if marker_type == MARKER_TYPE_CORNER or is_corner_aruco(int(aruco_id)) or is_physical_corner_aruco(int(aruco_id)):
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


def parse_position_payload(payload):
    text = payload.strip()
    if text.startswith("{"):
        return _parse_position_json(text)

    inner = parse_bracket_payload(text)
    corner_match = re.match(
        r'(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*"corner"',
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
