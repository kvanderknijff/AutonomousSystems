"""MQTT protocol constants and payload helpers (shared contract with Webots & robots)."""

import re

TOPIC_CONTROL_CONNECTING = "Robots/Control/Connecting"
TOPIC_CONTROL_STATUS = "Robots/Control/{mac}/Status"
TOPIC_DATA_POSITIONS = "Robots/Data/Positions"
TOPIC_DATA_COMMANDS = "Robots/Data/{mac}/Commands"

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


def control_status_topic(mac: str) -> str:
    return TOPIC_CONTROL_STATUS.format(mac=mac)


def data_commands_topic(mac: str) -> str:
    return TOPIC_DATA_COMMANDS.format(mac=mac)


def format_position_payload(aruco_id: int, x: int, y: int, orientation: int, led_status: str) -> str:
    return bracket_payload(f'{aruco_id}, {x}, {y}, {orientation}, "{led_status}"')


def parse_position_payload(payload: str) -> dict | None:
    inner = parse_bracket_payload(payload)
    match = re.match(
        r'(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*"(connecting|connected|off)"',
        inner,
    )
    if not match:
        return None
    return {
        "aruco_id": int(match.group(1)),
        "x": int(match.group(2)),
        "y": int(match.group(3)),
        "orientation": int(match.group(4)),
        "led_status": match.group(5),
    }
