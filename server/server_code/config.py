"""Server configuration — override via environment variables on the host machine."""

import os
from pathlib import Path

# MQTT broker (same host/IP that Webots robots and camera connect to)
MQTT_BROKER = os.getenv("MQTT_BROKER", "127.0.0.1")
MQTT_PORT = int(os.getenv("MQTT_PORT", "8883"))
MQTT_USER = os.getenv("MQTT_USER", "myuser")
MQTT_PASS = os.getenv("MQTT_PASS", "FormingFormsAS")
MQTT_CLIENT_ID = os.getenv("MQTT_CLIENT_ID", "central_server")

# SQLite database file (created automatically)
BASE_DIR = Path(__file__).resolve().parent
DATA_DIR = Path(os.getenv("SERVER_DATA_DIR", BASE_DIR / "data"))
DATABASE_PATH = Path(os.getenv("DATABASE_PATH", DATA_DIR / "robots.db"))

# How often to write position snapshots to history (seconds); 0 = every update
POSITION_HISTORY_INTERVAL = float(os.getenv("POSITION_HISTORY_INTERVAL", "1.0"))

# Path planner: "goals" = server sends target points, robots navigate locally;
# "commands" = legacy server-side FW/TL/TR steering every planner tick.
PLANNER_MODE = os.getenv("PLANNER_MODE", "goals").strip().lower()
if PLANNER_MODE not in ("goals", "commands"):
    PLANNER_MODE = "goals"

# Path planner loop interval (seconds)
PLANNER_INTERVAL = float(os.getenv("PLANNER_INTERVAL", "0.25"))
PLANNER_ROBOT_TIMEOUT = float(os.getenv("PLANNER_ROBOT_TIMEOUT", "10.0"))
# Ignore brief LED/camera gaps before marking a robot offline (consecutive "off" frames)
OFFLINE_LED_STREAK = int(os.getenv("OFFLINE_LED_STREAK", "5"))
# Keep planner state while camera telemetry is fresh, even if MQTT state flaps
PLANNER_DISCONNECT_GRACE = float(os.getenv("PLANNER_DISCONNECT_GRACE", "3.0"))

# Optional startup formation: line, plus, square, y, or empty to wait for web UI
FORMATION = os.getenv("FORMATION", "")
FORMATION_CENTER_X = float(os.getenv("FORMATION_CENTER_X", "320"))
FORMATION_CENTER_Y = float(os.getenv("FORMATION_CENTER_Y", "240"))
FORMATION_SPACING = float(os.getenv("FORMATION_SPACING", "40"))
FORMATION_MIN_ROBOTS = int(os.getenv("FORMATION_MIN_ROBOTS", "1"))
FORMATION_AUTO_CENTER = os.getenv("FORMATION_AUTO_CENTER", "false").lower() in ("1", "true", "yes")
COMMAND_RESEND_INTERVAL = int(os.getenv("COMMAND_RESEND_INTERVAL", "1"))

# Robot ArUco IDs (camera-tracked rovers)
PHYSICAL_ROBOT_ARUCO_FIRST = int(os.getenv("PHYSICAL_ROBOT_ARUCO_FIRST", "1"))
PHYSICAL_ROBOT_ARUCO_LAST = int(os.getenv("PHYSICAL_ROBOT_ARUCO_LAST", "4"))
WEBOTS_ROBOT_ARUCO_FIRST = int(os.getenv("WEBOTS_ROBOT_ARUCO_FIRST", "11"))
WEBOTS_ROBOT_ARUCO_LAST = int(os.getenv("WEBOTS_ROBOT_ARUCO_LAST", "14"))

# Goal arrival tolerance in camera pixels (sent to each robot via MQTT goals)
WEBOTS_GOAL_TOLERANCE = float(os.getenv("WEBOTS_GOAL_TOLERANCE", "2.0"))
PHYSICAL_GOAL_TOLERANCE = float(os.getenv("PHYSICAL_GOAL_TOLERANCE", "60.0"))

# Corner ArUco markers defining the playfield (camera pixels)
# Physical setup: IDs 5-8 on the real field
CORNER_ARUCO_FIRST = int(os.getenv("CORNER_ARUCO_FIRST", "5"))
CORNER_ARUCO_LAST = int(os.getenv("CORNER_ARUCO_LAST", "8"))
# Webots simulation: IDs 15-18 (robots use 11-14)
WEBOTS_CORNER_ARUCO_FIRST = int(os.getenv("WEBOTS_CORNER_ARUCO_FIRST", "15"))
WEBOTS_CORNER_ARUCO_LAST = int(os.getenv("WEBOTS_CORNER_ARUCO_LAST", "18"))
# Keep formation goals and robots inside the field by this many camera pixels
FIELD_MARGIN = float(os.getenv("FIELD_MARGIN", "20"))

# Formation web UI
WEB_ENABLED = os.getenv("WEB_ENABLED", "true").lower() in ("1", "true", "yes")
WEB_HOST = os.getenv("WEB_HOST", "0.0.0.0")
WEB_PORT = int(os.getenv("WEB_PORT", "8080"))
# Log each HTTP request from the formation web UI (noisy when /api/status polls every second)
WEB_ACCESS_LOG = os.getenv("WEB_ACCESS_LOG", "false").lower() in ("1", "true", "yes")
