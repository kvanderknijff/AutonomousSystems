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

# Path planner loop interval (seconds)
PLANNER_INTERVAL = float(os.getenv("PLANNER_INTERVAL", "0.25"))

# Optional startup formation: line, plus, square, or empty to wait for web UI
FORMATION = os.getenv("FORMATION", "")
FORMATION_CENTER_X = float(os.getenv("FORMATION_CENTER_X", "320"))
FORMATION_CENTER_Y = float(os.getenv("FORMATION_CENTER_Y", "240"))
FORMATION_SPACING = float(os.getenv("FORMATION_SPACING", "40"))
FORMATION_MIN_ROBOTS = int(os.getenv("FORMATION_MIN_ROBOTS", "2"))
FORMATION_AUTO_CENTER = os.getenv("FORMATION_AUTO_CENTER", "true").lower() in ("1", "true", "yes")
COMMAND_RESEND_INTERVAL = int(os.getenv("COMMAND_RESEND_INTERVAL", "4"))

# Formation web UI
WEB_ENABLED = os.getenv("WEB_ENABLED", "true").lower() in ("1", "true", "yes")
WEB_HOST = os.getenv("WEB_HOST", "0.0.0.0")
WEB_PORT = int(os.getenv("WEB_PORT", "8080"))
