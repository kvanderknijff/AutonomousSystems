import logging
import sys

from central_server import CentralServer
from config import (
    COMMAND_RESEND_INTERVAL,
    DATABASE_PATH,
    FORMATION,
    FORMATION_AUTO_CENTER,
    FORMATION_CENTER_X,
    FORMATION_CENTER_Y,
    FORMATION_MIN_ROBOTS,
    FORMATION_SPACING,
    MQTT_BROKER,
    MQTT_PORT,
    WEB_ENABLED,
    WEB_HOST,
    WEB_PORT,
)
from planner_service import PlannerController
from web_server import start_web_server

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    stream=sys.stdout,
)
logger = logging.getLogger(__name__)

if __name__ == "__main__":
    print(f"MQTT broker: {MQTT_BROKER}:{MQTT_PORT}")
    print(f"Database: {DATABASE_PATH}")
    print("Planner: event-driven (runs on each camera position update)")
    if WEB_ENABLED:
        print(f"Web UI: http://{WEB_HOST if WEB_HOST != '0.0.0.0' else 'localhost'}:{WEB_PORT}")
    else:
        print("Web UI: disabled (set WEB_ENABLED=true to enable)")
    if FORMATION:
        print(f"Startup formation: {FORMATION} (can be changed from web UI)")
    else:
        print("Formation: use web UI or set FORMATION=line|plus|square")
    print("Ctrl+C to stop.\n")

    server = CentralServer()
    controller = PlannerController(
        server,
        center_x=FORMATION_CENTER_X,
        center_y=FORMATION_CENTER_Y,
        spacing=FORMATION_SPACING,
        min_robots_for_formation=FORMATION_MIN_ROBOTS,
        command_resend_interval=COMMAND_RESEND_INTERVAL,
        formation_auto_center=FORMATION_AUTO_CENTER,
        initial_formation=FORMATION or None,
    )
    controller.attach(server)

    if WEB_ENABLED:
        start_web_server(controller, WEB_HOST, WEB_PORT)

    try:
        server.start(blocking=True)
    except KeyboardInterrupt:
        logger.info("Shutting down")
        server.stop()
