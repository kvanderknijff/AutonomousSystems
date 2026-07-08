import math
import os
import random
import sys

import paho.mqtt.client as mqtt
from controller import Supervisor
from paho.mqtt.client import CallbackAPIVersion

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from mqtt_protocol import (  # noqa: E402
    BROKER,
    CORNER_ARUCO_FIRST,
    CORNER_ARUCO_LAST,
    MQTT_PASS,
    MQTT_USER,
    PORT,
    TOPIC_DATA_POSITIONS_SIMULATION,
    format_corner_payload,
    format_position_payload,
    is_corner_aruco,
    is_physical_corner_aruco,
)
from webots_nodes import is_disabled_robot, is_physical_proxy, parse_custom_data, resolve_aruco_id  # noqa: E402


supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

camera = supervisor.getDevice("overhead_camera")
if camera is not None:
    camera.enable(timestep)

FIELD_SIZE_M = float(os.getenv("WEBOTS_FIELD_SIZE_M", "1.0"))
CAMERA_WIDTH = int(os.getenv("WEBOTS_CAMERA_WIDTH", "640"))
CAMERA_HEIGHT = int(os.getenv("WEBOTS_CAMERA_HEIGHT", "480"))
PUBLISH_INTERVAL_MS = int(os.getenv("WEBOTS_POSITION_INTERVAL_MS", "250"))
LOG_INTERVAL_MS = int(os.getenv("WEBOTS_CAMERA_LOG_INTERVAL_MS", "1000"))
HALF_FIELD = FIELD_SIZE_M / 2.0
CORNER_INSET_M = float(os.getenv("WEBOTS_CORNER_INSET_M", "0.035"))

client = mqtt.Client(
    protocol=mqtt.MQTTv311,
    callback_api_version=CallbackAPIVersion.VERSION2,
    client_id=f"webots_camera_{random.randint(0, 9999)}",
)
client.username_pw_set(MQTT_USER, MQTT_PASS)


def on_connect(client, userdata, flags, reason_code, properties):
    print(f"[Camera] MQTT connected ({reason_code}) -> {BROKER}:{PORT}")


client.on_connect = on_connect

try:
    client.connect(BROKER, PORT)
    client.loop_start()
except Exception as exc:
    print(f"[Camera] MQTT connection failed: {exc}")


def tracked_nodes():
    children = supervisor.getRoot().getField("children")
    robots = []
    corners = []
    for index in range(children.getCount()):
        node = children.getMFNode(index)
        aruco_id = resolve_aruco_id(node)
        if aruco_id is None:
            continue
        if is_physical_proxy(node) or is_disabled_robot(node):
            continue

        if is_physical_corner_aruco(aruco_id):
            continue

        if is_corner_aruco(aruco_id):
            corners.append((node, aruco_id))
        else:
            robots.append((node, aruco_id))
    return robots, corners


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def world_to_camera_pixels(world_x, world_y):
    x = round(((world_x + HALF_FIELD) / FIELD_SIZE_M) * CAMERA_WIDTH)
    y = round(((HALF_FIELD - world_y) / FIELD_SIZE_M) * CAMERA_HEIGHT)
    return clamp(x, 0, CAMERA_WIDTH), clamp(y, 0, CAMERA_HEIGHT)


def camera_orientation_degrees(node):
    orientation = node.getOrientation()
    forward_x = orientation[0]
    forward_y = orientation[3]
    world_yaw = math.degrees(math.atan2(forward_y, forward_x))
    return round((-world_yaw) % 360, 1)


def led_status(node):
    custom_field = node.getField("customData")
    if custom_field is None:
        return "connecting"

    custom = parse_custom_data(custom_field.getSFString())
    status = custom.get("status", "connecting").lower()
    if status not in ("connecting", "connected", "off"):
        return "connecting"
    return status


tracked_robots, tracked_corners = tracked_nodes()
print(
    f"[Camera] Tracking {len(tracked_robots)} robots, "
    f"{len(tracked_corners)} corner markers (ArUco {CORNER_ARUCO_FIRST}-{CORNER_ARUCO_LAST})"
)
if not tracked_corners:
    print(
        "[Camera] No Webots corner markers in scene — "
        "field bounds come from physical camera corners (ArUco 5-8)."
    )
last_publish = 0
last_log = 0

while supervisor.step(timestep) != -1:
    now = int(supervisor.getTime() * 1000)
    if now - last_publish < PUBLISH_INTERVAL_MS:
        continue
    last_publish = now

    tracked_robots, tracked_corners = tracked_nodes()
    published_robots = []
    published_corners = []

    for node, aruco_id in tracked_corners:
        world_x, world_y, _ = node.getPosition()
        pixel_x, pixel_y = world_to_camera_pixels(world_x, world_y)
        payload = format_corner_payload(aruco_id, pixel_x, pixel_y)
        client.publish(TOPIC_DATA_POSITIONS_SIMULATION, payload)
        published_corners.append(f"{aruco_id}:({pixel_x},{pixel_y})")

    for node, aruco_id in tracked_robots:
        world_x, world_y, _ = node.getPosition()
        pixel_x, pixel_y = world_to_camera_pixels(world_x, world_y)
        orientation = camera_orientation_degrees(node)
        status = led_status(node)
        payload = format_position_payload(aruco_id, pixel_x, pixel_y, orientation, status)
        client.publish(TOPIC_DATA_POSITIONS_SIMULATION, payload)
        published_robots.append(f"{aruco_id}:({pixel_x},{pixel_y},{orientation},{status})")

    if now - last_log >= LOG_INTERVAL_MS:
        last_log = now
        parts = []
        if published_corners:
            parts.append("corners " + " ".join(published_corners))
        if published_robots:
            parts.append("robots " + " ".join(published_robots))
        if parts:
            print("[Camera] Published " + " | ".join(parts))

client.loop_stop()
client.disconnect()
