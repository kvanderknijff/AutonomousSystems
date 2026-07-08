import os
import random
import re
import sys
import threading
import time
import math

import paho.mqtt.client as mqtt
from controller import Supervisor
from paho.mqtt.client import CallbackAPIVersion

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from coordinate_mapper import PhysicalFieldMapper  # noqa: E402
from mqtt_protocol import (  # noqa: E402
    BROKER,
    COMMANDS,
    GOAL_ACTION_CLEAR,
    GOAL_ACTION_SET,
    MARKER_TYPE_CORNER,
    MQTT_PASS,
    MQTT_USER,
    PORT,
    STATUS_CHECKING,
    STATUS_CONNECTED,
    STATUS_DISCONNECTED,
    TOPIC_CONTROL_CONNECTING,
    TOPIC_DATA_POSITIONS_PHYSICAL,
    TOPIC_DATA_POSITIONS_SIMULATION,
    bracket_payload,
    control_status_topic,
    data_commands_topic,
    data_config_topic,
    data_goals_topic,
    data_report_topic,
    format_report_payload,
    is_corner_aruco,
    is_physical_corner_aruco,
    is_physical_robot_aruco,
    parse_bracket_payload,
    parse_config_payload,
    parse_goal_payload,
    parse_position_payload,
)
from nav_goal import GoalNavigator  # noqa: E402
from webots_nodes import is_disabled_robot, parse_custom_data, resolve_aruco_id  # noqa: E402


robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

left_motor = robot.getDevice("left_motor")
right_motor = robot.getDevice("right_motor")
for motor in (left_motor, right_motor):
    motor.setPosition(float("inf"))
    motor.setVelocity(0)

led_connecting = robot.getDevice("led_connecting")
led_connected = robot.getDevice("led_connected")
self_node = robot.getSelf()
custom_data_field = self_node.getField("customData") if self_node else None
base_custom_data = custom_data_field.getSFString() if custom_data_field else ""


def parse_custom_data(raw):
    values = {}
    for part in raw.split(";"):
        if "=" not in part:
            continue
        key, value = part.split("=", 1)
        values[key.strip()] = value.strip()
    return values


def get_aruco_id():
    if not base_custom_data:
        return None
    aruco = parse_custom_data(base_custom_data).get("aruco")
    if aruco is None or not aruco.isdigit():
        return None
    return int(aruco)


try:
    ROBOT_MAC = sys.argv[1]
except IndexError:
    ROBOT_MAC = "00:B0:D0:63:C2:21"
    print(f"[Robot] No MAC in controllerArgs, using default {ROBOT_MAC}")

ROBOT_ARUCO_ID = get_aruco_id()

status_topic = control_status_topic(ROBOT_MAC)
commands_topic = data_commands_topic(ROBOT_MAC)
goals_topic = data_goals_topic(ROBOT_MAC)
config_topic = data_config_topic(ROBOT_MAC)
report_topic = data_report_topic(ROBOT_MAC)
connection_state = "connecting"

DRIVE_SPEED = float(os.getenv("WEBOTS_DRIVE_SPEED", "3.0"))
TURN_SPEED = float(os.getenv("WEBOTS_TURN_SPEED", "1.2"))

navigator = GoalNavigator(
    apf_influence_radius=float(os.getenv("WEBOTS_APF_RADIUS", "80")),
    apf_k_repel=float(os.getenv("WEBOTS_APF_REPEL", "100")),
    min_separation=float(os.getenv("WEBOTS_MIN_SEPARATION", "55")),
    forward_block_distance=float(os.getenv("WEBOTS_FORWARD_BLOCK", "75")),
    field_margin=float(os.getenv("WEBOTS_FIELD_MARGIN", "35")),
)
if ROBOT_ARUCO_ID is not None:
    navigator.set_aruco_id(ROBOT_ARUCO_ID)

FIELD_SIZE_M = float(os.getenv("WEBOTS_FIELD_SIZE_M", "1.0"))
CAMERA_WIDTH = int(os.getenv("WEBOTS_CAMERA_WIDTH", "640"))
CAMERA_HEIGHT = int(os.getenv("WEBOTS_CAMERA_HEIGHT", "480"))
HALF_FIELD = FIELD_SIZE_M / 2.0
CORNER_INSET_M = float(os.getenv("WEBOTS_CORNER_INSET_M", "0.035"))
WORLD_MIN = -HALF_FIELD + CORNER_INSET_M
WORLD_MAX = HALF_FIELD - CORNER_INSET_M
USE_SUPERVISOR_FLEET = os.getenv("WEBOTS_SUPERVISOR_FLEET", "true").lower() in (
    "1",
    "true",
    "yes",
)
physical_field_mapper = PhysicalFieldMapper()
physical_field_ready_logged = False
state_lock = threading.Lock()
last_applied_command = None


def set_custom_status(status):
    if custom_data_field is None:
        return

    parts = []
    seen_status = False
    for part in base_custom_data.split(";"):
        if not part:
            continue
        key = part.split("=", 1)[0].strip()
        if key == "status":
            parts.append(f"status={status}")
            seen_status = True
        else:
            parts.append(part)

    if not seen_status:
        parts.append(f"status={status}")
    custom_data_field.setSFString(";".join(parts))


def set_led_status(status):
    if status == "connected":
        led_connecting.set(0)
        led_connected.set(1)
    elif status == "off":
        led_connecting.set(0)
        led_connected.set(0)
    else:
        led_connecting.set(1)
        led_connected.set(0)
    set_custom_status(status)


def set_wheels(left, right):
    left_motor.setVelocity(left)
    right_motor.setVelocity(right)


def stop():
    set_wheels(0, 0)


def move_forward():
    set_wheels(DRIVE_SPEED, DRIVE_SPEED)


def move_backward():
    set_wheels(-DRIVE_SPEED, -DRIVE_SPEED)


def turn_left():
    set_wheels(DRIVE_SPEED, TURN_SPEED)


def turn_right():
    set_wheels(TURN_SPEED, DRIVE_SPEED)


def rotate_left():
    set_wheels(TURN_SPEED, -TURN_SPEED)


def rotate_right():
    set_wheels(-TURN_SPEED, TURN_SPEED)


command_handlers = {
    "forward": move_forward,
    "backward": move_backward,
    "turn_left": turn_left,
    "turn_right": turn_right,
    "rotate_left": rotate_left,
    "rotate_right": rotate_right,
    "stop": stop,
}


def apply_command_code(code):
    global last_applied_command
    action = COMMANDS.get(code)
    if action is None:
        print(f"[MQTT] Unknown command code: {code}")
        return
    if code == last_applied_command and code != "SS":
        return
    last_applied_command = code
    print(f"[NAV] {ROBOT_MAC}: {code} -> {action}")
    command_handlers[action]()


def handle_movement_command(raw_payload):
    code = parse_bracket_payload(raw_payload).upper()
    with state_lock:
        navigator.enter_manual_mode()
    apply_command_code(code)


def handle_goal_payload(raw_payload):
    goal = parse_goal_payload(raw_payload)
    if goal is None:
        print(f"[MQTT] Ignored invalid goal payload: {raw_payload}")
        return

    with state_lock:
        if goal["action"] == GOAL_ACTION_CLEAR:
            navigator.clear_goal()
            print(f"[GOAL] Cleared goal for {ROBOT_MAC}")
            return

        navigator.set_goal(
            goal["target_x"],
            goal["target_y"],
            tolerance=goal.get("tolerance"),
            seq=goal.get("seq", 0),
        )
        print(
            f"[GOAL] {ROBOT_MAC} -> ({goal['target_x']}, {goal['target_y']}) "
            f"tol={goal.get('tolerance')} seq={goal.get('seq', 0)}"
        )


def handle_config_payload(raw_payload):
    global ROBOT_ARUCO_ID
    config = parse_config_payload(raw_payload)
    if config is None:
        return
    ROBOT_ARUCO_ID = config["aruco_id"]
    navigator.set_aruco_id(ROBOT_ARUCO_ID)
    print(f"[CONFIG] {ROBOT_MAC} assigned ArUco ID {ROBOT_ARUCO_ID}")


def handle_position_payload(raw_payload):
    position = parse_position_payload(raw_payload)
    if position is None:
        return
    with state_lock:
        if position.get("kind") == MARKER_TYPE_CORNER:
            if is_physical_corner_aruco(position["aruco_id"]):
                _apply_physical_corner(
                    position["aruco_id"],
                    position["x"],
                    position["y"],
                )
            else:
                navigator.update_field_corner(
                    position["aruco_id"],
                    position["x"],
                    position["y"],
                )
            return
        if USE_SUPERVISOR_FLEET and is_physical_robot_aruco(position["aruco_id"]):
            return
        if not USE_SUPERVISOR_FLEET:
            navigator.update_fleet_position(
                position["aruco_id"],
                position["x"],
                position["y"],
            )
        if ROBOT_ARUCO_ID is not None and position["aruco_id"] == ROBOT_ARUCO_ID:
            if not USE_SUPERVISOR_FLEET:
                navigator.update_position(
                    position["x"],
                    position["y"],
                    position["orientation"],
                )


def _clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def _world_to_camera_pixels(world_x, world_y):
    x = round(((world_x + HALF_FIELD) / FIELD_SIZE_M) * CAMERA_WIDTH)
    y = round(((HALF_FIELD - world_y) / FIELD_SIZE_M) * CAMERA_HEIGHT)
    return _clamp(x, 0, CAMERA_WIDTH), _clamp(y, 0, CAMERA_HEIGHT)


def _physical_pixel_to_sim_pixel(px, py):
    world_x, world_y = physical_field_mapper.pixel_to_world(
        px,
        py,
        world_min=WORLD_MIN,
        world_max=WORLD_MAX,
    )
    return _world_to_camera_pixels(world_x, world_y)


def _apply_physical_corner(aruco_id, pixel_x, pixel_y):
    global physical_field_ready_logged
    physical_field_mapper.update_corner(aruco_id, pixel_x, pixel_y)
    sim_x, sim_y = _physical_pixel_to_sim_pixel(pixel_x, pixel_y)
    navigator.update_field_corner(aruco_id, sim_x, sim_y)
    if physical_field_mapper.ready and not physical_field_ready_logged:
        physical_field_ready_logged = True
        print(f"[Robot] Physical field corners loaded for {ROBOT_MAC}")


def _camera_orientation_degrees(node):
    orientation = node.getOrientation()
    forward_x = orientation[0]
    forward_y = orientation[3]
    world_yaw = math.degrees(math.atan2(forward_y, forward_x))
    return round((-world_yaw) % 360, 1)


def sync_fleet_from_supervisor():
    """Read live robot poses from Webots (no MQTT latency)."""
    children = robot.getRoot().getField("children")
    for index in range(children.getCount()):
        node = children.getMFNode(index)
        if is_disabled_robot(node):
            continue
        aruco_id = resolve_aruco_id(node)
        if aruco_id is None:
            continue

        if is_physical_corner_aruco(aruco_id):
            continue

        world_x, world_y, _ = node.getPosition()
        pixel_x, pixel_y = _world_to_camera_pixels(world_x, world_y)

        if is_corner_aruco(aruco_id):
            if physical_field_mapper.ready:
                continue
            navigator.update_field_corner(aruco_id, pixel_x, pixel_y)
            continue

        orientation = _camera_orientation_degrees(node)

        navigator.update_fleet_position(aruco_id, pixel_x, pixel_y)
        is_self = (
            (ROBOT_ARUCO_ID is not None and aruco_id == ROBOT_ARUCO_ID)
            or node == self_node
        )
        if is_self:
            navigator.update_position(pixel_x, pixel_y, orientation)
            if ROBOT_ARUCO_ID is None:
                navigator.set_aruco_id(aruco_id)


def publish_report(status, seq=0):
    if client is None:
        return
    with state_lock:
        x = navigator.x
        y = navigator.y
        seq = seq or navigator.seq
    payload = format_report_payload(status, seq=seq, x=x, y=y)
    client.publish(report_topic, payload)
    print(f"[REPORT] {ROBOT_MAC}: {payload}")


def subscribe_navigation_topics():
    client.subscribe(TOPIC_DATA_POSITIONS_SIMULATION)
    client.subscribe(TOPIC_DATA_POSITIONS_PHYSICAL)
    client.subscribe(goals_topic)
    client.subscribe(config_topic)
    print(
        f"[MQTT] Subscribed to {TOPIC_DATA_POSITIONS_SIMULATION}, "
        f"{TOPIC_DATA_POSITIONS_PHYSICAL}, {goals_topic}, {config_topic}"
    )


def connecting_payload():
    if ROBOT_ARUCO_ID is not None:
        return bracket_payload(f"{ROBOT_MAC}, {ROBOT_ARUCO_ID}")
    return bracket_payload(ROBOT_MAC)


def on_connect(client, userdata, flags, reason_code, properties):
    print(f"[MQTT] Connected ({reason_code}) as {ROBOT_MAC}")
    client.subscribe(status_topic)
    payload = connecting_payload()
    client.publish(TOPIC_CONTROL_CONNECTING, payload)
    print(f"[MQTT] Handshake request -> {TOPIC_CONTROL_CONNECTING}: {payload}")


def on_disconnect(client, userdata, disconnect_flags, reason_code, properties):
    global connection_state, last_applied_command
    print(f"[MQTT] Disconnected ({reason_code})")
    connection_state = "connecting"
    set_led_status("connecting")
    with state_lock:
        navigator.clear_goal()
    last_applied_command = None
    stop()


def on_message(client, userdata, msg):
    global connection_state

    payload = msg.payload.decode().strip()
    print(f"[MQTT] {msg.topic}: {payload}")

    if msg.topic == status_topic:
        status = parse_bracket_payload(payload).lower()
        if status == STATUS_CHECKING:
            connection_state = "connecting"
            set_led_status("connecting")
        elif status == STATUS_CONNECTED:
            connection_state = "connected"
            set_led_status("connected")
            client.subscribe(commands_topic)
            subscribe_navigation_topics()
            print(f"[MQTT] Listening on {commands_topic}")
        elif status == STATUS_DISCONNECTED:
            connection_state = "connecting"
            set_led_status("connecting")
            with state_lock:
                navigator.clear_goal()
            stop()
            client.unsubscribe(commands_topic)
            client.unsubscribe(goals_topic)
            client.unsubscribe(config_topic)
            client.unsubscribe(TOPIC_DATA_POSITIONS_SIMULATION)
            client.unsubscribe(TOPIC_DATA_POSITIONS_PHYSICAL)
        return

    if connection_state != "connected":
        return

    if msg.topic == commands_topic:
        handle_movement_command(payload)
    elif msg.topic == goals_topic:
        handle_goal_payload(payload)
    elif msg.topic == config_topic:
        handle_config_payload(payload)
    elif msg.topic in (TOPIC_DATA_POSITIONS_SIMULATION, TOPIC_DATA_POSITIONS_PHYSICAL):
        handle_position_payload(payload)


client = mqtt.Client(
    protocol=mqtt.MQTTv311,
    callback_api_version=CallbackAPIVersion.VERSION2,
    client_id=f"webots_robot_{ROBOT_MAC.replace(':', '')}_{random.randint(0, 9999)}",
)
client.on_connect = on_connect
client.on_disconnect = on_disconnect
client.on_message = on_message
client.username_pw_set(MQTT_USER, MQTT_PASS)

set_led_status("connecting")


def connection_delay_seconds(mac):
    try:
        suffix = int(mac.rsplit(":", 1)[1], 16)
    except (IndexError, ValueError):
        return 0.0

    first_suffix = int(os.getenv("WEBOTS_FIRST_MAC_SUFFIX", "21"), 16)
    delay_step = float(os.getenv("WEBOTS_HANDSHAKE_DELAY_STEP", "0.4"))
    return max(0.0, suffix - first_suffix) * delay_step


try:
    delay = connection_delay_seconds(ROBOT_MAC)
    if delay:
        print(f"[MQTT] Waiting {delay:.1f}s before handshake for deterministic ArUco mapping")
        time.sleep(delay)
    client.connect(BROKER, PORT)
except Exception as exc:
    print(f"[MQTT] Failed to connect to {BROKER}:{PORT}: {exc}")
else:
    threading.Thread(target=client.loop_forever, daemon=True).start()

while robot.step(timestep) != -1:
    if connection_state != "connected":
        continue

    with state_lock:
        if USE_SUPERVISOR_FLEET:
            sync_fleet_from_supervisor()
        result = navigator.tick()

    if result is None:
        continue

    if len(result) == 3:
        kind, value, seq = result
    else:
        kind, value = result
        seq = navigator.seq

    if kind == "command":
        apply_command_code(value)
    elif kind == "report":
        publish_report(value, seq=seq)
        apply_command_code("SS")

set_led_status("off")
stop()
client.disconnect()
