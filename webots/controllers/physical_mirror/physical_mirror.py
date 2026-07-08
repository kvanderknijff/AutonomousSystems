"""Mirror physical robot poses from the overhead camera into the Webots world."""

import math
import os
import random
import sys

import paho.mqtt.client as mqtt
from controller import Supervisor
from paho.mqtt.client import CallbackAPIVersion

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from coordinate_mapper import PhysicalFieldMapper  # noqa: E402
from mqtt_protocol import (  # noqa: E402
    BROKER,
    MARKER_TYPE_CORNER,
    MQTT_PASS,
    MQTT_USER,
    PHYSICAL_ROBOT_ARUCO_FIRST,
    PHYSICAL_ROBOT_ARUCO_LAST,
    PORT,
    TOPIC_DATA_POSITIONS_PHYSICAL,
    TOPIC_DATA_POSITIONS_WILD,
    is_physical_robot_aruco,
    parse_position_payload,
)
from webots_nodes import is_physical_proxy, resolve_aruco_id  # noqa: E402


supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

FIELD_SIZE_M = float(os.getenv("WEBOTS_FIELD_SIZE_M", "1.0"))
CORNER_INSET_M = float(os.getenv("WEBOTS_CORNER_INSET_M", "0.035"))
HALF_FIELD = FIELD_SIZE_M / 2.0
WORLD_MIN = -HALF_FIELD + CORNER_INSET_M
WORLD_MAX = HALF_FIELD - CORNER_INSET_M
STALE_MS = int(os.getenv("WEBOTS_MIRROR_STALE_MS", "1500"))
HIDDEN_Y = float(os.getenv("WEBOTS_PROXY_HIDDEN_Y", "1.5"))
HIDDEN_Z = float(os.getenv("WEBOTS_PROXY_HIDDEN_Z", "-0.2"))
HIDDEN_SPACING = float(os.getenv("WEBOTS_PROXY_HIDDEN_SPACING", "0.08"))
PROXY_VISIBLE_Z = float(os.getenv("WEBOTS_PROXY_Z", "0.037"))

SHOW_CORNERS = os.getenv("WEBOTS_SHOW_CORNERS", "true").lower() not in ("0", "false", "no")
CORNER_MARKER_RADIUS = float(os.getenv("WEBOTS_CORNER_MARKER_RADIUS", "0.04"))
CORNER_MARKER_DIR = os.getenv("WEBOTS_MARKER_DIR", "../markers")
CORNER_MARKER_Z = float(os.getenv("WEBOTS_CORNER_MARKER_Z", "0.02"))

mapper = PhysicalFieldMapper()
proxy_nodes: dict[int, object] = {}
corner_nodes: dict[int, object] = {}
proxy_led_fields: dict[int, dict] = {}
last_pose: dict[int, tuple[float, float, float, str, int]] = {}
field_ready_logged = False

# Colours used to reflect the physical robot's LED status on the Webots proxy.
LED_ON_COLOR = {
    "led_connecting": [0.0, 0.25, 1.0],
    "led_connected": [0.0, 1.0, 0.0],
}
LED_OFF_COLOR = [0.02, 0.02, 0.02]


def discover_proxy_nodes() -> dict[int, object]:
    nodes: dict[int, object] = {}
    children = supervisor.getRoot().getField("children")
    for index in range(children.getCount()):
        node = children.getMFNode(index)
        if not is_physical_proxy(node):
            continue
        aruco_id = resolve_aruco_id(node)
        if aruco_id is None:
            continue
        nodes[aruco_id] = node
    return nodes


def camera_degrees_to_world_rotation(orientation_deg: float) -> float:
    """Convert overhead-camera yaw to Webots Z rotation (radians)."""
    return math.radians((-float(orientation_deg) + 90.0) % 360.0)


def apply_pose(node, world_x: float, world_y: float, orientation_deg: float) -> None:
    translation = node.getField("translation")
    rotation = node.getField("rotation")
    if translation is not None:
        translation.setSFVec3f([world_x, world_y, PROXY_VISIBLE_Z])
    if rotation is not None:
        rotation.setSFRotation([0, 0, 1, camera_degrees_to_world_rotation(orientation_deg)])
    node.resetPhysics()


def hide_proxy(node, aruco_id: int) -> None:
    """Park proxy off the field until the physical camera reports a live pose."""
    translation = node.getField("translation")
    if translation is not None:
        index = aruco_id - PHYSICAL_ROBOT_ARUCO_FIRST
        hidden_x = (index - 1.5) * HIDDEN_SPACING
        translation.setSFVec3f([hidden_x, HIDDEN_Y, HIDDEN_Z])
        node.resetPhysics()
    apply_led_status(aruco_id, node, "off")


def _resolve_led_fields(node) -> dict:
    """Grab the proxy's exposed LED colour fields (writable per instance)."""
    return {
        "led_connecting": node.getField("ledConnectingColor"),
        "led_connected": node.getField("ledConnectedColor"),
    }


def apply_led_status(aruco_id: int, node, status: str) -> None:
    """Reflect the physical robot's LED status on the Webots proxy LEDs."""
    fields = proxy_led_fields.get(aruco_id)
    if fields is None:
        fields = _resolve_led_fields(node)
        proxy_led_fields[aruco_id] = fields
    if not fields:
        return

    for key, field in fields.items():
        if field is None:
            continue
        lit = (
            (key == "led_connecting" and status == "connecting")
            or (key == "led_connected" and status == "connected")
        )
        field.setSFColor(LED_ON_COLOR[key] if lit else LED_OFF_COLOR)


def spawn_corner_node(aruco_id: int):
    """Create a flat plate showing the real ArUco marker at one field corner (5-8)."""
    children = supervisor.getRoot().getField("children")
    index = children.getCount()
    side = 2.0 * CORNER_MARKER_RADIUS
    node_string = (
        f"DEF CORNER_MARKER_{aruco_id} Solid {{ "
        f"translation 0 {HIDDEN_Y} {HIDDEN_Z} "
        f'name "corner_marker_{aruco_id}" '
        f"children [ Shape {{ "
        f"appearance PBRAppearance {{ baseColor 1 1 1 "
        f'baseColorMap ImageTexture {{ url [ "{CORNER_MARKER_DIR}/marker{aruco_id}.png" ] }} '
        f"roughness 0.5 metalness 0 }} "
        f"geometry Box {{ size {side} {side} 0.012 }} "
        f"}} ] }}"
    )
    children.importMFNodeFromString(index, node_string)
    return children.getMFNode(index)


def update_corner_markers() -> None:
    """Place a marker at each reported field corner; spawn on first sighting."""
    if not (SHOW_CORNERS and mapper.ready):
        return
    for aruco_id, (pixel_x, pixel_y) in list(mapper.physical_corners.items()):
        node = corner_nodes.get(aruco_id)
        if node is None:
            node = spawn_corner_node(aruco_id)
            corner_nodes[aruco_id] = node
        world_x, world_y = mapper.pixel_to_world(
            pixel_x,
            pixel_y,
            world_min=WORLD_MIN,
            world_max=WORLD_MAX,
        )
        translation = node.getField("translation")
        if translation is not None:
            translation.setSFVec3f([world_x, world_y, CORNER_MARKER_Z])


client = mqtt.Client(
    protocol=mqtt.MQTTv311,
    callback_api_version=CallbackAPIVersion.VERSION2,
    client_id=f"webots_mirror_{random.randint(0, 9999)}",
)
client.username_pw_set(MQTT_USER, MQTT_PASS)


def on_connect(client, userdata, flags, reason_code, properties):
    print(f"[Mirror] MQTT connected ({reason_code}) -> {BROKER}:{PORT}")
    client.subscribe(TOPIC_DATA_POSITIONS_PHYSICAL)
    client.subscribe(TOPIC_DATA_POSITIONS_WILD)
    print(f"[Mirror] Subscribed to {TOPIC_DATA_POSITIONS_PHYSICAL}")


def on_message(client, userdata, msg):
    payload = msg.payload.decode().strip()
    if not (
        msg.topic == TOPIC_DATA_POSITIONS_PHYSICAL
        or msg.topic.endswith("/Physical")
        or msg.topic == "Robots/Data/Positions"
    ):
        return

    position = parse_position_payload(payload)
    if position is None:
        return

    if position.get("kind") == MARKER_TYPE_CORNER:
        mapper.update_corner(position["aruco_id"], position["x"], position["y"])
        global field_ready_logged
        if mapper.ready and not field_ready_logged:
            field_ready_logged = True
            print(f"[Mirror] Physical field corners loaded ({len(mapper.physical_corners)} markers)")
        return

    aruco_id = int(position["aruco_id"])
    if not is_physical_robot_aruco(aruco_id):
        return

    now_ms = int(supervisor.getTime() * 1000)
    last_pose[aruco_id] = (
        float(position["x"]),
        float(position["y"]),
        float(position["orientation"]),
        str(position.get("led_status", "off")),
        now_ms,
    )


client.on_connect = on_connect
client.on_message = on_message

try:
    client.connect(BROKER, PORT)
    client.loop_start()
except Exception as exc:
    print(f"[Mirror] MQTT connection failed: {exc}")

proxy_nodes = discover_proxy_nodes()
print(
    f"[Mirror] Tracking physical proxies ArUco "
    f"{PHYSICAL_ROBOT_ARUCO_FIRST}-{PHYSICAL_ROBOT_ARUCO_LAST}: "
    f"{sorted(proxy_nodes)}"
)
if not proxy_nodes:
    print(
        "[Mirror] WARNING: no physical_proxy_* nodes found in the world. "
        "Add FormationBot proxies with role=physical_proxy."
    )
else:
    for aruco_id, node in proxy_nodes.items():
        hide_proxy(node, aruco_id)
    print("[Mirror] Physical proxies hidden until camera telemetry arrives")

while supervisor.step(timestep) != -1:
    if not proxy_nodes:
        proxy_nodes = discover_proxy_nodes()
        for aruco_id, node in proxy_nodes.items():
            hide_proxy(node, aruco_id)

    update_corner_markers()

    now_ms = int(supervisor.getTime() * 1000)
    for aruco_id, node in proxy_nodes.items():
        pose = last_pose.get(aruco_id)
        if pose is None:
            hide_proxy(node, aruco_id)
            continue
        pixel_x, pixel_y, orientation, led_status, updated_ms = pose
        if now_ms - updated_ms > STALE_MS or not mapper.ready:
            hide_proxy(node, aruco_id)
            continue

        world_x, world_y = mapper.pixel_to_world(
            pixel_x,
            pixel_y,
            world_min=WORLD_MIN,
            world_max=WORLD_MAX,
        )
        apply_pose(node, world_x, world_y, orientation)
        apply_led_status(aruco_id, node, led_status)

client.loop_stop()
client.disconnect()
