import logging
import threading
import time
from typing import Callable, Dict

import paho.mqtt.client as mqtt
from paho.mqtt.client import CallbackAPIVersion

from coordinate_mapper import CoordinateMapper
from config import (
    MQTT_BROKER,
    MQTT_PORT,
    MQTT_USER,
    MQTT_PASS,
    MQTT_CLIENT_ID,
    POSITION_HISTORY_INTERVAL,
)
from database import Database
from field_bounds import FieldBounds
from mqtt_protocol import (
    TOPIC_CONTROL_CONNECTING,
    TOPIC_DATA_POSITIONS,
    TOPIC_DATA_POSITIONS_PHYSICAL,
    TOPIC_DATA_POSITIONS_SIMULATION,
    TOPIC_DATA_POSITIONS_WILD,
    TOPIC_DATA_REPORT_WILD,
    POSITION_SOURCE_LEGACY,
    POSITION_SOURCE_PHYSICAL,
    POSITION_SOURCE_SIMULATION,
    STATUS_CHECKING,
    STATUS_CONNECTED,
    STATUS_DISCONNECTED,
    LED_CONNECTING,
    LED_CONNECTED,
    MARKER_TYPE_CORNER,
    COMMANDS,
    GOAL_ACTION_CLEAR,
    bracket_payload,
    parse_bracket_payload,
    parse_connecting_payload,
    parse_position_payload,
    parse_report_payload,
    position_source_for_topic,
    is_physical_robot_aruco,
    control_status_topic,
    data_commands_topic,
    data_config_topic,
    data_goals_topic,
    format_config_payload,
    format_goal_payload,
)
from registry import RobotConnectionState, RobotRegistry

logger = logging.getLogger(__name__)

class CentralServer:
    def __init__(self, client_id: str = MQTT_CLIENT_ID):
        self.db = Database()
        cleared = self.db.clear_all()
        if cleared:
            logger.info("Server restart: cleared %d robot(s) from database", cleared)

        self.registry = RobotRegistry(db=self.db)
        self.registry.load_from_database()
        self.field_bounds = FieldBounds()
        self.coordinate_mapper = CoordinateMapper()
        self._last_physical_update: dict[int, float] = {}

        self._last_history_time: Dict[str, float] = {}
        self._client = mqtt.Client(
            protocol=mqtt.MQTTv311,
            callback_api_version=CallbackAPIVersion.VERSION2,
            client_id=client_id,
        )
        self._client.on_connect = self._on_connect
        self._client.on_message = self._on_message
        self._client.username_pw_set(MQTT_USER, MQTT_PASS)
        self._lock = threading.Lock()
        self._started = False
        self._position_listener: Callable[[str, float, float, float], None] | None = None
        self._report_listener: Callable[[str, dict], None] | None = None
        self._field_bounds_listener: Callable[[], None] | None = None
    
    def start(self, blocking: bool = False) -> None:
        if self._started:
            return
        self._client.connect(MQTT_BROKER, MQTT_PORT)
        self._client.loop_start()
        self._started = True
        logger.info("Central server connected to MQTT %s:%s", MQTT_BROKER, MQTT_PORT)
        logger.info("Database: %s", self.db.db_path)
        if blocking:
            try:
                threading.Event().wait()
            except KeyboardInterrupt:
                self.stop()
    
    def stop(self) -> None:
        if not self._started:
            return
        self._client.loop_stop()
        self._client.disconnect()
        self._started = False
        logger.info("Central server stopped")

    # --- API for path planner ---

    def get_positions(self) -> Dict[str, dict]:
        with self._lock:
            return self.registry.all_positions()

    def get_field_bounds(self) -> FieldBounds:
        with self._lock:
            return self.field_bounds

    def get_connected_positions(self) -> Dict[str, dict]:
        with self._lock:
            return {r.mac: r.position_dict() for r in self.registry.connected_robots()}

    def is_commandable(self, mac: str) -> bool:
        with self._lock:
            record = self.registry.get(mac)
            return record is not None and record.state == RobotConnectionState.CONNECTED

    def send_command(self, mac: str, command: str) -> bool:
        return self.publish(mac, command, "command")

    def send_status(self, mac: str, status: str) -> bool:
        return self.publish(mac, status, "status")

    def publish(self, mac: str, payload: str, topic_type: str) -> bool:
        """Route payload to the Status or Commands topic (Kai's topic_type pattern)."""
        if topic_type == "status":
            topic = control_status_topic(mac)
            mqtt_payload = bracket_payload(payload.strip().lower())
            event_type = "status"
        else:
            command = payload.strip().upper()
            if command not in COMMANDS:
                logger.warning("Unknown command: %s", command)
                return False
            with self._lock:
                record = self.registry.get(mac)
                if record is None or record.state != RobotConnectionState.CONNECTED:
                    logger.warning("Cannot send [%s] to %s — not connected", command, mac)
                    return False
            topic = data_commands_topic(mac)
            mqtt_payload = bracket_payload(command)
            event_type = "command"
            payload = command

        self._client.publish(topic, mqtt_payload)
        self.db.log_event(event_type, mac=mac, payload=payload)
        logger.info("Route [%s] -> %s to %s", topic_type, mqtt_payload, topic)
        return True

    def publish_goal(self, mac: str, goal: dict) -> bool:
        with self._lock:
            record = self.registry.get(mac)
            if record is None or record.state != RobotConnectionState.CONNECTED:
                logger.warning("Cannot send goal to %s — not connected", mac)
                return False

        action = str(goal.get("action", "set")).lower()
        if action == GOAL_ACTION_CLEAR:
            mqtt_payload = format_goal_payload(0, 0, action=GOAL_ACTION_CLEAR, seq=int(goal.get("seq", 0)))
        else:
            target_x = float(goal["target_x"])
            target_y = float(goal["target_y"])
            with self._lock:
                aruco_id = record.aruco_id
            if aruco_id is not None:
                target_x, target_y = self.coordinate_mapper.to_native_for_aruco(
                    target_x,
                    target_y,
                    aruco_id,
                )
            mqtt_payload = format_goal_payload(
                target_x,
                target_y,
                tolerance=float(goal.get("tolerance", 12)),
                seq=int(goal.get("seq", 0)),
            )

        topic = data_goals_topic(mac)
        self._client.publish(topic, mqtt_payload)
        self.db.log_event("goal", mac=mac, payload=mqtt_payload)
        logger.info("Route [goal] -> %s to %s", mqtt_payload, topic)
        return True

    def publish_robot_config(self, mac: str, aruco_id: int) -> None:
        topic = data_config_topic(mac)
        payload = format_config_payload(aruco_id)
        self._client.publish(topic, payload)
        self.db.log_event("config", mac=mac, payload=payload)
        logger.info("Route [config] -> %s to %s", payload, topic)

    def stop_robot(self, mac: str) -> bool:
        return self.send_command(mac, "SS")

    def set_position_listener(
        self,
        listener: Callable[[str, float, float, float], None] | None,
    ) -> None:
        """Register callback fired after each robot position update (mac, x, y, orientation)."""
        self._position_listener = listener

    def set_report_listener(
        self,
        listener: Callable[[str, dict], None] | None,
    ) -> None:
        self._report_listener = listener

    def set_field_bounds_listener(
        self,
        listener: Callable[[], None] | None,
    ) -> None:
        """Register callback fired when a corner marker updates field bounds."""
        self._field_bounds_listener = listener

    def _notify_report_listener(self, mac: str, report: dict) -> None:
        if self._report_listener is not None:
            self._report_listener(mac, report)

    def _notify_position_listener(
        self,
        mac: str,
        x: float,
        y: float,
        orientation: float,
    ) -> None:
        if self._position_listener is not None:
            self._position_listener(mac, x, y, orientation)

    def _notify_field_bounds_listener(self) -> None:
        if self._field_bounds_listener is not None:
            self._field_bounds_listener()

    # --- MQTT handlers ---

    def _on_connect(self, client, userdata, flags, reason_code, properties):
        if getattr(reason_code, "is_failure", False) or str(reason_code) != "Success":
            logger.error("MQTT connection failed: %s (check MQTT_USER / MQTT_PASS)", reason_code)
            return
        logger.info("MQTT connected (%s)", reason_code)
        client.subscribe(TOPIC_CONTROL_CONNECTING)
        client.subscribe(TOPIC_DATA_POSITIONS_WILD)
        client.subscribe(TOPIC_DATA_REPORT_WILD)

    def _mac_from_report_topic(self, topic: str) -> str | None:
        prefix = "Robots/Data/"
        suffix = "/Report"
        if not topic.startswith(prefix) or not topic.endswith(suffix):
            return None
        mac = topic[len(prefix) : -len(suffix)]
        return mac or None

    def _is_position_topic(self, topic: str) -> bool:
        return (
            topic == TOPIC_DATA_POSITIONS
            or topic == TOPIC_DATA_POSITIONS_PHYSICAL
            or topic == TOPIC_DATA_POSITIONS_SIMULATION
        )

    def _should_ignore_simulation_robot(
        self,
        aruco_id: int,
        source: str,
    ) -> bool:
        """Prefer the physical camera for robots that exist on the real field."""
        if source != POSITION_SOURCE_SIMULATION:
            return False
        if not is_physical_robot_aruco(aruco_id):
            return False
        last_physical = self._last_physical_update.get(aruco_id, 0.0)
        return time.time() - last_physical < 1.0

    def _on_message(self, client, userdata, msg):
        payload = msg.payload.decode().strip()
        if msg.topic == TOPIC_CONTROL_CONNECTING:
            self._handle_connecting(payload)
        elif self._is_position_topic(msg.topic):
            self._handle_position(payload, msg.topic)
        elif msg.topic.endswith("/Report"):
            self._handle_report(msg.topic, payload)

    def _handle_report(self, topic: str, payload: str) -> None:
        mac = self._mac_from_report_topic(topic)
        report = parse_report_payload(payload)
        if mac is None or report is None:
            logger.debug("Ignored robot report on %s: %s", topic, payload[:200])
            return
        self.db.log_event("robot_report", mac=mac, payload=payload)
        logger.info("Robot report from %s: %s", mac, report)
        self._notify_report_listener(mac, report)
    
    def _handle_connecting(self, payload: str) -> None:
        mac, fixed_aruco_id = parse_connecting_payload(payload)
        if not mac:
            return

        publish_connected = False
        was_connected = False
        with self._lock:
            _, was_connected = self.registry.register_connection_request(mac)
            if fixed_aruco_id is not None:
                publish_connected = self.registry.complete_handshake(mac, fixed_aruco_id)

        topic = control_status_topic(mac)
        if not publish_connected:
            if was_connected:
                self._client.publish(topic, bracket_payload(STATUS_DISCONNECTED))
                self.db.log_event("status", mac=mac, payload=STATUS_DISCONNECTED)
                logger.info("%s -> [disconnected] (handshake restart)", mac)
            self._client.publish(topic, bracket_payload(STATUS_CHECKING))
            self.db.log_event("checking", mac=mac)
            logger.info("%s -> [checking]", mac)
        else:
            self._client.publish(topic, bracket_payload(STATUS_CONNECTED))
            logger.info("ArUco %s <-> %s -> [connected] (handshake)", fixed_aruco_id, mac)
            self.publish_robot_config(mac, fixed_aruco_id)

    def _handle_position(self, payload: str, topic: str) -> None:
        position = parse_position_payload(payload)
        if position is None:
            logger.debug("Ignored unparseable position payload: %s", payload[:200])
            return

        source = position_source_for_topic(topic)

        if position.get("kind") == MARKER_TYPE_CORNER:
            with self._lock:
                self.coordinate_mapper.update_corner(
                    position["aruco_id"],
                    position["x"],
                    position["y"],
                )
                if source in (POSITION_SOURCE_PHYSICAL, POSITION_SOURCE_LEGACY):
                    self.field_bounds.update_corner(
                        position["aruco_id"],
                        position["x"],
                        position["y"],
                    )
            logger.debug(
                "Field corner %s @ (%s, %s) [%s]",
                position["aruco_id"],
                position["x"],
                position["y"],
                source,
            )
            self._notify_field_bounds_listener()
            return

        aruco_id = position["aruco_id"]
        if self._should_ignore_simulation_robot(aruco_id, source):
            return

        led_status = position["led_status"]
        with self._lock:
            canonical_x, canonical_y = self.coordinate_mapper.to_canonical(
                position["x"],
                position["y"],
                source=source,
            )
        x = float(canonical_x)
        y = float(canonical_y)
        orientation = float(position["orientation"])

        if source == POSITION_SOURCE_PHYSICAL and is_physical_robot_aruco(aruco_id):
            self._last_physical_update[aruco_id] = time.time()
        notify_mac: str | None = None
        notify_pose: tuple[float, float, float] | None = None
        publish_connected_mac: str | None = None

        with self._lock:
            mac = self.registry.get_mac_for_aruco(aruco_id)

            if mac is not None:
                record, went_offline = self.registry.update_position(
                    aruco_id,
                    int(round(canonical_x)),
                    int(round(canonical_y)),
                    position["orientation"],
                    led_status,
                )
                if record is not None:
                    self._maybe_record_history(record.mac, record)
                    if not went_offline:
                        notify_mac = mac
                        notify_pose = (x, y, orientation)
                if went_offline and record is not None:
                    # self._publish_status_unlocked(mac, STATUS_DISCONNECTED)
                    # self._publish_command_unlocked(mac, "SS")
                    logger.warning("%s offline - disconnected status and stop sent", mac)
            elif led_status in (LED_CONNECTING, LED_CONNECTED):
                pending_mac = self.registry.next_pending_mac()
                if pending_mac is not None and self.registry.complete_handshake(pending_mac, aruco_id):
                    self.registry.update_position(
                        aruco_id,
                        int(round(canonical_x)),
                        int(round(canonical_y)),
                        position["orientation"],
                        led_status,
                    )
                    record = self.registry.get(pending_mac)
                    if record:
                        self._maybe_record_history(pending_mac, record)
                    notify_mac = pending_mac
                    notify_pose = (x, y, orientation)
                    publish_connected_mac = pending_mac

        if publish_connected_mac is not None:
            self._client.publish(
                control_status_topic(publish_connected_mac),
                bracket_payload(STATUS_CONNECTED),
            )
            logger.info("ArUco %s <-> %s -> [connected]", aruco_id, publish_connected_mac)
            self.publish_robot_config(publish_connected_mac, aruco_id)

        if notify_mac is not None and notify_pose is not None:
            self._notify_position_listener(notify_mac, *notify_pose)

    def _maybe_record_history(self, mac: str, record) -> None:
        if POSITION_HISTORY_INTERVAL <= 0:
            self.db.record_position_history(
                mac, record.aruco_id, record.x, record.y, record.orientation, record.led_status
            )
            return
        now = time.time()
        last = self._last_history_time.get(mac, 0)
        if now - last >= POSITION_HISTORY_INTERVAL:
            self._last_history_time[mac] = now
            self.db.record_position_history(
                mac, record.aruco_id, record.x, record.y, record.orientation, record.led_status
            )

    def _publish_status_unlocked(self, mac: str, status: str) -> None:
        self._client.publish(control_status_topic(mac), bracket_payload(status))
        self.db.log_event("status", mac=mac, payload=status)

    def _publish_command_unlocked(self, mac: str, command: str) -> None:
        self._client.publish(data_commands_topic(mac), bracket_payload(command))
        self.db.log_event("command", mac=mac, payload=command)
