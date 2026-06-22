import logging
import threading
import time
from typing import Callable, Dict

import paho.mqtt.client as mqtt
from paho.mqtt.client import CallbackAPIVersion

from config import (
    MQTT_BROKER,
    MQTT_PORT,
    MQTT_USER,
    MQTT_PASS,
    MQTT_CLIENT_ID,
    POSITION_HISTORY_INTERVAL
)
from database import Database
from mqtt_protocol import (
    TOPIC_CONTROL_CONNECTING,
    TOPIC_DATA_POSITIONS,
    STATUS_CHECKING,
    STATUS_CONNECTED,
    STATUS_DISCONNECTED,
    LED_CONNECTING,
    COMMANDS,
    bracket_payload,
    parse_bracket_payload,
    parse_position_payload,
    control_status_topic,
    data_commands_topic
)
from registry import RobotConnectionState, RobotRegistry

logger = logging.getLogger(__name__)

class CentralServer:
    def __init__(self, client_id: str = MQTT_CLIENT_ID):
        self.db = Database()
        self.registry = RobotRegistry(db=self.db)
        self.registry.load_from_database()

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

    def stop_robot(self, mac: str) -> bool:
        return self.send_command(mac, "SS")

    def set_position_listener(
        self,
        listener: Callable[[str, float, float, float], None] | None,
    ) -> None:
        """Register callback fired after each robot position update (mac, x, y, orientation)."""
        self._position_listener = listener

    def _notify_position_listener(
        self,
        mac: str,
        x: float,
        y: float,
        orientation: float,
    ) -> None:
        if self._position_listener is not None:
            self._position_listener(mac, x, y, orientation)

    # --- MQTT handlers ---

    def _on_connect(self, client, userdata, flags, reason_code, properties):
        if getattr(reason_code, "is_failure", False) or str(reason_code) != "Success":
            logger.error("MQTT connection failed: %s (check MQTT_USER / MQTT_PASS)", reason_code)
            return
        logger.info("MQTT connected (%s)", reason_code)
        client.subscribe(TOPIC_CONTROL_CONNECTING)
        client.subscribe(TOPIC_DATA_POSITIONS)

    def _on_message(self, client, userdata, msg):
        payload = msg.payload.decode().strip()
        if msg.topic == TOPIC_CONTROL_CONNECTING:
            self._handle_connecting(payload)
        elif msg.topic == TOPIC_DATA_POSITIONS:
            self._handle_position(payload)
    
    def _handle_connecting(self, payload: str) -> None:
        mac = parse_bracket_payload(payload)
        if not mac:
            return

        with self._lock:
            self.registry.register_connection_request(mac)

        topic = control_status_topic(mac)
        self._client.publish(topic, bracket_payload(STATUS_CHECKING))
        self.db.log_event("checking", mac=mac)
        logger.info("%s -> [checking]", mac)

    def _handle_position(self, payload: str) -> None:
        position = parse_position_payload(payload)
        if position is None:
            return

        aruco_id = position["aruco_id"]
        led_status = position["led_status"]
        x = float(position["x"])
        y = float(position["y"])
        orientation = float(position["orientation"])
        notify_mac: str | None = None
        notify_pose: tuple[float, float, float] | None = None
        publish_connected_mac: str | None = None

        with self._lock:
            mac = self.registry.get_mac_for_aruco(aruco_id)

            if mac is not None:
                record, went_offline = self.registry.update_position(
                    aruco_id,
                    position["x"],
                    position["y"],
                    position["orientation"],
                    led_status,
                )
                if record is not None:
                    self._maybe_record_history(record.mac, record)
                    if not went_offline:
                        notify_mac = mac
                        notify_pose = (x, y, orientation)
                if went_offline and record is not None:
                    self._publish_status_unlocked(mac, STATUS_DISCONNECTED)
                    self._publish_command_unlocked(mac, "SS")
                    logger.warning("%s offline - disconnected status and stop sent", mac)
            elif led_status == LED_CONNECTING:
                pending_mac = self.registry.next_pending_mac()
                if pending_mac is not None and self.registry.complete_handshake(pending_mac, aruco_id):
                    self.registry.update_position(
                        aruco_id,
                        position["x"],
                        position["y"],
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
