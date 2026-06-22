"""In-memory robot registry, synced with the database."""

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional

from database import Database


class RobotConnectionState(Enum):
    PENDING = "pending"
    CONNECTED = "connected"
    OFFLINE = "offline"


@dataclass
class RobotRecord:
    mac: str
    state: RobotConnectionState = RobotConnectionState.PENDING
    aruco_id: Optional[int] = None
    x: int = 0
    y: int = 0
    orientation: int = 0
    led_status: str = "off"

    def position_dict(self) -> dict:
        return {
            "mac": self.mac,
            "aruco_id": self.aruco_id,
            "x": self.x,
            "y": self.y,
            "orientation": self.orientation,
            "led_status": self.led_status,
            "state": self.state.value,
        }

    @classmethod
    def from_db_row(cls, row: dict) -> "RobotRecord":
        return cls(
            mac=row["mac"],
            state=RobotConnectionState(row["state"]),
            aruco_id=row["aruco_id"],
            x=row["x"],
            y=row["y"],
            orientation=row["orientation"],
            led_status=row["led_status"],
        )


@dataclass
class RobotRegistry:
    db: Database
    robots: Dict[str, RobotRecord] = field(default_factory=dict)
    aruco_to_mac: Dict[int, str] = field(default_factory=dict)
    pending_macs: List[str] = field(default_factory=list)

    def load_from_database(self) -> None:
        for row in self.db.load_registry_state():
            record = RobotRecord.from_db_row(row)
            self.robots[record.mac] = record
            if record.aruco_id is not None and record.state == RobotConnectionState.CONNECTED:
                self.aruco_to_mac[record.aruco_id] = record.mac

    def get(self, mac: str) -> Optional[RobotRecord]:
        return self.robots.get(mac)

    def get_mac_for_aruco(self, aruco_id: int) -> Optional[str]:
        return self.aruco_to_mac.get(aruco_id)

    def register_connection_request(self, mac: str) -> RobotRecord:
        record = self.robots.get(mac)
        if record is None:
            record = RobotRecord(mac=mac)
            self.robots[mac] = record
            self.db.upsert_robot(mac, state="pending", led_status="off")
        else:
            if record.aruco_id is not None:
                self.aruco_to_mac.pop(record.aruco_id, None)
                self.db.clear_aruco_mapping(mac)
                record.aruco_id = None
            record.state = RobotConnectionState.PENDING
            record.led_status = "off"
            self.db.set_robot_state(mac, "pending")

        if mac not in self.pending_macs:
            self.pending_macs.append(mac)
        self.db.log_event("connecting", mac=mac, payload=mac)
        return record

    def next_pending_mac(self) -> Optional[str]:
        while self.pending_macs:
            mac = self.pending_macs[0]
            record = self.robots.get(mac)
            if record and record.state == RobotConnectionState.PENDING:
                return mac
            self.pending_macs.pop(0)
        return None

    def complete_handshake(self, mac: str, aruco_id: int) -> bool:
        record = self.robots.get(mac)
        if record is None or record.state != RobotConnectionState.PENDING:
            return False
        if aruco_id in self.aruco_to_mac and self.aruco_to_mac[aruco_id] != mac:
            return False

        record.aruco_id = aruco_id
        record.state = RobotConnectionState.CONNECTED
        self.aruco_to_mac[aruco_id] = mac
        if mac in self.pending_macs:
            self.pending_macs.remove(mac)

        self.db.set_robot_state(mac, "connected", aruco_id=aruco_id)
        self.db.log_event("connected", mac=mac, aruco_id=aruco_id)
        return True

    def update_position(
        self,
        aruco_id: int,
        x: int,
        y: int,
        orientation: int,
        led_status: str,
    ) -> tuple[Optional[RobotRecord], bool]:
        mac = self.aruco_to_mac.get(aruco_id)
        if mac is None:
            return None, False

        record = self.robots[mac]
        record.x = x
        record.y = y
        record.orientation = orientation
        record.led_status = led_status

        went_offline = False
        if led_status == "off" and record.state == RobotConnectionState.CONNECTED:
            record.state = RobotConnectionState.OFFLINE
            went_offline = True
            self.db.set_robot_state(mac, "offline")
            self.db.log_event("offline", mac=mac, aruco_id=aruco_id)
        else:
            self.db.update_robot_position(
                mac, x=x, y=y, orientation=orientation, led_status=led_status
            )

        return record, went_offline

    def connected_robots(self) -> List[RobotRecord]:
        return [r for r in self.robots.values() if r.state == RobotConnectionState.CONNECTED]

    def all_positions(self) -> Dict[str, dict]:
        return {mac: record.position_dict() for mac, record in self.robots.items()}
