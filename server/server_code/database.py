"""SQLite persistence for robot registry, positions, and events."""

import sqlite3
import threading
from contextlib import contextmanager
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

from config import DATABASE_PATH, DATA_DIR


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


class Database:
    def __init__(self, db_path: Path = DATABASE_PATH):
        DATA_DIR.mkdir(parents=True, exist_ok=True)
        self.db_path = db_path
        self._lock = threading.Lock()
        self._init_schema()

    @contextmanager
    def _connection(self):
        conn = sqlite3.connect(self.db_path, check_same_thread=False)
        conn.row_factory = sqlite3.Row
        try:
            yield conn
            conn.commit()
        finally:
            conn.close()

    def _init_schema(self) -> None:
        with self._lock, self._connection() as conn:
            conn.executescript(
                """
                CREATE TABLE IF NOT EXISTS robots (
                    mac TEXT PRIMARY KEY,
                    aruco_id INTEGER,
                    state TEXT NOT NULL DEFAULT 'pending',
                    x INTEGER NOT NULL DEFAULT 0,
                    y INTEGER NOT NULL DEFAULT 0,
                    orientation INTEGER NOT NULL DEFAULT 0,
                    led_status TEXT NOT NULL DEFAULT 'off',
                    created_at TEXT NOT NULL,
                    updated_at TEXT NOT NULL
                );

                CREATE TABLE IF NOT EXISTS position_history (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    mac TEXT NOT NULL,
                    aruco_id INTEGER,
                    x INTEGER NOT NULL,
                    y INTEGER NOT NULL,
                    orientation INTEGER NOT NULL,
                    led_status TEXT NOT NULL,
                    recorded_at TEXT NOT NULL,
                    FOREIGN KEY (mac) REFERENCES robots(mac)
                );

                CREATE TABLE IF NOT EXISTS events (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    mac TEXT,
                    aruco_id INTEGER,
                    event_type TEXT NOT NULL,
                    payload TEXT,
                    recorded_at TEXT NOT NULL
                );

                CREATE INDEX IF NOT EXISTS idx_position_history_mac
                    ON position_history(mac, recorded_at);
                CREATE INDEX IF NOT EXISTS idx_events_mac
                    ON events(mac, recorded_at);
                """
            )

    def upsert_robot(
        self,
        mac: str,
        *,
        aruco_id: Optional[int] = None,
        state: str = "pending",
        x: int = 0,
        y: int = 0,
        orientation: int = 0,
        led_status: str = "off",
    ) -> None:
        now = _utc_now()
        with self._lock, self._connection() as conn:
            conn.execute(
                """
                INSERT INTO robots (mac, aruco_id, state, x, y, orientation, led_status, created_at, updated_at)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
                ON CONFLICT(mac) DO UPDATE SET
                    aruco_id = COALESCE(excluded.aruco_id, robots.aruco_id),
                    state = excluded.state,
                    x = excluded.x,
                    y = excluded.y,
                    orientation = excluded.orientation,
                    led_status = excluded.led_status,
                    updated_at = excluded.updated_at
                """,
                (mac, aruco_id, state, x, y, orientation, led_status, now, now),
            )

    def update_robot_position(
        self,
        mac: str,
        *,
        x: int,
        y: int,
        orientation: int,
        led_status: str,
        state: Optional[str] = None,
    ) -> None:
        now = _utc_now()
        with self._lock, self._connection() as conn:
            if state is None:
                conn.execute(
                    """
                    UPDATE robots
                    SET x = ?, y = ?, orientation = ?, led_status = ?, updated_at = ?
                    WHERE mac = ?
                    """,
                    (x, y, orientation, led_status, now, mac),
                )
            else:
                conn.execute(
                    """
                    UPDATE robots
                    SET x = ?, y = ?, orientation = ?, led_status = ?, state = ?, updated_at = ?
                    WHERE mac = ?
                    """,
                    (x, y, orientation, led_status, state, now, mac),
                )

    def set_robot_state(self, mac: str, state: str, aruco_id: Optional[int] = None) -> None:
        now = _utc_now()
        with self._lock, self._connection() as conn:
            if aruco_id is not None:
                conn.execute(
                    "UPDATE robots SET state = ?, aruco_id = ?, updated_at = ? WHERE mac = ?",
                    (state, aruco_id, now, mac),
                )
            else:
                conn.execute(
                    "UPDATE robots SET state = ?, updated_at = ? WHERE mac = ?",
                    (state, now, mac),
                )

    def clear_aruco_mapping(self, mac: str) -> None:
        now = _utc_now()
        with self._lock, self._connection() as conn:
            conn.execute(
                "UPDATE robots SET aruco_id = NULL, updated_at = ? WHERE mac = ?",
                (now, mac),
            )

    def get_robot(self, mac: str) -> Optional[Dict[str, Any]]:
        with self._lock, self._connection() as conn:
            row = conn.execute("SELECT * FROM robots WHERE mac = ?", (mac,)).fetchone()
            return dict(row) if row else None

    def get_all_robots(self) -> List[Dict[str, Any]]:
        with self._lock, self._connection() as conn:
            rows = conn.execute("SELECT * FROM robots ORDER BY mac").fetchall()
            return [dict(row) for row in rows]

    def get_mac_by_aruco(self, aruco_id: int) -> Optional[str]:
        with self._lock, self._connection() as conn:
            row = conn.execute(
                "SELECT mac FROM robots WHERE aruco_id = ? AND state != 'offline'",
                (aruco_id,),
            ).fetchone()
            return row["mac"] if row else None

    def record_position_history(
        self,
        mac: str,
        aruco_id: Optional[int],
        x: int,
        y: int,
        orientation: int,
        led_status: str,
    ) -> None:
        with self._lock, self._connection() as conn:
            conn.execute(
                """
                INSERT INTO position_history (mac, aruco_id, x, y, orientation, led_status, recorded_at)
                VALUES (?, ?, ?, ?, ?, ?, ?)
                """,
                (mac, aruco_id, x, y, orientation, led_status, _utc_now()),
            )

    def log_event(
        self,
        event_type: str,
        *,
        mac: Optional[str] = None,
        aruco_id: Optional[int] = None,
        payload: Optional[str] = None,
    ) -> None:
        with self._lock, self._connection() as conn:
            conn.execute(
                """
                INSERT INTO events (mac, aruco_id, event_type, payload, recorded_at)
                VALUES (?, ?, ?, ?, ?)
                """,
                (mac, aruco_id, event_type, payload, _utc_now()),
            )

    def load_registry_state(self) -> List[Dict[str, Any]]:
        """Restore robots from DB on server startup."""
        return self.get_all_robots()

    def clear_all(self) -> int:
        """Drop all robots on server restart so every unit must handshake again."""
        with self._lock, self._connection() as conn:
            tables = ["robots", "events", "position_history", "sqlite_sequence"]
            for table in tables:
                result = conn.execute(f"DELETE FROM {table}")
                if table == "robots":
                    db_robots = result
            return db_robots.rowcount
