"""Playfield rectangle inferred from corner ArUco markers (camera pixels)."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class FieldBounds:
    corners: dict[int, tuple[float, float]] = field(default_factory=dict)

    def update_corner(self, aruco_id: int, x: float, y: float) -> None:
        self.corners[int(aruco_id)] = (float(x), float(y))

    @property
    def is_ready(self) -> bool:
        if len(self.corners) < 2:
            return False
        xs = {point[0] for point in self.corners.values()}
        ys = {point[1] for point in self.corners.values()}
        return len(xs) >= 2 and len(ys) >= 2

    def bounds(self) -> tuple[float, float, float, float] | None:
        if not self.corners:
            return None
        xs = [point[0] for point in self.corners.values()]
        ys = [point[1] for point in self.corners.values()]
        return min(xs), min(ys), max(xs), max(ys)

    def center(self) -> tuple[float, float] | None:
        rect = self.bounds()
        if rect is None:
            return None
        min_x, min_y, max_x, max_y = rect
        return (min_x + max_x) / 2, (min_y + max_y) / 2

    def clamp_point(self, x: float, y: float, margin: float) -> tuple[float, float]:
        rect = self.bounds()
        if rect is None:
            return float(x), float(y)
        min_x, min_y, max_x, max_y = rect
        return (
            max(min_x + margin, min(max_x - margin, float(x))),
            max(min_y + margin, min(max_y - margin, float(y))),
        )

    def inside(self, x: float, y: float, margin: float = 0.0) -> bool:
        rect = self.bounds()
        if rect is None:
            return True
        min_x, min_y, max_x, max_y = rect
        return (
            min_x + margin <= x <= max_x - margin
            and min_y + margin <= y <= max_y - margin
        )

    def as_dict(self) -> dict:
        rect = self.bounds()
        center = self.center()
        return {
            "corners": {
                str(aruco_id): {"x": round(x, 1), "y": round(y, 1)}
                for aruco_id, (x, y) in self.corners.items()
            },
            "ready": self.is_ready,
            "bounds": (
                {
                    "min_x": round(rect[0], 1),
                    "min_y": round(rect[1], 1),
                    "max_x": round(rect[2], 1),
                    "max_y": round(rect[3], 1),
                }
                if rect is not None
                else None
            ),
            "center": (
                {"x": round(center[0], 1), "y": round(center[1], 1)}
                if center is not None
                else None
            ),
        }
