"""Map camera pixels between the physical overhead camera and Webots simulation."""

from __future__ import annotations

import os

from config import (
    CORNER_ARUCO_FIRST,
    CORNER_ARUCO_LAST,
    WEBOTS_CAMERA_HEIGHT,
    WEBOTS_CAMERA_WIDTH,
    WEBOTS_CORNER_ARUCO_FIRST,
    WEBOTS_CORNER_ARUCO_LAST,
)


def _is_physical_corner(aruco_id: int) -> bool:
    return CORNER_ARUCO_FIRST <= int(aruco_id) <= CORNER_ARUCO_LAST


def _is_simulation_corner(aruco_id: int) -> bool:
    return WEBOTS_CORNER_ARUCO_FIRST <= int(aruco_id) <= WEBOTS_CORNER_ARUCO_LAST


def _bounds(corners: dict[int, tuple[float, float]]) -> tuple[float, float, float, float] | None:
    if len(corners) < 2:
        return None
    xs = [point[0] for point in corners.values()]
    ys = [point[1] for point in corners.values()]
    return min(xs), min(ys), max(xs), max(ys)


def _normalize_in_bounds(
    x: float,
    y: float,
    bounds: tuple[float, float, float, float],
) -> tuple[float, float]:
    min_x, min_y, max_x, max_y = bounds
    width = max_x - min_x
    height = max_y - min_y
    if width <= 0 or height <= 0:
        return float(x), float(y)
    u = (float(x) - min_x) / width
    v = (float(y) - min_y) / height
    return u, v


def _denormalize_in_bounds(
    u: float,
    v: float,
    bounds: tuple[float, float, float, float],
) -> tuple[float, float]:
    min_x, min_y, max_x, max_y = bounds
    width = max_x - min_x
    height = max_y - min_y
    return min_x + u * width, min_y + v * height


def _default_simulation_bounds() -> tuple[float, float, float, float]:
    """Match the Webots overhead camera frame when corner solids are not in the world."""
    inset_fraction = float(os.getenv("WEBOTS_CORNER_INSET_M", "0.35")) / (
        float(os.getenv("WEBOTS_FIELD_SIZE_M", "10.0")) / 2.0
    )
    margin_x = WEBOTS_CAMERA_WIDTH * inset_fraction
    margin_y = WEBOTS_CAMERA_HEIGHT * inset_fraction
    return (
        margin_x,
        margin_y,
        WEBOTS_CAMERA_WIDTH - margin_x,
        WEBOTS_CAMERA_HEIGHT - margin_y,
    )


class CoordinateMapper:
    """Unify telemetry in physical camera pixel space when both fields are calibrated."""

    def __init__(self) -> None:
        self.physical_corners: dict[int, tuple[float, float]] = {}
        self.simulation_corners: dict[int, tuple[float, float]] = {}

    def update_corner(self, aruco_id: int, x: float, y: float) -> None:
        aid = int(aruco_id)
        point = (float(x), float(y))
        if _is_physical_corner(aid):
            self.physical_corners[aid] = point
        elif _is_simulation_corner(aid):
            self.simulation_corners[aid] = point

    def _simulation_bounds(self) -> tuple[float, float, float, float] | None:
        measured = _bounds(self.simulation_corners)
        if measured is not None:
            return measured
        if _bounds(self.physical_corners) is not None:
            return _default_simulation_bounds()
        return None

    @property
    def mapping_ready(self) -> bool:
        return (
            _bounds(self.physical_corners) is not None
            and self._simulation_bounds() is not None
        )

    @property
    def uses_physical_space(self) -> bool:
        return _bounds(self.physical_corners) is not None

    def to_canonical(self, x: float, y: float, *, source: str) -> tuple[float, float]:
        """Convert incoming telemetry to the planner coordinate space."""
        if source == "physical" or not self.mapping_ready:
            return float(x), float(y)

        phys_bounds = _bounds(self.physical_corners)
        sim_bounds = self._simulation_bounds()
        assert phys_bounds is not None and sim_bounds is not None
        u, v = _normalize_in_bounds(x, y, sim_bounds)
        return _denormalize_in_bounds(u, v, phys_bounds)

    def to_native_for_aruco(self, x: float, y: float, aruco_id: int) -> tuple[float, float]:
        """Convert planner coordinates to the target robot's camera pixel space."""
        from mqtt_protocol import is_webots_robot_aruco

        if not is_webots_robot_aruco(aruco_id) or not self.mapping_ready:
            return float(x), float(y)

        phys_bounds = _bounds(self.physical_corners)
        sim_bounds = self._simulation_bounds()
        assert phys_bounds is not None and sim_bounds is not None
        u, v = _normalize_in_bounds(x, y, phys_bounds)
        return _denormalize_in_bounds(u, v, sim_bounds)

    def physical_pixel_to_world(
        self,
        x: float,
        y: float,
        *,
        world_min: float,
        world_max: float,
    ) -> tuple[float, float]:
        """Map physical camera pixels to Webots world metres (for the mirror controller)."""
        bounds = _bounds(self.physical_corners)
        if bounds is None:
            return float(x), float(y)

        u, v = _normalize_in_bounds(x, y, bounds)
        span = world_max - world_min
        return world_min + u * span, world_min + v * span
