"""Map physical camera pixels to Webots world coordinates using corner markers."""

from mqtt_protocol import PHYSICAL_CORNER_ARUCO_FIRST, PHYSICAL_CORNER_ARUCO_LAST


def _is_physical_corner(aruco_id: int) -> bool:
    aid = int(aruco_id)
    return PHYSICAL_CORNER_ARUCO_FIRST <= aid <= PHYSICAL_CORNER_ARUCO_LAST


def _bounds(corners: dict[int, tuple[float, float]]) -> tuple[float, float, float, float] | None:
    if len(corners) < 2:
        return None
    xs = [point[0] for point in corners.values()]
    ys = [point[1] for point in corners.values()]
    return min(xs), min(ys), max(xs), max(ys)


class PhysicalFieldMapper:
    def __init__(self) -> None:
        self.physical_corners: dict[int, tuple[float, float]] = {}

    def update_corner(self, aruco_id: int, x: float, y: float) -> None:
        if _is_physical_corner(aruco_id):
            self.physical_corners[int(aruco_id)] = (float(x), float(y))

    @property
    def ready(self) -> bool:
        return _bounds(self.physical_corners) is not None

    def pixel_to_world(
        self,
        x: float,
        y: float,
        *,
        world_min: float,
        world_max: float,
    ) -> tuple[float, float]:
        bounds = _bounds(self.physical_corners)
        if bounds is None:
            return float(x), float(y)

        min_x, min_y, max_x, max_y = bounds
        width = max_x - min_x
        height = max_y - min_y
        if width <= 0 or height <= 0:
            return float(x), float(y)

        u = (float(x) - min_x) / width
        v = (float(y) - min_y) / height
        span = world_max - world_min
        return world_min + u * span, world_min + v * span
