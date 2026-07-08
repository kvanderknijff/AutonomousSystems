"""Map physical camera pixels to Webots world coordinates using corner markers."""

from mqtt_protocol import PHYSICAL_CORNER_ARUCO_FIRST, PHYSICAL_CORNER_ARUCO_LAST


def _is_physical_corner(aruco_id: int) -> bool:
    aid = int(aruco_id)
    return PHYSICAL_CORNER_ARUCO_FIRST <= aid <= PHYSICAL_CORNER_ARUCO_LAST


def _ordered_corner_points(corners: dict[int, tuple[float, float]]) -> list[tuple[float, float]] | None:
    """Return corners by image position: bottom-left, bottom-right, top-right, top-left."""
    if len(corners) < 4:
        return None

    points = list(corners.values())
    top_left = min(points, key=lambda point: point[0] + point[1])
    bottom_right = max(points, key=lambda point: point[0] + point[1])
    top_right = max(points, key=lambda point: point[0] - point[1])
    bottom_left = max(points, key=lambda point: point[1] - point[0])

    ordered = [bottom_left, bottom_right, top_right, top_left]
    if len(set(ordered)) != 4:
        return None
    return ordered


def _solve_linear_system(matrix: list[list[float]], values: list[float]) -> list[float] | None:
    size = len(values)
    rows = [matrix[index][:] + [values[index]] for index in range(size)]

    for column in range(size):
        pivot = max(range(column, size), key=lambda row: abs(rows[row][column]))
        if abs(rows[pivot][column]) < 1e-9:
            return None
        rows[column], rows[pivot] = rows[pivot], rows[column]

        pivot_value = rows[column][column]
        rows[column] = [value / pivot_value for value in rows[column]]

        for row in range(size):
            if row == column:
                continue
            factor = rows[row][column]
            rows[row] = [
                rows[row][item] - factor * rows[column][item]
                for item in range(size + 1)
            ]

    return [rows[index][size] for index in range(size)]


def _homography(
    source: list[tuple[float, float]],
    destination: list[tuple[float, float]],
) -> list[float] | None:
    matrix = []
    values = []
    for (x, y), (target_x, target_y) in zip(source, destination):
        matrix.append([x, y, 1.0, 0.0, 0.0, 0.0, -x * target_x, -y * target_x])
        values.append(target_x)
        matrix.append([0.0, 0.0, 0.0, x, y, 1.0, -x * target_y, -y * target_y])
        values.append(target_y)

    solution = _solve_linear_system(matrix, values)
    if solution is None:
        return None
    return solution + [1.0]


def _apply_homography(transform: list[float], x: float, y: float) -> tuple[float, float]:
    denominator = transform[6] * x + transform[7] * y + transform[8]
    if abs(denominator) < 1e-9:
        return float(x), float(y)
    mapped_x = (transform[0] * x + transform[1] * y + transform[2]) / denominator
    mapped_y = (transform[3] * x + transform[4] * y + transform[5]) / denominator
    return mapped_x, mapped_y


class PhysicalFieldMapper:
    def __init__(self) -> None:
        self.physical_corners: dict[int, tuple[float, float]] = {}

    def update_corner(self, aruco_id: int, x: float, y: float) -> None:
        if _is_physical_corner(aruco_id):
            self.physical_corners[int(aruco_id)] = (float(x), float(y))

    @property
    def ready(self) -> bool:
        return _ordered_corner_points(self.physical_corners) is not None

    def pixel_to_world(
        self,
        x: float,
        y: float,
        *,
        world_min: float,
        world_max: float,
    ) -> tuple[float, float]:
        source_points = _ordered_corner_points(self.physical_corners)
        if source_points is None:
            return float(x), float(y)

        destination_points = [
            (world_min, world_min),
            (world_max, world_min),
            (world_max, world_max),
            (world_min, world_max),
        ]
        transform = _homography(source_points, destination_points)
        if transform is None:
            return float(x), float(y)

        return _apply_homography(transform, float(x), float(y))
