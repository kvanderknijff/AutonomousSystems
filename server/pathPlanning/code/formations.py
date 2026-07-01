import math
from assignment import optimal_robot_assignment

def calculate_line(robot_positions: dict[str, tuple[float, float]], 
                   center_x: float, 
                   center_y: float, 
                   spacing: float = 20.0
                   ) -> dict[str, tuple[float, float]]:
    """Generate linear formation targets and optimize robot assignment."""
    
    robot_ids = list(robot_positions.keys())
    num_robots = len(robot_ids)
    
    if num_robots == 0:
        return {}

    # 1. Generate discrete target coordinates along horizontal axis
    raw_targets: list[tuple[float, float]] = []
    start_x = center_x - ((num_robots - 1) * spacing) / 2
    
    for i in range(num_robots):
        target_point = (start_x + (i * spacing), center_y)
        raw_targets.append(target_point)
        
    # 2. Map coordinates optimally to minimize total travel distance
    return optimal_robot_assignment(robot_positions, raw_targets)


def calculate_plus(robot_positions: dict[str, tuple[float, float]],
                   center_x: float,
                   center_y: float,
                   spacing: float = 20.0
                   ) -> dict[str, tuple[float, float]]:
    """Generate cross formation targets and optimize robot assignment."""

    robot_ids = list(robot_positions.keys())
    n = len(robot_ids)
    
    if n == 0:
        return {}

    # 1. Generate discrete cross coordinates extending from center outward
    raw_targets: list[tuple[float, float]] = []
    
    for i in range(n):
        if i == 0:   # Center point
            raw_targets.append((center_x, center_y))
        elif i == 1: # Directional node: North
            raw_targets.append((center_x, center_y - spacing))
        elif i == 2: # Directional node: South
            raw_targets.append((center_x, center_y + spacing))
        elif i == 3: # Directional node: East
            raw_targets.append((center_x + spacing, center_y))
        elif i == 4: # Directional node: West
            raw_targets.append((center_x - spacing, center_y))
        else:
            # Scaled expansion for fleets larger than 5 robots
            layer = (i - 1) // 4 + 1
            offset = layer * spacing
            direction = (i - 1) % 4
            
            if direction == 0:   raw_targets.append((center_x, center_y - offset))
            elif direction == 1: raw_targets.append((center_x, center_y + offset))
            elif direction == 2: raw_targets.append((center_x + offset, center_y))
            else:                raw_targets.append((center_x - offset, center_y))
            
    # 2. Map coordinates optimally to minimize total travel distance
    return optimal_robot_assignment(robot_positions, raw_targets)


def calculate_square(robot_positions: dict[str, tuple[float, float]],
                     center_x: float,
                     center_y: float,
                     spacing: float = 30.0
                     ) -> dict[str, tuple[float, float]]:
    """Generate square bounding box targets and optimize robot assignment."""

    robot_ids = list(robot_positions.keys())
    n = len(robot_ids)
    
    if n == 0:
        return {}

    # 1. Compute bounding boundary vertices (Corners)
    raw_targets: list[tuple[float, float]] = []
    half_size = spacing
    
    corners = [
        (center_x - half_size, center_y - half_size), # Top-Left
        (center_x + half_size, center_y - half_size), # Top-Right
        (center_x + half_size, center_y + half_size), # Bottom-Right
        (center_x - half_size, center_y + half_size)  # Bottom-Left
    ]
    
    for i in range(n):
        if i < 4:
            # Primary structural validation (4 bounding corners)
            raw_targets.append(corners[i])
        else:
            # Secondary structural expansion (Edge midpoints for n > 4)
            if i == 4:   raw_targets.append((center_x, center_y - half_size)) # Midpoint: Top edge
            elif i == 5: raw_targets.append((center_x, center_y + half_size)) # Midpoint: Bottom edge
            elif i == 6: raw_targets.append((center_x - half_size, center_y)) # Midpoint: Left edge
            else:        raw_targets.append((center_x + half_size, center_y)) # Midpoint: Right edge

    # 2. Map coordinates optimally to minimize total travel distance
    return optimal_robot_assignment(robot_positions, raw_targets)


def calculate_Y(robot_positions: dict[str, tuple[float, float]],
                   center_x: float,
                   center_y: float,
                   spacing: float = 20.0
                   ) -> dict[str, tuple[float, float]]:
    """Generate 'Y' formation targets and optimize robot assignment."""

    robot_ids = list(robot_positions.keys())
    n = len(robot_ids)
    
    if n == 0:
        return {}

    # 1. Generate discrete Y-shape coordinates extending from center outward
    raw_targets: list[tuple[float, float]] = []
    
    # Pre-calculate trigonometric values for 30-degree angles (the upper branches)
    cos_30 = math.cos(math.radians(30))
    sin_30 = math.sin(math.radians(30))
    
    for i in range(n):
        if i == 0:
            # Center point where the three branches meet
            raw_targets.append((center_x, center_y))
        else:
            # Determine which branch to grow: 
            # 0 = Stem (Down), 1 = Top-Left Arm, 2 = Top-Right Arm
            branch = (i - 1) % 3
            layer = (i - 1) // 3 + 1
            offset = layer * spacing
            
            if branch == 0:
                # Vertical stem extending downwards
                raw_targets.append((center_x, center_y + offset))
            elif branch == 1:
                # Top-Left arm (moving left and up)
                raw_targets.append((center_x - offset * cos_30, center_y - offset * sin_30))
            else:
                # Top-Right arm (moving right and up)
                raw_targets.append((center_x + offset * cos_30, center_y - offset * sin_30))

    # 2. Map coordinates optimally to minimize total travel distance
    return optimal_robot_assignment(robot_positions, raw_targets)
