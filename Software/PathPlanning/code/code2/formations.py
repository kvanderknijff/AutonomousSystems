#import math
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
    optimized_assignments = optimal_robot_assignment(robot_positions, raw_targets)
    return optimized_assignments


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
    optimized_assignments = optimal_robot_assignment(robot_positions, raw_targets)
    return optimized_assignments


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