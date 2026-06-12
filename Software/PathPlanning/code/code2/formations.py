# formations.py
import math

def calculate_line(robot_ids: list[str], center_x: float, center_y: float, spacing: float = 20.0) -> dict[str, tuple[float, float]]:
    """Calculate target coordinates in a straight horizontal line centered around (center_x, center_y)."""
    targets: dict[str, tuple[float, float]] = {}
    n = len(robot_ids)
    
    # Calculate the starting X coordinate so the entire line is centered
    start_x = center_x - ((n - 1) * spacing) / 2
    
    for i, robot_id in enumerate(robot_ids):
        targets[robot_id] = (start_x + (i * spacing), center_y)
        
    return targets


def calculate_plus(robot_ids: list[str], center_x: float, center_y: float, spacing: float = 20.0) -> dict[str, tuple[float, float]]:
    """Calculate target coordinates in a '+' shape. Scales dynamically based on robot count."""
    targets: dict[str, tuple[float, float]] = {}
    
    if not robot_ids:
        return targets

    for i, robot_id in enumerate(robot_ids):
        if i == 0:   # Center point
            targets[robot_id] = (center_x, center_y)
        elif i == 1: # North
            targets[robot_id] = (center_x, center_y - spacing)
        elif i == 2: # South
            targets[robot_id] = (center_x, center_y + spacing)
        elif i == 3: # East
            targets[robot_id] = (center_x + spacing, center_y)
        elif i == 4: # West
            targets[robot_id] = (center_x - spacing, center_y)
        else:
            # For 6 or more robots, extend the arms of the plus further outward
            layer = (i - 1) // 4 + 1
            offset = layer * spacing
            direction = (i - 1) % 4
            
            if direction == 0:   targets[robot_id] = (center_x, center_y - offset) # North
            elif direction == 1: targets[robot_id] = (center_x, center_y + offset) # South
            elif direction == 2: targets[robot_id] = (center_x + offset, center_y) # East
            else:                targets[robot_id] = (center_x - offset, center_y) # West
            
    return targets
