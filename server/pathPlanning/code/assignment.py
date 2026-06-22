import math
from itertools import permutations

def calculate_distance(x1: float, 
                       y1: float, 
                       x2: float, 
                       y2: float
                       ) -> float:
    """Calculate the straight-line distance between two coordinates."""
    return math.hypot(x2 - x1, y2 - y1)


def optimal_robot_assignment(robot_positions: dict[str, tuple[float, float]],
                             target_positions: list[tuple[float, float]]
                             ) -> dict[str, tuple[float, float]]:
    """Minimize total fleet travel distance using permutation cost mapping."""

    robot_ids = list(robot_positions.keys())
    num_robots = len(robot_ids)
    
    if num_robots == 0 or len(target_positions) == 0:
        return {}

    # Brute-force cost matrix evaluation for optimal configuration mapping
    best_permutation: tuple[tuple[float, float], ...] | None = None
    min_total_distance = float('inf')

    # Iterate through all discrete target coordinate permutations
    for target_perm in permutations(target_positions, num_robots):
        current_total_distance = 0.0
        
        # Aggregate global cost sum for current fleet mapping
        for i, robot_id in enumerate(robot_ids):
            rx, ry = robot_positions[robot_id]
            tx, ty = target_perm[i]
            current_total_distance += calculate_distance(rx, ry, tx, ty)
            
        # Retain configuration if global spatial cost is minimized
        if current_total_distance < min_total_distance:
            min_total_distance = current_total_distance
            best_permutation = target_perm

    # Construct the optimized target assignment dictionary
    assignments: dict[str, tuple[float, float]] = {}
    if best_permutation:
        for i, robot_id in enumerate(robot_ids):
            assignments[robot_id] = best_permutation[i]

    return assignments