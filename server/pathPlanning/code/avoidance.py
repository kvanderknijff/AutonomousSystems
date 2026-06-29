import math

def calculate_apf_heading(current_x: float,
                          current_y: float,
                          target_x: float,
                          target_y: float,
                          other_robots_positions: list[tuple[float, float]],
                          influence_radius: float = 25.0,
                          k_attract: float = 1.0,
                          k_repel: float = 40.0
                          ) -> float:
    """Compute reactive heading vector using Artificial Potential Fields (APF)."""

    # 1. Calculate attractive potential force vector toward formation goal
    dx_target = target_x - current_x
    dy_target = target_y - current_y
    dist_target = math.hypot(dx_target, dy_target)

    # Prevent division by zero if exactly on target
    if dist_target == 0:
        return 0.0

    # Linear attraction vector scaling
    f_attr_x = k_attract * (dx_target / dist_target)
    f_attr_y = k_attract * (dy_target / dist_target)

    # 2. Calculate repulsive potential force vector from neighboring robots
    f_repel_x = 0.0
    f_repel_y = 0.0

    for ox, oy in other_robots_positions:
        dx_obstacle = current_x - ox
        dy_obstacle = current_y - oy
        dist_obstacle = math.hypot(dx_obstacle, dy_obstacle)

        # Ignore robots outside the designated spatial danger threshold (and itself)
        if dist_obstacle == 0 or dist_obstacle > influence_radius:
            continue

        # Exponential repulsive calculation to penalize close proximity
        force_magnitude = k_repel * (1.0 / dist_obstacle - 1.0 / influence_radius) / (dist_obstacle ** 2)
        
        f_repel_x += force_magnitude * (dx_obstacle / dist_obstacle)
        f_repel_y += force_magnitude * (dy_obstacle / dist_obstacle)

    # 3. Superimpose force vectors to establish the resultant path
    f_total_x = f_attr_x + f_repel_x
    f_total_y = f_attr_y + f_repel_y

    # 4. Extract final modified steering angle orientation
    adjusted_heading_rad = math.atan2(f_total_y, f_total_x)
    return math.degrees(adjusted_heading_rad)