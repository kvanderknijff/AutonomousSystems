"""Resolve ArUco IDs from Webots scene nodes (robots + corner solids)."""

import re

CORNER_NAME_RE = re.compile(r"corner_marker_(\d+)$", re.IGNORECASE)
ROBOT_NAME_RE = re.compile(r"formation_bot_(\d+)$", re.IGNORECASE)


def parse_custom_data(raw):
    values = {}
    for part in raw.split(";"):
        if "=" not in part:
            continue
        key, value = part.split("=", 1)
        values[key.strip()] = value.strip()
    return values


def resolve_aruco_id(node):
    """Read aruco ID from customData or node name (Solids lack customData in Webots API)."""
    name_field = node.getField("name")
    if name_field is not None:
        name = name_field.getSFString()
        corner_match = CORNER_NAME_RE.match(name)
        if corner_match:
            return int(corner_match.group(1))
        robot_match = ROBOT_NAME_RE.match(name)
        if robot_match:
            return int(robot_match.group(1))

    custom_field = node.getField("customData")
    if custom_field is None:
        return None

    custom = parse_custom_data(custom_field.getSFString())
    aruco = custom.get("aruco")
    if aruco is None or not re.fullmatch(r"\d+", aruco):
        return None
    return int(aruco)
