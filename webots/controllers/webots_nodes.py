"""Resolve ArUco IDs from Webots scene nodes (robots + corner solids)."""

import re

CORNER_NAME_RE = re.compile(r"corner_marker_(\d+)$", re.IGNORECASE)
ROBOT_NAME_RE = re.compile(r"formation_bot_(\d+)$", re.IGNORECASE)
PHYSICAL_PROXY_RE = re.compile(r"physical_proxy_(\d+)$", re.IGNORECASE)


def parse_custom_data(raw):
    values = {}
    for part in raw.split(";"):
        if "=" not in part:
            continue
        key, value = part.split("=", 1)
        values[key.strip()] = value.strip()
    return values


def is_disabled_robot(node) -> bool:
    custom_field = node.getField("customData")
    if custom_field is None:
        return False
    return parse_custom_data(custom_field.getSFString()).get("role") == "disabled"


def is_physical_proxy(node) -> bool:
    name_field = node.getField("name")
    if name_field is not None:
        name = name_field.getSFString()
        if PHYSICAL_PROXY_RE.match(name):
            return True

    custom_field = node.getField("customData")
    if custom_field is None:
        return False
    return parse_custom_data(custom_field.getSFString()).get("role") == "physical_proxy"


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
        proxy_match = PHYSICAL_PROXY_RE.match(name)
        if proxy_match:
            return int(proxy_match.group(1))

    custom_field = node.getField("customData")
    if custom_field is None:
        return None

    custom = parse_custom_data(custom_field.getSFString())
    aruco = custom.get("aruco")
    if aruco is None or not re.fullmatch(r"\d+", aruco):
        return None
    return int(aruco)
