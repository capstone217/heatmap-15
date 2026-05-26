import math

try:
    from config import DRIVE_SETTINGS
except Exception:
    DRIVE_SETTINGS = {}


DEFAULT_GRID_SIZE_M = 0.3


def default_grid_size():
    try:
        return float(DRIVE_SETTINGS.get("AUTO_GRID_SPACING_M", DEFAULT_GRID_SIZE_M))
    except (TypeError, ValueError):
        return DEFAULT_GRID_SIZE_M


def _resolve_grid_size(grid_size):
    if grid_size is None:
        grid_size = default_grid_size()
    try:
        grid_size = float(grid_size)
    except (TypeError, ValueError) as exc:
        raise ValueError("grid_size must be a positive number") from exc
    if grid_size <= 0:
        raise ValueError("grid_size must be greater than 0")
    return grid_size


def xy_to_grid_id(x, y, grid_size=None, floor="F1"):
    grid_size = _resolve_grid_size(grid_size)
    try:
        x = float(x)
        y = float(y)
    except (TypeError, ValueError) as exc:
        raise ValueError("x and y must be numbers") from exc

    grid_x = int(math.floor(x / grid_size))
    grid_y = int(math.floor(y / grid_size))
    return f"{floor}_{grid_x}_{grid_y}"


def parse_grid_id(grid_id):
    parts = str(grid_id).split("_")
    if len(parts) < 3:
        raise ValueError("grid_id must have format '<floor>_<grid_x>_<grid_y>'")

    floor = "_".join(parts[:-2])
    try:
        grid_x = int(parts[-2])
        grid_y = int(parts[-1])
    except ValueError as exc:
        raise ValueError("grid_x and grid_y in grid_id must be integers") from exc

    if not floor:
        raise ValueError("floor in grid_id must not be empty")

    return {
        "floor": floor,
        "grid_x": grid_x,
        "grid_y": grid_y,
    }


def grid_center_from_id(grid_id, grid_size=None):
    grid_size = _resolve_grid_size(grid_size)
    parsed = parse_grid_id(grid_id)
    return {
        "floor": parsed["floor"],
        "grid_x": parsed["grid_x"],
        "grid_y": parsed["grid_y"],
        "x_center": (parsed["grid_x"] + 0.5) * grid_size,
        "y_center": (parsed["grid_y"] + 0.5) * grid_size,
    }


if __name__ == "__main__":
    example_grid_id = xy_to_grid_id(1.25, 0.82, grid_size=0.3, floor="F1")
    print(example_grid_id)
    print(parse_grid_id(example_grid_id))
    print(grid_center_from_id(example_grid_id, grid_size=0.3))
