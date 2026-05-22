import math
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml
from PIL import Image


BASE_DIR = Path(__file__).resolve().parent.parent

MAP_YAML_PATH = BASE_DIR / "maps" / "map.yaml"
SDF_PATH = BASE_DIR / "maps" / "maze.sdf"

OUTPUT_MASK_PGM = BASE_DIR / "maps" / "danger_keepout_mask.pgm"
OUTPUT_MASK_YAML = BASE_DIR / "maps" / "danger_keepout_mask.yaml"

KEEP_OUT_RADIUS_M = 2.5

WORLD_TO_MAP_A = (
    (0.94706522, 0.32869565),
    (-0.33073370, 0.94217391),
)
WORLD_TO_MAP_T = (-0.10391304, -1.73847826)


def read_color_from_model(model):
    for tag in model.iter():
        if tag.tag not in ["ambient", "diffuse"]:
            continue

        if tag.text is None:
            continue

        values = tag.text.strip().split()

        if len(values) < 3:
            continue

        r = float(values[0])
        g = float(values[1])
        b = float(values[2])

        if r > 0.7 and g < 0.3 and b < 0.3:
            return "red"

    return "unknown"


def parse_pose(model):
    pose = model.find("pose")

    if pose is None or pose.text is None:
        return None

    values = pose.text.strip().split()

    if len(values) < 2:
        return None

    return float(values[0]), float(values[1])


def load_danger_points_from_sdf(sdf_path):
    tree = ET.parse(sdf_path)
    root = tree.getroot()

    danger_points = []

    for model in root.iter("model"):
        model_name = model.attrib.get("name", "").lower()
        color = read_color_from_model(model)

        is_danger = (
            color == "red"
            or "red" in model_name
            or "danger" in model_name
            or "fire" in model_name
        )

        if not is_danger:
            continue

        pose = parse_pose(model)

        if pose is None:
            continue

        danger_points.append(pose)

    return danger_points


def transform_world_to_map(point):
    x, y = point
    map_x = WORLD_TO_MAP_A[0][0] * x + WORLD_TO_MAP_A[0][1] * y + WORLD_TO_MAP_T[0]
    map_y = WORLD_TO_MAP_A[1][0] * x + WORLD_TO_MAP_A[1][1] * y + WORLD_TO_MAP_T[1]

    return map_x, map_y


def get_map_image_path(map_yaml):
    image_name = map_yaml["image"]
    return (MAP_YAML_PATH.parent / image_name).resolve()


def get_map_image_size(map_yaml):
    image_path = get_map_image_path(map_yaml)

    if not image_path.exists():
        raise FileNotFoundError(f"Map image not found: {image_path}")

    image = Image.open(image_path)
    return image.size


def world_to_pixel(x, y, origin_x, origin_y, resolution, height):
    pixel_x = int(round((x - origin_x) / resolution))
    pixel_y_from_bottom = int(round((y - origin_y) / resolution))

    pixel_y = height - 1 - pixel_y_from_bottom

    return pixel_x, pixel_y


def draw_filled_circle(mask, center_x, center_y, radius_px):
    height = len(mask)
    width = len(mask[0])

    min_x = max(0, center_x - radius_px)
    max_x = min(width - 1, center_x + radius_px)
    min_y = max(0, center_y - radius_px)
    max_y = min(height - 1, center_y + radius_px)

    for py in range(min_y, max_y + 1):
        for px in range(min_x, max_x + 1):
            dx = px - center_x
            dy = py - center_y

            if dx * dx + dy * dy <= radius_px * radius_px:
                mask[py][px] = 0


def write_pgm(path, mask):
    height = len(mask)
    width = len(mask[0])

    with open(path, "wb") as f:
        f.write(b"P5\n")
        f.write(f"{width} {height}\n".encode())
        f.write(b"255\n")

        for row in mask:
            f.write(bytes(row))


def write_mask_yaml(path, image_name, resolution, origin):
    data = {
        "image": image_name,
        "mode": "trinary",
        "resolution": float(resolution),
        "origin": origin,
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
    }

    with open(path, "w", encoding="utf-8") as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False)


def main():
    with open(MAP_YAML_PATH, "r", encoding="utf-8") as f:
        map_yaml = yaml.safe_load(f)

    resolution = float(map_yaml["resolution"])
    origin = map_yaml["origin"]

    origin_x = float(origin[0])
    origin_y = float(origin[1])

    width, height = get_map_image_size(map_yaml)
    sdf_danger_points = load_danger_points_from_sdf(SDF_PATH)
    danger_points = [transform_world_to_map(point) for point in sdf_danger_points]

    if not sdf_danger_points:
        print("No danger points found.")
        return

    mask = [[255 for _ in range(width)] for _ in range(height)]

    radius_px = int(math.ceil(KEEP_OUT_RADIUS_M / resolution))

    print("Map size:", width, height)
    print("Resolution:", resolution)
    print("Origin:", origin)
    print("SDF danger points:", sdf_danger_points)
    print("Map-frame danger points:", danger_points)
    print("Keepout radius:", KEEP_OUT_RADIUS_M, "m")
    print("Keepout radius:", radius_px, "px")

    for x, y in danger_points:
        px, py = world_to_pixel(x, y, origin_x, origin_y, resolution, height)
        print(f"Danger world=({x:.2f}, {y:.2f}) -> pixel=({px}, {py})")
        draw_filled_circle(mask, px, py, radius_px)

    write_pgm(OUTPUT_MASK_PGM, mask)

    write_mask_yaml(
        OUTPUT_MASK_YAML,
        OUTPUT_MASK_PGM.name,
        resolution,
        origin,
    )

    print("\nGenerated:")
    print(OUTPUT_MASK_PGM)
    print(OUTPUT_MASK_YAML)


if __name__ == "__main__":
    main()
