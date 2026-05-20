from pathlib import Path
import yaml


BASE_DIR = Path(__file__).resolve().parent.parent

PARAMS_PATH = BASE_DIR / "config" / "nav2_keepout_params.yaml"
KEEP_OUT_MASK_YAML = BASE_DIR / "maps" / "danger_keepout_mask.yaml"
KEEP_OUT_SERVERS_PATH = BASE_DIR / "config" / "keepout_servers.yaml"


def ensure_dict(parent, key):
    if key not in parent or parent[key] is None:
        parent[key] = {}

    return parent[key]


def patch_global_costmap(params):
    global_costmap = ensure_dict(params, "global_costmap")
    global_costmap_node = ensure_dict(global_costmap, "global_costmap")
    ros_params = ensure_dict(global_costmap_node, "ros__parameters")

    filters = ros_params.get("filters", [])

    if filters is None:
        filters = []

    if "keepout_filter" not in filters:
        filters.append("keepout_filter")

    ros_params["filters"] = filters

    ros_params["keepout_filter"] = {
        "plugin": "nav2_costmap_2d::KeepoutFilter",
        "enabled": True,
        "filter_info_topic": "costmap_filter_info",
    }


def write_keepout_servers_params():
    data = {
        "filter_mask_server": {
            "ros__parameters": {
                "use_sim_time": True,
                "yaml_filename": str(KEEP_OUT_MASK_YAML),
                "topic_name": "keepout_filter_mask",
                "frame_id": "map",
            }
        },
        "costmap_filter_info_server": {
            "ros__parameters": {
                "use_sim_time": True,
                "type": 0,
                "filter_info_topic": "costmap_filter_info",
                "mask_topic": "keepout_filter_mask",
                "base": 0.0,
                "multiplier": 1.0,
            }
        },
    }

    with open(KEEP_OUT_SERVERS_PATH, "w", encoding="utf-8") as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False)

    print("Generated keepout server params:")
    print(KEEP_OUT_SERVERS_PATH)


def main():
    if not PARAMS_PATH.exists():
        raise FileNotFoundError(
            f"{PARAMS_PATH} 파일이 없습니다. "
            "먼저 nav2_params.yaml을 복사하세요."
        )

    with open(PARAMS_PATH, "r", encoding="utf-8") as f:
        params = yaml.safe_load(f)

    if params is None:
        params = {}

    patch_global_costmap(params)

    with open(PARAMS_PATH, "w", encoding="utf-8") as f:
        yaml.dump(params, f, default_flow_style=False, sort_keys=False)

    print("Patched Nav2 params:")
    print(PARAMS_PATH)

    write_keepout_servers_params()


if __name__ == "__main__":
    main()