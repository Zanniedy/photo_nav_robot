import os, shutil
from config import MAPS_DIR


def list_maps() -> list[dict]:
    os.makedirs(MAPS_DIR, exist_ok=True)
    result = []
    for entry in sorted(os.scandir(MAPS_DIR), key=lambda e: e.name):
        if not entry.is_dir():
            continue
        yaml_path = os.path.join(entry.path, "map.yaml")
        if os.path.exists(yaml_path):
            result.append({"name": entry.name, "yaml_path": yaml_path})
    return result


def map_yaml_path(name: str) -> str:
    return os.path.join(MAPS_DIR, name, "map.yaml")


def save_map_files(src_yaml: str, dest_name: str) -> None:
    dest_dir = os.path.join(MAPS_DIR, dest_name)
    os.makedirs(dest_dir, exist_ok=True)
    src_dir = os.path.dirname(src_yaml)
    base = os.path.splitext(os.path.basename(src_yaml))[0]
    for ext in (".yaml", ".pgm"):
        src = os.path.join(src_dir, base + ext)
        if os.path.exists(src):
            shutil.copy2(src, os.path.join(dest_dir, "map" + ext))
