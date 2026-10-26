"""巡逻路线参数文件（config.yaml 的 patrol_file，默认 ~/max_robot/patrol.yaml）→ 地图旁 annot.json 的 routes

每台机器人一份，不进仓库（rsync --delete 不会碰），格式见 patrol.example.yaml：
  maps: {<地图名>: {routes: [{name, points: [点位名...], loop}]}}
后端启动时同步一次所有地图；每次起导航 / 切地图时再同步那张地图，所以改了文件重新进导航就生效，不用重启后端。
按路线名覆盖：文件里有的路线以文件为准（点、loop），网页 / API 建的其他路线不动，文件里删掉的路线也不会删。
只生成路线，不自动开始巡逻（POST /api/v1/tasks {"type": "patrol", "route": ...}）。
点名找不到、地图不存在等错误只写日志，跳过那条路线，不影响后端启动。
"""
import os
from typing import Callable

import yaml

import annot_store, maps
from config import PATROL_FILE


def load(path: str = PATROL_FILE) -> dict:
    """{地图名: [路线, ...]}；文件不存在返回 {}，格式错抛 ValueError"""
    if not os.path.isfile(path):
        return {}
    with open(path) as f:
        d = yaml.safe_load(f) or {}
    if not isinstance(d, dict) or not isinstance(d.get("maps") or {}, dict):
        raise ValueError("顶层要是 maps: {地图名: {routes: [...]}}")
    out = {}
    for name, m in (d.get("maps") or {}).items():
        routes = (m or {}).get("routes") or []
        if not isinstance(routes, list):
            raise ValueError(f"{name}.routes 要是列表")
        out[str(name)] = routes
    return out


def sync(only_map: str | None = None, path: str = PATROL_FILE,
         log: Callable[[str], None] = lambda s: None) -> int:
    """把文件里的路线写进对应地图；only_map = yaml 路径时只同步这张。返回写入（新建或改动）的路线数"""
    try:
        cfg = load(path)
    except (OSError, ValueError, yaml.YAMLError) as e:
        log(f"巡逻路线文件 {path} 读取失败: {e}")
        return 0
    n = 0
    for map_name, routes in cfg.items():
        try:
            yaml_path = maps.by_map_id(map_name)["id"]
        except KeyError:
            if only_map is None:
                log(f"巡逻路线文件：地图 {map_name} 不存在，跳过")
            continue
        except ValueError as e:
            log(f"巡逻路线文件：{e}，跳过")
            continue
        if only_map is not None and os.path.abspath(yaml_path) != os.path.abspath(only_map):
            continue
        for r in routes:
            try:
                n += _sync_route(yaml_path, r)
            except (annot_store.StoreError, OSError, KeyError, TypeError, AttributeError) as e:
                name = r.get("name") if isinstance(r, dict) else r
                log(f"巡逻路线 {map_name}/{name} 没生成: {e}")
    if n:
        log(f"巡逻路线文件：写入 {n} 条路线")
    return n


def _sync_route(yaml_path: str, r: dict) -> int:
    name, loop = str(r["name"]), bool(r.get("loop", True))
    points = [str(p) for p in r["points"]]
    old = annot_store.get_route(yaml_path, name)
    if old is None:
        annot_store.create_route(yaml_path, name, points, loop)
        return 1
    ids = {p["name"]: p["id"] for p in annot_store.load(yaml_path)["points"]}
    if old["loop"] == loop and old["points"] == [ids.get(p) for p in points]:
        return 0                                     # 没变，不重写 annot.json
    annot_store.update_route(yaml_path, old["id"], points=points, loop=loop)
    return 1
