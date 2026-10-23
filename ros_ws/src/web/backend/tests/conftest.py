"""
后端测试公共设置：在 import 任何后端模块之前，把配置指向临时目录（地图、标注、状态、robot.yaml 都写这里），
不碰 ~/max_robot。
"""
import os, shutil, sys, tempfile
import pytest
import yaml

BACKEND = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TMP = tempfile.mkdtemp(prefix="robot_web_test_")

with open(os.path.join(BACKEND, "config.yaml")) as f:
    _cfg = yaml.safe_load(f)
_cfg.update(
    map_dirs=[os.path.join(TMP, "maps")],
    state_file=os.path.join(TMP, "state.json"),
    annotations_dir=os.path.join(TMP, "annotations"),
    captures_dir=os.path.join(TMP, "captures"),
    robot_file=os.path.join(TMP, "robot.yaml"),
    autostart=[],
)
_cfg_path = os.path.join(TMP, "config.yaml")
with open(_cfg_path, "w") as f:
    yaml.safe_dump(_cfg, f, allow_unicode=True)
os.environ["ROBOT_UI_CONFIG"] = _cfg_path
sys.path.insert(0, BACKEND)


@pytest.fixture
def clean():
    """每个测试前清空临时目录（保留 config.yaml），并清掉模块缓存"""
    for name in os.listdir(TMP):
        if name == "config.yaml":
            continue
        p = os.path.join(TMP, name)
        shutil.rmtree(p) if os.path.isdir(p) else os.remove(p)
    os.makedirs(os.path.join(TMP, "maps"))
    import robot_config, maps
    robot_config._cache.update(key=None, data=None)
    getattr(maps, "_version_cache", {}).clear()   # Task 3 才有
    yield TMP
