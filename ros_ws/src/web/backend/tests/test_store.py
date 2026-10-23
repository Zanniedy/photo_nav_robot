import pytest
import annot_store, maps, state
from fakes import make_map


def test_map_id_and_version(clean):
    y = make_map("office")
    assert maps.name_of(y) == "office"
    v1 = maps.version(y)
    assert len(v1) == 12 and maps.version(y) == v1
    make_map("office", pixels=b"\xff" * 16)          # 图变了版本就变
    assert maps.version(y) != v1


def test_old_dir_layout_name(clean):
    assert maps.name_of(make_map("map", subdir="hall")) == "hall"


def test_by_map_id(clean):
    make_map("office")
    make_map("lab", subdir="x")
    assert maps.by_map_id("lab")["name"] == "lab"
    with pytest.raises(KeyError):
        maps.by_map_id("nope")
    make_map("lab", subdir="y")
    with pytest.raises(ValueError):
        maps.by_map_id("lab")


def test_routes_crud(clean):
    y = make_map("office")
    a = annot_store.create_point(y, "A", 1, 0)
    b = annot_store.create_point(y, "B", 2, 0)
    r = annot_store.create_route(y, "日常", ["A", b["id"]], loop=True)
    assert r["points"] == [a["id"], b["id"]] and r["loop"] is True
    assert annot_store.get_route(y, "日常")["id"] == r["id"]
    r2 = annot_store.update_route(y, r["id"], loop=False, points=["B"])
    assert r2["points"] == [b["id"]] and r2["loop"] is False and r2["name"] == "日常"
    for name, pts in (("日常", ["A"]), ("x", ["Z"]), ("x", [])):   # 重名 / 点不存在 / 空
        with pytest.raises(annot_store.StoreError):
            annot_store.create_route(y, name, pts)
    with pytest.raises(KeyError):
        annot_store.update_route(y, "nope", loop=True)
    assert annot_store.delete_route(y, r["id"]) is True
    assert annot_store.delete_route(y, r["id"]) is False


def test_delete_point_removes_from_routes(clean):
    y = make_map("office")
    annot_store.create_point(y, "A", 1, 0)
    b = annot_store.create_point(y, "B", 2, 0)
    r = annot_store.create_route(y, "r", ["A", "B", "A"])
    annot_store.save_shapes(y, [])                    # 存形状不能把路线冲掉
    assert annot_store.delete_point(y, "A") is True
    assert annot_store.get_route(y, r["id"])["points"] == [b["id"]]


def test_state_estop_default(clean):
    assert state.load()["estop"] is False
