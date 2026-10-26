import annot_store, patrol_file
from fakes import make_map


def write(tmp, text):
    p = f"{tmp}/patrol.yaml"
    open(p, "w").write(text)
    return p


def test_sync_creates_updates_and_keeps_others(clean):
    y = make_map("office")
    for n in ("A", "B", "C"):
        annot_store.create_point(y, n, 1, 0)
    annot_store.create_route(y, "网页建的", ["A"])
    p = write(clean, "maps:\n  office:\n    routes:\n      - {name: r1, points: [A, B, A]}\n")
    assert patrol_file.sync(path=p) == 1
    r = annot_store.get_route(y, "r1")
    ids = {q["name"]: q["id"] for q in annot_store.load(y)["points"]}
    assert r["points"] == [ids["A"], ids["B"], ids["A"]] and r["loop"] is True
    assert patrol_file.sync(path=p) == 0                       # 没变不重写
    write(clean, "maps:\n  office:\n    routes:\n      - {name: r1, points: [C], loop: false}\n")
    assert patrol_file.sync(path=p) == 1
    r2 = annot_store.get_route(y, "r1")
    assert r2["id"] == r["id"] and r2["points"] == [ids["C"]] and r2["loop"] is False
    assert annot_store.get_route(y, "网页建的") is not None


def test_bad_routes_logged_and_skipped(clean):
    y = make_map("office")
    annot_store.create_point(y, "A", 1, 0)
    logs = []
    p = write(clean, """maps:
  office:
    routes:
      - {name: ok, points: [A]}
      - {name: bad, points: [A, 不存在]}
      - {points: [A]}
  nomap:
    routes: [{name: x, points: [A]}]
""")
    assert patrol_file.sync(path=p, log=logs.append) == 1
    assert annot_store.get_route(y, "bad") is None
    text = "\n".join(logs)
    assert "不存在" in text and "nomap" in text and "写入 1 条" in text


def test_only_map_and_missing_or_broken_file(clean):
    y1, y2 = make_map("office"), make_map("lab")
    for y in (y1, y2):
        annot_store.create_point(y, "A", 1, 0)
    p = write(clean, "maps:\n  office: {routes: [{name: r, points: [A]}]}\n  lab: {routes: [{name: r, points: [A]}]}\n")
    assert patrol_file.sync(y2, path=p) == 1
    assert annot_store.get_route(y1, "r") is None and annot_store.get_route(y2, "r") is not None
    assert patrol_file.sync(path=f"{clean}/none.yaml") == 0
    logs = []
    assert patrol_file.sync(path=write(clean, "maps: [1, 2]\n"), log=logs.append) == 0 and "读取失败" in logs[0]
