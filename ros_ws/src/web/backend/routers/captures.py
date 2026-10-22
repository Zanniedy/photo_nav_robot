"""相机照片：camera_node 的 /camera/snapshot 存到 captures_dir，这里负责列表、下载、删除"""
import io, os, re, time, zipfile
from fastapi import APIRouter, HTTPException
from fastapi.responses import FileResponse, Response
from pydantic import BaseModel
from config import CAPTURE_DIR

router = APIRouter(prefix="/api/captures")

_NAME = re.compile(r"^[\w.-]+\.jpe?g$")


def _path(name: str) -> str:
    # 只认目录下的平铺文件名，挡住 ../ 之类
    if not _NAME.match(name):
        raise HTTPException(400, f"非法文件名: {name}")
    p = os.path.join(CAPTURE_DIR, name)
    if not os.path.isfile(p):
        raise HTTPException(404, f"不存在: {name}")
    return p


def _list() -> list[dict]:
    if not os.path.isdir(CAPTURE_DIR):
        return []
    out = []
    for e in os.scandir(CAPTURE_DIR):
        if e.is_file() and _NAME.match(e.name):
            st = e.stat()
            out.append({"name": e.name, "size": st.st_size, "mtime": st.st_mtime})
    return sorted(out, key=lambda x: x["name"])


class NamesReq(BaseModel):
    names: list[str] | None = None   # None = 全部


@router.get("")
def list_captures():
    items = _list()
    return {"dir": CAPTURE_DIR, "count": len(items), "bytes": sum(i["size"] for i in items),
            "items": items[-200:]}


@router.get("/file/{name}")
def get_capture(name: str):
    return FileResponse(_path(name), media_type="image/jpeg", filename=name)


@router.get("/zip")
def zip_captures():
    items = _list()
    if not items:
        raise HTTPException(404, "没有照片")
    buf = io.BytesIO()
    # jpeg 已经压缩过，不再 deflate
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_STORED) as z:
        for i in items:
            z.write(os.path.join(CAPTURE_DIR, i["name"]), i["name"])
    fn = time.strftime("captures_%Y%m%d_%H%M%S.zip")
    return Response(buf.getvalue(), media_type="application/zip",
                    headers={"Content-Disposition": f'attachment; filename="{fn}"'})


@router.post("/delete")
def delete_captures(req: NamesReq):
    names = req.names if req.names is not None else [i["name"] for i in _list()]
    n = 0
    for name in names:
        try:
            os.remove(_path(name))
            n += 1
        except HTTPException:
            pass
    return {"ok": True, "deleted": n}
