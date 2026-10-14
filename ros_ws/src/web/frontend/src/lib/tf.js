// 2D TF 树：订阅 /tf /tf_static，只保留 x/y/yaw（平面机器人，roll/pitch 忽略）
import { subscribe, yawOf } from './ros.js'

const edges = new Map()   // child -> {parent, x, y, yaw, t}

export function compose(a, b) {
  const c = Math.cos(a.yaw), s = Math.sin(a.yaw)
  return { x: a.x + c * b.x - s * b.y, y: a.y + s * b.x + c * b.y, yaw: a.yaw + b.yaw }
}

export function invert(a) {
  const c = Math.cos(a.yaw), s = Math.sin(a.yaw)
  return { x: -(c * a.x + s * a.y), y: -(-s * a.x + c * a.y), yaw: -a.yaw }
}

export function apply(T, x, y) {
  const c = Math.cos(T.yaw), s = Math.sin(T.yaw)
  return [T.x + c * x - s * y, T.y + s * x + c * y]
}

function onTf(msg) {
  const now = performance.now()
  for (const tr of msg.transforms) {
    const child = tr.child_frame_id.replace(/^\//, '')
    const parent = tr.header.frame_id.replace(/^\//, '')
    const t = tr.transform
    edges.set(child, { parent, x: t.translation.x, y: t.translation.y, yaw: yawOf(t.rotation), t: now })
  }
}

subscribe('/tf', 'tf2_msgs/TFMessage', onTf)
subscribe('/tf_static', 'tf2_msgs/TFMessage', onTf)

/** frame 在其根坐标系下的位姿 {root, pose} */
function toRoot(frame) {
  let pose = { x: 0, y: 0, yaw: 0 }
  let cur = frame
  for (let i = 0; i < 32; i++) {
    const e = edges.get(cur)
    if (!e) return { root: cur, pose }
    pose = compose(e, pose)
    cur = e.parent
  }
  return null   // 有环
}

/** source 系在 target 系下的位姿（即 T_target_source），查不到返回 null */
export function lookup(target, source) {
  target = target.replace(/^\//, '')
  source = (source || target).replace(/^\//, '')
  if (target === source) return { x: 0, y: 0, yaw: 0 }
  const a = toRoot(target), b = toRoot(source)
  if (!a || !b || a.root !== b.root) return null
  return compose(invert(a.pose), b.pose)
}

/** 某条边多久没更新（ms），用于提示 TF 断了 */
export function edgeAge(child) {
  const e = edges.get(child)
  return e ? performance.now() - e.t : Infinity
}

export function frames() {
  const s = new Set()
  edges.forEach((e, c) => { s.add(c); s.add(e.parent) })
  return [...s].sort()
}
