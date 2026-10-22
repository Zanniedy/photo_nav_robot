<template>
  <div class="view" ref="wrap">
    <canvas ref="cv"
            @pointerdown="onDown" @pointermove="onMove" @pointerup="onUp" @pointercancel="onUp"
            @wheel.prevent="onWheel" @contextmenu.prevent @dblclick="onDblClick" />

    <div class="toolbar">
      <div class="group">
        <button v-for="t in TOOLS" :key="t.id" :class="{ on: store.tool === t.id }"
                :title="t.tip" @click="store.tool = t.id">{{ t.label }}</button>
      </div>
      <div class="group">
        <button v-for="t in ANNOT_TOOLS" :key="t.id" :class="{ on: store.tool === t.id }"
                :title="t.tip" @click="store.tool = t.id">{{ t.label }}</button>
      </div>
      <div v-if="store.tool === 'draw'" class="group shapes-pick">
        <button v-for="t in SHAPE_TYPES" :key="t.id" :class="{ on: store.drawType === t.id }"
                :title="t.label" :aria-label="t.label" @click="setDrawType(t.id)">{{ t.icon }}</button>
        <input type="color" v-model="annot.style.color" title="颜色" aria-label="颜色" />
        <select v-model.number="annot.style.width" title="线宽" aria-label="线宽">
          <option v-for="w in [1, 2, 3, 5, 8]" :key="w" :value="w">{{ w }}px</option>
        </select>
        <label class="chk inline" title="闭合形状填充"><input type="checkbox" v-model="annot.style.fill" /> 填充</label>
      </div>
      <label v-if="store.tool === 'initpose'" class="chk" title="经 relocalize_node 扫描精配后再发 /initialpose">
        <input type="checkbox" v-model="store.refine" /> 精配
      </label>
      <div class="group">
        <button :class="{ on: store.follow }" @click="store.follow = !store.follow" title="视图跟随机器人">跟随</button>
        <button @click="centerRobot" title="居中到机器人">居中</button>
        <button @click="fitMap" title="缩放到整张地图">全图</button>
      </div>
      <div class="group">
        <button @click="rotateBy(90)" title="视图逆时针转 90°（Alt+滚轮细调 5°）" aria-label="逆时针旋转地图">⟲</button>
        <button class="deg" @click="setRot(0)" :title="'当前视图旋转 ' + rotDeg + '°，点击复位'">{{ rotDeg }}°</button>
        <button @click="rotateBy(-90)" title="视图顺时针转 90°（Alt+滚轮细调 5°）" aria-label="顺时针旋转地图">⟳</button>
      </div>
      <div class="group">
        <select v-model="store.fixedFrame" title="Fixed Frame">
          <option v-for="f in frameOptions" :key="f" :value="f">{{ f }}</option>
        </select>
        <button :class="{ on: showLayers }" @click="showLayers = !showLayers">图层 ▾</button>
      </div>
      <div v-if="showLayers" class="layers">
        <label v-for="l in LAYERS" :key="l.id" class="chk">
          <input type="checkbox" v-model="store.layers[l.id]" @change="saveLayers" />
          <span class="swatch" :style="{ background: l.color }" /> {{ l.label }}
          <span class="rate">{{ rateOf(l.topic) }}</span>
        </label>
      </div>
    </div>

    <div class="hud">
      <span v-if="cursor">光标 ({{ cursor.x.toFixed(2) }}, {{ cursor.y.toFixed(2) }})</span>
      <span>{{ (100 / view.scale).toFixed(1) }} cm/px</span>
      <span v-if="!robotVisible" class="warn">查不到 {{ store.fixedFrame }} → base_footprint</span>
      <span v-if="annot.saving">保存中…</span><span v-else-if="annot.dirty" class="warn">未保存</span>
    </div>
    <div v-if="!rosState.connected" class="center-msg">等待 rosbridge 连接 {{ rosState.url }} …</div>
    <div v-if="dragHint" class="drag-hint">{{ dragHint }}</div>
  </div>
</template>

<script setup>
import { ref, reactive, computed, onMounted, onUnmounted, watch } from 'vue'
import { rosState, subscribe, publish, yawOf, quatOf } from '../lib/ros.js'
import { lookup, compose, apply, frames } from '../lib/tf.js'
import { gridToCanvas, mapColor, costColor } from '../lib/grid.js'
import { store, saveLayers, toast } from '../lib/store.js'
import { annot, SHAPE_TYPES, addShape, updateShape, removeShape, beginEdit, addPoint, editPoint, deletePoint, undo } from '../lib/annot.js'
import { drawShape, drawPoint, hitShape, hitPoint } from '../lib/draw.js'
import { person, personOnline } from '../lib/person.js'

const TOOLS = [
  { id: 'pan', label: '✋ 平移', tip: '左键拖动平移，滚轮缩放（任何工具下右键也可平移）' },
  { id: 'initpose', label: '📍 初始位姿', tip: '按下定位置，拖动定朝向 → /initialpose' },
  { id: 'goal', label: '🎯 导航目标', tip: '按下定位置，拖动定朝向 → /goal_pose' },
  { id: 'waypoint', label: '➕ 航点', tip: '添加航点，在右侧面板执行' },
]
const ANNOT_TOOLS = [
  { id: 'draw', label: '✏️ 绘图', tip: '在当前地图上画标注，自动保存。折线/多边形：单击加点，双击或回车结束，Esc 取消' },
  { id: 'point', label: '📌 标点', tip: '按下定位置，拖动定朝向，然后输入点名' },
  { id: 'select', label: '⬚ 选择', tip: '点选形状/点位，拖动移动，Delete 删除，Ctrl+Z 撤销' },
  { id: 'erase', label: '🧽 擦除', tip: '点击形状删除（点位会先确认）' },
]
const LAYERS = [
  { id: 'grid', label: '1m 网格', color: '#556' },
  { id: 'map', label: '地图 /map', color: '#ddd', topic: '/map' },
  { id: 'globalCost', label: '全局 costmap', color: '#e2c', topic: '/global_costmap/costmap' },
  { id: 'localCost', label: '局部 costmap', color: '#2dd', topic: '/local_costmap/costmap' },
  { id: 'scan', label: '激光 /scan', color: '#f44', topic: '/scan' },
  { id: 'plan', label: '路径 /plan', color: '#4f4', topic: '/plan' },
  { id: 'trail', label: '轨迹', color: '#fa0' },
  { id: 'shapes', label: '绘图标注', color: '#fc0' },
  { id: 'points', label: '点位', color: '#2ee6a6' },
  { id: 'person', label: '人员跟踪（红点=记住的位置）', color: '#ff2a2a', topic: '/person/state' },
]
const ROBOT_RADIUS = 0.28

const wrap = ref(null)
const cv = ref(null)
const showLayers = ref(false)
const cursor = ref(null)
const dragHint = ref('')
const robotVisible = ref(true)
const frameList = ref([])
// rot：视图旋转（rad，逆时针为正），只影响显示，不改任何坐标数据
const ROT_KEY = 'robot-ui-maprot'
const readRot = () => { try { return Number(localStorage.getItem(ROT_KEY)) || 0 } catch { return 0 } }
const view = reactive({ cx: 0, cy: 0, scale: 40, rot: readRot() })   // scale: px / m
const rotDeg = computed(() => { const d = Math.round(view.rot * 180 / Math.PI) % 360; return d > 180 ? d - 360 : d <= -180 ? d + 360 : d })
const frameOptions = computed(() => {
  const s = new Set(['map', 'lio_odom', ...frameList.value])
  s.add(store.fixedFrame)
  return [...s]
})

// 最新数据
const layersData = { map: null, globalCost: null, localCost: null }   // {img, info, frame}
let scan = null, plan = null, localPlan = null
let trail = []
let W = 0, H = 0, dpr = 1
let dirty = true, raf = 0
const unsubs = []

function rateOf(topic) {
  if (!topic) return ''
  const r = rosState.rates[topic]
  return r ? `${r.toFixed(r < 2 ? 1 : 0)}Hz` : ''
}

// ---------- 坐标变换 ----------
// 世界偏移先转 rot 再缩放；屏幕 y 向下
function w2s(x, y) {
  const c = Math.cos(view.rot), sn = Math.sin(view.rot), dx = x - view.cx, dy = y - view.cy
  return [W / 2 + (c * dx - sn * dy) * view.scale, H / 2 - (sn * dx + c * dy) * view.scale]
}
/** 屏幕位移 → 世界位移 */
function s2wDelta(dsx, dsy) {
  const c = Math.cos(view.rot), sn = Math.sin(view.rot), u = dsx / view.scale, v = -dsy / view.scale
  return { x: c * u + sn * v, y: -sn * u + c * v }
}
function s2w(sx, sy) {
  const d = s2wDelta(sx - W / 2, sy - H / 2)
  return { x: view.cx + d.x, y: view.cy + d.y }
}

function evtPos(e) {
  const r = cv.value.getBoundingClientRect()
  return [e.clientX - r.left, e.clientY - r.top]
}

// ---------- 订阅 ----------
function gridSub(key, topic, colorOf, throttle) {
  unsubs.push(subscribe(topic, 'nav_msgs/OccupancyGrid', (msg) => {
    layersData[key] = { img: gridToCanvas(msg, colorOf), info: msg.info, frame: msg.header.frame_id || 'map' }
    dirty = true
  }, throttle ? { throttle_rate: throttle } : {}))
}

let firstMap = true
onMounted(() => {
  gridSub('map', '/map', mapColor, 0)
  gridSub('globalCost', '/global_costmap/costmap', costColor, 2000)
  gridSub('localCost', '/local_costmap/costmap', costColor, 500)
  unsubs.push(subscribe('/scan', 'sensor_msgs/LaserScan', (m) => { scan = m; dirty = true },
    { throttle_rate: 100, queue_length: 1 }))
  unsubs.push(subscribe('/plan', 'nav_msgs/Path', (m) => { plan = m; dirty = true }, { throttle_rate: 500 }))
  unsubs.push(subscribe('/local_plan', 'nav_msgs/Path', (m) => { localPlan = m; dirty = true }, { throttle_rate: 300 }))
  unsubs.push(subscribe('/tf', 'tf2_msgs/TFMessage', () => { dirty = true }))

  const ro = new ResizeObserver(resize)
  ro.observe(wrap.value)
  unsubs.push(() => ro.disconnect())
  resize()
  raf = requestAnimationFrame(loop)

  // 帧列表 + 机器人位姿同步到 store（5Hz，避免每帧触发 Vue 更新）
  const t = setInterval(() => {
    frameList.value = frames()
    const T = lookup(store.fixedFrame, 'base_footprint')
    robotVisible.value = !!T
    store.robot = T
  }, 200)
  unsubs.push(() => clearInterval(t))
})

onUnmounted(() => { cancelAnimationFrame(raf); unsubs.forEach((u) => u()) })
onMounted(() => window.addEventListener('keydown', onKey))
onUnmounted(() => window.removeEventListener('keydown', onKey))

watch(() => store.fixedFrame, () => { trail = []; dirty = true })
watch(() => [store.layers, store.waypoints, store.goal, store.tool, store.drawType], () => { dirty = true; if (store.tool !== 'draw') cancelDraft() }, { deep: true })
watch(() => [annot.shapes, annot.points, annot.selected, annot.selectedPoint], () => { dirty = true }, { deep: true })
watch(() => person.t, () => { dirty = true })

// 第一次收到地图时自动缩放到全图
watch(() => rosState.connected, () => { firstMap = true })

function resize() {
  dpr = window.devicePixelRatio || 1
  W = wrap.value.clientWidth
  H = wrap.value.clientHeight
  cv.value.width = Math.round(W * dpr)
  cv.value.height = Math.round(H * dpr)
  cv.value.style.width = `${W}px`
  cv.value.style.height = `${H}px`
  dirty = true
}

// ---------- 视图操作 ----------
function centerRobot() {
  const T = lookup(store.fixedFrame, 'base_footprint')
  if (T) { view.cx = T.x; view.cy = T.y; dirty = true } else toast('查不到机器人位姿', 'error')
}

function setRot(deg) {
  view.rot = deg * Math.PI / 180
  try { localStorage.setItem(ROT_KEY, String(view.rot)) } catch { /* 无痕模式 */ }
  dirty = true
}
function rotateBy(deg) { setRot(Math.round((view.rot * 180 / Math.PI + deg) * 1e6) / 1e6) }

function fitMap() {
  const d = layersData.map
  if (!d) { toast('还没收到 /map', 'error'); return false }
  const o = gridOrigin(d)
  if (!o) return false
  const { width: w, height: h, resolution: r } = d.info
  const corners = [[0, 0], [w * r, 0], [0, h * r], [w * r, h * r]].map(([x, y]) => apply(o, x, y))
  const xs = corners.map((c) => c[0]), ys = corners.map((c) => c[1])
  view.cx = (Math.max(...xs) + Math.min(...xs)) / 2
  view.cy = (Math.max(...ys) + Math.min(...ys)) / 2
  // 按旋转后的屏幕方向量包围盒
  const c = Math.cos(view.rot), sn = Math.sin(view.rot)
  const us = corners.map(([x, y]) => c * x - sn * y), vs = corners.map(([x, y]) => sn * x + c * y)
  const bw = Math.max(...us) - Math.min(...us), bh = Math.max(...vs) - Math.min(...vs)
  view.scale = Math.max(5, Math.min(400, 0.9 * Math.min(W / bw, H / bh)))
  dirty = true
  return true
}

function onWheel(e) {
  const [sx, sy] = evtPos(e)
  const before = s2w(sx, sy)
  if (e.altKey) {
    // Alt+滚轮：绕光标处旋转 5°
    rotateBy(e.deltaY > 0 ? -5 : 5)
  } else {
    const k = Math.exp(-e.deltaY * 0.0015)
    view.scale = Math.max(2, Math.min(800, view.scale * k))
  }
  const after = s2w(sx, sy)
  view.cx += before.x - after.x
  view.cy += before.y - after.y
  dirty = true
}

// ---------- 鼠标工具 ----------
// drag.kind: pan | pose(初始位姿/目标/航点/标点) | shape(两点形状/手绘) | move(选择后拖动)
let drag = null
let draft = null   // 折线/多边形绘制中 {type, pts:[[x,y]...]}（map 系）
const MIN_DRAG_PX = 4

/** 屏幕 px → map 系；查不到 map TF 返回 null */
function screenToMap(sx, sy) {
  const w = s2w(sx, sy)
  if (store.fixedFrame === 'map') return [w.x, w.y]
  const T = lookup('map', store.fixedFrame)
  return T ? apply(T, w.x, w.y) : null
}
/** map 系 → 屏幕 px 的函数；查不到返回 null */
function mapToScreenFn() {
  const T = lookup(store.fixedFrame, 'map')
  if (!T) return null
  const f = (x, y) => { const [wx, wy] = apply(T, x, y); return w2s(wx, wy) }
  f.yaw = (yaw) => -(yaw + T.yaw + view.rot)
  return f
}

function setDrawType(t) { cancelDraft(); store.drawType = t }
function cancelDraft() { if (draft) { draft = null; dirty = true } }
function finishDraft() {
  if (!draft) return
  const need = draft.type === 'polygon' ? 3 : 2
  if (draft.pts.length >= need) addShape({ type: draft.type, pts: draft.pts })
  draft = null
  dirty = true
}

/** 屏幕位置下最上层的点位/形状 */
function pick(sx, sy) {
  const toS = mapToScreenFn()
  if (!toS) return null
  if (store.layers.points) {
    for (let i = annot.points.length - 1; i >= 0; i--) if (hitPoint(annot.points[i], toS, sx, sy)) return { kind: 'point', item: annot.points[i] }
  }
  if (store.layers.shapes) {
    const ctx = cv.value.getContext('2d')
    for (let i = annot.shapes.length - 1; i >= 0; i--) {
      const s = annot.shapes[i]
      if (!s.hidden && hitShape(ctx, s, toS, sx, sy)) return { kind: 'shape', item: s }
    }
  }
  return null
}

function needMap() {
  if (!annot.map) { toast('标注绑定到地图：先进入导航模式', 'error'); return false }
  if (!mapToScreenFn()) { toast('查不到 map TF', 'error'); return false }
  return true
}

function onDown(e) {
  const [sx, sy] = evtPos(e)
  if (e.button === 1 || e.button === 2 || (e.button === 0 && store.tool === 'pan')) {
    cv.value.setPointerCapture(e.pointerId)
    drag = { kind: 'pan', sx, sy, cx: view.cx, cy: view.cy }
    store.follow = false
    return
  }
  if (e.button !== 0) return
  const tool = store.tool
  if (['draw', 'point', 'select', 'erase'].includes(tool) && !needMap()) return
  const mp = screenToMap(sx, sy)

  if (tool === 'select') {
    const hit = pick(sx, sy)
    annot.selected = hit?.kind === 'shape' ? hit.item.id : null
    annot.selectedPoint = hit?.kind === 'point' ? hit.item.id : null
    if (hit) {
      cv.value.setPointerCapture(e.pointerId)
      if (hit.kind === 'shape') beginEdit()
      drag = { kind: 'move', hit, start: mp, orig: hit.kind === 'shape' ? hit.item.pts.map((p) => [...p]) : { x: hit.item.x, y: hit.item.y }, sx, sy, moved: false }
    }
    dirty = true
    return
  }
  if (tool === 'erase') {
    const hit = pick(sx, sy)
    if (hit?.kind === 'shape') removeShape(hit.item.id)
    else if (hit?.kind === 'point' && confirm(`删除点位「${hit.item.name}」？`)) deletePoint(hit.item.id)
    return
  }
  if (tool === 'draw') {
    const t = store.drawType
    if (t === 'polyline' || t === 'polygon') {
      if (!draft) draft = { type: t, pts: [] }
      // 多边形点回起点附近结束
      const toS = mapToScreenFn()
      if (t === 'polygon' && draft.pts.length >= 3) {
        const [fx, fy] = toS(...draft.pts[0])
        if (Math.hypot(fx - sx, fy - sy) < 10) { finishDraft(); return }
      }
      draft.pts.push(mp)
      dirty = true
      return
    }
    if (t === 'text') {
      const text = prompt('文字内容')
      if (text && text.trim()) addShape({ type: 'text', pts: [mp], text: text.trim().slice(0, 200) })
      return
    }
    cv.value.setPointerCapture(e.pointerId)
    drag = { kind: 'shape', type: t, pts: [mp, mp], sx, sy }
    return
  }
  // initpose / goal / waypoint / point：按下定位置，拖动定朝向
  cv.value.setPointerCapture(e.pointerId)
  const p = s2w(sx, sy)
  drag = { kind: 'pose', start: p, cur: p, sx, sy }
  dragHint.value = '拖动设置朝向，松开确认'
}

function onMove(e) {
  const [sx, sy] = evtPos(e)
  cursor.value = s2w(sx, sy)
  if (draft) dirty = true   // 预览线跟着光标
  if (!drag) return
  if (drag.kind === 'pan') {
    const d = s2wDelta(sx - drag.sx, sy - drag.sy)
    view.cx = drag.cx - d.x
    view.cy = drag.cy - d.y
  } else if (drag.kind === 'pose') {
    drag.cur = s2w(sx, sy)
  } else if (drag.kind === 'shape') {
    const mp = screenToMap(sx, sy)
    if (!mp) return
    if (drag.type === 'freehand') {
      // 屏幕上移动超过 3px 才记一个点，控制点数
      const toS = mapToScreenFn()
      const [lx, ly] = toS(...drag.pts[drag.pts.length - 1])
      if (Math.hypot(lx - sx, ly - sy) > 3) drag.pts.push(mp)
    } else {
      drag.pts[1] = mp
    }
  } else if (drag.kind === 'move') {
    const mp = screenToMap(sx, sy)
    if (!mp || !drag.start) return
    if (!drag.moved && Math.hypot(sx - drag.sx, sy - drag.sy) < MIN_DRAG_PX) return
    drag.moved = true
    const dx = mp[0] - drag.start[0], dy = mp[1] - drag.start[1]
    if (drag.hit.kind === 'shape') {
      updateShape(drag.hit.item.id, { pts: drag.orig.map(([x, y]) => [x + dx, y + dy]) }, { undoable: false })
    } else {
      // 点位拖动只改本地显示，松手再写后端
      drag.hit.item.x = drag.orig.x + dx
      drag.hit.item.y = drag.orig.y + dy
    }
  }
  dirty = true
}

function onUp(e) {
  if (!drag) return
  const d = drag
  drag = null
  dragHint.value = ''
  dirty = true
  if (e.type === 'pointercancel') {
    if (d.kind === 'move' && d.hit.kind === 'point') Object.assign(d.hit.item, d.orig)
    return
  }
  const [sx, sy] = evtPos(e)
  const moved = Math.hypot(sx - d.sx, sy - d.sy) > 8

  if (d.kind === 'shape') {
    if (d.type === 'freehand' ? d.pts.length >= 2 : moved) addShape({ type: d.type, pts: d.pts })
    return
  }
  if (d.kind === 'move') {
    if (d.moved && d.hit.kind === 'point') editPoint(d.hit.item.id, { x: d.hit.item.x, y: d.hit.item.y })
    return
  }
  if (d.kind !== 'pose') return
  let yaw
  if (moved) yaw = Math.atan2(d.cur.y - d.start.y, d.cur.x - d.start.x)
  else if (store.robot && store.tool !== 'initpose' && store.tool !== 'point')   // 点一下：朝向沿机器人→目标
    yaw = Math.atan2(d.start.y - store.robot.y, d.start.x - store.robot.x)
  else yaw = store.robot ? store.robot.yaw : 0
  firePose(d.start.x, d.start.y, yaw)
}

function onDblClick() {
  if (draft) {
    // 双击的第二下已经加了一个重复点，去掉
    const n = draft.pts.length
    if (n >= 2) {
      const [a, b] = [draft.pts[n - 1], draft.pts[n - 2]]
      if (Math.hypot(a[0] - b[0], a[1] - b[1]) < 1e-6) draft.pts.pop()
    }
    finishDraft()
  } else if (store.tool === 'select' && annot.selectedPoint) {
    renamePoint(annot.points.find((p) => p.id === annot.selectedPoint))
  } else if (store.tool === 'select' && annot.selected) {
    const s = annot.shapes.find((x) => x.id === annot.selected)
    if (s?.type === 'text') {
      const t = prompt('文字内容', s.text)
      if (t != null && t.trim()) updateShape(s.id, { text: t.trim().slice(0, 200) })
    }
  }
}

async function renamePoint(p) {
  if (!p) return
  const name = prompt('点名', p.name)
  if (name != null && name.trim() && name.trim() !== p.name) await editPoint(p.id, { name: name.trim() })
}

function isTyping(e) { return ['INPUT', 'TEXTAREA', 'SELECT'].includes(e.target.tagName) }
function onKey(e) {
  if (isTyping(e)) return
  if (e.key === 'Escape') { cancelDraft(); annot.selected = annot.selectedPoint = null; dirty = true }
  else if (e.key === 'Enter' && draft) { e.preventDefault(); finishDraft() }
  else if ((e.key === 'Delete' || e.key === 'Backspace') && store.tool === 'select') {
    if (annot.selected) { e.preventDefault(); removeShape(annot.selected) }
    else if (annot.selectedPoint) {
      const p = annot.points.find((x) => x.id === annot.selectedPoint)
      if (p && confirm(`删除点位「${p.name}」？`)) deletePoint(p.id)
    }
  } else if ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === 'z') {
    e.preventDefault()
    if (!undo()) toast('没有可撤销的绘图操作')
  }
}

/** fixedFrame 下的位姿 → map 系（查不到就原样，frame 用 fixedFrame） */
function toMap(x, y, yaw) {
  if (store.fixedFrame === 'map') return { frame: 'map', x, y, yaw }
  const T = lookup('map', store.fixedFrame)
  if (!T) return { frame: store.fixedFrame, x, y, yaw }
  const p = compose(T, { x, y, yaw })
  return { frame: 'map', ...p }
}

function firePose(x, y, yaw) {
  const p = toMap(x, y, yaw)
  const pose = { position: { x: p.x, y: p.y, z: 0 }, orientation: quatOf(p.yaw) }
  const header = { frame_id: p.frame }
  const deg = (p.yaw * 180 / Math.PI).toFixed(0)
  if (store.tool === 'initpose') {
    const cov = new Array(36).fill(0)
    cov[0] = cov[7] = 0.25
    cov[35] = 0.0685
    const topic = store.refine ? '/initialpose_rough' : '/initialpose'
    if (publish(topic, 'geometry_msgs/PoseWithCovarianceStamped', { header, pose: { pose, covariance: cov } }))
      toast(`已发 ${topic} (${p.x.toFixed(2)}, ${p.y.toFixed(2)}, ${deg}°)`)
  } else if (store.tool === 'goal') {
    if (p.frame !== 'map') { toast('目标必须在 map 系，当前查不到 map TF', 'error'); return }
    if (publish('/goal_pose', 'geometry_msgs/PoseStamped', { header, pose })) {
      store.goal = { x: p.x, y: p.y, yaw: p.yaw }
      toast(`已发 /goal_pose (${p.x.toFixed(2)}, ${p.y.toFixed(2)}, ${deg}°)`)
    }
  } else if (store.tool === 'waypoint') {
    if (p.frame !== 'map') { toast('航点必须在 map 系', 'error'); return }
    store.waypoints.push({ x: p.x, y: p.y, yaw: p.yaw })
  } else if (store.tool === 'point') {
    if (p.frame !== 'map') { toast('点位必须在 map 系', 'error'); return }
    const name = prompt(`点名（${p.x.toFixed(2)}, ${p.y.toFixed(2)}, ${deg}°）`, `P${annot.points.length + 1}`)
    if (name && name.trim()) addPoint({ name: name.trim(), x: p.x, y: p.y, yaw: p.yaw }).then((r) => { if (r) annot.selectedPoint = r.id })
  }
}

// ---------- 渲染 ----------
function loop() {
  raf = requestAnimationFrame(loop)
  if (store.follow) {
    const T = lookup(store.fixedFrame, 'base_footprint')
    if (T && (Math.abs(T.x - view.cx) > 1e-3 || Math.abs(T.y - view.cy) > 1e-3)) {
      view.cx = T.x; view.cy = T.y; dirty = true
    }
  }
  if (firstMap && layersData.map && W > 0 && lookup(store.fixedFrame, layersData.map.frame)) {
    firstMap = !fitMap()
  }
  if (!dirty) return
  dirty = false
  draw()
}

function gridOrigin(d) {
  const T = lookup(store.fixedFrame, d.frame)
  if (!T) return null
  const o = d.info.origin
  return compose(T, { x: o.position.x, y: o.position.y, yaw: yawOf(o.orientation) })
}

function drawGridLayer(ctx, d, alpha = 1) {
  if (!d) return
  const o = gridOrigin(d)
  if (!o) return
  const r = d.info.resolution, h = d.info.height
  // 图像像素 (u, v) → 栅格系 (u·r, (h−v)·r)（图像第 0 行是 y 最大），再经 origin、视图到屏幕；仿射，取三点定矩阵
  const P = (u, v) => w2s(...apply(o, u * r, (h - v) * r))
  const [ex, ey] = P(0, 0), [ax, ay] = P(1, 0), [cx, cy] = P(0, 1)
  ctx.save()
  ctx.globalAlpha = alpha
  ctx.imageSmoothingEnabled = false
  ctx.setTransform(dpr * (ax - ex), dpr * (ay - ey), dpr * (cx - ex), dpr * (cy - ey), dpr * ex, dpr * ey)
  ctx.drawImage(d.img, 0, 0)
  ctx.restore()
}

function drawGrid(ctx) {
  const step = view.scale < 8 ? 10 : 1
  // 视图可能旋转：取屏幕四角的世界包围盒，网格线两端用 w2s 画
  const cs = [s2w(0, 0), s2w(W, 0), s2w(0, H), s2w(W, H)]
  const x0 = Math.min(...cs.map((p) => p.x)), x1 = Math.max(...cs.map((p) => p.x))
  const y0 = Math.min(...cs.map((p) => p.y)), y1 = Math.max(...cs.map((p) => p.y))
  ctx.strokeStyle = 'rgba(120,130,160,0.18)'
  ctx.lineWidth = 1
  ctx.beginPath()
  for (let x = Math.ceil(x0 / step) * step; x <= x1; x += step) {
    ctx.moveTo(...w2s(x, y0)); ctx.lineTo(...w2s(x, y1))
  }
  for (let y = Math.ceil(y0 / step) * step; y <= y1; y += step) {
    ctx.moveTo(...w2s(x0, y)); ctx.lineTo(...w2s(x1, y))
  }
  ctx.stroke()
  // fixedFrame 原点坐标轴
  const [ox, oy] = w2s(0, 0)
  ctx.lineWidth = 2
  ctx.strokeStyle = '#e44'; ctx.beginPath(); ctx.moveTo(ox, oy); ctx.lineTo(...w2s(0.5, 0)); ctx.stroke()
  ctx.strokeStyle = '#4e4'; ctx.beginPath(); ctx.moveTo(ox, oy); ctx.lineTo(...w2s(0, 0.5)); ctx.stroke()
}

function drawPath(ctx, msg, color, width) {
  if (!msg || !msg.poses.length) return
  const T = lookup(store.fixedFrame, msg.header.frame_id || 'map')
  if (!T) return
  ctx.strokeStyle = color
  ctx.lineWidth = width
  ctx.beginPath()
  msg.poses.forEach((p, i) => {
    const [wx, wy] = apply(T, p.pose.position.x, p.pose.position.y)
    const [sx, sy] = w2s(wx, wy)
    i ? ctx.lineTo(sx, sy) : ctx.moveTo(sx, sy)
  })
  ctx.stroke()
}

function drawScan(ctx) {
  if (!scan) return
  const T = lookup(store.fixedFrame, scan.header.frame_id)
  if (!T) return
  const { angle_min: a0, angle_increment: da, range_min: rmin, range_max: rmax, ranges } = scan
  const sz = Math.max(2, Math.min(4, view.scale * 0.03))
  ctx.fillStyle = '#ff4646'
  for (let i = 0; i < ranges.length; i++) {
    const r = ranges[i]
    if (r === null || !(r >= rmin && r <= rmax)) continue
    const a = a0 + i * da
    const [wx, wy] = apply(T, r * Math.cos(a), r * Math.sin(a))
    const [sx, sy] = w2s(wx, wy)
    ctx.fillRect(sx - sz / 2, sy - sz / 2, sz, sz)
  }
}

function drawArrow(ctx, x, y, yaw, color, len = 0.5, label = '') {
  const [sx, sy] = w2s(x, y)
  const L = Math.max(18, len * view.scale)
  ctx.save()
  ctx.translate(sx, sy)
  ctx.rotate(-(yaw + view.rot))
  ctx.strokeStyle = color
  ctx.fillStyle = color
  ctx.lineWidth = 3
  ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(L - 8, 0); ctx.stroke()
  ctx.beginPath(); ctx.moveTo(L, 0); ctx.lineTo(L - 10, -6); ctx.lineTo(L - 10, 6); ctx.closePath(); ctx.fill()
  ctx.beginPath(); ctx.arc(0, 0, 4, 0, Math.PI * 2); ctx.fill()
  ctx.restore()
  if (label) {
    ctx.fillStyle = color
    ctx.font = 'bold 12px system-ui, sans-serif'
    ctx.fillText(label, sx + 6, sy - 6)
  }
}

function drawRobot(ctx, T) {
  const [sx, sy] = w2s(T.x, T.y)
  const R = Math.max(6, ROBOT_RADIUS * view.scale)
  ctx.save()
  ctx.translate(sx, sy)
  ctx.rotate(-(T.yaw + view.rot))
  ctx.fillStyle = 'rgba(40,140,255,0.25)'
  ctx.strokeStyle = '#3a9bff'
  ctx.lineWidth = 2
  ctx.beginPath(); ctx.arc(0, 0, R, 0, Math.PI * 2); ctx.fill(); ctx.stroke()
  ctx.fillStyle = '#3a9bff'
  ctx.beginPath(); ctx.moveTo(R, 0); ctx.lineTo(-R * 0.4, -R * 0.55); ctx.lineTo(-R * 0.4, R * 0.55); ctx.closePath(); ctx.fill()
  ctx.restore()
}

function waypointsInFixed() {
  // 航点/目标存的是 map 系
  const T = lookup(store.fixedFrame, 'map')
  if (!T) return null
  return (p) => compose(T, p)
}

/** 正在画的形状预览 */
function drawAnnotPreview(ctx, toS) {
  const base = { ...annot.style, id: '_', text: '' }
  if (drag && drag.kind === 'shape') drawShape(ctx, { ...base, type: drag.type, pts: drag.pts }, toS, false)
  if (draft && draft.pts.length) {
    const cur = cursor.value && screenToMap(...w2s(cursor.value.x, cursor.value.y))
    const pts = cur ? [...draft.pts, cur] : draft.pts
    drawShape(ctx, { ...base, type: draft.type === 'polygon' && pts.length >= 3 ? 'polygon' : 'polyline', pts }, toS, false)
    ctx.fillStyle = '#fff'
    draft.pts.forEach(([x, y]) => { const [a, b] = toS(x, y); ctx.fillRect(a - 2.5, b - 2.5, 5, 5) })
  }
}

/** person_tracker 的轨迹（橙圈，目标绿圈）、记住的位置（红点）、导航目标（箭头） */
function drawPerson(ctx) {
  const st = person.state
  if (!st || !personOnline()) return
  const T = lookup(store.fixedFrame, st.frame)
  if (!T) return
  const toS = (x, y) => w2s(...apply(T, x, y))
  const R = Math.max(6, 0.25 * view.scale)
  ctx.save()
  ctx.font = '12px sans-serif'
  ctx.lineWidth = 2
  for (const t of st.tracks) {
    const [x, y] = toS(t.x, t.y)
    ctx.strokeStyle = t.target ? '#3ef06a' : '#ffa31a'
    ctx.fillStyle = t.target ? 'rgba(62,240,106,0.18)' : 'rgba(255,163,26,0.15)'
    ctx.beginPath(); ctx.arc(x, y, R, 0, Math.PI * 2); ctx.fill(); ctx.stroke()
    ctx.fillStyle = ctx.strokeStyle
    ctx.fillText(`#${t.id}`, x + R + 3, y - R)
  }
  if (st.goal) {
    const g = compose(T, { x: st.goal[0], y: st.goal[1], yaw: st.goal[2] })
    drawArrow(ctx, g.x, g.y, g.yaw, '#ff5ad2', 0.5)
  }
  if (st.mark) {
    const [x, y] = toS(...st.mark.xy)
    ctx.globalAlpha = st.mark.active ? 1 : 0.45
    ctx.fillStyle = '#ff2a2a'
    ctx.strokeStyle = '#fff'
    ctx.beginPath(); ctx.arc(x, y, 7, 0, Math.PI * 2); ctx.fill(); ctx.stroke()
    ctx.fillText(st.mark.active ? '记住' : '上次', x + 10, y + 4)
  }
  ctx.restore()
}

function draw() {
  const ctx = cv.value.getContext('2d')
  ctx.setTransform(1, 0, 0, 1, 0, 0)
  ctx.fillStyle = '#1b1e26'
  ctx.fillRect(0, 0, cv.value.width, cv.value.height)

  const L = store.layers
  if (L.map) drawGridLayer(ctx, layersData.map)
  if (L.globalCost) drawGridLayer(ctx, layersData.globalCost, 0.6)
  if (L.localCost) drawGridLayer(ctx, layersData.localCost, 0.7)

  ctx.setTransform(dpr, 0, 0, dpr, 0, 0)
  if (L.grid) drawGrid(ctx)

  const robot = lookup(store.fixedFrame, 'base_footprint')
  if (robot) {
    const last = trail[trail.length - 1]
    if (!last || Math.hypot(robot.x - last[0], robot.y - last[1]) > 0.03) {
      trail.push([robot.x, robot.y])
      if (trail.length > 5000) trail.shift()
    }
  }
  if (L.trail && trail.length > 1) {
    ctx.strokeStyle = 'rgba(255,170,0,0.7)'
    ctx.lineWidth = 2
    ctx.beginPath()
    trail.forEach(([x, y], i) => { const [sx, sy] = w2s(x, y); i ? ctx.lineTo(sx, sy) : ctx.moveTo(sx, sy) })
    ctx.stroke()
  }
  if (L.plan) {
    drawPath(ctx, plan, '#3ef06a', 3)
    drawPath(ctx, localPlan, '#f0e03e', 2)
  }
  if (L.scan) drawScan(ctx)

  const toS = mapToScreenFn()
  if (toS && L.shapes) {
    annot.shapes.forEach((sh) => { if (!sh.hidden) drawShape(ctx, sh, toS, sh.id === annot.selected) })
  }
  if (toS) drawAnnotPreview(ctx, toS)
  if (toS && L.points) {
    annot.points.forEach((pt, i) => drawPoint(ctx, pt, toS, toS.yaw, pt.id === annot.selectedPoint, i))
  }

  const fromMap = waypointsInFixed()
  if (fromMap) {
    if (store.waypoints.length) {
      ctx.strokeStyle = 'rgba(200,120,255,0.6)'
      ctx.setLineDash([6, 4])
      ctx.lineWidth = 2
      ctx.beginPath()
      store.waypoints.forEach((w, i) => {
        const p = fromMap(w); const [sx, sy] = w2s(p.x, p.y)
        i ? ctx.lineTo(sx, sy) : ctx.moveTo(sx, sy)
      })
      ctx.stroke()
      ctx.setLineDash([])
      store.waypoints.forEach((w, i) => { const p = fromMap(w); drawArrow(ctx, p.x, p.y, p.yaw, '#c878ff', 0.4, `${i + 1}`) })
    }
    if (store.goal) { const p = fromMap(store.goal); drawArrow(ctx, p.x, p.y, p.yaw, '#ff3d7f', 0.5) }
  }

  if (L.person !== false) drawPerson(ctx)
  if (robot) drawRobot(ctx, robot)

  if (drag && drag.kind === 'pose') {
    const color = { initpose: '#3ef06a', goal: '#ff3d7f', waypoint: '#c878ff', point: '#2ee6a6' }[store.tool]
    const yaw = Math.atan2(drag.cur.y - drag.start.y, drag.cur.x - drag.start.x)
    const len = Math.hypot(drag.cur.y - drag.start.y, drag.cur.x - drag.start.x)
    drawArrow(ctx, drag.start.x, drag.start.y, yaw, color, Math.max(len, 0.3))
  }
}
</script>

<style scoped>
.view { position: relative; flex: 1; min-width: 0; overflow: hidden; background: #1b1e26; }
canvas { display: block; touch-action: none; cursor: crosshair; }
.toolbar { position: absolute; top: 8px; left: 8px; right: 8px; display: flex; flex-wrap: wrap; gap: 6px; align-items: flex-start; pointer-events: none; }
.toolbar > * { pointer-events: auto; }
.group { display: flex; background: var(--panel); border: 1px solid var(--line); border-radius: 6px; overflow: hidden; }
.group button, .group select { border: 0; border-radius: 0; border-right: 1px solid var(--line); margin: 0; }
.group > :last-child { border-right: 0; }
.chk { display: flex; align-items: center; gap: 4px; font-size: 12px; background: var(--panel); border: 1px solid var(--line); border-radius: 6px; padding: 4px 8px; cursor: pointer; }
.layers { position: absolute; top: 36px; right: 0; display: flex; flex-direction: column; gap: 2px; background: var(--panel); border: 1px solid var(--line); border-radius: 6px; padding: 6px; }
.layers .chk { border: 0; padding: 3px 4px; }
.swatch { width: 10px; height: 10px; border-radius: 2px; display: inline-block; }
.shapes-pick button { min-width: 30px; }
.group .deg { min-width: 46px; font-variant-numeric: tabular-nums; }
.shapes-pick input[type=color] { width: 34px; padding: 0 2px; border: 0; border-right: 1px solid var(--line); border-radius: 0; height: auto; }
.chk.inline { border: 0; border-radius: 0; }
.rate { margin-left: auto; padding-left: 10px; color: var(--muted); font-variant-numeric: tabular-nums; }
.hud { position: absolute; left: 8px; bottom: 8px; display: flex; gap: 12px; font-size: 12px; color: var(--muted); background: rgba(20,22,30,0.75); padding: 3px 8px; border-radius: 4px; font-variant-numeric: tabular-nums; pointer-events: none; }
.warn { color: var(--warn); }
.center-msg { position: absolute; top: 50%; left: 50%; transform: translate(-50%, -50%); color: var(--muted); pointer-events: none; }
.view { user-select: none; }
.drag-hint { position: absolute; bottom: 40px; left: 50%; transform: translateX(-50%); background: rgba(0,0,0,0.6); color: #ffd23f; padding: 4px 12px; border-radius: 6px; font-size: 12px; pointer-events: none; }
</style>
