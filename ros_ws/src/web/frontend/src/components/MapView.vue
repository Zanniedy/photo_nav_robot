<template>
  <div class="view" ref="wrap">
    <canvas ref="cv"
            @pointerdown="onDown" @pointermove="onMove" @pointerup="onUp" @pointercancel="onUp"
            @wheel.prevent="onWheel" @contextmenu.prevent />

    <div class="toolbar">
      <div class="group">
        <button v-for="t in TOOLS" :key="t.id" :class="{ on: store.tool === t.id }"
                :title="t.tip" @click="store.tool = t.id">{{ t.label }}</button>
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

const TOOLS = [
  { id: 'pan', label: '✋ 平移', tip: '左键拖动平移，滚轮缩放（任何工具下右键也可平移）' },
  { id: 'initpose', label: '📍 初始位姿', tip: '按下定位置，拖动定朝向 → /initialpose' },
  { id: 'goal', label: '🎯 导航目标', tip: '按下定位置，拖动定朝向 → /goal_pose' },
  { id: 'waypoint', label: '➕ 航点', tip: '添加航点，在右侧面板执行' },
]
const LAYERS = [
  { id: 'grid', label: '1m 网格', color: '#556' },
  { id: 'map', label: '地图 /map', color: '#ddd', topic: '/map' },
  { id: 'globalCost', label: '全局 costmap', color: '#e2c', topic: '/global_costmap/costmap' },
  { id: 'localCost', label: '局部 costmap', color: '#2dd', topic: '/local_costmap/costmap' },
  { id: 'scan', label: '激光 /scan', color: '#f44', topic: '/scan' },
  { id: 'plan', label: '路径 /plan', color: '#4f4', topic: '/plan' },
  { id: 'trail', label: '轨迹', color: '#fa0' },
]
const ROBOT_RADIUS = 0.28

const wrap = ref(null)
const cv = ref(null)
const showLayers = ref(false)
const cursor = ref(null)
const dragHint = ref('')
const robotVisible = ref(true)
const frameList = ref([])
const view = reactive({ cx: 0, cy: 0, scale: 40 })   // scale: px / m
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
const w2s = (x, y) => [W / 2 + (x - view.cx) * view.scale, H / 2 - (y - view.cy) * view.scale]
const s2w = (sx, sy) => ({ x: view.cx + (sx - W / 2) / view.scale, y: view.cy - (sy - H / 2) / view.scale })

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

watch(() => store.fixedFrame, () => { trail = []; dirty = true })
watch(() => [store.layers, store.waypoints, store.goal, store.tool], () => { dirty = true }, { deep: true })

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

function fitMap() {
  const d = layersData.map
  if (!d) { toast('还没收到 /map', 'error'); return false }
  const o = gridOrigin(d)
  if (!o) return false
  const { width: w, height: h, resolution: r } = d.info
  const corners = [[0, 0], [w * r, 0], [0, h * r], [w * r, h * r]].map(([x, y]) => apply(o, x, y))
  const xs = corners.map((c) => c[0]), ys = corners.map((c) => c[1])
  const bw = Math.max(...xs) - Math.min(...xs), bh = Math.max(...ys) - Math.min(...ys)
  view.cx = (Math.max(...xs) + Math.min(...xs)) / 2
  view.cy = (Math.max(...ys) + Math.min(...ys)) / 2
  view.scale = Math.max(5, Math.min(400, 0.9 * Math.min(W / bw, H / bh)))
  dirty = true
  return true
}

function onWheel(e) {
  const [sx, sy] = evtPos(e)
  const before = s2w(sx, sy)
  const k = Math.exp(-e.deltaY * 0.0015)
  view.scale = Math.max(2, Math.min(800, view.scale * k))
  const after = s2w(sx, sy)
  view.cx += before.x - after.x
  view.cy += before.y - after.y
  dirty = true
}

// ---------- 鼠标工具 ----------
let drag = null   // {kind:'pan'|'pose', sx, sy, cx, cy, start:{x,y}, cur:{x,y}}

function onDown(e) {
  const [sx, sy] = evtPos(e)
  cv.value.setPointerCapture(e.pointerId)
  if (e.button === 1 || e.button === 2 || (e.button === 0 && store.tool === 'pan')) {
    drag = { kind: 'pan', sx, sy, cx: view.cx, cy: view.cy }
    store.follow = false
  } else if (e.button === 0) {
    const p = s2w(sx, sy)
    drag = { kind: 'pose', start: p, cur: p, sx, sy }
    dragHint.value = '拖动设置朝向，松开确认'
  }
}

function onMove(e) {
  const [sx, sy] = evtPos(e)
  cursor.value = s2w(sx, sy)
  if (!drag) return
  if (drag.kind === 'pan') {
    view.cx = drag.cx - (sx - drag.sx) / view.scale
    view.cy = drag.cy + (sy - drag.sy) / view.scale
  } else {
    drag.cur = s2w(sx, sy)
  }
  dirty = true
}

function onUp(e) {
  if (!drag) return
  const d = drag
  drag = null
  dragHint.value = ''
  dirty = true
  if (d.kind !== 'pose' || e.type === 'pointercancel') return
  const [sx, sy] = evtPos(e)
  const moved = Math.hypot(sx - d.sx, sy - d.sy) > 8
  let yaw
  if (moved) yaw = Math.atan2(d.cur.y - d.start.y, d.cur.x - d.start.x)
  else if (store.robot && store.tool !== 'initpose')   // 点一下：朝向沿机器人→目标
    yaw = Math.atan2(d.start.y - store.robot.y, d.start.x - store.robot.x)
  else yaw = store.robot ? store.robot.yaw : 0
  firePose(d.start.x, d.start.y, yaw)
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
  const s = view.scale, r = d.info.resolution, h = d.info.height
  const c = Math.cos(o.yaw), sn = Math.sin(o.yaw)
  ctx.save()
  ctx.globalAlpha = alpha
  ctx.imageSmoothingEnabled = false
  ctx.setTransform(
    dpr * s * r * c, -dpr * s * r * sn,
    dpr * s * r * sn, dpr * s * r * c,
    dpr * (W / 2 + s * (o.x - view.cx) - s * r * sn * h),
    dpr * (H / 2 - s * (o.y - view.cy) - s * r * c * h))
  ctx.drawImage(d.img, 0, 0)
  ctx.restore()
}

function drawGrid(ctx) {
  const step = view.scale < 8 ? 10 : 1
  const tl = s2w(0, 0), br = s2w(W, H)
  ctx.strokeStyle = 'rgba(120,130,160,0.18)'
  ctx.lineWidth = 1
  ctx.beginPath()
  for (let x = Math.ceil(tl.x / step) * step; x <= br.x; x += step) {
    const [sx] = w2s(x, 0); ctx.moveTo(sx, 0); ctx.lineTo(sx, H)
  }
  for (let y = Math.ceil(br.y / step) * step; y <= tl.y; y += step) {
    const [, sy] = w2s(0, y); ctx.moveTo(0, sy); ctx.lineTo(W, sy)
  }
  ctx.stroke()
  // fixedFrame 原点坐标轴
  const [ox, oy] = w2s(0, 0)
  ctx.lineWidth = 2
  ctx.strokeStyle = '#e44'; ctx.beginPath(); ctx.moveTo(ox, oy); ctx.lineTo(ox + 0.5 * view.scale, oy); ctx.stroke()
  ctx.strokeStyle = '#4e4'; ctx.beginPath(); ctx.moveTo(ox, oy); ctx.lineTo(ox, oy - 0.5 * view.scale); ctx.stroke()
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
  ctx.rotate(-yaw)
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
  ctx.rotate(-T.yaw)
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

  if (robot) drawRobot(ctx, robot)

  if (drag && drag.kind === 'pose') {
    const color = { initpose: '#3ef06a', goal: '#ff3d7f', waypoint: '#c878ff' }[store.tool]
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
.rate { margin-left: auto; padding-left: 10px; color: var(--muted); font-variant-numeric: tabular-nums; }
.hud { position: absolute; left: 8px; bottom: 8px; display: flex; gap: 12px; font-size: 12px; color: var(--muted); background: rgba(20,22,30,0.75); padding: 3px 8px; border-radius: 4px; font-variant-numeric: tabular-nums; pointer-events: none; }
.warn { color: var(--warn); }
.center-msg { position: absolute; top: 50%; left: 50%; transform: translate(-50%, -50%); color: var(--muted); }
.drag-hint { position: absolute; bottom: 40px; left: 50%; transform: translateX(-50%); background: rgba(0,0,0,0.6); color: #ffd23f; padding: 4px 12px; border-radius: 6px; font-size: 12px; pointer-events: none; }
</style>
