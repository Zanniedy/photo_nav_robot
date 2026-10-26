// 全局共享状态：后端进程状态轮询、视图/工具选择、航点、导航状态
import { reactive } from 'vue'
import { api, setUnauthorizedHandler } from './api.js'
import { subscribe } from './ros.js'

const LAYERS_KEY = 'robot-ui-layers'
function loadLayers() {
  const def = { map: true, globalCost: false, localCost: true, scan: true, plan: true, grid: true, shapes: true, points: true }
  try { return { ...def, ...JSON.parse(localStorage.getItem(LAYERS_KEY) || '{}') } } catch { return def }
}

export const store = reactive({
  auth: { checked: false, enabled: true, ok: false },
  status: { mode: 'idle', map: null, processes: {} },
  backendOk: false,
  maps: [],
  busy: '',                 // 正在执行的操作，用于禁用按钮
  toast: null,              // {text, kind}
  tool: 'pan',              // pan | initpose | goal | waypoint | draw | select | erase | point
  drawType: 'line',         // draw 工具下的形状，见 annot.js SHAPE_TYPES
  refine: true,             // 初始位姿是否走 relocalize_node 精配
  fixedFrame: 'map',
  follow: false,
  layers: loadLayers(),
  waypoints: [],            // [{x, y, yaw}]
  goal: null,               // 最近发出的单点目标
  robot: null,              // 机器人在 fixedFrame 下 {x, y, yaw}，由 MapView 每帧更新
  navStatus: { nav: null, wp: null },  // action 状态码
  cameraOpen: false,        // 相机浮窗
  velOpen: false,           // 速度曲线浮窗
})

export function saveLayers() {
  try { localStorage.setItem(LAYERS_KEY, JSON.stringify(store.layers)) } catch { /* 无痕模式 */ }
}

let toastTimer = null
export function toast(text, kind = 'info') {
  store.toast = { text, kind }
  clearTimeout(toastTimer)
  toastTimer = setTimeout(() => { store.toast = null }, kind === 'error' ? 8000 : 3000)
}

/** 包一层：busy 标记 + 错误 toast */
export async function run(label, fn) {
  store.busy = label
  try {
    const r = await fn()
    await refreshStatus()
    return r
  } catch (e) {
    toast(`${label}失败: ${e.message}`, 'error')
  } finally {
    store.busy = ''
  }
}

export async function refreshStatus() {
  if (!store.auth.ok) return
  try {
    store.status = await api.status()
    store.backendOk = true
  } catch {
    store.backendOk = false
  }
}

export async function refreshMaps() {
  if (!store.auth.ok) return
  try { store.maps = await api.maps() } catch { /* 状态栏会显示后端断开 */ }
}

setUnauthorizedHandler(() => { store.auth.ok = false })

export async function checkAuth() {
  try {
    const r = await api.me()
    Object.assign(store.auth, { checked: true, enabled: r.enabled, ok: r.ok })
  } catch {
    Object.assign(store.auth, { checked: true, ok: false })
  }
  if (store.auth.ok) { refreshStatus(); refreshMaps() }
  return store.auth.ok
}

setInterval(refreshStatus, 1500)

/** 地图 id（yaml 路径）→ 显示名 */
export function mapName(id) {
  if (!id) return ''
  const m = store.maps.find((x) => x.id === id)
  return m ? m.name : id.split('/').pop().replace(/\.ya?ml$/, '')
}

// GoalStatus: 1 accepted 2 executing 3 canceling 4 succeeded 5 canceled 6 aborted
function lastStatus(msg) {
  const l = msg.status_list
  return l.length ? l[l.length - 1].status : null
}
subscribe('/navigate_to_pose/_action/status', 'action_msgs/GoalStatusArray',
  (m) => { store.navStatus.nav = lastStatus(m) })
subscribe('/follow_waypoints/_action/status', 'action_msgs/GoalStatusArray',
  (m) => { store.navStatus.wp = lastStatus(m) })

export const GOAL_STATUS = {
  1: '已接受', 2: '执行中', 3: '取消中', 4: '已到达', 5: '已取消', 6: '失败',
}
