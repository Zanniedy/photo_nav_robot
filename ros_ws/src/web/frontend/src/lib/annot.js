// 地图标注：绘图形状 + 命名点位。跟随当前导航地图加载，形状改动后自动保存（防抖）
import { reactive, watch } from 'vue'
import { api } from './api.js'
import { store, toast } from './store.js'

export const SHAPE_TYPES = [
  { id: 'line', label: '直线', icon: '╱' },
  { id: 'arrow', label: '箭头', icon: '➝' },
  { id: 'polyline', label: '折线', icon: '⌇' },
  { id: 'polygon', label: '多边形', icon: '⬠' },
  { id: 'rect', label: '矩形', icon: '▭' },
  { id: 'circle', label: '圆', icon: '◯' },
  { id: 'freehand', label: '手绘', icon: '✎' },
  { id: 'text', label: '文字', icon: 'T' },
]
export const TYPE_LABEL = Object.fromEntries(SHAPE_TYPES.map((t) => [t.id, t.label]))

export const annot = reactive({
  map: null,            // 当前标注所属地图 id
  shapes: [],
  points: [],
  loading: false,
  dirty: false,         // 有未保存的形状改动
  saving: false,
  selected: null,       // 选中的形状 id
  selectedPoint: null,  // 选中的点 id
  showShapes: true,
  showPoints: true,
  style: { color: '#ffcc00', width: 2, fill: false },
})

const undoStack = []
const MAX_UNDO = 50
let saveTimer = null

export const newId = () => Math.random().toString(36).slice(2, 10)

async function load(mapId) {
  clearTimeout(saveTimer)
  if (annot.dirty && annot.map) await flush()
  undoStack.length = 0
  Object.assign(annot, { map: mapId, shapes: [], points: [], selected: null, selectedPoint: null, dirty: false })
  if (!mapId) return
  annot.loading = true
  try {
    const d = await api.annotations(mapId)
    if (annot.map !== mapId) return   // 加载期间又切了图
    annot.shapes = d.shapes
    annot.points = d.points
  } catch (e) {
    toast(`读取标注失败: ${e.message}`, 'error')
  } finally {
    annot.loading = false
  }
}

watch(() => store.status.map, (m) => { if (m !== annot.map) load(m) })

export const reloadAnnotations = () => load(annot.map)

// ── 形状 ───────────────────────────────────────────────────────────────

function snapshot() {
  undoStack.push(JSON.stringify(annot.shapes))
  if (undoStack.length > MAX_UNDO) undoStack.shift()
}

function changed() {
  annot.dirty = true
  clearTimeout(saveTimer)
  saveTimer = setTimeout(flush, 800)
}

export async function flush() {
  clearTimeout(saveTimer)
  if (!annot.map || !annot.dirty) return
  const mapId = annot.map
  annot.saving = true
  try {
    await api.saveShapes(mapId, annot.shapes)
    if (annot.map === mapId) annot.dirty = false
  } catch (e) {
    toast(`保存绘图失败: ${e.message}`, 'error')
  } finally {
    annot.saving = false
  }
}

export function addShape(s) {
  if (!annot.map) { toast('先进入导航模式（标注绑定到当前地图）', 'error'); return }
  snapshot()
  annot.shapes.push({ id: newId(), ...annot.style, text: '', hidden: false, ...s })
  changed()
}

export function updateShape(id, patch, { undoable = true } = {}) {
  const s = annot.shapes.find((x) => x.id === id)
  if (!s) return
  if (undoable) snapshot()
  Object.assign(s, patch)
  changed()
}

/** 拖动移动用：开始时调一次 snapshot，过程中不入栈 */
export const beginEdit = snapshot

export function removeShape(id) {
  if (!annot.shapes.some((s) => s.id === id)) return
  snapshot()
  annot.shapes = annot.shapes.filter((s) => s.id !== id)
  if (annot.selected === id) annot.selected = null
  changed()
}

export function clearShapes() {
  if (!annot.shapes.length) return
  snapshot()
  annot.shapes = []
  annot.selected = null
  changed()
}

export function undo() {
  const prev = undoStack.pop()
  if (prev == null) return false
  annot.shapes = JSON.parse(prev)
  if (!annot.shapes.some((s) => s.id === annot.selected)) annot.selected = null
  changed()
  return true
}

export const canUndo = () => undoStack.length > 0

// ── 点位（直接走后端，增删改查即时生效） ─────────────────────────────────

export async function addPoint(p) {
  if (!annot.map) { toast('先进入导航模式（点位绑定到当前地图）', 'error'); return null }
  try {
    const r = await api.createPoint({ map: annot.map, ...p })
    annot.points.push(r)
    return r
  } catch (e) {
    toast(`添加点失败: ${e.message}`, 'error')
    return null
  }
}

export async function editPoint(id, patch) {
  try {
    const r = await api.updatePoint(annot.map, id, patch)
    const i = annot.points.findIndex((p) => p.id === id)
    if (i >= 0) annot.points[i] = r
    return r
  } catch (e) {
    toast(`修改点失败: ${e.message}`, 'error')
    return null
  }
}

export async function deletePoint(id) {
  try {
    await api.deletePoint(annot.map, id)
    annot.points = annot.points.filter((p) => p.id !== id)
    if (annot.selectedPoint === id) annot.selectedPoint = null
  } catch (e) {
    toast(`删除点失败: ${e.message}`, 'error')
  }
}

export async function movePoint(i, d) {
  const j = i + d
  const pts = annot.points
  if (j < 0 || j >= pts.length) return
  ;[pts[i], pts[j]] = [pts[j], pts[i]]
  try { await api.reorderPoints(annot.map, pts.map((p) => p.id)) } catch (e) { toast(`排序失败: ${e.message}`, 'error') }
}

export async function gotoPoints(names) {
  try {
    await api.gotoPoints(names)
    toast(`出发: ${names.join(' → ')}`)
    return true
  } catch (e) {
    toast(`导航失败: ${e.message}`, 'error')
    return false
  }
}

// 关页面前把没存的形状发出去
window.addEventListener('beforeunload', () => {
  if (annot.dirty && annot.map) {
    navigator.sendBeacon?.('/api/annotations/shapes_beacon',
      new Blob([JSON.stringify({ map: annot.map, shapes: annot.shapes })], { type: 'application/json' }))
  }
})
