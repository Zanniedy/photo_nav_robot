<template>
  <div class="panel">
    <section v-if="mode === 'mapping'">
      <h4>保存地图</h4>
      <div class="line">
        <input v-model="saveName" placeholder="地图名" />
        <button @click="save" :disabled="!saveName || !!store.busy">保存</button>
      </div>
      <div class="small muted">保存到 map_dirs 第一个目录/{{ saveName || '…' }}.yaml/.pgm（同时存 posegraph）</div>
    </section>

    <section>
      <h4>地图 <button class="sm" @click="refreshMaps">重新搜索</button></h4>
      <div v-if="!store.maps.length" class="muted small">map_dirs 下没有找到地图</div>
      <div v-for="m in store.maps" :key="m.id" class="row" :title="m.id">
        <button class="sm star" :class="{ on: m.default }" :aria-label="m.default ? '取消默认地图' : '设为默认地图'"
                :title="m.default ? '开机自动以此地图导航（点击取消）' : '设为默认：开机自动以此地图导航'"
                @click="toggleDefault(m)">{{ m.default ? '★' : '☆' }}</button>
        <span class="name" :class="{ cur: m.id === store.status.map }">{{ m.name }}</span>
        <span v-if="m.pose" class="small ok-t" title="已保存初始位姿">📍</span>
        <span class="muted small">{{ fmtTime(m.mtime) }}</span>
        <button class="sm" :disabled="!!store.busy || m.id === store.status.map" @click="useMap(m)">
          {{ mode === 'navigation' ? '切换' : '导航' }}
        </button>
      </div>
      <div class="small muted">★ 默认地图：后端启动后自动进导航；📍 有保存的初始位姿，进导航时自动发送</div>
    </section>

    <section v-if="mode === 'navigation'">
      <h4>初始位姿 <span class="muted small">{{ mapName(store.status.map) }}</span></h4>
      <div class="line">
        <input class="num" type="number" step="0.1" v-model.number="init.x" placeholder="x" aria-label="初始 x" />
        <input class="num" type="number" step="0.1" v-model.number="init.y" placeholder="y" aria-label="初始 y" />
        <input class="num" type="number" step="15" v-model.number="init.deg" placeholder="°" aria-label="初始朝向（度）" />
        <label class="chk" title="经 relocalize_node 扫描精配"><input type="checkbox" v-model="store.refine" /> 精配</label>
        <button class="sm" @click="sendInit">发送</button>
      </div>
      <div class="line wrap">
        <button class="sm" @click="fillFromRobot" :disabled="!robotInMap">填入当前位姿</button>
        <button class="sm" @click="saveInit" title="保存为该地图的初始位姿，下次进导航自动发送">保存为该地图初始位姿</button>
        <button class="sm" v-if="curMap?.pose" @click="applySaved">发送已保存</button>
        <button class="sm" v-if="curMap?.pose" @click="clearSaved">删除已保存</button>
        <button class="sm" @click="relocalize" title="以当前定位为初值再扫描精配一次">原地重定位</button>
      </div>
      <div v-if="curMap?.pose" class="small muted">已保存 ({{ curMap.pose.x.toFixed(2) }}, {{ curMap.pose.y.toFixed(2) }}, {{ deg(curMap.pose.yaw) }}°)</div>
      <div class="small muted">也可以用地图上的「📍 初始位姿」工具拖拽设置</div>
    </section>

    <section>
      <h4>导航</h4>
      <div class="line">
        <button @click="store.tool = 'initpose'" :class="{ on: store.tool === 'initpose' }">📍 设初始位姿</button>
        <button @click="store.tool = 'goal'" :class="{ on: store.tool === 'goal' }">🎯 点选目标</button>
        <button class="danger" @click="cancel" :disabled="!!store.busy">取消导航</button>
      </div>
      <div class="line">
        <span class="small">目标坐标</span>
        <input class="num" type="number" step="0.1" v-model.number="manual.x" placeholder="x" />
        <input class="num" type="number" step="0.1" v-model.number="manual.y" placeholder="y" />
        <input class="num" type="number" step="15" v-model.number="manual.deg" placeholder="°" />
        <button class="sm" @click="sendManual">发送</button>
        <button class="sm" @click="addManual" title="加入航点">+航点</button>
      </div>
      <div v-if="store.goal" class="small muted">当前目标 ({{ store.goal.x.toFixed(2) }}, {{ store.goal.y.toFixed(2) }}, {{ deg(store.goal.yaw) }}°)</div>
    </section>

    <section>
      <h4>航点 ({{ store.waypoints.length }})
        <button class="sm" :class="{ on: store.tool === 'waypoint' }" @click="store.tool = 'waypoint'">➕ 地图上添加</button>
      </h4>
      <div v-for="(w, i) in store.waypoints" :key="i" class="row">
        <span class="name mono">{{ i + 1 }}. ({{ w.x.toFixed(2) }}, {{ w.y.toFixed(2) }}, {{ deg(w.yaw) }}°)</span>
        <button class="sm" :disabled="i === 0" @click="moveUp(i)">↑</button>
        <button class="sm" @click="store.waypoints.splice(i, 1)">✕</button>
      </div>
      <div class="line">
        <button @click="runWaypoints" :disabled="!store.waypoints.length || !!store.busy">执行航点</button>
        <button @click="store.waypoints = []" :disabled="!store.waypoints.length">清空</button>
      </div>
    </section>

    <Teleop />
  </div>
</template>

<script setup>
import { ref, reactive, computed, watch } from 'vue'
import { api } from '../lib/api.js'
import { publish, quatOf } from '../lib/ros.js'
import { store, run, refreshMaps, toast, mapName } from '../lib/store.js'
import Teleop from './Teleop.vue'

const mode = computed(() => store.status.mode)
const saveName = ref('')
const manual = reactive({ x: null, y: null, deg: 0 })
const init = reactive({ x: null, y: null, deg: 0 })
const curMap = computed(() => store.maps.find((m) => m.id === store.status.map))
const robotInMap = computed(() => store.fixedFrame === 'map' && !!store.robot)
const deg = (r) => (r * 180 / Math.PI).toFixed(0)

function defaultName() {
  const d = new Date(), p = (n) => String(n).padStart(2, '0')
  return `map_${d.getFullYear()}${p(d.getMonth() + 1)}${p(d.getDate())}_${p(d.getHours())}${p(d.getMinutes())}${p(d.getSeconds())}`
}
watch(mode, (m) => { if (m === 'mapping' && !saveName.value) saveName.value = defaultName() }, { immediate: true })

function fmtTime(t) {
  const d = new Date(t * 1000), p = (n) => String(n).padStart(2, '0')
  return `${p(d.getMonth() + 1)}-${p(d.getDate())} ${p(d.getHours())}:${p(d.getMinutes())}`
}

async function save() {
  const r = await run('保存地图', () => api.mappingSave(saveName.value))
  if (r) {
    toast(`已保存 ${r.yaml}`)
    saveName.value = defaultName()
    refreshMaps()
  }
}

async function useMap(m) {
  if (mode.value === 'mapping' && !confirm('将停止建图（未保存的地图会丢失），继续？')) return
  const r = mode.value === 'navigation'
    ? await run('切换地图', () => api.switchMap(m.id))
    : await run('启动导航', () => api.navStart(m.id))
  if (r) {
    store.goal = null; store.waypoints = []
    toast(m.pose ? `地图 ${m.name} 已加载，将自动发送已保存的初始位姿` : `地图 ${m.name} 已加载，请设置初始位姿`)
  }
}

async function toggleDefault(m) {
  if (await run('设置默认地图', () => api.setDefaultMap(m.default ? null : m.id))) refreshMaps()
}

function initPose() {
  if (typeof init.x !== 'number' || typeof init.y !== 'number') { toast('先填 x / y', 'error'); return null }
  return { x: init.x, y: init.y, yaw: (init.deg || 0) * Math.PI / 180 }
}
function fillFromRobot() {
  const r = store.robot
  init.x = +r.x.toFixed(3); init.y = +r.y.toFixed(3); init.deg = +(r.yaw * 180 / Math.PI).toFixed(1)
}
async function sendInit() {
  const p = initPose()
  if (p && await run('发送初始位姿', () => api.initialPose(p, store.refine))) toast('初始位姿已发送')
}
async function saveInit() {
  if (init.x == null && robotInMap.value) fillFromRobot()
  const p = initPose()
  if (p && await run('保存初始位姿', () => api.saveMapPose(store.status.map, p))) { toast('已保存，下次进导航自动发送'); refreshMaps() }
}
async function applySaved() {
  const p = curMap.value.pose
  if (await run('发送初始位姿', () => api.initialPose(p, false))) toast('已发送保存的初始位姿')
}
async function clearSaved() {
  if (await run('删除初始位姿', () => api.clearMapPose(store.status.map))) refreshMaps()
}
async function relocalize() {
  const r = await run('重定位', () => api.quick('relocalize'))
  if (r) toast(r.output.includes('success=True') ? '重定位成功' : `重定位结果:\n${r.output.slice(-300)}`, r.output.includes('success=True') ? 'info' : 'error')
}

async function cancel() {
  const r = await run('取消导航', api.navCancel)
  if (r) { store.goal = null; toast('已取消导航') }
}

function manualPose() {
  if (typeof manual.x !== 'number' || typeof manual.y !== 'number') { toast('先填 x / y', 'error'); return null }
  return { x: manual.x, y: manual.y, yaw: (manual.deg || 0) * Math.PI / 180 }
}
function sendManual() {
  const p = manualPose()
  if (!p) return
  if (publish('/goal_pose', 'geometry_msgs/PoseStamped', {
    header: { frame_id: 'map' },
    pose: { position: { x: p.x, y: p.y, z: 0 }, orientation: quatOf(p.yaw) },
  })) { store.goal = p; toast('已发 /goal_pose') } else toast('rosbridge 未连接', 'error')
}
function addManual() { const p = manualPose(); if (p) store.waypoints.push(p) }

function moveUp(i) {
  const w = store.waypoints
  ;[w[i - 1], w[i]] = [w[i], w[i - 1]]
}

async function runWaypoints() {
  const r = await run('执行航点', () => api.waypoints(store.waypoints))
  if (r) toast(`已发送 ${store.waypoints.length} 个航点`)
}
</script>

<style scoped>
.star { padding: 0 6px; color: var(--muted); }
.star.on { color: var(--warn); background: transparent; border-color: var(--line); }
</style>
