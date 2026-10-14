<template>
  <div class="panel">
    <section v-if="mode === 'mapping'">
      <h4>保存地图</h4>
      <div class="line">
        <input v-model="saveName" placeholder="地图名" />
        <button @click="save" :disabled="!saveName || !!store.busy">保存</button>
      </div>
      <div class="small muted">保存到 maps_dir/{{ saveName || '…' }}.yaml/.pgm（同时存 posegraph）</div>
    </section>

    <section>
      <h4>地图 <button class="sm" @click="refreshMaps">刷新</button></h4>
      <div v-if="!store.maps.length" class="muted small">maps_dir 下没有地图</div>
      <div v-for="m in store.maps" :key="m.name" class="row">
        <span class="name" :class="{ cur: m.name === store.status.map }">{{ m.name }}</span>
        <span class="muted small">{{ fmtTime(m.mtime) }}</span>
        <button class="sm" :disabled="!!store.busy || m.name === store.status.map" @click="useMap(m.name)">
          {{ mode === 'navigation' ? '切换' : '导航' }}
        </button>
      </div>
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
import { store, run, refreshMaps, toast } from '../lib/store.js'
import Teleop from './Teleop.vue'

const mode = computed(() => store.status.mode)
const saveName = ref('')
const manual = reactive({ x: null, y: null, deg: 0 })
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

async function useMap(name) {
  if (mode.value === 'mapping' && !confirm('将停止建图（未保存的地图会丢失），继续？')) return
  const r = mode.value === 'navigation'
    ? await run('切换地图', () => api.switchMap(name))
    : await run('启动导航', () => api.navStart(name))
  if (r) { store.goal = null; store.waypoints = []; toast(`地图 ${name} 已加载，请设置初始位姿`) }
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
