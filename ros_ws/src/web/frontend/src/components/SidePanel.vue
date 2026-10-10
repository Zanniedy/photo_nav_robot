<template>
  <aside class="side-panel">
    <!-- 系统自检 -->
    <section class="section">
      <div class="label">系统自检
        <button class="refresh-btn" @click="runCheck" :disabled="diagRunning">
          {{ diagRunning ? '检测中...' : '检测' }}
        </button>
      </div>
      <div v-if="diagItems.length === 0" class="msg">点击检测查看系统状态</div>
      <div v-for="item in diagItems" :key="item.key" class="diag-row">
        <span class="diag-icon">{{ item.ok ? '✓' : '✗' }}</span>
        <span class="diag-label" :class="{ 'diag-ok': item.ok, 'diag-fail': !item.ok }">{{ item.label }}</span>
        <span v-if="!item.ok" class="diag-reason">{{ item.reason }}</span>
      </div>
    </section>
    <section class="section">
      <div class="label">模式</div>
      <button :class="{ active: mode === 'mapping' }" @click="startMapping" :disabled="startingMapping">
        {{ startingMapping ? startMsg : '建图' }}
      </button>
      <button :class="{ active: mode === 'navigation' }" @click="showNavPanel = true">导航</button>
      <button v-if="mode !== 'idle'" @click="stopAll" class="stop-btn">停止</button>
      <div v-if="startError" class="msg err-msg">{{ startError }}</div>
    </section>

    <!-- 串口设备 -->
    <section class="section">
      <div class="label">串口设备 <button class="refresh-btn" @click="refreshPorts">刷新</button></div>
      <div v-if="serialPorts.length === 0" class="msg">未检测到设备</div>
      <div v-for="p in serialPorts" :key="p.port" class="port-row">
        <span class="port-name">{{ p.port }}</span>
        <span class="port-label">{{ p.label || '未知设备' }}</span>
      </div>
    </section>

    <!-- 激光雷达 -->
    <section class="section">
      <div class="label">激光雷达 (C1)</div>
      <button :class="{ active: lidarRunning }" @click="toggleLidar">
        {{ lidarRunning ? '● 运行中' : '启动' }}
      </button>
      <div v-if="lidarMsg" class="msg">{{ lidarMsg }}</div>
    </section>

    <!-- 建图：保存地图 -->
    <section v-if="mode === 'mapping'" class="section">
      <div class="label">保存地图</div>
      <input v-model="saveName" placeholder="地图名称" class="input" />
      <button @click="saveMap" :disabled="!saveName">保存</button>
      <div v-if="saveMsg" class="msg">{{ saveMsg }}</div>
    </section>

    <!-- 导航：地图列表 -->
    <section v-if="showNavPanel || mode === 'navigation'" class="section">
      <div class="label">选择地图</div>
      <div v-if="maps.length === 0" class="msg">无地图</div>
      <div v-for="m in maps" :key="m.name" class="map-row">
        <span>{{ m.name }}</span>
        <button @click="startNav(m.name)">启动</button>
      </div>
      <button @click="loadMaps" class="refresh-btn">刷新</button>
    </section>

    <!-- IMU -->
    <section class="section">
      <div class="label">IMU</div>
      <div class="imu-grid">
        <span class="imu-label">加速度</span>
        <span class="imu-val">x {{ imu.ax }}</span>
        <span class="imu-val">y {{ imu.ay }}</span>
        <span class="imu-val">z {{ imu.az }}</span>
        <span class="imu-label">角速度</span>
        <span class="imu-val">x {{ imu.gx }}</span>
        <span class="imu-val">y {{ imu.gy }}</span>
        <span class="imu-val">z {{ imu.gz }}</span>
      </div>
    </section>

    <!-- 进程日志 -->
    <section class="section logs-section">
      <div class="label">进程日志</div>
      <div ref="logsEl" class="logs">
        <div v-for="(line, i) in logs" :key="i" class="log-line">{{ line }}</div>
      </div>
    </section>
  </aside>
</template>

<script setup>
import { ref, reactive, nextTick, onMounted, onUnmounted } from 'vue'
import * as api from '../api.js'
import { useRosbridge } from '../composables/useRosbridge.js'

const emit = defineEmits(['update:mode'])
const props = defineProps({ mode: String })

const { subscribe } = useRosbridge()

const maps = ref([])
const saveName = ref('')
const saveMsg = ref('')
const showNavPanel = ref(false)
const lidarRunning = ref(false)
const lidarMsg = ref('')
const logs = ref([])
const logsEl = ref(null)
const imu = reactive({ ax: '--', ay: '--', az: '--', gx: '--', gy: '--', gz: '--' })
const serialPorts = ref([])
const diagItems = ref([])
const diagRunning = ref(false)
const startingMapping = ref(false)
const startMsg = ref('启动中...')
const startError = ref('')
let evtSource = null
let unsubs = []

function fmt(v) { return typeof v === 'number' ? v.toFixed(2) : '--' }

async function runCheck() {
  diagRunning.value = true
  try {
    const res = await api.runDiag()
    diagItems.value = Object.entries(res.items).map(([key, v]) => ({ key, ...v }))
  } catch (e) {
    diagItems.value = [{ key: 'err', ok: false, label: '自检失败', reason: e.message }]
  } finally {
    diagRunning.value = false
  }
}

onMounted(() => {
  loadMaps()
  initLogs()
  refreshPorts()
  unsubs.push(subscribe('/imu/data', 'sensor_msgs/Imu', (msg) => {
    const a = msg.linear_acceleration
    const g = msg.angular_velocity
    imu.ax = fmt(a?.x); imu.ay = fmt(a?.y); imu.az = fmt(a?.z)
    imu.gx = fmt(g?.x); imu.gy = fmt(g?.y); imu.gz = fmt(g?.z)
  }))
})

onUnmounted(() => { evtSource?.close(); unsubs.forEach(u => u()) })

async function refreshPorts() {
  try {
    const res = await api.getSerialPorts()
    serialPorts.value = res.ports
  } catch (e) {
    serialPorts.value = []
  }
}

async function loadMaps() {
  maps.value = await api.getMaps()
}

async function startMapping() {
  startingMapping.value = true
  startError.value = ''
  startMsg.value = '启动bridge...'
  try {
    await api.slamStart()
    emit('update:mode', 'mapping')
    showNavPanel.value = false
    startMsg.value = '建图'
  } catch (e) {
    startError.value = e.message
    emit('update:mode', 'idle')
  } finally {
    startingMapping.value = false
  }
}

async function startNav(name) {
  await api.navStart(name)
  emit('update:mode', 'navigation')
  showNavPanel.value = false
}

async function stopAll() {
  await api.slamStop()
  await api.navStop()
  emit('update:mode', 'idle')
}

async function saveMap() {
  saveMsg.value = '保存中...'
  try {
    await api.slamSave(saveName.value)
    saveMsg.value = '保存成功'
    saveName.value = ''
    await loadMaps()
  } catch (e) {
    saveMsg.value = '保存失败: ' + e.message
  }
}

async function toggleLidar() {
  try {
    if (lidarRunning.value) {
      await api.lidarStop()
      lidarRunning.value = false
      lidarMsg.value = '已停止'
    } else {
      await api.lidarStart()
      lidarRunning.value = true
      lidarMsg.value = ''
    }
  } catch (e) {
    lidarMsg.value = e.message
  }
}

async function initLogs() {
  evtSource = await api.getLogsEventSource()
  evtSource.onmessage = (e) => {
    const { line } = JSON.parse(e.data)
    logs.value.push(line)
    if (logs.value.length > 50) logs.value.shift()
    nextTick(() => {
      if (logsEl.value) logsEl.value.scrollTop = logsEl.value.scrollHeight
    })
  }
}
</script>

<style scoped>
.side-panel { width: 220px; background: #16162e; border-right: 1px solid #333;
  display: flex; flex-direction: column; overflow-y: auto; }
.section { padding: 10px; border-bottom: 1px solid #2a2a4a; flex-shrink: 0; }
.label { font-size: 0.7rem; color: #666; margin-bottom: 6px; text-transform: uppercase; }
button { background: #2a2a4a; color: #ccc; border: 1px solid #444; padding: 4px 10px;
  border-radius: 4px; cursor: pointer; margin: 2px; font-size: 0.8rem; }
button.active { background: #1a4a2a; color: #7f7; border-color: #4a7; }
button.stop-btn { background: #4a1a1a; color: #f77; border-color: #744; }
button:disabled { opacity: 0.4; cursor: default; }
.input { width: 100%; background: #111; color: #eee; border: 1px solid #444;
  padding: 4px; border-radius: 4px; font-size: 0.8rem; margin-bottom: 4px; }
.map-row { display: flex; justify-content: space-between; align-items: center;
  padding: 2px 0; font-size: 0.8rem; }
.msg { font-size: 0.75rem; color: #aaa; padding: 2px 0; }
.refresh-btn { font-size: 0.7rem; margin-top: 4px; }
.imu-grid { display: grid; grid-template-columns: auto 1fr 1fr 1fr; gap: 2px 6px;
  font-size: 0.7rem; font-family: monospace; }
.imu-label { color: #666; align-self: center; }
.imu-val { color: #8cf; text-align: right; }
.logs-section { flex: 1; display: flex; flex-direction: column; min-height: 120px; }
.logs { flex: 1; overflow-y: auto; font-size: 0.65rem; color: #6a6; font-family: monospace;
  background: #0a0a1a; padding: 4px; max-height: 200px; }
.port-row { display: flex; justify-content: space-between; align-items: center;
  padding: 2px 0; font-size: 0.75rem; font-family: monospace; }
.port-name { color: #8cf; }
.port-label { color: #aaa; font-size: 0.7rem; }
.log-line { word-break: break-all; line-height: 1.4; }
.diag-row { display: flex; align-items: flex-start; gap: 4px; padding: 2px 0; font-size: 0.75rem; flex-wrap: wrap; }
.diag-icon { width: 12px; flex-shrink: 0; }
.diag-label { font-family: monospace; }
.diag-ok { color: #4f4; }
.diag-fail { color: #f77; }
.diag-reason { color: #a88; font-size: 0.65rem; width: 100%; padding-left: 16px; line-height: 1.3; }
.err-msg { color: #f77; font-size: 0.72rem; margin-top: 4px; word-break: break-all; }
</style>
