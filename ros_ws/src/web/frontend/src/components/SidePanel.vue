<template>
  <aside class="side-panel">
    <!-- 模式控制 -->
    <section class="section">
      <div class="label">模式</div>
      <button :class="{ active: mode === 'mapping' }" @click="startMapping">建图</button>
      <button :class="{ active: mode === 'navigation' }" @click="showNavPanel = true">导航</button>
      <button v-if="mode !== 'idle'" @click="stopAll" class="stop-btn">停止</button>
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
let evtSource = null
let unsubs = []

function fmt(v) { return typeof v === 'number' ? v.toFixed(2) : '--' }

onMounted(() => {
  loadMaps()
  initLogs()
  unsubs.push(subscribe('/imu/data', 'sensor_msgs/Imu', (msg) => {
    const a = msg.linear_acceleration
    const g = msg.angular_velocity
    imu.ax = fmt(a?.x); imu.ay = fmt(a?.y); imu.az = fmt(a?.z)
    imu.gx = fmt(g?.x); imu.gy = fmt(g?.y); imu.gz = fmt(g?.z)
  }))
})

onUnmounted(() => { evtSource?.close(); unsubs.forEach(u => u()) })

async function loadMaps() {
  maps.value = await api.getMaps()
}

async function startMapping() {
  await api.slamStart()
  emit('update:mode', 'mapping')
  showNavPanel.value = false
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
.log-line { word-break: break-all; line-height: 1.4; }
</style>
