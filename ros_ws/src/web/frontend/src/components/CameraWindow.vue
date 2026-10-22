<template>
  <div class="cam">
    <div class="head">
      <span class="led" :class="camRunning ? (fps > 0 ? 'ok' : 'warn') : 'bad'" />
      <b>相机</b>
      <span class="muted small mono">{{ fps ? fps.toFixed(1) + ' fps' : '无图像' }}</span>
      <div class="spacer" />
      <button class="sm" :class="{ on: mode === 'preview' }" @click="setMode('preview')" @pointerdown.stop>预览</button>
      <button class="sm" :class="{ on: mode === 'calib' }" @click="setMode('calib')" @pointerdown.stop>标定</button>
      <button class="sm" @click="store.cameraOpen = false" @pointerdown.stop title="关闭">✕</button>
    </div>

    <div class="img-wrap">
      <img v-if="src" :src="src" alt="相机画面" />
      <div v-else class="empty muted">
        <template v-if="!camRunning">相机进程未运行
          <button class="sm" :disabled="!!store.busy" @click="startProc('camera')">启动相机</button></template>
        <template v-else-if="mode === 'calib' && !calibRunning">标定进程未运行
          <button class="sm" :disabled="!!store.busy" @click="startProc('calib')">启动标定</button></template>
        <template v-else>等待图像…</template>
      </div>
    </div>

    <div v-if="mode === 'preview'" class="calib">
      <div class="line wrap">
        <button :disabled="!camRunning || shooting" @click="snap()" title="把当前帧全分辨率原图存到机器人上">拍照</button>
        <button :class="{ on: burst }" :disabled="!camRunning" @click="toggleBurst" title="按间隔连续拍照，采量化校准图用">
          连拍 {{ burst ? '停' : '' }}</button>
        <input v-model.number="burstSec" class="num" type="number" min="0.2" step="0.5" title="连拍间隔（秒）" /> s
        <label class="chk" title="每拍一张同时下载到这台电脑"><input v-model="autoDl" type="checkbox" />同时下载</label>
      </div>
      <div class="line wrap">
        <span class="small">机器人上 <b>{{ cap.count }}</b> 张 · {{ (cap.bytes / 1048576).toFixed(1) }} MB</span>
        <span v-if="lastShot" class="small muted mono">最新 {{ lastShot }}</span>
        <div class="spacer" />
        <a v-if="lastShot" class="btn sm" :href="fileUrl(lastShot)" :download="lastShot">下载最新</a>
        <a class="btn sm" :class="{ dis: !cap.count }" :href="cap.count ? '/api/captures/zip' : undefined">全部下载 zip</a>
        <button class="sm danger" :disabled="!cap.count || burst" @click="clearCaps">清空</button>
      </div>
      <div class="small muted">存放目录 {{ cap.dir }}</div>
    </div>

    <div v-if="mode === 'calib'" class="calib">
      <div class="line wrap">
        <button :disabled="!st || !st.detected || !!st.busy" @click="call('capture')" title="采集当前帧（棋盘要被检测到）">采集</button>
        <button :class="{ on: st?.auto }" :disabled="!st" @click="toggleAuto" title="检测到新姿态且静止时自动采集">自动 {{ st?.auto ? '开' : '关' }}</button>
        <button :disabled="!st || st.captured < 10 || !!st.busy" @click="call('calibrate')">标定</button>
        <button :disabled="!st?.result || st.result.saved" @click="save">保存</button>
        <button class="danger" :disabled="!st || !!st.busy" @click="reset">清空</button>
        <div class="spacer" />
        <button v-if="calibRunning" class="sm" @click="stopProc('calib')" title="停止标定进程">停止</button>
        <button v-else class="sm" :disabled="!!store.busy" @click="startProc('calib')">启动标定</button>
      </div>
      <template v-if="st">
        <div class="kv"><span>已采集</span>
          <b :class="st.captured >= st.min_frames ? 'ok-t' : ''">{{ st.captured }} / {{ st.min_frames }}</b></div>
        <div class="kv"><span>棋盘</span>
          <b :class="st.detected ? (st.new_pose && st.still ? 'ok-t' : 'warn-t') : 'bad-t'">
            {{ st.detected ? (st.new_pose ? '新姿态' : '姿态重复') + (st.still ? '' : ' · 在动') : '未检测到' }}</b></div>
        <div class="msg small">{{ st.message }}</div>
        <div v-if="r" class="result">
          <div class="kv"><span>RMS</span><b :class="r.rms < 0.5 ? 'ok-t' : r.rms < 1 ? 'warn-t' : 'bad-t'">{{ r.rms.toFixed(3) }} px</b></div>
          <div class="kv"><span>fx fy</span><b class="mono">{{ r.fx.toFixed(1) }}  {{ r.fy.toFixed(1) }}</b></div>
          <div class="kv"><span>cx cy</span><b class="mono">{{ r.cx.toFixed(1) }}  {{ r.cy.toFixed(1) }}</b></div>
          <div class="kv"><span>D</span><b class="mono small">{{ r.D.map((v) => v.toFixed(4)).join(' ') }}</b></div>
          <div class="kv"><span>视场 H×V</span><b>{{ r.hfov.toFixed(1) }}° × {{ r.vfov.toFixed(1) }}°</b></div>
          <div class="kv"><span>帧数</span><b>{{ r.frames }}<template v-if="r.dropped">（剔除 {{ r.dropped }}）</template></b></div>
          <div class="small muted">{{ r.summary }}</div>
          <div v-for="w in r.warnings" :key="w" class="small warn-t">{{ w }}</div>
          <div v-if="r.saved" class="small ok-t">已保存到 {{ st.output }}</div>
        </div>
        <div class="small muted tip">
          棋盘 {{ st.board }} 内角点 / {{ st.square_mm }}mm。覆盖画面四角和边缘，远近都要有，左右上下各倾斜 30~45°，每个姿态停稳再采
        </div>
      </template>
    </div>
  </div>
</template>

<script setup>
import { ref, computed, watch, onUnmounted } from 'vue'
import { api } from '../lib/api.js'
import { subscribe, callService, rosState } from '../lib/ros.js'
import { store, run, toast } from '../lib/store.js'

const mode = ref('preview')
const src = ref('')
const st = ref(null)
const r = computed(() => st.value?.result)
const procs = computed(() => store.status.processes)
const camRunning = computed(() => !!procs.value.camera?.running)
const calibRunning = computed(() => !!procs.value.calib?.running)
const topic = computed(() => mode.value === 'calib' ? '/calib/preview/compressed' : '/camera/preview/compressed')
const fps = computed(() => rosState.rates[topic.value] || 0)

let unsubImg = null
let unsubSt = null
watch(topic, (t) => {
  unsubImg?.()
  src.value = ''
  unsubImg = subscribe(t, 'sensor_msgs/CompressedImage',
    (m) => { src.value = 'data:image/jpeg;base64,' + m.data }, { queue_length: 1 })
}, { immediate: true })
watch(mode, (m) => {
  unsubSt?.()
  unsubSt = null
  st.value = null
  if (m === 'calib') {
    unsubSt = subscribe('/calib/status', 'std_msgs/String', (msg) => {
      try { st.value = JSON.parse(msg.data) } catch { /* 忽略 */ }
    })
  }
}, { immediate: true })
// 进程停了就清掉旧画面，避免看着像还在出图
watch([camRunning, calibRunning], () => {
  if (!camRunning.value || (mode.value === 'calib' && !calibRunning.value)) { src.value = ''; if (mode.value === 'calib') st.value = null }
})
onUnmounted(() => { unsubImg?.(); unsubSt?.(); stopBurst() })

// ---------- 拍照 ----------
const cap = ref({ count: 0, bytes: 0, dir: '' })
const lastShot = ref('')
const shooting = ref(false)
const burst = ref(false)
const burstSec = ref(1)
const autoDl = ref(false)
let burstTimer = null
const fileUrl = (n) => `/api/captures/file/${encodeURIComponent(n)}`

async function refreshCaps() {
  try { cap.value = await api.captures() } catch { /* 后端断开时状态栏会显示 */ }
}
function download(name) {
  const a = document.createElement('a')
  a.href = fileUrl(name)
  a.download = name
  a.click()
}
async function snap(quiet = false) {
  shooting.value = true
  try {
    const res = await callService('/camera/snapshot', 'std_srvs/Trigger')
    if (!res.success) { toast(res.message, 'error'); stopBurst(); return }
    lastShot.value = res.message
    if (autoDl.value) download(res.message)
    if (!quiet) toast(`已保存 ${res.message}`)
    refreshCaps()
  } catch (e) {
    toast(`拍照失败: ${e.message}`, 'error')
    stopBurst()
  } finally {
    shooting.value = false
  }
}
function stopBurst() {
  clearInterval(burstTimer)
  burstTimer = null
  burst.value = false
}
function toggleBurst() {
  if (burst.value) { stopBurst(); return }
  burst.value = true
  snap(true)
  burstTimer = setInterval(() => { if (!shooting.value) snap(true) }, Math.max(0.2, burstSec.value || 1) * 1000)
}
async function clearCaps() {
  if (!confirm(`删除机器人上的 ${cap.value.count} 张照片？`)) return
  try {
    const r = await api.deleteCaptures(null)
    toast(`已删除 ${r.deleted} 张`)
    lastShot.value = ''
  } catch (e) { toast(`删除失败: ${e.message}`, 'error') }
  refreshCaps()
}
refreshCaps()

function setMode(m) {
  mode.value = m
  if (m === 'calib' && !calibRunning.value && camRunning.value) startProc('calib')
}

const startProc = (n) => run(`启动 ${n}`, () => api.procStart(n))
const stopProc = (n) => run(`停止 ${n}`, () => api.procStop(n))

async function call(name, type = 'std_srvs/Trigger', req = {}) {
  try {
    const res = await callService(`/calib/${name}`, type, req)
    if (!res.success) toast(res.message, 'error')
    return res
  } catch (e) {
    toast(`/calib/${name} 调用失败: ${e.message}`, 'error')
  }
}
const toggleAuto = () => call('auto', 'std_srvs/SetBool', { data: !st.value.auto })
async function save() {
  if (r.value && !r.value.ok && !confirm('标定结果有警告，仍然保存？')) return
  const res = await call('save')
  if (res?.success) toast(res.message)
}
async function reset() {
  if (st.value.captured && !confirm(`清空已采集的 ${st.value.captured} 帧？`)) return
  call('reset')
}

</script>

<style scoped>
.cam { background: var(--panel); border-bottom: 1px solid var(--line); display: flex; flex-direction: column; flex-shrink: 0; }
.head { display: flex; align-items: center; gap: 6px; padding: 5px 8px; border-bottom: 1px solid var(--line); user-select: none; }
.spacer { flex: 1; }
.img-wrap { background: #000; aspect-ratio: 16 / 9; display: flex; align-items: center; justify-content: center; }
.img-wrap img { width: 100%; height: 100%; object-fit: contain; display: block; }
.empty { display: flex; gap: 8px; align-items: center; }
.calib { padding: 6px 10px 10px; }
.line { display: flex; gap: 6px; align-items: center; margin: 4px 0; }
.line.wrap { flex-wrap: wrap; }
.kv { display: flex; justify-content: space-between; gap: 8px; padding: 2px 0; font-size: 12px; }
.kv b { font-weight: 500; font-variant-numeric: tabular-nums; }
.msg { margin: 4px 0; color: var(--text); }
.result { margin-top: 6px; padding-top: 6px; border-top: 1px solid var(--line); }
.tip { margin-top: 6px; }
.btn { display: inline-block; text-decoration: none; color: var(--text); background: #2b2f3b; border: 1px solid var(--line); border-radius: 4px; padding: 1px 8px; font-size: 12px; }
.btn:hover { background: #353a48; }
.btn.dis { opacity: 0.45; pointer-events: none; }
.num { width: 56px; }
</style>
