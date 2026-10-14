<template>
  <section>
    <h4>遥控
      <label class="chk"><input type="checkbox" v-model="keyboard" /> 键盘 WASD</label>
    </h4>
    <div class="teleop">
      <div class="pad" ref="pad" @pointerdown="padDown" @pointermove="padMove" @pointerup="padUp" @pointercancel="padUp">
        <div class="knob" :style="{ transform: `translate(${knob.x}px, ${knob.y}px)` }" />
      </div>
      <div class="sliders">
        <label>线速度 ≤ {{ maxV.toFixed(2) }} m/s<input type="range" min="0.05" max="0.5" step="0.05" v-model.number="maxV" /></label>
        <label>角速度 ≤ {{ maxW.toFixed(1) }} rad/s<input type="range" min="0.2" max="2" step="0.1" v-model.number="maxW" /></label>
        <div class="mono small">发送 {{ out.vx.toFixed(2) }} m/s · {{ out.wz.toFixed(2) }} rad/s</div>
        <button class="danger big" @click="estop">急停 (空格)</button>
      </div>
    </div>
    <div v-if="navActive" class="small warn-t">导航进行中：遥控会和 nav2 抢 /cmd_vel，先取消导航</div>
  </section>
</template>

<script setup>
import { ref, reactive, computed, onMounted, onUnmounted } from 'vue'
import { publish } from '../lib/ros.js'
import { api } from '../lib/api.js'
import { store, toast } from '../lib/store.js'

const R = 60   // 摇杆半径 px
const pad = ref(null)
const knob = reactive({ x: 0, y: 0 })
const keyboard = ref(false)
const maxV = ref(0.2)
const maxW = ref(0.8)
const out = reactive({ vx: 0, wz: 0 })
const keys = new Set()
let padActive = false
let timer = null
let zerosLeft = 0

const navActive = computed(() => [1, 2].includes(store.navStatus.nav) || [1, 2].includes(store.navStatus.wp))

function sendTwist(vx, wz) {
  publish('/cmd_vel', 'geometry_msgs/Twist', { linear: { x: vx, y: 0, z: 0 }, angular: { x: 0, y: 0, z: wz } })
}

// 10Hz：有输入就发；松开后补发几帧 0（bridge 自己也有 cmd_timeout 兜底）
function tick() {
  let vx = 0, wz = 0, active = false
  if (padActive) {
    vx = -knob.y / R * maxV.value
    wz = -knob.x / R * maxW.value
    active = true
  } else if (keyboard.value && keys.size) {
    if (keys.has('w')) vx += maxV.value
    if (keys.has('s')) vx -= maxV.value
    if (keys.has('a')) wz += maxW.value
    if (keys.has('d')) wz -= maxW.value
    active = true
  }
  out.vx = vx; out.wz = wz
  if (active) { sendTwist(vx, wz); zerosLeft = 3 } else if (zerosLeft > 0) { sendTwist(0, 0); zerosLeft-- }
}

function padPos(e) {
  const r = pad.value.getBoundingClientRect()
  let x = e.clientX - (r.left + r.width / 2), y = e.clientY - (r.top + r.height / 2)
  const d = Math.hypot(x, y)
  if (d > R) { x *= R / d; y *= R / d }
  knob.x = x; knob.y = y
}
function padDown(e) { pad.value.setPointerCapture(e.pointerId); padActive = true; padPos(e) }
function padMove(e) { if (padActive) padPos(e) }
function padUp() { padActive = false; knob.x = 0; knob.y = 0 }

async function estop() {
  padUp()
  keys.clear()
  for (let i = 0; i < 3; i++) sendTwist(0, 0)
  zerosLeft = 5
  if (navActive.value) {
    try { await api.navCancel() } catch (e) { toast(`取消导航失败: ${e.message}`, 'error') }
  }
  toast('急停：已发零速' + (navActive.value ? '并取消导航' : ''))
}

const MAP = { w: 'w', arrowup: 'w', s: 's', arrowdown: 's', a: 'a', arrowleft: 'a', d: 'd', arrowright: 'd' }
function isTyping(e) { return ['INPUT', 'TEXTAREA', 'SELECT'].includes(e.target.tagName) && e.target.type !== 'checkbox' && e.target.type !== 'range' }
function onKey(e) {
  if (isTyping(e)) return
  if (e.code === 'Space' && e.type === 'keydown') { e.preventDefault(); estop(); return }
  const k = MAP[e.key.toLowerCase()]
  if (!k || !keyboard.value) return
  e.preventDefault()
  e.type === 'keydown' ? keys.add(k) : keys.delete(k)
}
function onBlur() { keys.clear() }

onMounted(() => {
  timer = setInterval(tick, 100)
  window.addEventListener('keydown', onKey)
  window.addEventListener('keyup', onKey)
  window.addEventListener('blur', onBlur)
})
onUnmounted(() => {
  clearInterval(timer)
  window.removeEventListener('keydown', onKey)
  window.removeEventListener('keyup', onKey)
  window.removeEventListener('blur', onBlur)
})
</script>

<style scoped>
.teleop { display: flex; gap: 12px; align-items: center; }
.pad { width: 140px; height: 140px; border-radius: 50%; background: radial-gradient(circle, #2a3040 0%, #20242e 70%); border: 1px solid var(--line); position: relative; flex-shrink: 0; touch-action: none; cursor: grab; }
.pad::before, .pad::after { content: ''; position: absolute; background: var(--line); }
.pad::before { left: 50%; top: 10%; bottom: 10%; width: 1px; }
.pad::after { top: 50%; left: 10%; right: 10%; height: 1px; }
.knob { position: absolute; left: 50%; top: 50%; width: 40px; height: 40px; margin: -20px 0 0 -20px; border-radius: 50%; background: var(--accent); opacity: 0.85; pointer-events: none; z-index: 1; }
.sliders { display: flex; flex-direction: column; gap: 6px; flex: 1; min-width: 0; font-size: 12px; }
.sliders label { display: flex; flex-direction: column; gap: 2px; }
.big { padding: 8px; font-weight: 700; }
</style>
