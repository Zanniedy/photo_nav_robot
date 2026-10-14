<template>
  <div class="panel">
    <section>
      <h4>进程</h4>
      <div v-for="(p, name) in procs" :key="name" class="row">
        <span class="led" :class="p.running ? 'ok' : (p.exit_code != null && p.exit_code !== 0 ? 'bad' : '')" />
        <span class="name">{{ name }}</span>
        <span class="muted">{{ procText(p) }}</span>
        <button v-if="name === 'rosbridge'" class="sm" @click="run('重启 rosbridge', () => api.procStart('rosbridge'))">重启</button>
      </div>
    </section>

    <section>
      <h4>机器人</h4>
      <div class="kv"><span>位姿 ({{ store.fixedFrame }})</span>
        <b v-if="store.robot">{{ store.robot.x.toFixed(2) }}, {{ store.robot.y.toFixed(2) }}, {{ deg(store.robot.yaw) }}°</b>
        <b v-else class="muted">—</b></div>
      <div class="kv"><span>底盘 yaw</span><b>{{ yaw == null ? '—' : yaw.toFixed(1) + '°' }}</b></div>
      <div class="kv"><span>底盘 yaw_speed</span><b>{{ yawSpeed == null ? '—' : yawSpeed.toFixed(2) }}</b></div>
      <div class="kv"><span>EKF 速度</span>
        <b>{{ odom ? `${odom.vx.toFixed(2)} m/s  ${odom.wz.toFixed(2)} rad/s` : '—' }}</b></div>
      <div class="kv"><span>cmd_vel</span>
        <b>{{ cmd ? `${cmd.vx.toFixed(2)} m/s  ${cmd.wz.toFixed(2)} rad/s` : '—' }}</b></div>
      <div class="kv"><span>定位得分</span><b>{{ score == null ? '—' : score.toFixed(2) }}</b></div>
      <div class="kv"><span>导航</span>
        <b :class="navClass">{{ navText }}</b></div>
    </section>

    <section>
      <h4>话题频率</h4>
      <div v-for="t in TOPICS" :key="t.name" class="row">
        <span class="led" :class="rateClass(t)" />
        <span class="name mono">{{ t.name }}</span>
        <span class="muted mono">{{ rate(t.name) }}</span>
      </div>
      <div class="row"><span class="led" :class="tfOk ? 'ok' : 'bad'" />
        <span class="name">TF map→base_footprint</span><span class="muted">{{ tfOk ? '通' : '断' }}</span></div>
    </section>

    <section>
      <h4>自检 <button class="sm" @click="doDiag">检测</button></h4>
      <div v-for="i in diagItems" :key="i.label" class="row">
        <span class="led" :class="i.ok ? 'ok' : 'bad'" />
        <span class="name">{{ i.label }}</span>
        <span class="muted small">{{ i.reason }}</span>
      </div>
    </section>
  </div>
</template>

<script setup>
import { ref, computed, onMounted, onUnmounted } from 'vue'
import { api } from '../lib/api.js'
import { rosState, subscribe } from '../lib/ros.js'
import { lookup } from '../lib/tf.js'
import { store, run, GOAL_STATUS } from '../lib/store.js'

// expect: 正常频率下限，用来标红
const TOPICS = [
  { name: '/scan', expect: 5 },
  { name: '/robot/yaw', expect: 20 },
  { name: '/lio_odom', expect: 3 },
  { name: '/lio_ekf_odom', expect: 10 },
  { name: '/tf', expect: 10 },
  { name: '/map', expect: 0 },
]

const procs = computed(() => store.status.processes)
const yaw = ref(null), yawSpeed = ref(null), odom = ref(null), cmd = ref(null), score = ref(null)
const tfOk = ref(false)
const diagItems = ref([])
const unsubs = []

const deg = (r) => (r * 180 / Math.PI).toFixed(1)
const rate = (n) => { const r = rosState.rates[n]; return r ? `${r.toFixed(1)} Hz` : '—' }
function rateClass(t) {
  const r = rosState.rates[t.name] || 0
  if (t.expect === 0) return r > 0 ? 'ok' : ''
  return r >= t.expect ? 'ok' : r > 0 ? 'warn' : 'bad'
}
function procText(p) {
  if (p.running) return `运行 ${fmtDur(Date.now() / 1000 - p.started_at)}`
  if (p.exit_code != null) return `退出 code=${p.exit_code}`
  return '未启动'
}
function fmtDur(s) {
  s = Math.floor(s)
  return s < 60 ? `${s}s` : s < 3600 ? `${Math.floor(s / 60)}m${s % 60}s` : `${Math.floor(s / 3600)}h${Math.floor(s % 3600 / 60)}m`
}

const navText = computed(() => {
  const { nav, wp } = store.navStatus
  const parts = []
  if (nav) parts.push(`单点: ${GOAL_STATUS[nav] || nav}`)
  if (wp) parts.push(`航点: ${GOAL_STATUS[wp] || wp}`)
  return parts.join('  ') || '—'
})
const navClass = computed(() => {
  const s = store.navStatus.wp && store.navStatus.wp < 4 ? store.navStatus.wp : store.navStatus.nav
  return { 4: 'ok-t', 6: 'bad-t', 2: 'warn-t' }[s] || ''
})

async function doDiag() {
  try { diagItems.value = (await api.diag()).items } catch (e) { diagItems.value = [{ label: '自检失败', ok: false, reason: e.message }] }
}

onMounted(() => {
  unsubs.push(subscribe('/robot/yaw', 'std_msgs/Float32', (m) => { yaw.value = m.data }, { throttle_rate: 200 }))
  unsubs.push(subscribe('/robot/yaw_speed', 'std_msgs/Float32', (m) => { yawSpeed.value = m.data }, { throttle_rate: 200 }))
  unsubs.push(subscribe('/lio_ekf_odom', 'nav_msgs/Odometry', (m) => {
    odom.value = { vx: m.twist.twist.linear.x, wz: m.twist.twist.angular.z }
  }, { throttle_rate: 200 }))
  unsubs.push(subscribe('/lio_odom', 'nav_msgs/Odometry', () => {}))
  unsubs.push(subscribe('/cmd_vel', 'geometry_msgs/Twist', (m) => { cmd.value = { vx: m.linear.x, wz: m.angular.z } }))
  unsubs.push(subscribe('/localizer/score', 'std_msgs/Float32', (m) => { score.value = m.data }, { throttle_rate: 500 }))
  const t = setInterval(() => { tfOk.value = !!lookup('map', 'base_footprint') }, 500)
  unsubs.push(() => clearInterval(t))
  doDiag()
})
onUnmounted(() => unsubs.forEach((u) => u()))
</script>
