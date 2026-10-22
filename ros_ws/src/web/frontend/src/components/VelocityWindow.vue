<template>
  <div class="vel">
    <div class="head">
      <span class="led" :class="hz > 0 ? 'ok' : 'bad'" />
      <b>速度曲线</b>
      <span class="muted small mono">{{ hz ? hz.toFixed(0) + ' Hz' : '无数据' }}</span>
      <div class="spacer" />
      <button v-for="w in WINDOWS" :key="w" class="sm" :class="{ on: span === w }" @click="setSpan(w)" @pointerdown.stop>{{ w }}s</button>
      <button class="sm" :class="{ on: paused }" @click="togglePause" @pointerdown.stop :title="paused ? '继续' : '暂停（冻结画面，数据照收）'">{{ paused ? '继续' : '暂停' }}</button>
      <button class="sm" @click="clear" @pointerdown.stop title="清空曲线">清空</button>
      <button class="sm" @click="exportCsv" @pointerdown.stop :disabled="!count" title="下载缓存的全部数据（最多 120s）">CSV</button>
      <button class="sm" @click="store.velOpen = false" @pointerdown.stop title="关闭">✕</button>
    </div>

    <div v-if="!hz && !count" class="empty muted small">
      没有收到 /robot/cmd_debug。底盘（robot_bridge）没启动，或 bridge 还是旧版本。
    </div>

    <div v-for="c in CHARTS" :key="c.key" class="chart">
      <div class="ctitle">
        <b>{{ c.title }}</b><span class="muted small">{{ c.unit }}</span>
        <div class="spacer" />
        <span class="lg"><i class="sw" :style="{ background: C_TARGET }" />目标</span>
        <span class="lg"><i class="sw" :style="{ background: C_REAL }" />实际</span>
        <span class="small muted mono" :title="c.key === 'vx' ? '当前窗口内 |目标−实际| 的 RMS（含底盘滞后）' : ''">误差 RMS {{ fmt(stats[c.key].rms) }}</span>
      </div>
      <div class="cv-wrap" @pointermove="hover" @pointerleave="hoverT = null">
        <canvas :ref="(el) => (cvs[c.key] = el)" />
        <div v-if="tip && tip.key === c.key" class="tip mono small" :style="{ left: tip.x + 'px' }">
          <div class="muted">{{ tip.t }}</div>
          <div><i class="sw" :style="{ background: C_TARGET }" /> 目标 {{ tip.target }}</div>
          <div><i class="sw" :style="{ background: C_REAL }" /> 实际 {{ tip.real }}</div>
        </div>
      </div>
      <div class="kv small mono muted">
        <template v-if="c.key === 'vx'">u {{ fmt(last?.[3], 3) }} · 前馈增益 k {{ fmt(last?.[4], 3) }} · 平滑后指令 {{ fmt(last?.[1]) }}</template>
        <template v-else>u {{ fmt(last?.[8], 3) }} · PI 积分 {{ fmt(last?.[9], 3) }} · 平滑后指令 {{ fmt(last?.[6]) }}</template>
      </div>
    </div>
    <div class="note small muted">目标 = cmd_vel；实际：线速度来自 EKF 速度估计（约 0.2s 滞后），角速度来自陀螺。时间是浏览器收到消息的时刻。</div>
  </div>
</template>

<script setup>
import { ref, reactive, computed, onMounted, onUnmounted } from 'vue'
import { subscribe, rosState } from '../lib/ros.js'
import { store } from '../lib/store.js'

// /robot/cmd_debug 字段顺序（robot_bridge_node.cpp control_step）：
// [target_vx, cur_vx, meas_vx, u_vx, k_vx, target_wz, cur_wz, meas_wz, u_wz, i_wz]
const TOPIC = '/robot/cmd_debug'
const CHARTS = [
  { key: 'vx', title: '线速度', unit: 'm/s', target: 0, real: 2, minSpan: 0.1 },
  { key: 'wz', title: '角速度', unit: 'rad/s', target: 5, real: 7, minSpan: 0.3 },
]
const WINDOWS = [10, 30, 60]
const KEEP_S = 120
// 两个序列靠颜色区分：深色面板上校验过的蓝/橙（色弱下也能分开），图例和悬停提示带文字
const C_TARGET = '#3987e5'
const C_REAL = '#d95926'

const read = (k, d) => { try { return JSON.parse(localStorage.getItem(k)) ?? d } catch { return d } }
const span = ref(read('robot-ui-velspan', 10))
const setSpan = (w) => { span.value = w; try { localStorage.setItem('robot-ui-velspan', JSON.stringify(w)) } catch { /* 无痕模式 */ } }

// 环形缓冲：普通数组，不做响应式（50Hz × 120s）
let buf = []
const count = ref(0)
const last = ref(null)
const paused = ref(false)
let frozenNow = 0
const hz = computed(() => rosState.rates[TOPIC] || 0)
const stats = reactive({ vx: { rms: NaN }, wz: { rms: NaN } })

const unsub = subscribe(TOPIC, 'std_msgs/Float32MultiArray', (m) => {
  const t = performance.now() / 1000
  buf.push({ t, d: m.data })
  if (buf.length > 64 && t - buf[0].t > KEEP_S) buf = buf.filter((s) => t - s.t <= KEEP_S)
  last.value = m.data
  count.value = buf.length
  dirty = true
})

function clear() { buf = []; count.value = 0; dirty = true }
function togglePause() { paused.value = !paused.value; frozenNow = performance.now() / 1000; dirty = true }

function fmt(v, n = 2) { return v == null || Number.isNaN(v) ? '—' : (v >= 0 ? '+' : '') + Number(v).toFixed(n) }

// ---------- 绘制 ----------
const cvs = {}
let dirty = true
let raf = 0
const hoverT = ref(null)      // 悬停的时间（秒，相对 now，负数）
const tip = ref(null)

function niceStep(range, n = 4) {
  const raw = range / n, p = Math.pow(10, Math.floor(Math.log10(raw)))
  return [1, 2, 2.5, 5, 10].map((m) => m * p).find((s) => s >= raw)
}

function visible(now) {
  const t0 = now - span.value
  let i = buf.length
  while (i > 0 && buf[i - 1].t >= t0) i--
  return buf.slice(Math.max(0, i - 1))
}

function draw() {
  raf = requestAnimationFrame(draw)
  if (!dirty && paused.value) return
  dirty = false
  const now = paused.value ? frozenNow : performance.now() / 1000
  const rows = visible(now)
  const css = getComputedStyle(document.documentElement)
  const cText = css.getPropertyValue('--muted').trim() || '#8a93a6'
  const cGrid = css.getPropertyValue('--line').trim() || '#343947'
  tip.value = null

  for (const c of CHARTS) {
    const cv = cvs[c.key]
    if (!cv) continue
    const dpr = window.devicePixelRatio || 1
    const W = cv.clientWidth, H = cv.clientHeight
    if (cv.width !== Math.round(W * dpr) || cv.height !== Math.round(H * dpr)) { cv.width = Math.round(W * dpr); cv.height = Math.round(H * dpr) }
    const g = cv.getContext('2d')
    g.setTransform(dpr, 0, 0, dpr, 0, 0)
    g.clearRect(0, 0, W, H)
    const L = 44, R = 8, T = 6, B = 18, pw = W - L - R, ph = H - T - B

    // y 范围：包含 0，至少 minSpan，留 10% 余量
    let lo = 0, hi = 0, se = 0, ne = 0
    for (const s of rows) {
      const a = s.d[c.target], b = s.d[c.real]
      if (a < lo) lo = a; if (a > hi) hi = a
      if (!Number.isNaN(b)) { if (b < lo) lo = b; if (b > hi) hi = b; se += (a - b) ** 2; ne++ }
    }
    stats[c.key].rms = ne ? Math.sqrt(se / ne) : NaN
    if (hi - lo < c.minSpan) { const m = (hi + lo) / 2; lo = m - c.minSpan / 2; hi = m + c.minSpan / 2 }
    const pad = (hi - lo) * 0.1; lo -= pad; hi += pad
    const X = (t) => L + (1 - (now - t) / span.value) * pw
    const Y = (v) => T + (hi - v) / (hi - lo) * ph

    // 网格 + 刻度（弱化）
    g.font = '11px ui-monospace, monospace'
    g.fillStyle = cText; g.strokeStyle = cGrid; g.lineWidth = 1
    g.textAlign = 'right'; g.textBaseline = 'middle'
    const ys = niceStep(hi - lo)
    for (let v = Math.ceil(lo / ys) * ys; v <= hi + 1e-9; v += ys) {
      const y = Math.round(Y(v)) + 0.5
      g.globalAlpha = Math.abs(v) < 1e-9 ? 1 : 0.5
      g.beginPath(); g.moveTo(L, y); g.lineTo(L + pw, y); g.stroke()
      g.globalAlpha = 1
      g.fillText(+v.toFixed(3) + '', L - 4, y)
    }
    g.textAlign = 'center'; g.textBaseline = 'top'
    const xs = span.value <= 10 ? 2 : span.value <= 30 ? 5 : 10
    for (let s = 0; s <= span.value; s += xs) g.fillText(s ? `-${s}s` : '现在', L + pw - s / span.value * pw, T + ph + 4)

    // 曲线：2px 实线，目标蓝、实际橙
    g.save()
    g.beginPath(); g.rect(L, T, pw, ph); g.clip()
    g.lineWidth = 2; g.lineJoin = 'round'
    for (const [idx, color] of [[c.target, C_TARGET], [c.real, C_REAL]]) {
      g.strokeStyle = color; g.beginPath()
      let pen = false
      for (const s of rows) {
        const v = s.d[idx]
        if (Number.isNaN(v)) { pen = false; continue }   // 陀螺超时是 NaN：断开
        const x = X(s.t), y = Y(v)
        pen ? g.lineTo(x, y) : g.moveTo(x, y); pen = true
      }
      g.stroke()
    }
    g.restore()

    // 悬停十字线（两张图共用同一时刻）
    if (hoverT.value != null && rows.length) {
      const tt = now + hoverT.value
      let best = rows[0]
      for (const s of rows) if (Math.abs(s.t - tt) < Math.abs(best.t - tt)) best = s
      const x = X(best.t)
      g.strokeStyle = cText; g.lineWidth = 1
      g.beginPath(); g.moveTo(Math.round(x) + 0.5, T); g.lineTo(Math.round(x) + 0.5, T + ph); g.stroke()
      for (const [idx, color] of [[c.target, C_TARGET], [c.real, C_REAL]]) {
        const v = best.d[idx]
        if (Number.isNaN(v)) continue
        g.fillStyle = color; g.strokeStyle = (css.getPropertyValue('--panel').trim() || '#20232c')
        g.lineWidth = 2; g.beginPath(); g.arc(x, Y(v), 4, 0, 2 * Math.PI); g.fill(); g.stroke()
      }
      if (c.key === hoverKey) {
        tip.value = { key: c.key, x: Math.min(x + 10, W - 120), t: `${(best.t - now).toFixed(2)}s`,
          target: fmt(best.d[c.target], 3), real: fmt(best.d[c.real], 3) }
      }
    }
  }
}

let hoverKey = null
function hover(e) {
  const cv = e.currentTarget.querySelector('canvas')
  const r = cv.getBoundingClientRect()
  const L = 44, R = 8, pw = r.width - L - R
  const fx = (e.clientX - r.left - L) / pw
  hoverKey = CHARTS.find((c) => cvs[c.key] === cv)?.key
  hoverT.value = fx < 0 || fx > 1 ? null : -(1 - fx) * span.value
  dirty = true
}

function exportCsv() {
  const head = 't_s,target_vx,cur_vx,meas_vx,u_vx,k_vx,target_wz,cur_wz,meas_wz,u_wz,i_wz'
  const t0 = buf[0]?.t || 0
  const lines = buf.map((s) => [(s.t - t0).toFixed(3), ...Array.from(s.d, (v) => (Number.isNaN(v) ? '' : v.toFixed(4)))].join(','))
  const blob = new Blob([head + '\n' + lines.join('\n') + '\n'], { type: 'text/csv' })
  const a = document.createElement('a')
  a.href = URL.createObjectURL(blob)
  a.download = `cmd_debug_${new Date().toISOString().slice(0, 19).replace(/[:T]/g, '-')}.csv`
  a.click()
  setTimeout(() => URL.revokeObjectURL(a.href), 1000)
}

// 停靠栏改宽度时，暂停状态下也要按新尺寸重画
let ro = null
onMounted(() => {
  raf = requestAnimationFrame(draw)
  ro = new ResizeObserver(() => { dirty = true })
  Object.values(cvs).forEach((c) => c && ro.observe(c))
})
onUnmounted(() => { cancelAnimationFrame(raf); ro?.disconnect(); unsub() })

</script>

<style scoped>
.vel { background: var(--panel); border-bottom: 1px solid var(--line); display: flex; flex-direction: column; flex-shrink: 0; }
.head { display: flex; align-items: center; gap: 6px; padding: 5px 8px; border-bottom: 1px solid var(--line); user-select: none; flex-wrap: wrap; }
.spacer { flex: 1; }
.empty { padding: 10px; }
.chart { padding: 6px 10px 2px; }
.ctitle { display: flex; align-items: center; gap: 8px; margin-bottom: 2px; }
.lg { display: inline-flex; align-items: center; gap: 4px; font-size: 12px; }
.sw { display: inline-block; width: 16px; height: 2px; border-radius: 1px; }
.cv-wrap { position: relative; }
canvas { display: block; width: 100%; height: 150px; }
.tip { position: absolute; top: 4px; pointer-events: none; background: #15171d; border: 1px solid var(--line); border-radius: 4px; padding: 3px 6px; white-space: nowrap; }
.kv { padding: 2px 0 4px; }
.note { padding: 2px 10px 8px; }
</style>
