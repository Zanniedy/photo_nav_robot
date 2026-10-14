<template>
  <Login v-if="store.auth.checked && !store.auth.ok" />
  <div v-else-if="store.auth.ok" class="app">
    <TopBar />
    <div class="main">
      <MapView />
      <aside class="side" :style="{ width: sideW + 'px' }">
        <div class="resizer" @pointerdown="resizeStart" />
        <nav class="tabs">
          <button v-for="t in TABS" :key="t.id" :class="{ on: tab === t.id }" @click="setTab(t.id)">{{ t.label }}</button>
        </nav>
        <!-- v-show：切 tab 不销毁，日志和命令输出不丢 -->
        <div class="tab-body">
          <ControlPanel v-show="tab === 'control'" />
          <StatusPanel v-show="tab === 'status'" />
          <CommandPanel v-show="tab === 'cmd'" />
          <LogPanel v-show="tab === 'log'" />
        </div>
      </aside>
    </div>
  </div>
  <div v-if="store.toast" class="toast" :class="store.toast.kind" @click="store.toast = null">{{ store.toast.text }}</div>
</template>

<script setup>
import { ref, watch } from 'vue'
import Login from './components/Login.vue'
import TopBar from './components/TopBar.vue'
import MapView from './components/MapView.vue'
import ControlPanel from './components/ControlPanel.vue'
import StatusPanel from './components/StatusPanel.vue'
import CommandPanel from './components/CommandPanel.vue'
import LogPanel from './components/LogPanel.vue'
import { store, checkAuth } from './lib/store.js'
import { startRos } from './lib/ros.js'

checkAuth()
watch(() => store.auth.ok, (ok) => { if (ok) startRos() })

const TABS = [
  { id: 'control', label: '控制' },
  { id: 'status', label: '状态' },
  { id: 'cmd', label: '命令' },
  { id: 'log', label: '日志' },
]
const read = (k, d) => { try { return localStorage.getItem(k) || d } catch { return d } }
const tab = ref(read('robot-ui-tab', 'control'))
const sideW = ref(Number(read('robot-ui-side', 380)))

function setTab(id) {
  tab.value = id
  try { localStorage.setItem('robot-ui-tab', id) } catch { /* 无痕模式 */ }
}

function resizeStart(e) {
  const x0 = e.clientX, w0 = sideW.value
  const move = (ev) => { sideW.value = Math.max(300, Math.min(800, w0 + x0 - ev.clientX)) }
  const up = () => {
    window.removeEventListener('pointermove', move)
    window.removeEventListener('pointerup', up)
    try { localStorage.setItem('robot-ui-side', String(sideW.value)) } catch { /* 无痕模式 */ }
  }
  window.addEventListener('pointermove', move)
  window.addEventListener('pointerup', up)
}
</script>

<style>
:root {
  --bg: #16181f; --panel: #20232c; --line: #343947; --text: #dfe4ee; --muted: #8a93a6;
  --accent: #5fa8ff; --ok: #3ddc84; --warn: #ffc94d; --bad: #ff5c5c;
}
* { margin: 0; padding: 0; box-sizing: border-box; }
html, body, #app { height: 100%; overflow: hidden; }
body { background: var(--bg); color: var(--text); font: 13px/1.4 system-ui, -apple-system, 'PingFang SC', 'Microsoft YaHei', sans-serif; }
button, select, input { font: inherit; color: var(--text); background: #2b2f3b; border: 1px solid var(--line); border-radius: 4px; padding: 4px 10px; }
button { cursor: pointer; white-space: nowrap; }
button:hover:not(:disabled) { background: #353a48; }
button.on { background: #1d3d66; border-color: var(--accent); color: #cfe4ff; }
button.danger { background: #4a1f24; border-color: #7a3038; color: #ffb3b3; }
button:disabled { opacity: 0.45; cursor: default; }
button.sm { padding: 1px 8px; font-size: 12px; }
input { background: #15171d; min-width: 0; }
input:focus, select:focus { outline: 1px solid var(--accent); }
input.num { width: 64px; }
.mono { font-family: ui-monospace, 'JetBrains Mono', monospace; }
.small { font-size: 11px; }
.muted { color: var(--muted); }
.ok-t { color: var(--ok); } .warn-t { color: var(--warn); } .bad-t { color: var(--bad); }
.chk { display: inline-flex; align-items: center; gap: 4px; font-size: 12px; font-weight: normal; cursor: pointer; }

.panel { padding: 4px 10px 10px; }
.panel section { padding: 8px 0; border-bottom: 1px solid var(--line); }
.panel h4 { font-size: 12px; color: var(--muted); font-weight: 600; margin-bottom: 6px; display: flex; align-items: center; gap: 8px; }
.panel .row { display: flex; align-items: center; gap: 8px; padding: 2px 0; }
.panel .row .name { flex: 1; min-width: 0; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
.panel .row .name.cur { color: var(--accent); font-weight: 600; }
.panel .line { display: flex; gap: 6px; align-items: center; margin: 4px 0; }
.panel .line input:not(.num) { flex: 1; }
.panel .line.wrap { flex-wrap: wrap; }
.panel .kv { display: flex; justify-content: space-between; gap: 8px; padding: 2px 0; font-size: 12px; }
.panel .kv b { font-weight: 500; font-variant-numeric: tabular-nums; }
.led { width: 8px; height: 8px; border-radius: 50%; background: #4a5060; flex-shrink: 0; }
.led.ok { background: var(--ok); } .led.warn { background: var(--warn); } .led.bad { background: var(--bad); }
</style>

<style scoped>
.app { display: flex; flex-direction: column; height: 100%; }
.main { display: flex; flex: 1; min-height: 0; }
.side { position: relative; display: flex; flex-direction: column; background: var(--panel); border-left: 1px solid var(--line); flex-shrink: 0; }
.resizer { position: absolute; left: -3px; top: 0; bottom: 0; width: 6px; cursor: col-resize; z-index: 2; }
.tabs { display: flex; border-bottom: 1px solid var(--line); }
.tabs button { flex: 1; border: 0; border-radius: 0; background: transparent; padding: 8px 0; color: var(--muted); border-bottom: 2px solid transparent; }
.tabs button.on { color: var(--text); border-bottom-color: var(--accent); background: transparent; }
.tab-body { flex: 1; overflow: auto; min-height: 0; }
.tab-body > * { min-height: 100%; }
.toast { position: fixed; bottom: 16px; left: 50%; transform: translateX(-50%); background: #263247; border: 1px solid var(--accent); padding: 8px 16px; border-radius: 6px; max-width: 70vw; white-space: pre-wrap; z-index: 10; cursor: pointer; font-size: 12px; }
.toast.error { background: #3d1e22; border-color: var(--bad); }
</style>
