<template>
  <header class="bar">
    <span class="title">Robot UI</span>
    <span class="dot" :class="store.backendOk ? 'ok' : 'bad'">后端</span>
    <span class="dot" :class="rosState.connected ? 'ok' : 'bad'" :title="rosState.url">rosbridge</span>
    <span class="mode" :class="mode">{{ MODE_LABEL[mode] }}<template v-if="store.status.map"> · {{ store.status.map }}</template></span>

    <div class="spacer" />

    <button v-if="!procs.base?.running" @click="run('启动底盘', () => api.procStart('base'))" :disabled="!!store.busy"
            title="robot.launch.py slam:=false：bridge + 雷达 + lio + ekf + 静态 TF">启动底盘</button>
    <button @click="startMapping" :disabled="!!store.busy" :class="{ on: mode === 'mapping' }">建图</button>
    <select v-model="navMap" :disabled="!!store.busy" title="选择地图">
      <option value="" disabled>选择地图…</option>
      <option v-for="m in store.maps" :key="m.name" :value="m.name">{{ m.name }}</option>
    </select>
    <button @click="startNav" :disabled="!!store.busy || !navMap" :class="{ on: mode === 'navigation' }">
      {{ mode === 'navigation' ? '切换地图' : '导航' }}
    </button>
    <button v-if="mode === 'mapping' || mode === 'navigation'" @click="run('退出模式', api.stop)" :disabled="!!store.busy">退出{{ MODE_LABEL[mode] }}</button>
    <button class="danger" @click="stopAll" :disabled="!!store.busy" title="停掉底盘/建图/导航（保留 rosbridge）">全部停止</button>
    <span v-if="store.busy" class="busy">{{ store.busy }}…</span>
  </header>
</template>

<script setup>
import { ref, computed, watch } from 'vue'
import { api } from '../lib/api.js'
import { rosState } from '../lib/ros.js'
import { store, run, refreshMaps, toast } from '../lib/store.js'

const MODE_LABEL = { idle: '待机', base: '底盘运行', mapping: '建图', navigation: '导航' }
const mode = computed(() => store.status.mode)
const procs = computed(() => store.status.processes)
const navMap = ref('')

watch(() => store.status.map, (m) => { if (m) navMap.value = m }, { immediate: true })
watch(() => store.maps, (ms) => { if (!navMap.value && ms.length) navMap.value = ms[0].name })

async function startMapping() {
  if (mode.value === 'mapping') return
  if (mode.value === 'navigation' && !confirm('将停止导航并开始建图，继续？')) return
  await run('启动建图', api.mappingStart)
  store.goal = null
}

async function startNav() {
  if (mode.value === 'mapping' && !confirm('将停止建图（未保存的地图会丢失）并启动导航，继续？')) return
  const r = mode.value === 'navigation'
    ? await run('切换地图', () => api.switchMap(navMap.value))
    : await run('启动导航', () => api.navStart(navMap.value))
  if (r) {
    store.goal = null
    store.waypoints = []
    toast(`地图 ${navMap.value} 已加载，用「初始位姿」工具标定机器人位置`)
  }
}

async function stopAll() {
  if (!confirm('停止底盘、建图、导航所有进程？')) return
  await run('全部停止', api.stopAll)
}

refreshMaps()
</script>

<style scoped>
.bar { display: flex; align-items: center; gap: 8px; padding: 6px 12px; background: var(--panel); border-bottom: 1px solid var(--line); flex-wrap: wrap; }
.title { font-weight: 700; color: var(--accent); margin-right: 6px; }
.dot { font-size: 12px; color: var(--muted); }
.dot::before { content: '●'; margin-right: 3px; }
.dot.ok::before { color: var(--ok); }
.dot.bad::before { color: var(--bad); }
.mode { font-size: 12px; padding: 2px 10px; border-radius: 10px; background: #333a48; }
.mode.mapping { background: #1f4a2c; color: #8f8; }
.mode.navigation { background: #1f3256; color: #8bf; }
.mode.base { background: #403a1f; color: #fd7; }
.spacer { flex: 1; }
.busy { font-size: 12px; color: var(--warn); }
</style>
