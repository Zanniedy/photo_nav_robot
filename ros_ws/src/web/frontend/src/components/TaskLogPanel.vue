<template>
  <div class="panel tlog">
    <div class="line top">
      <h4 class="grow">巡逻</h4>
      <span v-if="!store.status?.map" class="small muted">先开导航</span>
      <template v-else-if="routes.length">
        <select v-model="routeId" :disabled="patrolActive">
          <option v-for="r in routes" :key="r.id" :value="r.id">{{ r.name }}（{{ r.point_names.length }} 点{{ r.loop ? '，循环' : '' }}）</option>
        </select>
        <button v-if="!patrolActive" class="sm on" :disabled="!!store.busy" @click="startPatrol">一键巡逻</button>
      </template>
      <span v-else class="small muted">这张地图没有路线（~/max_robot/patrol.yaml）</span>
      <button v-if="patrolActive" class="sm danger" :disabled="!!store.busy" @click="stopPatrol">停止巡逻</button>
    </div>
    <div v-if="routeHint" class="small muted">{{ routeHint }}</div>

    <h4>当前 <b v-if="m" class="mstate" :class="MCLASS[m.state]">{{ m.state }}</b>
      <span v-if="m" class="small muted">{{ m.since_s.toFixed(0) }}s</span></h4>
    <div v-if="loc" class="small">
      定位 <b :class="loc.state === 'tracking' ? 'ok-t' : 'warn-t'">{{ LOC[loc.state] || loc.state }}</b>
      · 内点率 {{ loc.score.toFixed(2) }} · 动态点 {{ (loc.dynamic * 100).toFixed(0) }}%
      <span v-if="loc.state === 'recovering'"> · 已搜到 {{ loc.recover.radius.toFixed(1) }}m，最好 {{ loc.recover.score.toFixed(2) }}{{ loc.recover.confirming ? '，确认中' : '' }}</span>
    </div>
    <div v-else class="small muted">定位：没开导航 / 没有 /localizer/state</div>
    <div v-if="patrol" class="small">
      巡逻 <b>{{ STATE[patrol.state] || patrol.state }}</b>
      · {{ patrol.index + 1 }}/{{ patrol.points.length }}<span v-if="patrol.target"> → {{ patrol.target }}</span>
      <span v-if="patrol.loop"> · 第 {{ patrol.laps + 1 }} 圈</span>
      <span v-if="patrol.wait_reason" class="warn-t"> · 暂停（{{ REASON[patrol.wait_reason] || patrol.wait_reason }}）</span>
    </div>
    <div v-if="fg" class="small">{{ TYPE[fg.type] || fg.type }} <b>{{ STATE[fg.state] || fg.state }}</b>
      <span v-if="fg.target"> → {{ fg.target }}</span></div>
    <div v-if="!patrol && !fg" class="small muted">没有任务</div>
    <div v-if="g" class="small">
      遇到人 <b :class="{ 'ok-t': g.state === 'greet', 'warn-t': g.state === 'search' }">{{ GREET[g.state] || g.state }}</b>
      <span v-if="g.distance != null && g.state !== 'idle'"> · {{ g.distance.toFixed(1) }}m</span>
      <span v-if="g.elapsed_s != null"> · {{ g.elapsed_s.toFixed(0) }}s</span>
      <span v-if="g.state === 'idle' && g.cooldown_s > 0" class="muted"> · 冷却 {{ g.cooldown_s.toFixed(0) }}s</span>
      <span v-if="!g.enabled" class="muted">（未启用）</span>
    </div>

    <div class="line">
      <h4 class="grow">状态机（每 3s）</h4>
      <button class="sm" @click="ticks = []">清空</button>
    </div>
    <div ref="tbox" class="box tick">
      <div v-for="l in ticks" :key="l.seq" class="ln"><span class="t">{{ fmt(l.t) }}</span> {{ l.text }}</div>
      <div v-if="!ticks.length" class="muted">后端每 3s 打一行当前状态</div>
    </div>

    <div class="line">
      <h4 class="grow">任务日志</h4>
      <label class="chk"><input v-model="autoScroll" type="checkbox" /> 自动滚动</label>
      <button class="sm" @click="lines = []">清空</button>
    </div>
    <div ref="box" class="box" @scroll="onScroll">
      <div v-for="l in lines" :key="l.seq" class="ln" :class="l.kind">
        <span class="t">{{ fmt(l.t) }}</span> {{ l.text }}
      </div>
      <div v-if="!lines.length" class="muted">还没有记录（巡逻开始、暂停、遇到人、打招呼都会记在这里）</div>
    </div>
  </div>
</template>

<script setup>
import { ref, computed, watch, nextTick, onMounted, onUnmounted } from 'vue'
import { api } from '../lib/api.js'
import { store, run, toast } from '../lib/store.js'

const STATE = { running: '执行中', waiting: '暂停', succeeded: '完成', failed: '失败', canceled: '已取消', preempted: '被顶掉' }
const REASON = { manual: '手动', person: '遇到人', subtask: '子任务' }
const TYPE = { goto: '前往', guide: '引导', patrol: '巡逻' }
const GREET = { idle: '空闲', search: '搜索（走向人）', greet: '打招呼' }
const LOC = { tracking: '跟踪', lost: '丢失', recovering: '恢复中' }
const MCLASS = { 巡逻: 'ok-t', 人脸: 'ok-t', 等待: 'ok-t', 丢失: 'bad-t', 恢复: 'warn-t', 暂停: 'warn-t' }

const cur = ref(null)
const g = ref(null)
const lines = ref([])
const ticks = ref([])
const m = ref(null)
const tbox = ref(null)
const loc = computed(() => m.value?.localizer)
const autoScroll = ref(true)
const box = ref(null)
const patrol = computed(() => cur.value?.patrol)
const fg = computed(() => cur.value?.foreground)
let seq = 0, timer = 0

// 巡逻：路线来自当前导航地图（patrol.yaml 同步进来的 + API 建的），记住上次选的
const routes = ref([])
const routeId = ref('')
const LAST_ROUTE = 'max.patrol.route'
const patrolActive = computed(() => ['running', 'waiting'].includes(patrol.value?.state))
const routeHint = computed(() => {
  const r = routes.value.find((x) => x.id === routeId.value)
  return r && !patrolActive.value ? r.point_names.join(' → ') + (r.loop ? ' → …' : '') : ''
})
async function loadRoutes() {
  if (!store.status?.map) { routes.value = []; return }
  try {
    routes.value = (await api.routes()).data.routes
  } catch { routes.value = [] }
  let last = ''
  try { last = localStorage.getItem(LAST_ROUTE) || '' } catch { /* 隐私模式 */ }
  if (!routes.value.some((r) => r.id === routeId.value))
    routeId.value = (routes.value.find((r) => r.id === last) || routes.value[0])?.id || ''
}
async function startPatrol() {
  const r = routes.value.find((x) => x.id === routeId.value)
  if (!r) return
  try { localStorage.setItem(LAST_ROUTE, r.id) } catch { /* 隐私模式 */ }
  const t = await run('开始巡逻', () => api.startPatrol(r.id))
  if (t) { toast(`开始巡逻：${r.name}`); poll() }
}
async function stopPatrol() {
  if (!patrol.value) return
  await run('停止巡逻', () => api.cancelTask(patrol.value.id))
  poll()
}
watch(() => store.status?.map, loadRoutes, { immediate: true })

const fmt = (t) => new Date(t * 1000).toLocaleTimeString('zh-CN', { hour12: false })

async function poll() {
  try {
    const r = await api.taskLog(seq)
    cur.value = r.current
    g.value = r.greeter
    m.value = r.mission
    if (r.lines.length) {
      seq = r.lines[r.lines.length - 1].seq
      lines.value.push(...r.lines.filter((l) => l.kind !== 'tick'))
      ticks.value.push(...r.lines.filter((l) => l.kind === 'tick'))
      if (lines.value.length > 500) lines.value.splice(0, lines.value.length - 500)
      if (ticks.value.length > 200) ticks.value.splice(0, ticks.value.length - 200)
    }
  } catch { /* 后端断开时状态栏会显示 */ }
}
function onScroll() {
  const b = box.value
  autoScroll.value = b.scrollHeight - b.scrollTop - b.clientHeight < 30
}
watch(() => ticks.value.length, () => nextTick(() => { if (tbox.value) tbox.value.scrollTop = tbox.value.scrollHeight }))
watch(() => lines.value.length, () => {
  if (autoScroll.value) nextTick(() => { if (box.value) box.value.scrollTop = box.value.scrollHeight })
})
onMounted(() => { poll(); timer = setInterval(poll, 1000) })
onUnmounted(() => clearInterval(timer))
</script>

<style scoped>
.tlog { display: flex; flex-direction: column; height: 100%; }
.line.top { margin-top: 0; flex-wrap: wrap; }
.line.top select { max-width: 220px; }
.grow { flex: 1; margin: 0; }
.line { display: flex; gap: 6px; align-items: center; margin-top: 10px; }
.box { flex: 1; overflow: auto; background: #12141a; border: 1px solid var(--line); border-radius: 4px; padding: 4px 6px;
  font: 12px/1.5 ui-monospace, monospace; margin-top: 6px; min-height: 120px; }
.ln { white-space: pre-wrap; word-break: break-all; color: #b9c3d3; }
.ln .t { color: #6b7385; }
.ln.greet { color: #7fd3ff; }
.ln.mission { color: #ffd166; }
.box.tick { flex: 0 0 auto; height: 150px; min-height: 0; }
.mstate { margin-left: 6px; }
.ln.estop { color: #ff7b7b; }
</style>
