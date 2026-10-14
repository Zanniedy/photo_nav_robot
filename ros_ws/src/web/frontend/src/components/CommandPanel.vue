<template>
  <div class="panel cmd">
    <section v-for="(items, group) in groups" :key="group">
      <h4>{{ group }}</h4>
      <div class="line wrap">
        <button v-for="c in items" :key="c.id" class="sm" :disabled="running" @click="quick(c)">{{ c.label }}</button>
      </div>
    </section>

    <section>
      <h4>ros2 命令</h4>
      <div class="line">
        <input class="mono" v-model="cli" placeholder="ros2 topic echo /robot/yaw --once" list="cli-hist"
               @keydown.enter="runCli" @keydown.up.prevent="histUp" @keydown.down.prevent="histDown" />
        <select v-model.number="timeout" title="超时（hz/echo 这类不退出的命令到点 Ctrl-C）">
          <option :value="3">3s</option><option :value="6">6s</option><option :value="15">15s</option><option :value="30">30s</option>
        </select>
        <button :disabled="running || !cli.trim()" @click="runCli">运行</button>
      </div>
      <div class="small muted">支持 ros2 topic / node / service / param / action / interface / lifecycle / pkg / doctor</div>
    </section>

    <section class="out-sec">
      <h4>输出 <span v-if="running" class="warn-t small">运行中…</span>
        <button class="sm" @click="output = ''">清空</button></h4>
      <pre class="out" ref="outEl">{{ output }}</pre>
    </section>
  </div>
</template>

<script setup>
import { ref, computed, nextTick, onMounted } from 'vue'
import { api } from '../lib/api.js'

const list = ref([])
const running = ref(false)
const output = ref('')
const outEl = ref(null)
const cli = ref('')
const timeout = ref(6)
const history = []
let histIdx = -1

const groups = computed(() => {
  const g = {}
  list.value.forEach((c) => { (g[c.group] ||= []).push(c) })
  return g
})

function append(text) {
  output.value += text + '\n'
  if (output.value.length > 60000) output.value = output.value.slice(-50000)
  nextTick(() => { if (outEl.value) outEl.value.scrollTop = outEl.value.scrollHeight })
}

async function exec(title, fn) {
  running.value = true
  append(`\n▶ ${title}`)
  try {
    const r = await fn()
    append(r.output.trimEnd() || '(无输出)')
    if (r.rc && !r.timed_out) append(`[rc=${r.rc}]`)
  } catch (e) {
    append(`✗ ${e.message}`)
  } finally {
    running.value = false
  }
}

const quick = (c) => exec(c.label, () => api.quick(c.id))

function runCli() {
  const cmd = cli.value.trim()
  if (!cmd || running.value) return
  if (history[0] !== cmd) history.unshift(cmd)
  histIdx = -1
  exec(`$ ${cmd}`, () => api.cli(cmd, timeout.value))
}
function histUp() { if (histIdx < history.length - 1) cli.value = history[++histIdx] }
function histDown() { if (histIdx > 0) cli.value = history[--histIdx]; else { histIdx = -1; cli.value = '' } }

onMounted(async () => {
  try { list.value = await api.quickList() } catch (e) { append(`✗ 读取快捷命令失败: ${e.message}`) }
})
</script>

<style scoped>
.cmd { display: flex; flex-direction: column; height: 100%; }
.out-sec { flex: 1; display: flex; flex-direction: column; min-height: 200px; }
.out { flex: 1; overflow: auto; background: #12141a; border: 1px solid var(--line); border-radius: 4px; padding: 6px; font-size: 11px; line-height: 1.4; white-space: pre-wrap; word-break: break-all; color: #cfd6e4; }
</style>
