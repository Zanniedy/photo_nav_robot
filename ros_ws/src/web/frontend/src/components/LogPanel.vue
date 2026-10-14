<template>
  <div class="panel logs">
    <div class="line">
      <select v-model="filter">
        <option value="">全部进程</option>
        <option v-for="p in procNames" :key="p" :value="p">{{ p }}</option>
      </select>
      <input v-model="grep" placeholder="过滤关键字" />
      <label class="chk"><input type="checkbox" v-model="autoScroll" /> 自动滚动</label>
      <button class="sm" @click="lines = []">清空</button>
    </div>
    <div class="box" ref="box" @scroll="onScroll">
      <div v-for="l in shown" :key="l.seq" class="ln" :class="levelOf(l.line)">
        <span class="p">[{{ l.proc }}]</span> {{ l.line }}
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref, computed, watch, nextTick, onMounted, onUnmounted } from 'vue'

const MAX = 3000
const lines = ref([])
const filter = ref('')
const grep = ref('')
const autoScroll = ref(true)
const box = ref(null)
let es = null, lastSeq = 0, retry = null

const procNames = computed(() => [...new Set(lines.value.map((l) => l.proc))].sort())
const shown = computed(() => {
  const g = grep.value.toLowerCase()
  return lines.value.filter((l) => (!filter.value || l.proc === filter.value) && (!g || l.line.toLowerCase().includes(g))).slice(-800)
})

function levelOf(s) {
  if (/\[(ERROR|FATAL)\]|error|Error|Traceback|exit code=[1-9]|code=-?[1-9]/.test(s)) return 'err'
  if (/\[WARN\]|warn/i.test(s)) return 'warn'
  if (s.startsWith('---')) return 'meta'
  return ''
}

function connect() {
  es = new EventSource(`/api/logs?since=${lastSeq}`)
  let batch = []
  let flush = null
  es.onmessage = (e) => {
    const d = JSON.parse(e.data)
    lastSeq = d.seq
    batch.push(d)
    flush ||= setTimeout(() => {
      lines.value.push(...batch)
      if (lines.value.length > MAX) lines.value.splice(0, lines.value.length - MAX)
      batch = []
      flush = null
    }, 150)
  }
  es.onerror = () => { es.close(); retry = setTimeout(connect, 2000) }
}

function onScroll() {
  const b = box.value
  autoScroll.value = b.scrollHeight - b.scrollTop - b.clientHeight < 30
}

watch(shown, () => {
  if (autoScroll.value) nextTick(() => { if (box.value) box.value.scrollTop = box.value.scrollHeight })
})

onMounted(connect)
onUnmounted(() => { es?.close(); clearTimeout(retry) })
</script>

<style scoped>
.logs { display: flex; flex-direction: column; height: 100%; }
.box { flex: 1; overflow: auto; background: #12141a; border: 1px solid var(--line); border-radius: 4px; padding: 4px 6px; font: 11px/1.45 ui-monospace, monospace; margin-top: 6px; }
.ln { white-space: pre-wrap; word-break: break-all; color: #b9c3d3; }
.ln.err { color: #ff7b7b; }
.ln.warn { color: #ffd166; }
.ln.meta { color: #7fb3ff; }
.p { color: #6a7486; }
</style>
