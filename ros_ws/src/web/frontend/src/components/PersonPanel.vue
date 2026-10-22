<template>
  <section>
    <h4>人员跟随 <span class="small" :class="online ? 'ok-t' : 'muted'">{{ online ? '在线' : '节点未运行' }}</span></h4>
    <template v-if="online">
      <div class="line">
        <label class="chk" title="true：按检测框粗匹配出人的位置（红点）并记住，直到导航结束都按它走；false：导航中跟着人更新目标">
          <input type="checkbox" :checked="s.params.face_front" @change="setFace($event.target.checked)" /> 导航到人面前（记住位置）
        </label>
      </div>
      <div class="line">
        <span class="small">停在离人</span>
        <input class="num" type="number" step="0.5" min="0.5" :value="s.params.approach_distance"
               aria-label="停在离人多远（米）" @change="setDist($event.target.value)" />
        <span class="small">m</span>
        <button class="sm" :disabled="s.busy" @click="start">开始</button>
        <button class="sm danger" :disabled="!s.busy" @click="cancel">取消</button>
      </div>
      <div class="small">
        状态 <b :class="stateClass">{{ STATE[s.approach.state] || s.approach.state }}</b>
        <span v-if="s.approach.target_id > 0"> · 目标 #{{ s.approach.target_id }}</span>
        <span v-if="s.approach.distance >= 0"> · 距离 {{ s.approach.distance.toFixed(2) }}m</span>
      </div>
      <div v-if="s.approach.message" class="small muted">{{ s.approach.message }}</div>
      <div v-if="s.mark" class="small">
        <span class="dot" :style="{ opacity: s.mark.active ? 1 : 0.45 }" />
        记住的位置 ({{ s.mark.xy[0].toFixed(2) }}, {{ s.mark.xy[1].toFixed(2) }})
        <span class="muted">{{ s.mark.face_front ? `${s.mark.frames} 帧平均` : `已更新 ${s.mark.frames} 次` }}{{ s.mark.active ? '' : '，上次的' }}</span>
      </div>
      <div class="small muted">
        检测 {{ s.det.hz.toFixed(1) }}Hz · {{ s.det.ms.toFixed(0) }}ms · 框 {{ s.det.n }} 个，测到距离 {{ s.det.with_range }} 个
        <span v-if="!s.camera_info" class="warn-t"> · 没有 camera_info（相机未标定）</span>
      </div>
      <div v-for="t in s.tracks" :key="t.id" class="row small mono">
        <span :class="{ 'ok-t': t.target }">#{{ t.id }}{{ t.target ? ' 🎯' : '' }}</span>
        <span>({{ t.x.toFixed(2) }}, {{ t.y.toFixed(2) }})</span>
        <span class="muted">{{ t.speed.toFixed(1) }}m/s · 相机 {{ t.cam_age.toFixed(1) }}s 前</span>
      </div>
      <div v-if="!s.tracks.length" class="small muted">当前没有跟踪到人</div>
    </template>
    <div v-else class="small muted">在状态页启动 person 进程（需要 camera 和导航在运行）</div>
  </section>
</template>

<script setup>
import { computed, ref, onMounted, onUnmounted } from 'vue'
import { person, personOnline, approachStart, approachCancel, setParam } from '../lib/person.js'
import { toast } from '../lib/store.js'

const STATE = { idle: '空闲', searching: '找人', approaching: '前往', arrived: '到达', lost: '目标丢失', succeeded: '成功', failed: '失败', canceled: '已取消' }
const s = computed(() => person.state)
const tick = ref(0)
let timer = 0
onMounted(() => { timer = setInterval(() => tick.value++, 1000) })
onUnmounted(() => clearInterval(timer))
const online = computed(() => (tick.value, person.t, personOnline()))
const stateClass = computed(() => ({ 'ok-t': ['succeeded', 'arrived'].includes(s.value.approach.state), 'warn-t': ['failed', 'lost'].includes(s.value.approach.state) }))

async function call(fn, label) {
  try {
    const r = await fn()
    if (r && 'success' in r) toast(r.message || label, r.success ? 'info' : 'error')
  } catch (e) { toast(`${label}: ${e.message}`, 'error') }
}
const start = () => call(approachStart, '开始')
const cancel = () => call(approachCancel, '取消')
const setFace = (v) => call(() => setParam('face_front', v), '设置 face_front')
const setDist = (v) => { if (Number(v) > 0) call(() => setParam('approach_distance', Number(v)), '设置距离') }
</script>

<style scoped>
.dot { display: inline-block; width: 9px; height: 9px; border-radius: 50%; background: #ff2a2a; margin-right: 4px; vertical-align: middle; }
.row { gap: 8px; }
</style>
