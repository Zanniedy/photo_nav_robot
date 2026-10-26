<template>
  <section>
    <h4>人员跟随 <span class="small" :class="online ? 'ok-t' : 'muted'">{{ online ? '在线' : procRunning ? '启动中…' : '节点未运行' }}</span>
      <button v-if="procRunning" class="sm" :disabled="!!store.busy" title="调试用：停止 person 进程（人体/人脸检测 + 跟踪）"
              @click="stopPerson">停止检测</button>
      <button v-else class="sm" :disabled="!!store.busy" title="调试用：启动 person 进程，不在 autostart 里" @click="startPerson">启动检测</button>
    </h4>
    <template v-if="online">
      <div class="line">
        <label class="chk" title="true：按检测框粗匹配出人的位置（红点）并记住，直到导航结束都按它走；false：导航中跟着人更新目标">
          <input type="checkbox" :checked="s.params.face_front" @change="setFace($event.target.checked)" /> 导航到人面前（记住位置）
        </label>
        <label class="chk" title="只锁定最近看到过人脸的人">
          <input type="checkbox" :checked="s.params.require_face" @change="setReqFace($event.target.checked)" /> 要看到人脸
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
        · 人脸 {{ s.det.faces }} 个 {{ s.det.face_ms.toFixed(0) }}ms
        <span v-if="!s.camera_info" class="warn-t"> · 没有 camera_info（相机未标定）</span>
        <span v-if="!s.face_model" class="warn-t"> · 没有人脸模型</span>
      </div>
      <div v-for="t in s.tracks" :key="t.id" class="row small mono">
        <span :class="{ 'ok-t': t.target }">#{{ t.id }}{{ t.target ? ' 🎯' : '' }}</span>
        <span>({{ t.x.toFixed(2) }}, {{ t.y.toFixed(2) }})</span>
        <span class="muted">{{ t.speed.toFixed(1) }}m/s · 相机 {{ t.cam_age.toFixed(1) }}s 前 · 人脸 {{ t.face_age == null ? '没看到' : `${t.face_age.toFixed(1)}s 前` }}</span>
      </div>
      <div v-if="!s.tracks.length" class="small muted">当前没有跟踪到人</div>

      <template v-if="ec">
        <h5 title="标相机相对车体的 yaw：画面里只站一个人，在 1~5m 处左右来回慢慢走，机器人别动">相机-雷达外参</h5>
        <div class="line">
          <button class="sm" :disabled="ec.on" @click="ecStart">开始采集</button>
          <button class="sm" :disabled="!ec.n" @click="ecSolve">{{ ec.on ? '停止并计算' : '重新计算' }}</button>
          <button class="sm" :disabled="!ec.result?.ok" @click="ecSave">保存</button>
        </div>
        <div v-if="ec.on || ec.n" class="small">
          {{ ec.on ? '采集中' : '已停止' }} · 样本 {{ ec.n }}
          <span class="muted">（跳过：人数≠1 {{ ec.skip.n_det }}，不同步 {{ ec.skip.sync }}，激光没对上 {{ ec.skip.no_match }}）</span>
        </div>
        <div v-if="ec.n" class="small mono cov" title="人相对光轴的方位分布，左边是画面左侧；每格 10°，越右越往画面右侧">
          <span class="muted">左</span>
          <span v-for="(c, i) in ec.coverage" :key="i" :class="c >= 3 ? 'ok-t' : 'warn-t'">{{ c }}</span>
          <span class="muted">右</span>
        </div>
        <template v-if="ec.result">
          <div v-if="ec.result.ok" class="small">
            yaw <b>{{ ec.result.yaw_deg.toFixed(2) }}°</b>
            <span class="muted">（当前 {{ ec.result.current_yaw_deg.toFixed(2) }}°，修正 {{ ec.result.delta_deg >= 0 ? '+' : '' }}{{ ec.result.delta_deg.toFixed(2) }}°）</span>
            · 残差 {{ ec.result.rms_deg.toFixed(2) }}° · {{ ec.result.inliers }}/{{ ec.result.n }}
            · 覆盖 {{ ec.result.spread_deg.toFixed(0) }}°
            <span v-if="ec.result.fx_pinhole"> · 针孔粗估 fx≈{{ ec.result.fx_pinhole.toFixed(0) }}</span>
          </div>
          <div v-if="ec.result.message" class="small warn-t">{{ ec.result.message }}</div>
        </template>
        <div v-if="ec.saved" class="small ok-t">已保存 {{ ec.saved }}，重启 base 后生效</div>
      </template>
    </template>
    <div v-else class="small muted">点「启动检测」（需要 camera 在运行；定位人和走到人面前还要导航）</div>
  </section>
</template>

<script setup>
import { computed, ref, onMounted, onUnmounted } from 'vue'
import { person, personOnline, approachStart, approachCancel, setParam, extcalStart, extcalSolve, extcalSave } from '../lib/person.js'
import { toast, store, run } from '../lib/store.js'
import { api } from '../lib/api.js'

const STATE = { idle: '空闲', searching: '找人', approaching: '前往', arrived: '到达', lost: '目标丢失', succeeded: '成功', failed: '失败', canceled: '已取消' }
const s = computed(() => person.state)
const procRunning = computed(() => !!store.status.processes?.person?.running)
const startPerson = () => run('启动 person', () => api.procStart('person'))
const stopPerson = () => run('停止 person', () => api.procStop('person'))
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
const setReqFace = (v) => call(() => setParam('require_face', v), '设置 require_face')
const ec = computed(() => s.value?.extcal)
// 计算的结果只在 /person/state 里刷新，这里只弹出服务返回的提示
const ecStart = () => call(extcalStart, '开始采集')
const ecSolve = () => call(extcalSolve, '计算')
const ecSave = () => call(extcalSave, '保存')
const setDist = (v) => { if (Number(v) > 0) call(() => setParam('approach_distance', Number(v)), '设置距离') }
</script>

<style scoped>
.dot { display: inline-block; width: 9px; height: 9px; border-radius: 50%; background: #ff2a2a; margin-right: 4px; vertical-align: middle; }
.row { gap: 8px; }
h5 { margin: 10px 0 4px; }
.cov { display: flex; gap: 6px; }
</style>
