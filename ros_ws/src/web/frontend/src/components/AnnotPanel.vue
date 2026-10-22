<template>
  <div class="panel">
    <div v-if="!annot.map" class="muted small hint">标注和点位绑定到地图：先进入导航模式（顶栏选地图 → 导航），这里就会加载该地图的标注。</div>
    <template v-else>
      <section>
        <h4>点位 ({{ annot.points.length }})
          <button class="sm" :class="{ on: store.tool === 'point' }" @click="store.tool = 'point'" title="在地图上点击添加">📌 地图上添加</button>
        </h4>
        <div class="line">
          <input v-model="query" placeholder="搜索点名 / 备注" aria-label="搜索点位" />
          <label class="chk" title="勾选多个点按列表顺序依次导航"><input type="checkbox" v-model="multi" /> 多选</label>
        </div>
        <div v-if="!shown.length" class="muted small">{{ query ? '没有匹配的点' : '还没有点位' }}</div>
        <div v-for="p in shown" :key="p.id" class="pt" :class="{ sel: p.id === annot.selectedPoint }" @click="select(p)">
          <template v-if="editing === p.id">
            <div class="line">
              <input v-model="form.name" aria-label="点名" />
            </div>
            <div class="line">
              <input class="num" type="number" step="0.1" v-model.number="form.x" aria-label="x" />
              <input class="num" type="number" step="0.1" v-model.number="form.y" aria-label="y" />
              <input class="num" type="number" step="15" v-model.number="form.deg" aria-label="朝向（度）" />
              <button class="sm" @click.stop="fillRobot(form)" :disabled="!robotInMap" title="用机器人当前位姿">当前</button>
            </div>
            <div class="line"><input v-model="form.note" placeholder="备注" aria-label="备注" /></div>
            <div class="line">
              <button class="sm on" @click.stop="saveEdit(p)">保存</button>
              <button class="sm" @click.stop="editing = null">取消</button>
            </div>
          </template>
          <div v-else class="row">
            <input v-if="multi" type="checkbox" :value="p.id" v-model="picked" @click.stop :aria-label="`选择 ${p.name}`" />
            <span class="idx muted">{{ indexOf(p) + 1 }}</span>
            <span class="name" :title="p.note">{{ p.name }}
              <span class="muted small mono">({{ p.x.toFixed(2) }}, {{ p.y.toFixed(2) }}, {{ deg(p.yaw) }}°)</span>
              <span v-if="p.note" class="muted small"> · {{ p.note }}</span>
            </span>
            <button class="sm" :disabled="!navOn" @click.stop="gotoPoints([p.name])" title="导航到这个点">🎯</button>
            <button class="sm" @click.stop="startEdit(p)" title="编辑">✎</button>
            <button class="sm" :disabled="!!query || indexOf(p) === 0" @click.stop="movePoint(indexOf(p), -1)" title="上移">↑</button>
            <button class="sm" @click.stop="remove(p)" title="删除">✕</button>
          </div>
        </div>
        <div v-if="multi" class="line">
          <button :disabled="!picked.length || !navOn" @click="gotoPicked">依次导航 {{ picked.length }} 个点</button>
          <button class="sm" @click="picked = shown.map((p) => p.id)">全选</button>
          <button class="sm" @click="picked = []">清空</button>
        </div>
      </section>

      <section>
        <h4>新增点位</h4>
        <div class="line"><input v-model="add.name" placeholder="点名" aria-label="新点名" /></div>
        <div class="line">
          <input class="num" type="number" step="0.1" v-model.number="add.x" placeholder="x" aria-label="新点 x" />
          <input class="num" type="number" step="0.1" v-model.number="add.y" placeholder="y" aria-label="新点 y" />
          <input class="num" type="number" step="15" v-model.number="add.deg" placeholder="°" aria-label="新点朝向（度）" />
          <button class="sm" @click="fillRobot(add)" :disabled="!robotInMap">当前位姿</button>
        </div>
        <div class="line">
          <input v-model="add.note" placeholder="备注（可选）" aria-label="新点备注" />
          <button @click="create">添加</button>
        </div>
      </section>

      <section>
        <h4>绘图 ({{ annot.shapes.length }})
          <button class="sm" :class="{ on: store.tool === 'draw' }" @click="store.tool = 'draw'">✏️ 画</button>
          <button class="sm" @click="undo() || toast('没有可撤销的操作')" title="Ctrl+Z">撤销</button>
          <span class="small" :class="annot.dirty ? 'warn-t' : 'muted'">{{ annot.saving ? '保存中…' : annot.dirty ? '未保存' : '已保存' }}</span>
        </h4>
        <div v-if="!annot.shapes.length" class="muted small">用地图上方的「✏️ 绘图」画直线、箭头、折线、多边形、矩形、圆、手绘、文字</div>
        <div v-for="s in [...annot.shapes].reverse()" :key="s.id" class="row shp" :class="{ sel: s.id === annot.selected }"
             @click="annot.selected = s.id; store.tool = 'select'">
          <span class="swatch" :style="{ background: s.color }" />
          <span class="name">{{ TYPE_LABEL[s.type] }}<span v-if="s.text" class="muted"> · {{ s.text }}</span></span>
          <input type="color" :value="s.color" @click.stop @change="updateShape(s.id, { color: $event.target.value })" aria-label="颜色" />
          <button class="sm" @click.stop="updateShape(s.id, { hidden: !s.hidden })" :title="s.hidden ? '显示' : '隐藏'">{{ s.hidden ? '🙈' : '👁' }}</button>
          <button class="sm" @click.stop="removeShape(s.id)" title="删除">✕</button>
        </div>
        <div class="line">
          <button class="sm danger" :disabled="!annot.shapes.length" @click="clearAll">清空绘图</button>
          <button class="sm" @click="reloadAnnotations">重新加载</button>
        </div>
      </section>
    </template>
  </div>
</template>

<script setup>
import { ref, reactive, computed } from 'vue'
import { store, toast } from '../lib/store.js'
import {
  annot, TYPE_LABEL, addPoint, editPoint, deletePoint, movePoint, gotoPoints,
  updateShape, removeShape, clearShapes, undo, reloadAnnotations,
} from '../lib/annot.js'

const query = ref('')
const multi = ref(false)
const picked = ref([])
const editing = ref(null)
const form = reactive({ name: '', x: 0, y: 0, deg: 0, note: '' })
const add = reactive({ name: '', x: null, y: null, deg: 0, note: '' })

const deg = (r) => (r * 180 / Math.PI).toFixed(0)
const navOn = computed(() => store.status.mode === 'navigation')
const robotInMap = computed(() => store.fixedFrame === 'map' && !!store.robot)
const shown = computed(() => {
  const q = query.value.trim().toLowerCase()
  return q ? annot.points.filter((p) => p.name.toLowerCase().includes(q) || (p.note || '').toLowerCase().includes(q)) : annot.points
})
const indexOf = (p) => annot.points.indexOf(p)

function select(p) { annot.selectedPoint = p.id; annot.selected = null }

function fillRobot(f) {
  const r = store.robot
  f.x = +r.x.toFixed(3); f.y = +r.y.toFixed(3); f.deg = +(r.yaw * 180 / Math.PI).toFixed(1)
}

function startEdit(p) {
  Object.assign(form, { name: p.name, x: p.x, y: p.y, deg: +(p.yaw * 180 / Math.PI).toFixed(1), note: p.note || '' })
  editing.value = p.id
}

async function saveEdit(p) {
  const r = await editPoint(p.id, { name: form.name.trim(), x: form.x, y: form.y, yaw: (form.deg || 0) * Math.PI / 180, note: form.note })
  if (r) editing.value = null
}

async function create() {
  const name = add.name.trim()
  if (!name) { toast('先填点名', 'error'); return }
  if (typeof add.x !== 'number' || typeof add.y !== 'number') { toast('先填 x / y，或点「当前位姿」', 'error'); return }
  const r = await addPoint({ name, x: add.x, y: add.y, yaw: (add.deg || 0) * Math.PI / 180, note: add.note })
  if (r) { Object.assign(add, { name: '', note: '' }); toast(`已添加 ${r.name}`) }
}

function remove(p) { if (confirm(`删除点位「${p.name}」？`)) deletePoint(p.id) }

function gotoPicked() {
  // 按列表顺序，而不是勾选顺序
  const names = annot.points.filter((p) => picked.value.includes(p.id)).map((p) => p.name)
  gotoPoints(names)
}

function clearAll() { if (confirm('清空这张地图上的所有绘图？（可撤销）')) clearShapes() }
</script>

<style scoped>
.hint { padding: 12px 0; }
.pt { border-radius: 4px; padding: 2px 4px; margin: 0 -4px; cursor: pointer; }
.pt.sel, .shp.sel { background: #1d2c44; }
.pt .name { white-space: normal; }
.idx { width: 18px; text-align: right; font-variant-numeric: tabular-nums; font-size: 11px; }
.shp { cursor: pointer; border-radius: 4px; padding: 2px 4px; margin: 0 -4px; }
.shp input[type=color] { width: 26px; height: 20px; padding: 0; border: 0; background: transparent; }
.swatch { width: 10px; height: 10px; border-radius: 2px; flex-shrink: 0; }
</style>
