<template>
  <div class="map-wrap" ref="wrapEl"
       @wheel.prevent="onWheel"
       @mousedown="onMouseDown"
       @mousemove="onMouseMove"
       @mouseup="onMouseUp"
       @contextmenu.prevent>
    <canvas ref="canvasEl" class="map-canvas" :style="canvasStyle" />
    <canvas ref="overlayEl" class="map-canvas overlay" :style="canvasStyle" />
    <div v-if="!connected" class="no-connection">等待 ROS 连接...</div>
    <div v-if="goalDragging && mode === 'navigation'" class="hint">松开鼠标确认目标朝向</div>
  </div>
</template>

<script setup>
import { ref, computed, onMounted, onUnmounted } from 'vue'
import { useRosbridge } from '../composables/useRosbridge.js'
import { useMapRenderer } from '../composables/useMapRenderer.js'

const props = defineProps({ mode: String })

const { subscribe, publish, connected } = useRosbridge()
const { renderMap, drawRobot, drawScan, drawGoalArrow, canvasToWorld, clientToCanvas } = useMapRenderer()

const canvasEl = ref(null)
const overlayEl = ref(null)
const wrapEl = ref(null)
const scale = ref(1)
const panX = ref(0)
const panY = ref(0)

// pan state
let isPanning = false
let panStart = { x: 0, y: 0 }

// goal drag state
let goalDragging = ref(false)
let goalStartClient = null  // client coords when mousedown
let goalStartPx = null      // canvas px coords

// latest data for overlay redraws
let lastPose = null
let lastScan = null

const canvasStyle = computed(() => ({
  transform: `translate(${panX.value}px, ${panY.value}px) scale(${scale.value})`,
  transformOrigin: '0 0',
}))

let unsubs = []

function quaternionToYaw(q) {
  return Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
}

onMounted(() => {
  unsubs.push(subscribe('/map', 'nav_msgs/OccupancyGrid', (msg) => {
    renderMap(canvasEl.value, msg)
    redrawOverlay()
  }))

  const poseTopic = props.mode === 'mapping' ? '/slam_toolbox/pose' : '/amcl_pose'
  unsubs.push(subscribe(poseTopic, 'geometry_msgs/PoseWithCovarianceStamped', (msg) => {
    const p = msg.pose.pose
    lastPose = {
      x: p.position.x,
      y: p.position.y,
      yaw: quaternionToYaw(p.orientation),
    }
    redrawOverlay()
  }))

  unsubs.push(subscribe('/scan', 'sensor_msgs/LaserScan', (msg) => {
    lastScan = msg
    redrawOverlay()
  }))
})

onUnmounted(() => unsubs.forEach(u => u()))

function redrawOverlay(goalCursorPx = null) {
  const oc = overlayEl.value
  const mc = canvasEl.value
  if (!oc || !mc) return
  oc.width = mc.width
  oc.height = mc.height
  const ctx = oc.getContext('2d')
  ctx.clearRect(0, 0, oc.width, oc.height)
  if (lastScan) drawScan(ctx, lastScan)
  if (lastPose) drawRobot(ctx, lastPose)
  if (goalCursorPx && goalStartPx) drawGoalArrow(ctx, goalStartPx, goalCursorPx)
}

function onWheel(e) {
  const factor = e.deltaY < 0 ? 1.1 : 0.9
  scale.value = Math.max(0.2, Math.min(10, scale.value * factor))
}

function onMouseDown(e) {
  // middle or right click → pan
  if (e.button === 1 || e.button === 2) {
    isPanning = true
    panStart = { x: e.clientX - panX.value, y: e.clientY - panY.value }
    return
  }
  // left click in navigation mode → start goal drag
  if (e.button === 0 && props.mode === 'navigation') {
    goalDragging.value = true
    goalStartClient = { x: e.clientX, y: e.clientY }
    goalStartPx = clientToCanvas(canvasEl.value, e.clientX, e.clientY)
  }
}

function onMouseMove(e) {
  if (isPanning) {
    panX.value = e.clientX - panStart.x
    panY.value = e.clientY - panStart.y
    return
  }
  if (goalDragging.value) {
    const curPx = clientToCanvas(canvasEl.value, e.clientX, e.clientY)
    redrawOverlay(curPx)
  }
}

function onMouseUp(e) {
  if (isPanning) { isPanning = false; return }

  if (goalDragging.value && e.button === 0) {
    goalDragging.value = false
    const world = canvasToWorld(canvasEl.value, goalStartClient.x, goalStartClient.y)
    if (!world) return

    const dx = e.clientX - goalStartClient.x
    const dy = e.clientY - goalStartClient.y
    const dist = Math.sqrt(dx * dx + dy * dy)

    // very short drag → no orientation, default forward
    let yaw = 0
    if (dist > 8) {
      // canvas Y is flipped vs world Y
      yaw = Math.atan2(-dy, dx)
    }

    publish('/goal_pose', 'geometry_msgs/PoseStamped', {
      header: { frame_id: 'map' },
      pose: {
        position: { x: world.x, y: world.y, z: 0 },
        orientation: {
          x: 0, y: 0,
          z: Math.sin(yaw / 2),
          w: Math.cos(yaw / 2),
        },
      },
    })
    redrawOverlay()
  }
}
</script>

<style scoped>
.map-wrap { position: relative; flex: 1; overflow: hidden; background: #111; cursor: crosshair; }
.map-canvas { position: absolute; top: 0; left: 0; image-rendering: pixelated; }
.overlay { pointer-events: none; }
.no-connection { position: absolute; top: 50%; left: 50%; transform: translate(-50%, -50%); color: #888; }
.hint { position: absolute; bottom: 12px; left: 50%; transform: translateX(-50%);
  background: rgba(0,0,0,0.6); color: #ffcc00; padding: 4px 12px; border-radius: 8px;
  font-size: 0.75rem; pointer-events: none; }
</style>
