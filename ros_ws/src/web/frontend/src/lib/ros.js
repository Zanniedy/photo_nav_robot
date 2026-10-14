// rosbridge 连接、订阅（断线自动重订阅）、发布、话题频率统计
import { reactive } from 'vue'

const ROSLIB = window.ROSLIB

// ?ros=host:port 可覆盖，默认和网页同主机 9090
const params = new URLSearchParams(location.search)
const url = `ws://${params.get('ros') || `${location.hostname}:9090`}`

export const rosState = reactive({ connected: false, url, rates: {} })

let ros = null
const subs = new Map()      // key -> {name, type, opts, cbs:Set, topic}
const pubs = new Map()      // name -> ROSLIB.Topic
const counts = {}           // name -> 自上次统计以来的消息数

function attach(s) {
  s.topic = new ROSLIB.Topic({ ros, name: s.name, messageType: s.type, ...s.opts })
  s.topic.subscribe((msg) => {
    counts[s.name] = (counts[s.name] || 0) + 1
    s.cbs.forEach((cb) => cb(msg))
  })
}

function connect() {
  ros = new ROSLIB.Ros({ url })
  ros.on('connection', () => {
    rosState.connected = true
    pubs.clear()
    subs.forEach(attach)
  })
  ros.on('error', () => {})
  ros.on('close', () => {
    rosState.connected = false
    subs.forEach((s) => { s.topic = null })
    setTimeout(connect, 2000)
  })
}
connect()

// 每秒统计一次频率
let lastTick = performance.now()
setInterval(() => {
  const now = performance.now()
  const dt = (now - lastTick) / 1000
  lastTick = now
  for (const name of Object.keys(rosState.rates)) if (!(name in counts)) rosState.rates[name] = 0
  for (const [name, n] of Object.entries(counts)) {
    rosState.rates[name] = n / dt
    delete counts[name]
  }
}, 1000)

/** 订阅，返回取消函数。同名同类型的订阅共用一个 rosbridge topic */
export function subscribe(name, type, cb, opts = {}) {
  const key = `${name}|${type}`
  let s = subs.get(key)
  if (!s) {
    s = { name, type, opts, cbs: new Set(), topic: null }
    subs.set(key, s)
    if (rosState.connected) attach(s)
  }
  s.cbs.add(cb)
  return () => {
    s.cbs.delete(cb)
    if (s.cbs.size === 0) {
      s.topic?.unsubscribe()
      subs.delete(key)
    }
  }
}

export function publish(name, type, msg) {
  if (!rosState.connected) return false
  let t = pubs.get(name)
  if (!t) {
    t = new ROSLIB.Topic({ ros, name, messageType: type })
    pubs.set(name, t)
  }
  t.publish(new ROSLIB.Message(msg))
  return true
}

export function yawOf(q) {
  return Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
}

export function quatOf(yaw) {
  return { x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) }
}
