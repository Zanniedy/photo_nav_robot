import { ref, readonly } from 'vue'

const ros = ref(null)
const connected = ref(false)
const robotIp = ref('')

async function initRos() {
  const res = await fetch('/api/config')
  const data = await res.json()
  robotIp.value = data.robot_ip

  const ROSLIB = window.ROSLIB
  const r = new ROSLIB.Ros({ url: `ws://${data.robot_ip}:9090` })

  r.on('connection', () => { connected.value = true })
  r.on('error', () => { connected.value = false })
  r.on('close', () => {
    connected.value = false
    setTimeout(initRos, 3000)
  })

  ros.value = r
}

function subscribe(topicName, messageType, callback) {
  if (!ros.value) return () => {}
  const ROSLIB = window.ROSLIB
  const topic = new ROSLIB.Topic({ ros: ros.value, name: topicName, messageType })
  topic.subscribe(callback)
  return () => topic.unsubscribe()
}

function publish(topicName, messageType, msg) {
  if (!ros.value) return
  const ROSLIB = window.ROSLIB
  const topic = new ROSLIB.Topic({ ros: ros.value, name: topicName, messageType })
  topic.publish(new ROSLIB.Message(msg))
}

export function useRosbridge() {
  if (!ros.value) initRos()
  return { ros: readonly(ros), connected: readonly(connected), robotIp: readonly(robotIp), subscribe, publish }
}
