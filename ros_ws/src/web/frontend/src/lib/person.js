// 人员跟踪调试：订阅 person_tracker 的 /person/state（JSON），调它的服务和参数
import { reactive } from 'vue'
import { subscribe, callService } from './ros.js'

// state: 节点发的 JSON（见 person_tracker_node.py publish_vis）；t: 收到的时间，用来判断节点是否在线
export const person = reactive({ state: null, t: 0 })

subscribe('/person/state', 'std_msgs/String', (m) => {
  try { person.state = JSON.parse(m.data); person.t = performance.now() } catch { /* 坏数据忽略 */ }
}, { throttle_rate: 200, queue_length: 1 })

export const personOnline = () => !!person.state && performance.now() - person.t < 2000

export function approachStart() {
  return callService('/person/approach_start', 'std_srvs/Trigger')
}
export function approachCancel() {
  return callService('/person/approach_cancel', 'std_srvs/Trigger')
}

// rcl_interfaces/ParameterType: 1=bool 3=double
export async function setParam(name, value) {
  const v = typeof value === 'boolean' ? { type: 1, bool_value: value } : { type: 3, double_value: Number(value) }
  const r = await callService('/person_tracker/set_parameters', 'rcl_interfaces/srv/SetParameters',
    { parameters: [{ name, value: v }] })
  const res = r.results?.[0]
  if (res && !res.successful) throw new Error(res.reason || `设置 ${name} 失败`)
  return r
}
