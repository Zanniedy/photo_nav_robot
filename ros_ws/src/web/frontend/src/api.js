let _baseUrl = ''

async function initBaseUrl() {
  if (_baseUrl) return
  const res = await fetch('/api/config')
  const { robot_ip } = await res.json()
  _baseUrl = `http://${robot_ip}:8000`
}

async function apiFetch(path, opts = {}) {
  await initBaseUrl()
  const res = await fetch(`${_baseUrl}${path}`, {
    headers: { 'Content-Type': 'application/json' },
    ...opts,
  })
  if (!res.ok) throw new Error(await res.text())
  return res.json()
}

export const getMaps = () => apiFetch('/api/maps')
export const switchMap = (name) => apiFetch('/api/maps/switch', { method: 'POST', body: JSON.stringify({ name }) })
export const slamStart = () => apiFetch('/api/slam/start', { method: 'POST' })
export const slamStop = () => apiFetch('/api/slam/stop', { method: 'POST' })
export const slamSave = (name) => apiFetch('/api/slam/save', { method: 'POST', body: JSON.stringify({ name }) })
export const navStart = (map_name) => apiFetch('/api/nav/start', { method: 'POST', body: JSON.stringify({ map_name }) })
export const navStop = () => apiFetch('/api/nav/stop', { method: 'POST' })
export const getStatus = () => apiFetch('/api/status')
export const lidarStart = () => apiFetch('/api/lidar/start', { method: 'POST' })
export const lidarStop = () => apiFetch('/api/lidar/stop', { method: 'POST' })
export const runDiag = () => apiFetch('/api/diag/check')

export async function getLogsEventSource() {
  await initBaseUrl()
  return new EventSource(`${_baseUrl}/api/process/logs`)
}
