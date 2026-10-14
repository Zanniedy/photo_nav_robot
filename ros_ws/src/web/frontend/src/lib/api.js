// 后端 REST，与网页同源
async function req(method, path, body) {
  const res = await fetch(path, {
    method,
    headers: body ? { 'Content-Type': 'application/json' } : {},
    body: body ? JSON.stringify(body) : undefined,
  })
  const text = await res.text()
  let data
  try { data = JSON.parse(text) } catch { data = text }
  if (!res.ok) throw new Error(typeof data === 'object' && data.detail ? data.detail : text)
  return data
}

export const api = {
  status: () => req('GET', '/api/status'),
  maps: () => req('GET', '/api/maps'),
  diag: () => req('GET', '/api/diag'),
  procStart: (n) => req('POST', `/api/proc/${n}/start`),
  procStop: (n) => req('POST', `/api/proc/${n}/stop`),
  mappingStart: () => req('POST', '/api/mapping/start'),
  mappingSave: (name) => req('POST', '/api/mapping/save', { name }),
  navStart: (name) => req('POST', '/api/nav/start', { name }),
  switchMap: (name) => req('POST', '/api/nav/switch_map', { name }),
  waypoints: (poses) => req('POST', '/api/nav/waypoints', { poses }),
  navCancel: () => req('POST', '/api/nav/cancel'),
  stop: () => req('POST', '/api/stop'),
  stopAll: () => req('POST', '/api/stop_all'),
  quickList: () => req('GET', '/api/cmd'),
  quick: (id) => req('POST', `/api/cmd/quick/${id}`),
  cli: (cmd, timeout) => req('POST', '/api/cmd/cli', { cmd, timeout }),
}
