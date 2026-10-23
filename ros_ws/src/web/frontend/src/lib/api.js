// 后端 REST，与网页同源（cookie 登录）
let onUnauthorized = () => {}
export function setUnauthorizedHandler(fn) { onUnauthorized = fn }

async function req(method, path, body) {
  const res = await fetch(path, {
    method,
    headers: body ? { 'Content-Type': 'application/json' } : {},
    body: body ? JSON.stringify(body) : undefined,
  })
  const text = await res.text()
  let data
  try { data = JSON.parse(text) } catch { data = text }
  if (res.status === 401 && !path.startsWith('/api/auth/')) onUnauthorized()
  if (!res.ok) throw new Error(typeof data === 'object' && (data.detail || data.msg) ? (data.detail || data.msg) : text)
  return data
}

const q = encodeURIComponent

export const api = {
  me: () => req('GET', '/api/auth/me'),
  login: (username, password) => req('POST', '/api/auth/login', { username, password }),
  logout: () => req('POST', '/api/auth/logout'),
  status: () => req('GET', '/api/status'),
  maps: () => req('GET', '/api/maps'),
  diag: () => req('GET', '/api/diag'),
  procStart: (n) => req('POST', `/api/proc/${n}/start`),
  procRestart: (n) => req('POST', `/api/proc/${n}/restart`),
  procStop: (n) => req('POST', `/api/proc/${n}/stop`),
  mappingStart: () => req('POST', '/api/mapping/start'),
  mappingSave: (name) => req('POST', '/api/mapping/save', { name }),
  navStart: (map) => req('POST', '/api/nav/start', { map }),
  switchMap: (map) => req('POST', '/api/nav/switch_map', { map }),
  setDefaultMap: (map) => req('POST', '/api/maps/default', { map }),
  saveMapPose: (map, p) => req('POST', '/api/maps/pose', { map, ...p }),
  clearMapPose: (map) => req('DELETE', `/api/maps/pose?map=${q(map)}`),
  initialPose: (p, refine) => req('POST', '/api/nav/initialpose', { ...p, refine }),
  waypoints: (poses) => req('POST', '/api/nav/waypoints', { poses }),
  navCancel: () => req('POST', '/api/nav/cancel'),
  stop: () => req('POST', '/api/stop'),
  stopAll: () => req('POST', '/api/stop_all'),
  annotations: (map) => req('GET', `/api/annotations?map=${q(map)}`),
  saveShapes: (map, shapes) => req('PUT', '/api/annotations/shapes', { map, shapes }),
  points: (map, query = '') => req('GET', `/api/points?map=${q(map)}&q=${q(query)}`),
  createPoint: (p) => req('POST', '/api/points', p),
  updatePoint: (map, id, patch) => req('PATCH', `/api/points/${q(id)}`, { map, ...patch }),
  deletePoint: (map, id) => req('DELETE', `/api/points/${q(id)}?map=${q(map)}`),
  reorderPoints: (map, ids) => req('POST', '/api/points_reorder', { map, ids }),
  gotoPoints: (names) => req('POST', '/api/points_goto', { names }),
  captures: () => req('GET', '/api/captures'),
  deleteCaptures: (names) => req('POST', '/api/captures/delete', { names }),
  quickList: () => req('GET', '/api/cmd'),
  quick: (id) => req('POST', `/api/cmd/quick/${id}`),
  cli: (cmd, timeout) => req('POST', '/api/cmd/cli', { cmd, timeout }),
  robot: () => req('GET', '/api/v1/robot'),
  setRobot: (body) => req('PUT', '/api/v1/robot', body),
  createToken: (name) => req('POST', '/api/v1/robot/tokens', { name }),
  deleteToken: (name) => req('DELETE', `/api/v1/robot/tokens/${q(name)}`),
}
