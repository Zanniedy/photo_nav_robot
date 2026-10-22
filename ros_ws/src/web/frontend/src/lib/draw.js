// 标注的渲染与命中测试。坐标约定：shape.pts / point 在 map 系；toS(x, y) 把 map 系坐标转成屏幕 px

const POINT_COLOR = '#2ee6a6'
const SEL_COLOR = '#5fa8ff'

function strokePath(ctx, sp, close) {
  ctx.beginPath()
  sp.forEach(([x, y], i) => (i ? ctx.lineTo(x, y) : ctx.moveTo(x, y)))
  if (close) ctx.closePath()
}

function withAlpha(hex, a) {
  let h = hex.replace('#', '')
  if (h.length === 3) h = [...h].map((c) => c + c).join('')
  const n = parseInt(h.slice(0, 6), 16)
  return `rgba(${n >> 16 & 255},${n >> 8 & 255},${n & 255},${a})`
}

/** 形状在屏幕上的点列（rect/circle 展开成多边形，便于统一画和测） */
export function screenOutline(s, toS) {
  const sp = s.pts.map(([x, y]) => toS(x, y))
  if (s.type === 'rect' && sp.length >= 2) {
    // 两个对角点在 map 系，矩形边与 map 轴平行
    const [[x0, y0], [x1, y1]] = s.pts
    return { pts: [toS(x0, y0), toS(x1, y0), toS(x1, y1), toS(x0, y1)], closed: true }
  }
  if (s.type === 'circle' && sp.length >= 2) {
    const [cx, cy] = sp[0]
    const r = Math.hypot(sp[1][0] - cx, sp[1][1] - cy)
    const n = 48
    return { pts: Array.from({ length: n }, (_, i) => [cx + r * Math.cos(2 * Math.PI * i / n), cy + r * Math.sin(2 * Math.PI * i / n)]), closed: true, circle: { cx, cy, r } }
  }
  return { pts: sp, closed: s.type === 'polygon' }
}

export function drawShape(ctx, s, toS, selected = false) {
  if (!s.pts.length) return
  ctx.save()
  ctx.lineJoin = ctx.lineCap = 'round'
  ctx.strokeStyle = s.color
  ctx.fillStyle = s.color
  ctx.lineWidth = s.width
  if (s.type === 'text') {
    const [x, y] = toS(...s.pts[0])
    ctx.font = `600 ${Math.round(10 + s.width * 2)}px system-ui, sans-serif`
    ctx.textBaseline = 'middle'
    ctx.lineWidth = 3
    ctx.strokeStyle = 'rgba(0,0,0,0.6)'
    ctx.strokeText(s.text || '文字', x, y)
    ctx.fillText(s.text || '文字', x, y)
    if (selected) {
      const w = ctx.measureText(s.text || '文字').width
      const h = 10 + s.width * 2
      ctx.strokeStyle = SEL_COLOR; ctx.lineWidth = 1; ctx.setLineDash([4, 3])
      ctx.strokeRect(x - 3, y - h / 2 - 3, w + 6, h + 6)
    }
    ctx.restore()
    return
  }
  const o = screenOutline(s, toS)
  strokePath(ctx, o.pts, o.closed)
  if (s.fill && o.closed) {
    ctx.fillStyle = withAlpha(s.color, 0.25)
    ctx.fill()
  }
  ctx.stroke()
  if (s.type === 'arrow' && o.pts.length >= 2) {
    const [x0, y0] = o.pts[o.pts.length - 2], [x1, y1] = o.pts[o.pts.length - 1]
    const a = Math.atan2(y1 - y0, x1 - x0), L = 8 + s.width * 2
    ctx.beginPath()
    ctx.moveTo(x1, y1)
    ctx.lineTo(x1 - L * Math.cos(a - 0.45), y1 - L * Math.sin(a - 0.45))
    ctx.lineTo(x1 - L * Math.cos(a + 0.45), y1 - L * Math.sin(a + 0.45))
    ctx.closePath()
    ctx.fillStyle = s.color
    ctx.fill()
  }
  if (selected) {
    ctx.strokeStyle = SEL_COLOR
    ctx.lineWidth = 1
    ctx.setLineDash([4, 3])
    strokePath(ctx, o.pts, o.closed)
    ctx.stroke()
    ctx.setLineDash([])
    ctx.fillStyle = SEL_COLOR
    const handles = s.type === 'freehand' ? [o.pts[0], o.pts[o.pts.length - 1]] : s.pts.map(([x, y]) => toS(x, y))
    handles.forEach(([x, y]) => ctx.fillRect(x - 3, y - 3, 6, 6))
  }
  ctx.restore()
}

export function drawPoint(ctx, p, toS, yawToS, selected = false, index = null) {
  const [x, y] = toS(p.x, p.y)
  const a = yawToS(p.yaw)
  ctx.save()
  // 朝向小三角
  ctx.translate(x, y)
  ctx.rotate(a)
  ctx.fillStyle = selected ? SEL_COLOR : POINT_COLOR
  ctx.beginPath(); ctx.moveTo(16, 0); ctx.lineTo(7, -5); ctx.lineTo(7, 5); ctx.closePath(); ctx.fill()
  ctx.restore()
  ctx.save()
  ctx.fillStyle = selected ? SEL_COLOR : POINT_COLOR
  ctx.strokeStyle = '#0b1a14'
  ctx.lineWidth = 2
  ctx.beginPath(); ctx.arc(x, y, 6, 0, Math.PI * 2); ctx.fill(); ctx.stroke()
  const label = index != null ? `${index + 1}. ${p.name}` : p.name
  ctx.font = '600 12px system-ui, sans-serif'
  ctx.textBaseline = 'bottom'
  ctx.lineWidth = 3
  ctx.strokeStyle = 'rgba(0,0,0,0.7)'
  ctx.strokeText(label, x + 8, y - 6)
  ctx.fillStyle = selected ? '#cfe4ff' : '#d8fff0'
  ctx.fillText(label, x + 8, y - 6)
  ctx.restore()
}

// ── 命中测试（屏幕 px） ───────────────────────────────────────────────

function segDist(px, py, [x0, y0], [x1, y1]) {
  const dx = x1 - x0, dy = y1 - y0
  const L2 = dx * dx + dy * dy
  const t = L2 ? Math.max(0, Math.min(1, ((px - x0) * dx + (py - y0) * dy) / L2)) : 0
  return Math.hypot(px - (x0 + t * dx), py - (y0 + t * dy))
}

function inside(px, py, poly) {
  let c = false
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
    const [xi, yi] = poly[i], [xj, yj] = poly[j]
    if ((yi > py) !== (yj > py) && px < (xj - xi) * (py - yi) / (yj - yi) + xi) c = !c
  }
  return c
}

export function hitShape(ctx, s, toS, px, py, tol = 6) {
  if (s.type === 'text') {
    const [x, y] = toS(...s.pts[0])
    const h = 10 + s.width * 2
    ctx.font = `600 ${Math.round(h)}px system-ui, sans-serif`
    const w = ctx.measureText(s.text || '文字').width
    return px >= x - 3 && px <= x + w + 3 && py >= y - h / 2 - 3 && py <= y + h / 2 + 3
  }
  const o = screenOutline(s, toS)
  const pts = o.pts
  if (pts.length === 1) return Math.hypot(px - pts[0][0], py - pts[0][1]) < tol
  const t = tol + s.width / 2
  for (let i = 1; i < pts.length; i++) if (segDist(px, py, pts[i - 1], pts[i]) < t) return true
  if (o.closed && segDist(px, py, pts[pts.length - 1], pts[0]) < t) return true
  return o.closed && s.fill && inside(px, py, pts)
}

export function hitPoint(p, toS, px, py, tol = 10) {
  const [x, y] = toS(p.x, p.y)
  return Math.hypot(px - x, py - y) < tol
}
