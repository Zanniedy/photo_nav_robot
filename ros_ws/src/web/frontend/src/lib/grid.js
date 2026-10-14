// OccupancyGrid → 离屏 canvas。图像第 0 行对应栅格最上一行（y 最大），即上下翻转
export function gridToCanvas(msg, colorOf) {
  const { width: w, height: h } = msg.info
  const cv = document.createElement('canvas')
  cv.width = w
  cv.height = h
  const ctx = cv.getContext('2d')
  const img = ctx.createImageData(w, h)
  const px = img.data
  const data = msg.data
  for (let row = 0; row < h; row++) {
    const src = row * w
    const dst = (h - 1 - row) * w * 4
    for (let col = 0; col < w; col++) {
      const c = colorOf(data[src + col])
      const i = dst + col * 4
      px[i] = c[0]; px[i + 1] = c[1]; px[i + 2] = c[2]; px[i + 3] = c[3]
    }
  }
  ctx.putImageData(img, 0, 0)
  return cv
}

const UNKNOWN = [70, 76, 88, 255]
const FREE = [222, 224, 228, 255]
const OCC = [18, 18, 22, 255]
export function mapColor(v) {
  if (v < 0) return UNKNOWN
  if (v >= 65) return OCC
  if (v <= 25) return FREE
  const g = 222 - (v - 25) * 5
  return [g, g, g + 4, 255]
}

// nav2 costmap: 100 致命障碍, 99 内切圆, 1..98 膨胀代价
const NONE = [0, 0, 0, 0]
const LETHAL = [230, 40, 200, 200]
const INSCRIBED = [40, 220, 230, 170]
const costLut = []
for (let v = 1; v <= 98; v++) {
  const t = v / 98
  costLut[v] = [Math.round(60 + 195 * t), Math.round(90 * (1 - t)), Math.round(255 * (1 - t)), Math.round(60 + 90 * t)]
}
export function costColor(v) {
  if (v <= 0) return NONE
  if (v >= 100) return LETHAL
  if (v === 99) return INSCRIBED
  return costLut[v]
}
