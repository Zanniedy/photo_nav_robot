export function useMapRenderer() {
  let mapMeta = null

  function renderMap(canvas, mapMsg) {
    const { info, data } = mapMsg
    mapMeta = {
      resolution: info.resolution,
      origin_x: info.origin.position.x,
      origin_y: info.origin.position.y,
      width: info.width,
      height: info.height,
    }
    canvas.width = info.width
    canvas.height = info.height
    const ctx = canvas.getContext('2d')
    const img = ctx.createImageData(info.width, info.height)

    for (let row = 0; row < info.height; row++) {
      for (let col = 0; col < info.width; col++) {
        const val = data[row * info.width + col]
        // flip Y: ROS map origin is bottom-left, canvas is top-left
        const px = ((info.height - 1 - row) * info.width + col) * 4
        if (val === -1) {
          img.data[px] = 128; img.data[px + 1] = 128; img.data[px + 2] = 128
        } else if (val === 0) {
          img.data[px] = 240; img.data[px + 1] = 240; img.data[px + 2] = 240
        } else {
          img.data[px] = 20; img.data[px + 1] = 20; img.data[px + 2] = 20
        }
        img.data[px + 3] = 255
      }
    }
    ctx.putImageData(img, 0, 0)
  }

  function drawRobot(ctx, pose) {
    if (!mapMeta) return
    const { resolution, origin_x, origin_y, height } = mapMeta
    const mx = (pose.x - origin_x) / resolution
    const my = height - (pose.y - origin_y) / resolution
    ctx.save()
    ctx.translate(mx, my)
    ctx.rotate(-pose.yaw)
    ctx.beginPath()
    ctx.moveTo(12, 0); ctx.lineTo(-8, -8); ctx.lineTo(-8, 8); ctx.closePath()
    ctx.fillStyle = '#00aaff'
    ctx.fill()
    ctx.restore()
  }

  function drawScan(ctx, scanMsg) {
    if (!mapMeta) return
    const { angle_min, angle_increment, range_min, range_max, ranges } = scanMsg
    const { resolution, origin_x, origin_y, height } = mapMeta
    ctx.fillStyle = 'rgba(255, 80, 80, 0.8)'
    for (let i = 0; i < ranges.length; i++) {
      const r = ranges[i]
      if (r < range_min || r > range_max || !isFinite(r)) continue
      const angle = angle_min + i * angle_increment
      const wx = r * Math.cos(angle)
      const wy = r * Math.sin(angle)
      const mx = (wx - origin_x) / resolution
      const my = height - (wy - origin_y) / resolution
      ctx.fillRect(mx - 1, my - 1, 2, 2)
    }
  }

  // Draw goal arrow preview while user drags
  function drawGoalArrow(ctx, startPx, endPx) {
    const dx = endPx.x - startPx.x
    const dy = endPx.y - startPx.y
    const len = Math.sqrt(dx * dx + dy * dy)
    if (len < 4) return
    const angle = Math.atan2(dy, dx)
    ctx.save()
    ctx.translate(startPx.x, startPx.y)
    ctx.rotate(angle)
    ctx.strokeStyle = '#ffcc00'
    ctx.lineWidth = 2
    ctx.beginPath()
    ctx.moveTo(0, 0)
    ctx.lineTo(len, 0)
    // arrowhead
    ctx.lineTo(len - 10, -6)
    ctx.moveTo(len, 0)
    ctx.lineTo(len - 10, 6)
    ctx.stroke()
    // goal circle at origin
    ctx.beginPath()
    ctx.arc(0, 0, 5, 0, Math.PI * 2)
    ctx.strokeStyle = '#ffcc00'
    ctx.stroke()
    ctx.restore()
  }

  function canvasToWorld(canvas, clientX, clientY) {
    if (!mapMeta) return null
    const rect = canvas.getBoundingClientRect()
    const scaleX = canvas.width / rect.width
    const scaleY = canvas.height / rect.height
    const cx = (clientX - rect.left) * scaleX
    const cy = (clientY - rect.top) * scaleY
    const { resolution, origin_x, origin_y, height } = mapMeta
    return {
      x: cx * resolution + origin_x,
      y: (height - cy) * resolution + origin_y,
    }
  }

  // Convert client coords to raw canvas pixel (for arrow drawing)
  function clientToCanvas(canvas, clientX, clientY) {
    const rect = canvas.getBoundingClientRect()
    const scaleX = canvas.width / rect.width
    const scaleY = canvas.height / rect.height
    return {
      x: (clientX - rect.left) * scaleX,
      y: (clientY - rect.top) * scaleY,
    }
  }

  return { renderMap, drawRobot, drawScan, drawGoalArrow, canvasToWorld, clientToCanvas }
}
