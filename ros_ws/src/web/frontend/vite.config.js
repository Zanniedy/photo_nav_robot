import { defineConfig } from 'vite'
import vue from '@vitejs/plugin-vue'

// 开发：ROBOT=192.168.10.42 npm run dev，浏览器打开 http://localhost:5173（/api 和 /ros 都代理到机器人后端）
const robot = process.env.ROBOT || 'localhost'

export default defineConfig({
  plugins: [vue()],
  build: {
    outDir: '../backend/static',
    emptyOutDir: true,
  },
  server: {
    proxy: {
      '/api': { target: `http://${robot}:8000`, changeOrigin: true },
      '/ros': { target: `ws://${robot}:8000`, ws: true, changeOrigin: true },
    },
  },
})
