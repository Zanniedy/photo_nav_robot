import { defineConfig } from 'vite'
import vue from '@vitejs/plugin-vue'

// 开发：ROBOT=192.168.10.42 npm run dev，浏览器打开 http://localhost:5173/?ros=192.168.10.42:9090
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
    },
  },
})
