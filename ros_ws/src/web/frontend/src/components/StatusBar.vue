<template>
  <header class="status-bar">
    <span class="title">Robot Debug UI</span>
    <span class="mode-badge" :class="mode">{{ modeLabel }}</span>
    <span class="conn" :class="{ ok: connected }">
      {{ connected ? '● ROS 已连接' : '○ 未连接' }}
    </span>
  </header>
</template>

<script setup>
import { computed } from 'vue'
import { useRosbridge } from '../composables/useRosbridge.js'

const props = defineProps({ mode: String })
const { connected } = useRosbridge()

const modeLabel = computed(() => ({
  mapping: '建图模式', navigation: '导航模式', idle: '待机',
}[props.mode] || '待机'))
</script>

<style scoped>
.status-bar { display: flex; align-items: center; gap: 12px; padding: 8px 16px;
  background: #12122a; border-bottom: 1px solid #333; }
.title { font-weight: bold; font-size: 1rem; color: #adf; flex: 1; }
.mode-badge { padding: 2px 10px; border-radius: 12px; font-size: 0.8rem; background: #333; }
.mode-badge.mapping { background: #2a5a2a; color: #7f7; }
.mode-badge.navigation { background: #2a3a5a; color: #7af; }
.conn { font-size: 0.8rem; color: #666; }
.conn.ok { color: #4f4; }
</style>
