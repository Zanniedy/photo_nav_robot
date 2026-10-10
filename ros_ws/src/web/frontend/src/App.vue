<template>
  <div class="app">
    <StatusBar :mode="mode" />
    <div class="main">
      <SidePanel :mode="mode" @update:mode="mode = $event" />
      <MapCanvas :mode="mode" />
    </div>
  </div>
</template>

<script setup>
import { ref } from 'vue'
import StatusBar from './components/StatusBar.vue'
import SidePanel from './components/SidePanel.vue'
import MapCanvas from './components/MapCanvas.vue'

const mode = ref('idle')
</script>

<style>
html, body, #app, .app { height: 100%; overflow: hidden; }
.app { display: flex; flex-direction: column; }
.main { display: flex; flex: 1; min-height: 0; }
</style>
