<template>
  <div class="login">
    <form class="card" @submit.prevent="submit">
      <h1>MAX_NAV_ROBOT</h1>
      <label>用户名<input v-model="username" autocomplete="username" required /></label>
      <label>密码<input v-model="password" type="password" autocomplete="current-password" required autofocus /></label>
      <button class="on" type="submit" :disabled="busy">{{ busy ? '登录中…' : '登录' }}</button>
      <div v-if="error" class="err" role="alert">{{ error }}</div>
    </form>
  </div>
</template>

<script setup>
import { ref } from 'vue'
import { api } from '../lib/api.js'
import { checkAuth } from '../lib/store.js'

const username = ref('max_robot')
const password = ref('')
const busy = ref(false)
const error = ref('')

async function submit() {
  busy.value = true
  error.value = ''
  try {
    await api.login(username.value, password.value)
    await checkAuth()
  } catch (e) {
    error.value = e.message
  } finally {
    busy.value = false
  }
}
</script>

<style scoped>
.login { height: 100%; display: flex; align-items: center; justify-content: center; padding: 16px; }
.card { width: 100%; max-width: 320px; display: flex; flex-direction: column; gap: 12px; background: var(--panel); border: 1px solid var(--line); border-radius: 8px; padding: 24px; }
h1 { font-size: 18px; color: var(--accent); text-align: center; margin-bottom: 4px; }
label { display: flex; flex-direction: column; gap: 4px; font-size: 12px; color: var(--muted); }
input { padding: 8px 10px; color: var(--text); }
button { padding: 8px; font-weight: 600; }
.err { color: var(--bad); font-size: 12px; }
</style>
