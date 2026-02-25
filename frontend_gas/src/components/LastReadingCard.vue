<script setup lang="ts">
import { ref, onMounted } from 'vue'
import { fetchLatest } from '../services/gasApi'

const latestValue = ref<number | null>(null)
const latestTime = ref<string | null>(null)
const latestStatus = ref('Desconocido')
const lastUpdated = ref<Date | null>(null)
const loading = ref(false)
const error = ref<string | null>(null)

function computeStatus(value: number | null) {
  if (value == null) return 'Desconocido'
  if (value < 50) return 'Normal'
  if (value < 100) return 'Alerta'
  return 'Peligro'
}

async function loadLatest() {
  loading.value = true
  error.value = null
  try {
    const data = await fetchLatest()
    latestValue.value = data.value
    latestTime.value = data.time
    latestStatus.value = computeStatus(data.value)
    lastUpdated.value = new Date()
  } catch (e: any) {
    error.value = e.message ?? 'Error al obtener la última lectura'
  } finally {
    loading.value = false
  }
}

onMounted(() => {
  loadLatest()
  setInterval(loadLatest, 5000)
})
</script>

<template>
  <div class="card">
    <h2>Última lectura</h2>

    <p v-if="loading">Cargando...</p>
    <p v-else-if="error" class="error">{{ error }}</p>
    <p v-else class="value">
      {{ latestValue !== null ? latestValue.toFixed(2) + ' ppm' : 'Sin datos' }}
    </p>

    <p class="time" v-if="latestTime">
      Hora: {{ new Date(latestTime).toLocaleTimeString() }}
    </p>
    <p class="update" v-if="lastUpdated">
      Última actualización del dashboard: {{ lastUpdated.toLocaleTimeString() }}
    </p>

    <p class="status" :class="latestStatus.toLowerCase()">
      Estado: {{ latestStatus }}
    </p>
  </div>
</template>
