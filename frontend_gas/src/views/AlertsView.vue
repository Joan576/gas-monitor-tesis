<script setup>
import { ref, onMounted, computed } from 'vue'

const latestValue = ref(null)
const latestTime = ref(null)
const loading = ref(false)
const error = ref(null)

function computeStatus(value) {
  if (value == null) return 'Desconocido'
  if (value < 50) return 'Normal'
  if (value < 100) return 'Alerta'
  return 'Peligro'
}

const currentStatus = computed(() => computeStatus(latestValue.value))

async function fetchLatest() {
  loading.value = true
  error.value = null
  try {
    const resp = await fetch('http://localhost:8000/api/latest')
    if (!resp.ok) throw new Error('Error HTTP ' + resp.status)
    const data = await resp.json()
    latestValue.value = data.value
    latestTime.value = data.time
  } catch (err) {
    error.value = err.message || 'Error al obtener la última lectura'
  } finally {
    loading.value = false
  }
}

function statusClass(estado) {
  if (!estado) return ''
  const e = estado.toLowerCase()
  if (e === 'normal') return 'status-pill normal'
  if (e === 'alerta') return 'status-pill alerta'
  if (e === 'peligro') return 'status-pill peligro'
  return 'status-pill'
}

onMounted(() => {
  fetchLatest()
  setInterval(fetchLatest, 5000)
})
</script>

<template>
  <div class="alerts">
    <h1 class="title">Alertas</h1>

    <div class="status-card">
      <h2>Estado actual del sistema</h2>

      <p v-if="loading">Cargando...</p>
      <p v-else-if="error" class="error">{{ error }}</p>
      <div v-else class="status-content">
        <div class="status-main">
          <p class="value">
            {{ latestValue !== null ? latestValue.toFixed(2) + ' ppm' : 'Sin datos' }}
          </p>
          <p class="time" v-if="latestTime">
            Última lectura: {{ new Date(latestTime).toLocaleTimeString() }}
          </p>
        </div>
        <div class="status-badge">
          <span class="status-label">Estado</span>
          <span :class="statusClass(currentStatus)">
            {{ currentStatus }}
          </span>
        </div>
      </div>
    </div>

    <div class="threshold-card">
      <h2>Umbrales de alerta</h2>

      <table class="threshold-table">
        <thead>
          <tr>
            <th>Nivel</th>
            <th>Rango (ppm)</th>
            <th>Descripción</th>
          </tr>
        </thead>
        <tbody>
          <tr>
            <td><span class="status-pill normal">Normal</span></td>
            <td>&lt; 50</td>
            <td>Operación segura del robot, sin presencia significativa de gas.</td>
          </tr>
          <tr>
            <td><span class="status-pill alerta">Alerta</span></td>
            <td>50 – 100</td>
            <td>Incremento moderado de gas, se recomienda supervisión y ventilación.</td>
          </tr>
          <tr>
            <td><span class="status-pill peligro">Peligro</span></td>
            <td>&gt; 100</td>
            <td>Nivel crítico, se requieren acciones inmediatas y posible detención del robot.</td>
          </tr>
        </tbody>
      </table>
    </div>
  </div>
</template>

<style scoped>
.alerts {
  padding: 24px;
  color: #e5e7eb;
}

.title {
  font-size: 1.8rem;
  margin-bottom: 16px;
}

/* Tarjeta ancha para estado actual */
.status-card {
  background-color: #111827;
  border-radius: 12px;
  padding: 16px;
  box-shadow: 0 10px 15px -3px rgba(15, 23, 42, 0.7);
  margin-bottom: 24px;
}

.status-content {
  margin-top: 8px;
  display: flex;
  justify-content: space-between;
  align-items: center;
  gap: 16px;
  flex-wrap: wrap;
}

.status-main .value {
  font-size: 2rem;
  font-weight: bold;
  margin-top: 4px;
}

.status-main .time {
  margin-top: 4px;
  font-size: 0.9rem;
  color: #9ca3af;
}

.status-badge {
  display: flex;
  flex-direction: column;
  align-items: flex-end;
  gap: 4px;
}

.status-label {
  font-size: 0.8rem;
  color: #9ca3af;
}

/* Tarjeta de umbrales */
.threshold-card {
  background-color: #111827;
  border-radius: 12px;
  padding: 16px;
  box-shadow: 0 10px 15px -3px rgba(15, 23, 42, 0.7);
}

.threshold-table {
  width: 100%;
  border-collapse: collapse;
  margin-top: 12px;
}

.threshold-table th,
.threshold-table td {
  padding: 10px 12px;
  font-size: 0.95rem;
}

.threshold-table thead {
  background-color: #020617;
}

.threshold-table tbody tr:nth-child(odd) {
  background-color: #020617;
}

.threshold-table tbody tr:nth-child(even) {
  background-color: #030712;
}

.threshold-table th {
  text-align: left;
  color: #9ca3af;
  font-weight: 600;
}

.status-pill {
  display: inline-block;
  padding: 4px 10px;
  border-radius: 999px;
  font-size: 0.85rem;
}

.status-pill.normal {
  background-color: rgba(22, 163, 74, 0.15);
  color: #22c55e;
}

.status-pill.alerta {
  background-color: rgba(245, 158, 11, 0.15);
  color: #facc15;
}

.status-pill.peligro {
  background-color: rgba(220, 38, 38, 0.15);
  color: #ef4444;
}

.error {
  color: #f97316;
  font-size: 0.9rem;
}
</style>

