<script setup>
import { ref, onMounted } from 'vue'

const robots = ref([])
const loading = ref(false)
const error = ref(null)

async function fetchRobots() {
  loading.value = true
  error.value = null
  try {
    const resp = await fetch('http://localhost:8000/api/robots')
    if (!resp.ok) throw new Error('Error HTTP ' + resp.status)
    robots.value = await resp.json()
  } catch (err) {
    error.value = err.message || 'Error al obtener robots'
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

onMounted(fetchRobots)
</script>

<template>
  <div class="robots">
    <h1 class="title">Robots</h1>

    <p v-if="loading">Cargando robots...</p>
    <p v-else-if="error" class="error">{{ error }}</p>

    <table v-else class="robots-table">
      <thead>
        <tr>
          <th>ID</th>
          <th>Nombre</th>
          <th>Última lectura (ppm)</th>
          <th>Estado</th>
          <th>Último reporte</th>
        </tr>
      </thead>
      <tbody>
        <tr v-for="robot in robots" :key="robot.id">
          <td>{{ robot.id }}</td>
          <td>{{ robot.nombre }}</td>
          <td>
            {{ robot.ultima_lectura !== null && robot.ultima_lectura !== undefined
              ? robot.ultima_lectura.toFixed(2)
              : 'Sin datos'
            }}
          </td>
          <td>
            <span :class="statusClass(robot.estado)">
              {{ robot.estado }}
            </span>
          </td>
          <td>
            {{ robot.ultimo_reporte ? new Date(robot.ultimo_reporte).toLocaleTimeString() : 'Sin datos' }}
          </td>
        </tr>
      </tbody>
    </table>
  </div>
</template>

<style scoped>
.robots {
  padding: 24px;
  color: #e5e7eb;
}

.title {
  font-size: 1.8rem;
  margin-bottom: 16px;
}

.robots-table {
  width: 100%;
  border-collapse: collapse;
  background-color: #111827;
  border-radius: 12px;
  overflow: hidden;
  box-shadow: 0 10px 15px -3px rgba(15, 23, 42, 0.7);
}

.robots-table th,
.robots-table td {
  padding: 12px 16px;
  font-size: 0.95rem;
}

.robots-table thead {
  background-color: #020617;
}

.robots-table tbody tr:nth-child(odd) {
  background-color: #020617;
}

.robots-table tbody tr:nth-child(even) {
  background-color: #030712;
}

.robots-table th {
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
