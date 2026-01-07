<script setup>
import { ref, onMounted, computed } from 'vue'
import {
  Chart as ChartJS,
  LineElement,
  PointElement,
  LineController,
  CategoryScale,
  LinearScale,
  Tooltip,
  Legend
} from 'chart.js'
import { Line } from 'vue-chartjs'


const lastUpdated = ref(null)

// Registrar componentes necesarios de Chart.js
ChartJS.register(
  LineElement,
  PointElement,
  LineController,
  CategoryScale,
  LinearScale,
  Tooltip,
  Legend
)

// Estado para último valor
const latestValue = ref(null)
const latestTime = ref(null)
const latestStatus = ref('Desconocido')
const loadingLatest = ref(false)
const errorLatest = ref(null)

// Estado para histórico
const historyLabels = ref([])
const historyData = ref([])
const loadingHistory = ref(false)
const errorHistory = ref(null)
const selectedMinutes = ref(10)

// chartData derivado de labels y data
const chartData = computed(() => ({
  labels: historyLabels.value,
  datasets: [
    {
      label: 'Concentración de gas (ppm)',
      data: historyData.value,
      borderColor: '#38bdf8',
      backgroundColor: 'rgba(56, 189, 248, 0.15)',
      tension: 0.25,
      pointRadius: 0,
      fill: true
    }
  ]
}))


const chartOptions = {
  responsive: true,
  maintainAspectRatio: false,
  scales: {
    x: {
      title: {
        display: true,
        text: 'Tiempo',
        color: '#9ca3af'
      },
      ticks: {
        color: '#9ca3af'
      },
      grid: {
        color: '#1f2933'
      }
    },
    y: {
      title: {
        display: true,
        text: 'Concentración (ppm)',
        color: '#9ca3af'
      },
      ticks: {
        color: '#9ca3af'
      },
      grid: {
        color: '#1f2933'
      }
    }
  },
  plugins: {
    legend: {
      labels: {
        color: '#e5e7eb'
      }
    }
  }
}

function computeStatus(value) {
  if (value == null) return 'Desconocido'
  if (value < 50) return 'Normal'
  if (value < 100) return 'Alerta'
  return 'Peligro'
}

async function fetchLatest() {
  loadingLatest.value = true
  errorLatest.value = null
  try {
    const resp = await fetch('http://localhost:8000/api/latest')
    if (!resp.ok) throw new Error('Error HTTP ' + resp.status)
    const data = await resp.json()
    latestValue.value = data.value
    latestTime.value = data.time
    latestStatus.value = computeStatus(data.value)
    lastUpdated.value = new Date()   // hora en que el dashboard se actualizó
  } catch (err) {
    errorLatest.value = err.message || 'Error al obtener la última lectura'
  } finally {
    loadingLatest.value = false
  }
}


function aggregatePoints(points, minutes) {
  const MAX_POINTS = 400

  if (!points || points.length === 0) {
    return []
  }

  // Si hay pocos puntos, no se agregan
  if (points.length <= MAX_POINTS) {
    return points
  }

  // Escogemos tamaño de bloque según rango
  let blockSize
  if (minutes <= 10) {
    blockSize = 2      // agrupa cada 2 puntos
  } else if (minutes <= 30) {
    blockSize = 4       // agrupa cada 3 puntos
  } else {
    blockSize = 6      // agrupa cada 3 puntos
  }

  const aggregated = []
  for (let i = 0; i < points.length; i += blockSize) {
    const block = points.slice(i, i + blockSize)
    if (block.length === 0) continue

    const avgValue =
      block.reduce((sum, p) => sum + p.value, 0) / block.length
    const mid = block[Math.floor(block.length / 2)]

    aggregated.push({
      time: mid.time,
      value: avgValue
    })
  }

  return aggregated
}

async function fetchHistory(minutes = 10) {
  loadingHistory.value = true
  errorHistory.value = null
  selectedMinutes.value = minutes
  try {
    const resp = await fetch(`http://localhost:8000/api/history?minutes=${minutes}`)
    if (!resp.ok) throw new Error('Error HTTP ' + resp.status)
    let points = await resp.json()

    // Si no hay puntos, limpiamos y salimos
    if (!points || points.length === 0) {
      historyLabels.value = []
      historyData.value = []
      return
    }

    // Agregar / simplificar según rango
    points = aggregatePoints(points, minutes)

    historyLabels.value = points.map(p => new Date(p.time).toLocaleTimeString())
    historyData.value = points.map(p => p.value)
  } catch (err) {
    errorHistory.value = err.message || 'Error al obtener histórico'
  } finally {
    loadingHistory.value = false
  }
}



onMounted(() => {
  fetchLatest()
  fetchHistory(10)
  setInterval(fetchLatest, 5000)
})
</script>


<template>
  <div class="dashboard">
    <h1 class="title">Monitor de Gas – Robot</h1>

    <div class="cards">
      <div class="card">
        <h2>Última lectura</h2>

        <p v-if="loadingLatest">Cargando...</p>
        <p v-else-if="errorLatest" class="error">{{ errorLatest }}</p>
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
    </div>

    <div class="chart-card">
      <div class="chart-header">
        <h2>Histórico de concentración – {{ selectedMinutes }} min</h2>

        <div class="range-selector">
          <span class="range-label">Rango histórico</span>
          <div class="range-buttons">
            <button @click="fetchHistory(10)">10 min</button>
            <button @click="fetchHistory(30)">30 min</button>
            <button @click="fetchHistory(60)">60 min</button>
          </div>
        </div>
      </div>

      <p class="data-flow">
        Datos generados por ROS2 (simulación del robot), almacenados en InfluxDB y servidos por FastAPI al frontend en Vue.
      </p>

      <p class="axis-hint">
        Eje X: tiempo (hh:mm:ss). Eje Y: concentración de gas (ppm).
      </p>

      <p v-if="!loadingHistory && historyData.length === 0" class="no-data">
        Sin datos suficientes en el rango seleccionado. Asegúrate de que el robot está publicando lecturas.
      </p>

      <div class="chart-wrapper">
        <Line :data="chartData" :options="chartOptions" />
      </div>
    </div>
  </div>
</template>


<style scoped>

.no-data {
  margin-top: 8px;
  font-size: 0.9rem;
  color: #9ca3af;
}
  
.dashboard {
  padding: 24px;
  color: #e5e7eb;
}

.title {
  font-size: 1.8rem;
  margin-bottom: 16px;
}

.cards {
  display: flex;
  gap: 16px;
  margin-bottom: 24px;
  flex-wrap: wrap;
}

.card {
  background-color: #111827;
  border-radius: 12px;
  padding: 16px;
  flex: 1;
  min-width: 260px;
  box-shadow: 0 10px 15px -3px rgba(15, 23, 42, 0.7);
}

.value {
  font-size: 2rem;
  font-weight: bold;
  margin-top: 8px;
}

.time {
  margin-top: 4px;
  font-size: 0.9rem;
  color: #9ca3af;
}

.update {
  margin-top: 2px;
  font-size: 0.8rem;
  color: #6b7280;
}

.data-flow {
  margin-top: 4px;
  font-size: 0.9rem;
  color: #9ca3af;
  padding-bottom: 5px;
}



.status {
  margin-top: 8px;
  font-weight: 600;
}

.status.normal {
  color: #22c55e;
}

.status.alerta {
  color: #facc15;
}

.status.peligro {
  color: #ef4444;
}

.range-buttons {
  display: flex;
  gap: 8px;
  margin-top: 8px;
}

.range-buttons button {
  background-color: #1f2937;
  color: #e5e7eb;
  border: 1px solid #374151;
  border-radius: 6px;
  padding: 6px 10px;
  cursor: pointer;
  font-size: 0.9rem;
}

.range-buttons button:hover {
  background-color: #111827;
}

.chart-card {
  background-color: #111827;
  border-radius: 12px;
  padding: 16px;
  box-shadow: 0 10px 15px -3px rgba(15, 23, 42, 0.7);
}

.chart-wrapper {
  margin-top: 8px;
  height: 260px;
}

.axis-hint {
  margin-top: 4px;
  font-size: 0.85rem;
  color: #9ca3af;
}


.error {
  color: #f97316;
  font-size: 0.9rem;
}

.chart-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  gap: 12px;
}

.range-selector {
  display: flex;
  flex-direction: column;
  align-items: flex-end;
  gap: 4px;
}

.range-label {
  font-size: 0.8rem;
  color: #9ca3af;
}

.range-buttons {
  display: flex;
  gap: 8px;
}


</style>
