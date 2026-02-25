<script setup lang="ts">
import { ref, computed } from 'vue'
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
import { fetchHistory } from '../services/gasApi'

ChartJS.register(
  LineElement,
  PointElement,
  LineController,
  CategoryScale,
  LinearScale,
  Tooltip,
  Legend
)

const historyLabels = ref<string[]>([])
const historyData = ref<number[]>([])
const loading = ref(false)
const error = ref<string | null>(null)
const selectedMinutes = ref(10)

function aggregatePoints(points: any[], minutes: number) {
  const MAX_POINTS = 400
  if (!points || points.length === 0) return []
  if (points.length <= MAX_POINTS) return points

  let blockSize
  if (minutes <= 10) blockSize = 2
  else if (minutes <= 30) blockSize = 4
  else blockSize = 6

  const aggregated: { time: string; value: number }[] = []
  for (let i = 0; i < points.length; i += blockSize) {
    const block = points.slice(i, i + blockSize)
    if (!block.length) continue
    const avgValue = block.reduce((s: number, p: any) => s + p.value, 0) / block.length
    const mid = block[Math.floor(block.length / 2)]
    aggregated.push({ time: mid.time, value: avgValue })
  }
  return aggregated
}

async function loadHistory(minutes = 10) {
  loading.value = true
  error.value = null
  selectedMinutes.value = minutes
  try {
    let points = await fetchHistory(minutes)
    if (!points || !points.length) {
      historyLabels.value = []
      historyData.value = []
      return
    }
    points = aggregatePoints(points, minutes)
    historyLabels.value = points.map(p => new Date(p.time).toLocaleTimeString())
    historyData.value = points.map(p => p.value)
  } catch (e: any) {
    error.value = e.message ?? 'Error al obtener histórico'
  } finally {
    loading.value = false
  }
}

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
      title: { display: true, text: 'Tiempo', color: '#9ca3af' },
      ticks: { color: '#9ca3af' },
      grid: { color: '#1f2933' }
    },
    y: {
      title: { display: true, text: 'Concentración (ppm)', color: '#9ca3af' },
      ticks: { color: '#9ca3af' },
      grid: { color: '#1f2933' }
    }
  },
  plugins: {
    legend: {
      labels: { color: '#e5e7eb' }
    }
  }
}

loadHistory(10)
</script>

<template>
  <div class="chart-card">
    <div class="chart-header">
      <h2>Histórico de concentración – {{ selectedMinutes }} min</h2>
      <div class="range-selector">
        <span class="range-label">Rango histórico</span>
        <div class="range-buttons">
          <button @click="loadHistory(10)">10 min</button>
          <button @click="loadHistory(30)">30 min</button>
          <button @click="loadHistory(60)">60 min</button>
        </div>
      </div>
    </div>

    <p class="data-flow">
      Datos generados por ROS2, almacenados en InfluxDB y servidos por FastAPI.
    </p>
    <p class="axis-hint">
      Eje X: tiempo (hh:mm:ss). Eje Y: concentración de gas (ppm).
    </p>

    <p v-if="!loading && historyData.length === 0" class="no-data">
      Sin datos suficientes en el rango seleccionado.
    </p>

    <div class="chart-wrapper">
      <Line :data="chartData" :options="chartOptions" />
    </div>
  </div>
</template>
