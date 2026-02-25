<script setup lang="ts">
import { ref, onMounted, computed } from 'vue'
import {
  Chart as ChartJS,
  LinearScale,
  Tooltip,
  Legend
} from 'chart.js'
import { MatrixController, MatrixElement } from 'chartjs-chart-matrix'
import { Chart } from 'vue-chartjs'
import { fetchHeatmapGrid } from '../services/gasApi'

ChartJS.register(MatrixController, MatrixElement, LinearScale, Tooltip, Legend)

const cells = ref<any[]>([])
const loading = ref(false)
const error = ref<string | null>(null)
const selectedMinutes = ref(10)

// resolución de la grilla (x_bins = y_bins)
const resolution = ref(20)

// rango espacial (de momento fijo a 0–10 para X y Y)
const minX = ref(0)
const maxX = ref(10)
const minY = ref(0)
const maxY = ref(10)

// función simple para mapear concentración a color
function getColor(conc: number) {
  if (conc < 40) return 'rgba(34, 197, 94, 0.8)'     // verde
  if (conc < 80) return 'rgba(234, 179, 8, 0.9)'     // amarillo
  return 'rgba(239, 68, 68, 0.95)'                   // rojo
}

async function loadHeatmap(minutes = 10) {
  loading.value = true
  error.value = null
  selectedMinutes.value = minutes
  try {
    const data = await fetchHeatmapGrid(minutes, resolution.value, resolution.value)
    cells.value = data.cells
    // si quieres usar el rango real devuelto por el backend:
    minX.value = data.min_x
    maxX.value = data.max_x
    minY.value = data.min_y
    maxY.value = data.max_y
  } catch (e: any) {
    error.value = e.message ?? 'Error al obtener datos del mapa de calor'
  } finally {
    loading.value = false
  }
}

const chartData = computed(() => {
  if (!cells.value.length) {
    return { datasets: [] }
  }

  const xBins = resolution.value
  const yBins = resolution.value

  const width = (maxX.value - minX.value) / xBins
  const height = (maxY.value - minY.value) / yBins

  const matrixData = cells.value.map((c: any) => ({
    x: c.x,
    y: c.y,
    v: c.concentration,
    width,
    height
  }))

  return {
    datasets: [
      {
        label: 'Concentración espacial de gas (promedio por celda)',
        data: matrixData,
        backgroundColor(ctx: any) {
          const value = ctx.raw.v as number
          return getColor(value)
        },
        borderWidth: 0
      }
    ]
  }
})

const chartOptions = computed(() => ({
  responsive: true,
  maintainAspectRatio: false,
  scales: {
    x: {
      type: 'linear' as const,
      position: 'bottom' as const,
      title: {
        display: true,
        text: 'Posición X (m)',
        color: '#9ca3af'
      },
      min: minX.value,
      max: maxX.value,
      ticks: { color: '#9ca3af' },
      grid: { color: '#1f2933' }
    },
    y: {
      type: 'linear' as const,
      title: {
        display: true,
        text: 'Posición Y (m)',
        color: '#9ca3af'
      },
      min: minY.value,
      max: maxY.value,
      ticks: { color: '#9ca3af' },
      grid: { color: '#1f2933' }
    }
  },
  plugins: {
    legend: {
      labels: { color: '#e5e7eb' }
    },
    tooltip: {
      callbacks: {
        label(context: any) {
          const r = context.raw
          const conc = r.v?.toFixed(2) ?? 'N/A'
          return `x: ${r.x.toFixed(2)}, y: ${r.y.toFixed(2)}, conc: ${conc} ppm`
        }
      }
    }
  }
}))

onMounted(() => {
  loadHeatmap(10)
  setInterval(() => loadHeatmap(selectedMinutes.value), 5000)
})
</script>

<template>
  <div class="chart-card">
    <div class="chart-header">
      <h2>Mapa espacial de concentración – {{ selectedMinutes }} min</h2>
      <div class="range-selector">
        <span class="range-label">Rango de datos</span>
        <div class="range-buttons">
          <button @click="loadHeatmap(10)">10 min</button>
          <button @click="loadHeatmap(30)">30 min</button>
          <button @click="loadHeatmap(60)">60 min</button>
        </div>

        <div class="resolution-control">
          <span class="range-label">Resolución: {{ resolution }}×{{ resolution }}</span>
          <input
            type="range"
            min="10"
            max="40"
            step="2"
            v-model.number="resolution"
            @change="loadHeatmap(selectedMinutes)"
          />
        </div>
      </div>
    </div>

    <p class="data-flow">
      Cada celda representa la concentración promedio de gas en una región del entorno
      simulado. El color indica el nivel de concentración.
    </p>

    <p v-if="!loading && cells.length === 0" class="no-data">
      Sin datos suficientes para el mapa de calor en el rango seleccionado.
    </p>

    <div class="chart-wrapper">
      <Chart type="matrix" :data="chartData" :options="chartOptions" />
    </div>
  </div>
</template>

<style scoped>
.chart-wrapper {
  margin-top: 8px;
  height: 280px;
}

.resolution-control {
  margin-top: 8px;
  display: flex;
  flex-direction: column;
  align-items: flex-end;
  gap: 4px;
}

.resolution-control input[type="range"] {
  width: 160px;
}
</style>
