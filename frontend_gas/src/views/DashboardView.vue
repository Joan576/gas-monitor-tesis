<script setup lang="ts">
import LastReadingCard from '../components/LastReadingCard.vue'
import GasHistoryChart from '../components/GasHistoryChart.vue'
import HeatmapGas from '../components/HeatmapGas.vue' 
</script>

<template>
  <div class="dashboard">
    <h1 class="title">Monitor de Gas – Robot</h1>

    <div class="cards">
      <LastReadingCard />
    </div>

    <GasHistoryChart />


    <HeatmapGas />
  </div>
</template>


<style>

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
