import axios from 'axios'

const BASE_URL = 'http://localhost:8000'

export async function fetchLatest() {
  const resp = await axios.get(`${BASE_URL}/api/latest`)
  return resp.data
}

export async function fetchHistory(minutes: number) {
  const resp = await axios.get(`${BASE_URL}/api/history`, {
    params: { minutes }
  })
  return resp.data as { time: string; value: number; robot_id: string }[]
}

export async function fetchHeatmapGrid(minutes = 10, xBins = 20, yBins = 20) {
  const res = await axios.get(`${BASE_URL}/api/heatmap_grid`, {
    params: { minutes, x_bins: xBins, y_bins: yBins }
  })
  return res.data as {
    cells: { x: number; y: number; concentration: number; ix: number; iy: number }[]
    x_bins: number
    y_bins: number
    min_x: number
    max_x: number
    min_y: number
    max_y: number
  }
}

