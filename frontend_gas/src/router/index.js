import { createRouter, createWebHistory } from 'vue-router'
import DashboardView from '../views/DashboardView.vue'
import RobotsView from '../views/RobotsView.vue'
import AlertsView from '../views/AlertsView.vue'

const routes = [
  { path: '/', name: 'dashboard', component: DashboardView },
  { path: '/robots', name: 'robots', component: RobotsView },
  { path: '/alerts', name: 'alerts', component: AlertsView },
]

const router = createRouter({
  history: createWebHistory(),
  routes,
})

export default router
