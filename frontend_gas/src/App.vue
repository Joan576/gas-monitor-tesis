<template>
  <div class="app">
    <nav class="sidebar">
      <h2>Gas Monitor</h2>
      <router-link to="/">Dashboard</router-link>
      <router-link to="/robots">Robots</router-link>
      <router-link to="/alerts">Alertas</router-link>
    </nav>
    <section class="content">
      <router-view />
    </section>
  </div>
</template>

<style scoped>
.app {
  display: flex;
  min-height: 100vh;
  font-family: system-ui;
  background: #111;
  color: #f5f5f5;
}

.sidebar {
  width: 220px;
  padding: 1.5rem;
  background: #181818;
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.sidebar h2 {
  margin-bottom: 1rem;
}

.sidebar a {
  color: #ccc;
  text-decoration: none;
  padding: 0.4rem 0;
}

.sidebar a.router-link-active {
  color: #42b883;
  font-weight: 600;
}

.content {
  flex: 1;
  padding: 2rem;
}
</style>
