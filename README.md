# Monitor de Gas – Robot (Tesis)

Proyecto de tesis que simula un robot en ROS2 que mide concentración de gas, guarda los datos en InfluxDB y los visualiza en una aplicación web (FastAPI + Vue).

## Estructura

- `backend_gas/`: API FastAPI que lee de InfluxDB y expone `/api/latest`, `/api/history`, `/api/robots`.
- `frontend_gas/`: SPA en Vue 3 + Vite con vistas Dashboard, Robots y Alertas.
- `ros2_ws/`: Workspace de ROS2 con el paquete `robot_gas` (nodos `gas_sensor` y `gas_to_influx`).
- `docs/`: Documentación e informe de la tesis.

## Cómo ejecutar

### 1. Backend (FastAPI)

```bash
cd backend_gas
python -m venv venv
source venv/bin/activate
pip install -r requirements.txt
uvicorn main:app --reload --host 0.0.0.0 --port 8000


### 2. Frontend (Vue)

cd frontend_gas
npm install
npm run dev


### 3. ROS2 + InfluxDB

cd ros2_ws
colcon build
source install/setup.bash
ros2 run robot_gas gas_sensor
ros2 run robot_gas gas_to_influx


http://localhost:8086
