from fastapi import FastAPI
from influxdb_client import InfluxDBClient
from typing import List
from fastapi import Query
from fastapi.middleware.cors import CORSMiddleware
from fastapi import HTTPException, Query
# ==== CONFIGURACIÓN INFLUXDB (AJUSTA ESTOS VALORES) ====
INFLUX_URL = "http://localhost:8086"
INFLUX_TOKEN = "NIuEJwL_3sdzCMFRnlJW4AH4ZhsgHgxVhCFXcWl1eLnUeSkqnCk-FxuKf5IkxnJEH_qeD9l0eF_u_Q1UI06XYQ=="
INFLUX_ORG = "sebas"
INFLUX_BUCKET = "gas_data"

client = InfluxDBClient(
    url=INFLUX_URL,
    token=INFLUX_TOKEN,
    org=INFLUX_ORG,
)

query_api = client.query_api()

app = FastAPI(title="Gas Monitor API")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:5173"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)



@app.get("/api/latest")
def get_latest_reading():
    """
    Devuelve la última lectura de gas del robot sim_robot_1.
    """
    query = f'''
    from(bucket: "{INFLUX_BUCKET}")
      |> range(start: -10m)
      |> filter(fn: (r) => r._measurement == "sensor")
      |> filter(fn: (r) => r["robot_id"] == "sim_robot_1")
      |> filter(fn: (r) => r["_field"] == "value")
      |> last()
    '''

    tables = query_api.query(query=query, org=INFLUX_ORG)

    # Si no hay datos, devolvemos algo vacío
    if not tables or not tables[0].records:
        return {"value": None, "time": None, "robot_id": "sim_robot_1"}

    record = tables[0].records[0]
    return {
        "value": record.get_value(),
        "time": record.get_time().isoformat(),
        "robot_id": record.values.get("robot_id", "sim_robot_1"),
    }






@app.get("/api/history")
def get_history(minutes: int = Query(10, ge=1, le=1440)) -> List[dict]:
    query = f"""
    from(bucket: "{INFLUX_BUCKET}")
      |> range(start: -{minutes}m)
      |> filter(fn: (r) => r._measurement == "sensor")
      |> filter(fn: (r) => r._field == "value")
      |> sort(columns: ["_time"])
    """
    print("Flux query history:\n", query)

    try:
        tables = query_api.query(org=INFLUX_ORG, query=query)
    except Exception as e:
        print("Error consultando Influx en /api/history:", e)
        raise HTTPException(status_code=500, detail=str(e))

    points = []
    for table in tables:
        for record in table.records:
            points.append({
                "time": record.get_time().isoformat(),
                "value": record.get_value(),
                "robot_id": record.values.get("robot_id", "sim_robot_1"),
            })

    return points

@app.get("/api/robots")
def get_robots() -> List[dict]:
    """
    Devuelve la lista de robots y su estado actual.
    Por ahora solo sim_robot_1 basado en la última lectura.
    """
    latest = get_latest_reading()

    value = latest["value"] if latest else None
    status = "Desconocido"
    if value is not None:
        if value < 50:
            status = "Normal"
        elif value < 100:
            status = "Alerta"
        else:
            status = "Peligro"

    return [
        {
            "id": "RP1",
            "nombre": "Robot Simulado 1",
            "ultima_lectura": value,
            "estado": status,
            "ultimo_reporte": latest["time"] if latest else None,
        }
    ]

    return points
