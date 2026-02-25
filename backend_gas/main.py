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


 #Implementacion del mapa de calor
@app.get("/api/heatmap_grid")
def get_heatmap_grid(
    minutes: int = Query(10, ge=1, le=1440),
    x_bins: int = Query(20, ge=2, le=200),
    y_bins: int = Query(20, ge=2, le=200),
):
    """
    Devuelve una grilla espacial agregada: cada celda contiene la
    concentración promedio de las muestras que cayeron dentro.
    x_bins e y_bins controlan la resolución (número de celdas).
    """
    query = f"""
    from(bucket: "{INFLUX_BUCKET}")
    |> range(start: -{minutes}m)
    |> filter(fn: (r) => r._measurement == "sensor_xy")
    |> filter(fn: (r) => r["robot_id"] == "sim_robot_1")
    |> pivot(rowKey:["_time"], columnKey:["_field"], valueColumn:"_value")
    |> keep(columns: ["_time", "x", "y", "concentration"])
    """


    try:
        tables = query_api.query(org=INFLUX_ORG, query=query)
    except Exception as e:
        print("Error consultando Influx en /api/heatmap_grid:", e)
        raise HTTPException(status_code=500, detail=str(e))

    raw_points = []
    for table in tables:
        for record in table.records:
            v = record.values
            if "x" in v and "y" in v and "concentration" in v:
                try:
                    raw_points.append({
                        "x": float(v["x"]),
                        "y": float(v["y"]),
                        "concentration": float(v["concentration"]),
                    })
                except (TypeError, ValueError):
                    continue

    if not raw_points:
        return {"cells": [], "x_bins": x_bins, "y_bins": y_bins}

    # Rango espacial 
    xs = [p["x"] for p in raw_points]
    ys = [p["y"] for p in raw_points]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)

    # Paso de cada celda
    x_step = (max_x - min_x) / x_bins if max_x > min_x else 1.0
    y_step = (max_y - min_y) / y_bins if max_y > min_y else 1.0

    # Acumuladores: (suma, conteo) por celda
    grid = {}  # clave (ix, iy) -> {"sum": ..., "count": ...}

    for p in raw_points:
        ix = int((p["x"] - min_x) / x_step) if x_step > 0 else 0
        iy = int((p["y"] - min_y) / y_step) if y_step > 0 else 0

        # Limitar índices al rango [0, bins-1]
        ix = min(max(ix, 0), x_bins - 1)
        iy = min(max(iy, 0), y_bins - 1)

        key = (ix, iy)
        if key not in grid:
            grid[key] = {"sum": 0.0, "count": 0}
        grid[key]["sum"] += p["concentration"]
        grid[key]["count"] += 1

    cells = []
    for (ix, iy), acc in grid.items():
        if acc["count"] == 0:
            continue
        avg = acc["sum"] / acc["count"]

        # Centro de la celda
        cell_x = min_x + (ix + 0.5) * x_step
        cell_y = min_y + (iy + 0.5) * y_step

        cells.append({
            "x": cell_x,
            "y": cell_y,
            "concentration": avg,
            "ix": ix,
            "iy": iy,
        })

    return {
        "cells": cells,
        "x_bins": x_bins,
        "y_bins": y_bins,
        "min_x": min_x,
        "max_x": max_x,
        "min_y": min_y,
        "max_y": max_y,
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
