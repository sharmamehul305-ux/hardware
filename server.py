"""FastAPI dashboard and fleet command API."""

from __future__ import annotations

import argparse
import logging
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from pydantic import BaseModel, Field

from fleet import Fleet

LOGGER = logging.getLogger(__name__)
BASE_DIR = Path(__file__).resolve().parent


class DroneSelection(BaseModel):
    drone: str | None = None


class TakeoffRequest(DroneSelection):
    altitude_m: float = Field(default=10.0, gt=0)


class GotoRequest(DroneSelection):
    latitude_deg: float
    longitude_deg: float
    altitude_m: float = Field(ge=0)


class FormationRequest(DroneSelection):
    formation: str
    altitude_m: float = Field(default=10.0, ge=0)
    spacing_m: float = Field(default=8.0, gt=0)


class MissionRequest(DroneSelection):
    path: str = "missions/sample.json"


class EmergencyRequest(BaseModel):
    confirm: bool = False


def create_app(fleet: Fleet) -> FastAPI:
    @asynccontextmanager
    async def lifespan(_: FastAPI):
        try:
            results = await fleet.connect_all()
            LOGGER.info("Initial connection results: %s", results)
            yield
        finally:
            await fleet.close()

    app = FastAPI(title="Multi-Drone Fleet Control", lifespan=lifespan)

    async def invoke(operation: Any, *args: Any, **kwargs: Any) -> Any:
        try:
            return await operation(*args, **kwargs)
        except (ValueError, RuntimeError) as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        except Exception as exc:
            LOGGER.exception("Fleet command failed")
            raise HTTPException(status_code=502, detail=f"Fleet command failed: {exc}") from exc

    @app.get("/")
    async def dashboard() -> FileResponse:
        return FileResponse(BASE_DIR / "static" / "index.html")

    @app.get("/api/state")
    async def state() -> dict[str, Any]:
        return fleet.snapshot()

    @app.post("/api/connect")
    async def connect() -> dict[str, bool]:
        return await fleet.connect_all()

    @app.post("/api/arm")
    async def arm(request: DroneSelection) -> dict[str, str]:
        return await invoke(fleet.arm, request.drone)

    @app.post("/api/takeoff")
    async def takeoff(request: TakeoffRequest) -> dict[str, Any]:
        return await invoke(fleet.takeoff, request.drone, request.altitude_m)

    @app.post("/api/goto")
    async def goto(request: GotoRequest) -> dict[str, Any]:
        return await invoke(
            fleet.goto,
            request.latitude_deg,
            request.longitude_deg,
            request.altitude_m,
            request.drone,
        )

    @app.post("/api/land")
    async def land(request: DroneSelection) -> dict[str, str]:
        return await invoke(fleet.land, request.drone)

    @app.post("/api/rtl")
    async def rtl(request: DroneSelection) -> dict[str, str]:
        return await invoke(fleet.rtl, request.drone)

    @app.post("/api/formation")
    async def formation(request: FormationRequest) -> dict[str, Any]:
        return await invoke(
            fleet.formation,
            request.formation,
            request.altitude_m,
            request.spacing_m,
            request.drone,
        )

    @app.post("/api/mission")
    async def mission(request: MissionRequest) -> dict[str, str]:
        missions_dir = (BASE_DIR / "missions").resolve()
        mission_path = Path(request.path)
        if not mission_path.is_absolute():
            mission_path = BASE_DIR / mission_path
        mission_path = mission_path.resolve()
        if mission_path != missions_dir and missions_dir not in mission_path.parents:
            raise HTTPException(status_code=400, detail="Mission file must be inside the missions directory")
        return await invoke(fleet.run_mission_file, str(mission_path), request.drone)

    @app.post("/api/emergency-stop")
    async def emergency_stop(request: EmergencyRequest) -> dict[str, str]:
        return await invoke(fleet.emergency_stop, request.confirm)

    @app.websocket("/ws")
    async def websocket_endpoint(websocket: WebSocket) -> None:
        await websocket.accept()
        queue = await fleet.subscribe()
        try:
            while True:
                await websocket.send_json(await queue.get())
        except WebSocketDisconnect:
            pass
        except Exception:
            LOGGER.exception("Dashboard WebSocket failed")
        finally:
            fleet.unsubscribe(queue)

    return app


def main() -> None:
    parser = argparse.ArgumentParser(description="Multi-drone MAVSDK fleet controller")
    parser.add_argument("--config", default=str(BASE_DIR / "config.yaml"))
    parser.add_argument(
        "--real",
        action="store_true",
        help="Allow real-hardware endpoints from config (never enabled by default)",
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
        handlers=[
            logging.StreamHandler(),
            logging.FileHandler(BASE_DIR / "fleet.log", encoding="utf-8"),
        ],
    )
    try:
        fleet = Fleet(args.config, real_mode=args.real)
    except ValueError as exc:
        parser.error(str(exc))
        return

    import uvicorn

    uvicorn.run(create_app(fleet), host=args.host, port=args.port, log_level="info")


if __name__ == "__main__":
    main()
