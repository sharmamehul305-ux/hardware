"""Fleet orchestration, configuration, and structured logging."""

from __future__ import annotations

import asyncio
import csv
import json
import logging
import os
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml

from drone import DroneController, DroneTelemetry
from formations import generate_formation
from safety import (
    GeoPoint,
    SafetyLimits,
    separation_warnings,
    validate_separation,
    validate_target,
)

LOGGER = logging.getLogger(__name__)


class Fleet:
    """Coordinates connections, commands, telemetry, and safety across drones."""

    def __init__(self, config_path: str | Path, real_mode: bool = False) -> None:
        self.config_path = Path(config_path)
        self.real_mode = real_mode
        self.config = self._read_config(self.config_path)
        self.autopilot = str(self.config.get("autopilot", "PX4"))
        if self.autopilot not in {"PX4", "ArduPilot"}:
            raise ValueError("autopilot must be either 'PX4' or 'ArduPilot'")
        home_config = self.config["home"]
        self.home = GeoPoint(
            float(home_config["latitude_deg"]),
            float(home_config["longitude_deg"]),
            float(home_config.get("altitude_m", 0.0)),
        )
        limits_config = self.config.get("safety", {})
        self.limits = SafetyLimits(
            max_radius_m=float(limits_config.get("max_radius_m", 200)),
            max_altitude_m=float(limits_config.get("max_altitude_m", 50)),
            min_separation_m=float(limits_config.get("min_separation_m", 5)),
            low_battery_percent=float(limits_config.get("low_battery_percent", 25)),
            link_loss_seconds=float(limits_config.get("link_loss_seconds", 5)),
        )
        self.drones: dict[str, DroneController] = {}
        self.telemetry: dict[str, DroneTelemetry] = {}
        self._subscribers: set[asyncio.Queue[dict[str, Any]]] = set()
        self._telemetry_path = Path(self.config.get("logging", {}).get("directory", "logs"))
        self._telemetry_path.mkdir(parents=True, exist_ok=True)
        self._telemetry_csv = self._telemetry_path / "telemetry.csv"
        self._commands_csv = self._telemetry_path / "commands.csv"
        self._ensure_csv_headers()
        for entry in self.config.get("drones", []):
            name = str(entry["name"])
            system_address = entry.get("system_address")
            if real_mode:
                system_address = entry.get("real_system_address")
                if not system_address:
                    raise ValueError(
                        f"Drone {name} has no real_system_address; configure it explicitly"
                    )
            if not system_address:
                raise ValueError(f"Drone {name} has no system_address")
            self.drones[name] = DroneController(
                name=name,
                system_address=str(system_address),
                grpc_port=int(entry["grpc_port"]),
                home=self.home,
                limits=self.limits,
                telemetry_callback=self._on_telemetry,
                real_mode=real_mode,
            )
        if not self.drones:
            raise ValueError("Configuration must define at least one drone")

    @staticmethod
    def _read_config(path: Path) -> dict[str, Any]:
        try:
            with path.open("r", encoding="utf-8") as config_file:
                result = yaml.safe_load(config_file)
        except (OSError, yaml.YAMLError) as exc:
            raise ValueError(f"Could not load configuration {path}: {exc}") from exc
        if not isinstance(result, dict) or "home" not in result:
            raise ValueError("Configuration must be a YAML object with a home section")
        return result

    def _ensure_csv_headers(self) -> None:
        if not self._telemetry_csv.exists():
            with self._telemetry_csv.open("w", newline="", encoding="utf-8") as output:
                writer = csv.DictWriter(
                    output,
                    fieldnames=[
                        "timestamp",
                        "name",
                        "connected",
                        "latitude_deg",
                        "longitude_deg",
                        "altitude_m",
                        "speed_m_s",
                        "battery_percent",
                        "flight_mode",
                        "gps_healthy",
                        "is_armable",
                    ],
                )
                writer.writeheader()
        if not self._commands_csv.exists():
            with self._commands_csv.open("w", newline="", encoding="utf-8") as output:
                csv.writer(output).writerow(["timestamp", "command", "drone", "details", "result"])

    def _on_telemetry(self, state: DroneTelemetry) -> None:
        self.telemetry[state.name] = state
        payload = self.snapshot()
        for subscriber in tuple(self._subscribers):
            try:
                subscriber.put_nowait(payload)
            except asyncio.QueueFull:
                try:
                    subscriber.get_nowait()
                    subscriber.put_nowait(payload)
                except asyncio.QueueEmpty:
                    pass
        row = state.to_dict()
        row["timestamp"] = datetime.now(timezone.utc).isoformat()
        try:
            with self._telemetry_csv.open("a", newline="", encoding="utf-8") as output:
                writer = csv.DictWriter(output, fieldnames=row.keys())
                writer.writerow(row)
        except OSError:
            LOGGER.exception("Unable to append telemetry CSV")

    def snapshot(self) -> dict[str, Any]:
        return {
            "mode": "real" if self.real_mode else "simulation",
            "autopilot": self.autopilot,
            "drones": {
                name: state.to_dict() for name, state in sorted(self.telemetry.items())
            },
        }

    async def connect_all(self, timeout_s: float = 20.0) -> dict[str, bool]:
        results = await asyncio.gather(
            *(drone.connect(timeout_s) for drone in self.drones.values()),
            return_exceptions=True,
        )
        outcome: dict[str, bool] = {}
        for name, result in zip(self.drones, results):
            if isinstance(result, asyncio.CancelledError):
                raise result
            if isinstance(result, Exception):
                LOGGER.error("%s connection raised: %s", name, result)
                outcome[name] = False
            else:
                outcome[name] = result
        return outcome

    def _selected(self, name: str | None) -> list[DroneController]:
        if name is None:
            selected = list(self.drones.values())
        else:
            try:
                selected = [self.drones[name]]
            except KeyError as exc:
                raise ValueError(f"Unknown drone {name!r}") from exc
        if not selected:
            raise RuntimeError("No drones are configured")
        return selected

    def _log_command(self, command: str, names: list[str], details: Any, result: str) -> None:
        try:
            with self._commands_csv.open("a", newline="", encoding="utf-8") as output:
                writer = csv.writer(output)
                writer.writerow(
                    [
                        datetime.now(timezone.utc).isoformat(),
                        command,
                        ",".join(names),
                        json.dumps(details, sort_keys=True),
                        result,
                    ]
                )
        except OSError:
            LOGGER.exception("Unable to append command CSV")

    async def _run_command(
        self,
        command: str,
        drones: list[DroneController],
        operation: Any,
        details: Any = None,
    ) -> dict[str, str]:
        names = [drone.name for drone in drones]
        results = await asyncio.gather(
            *(operation(drone) for drone in drones), return_exceptions=True
        )
        status: dict[str, str] = {}
        for drone, result in zip(drones, results):
            if isinstance(result, asyncio.CancelledError):
                raise result
            if isinstance(result, Exception):
                status[drone.name] = str(result)
                LOGGER.error("%s command %s failed: %s", drone.name, command, result)
            else:
                status[drone.name] = "ok"
        self._log_command(command, names, details, json.dumps(status, sort_keys=True))
        return status

    async def arm(self, name: str | None = None) -> dict[str, str]:
        return await self._run_command("arm", self._selected(name), lambda d: d.arm())

    async def takeoff(self, name: str | None = None, altitude_m: float = 10.0) -> dict[str, str]:
        selected = self._selected(name)
        targets = [
            altitude_m + index * float(self.config.get("takeoff_stagger_m", 3.0))
            for index in range(len(selected))
        ]
        for altitude in targets:
            validate_target(GeoPoint(self.home.latitude_deg, self.home.longitude_deg, altitude), self.home, self.limits)
        return await self._run_command(
            "takeoff",
            selected,
            lambda drone: drone.takeoff(
                altitude_m
                + selected.index(drone) * float(self.config.get("takeoff_stagger_m", 3.0))
            ),
            {"base_altitude_m": altitude_m, "stagger_m": self.config.get("takeoff_stagger_m", 3.0)},
        )

    async def goto(
        self, latitude_deg: float, longitude_deg: float, altitude_m: float, name: str | None = None
    ) -> dict[str, Any]:
        selected = self._selected(name)
        target = GeoPoint(latitude_deg, longitude_deg, altitude_m)
        validate_target(target, self.home, self.limits)
        destinations = {drone.name: target for drone in selected}
        warnings = separation_warnings(destinations, self.limits.min_separation_m)
        for drone in selected:
            peers = [
                (other.name, state.position())
                for other in self.drones.values()
                if other.name != drone.name
                and other.name not in destinations
                and (state := self.telemetry.get(other.name)) is not None
                and state.position() is not None
            ]
            warnings.extend(
                validate_separation(
                    target,
                    [(peer, position) for peer, position in peers if position is not None],
                    self.limits.min_separation_m,
                    drone.name,
                )
            )
        for warning in warnings:
            LOGGER.warning("Separation warning: %s", warning)
        status = await self._run_command(
            "goto",
            selected,
            lambda drone: drone.goto(target),
            target.__dict__,
        )
        return {"results": status, "warnings": warnings}

    async def land(self, name: str | None = None) -> dict[str, str]:
        return await self._run_command("land", self._selected(name), lambda d: d.land())

    async def rtl(self, name: str | None = None) -> dict[str, str]:
        return await self._run_command("rtl", self._selected(name), lambda d: d.return_to_launch())

    async def emergency_stop(self, confirmed: bool) -> dict[str, str]:
        if confirmed is not True:
            raise ValueError("Emergency stop requires explicit confirmation")
        return await self._run_command(
            "emergency_land", self._selected(None), lambda drone: drone.land()
        )

    async def formation(
        self,
        formation: str,
        altitude_m: float,
        spacing_m: float = 8.0,
        name: str | None = None,
    ) -> dict[str, Any]:
        selected = self._selected(name)
        points = generate_formation(
            formation, self.home, len(selected), spacing_m=spacing_m, altitude_m=altitude_m
        )
        destinations = dict(zip((drone.name for drone in selected), points))
        for target in points:
            validate_target(target, self.home, self.limits)
        warnings = []
        for drone_name, target in destinations.items():
            warnings.extend(
                validate_separation(
                    target,
                    [
                        (other, point)
                        for other, point in destinations.items()
                        if other != drone_name
                    ],
                    self.limits.min_separation_m,
                    drone_name,
                )
            )
        for warning in warnings:
            LOGGER.warning("Separation warning: %s", warning)
        status = await self._run_command(
            f"formation:{formation}",
            selected,
            lambda drone: drone.goto(destinations[drone.name]),
            {"altitude_m": altitude_m, "spacing_m": spacing_m},
        )
        return {"results": status, "warnings": sorted(set(warnings))}

    async def run_mission_file(self, path: str, name: str | None = None) -> dict[str, str]:
        selected = self._selected(name)
        mission_path = Path(path)
        try:
            with mission_path.open("r", encoding="utf-8") as source:
                mission_data = json.load(source)
        except (OSError, json.JSONDecodeError) as exc:
            raise ValueError(f"Could not read mission JSON {mission_path}: {exc}") from exc
        if isinstance(mission_data, list):
            waypoints = mission_data
        elif isinstance(mission_data, dict) and isinstance(mission_data.get("waypoints"), list):
            waypoints = mission_data["waypoints"]
        else:
            raise ValueError("Mission JSON must be a waypoint list or an object with 'waypoints'")
        return await self._run_command(
            "mission",
            selected,
            lambda drone: drone.upload_and_start_mission(waypoints),
            {"path": os.fspath(mission_path), "waypoint_count": len(waypoints)},
        )

    async def subscribe(self) -> asyncio.Queue[dict[str, Any]]:
        queue: asyncio.Queue[dict[str, Any]] = asyncio.Queue(maxsize=1)
        self._subscribers.add(queue)
        await queue.put(self.snapshot())
        return queue

    def unsubscribe(self, queue: asyncio.Queue[dict[str, Any]]) -> None:
        self._subscribers.discard(queue)

    async def close(self) -> None:
        await asyncio.gather(*(drone.close() for drone in self.drones.values()))
