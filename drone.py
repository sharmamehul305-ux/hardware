"""MAVSDK-backed controller for one drone."""

from __future__ import annotations

import asyncio
import logging
import math
import time
from collections.abc import AsyncIterable
from dataclasses import asdict, dataclass
from typing import Any, Callable

from mavsdk import System

from safety import GeoPoint, SafetyLimits, validate_target

LOGGER = logging.getLogger(__name__)


@dataclass
class DroneTelemetry:
    name: str
    connected: bool = False
    latitude_deg: float | None = None
    longitude_deg: float | None = None
    altitude_m: float | None = None
    speed_m_s: float | None = None
    battery_percent: float | None = None
    flight_mode: str = "UNKNOWN"
    gps_healthy: bool = False
    is_armable: bool = False
    last_update: float | None = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    def position(self) -> GeoPoint | None:
        if self.latitude_deg is None or self.longitude_deg is None or self.altitude_m is None:
            return None
        return GeoPoint(self.latitude_deg, self.longitude_deg, self.altitude_m)


class DroneController:
    """Owns one MAVSDK connection and its telemetry/failsafe tasks."""

    def __init__(
        self,
        name: str,
        system_address: str,
        grpc_port: int,
        home: GeoPoint,
        limits: SafetyLimits,
        telemetry_callback: Callable[[DroneTelemetry], None],
        real_mode: bool = False,
    ) -> None:
        self.name = name
        self.system_address = system_address
        self.grpc_port = grpc_port
        self.home = home
        self.limits = limits
        self.telemetry_callback = telemetry_callback
        self.real_mode = real_mode
        self.system = System(port=grpc_port)
        self.state = DroneTelemetry(name=name)
        self.connected = False
        self._tasks: list[asyncio.Task[None]] = []
        self._failsafe_lock = asyncio.Lock()
        self._failsafe_triggered = False

    async def connect(self, timeout_s: float = 20.0) -> bool:
        """Connect and wait for the MAVLink link; return False on timeout/failure."""
        if self.connected:
            return True
        deadline = asyncio.get_running_loop().time() + timeout_s
        try:
            await asyncio.wait_for(
                self.system.connect(system_address=self.system_address), timeout_s
            )
            remaining = max(0.0, deadline - asyncio.get_running_loop().time())
            await asyncio.wait_for(self._wait_connected(), remaining)
            self.connected = True
            self.state.connected = True
            self._start_telemetry()
            self.telemetry_callback(self.state)
            LOGGER.info("%s connected on %s", self.name, self.system_address)
            return True
        except asyncio.CancelledError:
            raise
        except asyncio.TimeoutError:
            LOGGER.error("%s connection timed out after %.1f seconds", self.name, timeout_s)
        except Exception:
            LOGGER.exception("%s connection failed", self.name)
        self.connected = False
        self.state.connected = False
        self.telemetry_callback(self.state)
        return False

    async def _wait_connected(self) -> None:
        async for connection_state in self.system.core.connection_state():
            if connection_state.is_connected:
                return

    def _start_telemetry(self) -> None:
        streams: list[tuple[str, Awaitable[Any], Callable[[Any], None]]] = [
            ("position", self.system.telemetry.position(), self._set_position),
            ("velocity", self.system.telemetry.velocity_ned(), self._set_speed),
            ("battery", self.system.telemetry.battery(), self._set_battery),
            ("flight mode", self.system.telemetry.flight_mode(), self._set_flight_mode),
            ("health", self.system.telemetry.health(), self._set_health),
        ]
        for stream_name, stream, setter in streams:
            self._tasks.append(
                asyncio.create_task(self._consume_stream(stream_name, stream, setter))
            )
        self._tasks.append(asyncio.create_task(self._watch_connection()))
        self._tasks.append(asyncio.create_task(self._publish_telemetry()))

    async def _consume_stream(
        self, name: str, stream: AsyncIterable[Any], setter: Callable[[Any], None]
    ) -> None:
        try:
            async for value in stream:
                setter(value)
                self.state.last_update = time.time()
        except asyncio.CancelledError:
            raise
        except Exception:
            LOGGER.exception("%s %s telemetry stream failed", self.name, name)
            self.state.connected = False
            self.connected = False

    def _set_position(self, value: Any) -> None:
        self.state.latitude_deg = float(value.latitude_deg)
        self.state.longitude_deg = float(value.longitude_deg)
        self.state.altitude_m = float(value.relative_altitude_m)

    def _set_speed(self, value: Any) -> None:
        self.state.speed_m_s = math.sqrt(
            value.north_m_s**2 + value.east_m_s**2 + value.down_m_s**2
        )

    def _set_battery(self, value: Any) -> None:
        self.state.battery_percent = float(value.remaining_percent) * 100
        if self.state.battery_percent < self.limits.low_battery_percent:
            asyncio.create_task(self._failsafe("low battery", land=False))

    def _set_flight_mode(self, value: Any) -> None:
        self.state.flight_mode = str(value).split(".")[-1]

    def _set_health(self, value: Any) -> None:
        self.state.gps_healthy = bool(value.is_global_position_ok)
        self.state.is_armable = bool(value.is_armable)

    async def _publish_telemetry(self) -> None:
        while self.connected:
            self.telemetry_callback(self.state)
            await asyncio.sleep(1)

    async def _watch_connection(self) -> None:
        lost_since: float | None = None
        iterator = self.system.core.connection_state().__aiter__()
        pending: asyncio.Task[Any] | None = asyncio.create_task(anext(iterator))
        try:
            while True:
                done, _ = await asyncio.wait({pending}, timeout=0.2)
                if pending in done:
                    state = pending.result()
                    if state.is_connected:
                        lost_since = None
                        self.state.connected = True
                        self.connected = True
                    else:
                        self.state.connected = False
                        self.connected = False
                        if lost_since is None:
                            lost_since = time.monotonic()
                    pending = asyncio.create_task(anext(iterator))
                if (
                    lost_since is not None
                    and time.monotonic() - lost_since >= self.limits.link_loss_seconds
                ):
                    await self._failsafe("connection lost", land=True)
                    return
        except asyncio.CancelledError:
            raise
        except Exception:
            LOGGER.exception("%s connection monitor failed", self.name)
            self.state.connected = False
            self.connected = False
            await asyncio.sleep(self.limits.link_loss_seconds)
            await self._failsafe("connection monitor stopped", land=True)
        finally:
            if pending is not None and not pending.done():
                pending.cancel()

    async def _failsafe(self, reason: str, land: bool) -> None:
        async with self._failsafe_lock:
            if self._failsafe_triggered:
                return
            self._failsafe_triggered = True
        LOGGER.critical("%s failsafe activated: %s", self.name, reason)
        try:
            if land:
                await self.system.action.land()
            else:
                await self.system.action.return_to_launch()
        except Exception:
            LOGGER.exception("%s failsafe command failed", self.name)

    def require_connected(self) -> None:
        if not self.connected:
            raise RuntimeError(f"Drone {self.name} is not connected")

    async def arm(self) -> None:
        self.require_connected()
        await self.system.action.arm()

    async def takeoff(self, altitude_m: float) -> None:
        self.require_connected()
        validate_target(
            GeoPoint(self.home.latitude_deg, self.home.longitude_deg, altitude_m),
            self.home,
            self.limits,
        )
        await self.system.action.set_takeoff_altitude(altitude_m)
        await self.system.action.takeoff()

    async def goto(self, target: GeoPoint) -> None:
        self.require_connected()
        validate_target(target, self.home, self.limits)
        await self.system.action.goto_location(
            target.latitude_deg, target.longitude_deg, target.altitude_m, 0.0
        )

    async def land(self) -> None:
        self.require_connected()
        await self.system.action.land()

    async def return_to_launch(self) -> None:
        self.require_connected()
        await self.system.action.return_to_launch()

    async def upload_and_start_mission(self, waypoints: list[dict[str, Any]]) -> None:
        """Upload and execute validated mission waypoints."""
        self.require_connected()
        if not waypoints:
            raise ValueError("Mission must contain at least one waypoint")
        from mavsdk.mission import MissionItem, MissionPlan

        items = []
        for waypoint in waypoints:
            point = GeoPoint(
                float(waypoint["latitude_deg"]),
                float(waypoint["longitude_deg"]),
                float(waypoint["altitude_m"]),
            )
            validate_target(point, self.home, self.limits)
            speed = float(waypoint.get("speed_m_s", 5.0))
            if speed <= 0:
                raise ValueError("Waypoint speed_m_s must be greater than zero")
            items.append(
                MissionItem(
                    point.latitude_deg,
                    point.longitude_deg,
                    point.altitude_m,
                    speed,
                    bool(waypoint.get("is_fly_through", False)),
                    float(waypoint.get("gimbal_pitch_deg", 0.0)),
                    float(waypoint.get("gimbal_yaw_deg", 0.0)),
                    MissionItem.CameraAction.NONE,
                )
            )
        await self.system.mission.upload_mission(MissionPlan(items))
        await self.system.mission.start_mission()

    async def close(self) -> None:
        self.connected = False
        for task in self._tasks:
            task.cancel()
        if self._tasks:
            await asyncio.gather(*self._tasks, return_exceptions=True)
