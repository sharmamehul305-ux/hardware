"""Geographic and fleet-level safety checks."""

from __future__ import annotations

from dataclasses import dataclass
from math import asin, cos, isfinite, radians, sin, sqrt
from typing import Iterable


EARTH_RADIUS_M = 6_371_000.0


@dataclass(frozen=True)
class GeoPoint:
    latitude_deg: float
    longitude_deg: float
    altitude_m: float


@dataclass(frozen=True)
class SafetyLimits:
    max_radius_m: float = 200.0
    max_altitude_m: float = 50.0
    min_separation_m: float = 5.0
    low_battery_percent: float = 25.0
    link_loss_seconds: float = 5.0


def haversine_m(lat1_deg: float, lon1_deg: float, lat2_deg: float, lon2_deg: float) -> float:
    """Return the great-circle distance between two GPS coordinates in metres."""
    lat1, lat2 = radians(lat1_deg), radians(lat2_deg)
    dlat = lat2 - lat1
    dlon = radians(lon2_deg - lon1_deg)
    a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2
    return 2 * EARTH_RADIUS_M * asin(sqrt(a))


def distance_3d_m(first: GeoPoint, second: GeoPoint) -> float:
    """Return straight-line separation using horizontal GPS and relative altitude."""
    horizontal = haversine_m(
        first.latitude_deg,
        first.longitude_deg,
        second.latitude_deg,
        second.longitude_deg,
    )
    return sqrt(horizontal**2 + (first.altitude_m - second.altitude_m) ** 2)


def validate_altitude(altitude_m: float, limits: SafetyLimits) -> None:
    """Reject a target altitude outside the configured operating envelope."""
    if not 0 <= altitude_m <= limits.max_altitude_m:
        raise ValueError(
            f"Altitude {altitude_m:.1f} m is outside 0-{limits.max_altitude_m:.1f} m"
        )


def validate_target(target: GeoPoint, home: GeoPoint, limits: SafetyLimits) -> None:
    """Reject a target outside the home-centred radius or altitude limit."""
    if not all(
        isfinite(value)
        for value in (
            target.latitude_deg,
            target.longitude_deg,
            target.altitude_m,
            home.latitude_deg,
            home.longitude_deg,
        )
    ):
        raise ValueError("Target and home coordinates must be finite numbers")
    if not -90 <= target.latitude_deg <= 90:
        raise ValueError("Latitude must be between -90 and 90 degrees")
    if not -180 <= target.longitude_deg <= 180:
        raise ValueError("Longitude must be between -180 and 180 degrees")
    if not -90 <= home.latitude_deg <= 90 or not -180 <= home.longitude_deg <= 180:
        raise ValueError("Configured home coordinates are outside valid GPS bounds")
    validate_altitude(target.altitude_m, limits)
    distance = haversine_m(
        home.latitude_deg,
        home.longitude_deg,
        target.latitude_deg,
        target.longitude_deg,
    )
    if distance > limits.max_radius_m:
        raise ValueError(
            f"Target is {distance:.1f} m from home; maximum radius is "
            f"{limits.max_radius_m:.1f} m"
        )


def separation_warnings(
    targets: dict[str, GeoPoint], min_separation_m: float
) -> list[str]:
    """Report target pairs closer than the minimum; warnings do not reject commands."""
    warnings: list[str] = []
    names = list(targets)
    for index, name in enumerate(names):
        for other in names[index + 1 :]:
            a, b = targets[name], targets[other]
            distance = distance_3d_m(a, b)
            if distance < min_separation_m:
                warnings.append(
                    f"{name} and {other} are {distance:.1f} m apart "
                    f"(minimum {min_separation_m:.1f} m)"
                )
    return warnings


def validate_separation(
    target: GeoPoint,
    other_positions: Iterable[tuple[str, GeoPoint]],
    min_separation_m: float,
    drone_name: str,
) -> list[str]:
    """Return warnings for a proposed position close to another drone."""
    warnings: list[str] = []
    for other_name, other in other_positions:
        distance = distance_3d_m(target, other)
        if distance < min_separation_m:
            warnings.append(
                f"{drone_name} and {other_name} would be {distance:.1f} m apart "
                f"(minimum {min_separation_m:.1f} m)"
            )
    return warnings
