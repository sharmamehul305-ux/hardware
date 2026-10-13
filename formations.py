"""Formation geometry in a local north/east tangent plane."""

from __future__ import annotations

from math import cos, pi, sin

from safety import GeoPoint


FORMATION_NAMES = ("line", "grid", "circle", "v")


def _offset_point(center: GeoPoint, north_m: float, east_m: float, altitude_m: float) -> GeoPoint:
    """Convert local metre offsets to an approximate GPS coordinate."""
    latitude = center.latitude_deg + north_m / 111_320.0
    longitude_scale = 111_320.0 * cos(center.latitude_deg * pi / 180)
    if abs(longitude_scale) < 1e-6:
        raise ValueError("Cannot calculate local formation offsets at this latitude")
    longitude = center.longitude_deg + east_m / longitude_scale
    return GeoPoint(latitude, longitude, altitude_m)


def generate_formation(
    formation: str,
    center: GeoPoint,
    count: int,
    spacing_m: float = 8.0,
    altitude_m: float | None = None,
) -> list[GeoPoint]:
    """Generate ordered target positions for line, grid, circle, or V formations."""
    if formation not in FORMATION_NAMES:
        raise ValueError(f"Unsupported formation {formation!r}; choose {FORMATION_NAMES}")
    if count < 1:
        raise ValueError("A formation requires at least one drone")
    if spacing_m <= 0:
        raise ValueError("Formation spacing must be greater than zero")

    altitude = center.altitude_m if altitude_m is None else altitude_m
    offsets: list[tuple[float, float]] = []
    if formation == "line":
        offsets = [(0.0, (i - (count - 1) / 2) * spacing_m) for i in range(count)]
    elif formation == "grid":
        columns = max(1, int(count**0.5))
        rows = (count + columns - 1) // columns
        offsets = [
            (
                (rows - 1) * spacing_m / 2 - (i // columns) * spacing_m,
                (i % columns - (min(columns, count - (i // columns) * columns) - 1) / 2)
                * spacing_m,
            )
            for i in range(count)
        ]
    elif formation == "circle":
        radius = spacing_m if count > 1 else 0.0
        offsets = [
            (
                radius * cos(2 * pi * i / count),
                radius * sin(2 * pi * i / count),
            )
            for i in range(count)
        ]
    else:
        offsets = [(0.0, 0.0)]
        for i in range(1, count):
            row = (i + 1) // 2
            side = -1 if i % 2 else 1
            offsets.append((-row * spacing_m, side * row * spacing_m))

    return [
        _offset_point(center, north, east, altitude) for north, east in offsets
    ]
