"""Streamlit agriculture-drone simulation dashboard.

All flight state and movement in this application are simulated locally.
This module intentionally contains no hardware or MAVSDK connections.
"""

from __future__ import annotations

import math
import random
import time
from typing import Any

import plotly.graph_objects as go
import streamlit as st

FIELD_SURVEY = "Field survey"
CROP_MONITORING = "Crop monitoring"
SIMULATED_SPRAYING = "Simulated spraying"
MISSION_TYPES = (FIELD_SURVEY, CROP_MONITORING, SIMULATED_SPRAYING)

HOME_LATITUDE = 47.397742
HOME_LONGITUDE = 8.545594
GRID_ROWS = 18
GRID_COLUMNS = 24
SPRAY_SWATH_M = 12.0
MISSION_SPACING_M = {
    FIELD_SURVEY: 24.0,
    CROP_MONITORING: 32.0,
    SIMULATED_SPRAYING: 10.0,
}


def build_mission_route(
    mission_type: str, field_length_m: float, field_width_m: float
) -> list[tuple[float, float]]:
    """Build a serpentine route in metres centered on the simulated field."""
    if mission_type not in MISSION_TYPES:
        raise ValueError(f"Unsupported mission type: {mission_type}")
    if not all(
        math.isfinite(dimension) and dimension > 0
        for dimension in (field_length_m, field_width_m)
    ):
        raise ValueError("Field dimensions must be positive finite numbers")

    lane_spacing = MISSION_SPACING_M[mission_type]
    half_length = field_length_m / 2
    half_width = field_width_m / 2
    lane_count = max(2, math.ceil(field_width_m / lane_spacing) + 1)
    y_positions = [
        -half_width + index * field_width_m / (lane_count - 1)
        for index in range(lane_count)
    ]

    route: list[tuple[float, float]] = []
    for index, y_position in enumerate(y_positions):
        start_x, end_x = (
            (-half_length, half_length)
            if index % 2 == 0
            else (half_length, -half_length)
        )
        if not route:
            route.append((start_x, y_position))
        else:
            route.append((route[-1][0], y_position))
        route.append((end_x, y_position))
    return route


def route_length_m(route: list[tuple[float, float]]) -> float:
    """Calculate the total two-dimensional length of a waypoint route."""
    return sum(
        math.dist(first, second)
        for first, second in zip(route, route[1:])
    )


def position_on_route(
    route: list[tuple[float, float]], distance_m: float
) -> tuple[float, float]:
    """Interpolate a position along a route at the requested travelled distance."""
    if not route:
        raise ValueError("Cannot find a position on an empty route")
    if distance_m <= 0:
        return route[0]

    remaining = distance_m
    for start, end in zip(route, route[1:]):
        segment_length = math.dist(start, end)
        if remaining <= segment_length:
            fraction = remaining / segment_length if segment_length else 0.0
            return (
                start[0] + (end[0] - start[0]) * fraction,
                start[1] + (end[1] - start[1]) * fraction,
            )
        remaining -= segment_length
    return route[-1]


def gps_from_local_offset(
    east_m: float,
    north_m: float,
    home_latitude: float = HOME_LATITUDE,
    home_longitude: float = HOME_LONGITUDE,
) -> tuple[float, float]:
    """Convert local east/north offsets to approximate GPS coordinates."""
    latitude = home_latitude + north_m / 111_320.0
    longitude_scale = 111_320.0 * math.cos(math.radians(home_latitude))
    if abs(longitude_scale) < 1e-6:
        raise ValueError("GPS offset is undefined near the poles")
    longitude = home_longitude + east_m / longitude_scale
    return latitude, longitude


def initial_crop_health(seed: int = 305) -> list[list[float]]:
    """Return a repeatable simulated crop-health raster, in percent."""
    rng = random.Random(seed)
    return [
        [
            round(
                min(
                    99.0,
                    max(
                        48.0,
                        84
                        + 8 * math.sin(row * 0.57)
                        + 5 * math.cos(column * 0.41)
                        + rng.uniform(-7, 7),
                    ),
                ),
                1,
            )
            for column in range(GRID_COLUMNS)
        ]
        for row in range(GRID_ROWS)
    ]


def advance_simulation(state: dict[str, Any], now: float | None = None) -> None:
    """Advance active flight by elapsed wall-clock time, bounded after app pauses."""
    current_time = time.monotonic() if now is None else now
    previous_time = float(state.get("last_tick", current_time))
    elapsed = max(0.0, min(current_time - previous_time, 3.0))
    state["last_tick"] = current_time
    if state.get("status") != "Flying" or elapsed == 0:
        return

    total_distance = float(state["route_length_m"])
    travelled = min(
        total_distance,
        float(state["distance_flown_m"]) + float(state["speed_m_s"]) * elapsed,
    )
    state["distance_flown_m"] = travelled
    state["progress"] = travelled / total_distance if total_distance > 0 else 1.0
    state["position_m"] = position_on_route(state["route"], travelled)
    state["altitude_m"] = float(state["commanded_altitude_m"])
    state["battery_percent"] = max(
        0.0, float(state["battery_percent"]) - elapsed * 0.025
    )

    if state["mission_type"] == SIMULATED_SPRAYING:
        _apply_simulated_spray(state)

    if state["progress"] >= 1.0:
        state["progress"] = 1.0
        state["status"] = "Mission complete"
        state["altitude_m"] = 0.0
        state["speed_m_s"] = 0.0


def _apply_simulated_spray(state: dict[str, Any]) -> None:
    """Visually mark cells near the simulated sprayer and estimate coverage."""
    x, y = state["position_m"]
    width = float(state["field_width_m"])
    length = float(state["field_length_m"])
    row = int((y + width / 2) / width * GRID_ROWS)
    column = int((x + length / 2) / length * GRID_COLUMNS)
    if 0 <= row < GRID_ROWS and 0 <= column < GRID_COLUMNS:
        crop_health = state["crop_health"]
        crop_health[row][column] = min(100.0, crop_health[row][column] + 0.3)

    field_area = width * length
    covered_area = min(
        field_area,
        float(state["distance_flown_m"]) * SPRAY_SWATH_M,
    )
    state["covered_area_m2"] = covered_area
    state["spray_used_l"] = covered_area * 0.0012


def _new_simulation_state() -> dict[str, Any]:
    return {
        "status": "Ready",
        "mission_type": FIELD_SURVEY,
        "route": [],
        "route_length_m": 0.0,
        "distance_flown_m": 0.0,
        "progress": 0.0,
        "position_m": (0.0, 0.0),
        "altitude_m": 0.0,
        "commanded_altitude_m": 18.0,
        "speed_m_s": 0.0,
        "cruise_speed_m_s": 7.0,
        "battery_percent": 100.0,
        "field_length_m": 240.0,
        "field_width_m": 160.0,
        "crop_health": initial_crop_health(),
        "covered_area_m2": 0.0,
        "spray_used_l": 0.0,
        "last_tick": time.monotonic(),
    }


def _get_simulation_state() -> dict[str, Any]:
    if "simulation" not in st.session_state:
        st.session_state.simulation = _new_simulation_state()
    return st.session_state.simulation


def _reset_simulation(
    state: dict[str, Any],
    mission_type: str,
    field_length_m: float,
    field_width_m: float,
    cruise_speed_m_s: float,
    altitude_m: float,
) -> None:
    """Reset telemetry and prepare the selected mission route."""
    fresh_state = _new_simulation_state()
    fresh_state.update(
        {
            "mission_type": mission_type,
            "route": build_mission_route(mission_type, field_length_m, field_width_m),
            "field_length_m": field_length_m,
            "field_width_m": field_width_m,
            "cruise_speed_m_s": cruise_speed_m_s,
            "commanded_altitude_m": altitude_m,
            "crop_health": initial_crop_health(),
            "last_tick": time.monotonic(),
        }
    )
    fresh_state["route_length_m"] = route_length_m(fresh_state["route"])
    fresh_state["position_m"] = fresh_state["route"][0]
    state.clear()
    state.update(fresh_state)


def _format_duration(seconds: float) -> str:
    """Format mission duration in minutes and seconds."""
    total_seconds = max(0, int(seconds))
    return f"{total_seconds // 60:02d}:{total_seconds % 60:02d}"


def _build_field_figure(state: dict[str, Any]) -> go.Figure:
    """Create the crop-health field map, route, and live drone marker."""
    length = float(state["field_length_m"])
    width = float(state["field_width_m"])
    x_values = [
        -length / 2 + index * length / (GRID_COLUMNS - 1)
        for index in range(GRID_COLUMNS)
    ]
    y_values = [
        -width / 2 + index * width / (GRID_ROWS - 1)
        for index in range(GRID_ROWS)
    ]
    figure = go.Figure()
    figure.add_trace(
        go.Heatmap(
            x=x_values,
            y=y_values,
            z=state["crop_health"],
            colorscale=[
                [0.0, "#b64c3b"],
                [0.45, "#e9bd55"],
                [0.72, "#8ebc69"],
                [1.0, "#277b56"],
            ],
            zmin=45,
            zmax=100,
            colorbar={"title": "Crop health", "ticksuffix": "%"},
            hovertemplate="East %{x:.0f} m<br>North %{y:.0f} m<br>Health %{z:.1f}%<extra></extra>",
        )
    )
    route = state["route"]
    if route:
        figure.add_trace(
            go.Scatter(
                x=[point[0] for point in route],
                y=[point[1] for point in route],
                mode="lines",
                name="Planned route",
                line={"color": "#edf4f0", "width": 2, "dash": "dot"},
                hoverinfo="skip",
            )
        )
        flown = position_on_route(route, float(state["distance_flown_m"]))
        remaining_distance = float(state["distance_flown_m"])
        travelled_route = [route[0]]
        for start, end in zip(route, route[1:]):
            segment_length = math.dist(start, end)
            if remaining_distance >= segment_length:
                travelled_route.append(end)
                remaining_distance -= segment_length
            else:
                if remaining_distance > 0:
                    fraction = remaining_distance / segment_length if segment_length else 0.0
                    travelled_route.append(
                        (
                            start[0] + (end[0] - start[0]) * fraction,
                            start[1] + (end[1] - start[1]) * fraction,
                        )
                    )
                break
        figure.add_trace(
            go.Scatter(
                x=[point[0] for point in travelled_route],
                y=[point[1] for point in travelled_route],
                mode="lines",
                name="Flown path",
                line={"color": "#45dc8a", "width": 4},
                hoverinfo="skip",
            )
        )
        figure.add_trace(
            go.Scatter(
                x=[flown[0]],
                y=[flown[1]],
                mode="markers",
                name="Simulated drone",
                marker={
                    "size": 17,
                    "color": "#18a7a1",
                    "line": {"color": "white", "width": 2},
                    "symbol": "triangle-up",
                },
                hovertemplate="Drone<br>East %{x:.1f} m<br>North %{y:.1f} m<extra></extra>",
            )
        )
    figure.update_layout(
        height=470,
        margin={"l": 12, "r": 12, "t": 18, "b": 12},
        paper_bgcolor="rgba(0,0,0,0)",
        plot_bgcolor="rgba(0,0,0,0)",
        font={"color": "#dce8e3", "family": "Inter, sans-serif"},
        legend={"orientation": "h", "y": 1.08, "x": 0},
        xaxis={"title": "East / west (m)", "showgrid": False, "zeroline": False},
        yaxis={
            "title": "North / south (m)",
            "showgrid": False,
            "zeroline": False,
            "scaleanchor": "x",
            "scaleratio": 1,
        },
        uirevision="agri-field",
    )
    return figure


def _render_flight_panel() -> None:
    """Render controls and live metrics; update flight at one-second intervals."""
    state = _get_simulation_state()
    advance_simulation(state)

    mission_choice = st.session_state.mission_choice
    field_length = float(st.session_state.field_length)
    field_width = float(st.session_state.field_width)
    cruise_speed = float(st.session_state.cruise_speed)
    commanded_altitude = float(st.session_state.commanded_altitude)

    if state["status"] not in {"Flying", "Paused"}:
        if (
            state["mission_type"] != mission_choice
            or state["field_length_m"] != field_length
            or state["field_width_m"] != field_width
            or not state["route"]
        ):
            _reset_simulation(
                state,
                mission_choice,
                field_length,
                field_width,
                cruise_speed,
                commanded_altitude,
            )

    top_left, top_right = st.columns([3, 1])
    with top_left:
        st.markdown("### Field operations")
        st.caption(
            f"{state['mission_type']} · {state['field_length_m']:.0f} × "
            f"{state['field_width_m']:.0f} m field"
        )
    with top_right:
        status = state["status"]
        status_class = "live" if status == "Flying" else "paused" if status == "Paused" else "ready"
        st.markdown(
            f'<div class="status-pill {status_class}"><span></span>{status}</div>',
            unsafe_allow_html=True,
        )

    start_col, pause_col, reset_col = st.columns(3)
    with start_col:
        start_label = "Resume mission" if state["status"] == "Paused" else "Start mission"
        if st.button(start_label, type="primary", width="stretch", key="start_flight"):
            if state["status"] in {"Ready", "Mission complete"}:
                _reset_simulation(
                    state,
                    mission_choice,
                    field_length,
                    field_width,
                    cruise_speed,
                    commanded_altitude,
                )
            state["mission_type"] = mission_choice if state["status"] != "Paused" else state["mission_type"]
            state["speed_m_s"] = cruise_speed
            state["commanded_altitude_m"] = commanded_altitude
            state["status"] = "Flying"
            state["last_tick"] = time.monotonic()
    with pause_col:
        if st.button(
            "Pause flight",
            width="stretch",
            disabled=state["status"] != "Flying",
            key="pause_flight",
        ):
            advance_simulation(state)
            state["status"] = "Paused"
            state["speed_m_s"] = 0.0
            state["last_tick"] = time.monotonic()
    with reset_col:
        if st.button("Reset simulation", width="stretch", key="reset_flight"):
            _reset_simulation(
                state,
                mission_choice,
                field_length,
                field_width,
                cruise_speed,
                commanded_altitude,
            )

    if state["status"] == "Flying":
        state["speed_m_s"] = cruise_speed
        state["commanded_altitude_m"] = commanded_altitude

    latitude, longitude = gps_from_local_offset(*state["position_m"])
    total_distance = float(state["route_length_m"])
    remaining_seconds = (
        (total_distance - float(state["distance_flown_m"])) / cruise_speed
        if cruise_speed > 0
        else 0.0
    )
    progress_columns = st.columns([4, 1])
    with progress_columns[0]:
        st.progress(
            min(1.0, max(0.0, float(state["progress"]))),
            text=f"Mission progress · {float(state['progress']) * 100:.0f}%",
        )
    with progress_columns[1]:
        st.markdown(
            f'<div class="eta-label">EST. REMAINING<br><b>{_format_duration(remaining_seconds)}</b></div>',
            unsafe_allow_html=True,
        )

    metric_columns = st.columns(5)
    metric_columns[0].metric("Altitude", f"{state['altitude_m']:.1f} m")
    metric_columns[1].metric("Ground speed", f"{state['speed_m_s']:.1f} m/s")
    metric_columns[2].metric(
        "Battery",
        f"{state['battery_percent']:.0f}%",
        delta="SIMULATED",
        delta_color="off",
    )
    metric_columns[3].metric(
        "GPS", f"{latitude:.6f}, {longitude:.6f}", help="Simulated coordinates"
    )
    if state["mission_type"] == SIMULATED_SPRAYING:
        metric_columns[4].metric("Area treated", f"{state['covered_area_m2']:.0f} m²")
    else:
        metric_columns[4].metric("Route flown", f"{state['distance_flown_m']:.0f} m")

    if state["mission_type"] == SIMULATED_SPRAYING:
        st.caption(
            f"Spray simulation only · {state['spray_used_l']:.2f} L estimated "
            f"application · no physical payload is controlled."
        )
    elif state["mission_type"] == CROP_MONITORING:
        mean_health = sum(map(sum, state["crop_health"])) / (
            GRID_ROWS * GRID_COLUMNS
        )
        st.caption(f"Crop monitoring · current simulated mean crop-health index: {mean_health:.1f}%")
    else:
        st.caption("Field survey · collecting a simulated field coverage route.")

    st.plotly_chart(
        _build_field_figure(state),
        width="stretch",
        config={"displayModeBar": False, "scrollZoom": True},
        key="agriculture_field_chart",
    )


def run_app() -> None:
    """Run the Streamlit agriculture-drone simulation."""
    st.set_page_config(
        page_title="AgriFlight · Drone Simulation",
        page_icon="🌱",
        layout="wide",
        initial_sidebar_state="expanded",
    )
    st.markdown(
        """
        <style>
        @import url('https://fonts.googleapis.com/css2?family=DM+Sans:wght@400;500;600;700&family=Manrope:wght@500;600;700;800&display=swap');
        :root { color-scheme: dark; }
        .stApp { background: #091510; color: #e5eee8; font-family: 'DM Sans', sans-serif; }
        [data-testid="stHeader"] { background: rgba(9, 21, 16, 0.86); }
        [data-testid="stSidebar"] { background: #0e1d16; border-right: 1px solid #20372a; }
        h1, h2, h3 { font-family: 'Manrope', sans-serif !important; letter-spacing: -0.035em; }
        .hero { padding: .25rem 0 1rem; }
        .eyebrow { color: #70d69a; font-size: .75rem; font-weight: 700; letter-spacing: .18em; text-transform: uppercase; }
        .hero h1 { font-size: clamp(2.1rem, 4vw, 3.4rem); margin: .3rem 0 0; }
        .hero p { color: #9db0a4; margin-top: .35rem; }
        .status-pill { display: inline-flex; align-items: center; gap: .5rem; padding: .45rem .8rem; border: 1px solid #294333; border-radius: 99px; background: #13251b; color: #bdd2c3; font-weight: 600; font-size: .84rem; float: right; margin-top: .3rem; }
        .status-pill span { width: .55rem; height: .55rem; display: inline-block; border-radius: 50%; background: #8c9b91; }
        .status-pill.live { color: #91edb2; border-color: #276340; }
        .status-pill.live span { background: #53d884; box-shadow: 0 0 12px #53d884; }
        .status-pill.paused span { background: #f0ba52; }
        .status-pill.ready span { background: #8c9b91; }
        .eta-label { color: #7f9687; text-align: right; font-size: .68rem; letter-spacing: .1em; padding-top: .15rem; }
        .eta-label b { color: #e6f0e9; font-size: 1.12rem; letter-spacing: 0; }
        [data-testid="stMetric"] { background: #102018; border: 1px solid #21392b; padding: .85rem 1rem; border-radius: .8rem; min-height: 104px; }
        [data-testid="stMetricLabel"] { color: #94a99a; }
        [data-testid="stMetricValue"] { font-family: 'Manrope', sans-serif; font-size: clamp(.95rem, 1.6vw, 1.4rem); }
        div.stButton > button { border-radius: .65rem; min-height: 2.65rem; font-weight: 600; }
        div.stButton > button[kind="primary"] { background: #2b9a5c; border-color: #2b9a5c; }
        [data-testid="stPlotlyChart"] { border: 1px solid #20372a; border-radius: .9rem; overflow: hidden; }
        @media (max-width: 700px) { .status-pill { float: none; } [data-testid="stMetric"] { padding: .65rem; } }
        </style>
        """,
        unsafe_allow_html=True,
    )

    with st.sidebar:
        st.markdown("## 🌱 AgriFlight")
        st.caption("SIMULATION CONTROL CENTER")
        st.divider()
        st.markdown("### Mission setup")
        st.selectbox("Operation", MISSION_TYPES, key="mission_choice")
        st.slider("Field length (m)", 100, 400, 240, step=20, key="field_length")
        st.slider("Field width (m)", 80, 240, 160, step=20, key="field_width")
        st.slider("Cruise speed (m/s)", 2.0, 12.0, 7.0, step=0.5, key="cruise_speed")
        st.slider("Flight altitude (m)", 5, 40, 18, key="commanded_altitude")
        st.divider()
        st.markdown("### Vehicle")
        st.markdown("**AG-01** &nbsp; · &nbsp; Multirotor")
        st.markdown("🔋 Electric propulsion &nbsp; · &nbsp; GPS enabled")
        st.caption("All movement, telemetry and spraying are simulated.")

    state = _get_simulation_state()
    if "mission_choice" not in st.session_state:
        st.session_state.mission_choice = state["mission_type"]
    if "field_length" not in st.session_state:
        st.session_state.field_length = int(state["field_length_m"])
    if "field_width" not in st.session_state:
        st.session_state.field_width = int(state["field_width_m"])
    if "cruise_speed" not in st.session_state:
        st.session_state.cruise_speed = float(state["cruise_speed_m_s"])
    if "commanded_altitude" not in st.session_state:
        st.session_state.commanded_altitude = int(state["commanded_altitude_m"])

    st.markdown(
        """
        <div class="hero">
          <div class="eyebrow">Precision agriculture · virtual flight deck</div>
          <h1>Field mission console</h1>
          <p>Plan a survey, monitor crop health, or simulate a precision spray pass.</p>
        </div>
        """,
        unsafe_allow_html=True,
    )
    st.info(
        "Simulation environment · No connection to real aircraft or physical spraying equipment.",
        icon="🛡️",
    )

    st.fragment(run_every="1s")(_render_flight_panel)()


if __name__ == "__main__":
    run_app()
