# AgriFlight · Agriculture Drone Simulation

A Streamlit dashboard prototype for simulated agricultural drone missions.
It visualizes field survey, crop monitoring, and simulated spraying missions,
with live flight controls, telemetry, mission progress, and crop-health maps.

**Simulation only:** the app does not connect to or control real drone
hardware. All flight, GPS, battery, crop-health changes, and spray coverage
are simulated.

## Local setup

Python 3.12 is recommended (Python 3.10+ is supported):

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
pip install -r requirements.txt
streamlit run drone.py
```

Open the local URL printed by Streamlit, normally <http://localhost:8501>.
The app refreshes the flight panel every second while keeping simulation state
in the current Streamlit session.
On macOS/Linux, use `python3.12 -m venv .venv` and
`source .venv/bin/activate`.

Run the repository tests with:

```powershell
python -m pytest -q
```

## Dashboard

- Select a field survey, crop monitoring, or simulated spraying mission.
- Set field dimensions, cruise speed, and altitude in the sidebar.
- Start, pause/resume, and reset the simulated flight.
- Monitor altitude, speed, battery, simulated GPS, route progress, and coverage.
- Inspect the interactive crop-health field chart and simulated flight path.

## Deploy to Streamlit Community Cloud

Create a new app from this repository with:

| Setting | Value |
| --- | --- |
| Repository | `sharmamehul305-ux/hardware` |
| Branch | `main` |
| Main file path | `drone.py` |
| Python version | `3.12` |

Streamlit installs the packages listed in the root `requirements.txt`.
