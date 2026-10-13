# Multi-Drone Fleet Control

Async Python ground-control service using MAVSDK-Python, FastAPI, WebSockets,
and a Leaflet dashboard. It supports simultaneous drone connections, per-drone
or all-drone commands, waypoint missions, basic formations, telemetry logging,
and geofence/battery/link safety checks.

The sample configuration selects PX4. Set `autopilot: ArduPilot` when using
ArduPilot; the MAVSDK communication layer remains the same, but simulator
MAVLink endpoints and mission/action behavior must be verified for the
particular ArduPilot vehicle and firmware.

> **Safety notice:** This is example software, not a certified flight-control
> system. SITL-test every change and use a qualified pilot, a controlled test
> area, and independent RC/emergency procedures. Minimum-separation checks
> report warnings and do not replace collision avoidance. Configure a valid
> home point before flight.

## Requirements

- Python 3.10+
- PX4 SITL and QGroundControl for the default simulator workflow
- Windows, Linux, or macOS

## Install

From the project directory:

```powershell
py -3.10 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
pip install -r requirements.txt
```

For Linux/macOS, activate with `source .venv/bin/activate`.

## Run PX4 SITL

Start PX4 SITL using a supported simulator such as Gazebo, and ensure MAVLink
UDP connections are available on ports `14540` and `14541`. This application
does not launch PX4 itself; it connects to an already-running simulator.
Configure each simulated vehicle to publish to its corresponding UDP endpoint
in [config.yaml](./config.yaml). For a single drone, remove the second drone
entry or start another PX4 instance on the second port.

## Run the ground-control application

```powershell
python server.py
```

Open <http://127.0.0.1:8000>. Startup attempts all configured connections in
parallel and reports failures without preventing connected drones from being
used. The dashboard displays positions, GPS health, altitude, speed, battery,
and flight mode. Commands are available for all drones or one selected drone.

The dashboard uses Leaflet and OpenStreetMap tiles from public CDN endpoints,
so map tiles require internet access.

## Real hardware

Real endpoints are intentionally unset in the sample config. Add the correct
`real_system_address` for each vehicle and run:

```powershell
python server.py --real
```

`--real` is required to select those addresses. Never use simulator UDP
addresses for real hardware. Verify autopilot parameters, failsafe behavior,
geofence/home, telemetry, and emergency procedures before arming.

## Commands and missions

- Arm, takeoff, go-to, land, and return-to-launch can target all or one drone.
- Fleet takeoff uses `takeoff_stagger_m` for increasing altitudes and rejects
  any target beyond configured limits.
- Formations supported: `line`, `grid`, `circle`, `v`.
- Emergency Land All requires browser confirmation and then issues land
  commands; it does **not** cut motor power.
- Mission JSON is either a list of waypoint objects or
  `{"waypoints": [...]}`. Each waypoint has `latitude_deg`, `longitude_deg`,
  and `altitude_m`; optional keys are `speed_m_s`, `is_fly_through`,
  `gimbal_pitch_deg`, and `gimbal_yaw_deg`. Waypoints are checked against the
  geofence, uploaded, and started through MAVSDK.

Run tests:

```powershell
python -m pytest -q
```

## Safety and logs

`config.yaml` defaults to a 200 m home-centred radius, 50 m relative-altitude
maximum, 5 m separation warning, RTL below 25% battery, and land after a link
loss lasting 5 seconds. Out-of-fence and over-altitude commands are rejected.
Separation is warning-only: simultaneous movements and actual vehicle dynamics
can still create collision risk.

Telemetry is appended to `logs/telemetry.csv`, commands to
`logs/commands.csv`, and application events to `fleet.log`. The dashboard
streams a state snapshot on connection and updates at approximately one-second
intervals.
