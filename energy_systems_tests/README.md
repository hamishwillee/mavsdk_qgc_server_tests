# Energy Systems Tests

A simulated vehicle for testing QGroundControl's support for the MAVLink
**Energy Systems Protocol** ([devguide PR #755](https://github.com/mavlink/mavlink-devguide/pull/755),
messages in [mavlink/mavlink#2526](https://github.com/mavlink/mavlink/pull/2526)).

The vehicle's energy systems, batteries and fuel tanks, how their levels change, and
how it answers the GCS's discovery/streaming requests are all defined in a
**scenario** YAML file. 22 scenarios ship in [`scenarios/`](scenarios/), covering the
documentation examples, every standard energy-system flag, user-defined systems, vehicles
with and without source (battery/fuel) messages, and the protocol's fallback and rejoin flows.

## Prerequisites

- Python 3.10+, Linux or WSL2 (`SO_REUSEPORT` is needed, as for `battery_tests`)
- QGroundControl listening on UDP 14550 (the default)

```bash
python3 -m venv .venv && . .venv/bin/activate
pip install -r requirements.txt
```

## Running

```bash
python3 energy_sim.py --list-scenarios
python3 energy_sim.py --scenario vtol_mc_fw            # name or path
python3 energy_sim.py --scenario all_standard --speed 10   # 10x faster: reach LOW/CRITICAL/EMERGENCY quickly
python3 energy_sim.py --scenario faults --quiet        # only protocol events + 5 s summary
python3 energy_sim.py --scenario my.yaml --check       # validate a scenario and exit
```

Options: `--port` (default 14550), `--speed N`, `--quiet`, `--mavlink-direct` (see below), `--autopilot generic|px4`
(the HEARTBEAT's `MAV_AUTOPILOT`; default `generic`, so that QGC uses its generic firmware
plugin and nothing PX4/ArduPilot specific gets involved).

The simulator logs every request it gets from QGC and its reply, e.g.:

```
[GCS-> REQUEST_MESSAGE] ENERGY_SYSTEM_INFO (id=365) instance=all  -> ACCEPTED
[-> ENERGY_SYSTEM_INFO  ] index=0 energy_systems=PROPULSION|AVIONICS
[GCS-> SET_MESSAGE_INTERVAL] ENERGY_SYSTEM_STATUS (id=366) 2 s (0.5 Hz)  -> ACCEPTED
[TIMELINE t=40s] es 0: add_flags=0x8 (transition to FW)
[t=   50s] ES0 76% [INACTIVE] | ES1 76% [0]  streams: ENERGY_SYSTEM_STATUS@2s, BATTERY_STATUS_V2@8s
```

## Scenarios

| Scenario | What it covers |
| --- | --- |
| `single_battery` | Docs example: one PROPULSION\|AVIONICS system, one battery |
| `dual_battery` | Docs example: separate propulsion and avionics systems, one battery each |
| `vtol_mc_fw` | Docs example: VTOL MC/FW systems sharing a battery, fuel tank for FW only; INACTIVE toggles every 40 s |
| `*_no_sources` | Each of the above with only `ENERGY_SYSTEM_*` (source requests NACKed) |
| `all_standard` | PROPULSION, AVIONICS, PAYLOAD (FLIGHT_CRITICAL set at 60 s) and a combined "all" system |
| `user_defined` | USER_DEFINED1-4: 25-byte unterminated name, UTF-8 names, empty name |
| `hybrid_fuel_battery` | LIQUID, GAS and UNKNOWN fuel types alongside batteries |
| `multi_source_parallel` | Same sources reported `weighted` (ArduPilot Sum-like) vs `min` (PX4-like) |
| `source_in_two_systems` | One battery listed under two systems |
| `source_zero_mask` | `energy_systems = 0` on a battery and a fuel tank |
| `orphan_source` | A source whose mask matches no system |
| `faults` | Battery/fuel fault flags on a timeline (drill-down only, not system severity) |
| `unavailable_system` | UNAVAILABLE static and toggled |
| `cell_status` | BATTERY_CELL_STATUS, 12S (two instances) and 4S |
| `nan_fields` | NaN / UINT16_MAX "not provided" values; severity without a percentage |
| `legacy_only` | ENERGY_SYSTEM_INFO UNSUPPORTED, legacy BATTERY_STATUS streams: GCS must fall back |
| `legacy_until_disabled` | Both protocols; BATTERY_STATUS stops when the GCS disables it |
| `sources_unsupported` | Sources exist but BATTERY_INFO/FUEL_INFORMATION_V2 are NACKed |
| `info_only` | Source _INFO sent, but source status streams NACKed |
| `rejoin` | Status streams already running before the GCS asks |
| `smart_battery` | Smart battery on component 180 (same battery id as the autopilot's monitor) streams BATTERY_STATUS_V2 unprompted; GCS must request its BATTERY_INFO from component 180 |
| `smart_battery_heartbeat` | Smart battery on component 181 that only sends a `MAV_TYPE_BATTERY` HEARTBEAT; GCS must find it from the heartbeat |

Each scenario's `description` says what QGC should show.

## Scenario format

```yaml
name: example
description: |
  What this tests and what QGC should show.

behaviour:                        # optional; defaults follow the documented flow
  energy_systems: supported       # supported | unsupported (-> legacy fallback)
  sources: supported              # supported | unsupported | info_only (default for both kinds)
  batteries: supported            #   ... or set per kind
  fuel: supported
  legacy_battery_status: off      # off | on (always; disable is DENIED) | until_disabled
  stream_by_default:              # list of names, or name: interval (rejoin case)
    ENERGY_SYSTEM_STATUS: 1s
  fallback_timeout: 5s            # see "Fallback" below; off to disable

energy_systems:
  - index: 0
    flags: [PROPULSION, AVIONICS] # MAV_ENERGY_SYSTEM_FLAGS names (prefix optional) or ints
    name: ""                      # energy_system_name, <= 25 bytes UTF-8
    aggregate: min                # min (default) | weighted | explicit
    profile: {start: 95, end: 5, period: 3m, loop: true}   # explicit only; or a constant, e.g. 50
    thresholds: {low: 25, critical: 15, emergency: 5}       # % for LOW/CRITICAL/EMERGENCY
    status_flags: [FLIGHT_CRITICAL]   # static MAV_ENERGY_STATUS_FLAGS
    endurance: 20m                # remaining_time = endurance * level; omit -> NaN
    report_percent: true          # false -> remaining_capacity NaN
    component: 1                  # reporting MAVLink component (default 1, the autopilot)

sources:
  batteries:
    - id: 0
      energy_systems: [PROPULSION, AVIONICS]
      capacity_ah: 5.0
      cells: 4
      current_a: 15
      profile: {start: 95, end: 5, period: 3m}
      status_flags: [TRACKS_CHARGE]   # MAV_BATTERY_STATUS_FLAGS
      cell_status: false              # also stream BATTERY_CELL_STATUS
      report_level: true              # false -> state_of_charge UINT16_MAX
      weight: 5.0                     # for `weighted` (default capacity_ah)
      info: {name: Acme_X, battery_function: AVIONICS}   # BATTERY_INFO overrides
      component: 1                    # e.g. 180 for a smart battery on its own component
  fuel:
    - id: 0
      energy_systems: [PROPULSION]
      fuel_type: LIQUID               # LIQUID (ml) | GAS (kPa) | UNKNOWN (normalized)
      maximum_fuel: 20000
      flow_rate: 12.5                 # omit to derive from the profile; .nan for NaN
      temperature_k: 293.15
      profile: {start: 100, end: 10, period: 4m}
      status_flags: []                # MAV_FUEL_STATUS_FLAGS
      report_level: true

components:                       # optional: behaviour of components other than 1
  180: {heartbeat: false, stream_status: true}   # MAV_TYPE_BATTERY heartbeat; stream status unprompted

timeline:                         # optional changes at simulation times (scaled by --speed)
  - at: 40s
    repeat: 80s                   # optional
    description: transition to FW
    set:                          # one change or a list; target es/battery/fuel by index/id
      - {es: 0, add_flags: [INACTIVE]}
      - {es: 1, clear_flags: [INACTIVE]}
      - {battery: 0, status_flags: [FAULT_OVER_TEMPERATURE]}
      - {battery: 0, component: 180, add_flags: [CHARGING]}   # component needed if the id is on several
```

### Components

Energy systems and sources are keyed by (component, index/id), because ids only have to be
unique within a component: a smart battery may use id 0 just like the autopilot's power monitor.
Component 1 is the autopilot that the GCS runs the connection flow against, and `behaviour:`
applies to it. Any other component acts as an independent device:
- It streams its own status messages from start-up, unless `stream_status: false`.
- It optionally sends a `MAV_TYPE_BATTERY` HEARTBEAT.
- It answers `REQUEST_MESSAGE` for its own `_INFO` messages, with ACKs sent from that component.
- It ignores everything else.

A request addressed to a component that the scenario doesn't have gets no reply.

Timeline keys: `status_flags` (replace), `add_flags`, `clear_flags`, `profile` (restarts
from that time; makes an energy system `explicit`), and for energy systems
`auto_severity`, `report_percent`, `endurance`; for batteries `current_a`; for fuel `flow_rate`.

### How energy-system values are produced

The protocol leaves the algorithm to the emitter, so each system chooses one:

- `min`: the lowest level of its matching sources. PX4 does this across batteries: lowest
  remaining, worst warning, lowest time.
- `weighted`: Σ(level × weight) / Σ weight, i.e. pooled capacity. ArduPilot's Sum battery
  monitor works this way.
- `explicit`: the system's own `profile`. This is required when the system has no sources.

A source matches a system when `source.energy_systems & system.flags != 0`. LOW, CRITICAL
and EMERGENCY are set from `thresholds` (only one at a time). Systems with INACTIVE set
never get a severity flag. Use `auto_severity: false` plus `status_flags` to control
severity by hand.

## Protocol behaviour

- `MAV_CMD_REQUEST_MESSAGE` for any of the energy messages: ACK, then one message per
  instance. `param2 = 0` means all instances (the docs say "all"); any other value requests
  that id (FAILED if it doesn't exist). The reply is UNSUPPORTED if `behaviour` says so or
  the vehicle has no instances of that message.
- `MAV_CMD_SET_MESSAGE_INTERVAL` starts, re-rates or (with -1) stops that message's stream
  for all instances.
- **Fallback.** If an `_INFO` message was requested but its status stream wasn't requested
  within `fallback_timeout`, the simulator starts the stream at the documented rate and
  logs `[FALLBACK]`. This never happened with QGC or `fake_gcs.py`: the GCS's commands
  reach the simulator. It is a safety net for a GCS that doesn't request status streams.

## Checking the simulator itself

`tools/fake_gcs.py` is a minimal stand-in for QGC. It binds the GCS port like QGC's UDP link,
runs the documented connection flow, and checks the results against the scenario. It decodes
using a pymavlink dialect generated from `xml/energy_messages.xml`, so it is independent of
mavsdk_server and also confirms the messages are wire-compatible with a GCS built from the
same XML (CRC_EXTRA, field order, extension fields, UTF-8 names).

```bash
python3 -m unittest discover -s tests       # offline unit tests (model, scenarios, protocol)
python3 tools/check_scenarios.py            # every scenario: sim + fake GCS (~5 min). Close QGC first.
python3 tools/check_scenarios.py faults --keep-logs   # logs in .generated/logs/
```

## Testing QGC

The QGC implementation is on branch [`energy-systems`](https://github.com/hamishwillee/qgroundcontrol/tree/energy-systems) of `hamishwillee/qgroundcontrol`.
Build it, start it, then run a scenario. Use `--speed 10` to see the alerts quickly. For the
QGC side of the protocol flow, start QGC with `--logging:Vehicle.EnergySystemManager`.

Verified end to end with that build (`vtol_mc_fw --speed 10`):
- QGC requests ENERGY_SYSTEM_INFO, then streams ENERGY_SYSTEM_STATUS at 2 s.
- It disables BATTERY_STATUS.
- It requests BATTERY_INFO and FUEL_INFORMATION_V2, then their status streams at 8 s.
- It announces "warning, Hover propulsion energy critical" and similar as severity rises.
- All QGC commands reached the simulator, so no `[FALLBACK]` was needed.

## How messages are sent

All telemetry is packed with pymavlink and sent from the responder's `ka` socket as sysid 1,
from the component each item is on (1 by default). It shares one sequence numbering with the
heartbeats and command ACKs. MAVSDK/mavsdk_server is still used for the vehicle connection
(bootstrap and routing of the GCS's commands), but **not** for telemetry, because:

- mavsdk_server sends MavlinkDirect messages from its own component id (191) and ignores
  `MavlinkMessage.component_id`. A scenario couldn't then control which component reports
  what, and `_INFO` replies would come from a different component than the request was
  addressed to (so QGC's request handling wouldn't match them). `tools/fake_gcs.py` checks
  that every instance comes from its scenario component.
- mavsdk_server's built-in `BATTERY_INFO` lacks the new `energy_systems` extension, and
  `load_custom_xml()` doesn't replace it (libmav: `Failed to set unsigned integer field
  energy_systems`).

`--mavlink-direct` restores MavlinkDirect sending for comparison.

## Known limitations / gaps

- `remaining_time` is `endurance × level`. It is not modelled from current draw.
- `FUEL_STATUS_V2.flow_rate`, when derived from the profile, is in `maximum_fuel` units per
  second. For GAS that is kPa/s, not the ml/s the XML specifies; set `flow_rate`
  explicitly if it matters.
- Values are mostly self-consistent but not physically modelled: voltage is linear in level,
  and current is constant.
- At high `--speed`, the VTOL scenarios toggle INACTIVE every few seconds. QGC resets an
  inactive system's alert state, so it re-announces the current level each time the system
  becomes active again (by design).
