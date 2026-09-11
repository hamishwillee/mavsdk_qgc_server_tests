# Condition Gate Tests

Uploads a mission containing `MAV_CMD_CONDITION_GATE` to a real running PX4 instance
(SIH -- Simulation In Hardware, PX4's built-in physics sim, no external simulator
needed) so its behaviour can be observed live in QGroundControl.

Unlike `battery_tests`, this does not fake a vehicle: it drives an actual PX4
flight stack, because `CONDITION_GATE` needs real navigator logic (EKF, mission
state machine, position control) to produce anything to observe.

## What is tested

A 7-item mission:

| # | Command                  | Purpose                                                            |
| - | ------------------------- | ------------------------------------------------------------------- |
| 0 | `NAV_TAKEOFF`             | climb to mission altitude at the home position                      |
| 1 | `NAV_WAYPOINT` (close)    | 15 m north of home                                                   |
| 2 | `CONDITION_GATE`          | 20 m *off* the WP1 -> WP2 line (not on it -- see below)              |
| 3 | `DO_SET_ROI_LOCATION`     | visual marker ~8 m from the gate (map pin only, not part of the mission flow) |
| 4 | `DO_CHANGE_SPEED`         | slow to 1.5 m/s -- fires once the gate is marked reached             |
| 5 | `NAV_WAYPOINT` (far)      | ~15 s flight east of WP1 (straight line from WP1)                    |
| 6 | `NAV_RETURN_TO_LAUNCH`    | return home after WP2                                                |

The gate is deliberately placed off to the side of the flight path, not on it,
to make clear that the vehicle does not need to fly over the gate's
coordinates -- only to cross the plane through it (see below). The
`DO_SET_ROI_LOCATION` item exists purely so QGC renders a visible pin near
the gate's position (the gate's own Plan-view icon is easy to miss/generic).
It carries no navigational meaning here and is skipped by the same
next-position lookahead described below (`mission_item_contains_position()`
does not count `DO_*` commands), so it has no effect on the flight path or
on when the gate is judged "reached".

`MAV_CMD_CONDITION_GATE` params, per `common.xml`:

- param1 `Geometry` = 0 (only defined value)
- param2 `UseAltitude` = 0 / `MAV_BOOL_FALSE` -- PX4 only implements the 2D
  (horizontal) gate-crossing check; altitude is ignored
- param3, param4 = `NaN` (undefined/reserved)
- param5, param6, param7 = gate latitude, longitude, altitude

### The gate does not bend the flight path

The first version of this test placed the gate off to the side (WP1/gate/WP2
as a right-angle triangle), expecting the vehicle to fly to the gate and then
turn. **It doesn't.** Flying the mission showed the vehicle going straight
from WP1 to WP2, skipping the gate's position entirely.

The reason is in PX4's navigator, `src/modules/navigator/mission.cpp`
(`Mission::set_mission_items()`):

```cpp
} else if (item_contains_gate(_mission_item)) {
    // The mission item is a gate, let's check if the next item in the list provides
    // a position to go towards.
    if (num_found_items > 0u) {
        mission_item_to_position_setpoint(next_mission_items[0u], &pos_sp_triplet->current);
    }
    if (num_found_items >= 2u) {
        mission_item_to_position_setpoint(next_mission_items[1u], &pos_sp_triplet->next);
    }
```

When the *current* mission item is a gate, the navigator sets the flight
target straight to the *next real waypoint* -- never to the gate's own
lat/lon. The gate is not a navigation target; the vehicle was always going to
fly a straight line from WP1 to WP2 no matter where the gate sits.

What the gate actually does is gate the **mission sequence index**: PX4 keeps
item 2 (the gate) "current" -- delaying advancement to item 3 -- until the
vehicle's position, projected relative to the gate, crosses a plane through
the gate oriented toward the next waypoint (the dot-product check in
`MissionBlock::is_mission_item_reached_or_completed()`, `mission_block.cpp`).
That's what "Delay mission state machine until gate has been reached" in
`common.xml` means. It exists to let a `DO_*` mission item that follows the
gate (change speed, trigger camera, set ROI, ...) fire at a precise
line-crossing mid-flight, without needing its own waypoint.

This test demonstrates exactly that: item 3 (`DO_CHANGE_SPEED`) only executes
once the gate is satisfied, so the vehicle visibly slows down partway through
the straight WP1 -> WP2 leg -- at the point it crosses the gate's plane, not
at either waypoint. That speed drop is the observable signature of the gate
working; the flight path itself never deviates.

## A PX4 firmware bug this test uncovered

As of this writing, PX4's `mavlink_command_params.hpp` mission-upload validator
rejects `CONDITION_GATE` items that have a location set at all:

```cpp
{ 4501, 0x00, 0x00 }, // CONDITION_GATE:              no params used by PX4
```

The `0x00` mask says none of param1-7 may be set, but `common.xml` defines
param5-7 as the gate's lat/lon/alt, and PX4's own navigator
(`mission_block.cpp`) reads `_mission_item.lat`/`lon` to compute the gate
plane. Uploading a gate with a location returns `MISSION_ACK` type 10
(`MAV_MISSION_INVALID_PARAM5_X`). The row should be `0x73` (param1, param2,
param5-7 allowed), matching the pattern of the neighbouring
`NAV_FENCE_RETURN_POINT` (`0x70`) entry.

Running this test against unpatched PX4 requires a local, uncommitted one-line
fix to that table (and a rebuild) -- see git history / diff in your local
PX4-Autopilot checkout. Consider reporting this upstream.

## Prerequisites

- A local PX4-Autopilot checkout with the `px4_sitl_sih` target built
  (`make px4_sitl_sih` from the PX4-Autopilot root), including the
  `mavlink_command_params.hpp` fix above.
- Python 3.10+, `pymavlink` (see `requirements.txt`).
- QGroundControl (any build capable of connecting over UDP).

Install dependencies:

```bash
pip install -r requirements.txt
```

## How to run

1. Start PX4 SIH (quadcopter airframe) in one terminal:

   ```bash
   cd <PX4-Autopilot>/build/px4_sitl_sih
   PX4_SIMULATOR=sihsim PX4_SIM_MODEL=sihsim_quadx ./bin/px4 -i 0 -d "$PWD"
   ```

   Wait for `Ready for takeoff!` in the log.

2. Upload the mission:

   ```bash
   python3 upload_condition_gate_mission.py
   ```

   This only uploads the mission -- it does not arm or start it, so you can
   connect QGC first and watch the whole flight including takeoff.

3. Connect QGroundControl and start the mission from the Fly view (see the
   root-level instructions the assistant gave you for connection details).

## Why raw MAVLink (pymavlink) instead of MAVSDK

MAVSDK's `mission_raw` plugin was tried first and rejected the mission
client-side (`INVALID_ARGUMENT`) before anything reached PX4: `mavsdk_server`
has its own internal command table and doesn't recognise `MAV_CMD_CONDITION_GATE`
(4501) since it's marked `<wip/>` in `common.xml`. `pymavlink` has no such
allow-list -- `MISSION_ITEM_INT.command` is just a `uint16` on the wire -- so
it carries the command through untouched. This script builds and uploads the
mission with the standard `MISSION_COUNT` / `MISSION_REQUEST_INT` /
`MISSION_ITEM_INT` / `MISSION_ACK` handshake directly.

## Known gotchas found while building this test

- **RTL item must use `MAV_FRAME_MISSION` (2), not a global frame.** PX4's
  mission item parser has a separate switch statement for "location-less"
  commands (`MAV_FRAME_MISSION`) vs. GPS-frame commands; `NAV_RETURN_TO_LAUNCH`
  is only handled in the former. Sending it with `MAV_FRAME_GLOBAL_RELATIVE_ALT_INT`
  falls through to `default:` and PX4 rejects the whole mission with
  `MAV_MISSION_UNSUPPORTED`.
- **Unused `x`/`y` (int32) fields need the sentinel value, not 0.** PX4's
  param-mask validator treats float params `0.0` as an acceptable "unused"
  sentinel (matching common GCS behaviour), but for `MISSION_ITEM_INT`'s
  int32 `x`/`y` fields only `INT32_MAX`/`INT32_MIN` count as unused --
  literal `0` is treated as a real (and, for masked-out params, invalid)
  value.
