#!/usr/bin/env python3
"""
upload_condition_gate_mission.py - Build and upload a mission containing
MAV_CMD_CONDITION_GATE to a running PX4 instance, using raw MAVLink mission
protocol messages via pymavlink.

Requires: pip install pymavlink

Usage:
  python3 upload_condition_gate_mission.py                     # connect to udp://:14540 (PX4 SITL/SIH default)
  python3 upload_condition_gate_mission.py --port 14540
  python3 upload_condition_gate_mission.py --alt 30 --speed 5   # override altitude / leg speed

This script only uploads the mission -- it does NOT arm or start it. Start
the mission from QGroundControl once it is connected, so the whole flight
(including takeoff) can be observed.

Why pymavlink and not MAVSDK's mission_raw plugin
--------------------------------------------------
MAVSDK's bundled mavsdk_server rejects MAV_CMD_CONDITION_GATE (4501) client
side: it isn't in mavsdk_server's own command table (the command carries a
<wip/> tag in common.xml), so upload_mission() fails with INVALID_ARGUMENT
before anything is even sent to the vehicle. pymavlink has no such
allow-list -- MISSION_ITEM_INT.command is just a uint16 on the wire -- so it
can carry MAV_CMD_CONDITION_GATE through untouched.
"""

import argparse
import math

from pymavlink import mavutil

# ---------------------------------------------------------------------------
# MAV_CMD / MAV_FRAME ids used in this mission
# ---------------------------------------------------------------------------
MAV_CMD_NAV_WAYPOINT            = 16
MAV_CMD_NAV_TAKEOFF             = 22
MAV_CMD_NAV_RETURN_TO_LAUNCH    = 20
MAV_CMD_CONDITION_GATE          = 4501   # WIP command, common.xml
MAV_CMD_DO_CHANGE_SPEED         = 178
MAV_CMD_DO_SET_ROI_LOCATION     = 195

SPEED_TYPE_GROUNDSPEED = 1

MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6
MAV_FRAME_MISSION                 = 2    # "not a coordinate frame" -- used for location-less items (e.g. RTL)
MAV_MISSION_TYPE_MISSION          = 0
MAV_MISSION_ACCEPTED              = 0

NAN = float('nan')
INT32_UNSET = 2147483647   # PX4's "param not used" sentinel for MISSION_ITEM_INT x/y (see mavlink_command_params.hpp)
EARTH_RADIUS_M = 6378137.0


def offset_latlon(lat_deg: float, lon_deg: float, north_m: float, east_m: float) -> tuple[float, float]:
    """Flat-earth offset (fine for the tens-of-metres distances used here)."""
    dlat = north_m / EARTH_RADIUS_M
    dlon = east_m / (EARTH_RADIUS_M * math.cos(math.radians(lat_deg)))
    return lat_deg + math.degrees(dlat), lon_deg + math.degrees(dlon)


def waypoint_item(seq: int, lat: float, lon: float, alt_rel: float, current: bool = False) -> tuple:
    return (
        seq, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, MAV_CMD_NAV_WAYPOINT,
        int(current), 1,
        0.0, 0.0, 0.0, NAN,               # hold time, accept radius (0=default), pass radius, yaw
        int(round(lat * 1e7)), int(round(lon * 1e7)), alt_rel,
        MAV_MISSION_TYPE_MISSION,
    )


def gate_item(seq: int, lat: float, lon: float, alt_rel: float) -> tuple:
    """MAV_CMD_CONDITION_GATE.

    param1 Geometry     = 0   (only defined value: plane orthogonal to the path)
    param2 UseAltitude  = 0   (MAV_BOOL_FALSE -- PX4 only implements the 2D/horizontal
                                gate-crossing check; altitude is ignored)
    param3, param4      = NaN (undefined / reserved)
    param5, param6, param7 = lat, lon, altitude of the gate
    """
    return (
        seq, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, MAV_CMD_CONDITION_GATE,
        0, 1,
        0.0, 0.0, NAN, NAN,
        int(round(lat * 1e7)), int(round(lon * 1e7)), alt_rel,
        MAV_MISSION_TYPE_MISSION,
    )


def roi_item(seq: int, lat: float, lon: float, alt_rel: float) -> tuple:
    """MAV_CMD_DO_SET_ROI_LOCATION -- placed near the gate purely so QGC renders
    a visible marker there (the gate item's own icon is easy to miss / generic).
    It is not a flight target: mission_item_contains_position() in PX4's
    navigator only counts NAV_WAYPOINT/LOITER*/LAND/TAKEOFF, so this is
    transparently skipped by the gate's and waypoints' "next position" lookup
    and has no effect on the flight path or timing.

    Mission-item mask for this command only allows param5-7 (lat/lon/alt);
    param1 (gimbal device id) is a COMMAND_LONG-only field here.
    """
    return (
        seq, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, MAV_CMD_DO_SET_ROI_LOCATION,
        0, 1,
        NAN, NAN, NAN, NAN,
        int(round(lat * 1e7)), int(round(lon * 1e7)), alt_rel,
        MAV_MISSION_TYPE_MISSION,
    )


def takeoff_item(seq: int, lat: float, lon: float, alt_rel: float) -> tuple:
    return (
        seq, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, MAV_CMD_NAV_TAKEOFF,
        1, 1,
        0.0, 0.0, 0.0, NAN,
        int(round(lat * 1e7)), int(round(lon * 1e7)), alt_rel,
        MAV_MISSION_TYPE_MISSION,
    )


def change_speed_item(seq: int, speed_mps: float) -> tuple:
    """MAV_CMD_DO_CHANGE_SPEED, placed right after the gate.

    This is the observable proof that the gate was "reached": PX4 only issues
    this command once the mission sequence advances past the gate item, which
    (per mission.cpp) happens while the vehicle is already flying a straight
    line from the previous waypoint to the next one -- the gate never becomes
    a navigation target itself. Watch the vehicle's speed drop mid-leg, at the
    point it crosses the gate's plane, not at either waypoint.
    """
    return (
        seq, MAV_FRAME_MISSION, MAV_CMD_DO_CHANGE_SPEED,
        0, 1,
        float(SPEED_TYPE_GROUNDSPEED), speed_mps, -1.0, 0.0,
        INT32_UNSET, INT32_UNSET, NAN,
        MAV_MISSION_TYPE_MISSION,
    )


def rtl_item(seq: int) -> tuple:
    return (
        seq, MAV_FRAME_MISSION, MAV_CMD_NAV_RETURN_TO_LAUNCH,
        0, 1,
        NAN, NAN, NAN, NAN,
        INT32_UNSET, INT32_UNSET, NAN,
        MAV_MISSION_TYPE_MISSION,
    )


def get_cruise_speed_mps(conn, fallback: float, timeout: float = 3.0) -> float:
    conn.mav.param_request_read_send(conn.target_system, conn.target_component, b'MPC_XY_CRUISE', -1)
    msg = conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=timeout)
    if msg is None or msg.param_id.rstrip('\x00') != 'MPC_XY_CRUISE':
        print(f"  [param] Could not read MPC_XY_CRUISE in time; using fallback {fallback} m/s", flush=True)
        return fallback
    return float(msg.param_value)


def upload_mission(conn, items: list) -> bool:
    tsys, tcomp = conn.target_system, conn.target_component
    conn.mav.mission_count_send(tsys, tcomp, len(items), MAV_MISSION_TYPE_MISSION)

    requested = set()
    for _ in range(2 * len(items) + 5):
        msg = conn.recv_match(type=['MISSION_REQUEST_INT', 'MISSION_REQUEST', 'MISSION_ACK'], blocking=True, timeout=5)
        if msg is None:
            print("  Timed out waiting for the vehicle during mission upload.")
            return False

        if msg.get_type() == 'MISSION_ACK':
            if msg.type == MAV_MISSION_ACCEPTED:
                return True
            print(f"  Mission upload rejected: MAV_MISSION_RESULT={msg.type}")
            return False

        seq = msg.seq
        requested.add(seq)
        conn.mav.mission_item_int_send(tsys, tcomp, *items[seq])

    print("  Mission upload did not complete (no ACK received).")
    return False


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--port', type=int, default=14540,
                         help='Local UDP port to listen on for the PX4 "onboard/API" mavlink link '
                              '(default: 14540, PX4 SITL/SIH default)')
    parser.add_argument('--alt', type=float, default=30.0,
                         help='Relative altitude (m) for all mission items (default: 30)')
    parser.add_argument('--speed', type=float, default=None,
                         help='Override cruise speed (m/s) used to size the ~15 s legs '
                              '(default: read MPC_XY_CRUISE from the vehicle)')
    args = parser.parse_args()

    print(f"Listening for PX4 on udp:0.0.0.0:{args.port} ...", flush=True)
    conn = mavutil.mavlink_connection(f'udpin:0.0.0.0:{args.port}')
    conn.wait_heartbeat(timeout=15)
    print(f"Connected to system {conn.target_system} component {conn.target_component}.", flush=True)

    print("Requesting HOME_POSITION ...", flush=True)
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION, 0, 0, 0, 0, 0, 0,
    )
    home = conn.recv_match(type='HOME_POSITION', blocking=True, timeout=10)
    if home is None:
        print("Did not receive HOME_POSITION -- is the vehicle's GPS/EKF ready?")
        return
    home_lat = home.latitude / 1e7
    home_lon = home.longitude / 1e7
    print(f"Home position: lat={home_lat:.7f} lon={home_lon:.7f} alt={home.altitude / 1000.0:.1f} m AMSL",
          flush=True)

    cruise_speed = args.speed if args.speed else get_cruise_speed_mps(conn, fallback=5.0)
    leg_m = cruise_speed * 15.0
    print(f"Cruise speed = {cruise_speed:.2f} m/s -> ~15 s leg = {leg_m:.1f} m", flush=True)

    # WP1: close-by waypoint, 15 m north of home.
    wp1_lat, wp1_lon = offset_latlon(home_lat, home_lon, north_m=15.0, east_m=0.0)

    # WP2: ~15 s flight east of WP1. The flight path is always the straight
    # line WP1 -> WP2 -- PX4's navigator never flies *to* the gate itself
    # (see the "gate does not bend the flight path" note in the README) -- so
    # this is fixed before the gate is placed.
    wp2_lat, wp2_lon = offset_latlon(wp1_lat, wp1_lon, north_m=0.0, east_m=leg_m)

    # Gate: offset OFF the WP1->WP2 line (not on it), roughly at the line's
    # midpoint, 20 m to the side. The crossing test PX4 uses is a plane
    # through the gate's own position, oriented perpendicular to the
    # gate -> next-waypoint (WP2) direction -- it does not require the
    # vehicle to actually fly over the gate's coordinates, only to cross that
    # plane, which a straight WP1->WP2 flight still does even with the gate
    # sitting off to the side.
    gate_offset_m = 20.0
    gate_lat, gate_lon = offset_latlon(wp1_lat, wp1_lon, north_m=gate_offset_m, east_m=leg_m / 2.0)

    # ROI marker: a few metres from the gate, purely so QGC shows a visible
    # pin near it (the gate's own Plan-view icon is easy to miss/generic).
    # Not part of the mission's control flow -- see roi_item() docstring.
    roi_lat, roi_lon = offset_latlon(gate_lat, gate_lon, north_m=0.0, east_m=8.0)

    alt = args.alt
    slow_speed = 1.5  # m/s -- dramatically slower than cruise, to make the change obvious

    items = [
        takeoff_item(0, home_lat, home_lon, alt),
        waypoint_item(1, wp1_lat, wp1_lon, alt),
        gate_item(2, gate_lat, gate_lon, alt),
        roi_item(3, roi_lat, roi_lon, alt),
        change_speed_item(4, slow_speed),
        waypoint_item(5, wp2_lat, wp2_lon, alt),
        rtl_item(6),
    ]

    print("\nMission items:")
    print(f"  0 TAKEOFF            lat={home_lat:.7f} lon={home_lon:.7f} alt={alt:.0f} m")
    print(f"  1 WAYPOINT  (close)  lat={wp1_lat:.7f} lon={wp1_lon:.7f} alt={alt:.0f} m  (15 m north of home)")
    print(f"  2 CONDITION_GATE     lat={gate_lat:.7f} lon={gate_lon:.7f} alt={alt:.0f} m  "
          f"(param1=0 param2=0/2D-only param3,4=NaN; {gate_offset_m:.0f} m off the WP1->WP2 line, NOT a flight target)")
    print(f"  3 DO_SET_ROI_LOCATION lat={roi_lat:.7f} lon={roi_lon:.7f} alt={alt:.0f} m  "
          f"(visual marker ~8 m from the gate; not part of the mission flow)")
    print(f"  4 DO_CHANGE_SPEED    -> {slow_speed:.1f} m/s  (fires once the gate is marked reached)")
    print(f"  5 WAYPOINT  (far)    lat={wp2_lat:.7f} lon={wp2_lon:.7f} alt={alt:.0f} m  "
          f"(~{leg_m:.0f} m east of wp1 -- straight line the whole way, gate does not bend it)")
    print(f"  6 RETURN_TO_LAUNCH\n")
    print("The gate sits off to the side of the flight path -- look for its (and the ROI's)")
    print("pin on the map, away from the straight WP1 -> WP2 line the vehicle actually flies.")
    print("Watch the vehicle's speed: it should visibly slow down partway through that leg,")
    print("at the moment it crosses the gate's plane -- not at either waypoint.\n")

    print("Uploading mission ...", flush=True)
    if upload_mission(conn, items):
        print("Mission uploaded successfully.")
        print("\nThis script does NOT arm or start the mission.")
        print("Connect QGroundControl, then use 'Start Mission' from the Fly view")
        print("so you can observe the whole flight, including takeoff.")
    else:
        print("Mission upload failed.")


if __name__ == '__main__':
    main()
