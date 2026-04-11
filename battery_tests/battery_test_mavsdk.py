#!/usr/bin/env python3
"""
battery_test_mavsdk.py - Send MAVLink battery messages to QGroundControl using MAVSDK-Python.

Requires: pip install mavsdk  (bundles mavsdk_server — no separate install needed)

Usage:
  python3 battery_test_mavsdk.py --v1                 # Send BATTERY_STATUS (legacy)
  python3 battery_test_mavsdk.py --v2                 # Send BATTERY_STATUS_V2 + BATTERY_INFO
  python3 battery_test_mavsdk.py --v1 --v2            # Send both (dual-stack)
  python3 battery_test_mavsdk.py --auto               # Start V1; switch to V2+INFO when QGC requests
  python3 battery_test_mavsdk.py --v2 --fault         # Send V2 with fault flags
  python3 battery_test_mavsdk.py --v2 --no-info       # Send V2 without BATTERY_INFO
  python3 battery_test_mavsdk.py --v2 --count 2       # Two batteries

  --port PORT   UDP port QGC listens on (default: 14550)
  --count N     Number of battery instances (default: 1)
  --fast        Cycle battery 5× faster (useful for testing LOW/CRITICAL alerts)
"""

import argparse
import asyncio
import json
import math
import select
import socket
import struct
import threading
import time

from mavsdk import System
from mavsdk.mavlink_direct import MavlinkDirect, MavlinkMessage

# ---------------------------------------------------------------------------
# MAVLink message IDs and command codes
# ---------------------------------------------------------------------------
MAVLINK_MSG_ID_BATTERY_STATUS    = 147
MAVLINK_MSG_ID_BATTERY_STATUS_V2 = 369
MAVLINK_MSG_ID_BATTERY_INFO      = 372

_MSG_NAMES = {
    MAVLINK_MSG_ID_BATTERY_STATUS:    'BATTERY_STATUS',
    MAVLINK_MSG_ID_BATTERY_STATUS_V2: 'BATTERY_STATUS_V2',
    MAVLINK_MSG_ID_BATTERY_INFO:      'BATTERY_INFO',
}

MAV_CMD_SET_MESSAGE_INTERVAL = 511
MAV_RESULT_ACCEPTED     = 0
MAV_RESULT_UNSUPPORTED  = 3

# ---------------------------------------------------------------------------
# MAV_BATTERY_STATUS_FLAGS values (development dialect)
# ---------------------------------------------------------------------------
FLAG_NOT_READY_TO_USE          = 1
FLAG_CHARGING                  = 2
FLAG_CELL_BALANCING            = 4
FLAG_FAULT_CELL_IMBALANCE      = 8
FLAG_FAULT_PROTECTION          = 256
FLAG_FAULT_OVER_VOLT           = 512
FLAG_FAULT_UNDER_VOLT          = 1024
FLAG_FAULT_OVER_TEMP           = 2048
FLAG_FAULT_UNDER_TEMP          = 4096
FLAG_FAULT_OVER_CURRENT        = 8192
FLAG_FAULT_SHORT_CIRCUIT       = 16384
FLAG_FAULT_INCOMPATIBLE_VOLT   = 32768
FLAG_FAULT_INCOMPATIBLE_FW     = 65536
FLAG_FAULT_INCOMPATIBLE_CELLS  = 131072
FLAG_CAPACITY_RELATIVE_TO_FULL = 262144

# ---------------------------------------------------------------------------
# Custom XML for BATTERY_STATUS_V2 and BATTERY_INFO
# (development dialect — not in mavsdk_server's default common.xml)
# Field order must match the MAVLink spec exactly to produce the correct CRC_EXTRA.
# ---------------------------------------------------------------------------
_V2_XML = """\
<?xml version="1.0"?>
<mavlink>
  <messages>
    <message id="369" name="BATTERY_STATUS_V2">
      <description>Battery dynamic information.</description>
      <field type="float"    name="voltage">Voltage (V). NaN if not known.</field>
      <field type="float"    name="current">Battery current (A). Positive when discharging. NaN if not known.</field>
      <field type="float"    name="capacity_consumed">Consumed charge (Ah). NaN if not known.</field>
      <field type="float"    name="capacity_remaining">Remaining charge (Ah). NaN if not known.</field>
      <field type="uint32_t" name="status_flags">Fault, health, readiness, and other status indications.</field>
      <field type="int16_t"  name="temperature">Temperature of the battery (cdegC). INT16_MAX if not known.</field>
      <field type="uint8_t"  name="id">Battery ID (0-indexed).</field>
      <field type="uint8_t"  name="percent_remaining">Remaining battery energy. Values: [0-100], UINT8_MAX if not known.</field>
    </message>
  </messages>
</mavlink>"""

_INFO_XML = """\
<?xml version="1.0"?>
<mavlink>
  <messages>
    <message id="372" name="BATTERY_INFO">
      <description>Battery information that is static, or changes very slowly.</description>
      <field type="float"    name="discharge_minimum_voltage">Minimum per-cell voltage when discharging.</field>
      <field type="float"    name="charging_minimum_voltage">Minimum per-cell voltage when charging.</field>
      <field type="float"    name="resting_minimum_voltage">Minimum per-cell voltage at rest.</field>
      <field type="float"    name="charging_maximum_voltage">Maximum per-cell voltage when charged.</field>
      <field type="float"    name="charging_maximum_current">Maximum pack continuous charge current.</field>
      <field type="float"    name="nominal_voltage">Battery nominal voltage.</field>
      <field type="float"    name="discharge_maximum_current">Maximum pack continuous discharge current.</field>
      <field type="float"    name="discharge_maximum_burst_current">Maximum pack burst discharge current.</field>
      <field type="float"    name="design_capacity">Designed battery capacity (Ah).</field>
      <field type="float"    name="full_charge_capacity">Predicted battery capacity when fully charged (Ah).</field>
      <field type="uint16_t" name="cycle_count">Number of charge/discharge cycles.</field>
      <field type="uint16_t" name="weight">Battery weight (g).</field>
      <field type="uint8_t"  name="id">Battery ID (0-indexed).</field>
      <field type="uint8_t"  name="battery_function">Function of the battery.</field>
      <field type="uint8_t"  name="type">Type of the battery.</field>
      <field type="uint8_t"  name="state_of_health">State of Health (SOH) estimate.</field>
      <field type="uint8_t"  name="cells_in_series">Number of battery cells in series.</field>
      <field type="char[9]"  name="manufacture_date">Manufacture date (DD/MM/YYYY).</field>
      <field type="char[32]" name="serial_number">Serial number.</field>
      <field type="char[50]" name="name">Battery device name.</field>
    </message>
  </messages>
</mavlink>"""


# ---------------------------------------------------------------------------
# Stream state (used by --auto mode)
# ---------------------------------------------------------------------------
class StreamState:
    def __init__(self):
        self.v1                    = True   # streaming BATTERY_STATUS
        self.v2                    = False  # streaming BATTERY_STATUS_V2
        self.info                  = False  # streaming BATTERY_INFO
        self.send_info_now         = False  # main loop should send BATTERY_INFO immediately
        self.handshake_complete_at = None   # monotonic time when AUTOPILOT_VERSION was sent


# ---------------------------------------------------------------------------
# Simulation helpers
# ---------------------------------------------------------------------------
def simulate(percent, full_ah=4.8):
    """Return (voltage, current_a, consumed_ah, remaining_ah) for a given percent."""
    voltage      = 14.0 + (16.8 - 14.0) * (percent / 100.0)
    current      = 10.0 + 5.0 * (percent / 100.0)
    consumed_ah  = full_ah * (1.0 - percent / 100.0)
    remaining_ah = full_ah * (percent / 100.0)
    return voltage, current, consumed_ah, remaining_ah


def _fj(fields: dict) -> str:
    """Serialise a fields dict to JSON string, converting NaN to null."""
    # JSON doesn't support NaN; mavsdk_server expects null for NaN floats
    return json.dumps(
        {k: (None if (isinstance(v, float) and math.isnan(v)) else v)
         for k, v in fields.items()}
    )


# ---------------------------------------------------------------------------
# Message senders
# ---------------------------------------------------------------------------
async def send_v1(mavlink: MavlinkDirect, battery_id: int,
                  percent: int, voltage_v: float, current_a: float,
                  mah_consumed: float):
    cell_mv = int((voltage_v / 4) * 1000)

    charge_state = 1  # MAV_BATTERY_CHARGE_STATE_OK
    if percent < 10:
        charge_state = 5  # EMERGENCY
    elif percent < 20:
        charge_state = 4  # CRITICAL
    elif percent < 30:
        charge_state = 3  # LOW

    # BATTERY_STATUS voltages field: 10 cells, unused = UINT16_MAX (65535)
    voltages = [cell_mv] * 4 + [65535] * 6

    msg = MavlinkMessage(
        message_name        = "BATTERY_STATUS",
        system_id           = 1,
        component_id        = 1,
        target_system_id    = 0,
        target_component_id = 0,
        fields_json         = _fj({
            "id":               battery_id,
            "battery_function": 0,   # MAV_BATTERY_FUNCTION_ALL
            "type":             0,   # MAV_BATTERY_TYPE_LIPO
            "temperature":      2800,
            "voltages":         voltages,
            "current_battery":  int(current_a * 100),
            "current_consumed": int(mah_consumed),
            "energy_consumed":  -1,
            "battery_remaining": percent,
            "time_remaining":   0,
            "charge_state":     charge_state,
            "voltages_ext":     [0, 0, 0, 0],
            "mode":             0,
            "fault_bitmask":    0,
        }),
    )
    await mavlink.send_message(msg)
    print(f"  [V1 BATTERY_STATUS      ] id={battery_id} {percent:3d}% "
          f"{voltage_v:.2f}V {current_a:.1f}A {mah_consumed:.0f}mAh "
          f"charge_state={charge_state}", flush=True)


async def send_battery_info(mavlink: MavlinkDirect, battery_id: int,
                             full_charge_ah: float = None):
    nan = float('nan')
    fcc = full_charge_ah if full_charge_ah is not None else nan

    msg = MavlinkMessage(
        message_name        = "BATTERY_INFO",
        system_id           = 1,
        component_id        = 1,
        target_system_id    = 0,
        target_component_id = 0,
        fields_json         = _fj({
            "id":                          battery_id,
            "battery_function":            1,
            "type":                        1,
            "state_of_health":             92,
            "cells_in_series":             4,
            "cycle_count":                 47,
            "weight":                      245,
            "charging_maximum_voltage":    16.8,
            "charging_minimum_voltage":    0.0,
            "resting_minimum_voltage":     14.8,
            "discharge_minimum_voltage":   14.0,
            "charging_maximum_current":    10.0,
            "nominal_voltage":             14.8,
            "discharge_maximum_current":   100.0,
            "discharge_maximum_burst_current": 200.0,
            "design_capacity":             5.2,
            "full_charge_capacity":        fcc,
            "manufacture_date":            "01012024",
            "serial_number":               f"SN-{1000 + battery_id}",
            "name":                        "Acme_LiPo4S5000",
        }),
    )
    await mavlink.send_message(msg)
    cap_str = f"{fcc:.2f}Ah" if full_charge_ah is not None else "NaN (not provided)"
    print(f"  [BATTERY_INFO           ] id={battery_id} "
          f"name=Acme_LiPo4S5000 cells=4 soh=92% "
          f"full_charge_capacity={cap_str}", flush=True)


async def send_v2(mavlink: MavlinkDirect, battery_id: int,
                  percent: int, voltage_v: float, current_a: float,
                  capacity_consumed_ah: float, capacity_remaining_ah: float,
                  status_flags: int, provide_consumed: bool):
    nan = float('nan')
    consumed  = capacity_consumed_ah  if     provide_consumed else nan
    remaining = capacity_remaining_ah if not provide_consumed else nan

    msg = MavlinkMessage(
        message_name        = "BATTERY_STATUS_V2",
        system_id           = 1,
        component_id        = 1,
        target_system_id    = 0,
        target_component_id = 0,
        fields_json         = _fj({
            "id":                 battery_id,
            "temperature":        2800,
            "voltage":            voltage_v,
            "current":            current_a,
            "capacity_consumed":  consumed,
            "capacity_remaining": remaining,
            "percent_remaining":  percent,
            "status_flags":       status_flags,
        }),
    )
    await mavlink.send_message(msg)

    c_str = f"{consumed:.3f}Ah"   if not math.isnan(consumed)   else "NaN"
    r_str = f"{remaining:.3f}Ah"  if not math.isnan(remaining)  else "NaN (→inferred)"
    flag_str = f" flags=0x{status_flags:08x}" if status_flags else ""
    print(f"  [V2 BATTERY_STATUS_V2   ] id={battery_id} {percent:3d}% "
          f"{voltage_v:.2f}V {current_a:.1f}A "
          f"consumed={c_str} remaining={r_str}{flag_str}", flush=True)


# ---------------------------------------------------------------------------
# MAVLink helpers for the responder thread
# ---------------------------------------------------------------------------
def _crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if crc & 1 else crc >> 1
    return crc


def _mavlink2_frame(msg_id: int, crc_extra: int, payload: bytes,
                    seq: int, sys_id: int = 1, comp_id: int = 1) -> bytes:
    """Pack a MAVLink2 frame (no signing)."""
    header = struct.pack('<BBBBBBBBBB',
        0xFD, len(payload), 0, 0, seq & 0xFF,
        sys_id, comp_id,
        msg_id & 0xFF, (msg_id >> 8) & 0xFF, (msg_id >> 16) & 0xFF)
    crc_data = header[1:] + payload + bytes([crc_extra])
    return header + payload + struct.pack('<H', _crc16(crc_data))


def _vehicle_heartbeat(seq: int = 0) -> bytes:
    """MAVLink2 HEARTBEAT from sysid=1 compid=1, type=QUADROTOR, autopilot=PX4."""
    payload  = struct.pack('<IBBBBB', 0, 1, 12, 0, 4, 3)   # QUADROTOR/PX4/ACTIVE
    header   = struct.pack('<BBBBBBBBBB', 0xFD, 9, 0, 0, seq & 0xFF, 1, 1, 0, 0, 0)
    crc_data = header[1:] + payload + bytes([50])            # CRC_EXTRA for HEARTBEAT = 50
    return header + payload + struct.pack('<H', _crc16(crc_data))


def _parse_command_long(payload: bytes) -> tuple[int, list[float]]:
    """Parse COMMAND_LONG payload → (command_id, [param1..7])."""
    if len(payload) < 30:
        return 0, []
    params  = list(struct.unpack_from('<7f', payload, 0))    # param1..7
    command = struct.unpack_from('<H', payload, 28)[0]
    return command, params


# ---------------------------------------------------------------------------
# Raw MAVLink2 payload builders for the responder thread
#
# All handshake responses (COMMAND_ACK, AUTOPILOT_VERSION, PARAM_VALUE,
# MISSION_COUNT) must be sent from the SAME socket that receives QGC's
# commands (the `ka` keep-alive socket).  QGC's requestMessage() infrastructure
# correlates responses to the link/port the request was sent to — if we reply
# via MAVSDK's socket (a different UDP port) QGC will not accept the response.
# ---------------------------------------------------------------------------
def _raw_command_ack(cmd: int, result: int = MAV_RESULT_ACCEPTED) -> bytes:
    """COMMAND_ACK wire payload (msg_id=77, CRC_EXTRA=143).

    Base fields only (3 bytes):  command(u16), result(u8).
    progress/result_param2/target_system/target_component are EXTENSION fields
    (they appear after <extensions/> in common.xml) so they are NOT sorted to the
    front by the MAVLink wire-ordering rules.  Sending just the 3 base bytes is
    valid; QGC zeroes the extensions when absent.
    """
    return struct.pack('<HB', cmd & 0xFFFF, result & 0xFF)


def _raw_autopilot_version() -> bytes:
    """AUTOPILOT_VERSION wire payload (msg_id=148, CRC_EXTRA=178, 60 bytes)."""
    # Wire order (descending size): capabilities(u8), uid(u8), fw(u4), mw(u4),
    #   os(u4), board(u4), vendor(u2), product(u2), fw_custom(8s),
    #   mw_custom(8s), os_custom(8s)
    capabilities = 1  # MAV_PROTOCOL_CAPABILITY_MAVLINK2
    return struct.pack('<QQIIIIHH8s8s8s',
        capabilities, 0,
        0x01000000, 0, 0, 0,
        0, 0,
        b'\x00' * 8, b'\x00' * 8, b'\x00' * 8,
    )


def _raw_param_value(param_index: int = 0) -> bytes:
    """PARAM_VALUE wire payload (msg_id=22, CRC_EXTRA=220, 25 bytes)."""
    # Wire order: param_value(f4), param_count(u2), param_index(u2),
    #             param_id(16s), param_type(u1)
    param_id = b'DUMMY\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'
    return struct.pack('<fHH16sB', 0.0, 1, param_index & 0xFFFF, param_id, 6)


def _raw_mission_count() -> bytes:
    """MISSION_COUNT wire payload (msg_id=44, CRC_EXTRA=221, 4 bytes)."""
    # Wire order: count(u2), target_system(u1), target_component(u1)
    return struct.pack('<HBB', 0, 0, 0)


def _handle_set_message_interval(msg_id: int, interval: float,
                                  state: 'StreamState | None') -> None:
    name = _MSG_NAMES.get(msg_id, f'msg#{msg_id}')
    if interval > 0:
        hz_str = f'{1_000_000 / interval:.2f} Hz'
    elif interval == 0:
        hz_str = 'default rate'
    else:
        hz_str = 'disabled'
    print(f"  [QGC→ SET_MESSAGE_INTERVAL] {name} (id={msg_id})"
          f"  interval={interval:.0f} µs  ({hz_str})", flush=True)

    if state is not None:
        if msg_id == MAVLINK_MSG_ID_BATTERY_STATUS_V2 and interval > 0:
            if not state.v2:
                state.v2 = True
                state.send_info_now = True
                print("  [AUTO] BATTERY_STATUS_V2 stream ON", flush=True)
        elif msg_id == MAVLINK_MSG_ID_BATTERY_INFO and interval > 0:
            if not state.info:
                state.info = True
                print("  [AUTO] BATTERY_INFO stream ON", flush=True)
        elif msg_id == MAVLINK_MSG_ID_BATTERY_STATUS and interval <= 0:
            if state.v1:
                state.v1 = False
                print("  [AUTO] BATTERY_STATUS stream OFF", flush=True)


# ---------------------------------------------------------------------------
# Bootstrap helper + responder thread
#
# Why this approach instead of MAVSDK's message() API:
#   mavsdk_server's SubscribeMessage only forwards messages from the
#   "discovered" system (sysid=1 via our bootstrap heartbeats).  QGC sends
#   as sysid=255 (GCS), so its COMMAND_LONG messages are never delivered.
#
#   However, mavsdk_server acts as a MAVLink routing proxy: it receives
#   packets from QGC addressed to sysid=1 and forwards them to the most
#   recent source of sysid=1 heartbeats.  After bootstrap, we keep sending
#   heartbeats from a persistent keep-alive socket; mavsdk_server routes
#   QGC's commands to that socket, which we can read directly in Python.
# ---------------------------------------------------------------------------
def _bootstrap_grpc(qgc_port: int, timeout: float = 10.0) -> int | None:
    """
    Sniffs mavsdk_server's ephemeral port, sends 10 bootstrap heartbeats
    via SO_REUSEPORT to trigger gRPC startup, then closes the spy socket.
    Returns mavsdk_server's ephemeral port, or None on timeout.

    Note: requires SO_REUSEPORT (Linux ≥ 3.9, macOS).  On Windows the port
    share will fail; consider running under WSL instead.
    """
    spy = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    spy.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        spy.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    except (AttributeError, OSError) as exc:
        print(f"  [bootstrap] WARNING: SO_REUSEPORT not available ({exc}); "
              f"bind may fail if mavsdk_server already holds port {qgc_port}", flush=True)
    spy.bind(('0.0.0.0', qgc_port))
    spy.settimeout(0.5)

    deadline    = time.monotonic() + timeout
    mavsdk_port = None

    while time.monotonic() < deadline and not mavsdk_port:
        try:
            data, addr = spy.recvfrom(1024)
            if len(data) >= 6 and data[0] == 0xFD:
                mavsdk_port = addr[1]
                print(f"  [bootstrap] mavsdk_server at port {mavsdk_port}", flush=True)
        except socket.timeout:
            pass

    if mavsdk_port:
        for seq in range(10):
            spy.sendto(_vehicle_heartbeat(seq), ('127.0.0.1', mavsdk_port))
            time.sleep(0.05)

    spy.close()
    print("  [bootstrap] done — spy socket closed", flush=True)
    return mavsdk_port


def _responder_loop(qgc_port: int, bootstrap_port_ref: 'list[int | None]',
                    state: 'StreamState | None',
                    stop_evt: threading.Event) -> None:
    """
    Background thread.

    Sends vehicle HEARTBEAT (sysid=1, compid=1, type=QUADROTOR) DIRECTLY to
    QGC so that QGC identifies sysid=1 as a vehicle (not a GCS).  All
    handshake responses (COMMAND_ACK, AUTOPILOT_VERSION, PARAM_VALUE,
    MISSION_COUNT) are also sent from this same socket so that QGC's
    requestMessage() correlates them correctly with the link it used to send
    the original request.  Sending via MAVSDK (a different UDP port) would
    cause QGC to silently ignore the response and retry indefinitely.
    """
    ka       = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    hb_seq   = 20
    resp_seq = 0
    last_hb  = 0.0
    # Default reply address; overwritten on each received packet.
    resp_addr: tuple = ('127.0.0.1', qgc_port)

    def _raw_send(msg_id_out: int, crc_extra: int, raw_payload: bytes) -> None:
        nonlocal resp_seq
        frame = _mavlink2_frame(msg_id_out, crc_extra, raw_payload, resp_seq)
        try:
            ka.sendto(frame, resp_addr)
        except OSError:
            pass
        resp_seq = (resp_seq + 1) & 0xFF

    def _ack(cmd: int, result: int = MAV_RESULT_ACCEPTED) -> None:
        _raw_send(77, 143, _raw_command_ack(cmd, result))

    while not stop_evt.is_set():
        mavsdk_port = bootstrap_port_ref[0]   # None until bootstrap completes
        now = time.monotonic()
        if now - last_hb >= 1.0:
            try:
                ka.sendto(_vehicle_heartbeat(hb_seq & 0xFF), ('127.0.0.1', qgc_port))
                if mavsdk_port:
                    ka.sendto(_vehicle_heartbeat(hb_seq & 0xFF), ('127.0.0.1', mavsdk_port))
            except OSError:
                pass
            hb_seq  += 1
            last_hb  = now

        readable, _, _ = select.select([ka], [], [], 0.2)
        if not readable:
            continue

        try:
            data, addr = ka.recvfrom(4096)
        except OSError:
            continue

        # Minimal MAVLink2 parse
        if len(data) < 12 or data[0] != 0xFD:
            continue
        payload_len = data[1]
        if len(data) < 10 + payload_len:
            continue
        msg_id  = struct.unpack_from('<I', data[7:10] + b'\x00')[0]
        payload = data[10:10 + payload_len]

        # Reply to whoever sent the command/request.  Commands arrive routed
        # from mavsdk_server (not directly from QGC), so we reply to mavsdk_server
        # which then routes our responses to QGC on the correct vehicle link.
        # For heartbeats we don't reply, so skip updating resp_addr for them.
        if msg_id != 0:   # 0 = HEARTBEAT
            resp_addr = addr

        if msg_id == 76:   # COMMAND_LONG
            cmd, params = _parse_command_long(payload)

            if cmd == MAV_CMD_SET_MESSAGE_INTERVAL:
                _ack(cmd)
                msg_id_req = int(params[0]) if params else 0
                interval   = float(params[1]) if len(params) > 1 else 0.0
                _handle_set_message_interval(msg_id_req, interval, state)

            elif cmd == 512:   # MAV_CMD_REQUEST_MESSAGE
                req_msg_id = int(params[0]) if params else 0
                if req_msg_id == 148:   # AUTOPILOT_VERSION
                    _ack(cmd)
                    _raw_send(148, 178, _raw_autopilot_version())
                    print("  [HANDSHAKE] Sent AUTOPILOT_VERSION", flush=True)
                    # Record handshake time so the main loop can activate V2
                    # after QGC's InitialConnectStateMachine has had time to
                    # complete (handled internally by mavsdk_server).
                    if state is not None and state.handshake_complete_at is None:
                        state.handshake_complete_at = time.monotonic()
                else:
                    # StandardModes, ComponentMetadata, GimbalManager, etc.
                    # UNSUPPORTED lets QGC advance the state machine immediately
                    # rather than waiting 30-60 s for a message that never arrives.
                    _ack(cmd, MAV_RESULT_UNSUPPORTED)

            else:
                _ack(cmd)

        elif msg_id == 20:  # PARAM_REQUEST_READ
            idx = struct.unpack_from('<h', payload, 0)[0] if len(payload) >= 2 else 0
            _raw_send(22, 220, _raw_param_value(max(0, idx)))

        elif msg_id == 21:  # PARAM_REQUEST_LIST
            _raw_send(22, 220, _raw_param_value(0))

        elif msg_id == 43:  # MISSION_REQUEST_LIST
            _raw_send(44, 221, _raw_mission_count())


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
async def run(args):
    # compid=191: mavsdk_server sends heartbeats as type=GCS (hardcoded).
    # Using compid=191 (non-autopilot) prevents QGC from treating the system
    # as a GCS.  Our responder thread sends its own compid=1/type=QUADROTOR
    # heartbeat directly to QGC so QGC identifies sysid=1 as a vehicle.
    system = System(sysid=1, compid=191)
    print(f"Starting mavsdk_server → QGC on UDP 127.0.0.1:{args.port} …", flush=True)

    # bootstrap_port_ref[0] is set by the bootstrap thread after it finds the port.
    bootstrap_port_ref: list[int | None] = [None]

    def _bootstrap_thread():
        bootstrap_port_ref[0] = _bootstrap_grpc(args.port)

    bootstrap = threading.Thread(target=_bootstrap_thread, daemon=True)
    bootstrap.start()

    await system.connect(system_address=f"udpout://127.0.0.1:{args.port}")
    print("Connected — gRPC ready.", flush=True)

    # Load custom XML for development dialect messages
    await system.mavlink_direct.load_custom_xml(_V2_XML)
    await system.mavlink_direct.load_custom_xml(_INFO_XML)
    print("Loaded custom XML for BATTERY_STATUS_V2 and BATTERY_INFO.", flush=True)

    mavlink = system.mavlink_direct
    state   = StreamState() if args.auto else None

    # Start the responder thread immediately — don't wait for bootstrap to complete.
    # All handshake responses are sent directly from the ka socket so QGC correlates
    # them with the link it used to send the request.
    stop_evt = threading.Event()
    threading.Thread(
        target=_responder_loop,
        args=(args.port, bootstrap_port_ref, state, stop_evt),
        daemon=True,
    ).start()

    # Wait for bootstrap in the background — just for logging; not on the critical path.
    threading.Thread(target=lambda: (bootstrap.join(timeout=15),
                                     bootstrap_port_ref[0] or
                                     print("  [bootstrap] WARNING: mavsdk_server port not found",
                                           flush=True)),
                     daemon=True).start()

    mode_str = ('auto' if args.auto else '') + (' v1' if args.v1 else '') + (' v2' if args.v2 else '')
    print(f"\n--- Streaming at 1 Hz (Ctrl+C to stop) ---")
    print(f"    mode={mode_str.strip()}  fault={args.fault}  no_info={args.no_info}")
    if args.auto:
        print("    [AUTO] Starting with BATTERY_STATUS only. Waiting for QGC to request V2…")

    tick = 0
    # Normal: 1% per 2 s  → 95%→5% in ~180 s (3 min), cycle every 181 ticks
    # Fast:   3% per tick → 95%→5% in ~30 s,           cycle every 31 ticks
    if args.fast:
        cycle_ticks = 31
        cycle_step  = 3
    else:
        cycle_ticks = 181
        cycle_step  = None  # uses (tick % 181) // 2 for 0.5%/s
    try:
        while True:
            t = tick % cycle_ticks
            if args.fast:
                percent = max(5, 95 - t * cycle_step)
            else:
                percent = max(5, 95 - t // 2)

            for bid in range(args.count):
                voltage, current, consumed_ah, remaining_ah = simulate(percent)

                # --- explicit --v1 ---
                if args.v1:
                    await send_v1(mavlink, bid, percent,
                                  voltage_v    = voltage + bid * 0.05,
                                  current_a    = current,
                                  mah_consumed = consumed_ah * 1000)

                # --- explicit --v2 ---
                if args.v2:
                    if tick % 10 == 0:
                        await send_battery_info(mavlink, bid,
                                                full_charge_ah=None if args.no_info else 4.8)
                    flags = FLAG_FAULT_OVER_TEMP if args.fault else 0
                    await send_v2(mavlink, bid, percent,
                                  voltage_v             = voltage + bid * 0.05,
                                  current_a             = current,
                                  capacity_consumed_ah  = consumed_ah,
                                  capacity_remaining_ah = remaining_ah,
                                  status_flags          = flags,
                                  provide_consumed      = (tick % 4) < 2)

                # --- --auto mode ---
                if state is not None:
                    # Fallback: if QGC's InitialConnectStateMachine completes
                    # via mavsdk_server's internal handlers, SET_MESSAGE_INTERVAL
                    # may never reach our responder.  After 15 s from handshake
                    # we activate V2 regardless so QGC can pick it up via
                    # handleMessageForFactGroupCreation.
                    if (not state.v2
                            and state.handshake_complete_at is not None
                            and (time.monotonic() - state.handshake_complete_at) > 15.0):
                        state.v2 = True
                        state.send_info_now = True
                        print("  [AUTO] Activating V2 (15 s after handshake)", flush=True)

                    if state.v1:
                        await send_v1(mavlink, bid, percent,
                                      voltage_v    = voltage + bid * 0.05,
                                      current_a    = current,
                                      mah_consumed = consumed_ah * 1000)

                    if state.v2:
                        if state.send_info_now:
                            await send_battery_info(mavlink, bid, full_charge_ah=4.8)
                            state.send_info_now = False
                        elif state.info and tick % 10 == 0:
                            await send_battery_info(mavlink, bid, full_charge_ah=4.8)

                        flags = FLAG_FAULT_OVER_TEMP if args.fault else 0
                        await send_v2(mavlink, bid, percent,
                                      voltage_v             = voltage + bid * 0.05,
                                      current_a             = current,
                                      capacity_consumed_ah  = consumed_ah,
                                      capacity_remaining_ah = remaining_ah,
                                      status_flags          = flags,
                                      provide_consumed      = (tick % 4) < 2)

            tick += 1
            await asyncio.sleep(1.0)
    finally:
        stop_evt.set()


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--v1',      action='store_true', help='Send BATTERY_STATUS (V1)')
    parser.add_argument('--v2',      action='store_true', help='Send BATTERY_STATUS_V2 + BATTERY_INFO')
    parser.add_argument('--auto',    action='store_true',
                        help='Start with BATTERY_STATUS only; switch to V2+INFO when QGC requests')
    parser.add_argument('--fault',   action='store_true', help='(V2) Set fault flags → FAILED charge state')
    parser.add_argument('--no-info', dest='no_info', action='store_true',
                        help='(V2) Skip BATTERY_INFO (no full_charge_capacity → inference disabled)')
    parser.add_argument('--port',  type=int, default=14550)
    parser.add_argument('--count', type=int, default=1, help='Number of battery instances')
    parser.add_argument('--fast',  action='store_true',
                        help='Cycle battery 5× faster (95%%→5%% in ~36 s) for alert threshold testing')
    args = parser.parse_args()

    if not args.v1 and not args.v2 and not args.auto:
        parser.error("Specify --v1, --v2, and/or --auto")

    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == '__main__':
    main()
