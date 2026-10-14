#!/usr/bin/env python3
"""
energy_sim.py - Simulate a vehicle that implements the MAVLink Energy Systems Protocol,
for testing QGroundControl. The vehicle's energy systems, batteries and fuel tanks,
and how it responds to the GCS, are defined in a scenario YAML file.

Requires: pip install -r requirements.txt  (mavsdk-grpc bundles mavsdk_server)

Usage:
  python3 energy_sim.py --scenario scenarios/single_battery.yaml
  python3 energy_sim.py --scenario scenarios/vtol_mc_fw.yaml --speed 10
  python3 energy_sim.py --list-scenarios

  --port PORT    UDP port QGC listens on (default: 14550)
  --speed N      Run the scenario's levels and timeline N times faster (default: 1)
  --quiet        Only log protocol events and a periodic summary, not every message sent
"""

import argparse
import asyncio
import functools
import json
import math
import os
import select
import socket
import struct
import sys
import threading
import time
from pathlib import Path

try:
    # The gRPC wrapper was renamed from `mavsdk` to `mavsdk-grpc` when `mavsdk` 4.x became
    # the native binding (a different API).
    from mavsdk_grpc import System
    from mavsdk_grpc.mavlink_direct import MavlinkMessage
except ImportError:
    from mavsdk import System
    from mavsdk.mavlink_direct import MavlinkMessage

from pymavlink.dialects.v20 import common as mavcommon

from dialect import load_energy_dialect
from model import EnergyModel
from protocol import (MAV_CMD_REQUEST_MESSAGE, MAV_CMD_SET_MESSAGE_INTERVAL, MAV_RESULT_ACCEPTED,
                      MAV_RESULT_UNSUPPORTED, MSG_BATTERY_CELL_STATUS, MSG_BATTERY_INFO,
                      MSG_BATTERY_STATUS, MSG_BATTERY_STATUS_V2, MSG_ENERGY_SYSTEM_INFO,
                      MSG_ENERGY_SYSTEM_STATUS, MSG_FUEL_INFORMATION_V2, MSG_FUEL_STATUS_V2,
                      Protocol, msg_name)
from scenario import (FLOW_COMPONENT, MAV_BATTERY_STATUS_FLAGS, MAV_ENERGY_STATUS_FLAGS, MAV_ENERGY_SYSTEM_FLAGS,
                      MAV_FUEL_STATUS_FLAGS, ScenarioError, flag_names, load_scenario)

HERE = Path(__file__).resolve().parent
XML_PATH = HERE / 'xml' / 'energy_messages.xml'
SCENARIO_DIR = HERE / 'scenarios'


_DEBUG = bool(os.environ.get('ENERGY_SIM_DEBUG'))


def _log(text: str) -> None:
    print(f"  {text}", flush=True)


def _fj(fields: dict) -> str:
    """Serialise a fields dict to JSON, converting NaN to null (mavsdk_server expects null)."""
    return json.dumps({k: (None if (isinstance(v, float) and math.isnan(v)) else v)
                       for k, v in fields.items()}, ensure_ascii=False)


def _f(v, fmt='.2f') -> str:
    return 'NaN' if isinstance(v, float) and math.isnan(v) else format(v, fmt)


# ---------------------------------------------------------------------------
# Raw MAVLink2 helpers for the responder thread (from battery_tests)
#
# Command replies (COMMAND_ACK, AUTOPILOT_VERSION, ...) must be sent from the same
# socket that receives QGC's commands (the `ka` keep-alive socket): QGC correlates
# responses with the link/port it sent the request on.
# ---------------------------------------------------------------------------
class _Sequence:
    """MAVLink sequence numbers for everything sent from the `ka` socket (sysid 1, compid 1).
    Shared by the responder thread and the emitter so that QGC sees one gap-free stream."""

    def __init__(self):
        self._lock = threading.Lock()
        self._seq = 0

    def next(self) -> int:
        with self._lock:
            seq = self._seq
            self._seq = (self._seq + 1) & 0xFF
            return seq


_SEQ = _Sequence()


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


AUTOPILOTS = {'generic': 0, 'px4': 12}
_autopilot = AUTOPILOTS['generic']


MAV_TYPE_BATTERY = 36


def _component_heartbeat(seq: int, comp_id: int) -> bytes:
    """HEARTBEAT of a smart battery component: type=MAV_TYPE_BATTERY, autopilot=INVALID, state=ACTIVE."""
    payload = struct.pack('<IBBBBB', 0, MAV_TYPE_BATTERY, 8, 0, 4, 3)
    return _mavlink2_frame(0, 50, payload, seq, comp_id=comp_id)


def _vehicle_heartbeat(seq: int = 0) -> bytes:
    """HEARTBEAT from sysid=1 compid=1: type=QUADROTOR, state=ACTIVE.

    MAV_AUTOPILOT_GENERIC by default, so that QGC uses its generic firmware plugin:
    the energy systems implementation must not depend on PX4/ArduPilot code.
    """
    payload = struct.pack('<IBBBBB', 0, 2, _autopilot, 0, 4, 3)   # custom_mode, type, autopilot, base_mode, state, version
    return _mavlink2_frame(0, 50, payload, seq)


def _parse_command_long(payload: bytes) -> tuple[int, list[float], int]:
    """COMMAND_LONG payload -> (command_id, [param1..7], target_component). Trailing zeros may be truncated."""
    payload = payload.ljust(33, b'\x00')
    params = list(struct.unpack_from('<7f', payload, 0))
    command = struct.unpack_from('<H', payload, 28)[0]
    target_component = payload[31]
    return command, params, target_component


def _raw_command_ack(cmd: int, result: int = MAV_RESULT_ACCEPTED) -> bytes:
    """COMMAND_ACK wire payload (msg_id=77, CRC_EXTRA=143): base fields only."""
    return struct.pack('<HB', cmd & 0xFFFF, result & 0xFF)


def _raw_autopilot_version() -> bytes:
    """AUTOPILOT_VERSION wire payload (msg_id=148, CRC_EXTRA=178, 60 bytes)."""
    capabilities = 1                     # MAV_PROTOCOL_CAPABILITY_MAVLINK2 (as battery_tests)
    return struct.pack('<QQIIIIHH8s8s8s', capabilities, 0, 0x01000000, 0, 0, 0, 0, 0,
                       b'\x00' * 8, b'\x00' * 8, b'\x00' * 8)


def _raw_param_value(param_index: int = 0) -> bytes:
    """PARAM_VALUE wire payload (msg_id=22, CRC_EXTRA=220)."""
    param_id = b'DUMMY'.ljust(16, b'\x00')
    return struct.pack('<fHH16sB', 0.0, 1, param_index & 0xFFFF, param_id, 9)


def _raw_mission_count() -> bytes:
    """MISSION_COUNT wire payload (msg_id=44, CRC_EXTRA=221)."""
    return struct.pack('<HBB', 0, 0, 0)


# ---------------------------------------------------------------------------
# Bootstrap + responder thread (from battery_tests; see CLAUDE.md there)
# ---------------------------------------------------------------------------
def _bootstrap_grpc(qgc_port: int, timeout: float = 10.0) -> int | None:
    """Sniff mavsdk_server's ephemeral port and send it heartbeats so that it discovers
    sysid=1 and System.connect() returns."""
    spy = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    spy.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        spy.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    except (AttributeError, OSError) as exc:
        _log(f"[bootstrap] WARNING: SO_REUSEPORT not available ({exc})")
    spy.bind(('0.0.0.0', qgc_port))
    spy.settimeout(0.5)
    deadline = time.monotonic() + timeout
    mavsdk_port = None
    while time.monotonic() < deadline and not mavsdk_port:
        try:
            data, addr = spy.recvfrom(1024)
            if len(data) >= 6 and data[0] == 0xFD and data[5] != 255:   # not from the GCS (sysid 255)
                mavsdk_port = addr[1]
                _log(f"[bootstrap] mavsdk_server at port {mavsdk_port}")
        except socket.timeout:
            pass
    if mavsdk_port:
        for seq in range(10):
            spy.sendto(_vehicle_heartbeat(seq), ('127.0.0.1', mavsdk_port))
            time.sleep(0.05)
    spy.close()
    _log("[bootstrap] done - spy socket closed")
    return mavsdk_port


def _responder_loop(ka: socket.socket, qgc_port: int, bootstrap_port_ref: list, protocol: Protocol,
                    stop_evt: threading.Event) -> None:
    """Background thread: vehicle HEARTBEAT, plus replies to QGC's commands and requests."""
    last_hb = 0.0
    resp_addr: tuple = ('127.0.0.1', qgc_port)
    seen_sources: set = set()
    recent_frames: dict = {}     # (sysid, compid, seq, msg_id) -> time, to drop duplicates

    def _raw_send(msg_id_out: int, crc_extra: int, raw_payload: bytes, comp_id: int = FLOW_COMPONENT) -> None:
        try:
            ka.sendto(_mavlink2_frame(msg_id_out, crc_extra, raw_payload, _SEQ.next(), comp_id=comp_id), resp_addr)
        except OSError:
            pass

    def _ack(cmd: int, result: int | None = MAV_RESULT_ACCEPTED, comp_id: int = FLOW_COMPONENT) -> None:
        if result is not None:      # None: the addressed component doesn't exist, so nobody replies
            _raw_send(77, 143, _raw_command_ack(cmd, result), comp_id)

    while not stop_evt.is_set():
        mavsdk_port = bootstrap_port_ref[0]
        now = time.monotonic()
        if now - last_hb >= 1.0:
            heartbeats = [_vehicle_heartbeat(_SEQ.next())]
            heartbeats += [_component_heartbeat(_SEQ.next(), c.id)
                           for c in protocol.scenario.components.values() if c.heartbeat]
            try:
                for heartbeat in heartbeats:
                    ka.sendto(heartbeat, ('127.0.0.1', qgc_port))
                    if mavsdk_port:
                        ka.sendto(heartbeat, ('127.0.0.1', mavsdk_port))
            except OSError:
                pass
            last_hb = now

        readable, _, _ = select.select([ka], [], [], 0.2)
        if not readable:
            continue
        try:
            data, addr = ka.recvfrom(4096)
        except OSError:
            continue
        if len(data) < 12 or data[0] != 0xFD:
            continue
        payload_len = data[1]
        if len(data) < 10 + payload_len:
            continue
        msg_id = data[7] | (data[8] << 8) | (data[9] << 16)
        payload = data[10:10 + payload_len]
        if msg_id == 0 or data[5] == 1:
            # HEARTBEATs need no reply. sysid 1 is ourselves: mavsdk_server (1/191) sends its
            # own requests and echoes our telemetry to this socket.
            continue
        if _DEBUG:
            cmd_dbg = struct.unpack_from('<H', payload.ljust(30, b'\x00'), 28)[0] if msg_id == 76 else ''
            _log(f"[debug] rx msg={msg_id} {cmd_dbg} sys={data[5]} comp={data[6]} seq={data[4]} from {addr}")
        # QGC sends to every address it has heard sysid 1 from, and mavsdk_server may also
        # forward the same frame: handle each identical frame once.
        key = (data[5], data[6], data[4], msg_id, bytes(payload))
        if now - recent_frames.get(key, -10.0) < 2.0:
            continue
        recent_frames[key] = now
        if len(recent_frames) > 512:
            for k in [k for k, ts in recent_frames.items() if now - ts > 2.0]:
                del recent_frames[k]
        resp_addr = addr
        if addr not in seen_sources:
            seen_sources.add(addr)
            _log(f"[responder] receiving GCS traffic from {addr[0]}:{addr[1]}")

        if msg_id == 76:    # COMMAND_LONG
            cmd, params, target = _parse_command_long(payload)
            if target not in (0, FLOW_COMPONENT):
                # Addressed to another simulated component (e.g. a smart battery). Only energy
                # message requests are meaningful there; anything else gets no reply.
                req_id = int(params[0])
                if cmd == MAV_CMD_REQUEST_MESSAGE and protocol.handles(req_id):
                    _ack(cmd, protocol.request_message(req_id, params[1], target), target)
                elif cmd == MAV_CMD_SET_MESSAGE_INTERVAL and protocol.handles(req_id):
                    _ack(cmd, protocol.set_message_interval(req_id, params[1], target), target)
                elif target in protocol.components():
                    _ack(cmd, MAV_RESULT_UNSUPPORTED, target)
                continue
            if cmd == MAV_CMD_SET_MESSAGE_INTERVAL:
                req_id = int(params[0])
                if protocol.handles(req_id):
                    _ack(cmd, protocol.set_message_interval(req_id, params[1]))
                else:
                    _ack(cmd)
                    _log(f"[GCS-> SET_MESSAGE_INTERVAL] {msg_name(req_id)} (id={req_id}) "
                         f"{params[1]:.0f} us  -> ACCEPTED (ignored: not an energy message)")
            elif cmd == MAV_CMD_REQUEST_MESSAGE:
                req_id = int(params[0])
                if req_id == 148:                  # AUTOPILOT_VERSION
                    _ack(cmd)
                    _raw_send(148, 178, _raw_autopilot_version())
                    _log("[HANDSHAKE] Sent AUTOPILOT_VERSION")
                elif protocol.handles(req_id):
                    _ack(cmd, protocol.request_message(req_id, params[1]))
                else:
                    # StandardModes, ComponentMetadata, ...: UNSUPPORTED lets QGC's
                    # initial-connect state machine advance without waiting for timeouts.
                    _ack(cmd, MAV_RESULT_UNSUPPORTED)
            else:
                _ack(cmd)
        elif msg_id == 20:  # PARAM_REQUEST_READ
            idx = struct.unpack_from('<h', payload.ljust(2, b'\x00'), 0)[0]
            _raw_send(22, 220, _raw_param_value(max(0, idx)))
        elif msg_id == 21:  # PARAM_REQUEST_LIST
            _raw_send(22, 220, _raw_param_value(0))
        elif msg_id == 43:  # MISSION_REQUEST_LIST
            _raw_send(44, 221, _raw_mission_count())


# ---------------------------------------------------------------------------
# Message emission
# ---------------------------------------------------------------------------
# By default every message is packed with pymavlink and sent raw from the `ka` socket as
# sysid 1 / compid 1 (the autopilot). MavlinkDirect (--mavlink-direct) is not suitable for
# testing QGC's energy systems support:
#  - mavsdk_server sends MavlinkDirect messages from its own component id (191), ignoring
#    MavlinkMessage.component_id, and QGC only uses energy messages from the autopilot.
#  - mavsdk_server already has BATTERY_INFO (from its own common.xml, without the new
#    `energy_systems` extension field) and load_custom_xml() does not replace it, so libmav
#    fails with "Failed to set unsigned integer field energy_systems". BATTERY_INFO is
#    therefore always sent raw.
RAW_MESSAGES = {'BATTERY_INFO'}


class Emitter:
    def __init__(self, mavlink, model: EnergyModel, quiet: bool, raw_sock: socket.socket, raw_addr: tuple,
                 mavlink_direct: bool = False):
        self.mavlink = mavlink
        self.model = model
        self.quiet = quiet
        self.raw_sock = raw_sock
        self.raw_addr = raw_addr
        self.mavlink_direct = mavlink_direct
        self.dialect = load_energy_dialect()
        self.raw_mav = self.dialect.MAVLink(None, srcSystem=1, srcComponent=1)
        self.common_mav = mavcommon.MAVLink(None, srcSystem=1, srcComponent=1)

    def _send_raw(self, name: str, fields: dict, component: int) -> None:
        if hasattr(self.dialect, f'MAVLink_{name.lower()}_message'):
            cls, mav = getattr(self.dialect, f'MAVLink_{name.lower()}_message'), self.raw_mav
        else:   # e.g. legacy BATTERY_STATUS
            cls, mav = getattr(mavcommon, f'MAVLink_{name.lower()}_message'), self.common_mav
        args = {k: (v.encode('utf-8') if isinstance(v, str) else v) for k, v in fields.items()}
        mav.seq = _SEQ.next()
        mav.srcComponent = component
        self.raw_sock.sendto(cls(**args).pack(mav), self.raw_addr)

    async def _send(self, name: str, fields: dict, summary: str, component: int = FLOW_COMPONENT) -> None:
        try:
            if not self.mavlink_direct or name in RAW_MESSAGES or component != FLOW_COMPONENT:
                self._send_raw(name, fields, component)
            else:
                msg = MavlinkMessage(message_name=name, system_id=1, component_id=1,
                                     target_system_id=0, target_component_id=0, fields_json=_fj(fields))
                await self.mavlink.send_message(msg)
        except Exception as exc:     # surface encoding errors (e.g. unknown field) loudly
            _log(f"[ERROR] sending {name}: {exc}  fields={_fj(fields)}")
            return
        if not self.quiet:
            who = '' if component == FLOW_COMPONENT else f" [component {component}]"
            _log(f"[-> {name:<20}]{who} {summary}")

    async def send(self, msg_id: int, keys: list[tuple[int, int]], t: float) -> None:
        """Send msg_id for each instance key (component, index/id), from that component."""
        m = self.model
        for key in keys:
            component, i = key
            send = functools.partial(self._send, component=component)
            if msg_id == MSG_ENERGY_SYSTEM_INFO:
                f = m.system_info_fields(key)
                await send('ENERGY_SYSTEM_INFO', f,
                                 f"index={i} energy_systems={flag_names(MAV_ENERGY_SYSTEM_FLAGS, f['energy_systems'])}"
                                 + (f" name={f['energy_system_name']!r}" if f['energy_system_name'] else ''))
            elif msg_id == MSG_ENERGY_SYSTEM_STATUS:
                f = m.system_status_fields(key, t)
                await send('ENERGY_SYSTEM_STATUS', f,
                                 f"index={i} {_f(f['remaining_capacity'])}% time={_f(f['remaining_time'], '.0f')}s "
                                 f"flags={flag_names(MAV_ENERGY_STATUS_FLAGS, f['status_flags'])}")
            elif msg_id == MSG_BATTERY_INFO:
                f = m.battery_info_fields(key)
                await send('BATTERY_INFO', f,
                                 f"id={i} name={f['name']} cells={f['cells_in_series']} "
                                 f"energy_systems={flag_names(MAV_ENERGY_SYSTEM_FLAGS, f['energy_systems'])}")
            elif msg_id == MSG_BATTERY_STATUS_V2:
                f = m.battery_status_fields(key, t)
                soc = 'N/A' if f['state_of_charge'] == 65535 else f"{f['state_of_charge'] * 0.002:.1f}%"
                await send('BATTERY_STATUS_V2', f,
                                 f"id={i} soc={soc} {f['voltage']:.2f}V {f['current']:.1f}A "
                                 f"flags={flag_names(MAV_BATTERY_STATUS_FLAGS, f['status_flags'])}")
            elif msg_id == MSG_BATTERY_CELL_STATUS:
                for f in m.battery_cell_fields(key, t):
                    cells = [v for v in f['voltages'] if v != 65535]
                    await send('BATTERY_CELL_STATUS', f,
                                     f"id={i} cells_instance={f['cells_instance']} mV={cells}")
            elif msg_id == MSG_FUEL_INFORMATION_V2:
                f = m.fuel_info_fields(key)
                await send('FUEL_INFORMATION_V2', f,
                                 f"id={i} fuel_type={f['fuel_type']} maximum_fuel={f['maximum_fuel']:g} "
                                 f"energy_systems={flag_names(MAV_ENERGY_SYSTEM_FLAGS, f['energy_systems'])}")
            elif msg_id == MSG_FUEL_STATUS_V2:
                f = m.fuel_status_fields(key, t)
                await send('FUEL_STATUS_V2', f,
                                 f"id={i} {f['percent_remaining'] * 0.002:.1f}% flow={f['flow_rate']:g} "
                                 f"flags={flag_names(MAV_FUEL_STATUS_FLAGS, f['status_flags'])}")
            elif msg_id == MSG_BATTERY_STATUS:
                f = m.legacy_battery_status_fields(key, t)
                await send('BATTERY_STATUS', f,
                                 f"id={i} {f['battery_remaining']}% charge_state={f['charge_state']}")


def _summary(model: EnergyModel, protocol: Protocol, t: float) -> str:
    parts = []
    for key in model.systems:
        f = model.system_status_fields(key, t)
        name = f"ES{key[1]}" + ('' if key[0] == FLOW_COMPONENT else f"@{key[0]}")
        parts.append(f"{name} {_f(f['remaining_capacity'], '.0f')}% "
                     f"[{flag_names(MAV_ENERGY_STATUS_FLAGS, f['status_flags'])}]")
    streams = ', '.join(f"{msg_name(m)}{'' if c == FLOW_COMPONENT else f'@{c}'}@{iv:g}s"
                        for (m, c), iv in sorted(protocol.streams().items())) or 'none'
    return f"[t={t:5.0f}s] {' | '.join(parts) or 'no energy systems'}  streams: {streams}"


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def _describe(scenario) -> None:
    print(f"\nScenario: {scenario.name}" + (f" ({scenario.path})" if scenario.path else ''))
    if scenario.description:
        for line in scenario.description.splitlines():
            print(f"  {line}")
    b = scenario.behaviour
    print(f"  behaviour: energy_systems={b.energy_systems} batteries={b.batteries} fuel={b.fuel} "
          f"legacy_battery_status={b.legacy_battery_status} "
          f"stream_by_default={[msg_name(m) for m in b.stream_by_default] or 'none'}")
    for es in scenario.energy_systems:
        print(f"  ES{es.index}: {flag_names(MAV_ENERGY_SYSTEM_FLAGS, es.flags)}"
              + (f" name={es.name!r}" if es.name else '')
              + f" aggregate={es.aggregate}"
              + (f" static_flags={flag_names(MAV_ENERGY_STATUS_FLAGS, es.status_flags)}" if es.status_flags else ''))
    for bat in scenario.batteries:
        print(f"  Battery {bat.id}{'' if bat.component == FLOW_COMPONENT else f' (component {bat.component})'}: "
              f"-> {flag_names(MAV_ENERGY_SYSTEM_FLAGS, bat.energy_systems)} "
              f"{bat.cells}S {bat.capacity_ah:g}Ah" + (" cell_status" if bat.cell_status else ''))
    for fuel in scenario.fuel:
        print(f"  Fuel {fuel.id}{'' if fuel.component == FLOW_COMPONENT else f' (component {fuel.component})'}: "
              f"-> {flag_names(MAV_ENERGY_SYSTEM_FLAGS, fuel.energy_systems)} "
              f"type={fuel.fuel_type} max={fuel.maximum_fuel:g}")
    for ev in scenario.timeline:
        print(f"  timeline: at {ev.at:g}s{f' every {ev.repeat:g}s' if ev.repeat else ''}: "
              f"{ev.target} {ev.instance} {ev.description or ''}")
    print()


async def run(args, scenario) -> None:
    system = System(sysid=1, compid=191)
    _log(f"Starting mavsdk_server -> QGC on UDP 127.0.0.1:{args.port} ...")
    bootstrap_port_ref: list = [None]
    bootstrap = threading.Thread(target=lambda: bootstrap_port_ref.__setitem__(0, _bootstrap_grpc(args.port)),
                                 daemon=True)
    bootstrap.start()
    await system.connect(system_address=f"udpout://127.0.0.1:{args.port}")
    _log("Connected - gRPC ready.")

    await system.mavlink_direct.load_custom_xml(XML_PATH.read_text(encoding='utf-8'))
    _log(f"Loaded custom XML: {XML_PATH.name}")

    model = EnergyModel(scenario)
    protocol = Protocol(scenario)
    ka = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    emitter = Emitter(system.mavlink_direct, model, args.quiet, ka, ('127.0.0.1', args.port),
                      mavlink_direct=args.mavlink_direct)

    stop_evt = threading.Event()
    threading.Thread(target=_responder_loop, args=(ka, args.port, bootstrap_port_ref, protocol, stop_evt),
                     daemon=True).start()

    print(f"\n--- Running scenario '{scenario.name}' at speed x{args.speed:g} (Ctrl+C to stop) ---\n", flush=True)
    start = time.monotonic()
    last_summary = -1e9
    try:
        while True:
            t = (time.monotonic() - start) * args.speed
            for line in model.apply_timeline(t):
                _log(line)
            for msg_id, component, instance, _reason in protocol.due():
                instances = (protocol.instances(msg_id, component) if instance is None
                             else [(component, instance)])
                await emitter.send(msg_id, instances, t)
            if time.monotonic() - last_summary >= 5.0:
                _log(_summary(model, protocol, t))
                last_summary = time.monotonic()
            await asyncio.sleep(0.05)
    finally:
        stop_evt.set()


def list_scenarios() -> None:
    for path in sorted(SCENARIO_DIR.glob('*.yaml')):
        try:
            s = load_scenario(path)
            first = s.description.splitlines()[0] if s.description else ''
            print(f"  {path.name:<32} {first}")
        except ScenarioError as exc:
            print(f"  {path.name:<32} INVALID: {exc}")


def resolve_scenario(arg: str) -> Path:
    p = Path(arg)
    if p.exists():
        return p
    for candidate in (SCENARIO_DIR / arg, SCENARIO_DIR / f"{arg}.yaml"):
        if candidate.exists():
            return candidate
    raise ScenarioError(f"scenario not found: {arg}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--scenario', '-s', help='Scenario YAML file (or name in scenarios/)')
    parser.add_argument('--port', type=int, default=14550)
    parser.add_argument('--speed', type=float, default=1.0,
                        help='Time multiplier for levels and timeline (e.g. 10 to reach LOW/CRITICAL quickly)')
    parser.add_argument('--quiet', action='store_true', help="Don't log every message sent")
    parser.add_argument('--mavlink-direct', action='store_true',
                        help="Send telemetry through MAVSDK MavlinkDirect (arrives from mavsdk_server's "
                             "component id 191, which QGC ignores for energy systems; for comparison only)")
    parser.add_argument('--autopilot', choices=sorted(AUTOPILOTS), default='generic',
                        help='MAV_AUTOPILOT in the HEARTBEAT (default: generic)')
    parser.add_argument('--list-scenarios', action='store_true')
    parser.add_argument('--check', action='store_true', help='Validate the scenario and exit')
    args = parser.parse_args()

    global _autopilot
    _autopilot = AUTOPILOTS[args.autopilot]
    if args.list_scenarios:
        list_scenarios()
        return
    if not args.scenario:
        parser.error('--scenario is required (see --list-scenarios)')
    try:
        scenario = load_scenario(resolve_scenario(args.scenario))
    except ScenarioError as exc:
        sys.exit(f"Scenario error: {exc}")
    _describe(scenario)
    if args.check:
        return
    try:
        asyncio.run(run(args, scenario))
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == '__main__':
    main()
