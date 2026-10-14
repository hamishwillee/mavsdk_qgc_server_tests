#!/usr/bin/env python3
"""
fake_gcs.py - A minimal stand-in for QGC that runs the Energy Systems Protocol
connection flow against energy_sim.py and checks the result.

It decodes every frame with a pymavlink dialect generated from
xml/energy_messages.xml, independently of mavsdk_server/libmav, so it also checks
that the simulator's messages are wire-compatible (CRC_EXTRA, field layout,
extension fields) with what a C/C++ GCS generated from the same XML expects.

Like QGC's UDP link, it binds the GCS port and sends to every address it has
received vehicle traffic from.

Usage (run QGC *closed*, since this binds the same port):
  python3 tools/fake_gcs.py --scenario scenarios/single_battery.yaml   # flow + checks
  python3 tools/fake_gcs.py --listen-only --duration 15                 # just decode/print

Exit status is 0 if all checks pass, 1 otherwise.
"""

from __future__ import annotations

import argparse
import math
import socket
import sys
import time
from collections import defaultdict
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT))

from pymavlink.dialects.v20 import common as mavcommon  # noqa: E402

from protocol import (MSG_BATTERY_CELL_STATUS, MSG_BATTERY_INFO, MSG_BATTERY_STATUS,  # noqa: E402
                      MSG_BATTERY_STATUS_V2, MSG_ENERGY_SYSTEM_INFO, MSG_ENERGY_SYSTEM_STATUS,
                      MSG_FUEL_INFORMATION_V2, MSG_FUEL_STATUS_V2, ENERGY_MESSAGES, RESULT_NAMES,
                      msg_name)
from scenario import load_scenario  # noqa: E402
from dialect import load_energy_dialect  # noqa: E402

MAV_CMD_SET_MESSAGE_INTERVAL = 511
MAV_CMD_REQUEST_MESSAGE = 512


class FakeGcs:
    def __init__(self, port: int, verbose: bool):
        self.energy = load_energy_dialect()
        self.mav_out = mavcommon.MAVLink(None, srcSystem=255, srcComponent=190)
        self.mav_out.robust_parsing = True
        self.dec_common = mavcommon.MAVLink(None, srcSystem=255, srcComponent=190)
        self.dec_energy = self.energy.MAVLink(None, srcSystem=255, srcComponent=190)
        self.dec_common.robust_parsing = True
        self.dec_energy.robust_parsing = True
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # SO_REUSEADDR only, as QGC (Qt ShareAddress) does. energy_sim's bootstrap spy then
        # binds the same port later and, on Linux, briefly receives the unicast traffic
        # (SO_REUSEPORT here would instead hash mavsdk_server's packets to this socket).
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('0.0.0.0', port))
        self.sock.settimeout(0.05)
        self.remotes: set = set()
        self.verbose = verbose
        self.last_hb = 0.0
        self.start = time.monotonic()
        # Observations
        self.acks: dict[tuple[int, int], list[int]] = defaultdict(list)   # (cmd, msg_id) -> results
        self.pending_cmds: list[tuple[int, int]] = []
        self.counts: dict[int, int] = defaultdict(int)
        self.comp_acks: dict[tuple[int, int, int], list[int]] = defaultdict(list)   # (cmd, msg_id, comp) -> results
        self.active = False                                      # ENERGY_SYSTEM_INFO accepted
        self.discovery_requested: set = set()                    # (comp, info msg_id) requested on first sight
        self.discovery_pending: set = set()                      # seen before active
        self.first_seen: dict[int, float] = {}
        self.last_seen: dict[int, float] = {}
        self.by_instance: dict[int, dict] = defaultdict(dict)   # msg_id -> (comp, instance) -> last msg
        self.cell_instances: set = set()
        self.bad_frames = 0
        self.vehicle_seen = False
        self.interval_sent_at: dict[int, float] = {}

    def now(self) -> float:
        return time.monotonic() - self.start

    # ------------------------------------------------------------------
    def _send(self, msg) -> None:
        buf = msg.pack(self.mav_out)
        self.mav_out.seq = (self.mav_out.seq + 1) % 256     # pack() doesn't advance it; send() would
        for addr in self.remotes:
            try:
                self.sock.sendto(buf, addr)
            except OSError:
                pass

    def command(self, cmd: int, p1: float, p2: float = 0.0, target: int = 1) -> None:
        msg = mavcommon.MAVLink_command_long_message(1, target, cmd, 0, p1, p2, 0, 0, 0, 0, 0)
        self.pending_cmds.append((cmd, int(p1), target))
        name = 'REQUEST_MESSAGE' if cmd == MAV_CMD_REQUEST_MESSAGE else 'SET_MESSAGE_INTERVAL'
        extra = f" interval={p2:.0f}us" if cmd == MAV_CMD_SET_MESSAGE_INTERVAL else ''
        who = '' if target == 1 else f" to component {target}"
        print(f"[{self.now():6.1f}] GCS -> {name}({msg_name(int(p1))}){extra}{who}", flush=True)
        if cmd == MAV_CMD_SET_MESSAGE_INTERVAL:
            self.interval_sent_at[int(p1)] = self.now()
        self._send(msg)

    # ------------------------------------------------------------------
    def pump(self, duration: float) -> None:
        end = time.monotonic() + duration
        while time.monotonic() < end:
            if time.monotonic() - self.last_hb >= 1.0 and self.remotes:
                self._send(mavcommon.MAVLink_heartbeat_message(6, 8, 0, 0, 4, 3))   # GCS, INVALID
                self.last_hb = time.monotonic()
            try:
                data, addr = self.sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                continue
            if len(data) < 12 or data[0] != 0xFD:
                continue
            if data[5] == 255:      # our own / another GCS
                continue
            self.remotes.add(addr)
            msg_id = data[7] | (data[8] << 8) | (data[9] << 16)
            dec = self.dec_energy if msg_id in ENERGY_MESSAGES and msg_id != MSG_BATTERY_STATUS else self.dec_common
            try:
                msgs = dec.parse_buffer(data) or []
            except Exception as exc:
                self.bad_frames += 1
                print(f"[{self.now():6.1f}] DECODE ERROR {msg_name(msg_id)} from {addr}: {exc}", flush=True)
                continue
            if not msgs:
                continue
            for m in msgs:
                if m.get_type() == 'BAD_DATA':
                    self.bad_frames += 1
                    print(f"[{self.now():6.1f}] BAD FRAME {msg_name(msg_id)}: {m.reason}", flush=True)
                    continue
                self._on_message(m)

    def _on_message(self, m) -> None:
        mid = m.get_msgId()
        t = self.now()
        comp = m.get_srcComponent()
        if m.get_srcSystem() == 1 and mid == 0:
            if comp == 1:
                self.vehicle_seen = True
            elif m.type == 36:    # MAV_TYPE_BATTERY: ask it for its BATTERY_INFO (as QGC does)
                self._discover(comp, MSG_BATTERY_INFO)
        if mid == 77:   # COMMAND_ACK
            for cmd, req, target in list(self.pending_cmds):
                if cmd == m.command and target == comp:
                    self.pending_cmds.remove((cmd, req, target))
                    if comp == 1:
                        self.acks[(cmd, req)].append(m.result)
                    self.comp_acks[(cmd, req, comp)].append(m.result)
                    name = 'REQUEST_MESSAGE' if cmd == MAV_CMD_REQUEST_MESSAGE else 'SET_MESSAGE_INTERVAL'
                    print(f"[{t:6.1f}] GCS <- ACK {name}({msg_name(req)}) = "
                          f"{RESULT_NAMES.get(m.result, m.result)}{'' if comp == 1 else f' from component {comp}'}",
                          flush=True)
                    break
            return
        if mid not in ENERGY_MESSAGES:
            return
        self.counts[mid] += 1
        if mid not in self.first_seen:
            self.first_seen[mid] = t
            since = ''
            if mid in self.interval_sent_at:
                since = f" ({t - self.interval_sent_at[mid]:.1f}s after SET_MESSAGE_INTERVAL)"
            print(f"[{t:6.1f}] first {msg_name(mid)}{since}", flush=True)
        self.last_seen[mid] = t
        inst = getattr(m, 'index', None) if mid in (MSG_ENERGY_SYSTEM_INFO, MSG_ENERGY_SYSTEM_STATUS) else m.id
        self.by_instance[mid][(comp, inst)] = m
        if mid == MSG_BATTERY_CELL_STATUS:
            self.cell_instances.add((comp, m.id, m.cells_instance))
        # A source on another component (e.g. a smart battery): ask it for its _INFO once (as QGC does)
        if comp != 1 and mid in (MSG_BATTERY_STATUS_V2, MSG_BATTERY_CELL_STATUS):
            self._discover(comp, MSG_BATTERY_INFO)
        elif comp != 1 and mid == MSG_FUEL_STATUS_V2:
            self._discover(comp, MSG_FUEL_INFORMATION_V2)
        if self.verbose:
            print(f"[{t:6.1f}]   {m}", flush=True)

    def _discover(self, comp: int, info_id: int) -> None:
        key = (comp, info_id)
        if key in self.discovery_requested:
            return
        if not self.active:
            self.discovery_pending.add(key)
            return
        if any(c == comp for c, _ in self.by_instance.get(info_id, {})):
            return
        self.discovery_requested.add(key)
        self.command(MAV_CMD_REQUEST_MESSAGE, info_id, 0, target=comp)

    def activate(self) -> None:
        self.active = True
        for comp, info_id in sorted(self.discovery_pending):
            self._discover(comp, info_id)
        self.discovery_pending.clear()

    def wait_ack(self, cmd: int, req: int, timeout: float = 3.0) -> int | None:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self.acks.get((cmd, req)):
                return self.acks[(cmd, req)][-1]
            self.pump(0.05)
        print(f"[{self.now():6.1f}] no ACK for {msg_name(req)} within {timeout:g}s", flush=True)
        return None


# ---------------------------------------------------------------------------
# Connection flow (devguide: Energy Systems Protocol > Connection Flow)
# ---------------------------------------------------------------------------
def run_flow(gcs: FakeGcs, duration: float) -> None:
    print("--- Waiting for vehicle heartbeat ---", flush=True)
    end = time.monotonic() + 20
    while not gcs.vehicle_seen and time.monotonic() < end:
        gcs.pump(0.2)
    if not gcs.vehicle_seen:
        sys.exit("No vehicle heartbeat received (is energy_sim.py running?)")
    gcs.pump(1.0)

    print("--- 1. Discover and stream energy systems ---", flush=True)
    gcs.command(MAV_CMD_REQUEST_MESSAGE, MSG_ENERGY_SYSTEM_INFO, 0)
    if gcs.wait_ack(MAV_CMD_REQUEST_MESSAGE, MSG_ENERGY_SYSTEM_INFO) != 0:
        print("--- Energy systems not supported: legacy fallback (observing BATTERY_STATUS) ---", flush=True)
        gcs.pump(duration)
        return
    gcs.pump(1.0)
    gcs.activate()
    gcs.command(MAV_CMD_SET_MESSAGE_INTERVAL, MSG_ENERGY_SYSTEM_STATUS, 2_000_000)
    gcs.wait_ack(MAV_CMD_SET_MESSAGE_INTERVAL, MSG_ENERGY_SYSTEM_STATUS)

    print("--- 2. Discover and stream energy sources ---", flush=True)
    gcs.command(MAV_CMD_SET_MESSAGE_INTERVAL, MSG_BATTERY_STATUS, -1)
    gcs.wait_ack(MAV_CMD_SET_MESSAGE_INTERVAL, MSG_BATTERY_STATUS)
    for info_id, status_ids in ((MSG_BATTERY_INFO, (MSG_BATTERY_STATUS_V2, MSG_BATTERY_CELL_STATUS)),
                                (MSG_FUEL_INFORMATION_V2, (MSG_FUEL_STATUS_V2,))):
        gcs.command(MAV_CMD_REQUEST_MESSAGE, info_id, 0)
        if gcs.wait_ack(MAV_CMD_REQUEST_MESSAGE, info_id) != 0:
            continue
        gcs.pump(1.0)
        if any(comp == 1 for comp, _ in gcs.by_instance.get(info_id, {})):
            for sid in status_ids:
                gcs.command(MAV_CMD_SET_MESSAGE_INTERVAL, sid, 8_000_000)
                gcs.wait_ack(MAV_CMD_SET_MESSAGE_INTERVAL, sid)

    print(f"--- Observing for {duration:g}s ---", flush=True)
    gcs.pump(duration)


# ---------------------------------------------------------------------------
# Checks against the scenario
# ---------------------------------------------------------------------------
def check(gcs: FakeGcs, scenario) -> list[str]:
    failures: list[str] = []
    ok: list[str] = []

    def expect(cond: bool, text: str) -> None:
        (ok if cond else failures).append(text)

    b = scenario.behaviour
    expect(gcs.bad_frames == 0, f"no undecodable frames (got {gcs.bad_frames})")

    # Every instance must come from the component the scenario puts it on
    expected_keys = {
        MSG_ENERGY_SYSTEM_INFO: {es.key for es in scenario.energy_systems},
        MSG_ENERGY_SYSTEM_STATUS: {es.key for es in scenario.energy_systems},
        MSG_BATTERY_INFO: {bt.key for bt in scenario.batteries},
        MSG_BATTERY_STATUS_V2: {bt.key for bt in scenario.batteries},
        MSG_BATTERY_CELL_STATUS: {bt.key for bt in scenario.batteries if bt.cell_status},
        MSG_FUEL_INFORMATION_V2: {f.key for f in scenario.fuel},
        MSG_FUEL_STATUS_V2: {f.key for f in scenario.fuel},
        MSG_BATTERY_STATUS: {bt.key for bt in scenario.batteries if bt.component == 1},
    }
    for mid, got in sorted(gcs.by_instance.items()):
        unexpected = set(got) - expected_keys.get(mid, set())
        expect(not unexpected, f"{msg_name(mid)} only from expected (component, id) "
                               f"(unexpected: {sorted(unexpected)})")

    es_ack = gcs.acks.get((MAV_CMD_REQUEST_MESSAGE, MSG_ENERGY_SYSTEM_INFO), [None])[-1]
    want = 0 if b.energy_systems == 'supported' else 3
    expect(es_ack == want, f"REQUEST_MESSAGE(ENERGY_SYSTEM_INFO) ACK = {RESULT_NAMES.get(want)} "
                           f"(got {RESULT_NAMES.get(es_ack, es_ack)})")

    if b.energy_systems == 'supported':
        infos = gcs.by_instance.get(MSG_ENERGY_SYSTEM_INFO, {})
        want_systems = {es.key for es in scenario.energy_systems}
        expect(set(infos) == want_systems,
               f"ENERGY_SYSTEM_INFO (component, index) {sorted(infos)} == {sorted(want_systems)}")
        for es in scenario.energy_systems:
            m = infos.get(es.key)
            if m is None:
                continue
            expect(m.energy_systems == es.flags,
                   f"ES{es.index} energy_systems {m.energy_systems} == {es.flags}")
            # pymavlink decodes char[] as ASCII; the field is UTF-8 (and unterminated when
            # 25 bytes long), so decode the raw bytes pymavlink keeps alongside.
            name = bytes(m._energy_system_name_raw).split(b'\x00')[0].decode('utf-8', errors='replace')
            expect(name == es.name, f"ES{es.index} name {name!r} == {es.name!r}")
        statuses = gcs.by_instance.get(MSG_ENERGY_SYSTEM_STATUS, {})
        expect(set(statuses) == want_systems,
               f"ENERGY_SYSTEM_STATUS streamed for (component, index) {sorted(statuses)}")
        for key, m in statuses.items():
            pct = m.remaining_capacity
            expect(math.isnan(pct) or 0 <= pct <= 100, f"ES{key} remaining_capacity {pct} in [0,100] or NaN")

        # Sources. The flow component (1) follows `behaviour:`; other components (e.g. smart
        # batteries) answer _INFO requests and stream status if configured to.
        def streams_unprompted(item) -> bool:
            return item.component != 1 and scenario.components[item.component].stream_status

        for kind, info_id, status_id, items in (
                ('batteries', MSG_BATTERY_INFO, MSG_BATTERY_STATUS_V2, scenario.batteries),
                ('fuel', MSG_FUEL_INFORMATION_V2, MSG_FUEL_STATUS_V2, scenario.fuel)):
            support = getattr(b, kind)
            got_info = set(gcs.by_instance.get(info_id, {}))
            got_status = set(gcs.by_instance.get(status_id, {}))
            want_info = {s.key for s in items if s.component != 1 or support != 'unsupported'}
            want_status = {s.key for s in items
                           if streams_unprompted(s) or (s.component == 1 and support == 'supported')
                           or (s.component == 1 and status_id in b.stream_by_default)}
            expect(got_info == want_info, f"{msg_name(info_id)} (component, id) {sorted(got_info)} == {sorted(want_info)}")
            expect(got_status == want_status,
                   f"{msg_name(status_id)} (component, id) {sorted(got_status)} == {sorted(want_status)}")
            for s in items:
                m = gcs.by_instance.get(info_id, {}).get(s.key)
                if m is not None:
                    expect(m.energy_systems == s.energy_systems,
                           f"{msg_name(info_id)} {s.key} energy_systems {m.energy_systems} == {s.energy_systems}")
        want_cells = {(bt.component, bt.id, i) for bt in scenario.batteries if bt.cell_status
                      and (streams_unprompted(bt) or (bt.component == 1 and b.batteries == 'supported'))
                      for i in range(math.ceil(bt.cells / 6))}
        expect(gcs.cell_instances == want_cells,
               f"BATTERY_CELL_STATUS (component, id, cells_instance) {sorted(gcs.cell_instances)} == {sorted(want_cells)}")
        # Components other than the flow component were asked for their _INFO only once
        for (cmd, req, comp), results in sorted(gcs.comp_acks.items()):
            if comp != 1:
                expect(len(results) == 1, f"{msg_name(req)} requested from component {comp} once (got {len(results)})")

    # Streams requested with SET_MESSAGE_INTERVAL should start promptly. A ~5 s delay
    # means the command never reached the simulator and its fallback kicked in.
    for mid, sent_at in gcs.interval_sent_at.items():
        acks = gcs.acks.get((MAV_CMD_SET_MESSAGE_INTERVAL, mid), [])
        if mid in b.stream_by_default or not acks or acks[-1] != 0 or mid == MSG_BATTERY_STATUS:
            continue
        first = gcs.first_seen.get(mid)
        expect(first is not None and first - sent_at < 3.0,
               f"{msg_name(mid)} started within 3 s of SET_MESSAGE_INTERVAL "
               f"({'never' if first is None else f'{first - sent_at:.1f} s'})")

    # Legacy BATTERY_STATUS
    legacy = b.legacy_battery_status
    seen = MSG_BATTERY_STATUS in gcs.counts
    if legacy == 'off':
        expect(not seen, "no legacy BATTERY_STATUS")
    elif legacy == 'on' or b.energy_systems != 'supported':
        expect(seen, "legacy BATTERY_STATUS streamed")
        if legacy == 'on' or b.energy_systems != 'supported':
            expect(gcs.last_seen.get(MSG_BATTERY_STATUS, 0) > gcs.now() - 5,
                   "legacy BATTERY_STATUS still streaming at end")
    else:   # until_disabled, protocol supported -> GCS disabled it
        expect(gcs.last_seen.get(MSG_BATTERY_STATUS, 0) < gcs.now() - 5,
               "legacy BATTERY_STATUS stopped after SET_MESSAGE_INTERVAL(-1)")

    print("\n=== Checks ===")
    for line in ok:
        print(f"  PASS  {line}")
    for line in failures:
        print(f"  FAIL  {line}")
    print("\n=== Message counts ===")
    for mid, n in sorted(gcs.counts.items()):
        print(f"  {msg_name(mid):<22} {n}")
    return failures


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--scenario', '-s', help='Scenario file to check against')
    parser.add_argument('--port', type=int, default=14550)
    parser.add_argument('--duration', type=float, default=20.0, help='Observation time after the flow (s)')
    parser.add_argument('--listen-only', action='store_true', help="Don't run the flow; just decode")
    parser.add_argument('--verbose', '-v', action='store_true', help='Print every decoded energy message')
    args = parser.parse_args()

    scenario = None
    if args.scenario:
        p = Path(args.scenario)
        if not p.exists():
            p = ROOT / 'scenarios' / (args.scenario if args.scenario.endswith('.yaml') else f"{args.scenario}.yaml")
        scenario = load_scenario(p)

    gcs = FakeGcs(args.port, args.verbose)
    if args.listen_only:
        gcs.pump(args.duration)
    else:
        run_flow(gcs, args.duration)
    if scenario is None:
        for mid, n in sorted(gcs.counts.items()):
            print(f"  {msg_name(mid):<22} {n}")
        sys.exit(1 if gcs.bad_frames else 0)
    failures = check(gcs, scenario)
    print(f"\n{'FAILED' if failures else 'PASSED'}: {scenario.name}")
    sys.exit(1 if failures else 0)


if __name__ == '__main__':
    main()
