"""
scenario.py - Load and validate energy-system scenario YAML files.

A scenario describes the energy systems and energy sources (batteries, fuel tanks)
a simulated vehicle has, how their levels change over time, and how the vehicle
responds to the GCS discovery/streaming requests of the Energy Systems Protocol.
See README.md for the full schema.

Flag and enum values may be given as names (prefix optional, e.g. `PROPULSION` or
`MAV_ENERGY_SYSTEM_FLAGS_PROPULSION`) or as integers.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass, field
from pathlib import Path

import yaml

# ---------------------------------------------------------------------------
# Enum/flag tables (mavlink/mavlink#2526, head 2d4dd92)
# ---------------------------------------------------------------------------
MAV_ENERGY_SYSTEM_FLAGS = {
    'PROPULSION':          1,
    'AVIONICS':            2,
    'PAYLOAD':             4,
    'PROPULSION_VTOL_MC':  8,
    'PROPULSION_VTOL_FW':  16,
    'USER_DEFINED1':       32,
    'USER_DEFINED2':       64,
    'USER_DEFINED3':       128,
    'USER_DEFINED4':       256,
}

MAV_ENERGY_STATUS_FLAGS = {
    'LOW':             1,
    'CRITICAL':        2,
    'EMERGENCY':       4,
    'INACTIVE':        8,
    'UNAVAILABLE':     16,
    'FLIGHT_CRITICAL': 32,
}
SEVERITY_MASK = (MAV_ENERGY_STATUS_FLAGS['LOW'] | MAV_ENERGY_STATUS_FLAGS['CRITICAL']
                 | MAV_ENERGY_STATUS_FLAGS['EMERGENCY'])

MAV_BATTERY_STATUS_FLAGS = {
    'NOT_READY_TO_USE':                    1,
    'CHARGING':                            2,
    'CELL_BALANCING':                      4,
    'FAULT_CELL_IMBALANCE':                8,
    'AUTO_DISCHARGING':                    16,
    'REQUIRES_SERVICE':                    32,
    'BAD_BATTERY':                         64,
    'PROTECTIONS_ENABLED':                 128,
    'FAULT_PROTECTION_SYSTEM':             256,
    'FAULT_OVER_VOLT':                     512,
    'FAULT_UNDER_VOLT':                    1024,
    'FAULT_OVER_TEMPERATURE':              2048,
    'FAULT_UNDER_TEMPERATURE':             4096,
    'FAULT_OVER_CURRENT':                  8192,
    'FAULT_SHORT_CIRCUIT':                 16384,
    'FAULT_INCOMPATIBLE_VOLTAGE':          32768,
    'FAULT_INCOMPATIBLE_FIRMWARE':         65536,
    'FAULT_INCOMPATIBLE_CELLS_CONFIGURATION': 131072,
    'TRACKS_CHARGE':                       262144,
}

MAV_FUEL_STATUS_FLAGS = {
    'NOT_READY_TO_USE': 1,
    'TANK_EMPTY':       2,
    'LEAK_DETECTED':    4,
    'SENSOR_FAULT':     8,
}

MAV_FUEL_TYPE = {'UNKNOWN': 0, 'LIQUID': 1, 'GAS': 2}
MAV_BATTERY_TYPE = {'UNKNOWN': 0, 'LIPO': 1, 'LIFE': 2, 'LION': 3, 'NIMH': 4}
MAV_BATTERY_FUNCTION = {'UNKNOWN': 0, 'ALL': 1, 'PROPULSION': 2, 'AVIONICS': 3, 'PAYLOAD': 4}

_ENUM_PREFIXES = {
    id(MAV_ENERGY_SYSTEM_FLAGS):  'MAV_ENERGY_SYSTEM_FLAGS_',
    id(MAV_ENERGY_STATUS_FLAGS):  'MAV_ENERGY_STATUS_FLAGS_',
    id(MAV_BATTERY_STATUS_FLAGS): 'MAV_BATTERY_STATUS_FLAGS_',
    id(MAV_FUEL_STATUS_FLAGS):    'MAV_FUEL_STATUS_FLAGS_',
    id(MAV_FUEL_TYPE):            'MAV_FUEL_TYPE_',
    id(MAV_BATTERY_TYPE):         'MAV_BATTERY_TYPE_',
    id(MAV_BATTERY_FUNCTION):     'MAV_BATTERY_FUNCTION_',
}

# Messages that can be named in behaviour.stream_by_default
STREAMABLE = {
    'ENERGY_SYSTEM_STATUS': 366,
    'BATTERY_STATUS_V2':    369,
    'BATTERY_CELL_STATUS':  364,
    'FUEL_STATUS_V2':       368,
    'BATTERY_STATUS':       147,
    'ENERGY_SYSTEM_INFO':   365,
    'BATTERY_INFO':         372,
    'FUEL_INFORMATION_V2':  367,
}

SUPPORT_VALUES = ('supported', 'unsupported', 'info_only')
LEGACY_VALUES = ('off', 'on', 'until_disabled')
AGGREGATES = ('min', 'weighted', 'explicit')


# Component the GCS runs the connection flow against (the autopilot). Energy systems and
# sources default to it; others (e.g. a smart battery on MAV_COMP_ID_BATTERY = 180) behave as an
# independent device: status streams unprompted, _INFO is sent when that component is asked.
FLOW_COMPONENT = 1


class ScenarioError(ValueError):
    pass


def _enum_value(table: dict, value, where: str) -> int:
    if isinstance(value, bool):
        raise ScenarioError(f"{where}: expected a name or integer, got {value!r}")
    if isinstance(value, int):
        return value
    if isinstance(value, str):
        name = value.strip().upper()
        prefix = _ENUM_PREFIXES[id(table)]
        if name.startswith(prefix):
            name = name[len(prefix):]
        if name in table:
            return table[name]
        raise ScenarioError(f"{where}: unknown value {value!r} (valid: {', '.join(table)})")
    raise ScenarioError(f"{where}: expected a name or integer, got {value!r}")


def _flags(table: dict, value, where: str) -> int:
    """Flags may be a single name/int or a list of them; they are OR-ed together."""
    if value is None:
        return 0
    if not isinstance(value, list):
        value = [value]
    result = 0
    for v in value:
        result |= _enum_value(table, v, where)
    return result


def flag_names(table: dict, value: int) -> str:
    names = [n for n, v in table.items() if value & v]
    known = 0
    for v in table.values():
        known |= v
    if value & ~known:
        names.append(f"0x{value & ~known:x}")
    return '|'.join(names) if names else '0'


def parse_duration(value, where: str) -> float:
    """Seconds as a number, or a string such as '30s', '2m', '1m30s', '500ms'."""
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return float(value)
    if isinstance(value, str):
        text = value.strip().lower().replace(' ', '')
        parts = re.findall(r'(\d+(?:\.\d+)?)(ms|s|m|h)', text)
        if parts and ''.join(n + u for n, u in parts) == text:
            scale = {'ms': 0.001, 's': 1.0, 'm': 60.0, 'h': 3600.0}
            return sum(float(n) * scale[u] for n, u in parts)
    raise ScenarioError(f"{where}: invalid duration {value!r} (e.g. 30, '30s', '2m', '1m30s')")


def _check_keys(d: dict, allowed: set, where: str) -> None:
    unknown = set(d) - allowed
    if unknown:
        raise ScenarioError(f"{where}: unknown key(s) {sorted(unknown)} (allowed: {sorted(allowed)})")


# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------
@dataclass
class Profile:
    """Linear level change from `start` to `end` % over `period_s`, optionally looping."""
    start: float = 95.0
    end: float = 5.0
    period_s: float = 180.0
    loop: bool = True

    def level(self, t: float) -> float:
        if self.period_s <= 0:
            return self.end
        if self.loop:
            t = t % (self.period_s * 1.05)       # brief hold at `end` before restarting
        frac = min(1.0, max(0.0, t / self.period_s))
        return self.start + (self.end - self.start) * frac


def _profile(d, where: str, default: Profile | None = None) -> Profile:
    if d is None:
        return default if default is not None else Profile()
    if isinstance(d, (int, float)) and not isinstance(d, bool):
        return Profile(start=float(d), end=float(d), period_s=1.0, loop=False)  # constant
    if not isinstance(d, dict):
        raise ScenarioError(f"{where}: profile must be a mapping or a constant number")
    _check_keys(d, {'start', 'end', 'period', 'period_s', 'loop'}, where)
    period = d.get('period_s', d.get('period', 180))
    p = Profile(start=float(d.get('start', 95)), end=float(d.get('end', 5)),
                period_s=parse_duration(period, f"{where}.period"),
                loop=bool(d.get('loop', True)))
    for name in ('start', 'end'):
        v = getattr(p, name)
        if not 0 <= v <= 100:
            raise ScenarioError(f"{where}.{name}: must be in [0, 100], got {v}")
    return p


@dataclass
class Thresholds:
    low: float = 25.0
    critical: float = 15.0
    emergency: float = 5.0


@dataclass
class EnergySystem:
    index: int
    flags: int
    name: str = ''
    aggregate: str = 'min'
    profile: Profile | None = None
    thresholds: Thresholds = field(default_factory=Thresholds)
    status_flags: int = 0                 # static flags (INACTIVE, UNAVAILABLE, FLIGHT_CRITICAL, ...)
    auto_severity: bool = True            # derive LOW/CRITICAL/EMERGENCY from thresholds
    endurance_s: float | None = None      # time remaining at 100 %; None -> remaining_time NaN
    report_percent: bool = True           # False -> remaining_capacity NaN
    component: int = FLOW_COMPONENT       # MAVLink component that reports it

    @property
    def key(self) -> tuple[int, int]:
        return (self.component, self.index)


@dataclass
class Battery:
    id: int
    energy_systems: int
    capacity_ah: float = 5.0
    cells: int = 4
    current_a: float = 15.0
    profile: Profile = field(default_factory=Profile)
    status_flags: int = 0
    cell_status: bool = False
    weight: float | None = None           # for `weighted` aggregation (default capacity_ah)
    report_level: bool = True             # False -> state_of_charge UINT16_MAX (not provided)
    info: dict = field(default_factory=dict)  # BATTERY_INFO field overrides
    component: int = FLOW_COMPONENT       # MAVLink component that reports it (e.g. 180 for a smart battery)

    @property
    def key(self) -> tuple[int, int]:
        return (self.component, self.id)


@dataclass
class Fuel:
    id: int
    energy_systems: int
    fuel_type: int = 1
    maximum_fuel: float = 20000.0      # units per fuel_type: ml (LIQUID), kPa (GAS), 1 (UNKNOWN)
    flow_rate: float | None = None        # None -> derived from profile
    temperature_k: float = 293.15
    profile: Profile = field(default_factory=Profile)
    status_flags: int = 0
    weight: float | None = None           # for `weighted` aggregation (default maximum_fuel)
    report_level: bool = True             # False -> percent_remaining UINT16_MAX (not provided)
    component: int = FLOW_COMPONENT       # MAVLink component that reports it

    @property
    def key(self) -> tuple[int, int]:
        return (self.component, self.id)


@dataclass
class Behaviour:
    energy_systems: str = 'supported'     # supported | unsupported
    batteries: str = 'supported'          # supported | unsupported | info_only
    fuel: str = 'supported'               # supported | unsupported | info_only
    stream_by_default: dict = field(default_factory=dict)   # msg_id -> interval_s
    legacy_battery_status: str = 'off'    # off | on | until_disabled
    fallback_timeout_s: float | None = 5.0  # see protocol.py; None disables


@dataclass
class TimelineEvent:
    at: float
    target: str                 # 'es' | 'battery' | 'fuel'
    instance: int
    component: int | None       # None: whichever component has that instance (must be unique)
    changes: dict
    repeat: float | None = None
    description: str = ''


@dataclass
class Component:
    """How a component other than the flow component (e.g. a smart battery) behaves."""
    id: int
    heartbeat: bool = False         # send HEARTBEAT with MAV_TYPE_BATTERY (36) at 1 Hz
    stream_status: bool = True      # stream its status messages unprompted (else only on request)


@dataclass
class Scenario:
    name: str
    description: str
    behaviour: Behaviour
    energy_systems: list[EnergySystem]
    batteries: list[Battery]
    fuel: list[Fuel]
    timeline: list[TimelineEvent]
    components: dict = field(default_factory=dict)   # component id -> Component (non-flow components)
    path: Path | None = None


# Nominal stream intervals (seconds) when a message streams by default, and the
# rates the docs suggest a GCS request (used by the fallback in protocol.py).
DEFAULT_STREAM_INTERVAL_S = {
    366: 2.0,    # ENERGY_SYSTEM_STATUS
    369: 1.0,    # BATTERY_STATUS_V2 (XML: nominally 1 Hz)
    364: 1.0,    # BATTERY_CELL_STATUS
    368: 1.0,    # FUEL_STATUS_V2 (XML: nominally 1 Hz)
    147: 2.0,    # BATTERY_STATUS (legacy)
    365: 10.0,   # _INFO messages: very low rate if streamed at all
    372: 10.0,
    367: 10.0,
}
DOC_REQUEST_INTERVAL_S = {366: 2.0, 369: 8.0, 364: 8.0, 368: 8.0}


# ---------------------------------------------------------------------------
# Parsing
# ---------------------------------------------------------------------------
def _behaviour(d) -> Behaviour:
    if d is None:
        return Behaviour()
    where = 'behaviour'
    _check_keys(d, {'energy_systems', 'sources', 'batteries', 'fuel', 'stream_by_default',
                    'legacy_battery_status', 'fallback_timeout'}, where)
    b = Behaviour()
    es = d.get('energy_systems', 'supported')
    if es not in ('supported', 'unsupported'):
        raise ScenarioError(f"{where}.energy_systems: must be supported|unsupported")
    b.energy_systems = es
    sources = d.get('sources', 'supported')
    if sources not in SUPPORT_VALUES:
        raise ScenarioError(f"{where}.sources: must be one of {SUPPORT_VALUES}")
    b.batteries = d.get('batteries', sources)
    b.fuel = d.get('fuel', sources)
    for k in ('batteries', 'fuel'):
        if getattr(b, k) not in SUPPORT_VALUES:
            raise ScenarioError(f"{where}.{k}: must be one of {SUPPORT_VALUES}")
    legacy = d.get('legacy_battery_status', 'off')
    if isinstance(legacy, bool):    # YAML 1.1 reads bare on/off as booleans
        legacy = 'on' if legacy else 'off'
    if legacy not in LEGACY_VALUES:
        raise ScenarioError(f"{where}.legacy_battery_status: must be one of {LEGACY_VALUES}")
    b.legacy_battery_status = legacy
    sbd = d.get('stream_by_default') or []
    if isinstance(sbd, list):
        sbd = {name: None for name in sbd}
    if not isinstance(sbd, dict):
        raise ScenarioError(f"{where}.stream_by_default: must be a list of message names or a "
                            "mapping of name -> interval")
    for name, interval in sbd.items():
        if name not in STREAMABLE:
            raise ScenarioError(f"{where}.stream_by_default: unknown message {name!r} "
                                f"(valid: {', '.join(STREAMABLE)})")
        msg_id = STREAMABLE[name]
        b.stream_by_default[msg_id] = (DEFAULT_STREAM_INTERVAL_S[msg_id] if interval is None
                                       else parse_duration(interval, f"{where}.stream_by_default.{name}"))
    if 'fallback_timeout' in d:
        ft = d['fallback_timeout']
        b.fallback_timeout_s = None if ft in (None, False, 'off') else parse_duration(ft, f"{where}.fallback_timeout")
    return b


def _energy_system(d, i: int) -> EnergySystem:
    where = f"energy_systems[{i}]"
    if not isinstance(d, dict):
        raise ScenarioError(f"{where}: must be a mapping")
    _check_keys(d, {'index', 'flags', 'name', 'aggregate', 'profile', 'thresholds', 'status_flags',
                    'auto_severity', 'endurance', 'endurance_s', 'report_percent', 'component'}, where)
    if 'index' not in d or 'flags' not in d:
        raise ScenarioError(f"{where}: 'index' and 'flags' are required")
    es = EnergySystem(index=int(d['index']),
                      flags=_flags(MAV_ENERGY_SYSTEM_FLAGS, d['flags'], f"{where}.flags"))
    if es.flags == 0:
        raise ScenarioError(f"{where}.flags: ENERGY_SYSTEM_* must never be emitted with a zero value")
    es.name = str(d.get('name', '') or '')
    if len(es.name.encode('utf-8')) > 25:
        raise ScenarioError(f"{where}.name: {len(es.name.encode('utf-8'))} bytes; max 25 (UTF-8)")
    es.aggregate = d.get('aggregate', 'explicit' if 'profile' in d else 'min')
    if es.aggregate not in AGGREGATES:
        raise ScenarioError(f"{where}.aggregate: must be one of {AGGREGATES}")
    if es.aggregate == 'explicit':
        es.profile = _profile(d.get('profile'), f"{where}.profile")
    elif 'profile' in d:
        raise ScenarioError(f"{where}: 'profile' is only used with aggregate: explicit")
    th = d.get('thresholds') or {}
    _check_keys(th, {'low', 'critical', 'emergency'}, f"{where}.thresholds")
    es.thresholds = Thresholds(low=float(th.get('low', 25)), critical=float(th.get('critical', 15)),
                               emergency=float(th.get('emergency', 5)))
    es.status_flags = _flags(MAV_ENERGY_STATUS_FLAGS, d.get('status_flags'), f"{where}.status_flags")
    es.auto_severity = bool(d.get('auto_severity', not (es.status_flags & SEVERITY_MASK)))
    end = d.get('endurance_s', d.get('endurance'))
    es.endurance_s = None if end is None else parse_duration(end, f"{where}.endurance")
    es.report_percent = bool(d.get('report_percent', True))
    es.component = _component(d, where)
    return es


def _component(d: dict, where: str) -> int:
    component = d.get('component', FLOW_COMPONENT)
    if isinstance(component, bool) or not isinstance(component, int) or not 1 <= component <= 255:
        raise ScenarioError(f"{where}.component: must be a MAVLink component id 1-255")
    return component


def _battery(d, i: int) -> Battery:
    where = f"sources.batteries[{i}]"
    if not isinstance(d, dict):
        raise ScenarioError(f"{where}: must be a mapping")
    _check_keys(d, {'id', 'energy_systems', 'capacity_ah', 'cells', 'current_a', 'profile',
                    'status_flags', 'cell_status', 'weight', 'report_level', 'info', 'component'}, where)
    if 'id' not in d:
        raise ScenarioError(f"{where}: 'id' is required")
    b = Battery(id=int(d['id']),
                energy_systems=_flags(MAV_ENERGY_SYSTEM_FLAGS, d.get('energy_systems'),
                                      f"{where}.energy_systems"))
    b.capacity_ah = float(d.get('capacity_ah', b.capacity_ah))
    b.cells = int(d.get('cells', b.cells))
    b.current_a = float(d.get('current_a', b.current_a))
    b.profile = _profile(d.get('profile'), f"{where}.profile")
    b.status_flags = _flags(MAV_BATTERY_STATUS_FLAGS, d.get('status_flags'), f"{where}.status_flags")
    b.cell_status = bool(d.get('cell_status', False))
    b.weight = None if d.get('weight') is None else float(d['weight'])
    b.report_level = bool(d.get('report_level', True))
    b.component = _component(d, where)
    info = dict(d.get('info') or {})
    if 'type' in info:
        info['type'] = _enum_value(MAV_BATTERY_TYPE, info['type'], f"{where}.info.type")
    if 'battery_function' in info:
        info['battery_function'] = _enum_value(MAV_BATTERY_FUNCTION, info['battery_function'],
                                               f"{where}.info.battery_function")
    b.info = info
    return b


def _fuel(d, i: int) -> Fuel:
    where = f"sources.fuel[{i}]"
    if not isinstance(d, dict):
        raise ScenarioError(f"{where}: must be a mapping")
    _check_keys(d, {'id', 'energy_systems', 'fuel_type', 'maximum_fuel', 'flow_rate',
                    'temperature_k', 'profile', 'status_flags', 'weight', 'report_level', 'component'}, where)
    if 'id' not in d:
        raise ScenarioError(f"{where}: 'id' is required")
    f = Fuel(id=int(d['id']),
             energy_systems=_flags(MAV_ENERGY_SYSTEM_FLAGS, d.get('energy_systems'),
                                   f"{where}.energy_systems"))
    f.fuel_type = _enum_value(MAV_FUEL_TYPE, d.get('fuel_type', 'LIQUID'), f"{where}.fuel_type")
    f.maximum_fuel = float(d.get('maximum_fuel', f.maximum_fuel))
    f.flow_rate = None if d.get('flow_rate') is None else float(d['flow_rate'])
    f.temperature_k = float(d.get('temperature_k', f.temperature_k))
    f.profile = _profile(d.get('profile'), f"{where}.profile")
    f.status_flags = _flags(MAV_FUEL_STATUS_FLAGS, d.get('status_flags'), f"{where}.status_flags")
    f.weight = None if d.get('weight') is None else float(d['weight'])
    f.report_level = bool(d.get('report_level', True))
    f.component = _component(d, where)
    return f


_TIMELINE_KEYS = {
    'es':      {'status_flags', 'add_flags', 'clear_flags', 'auto_severity', 'profile',
                'report_percent', 'endurance'},
    'battery': {'status_flags', 'add_flags', 'clear_flags', 'profile', 'current_a'},
    'fuel':    {'status_flags', 'add_flags', 'clear_flags', 'profile', 'flow_rate'},
}
_TIMELINE_FLAG_TABLES = {'es': MAV_ENERGY_STATUS_FLAGS, 'battery': MAV_BATTERY_STATUS_FLAGS,
                         'fuel': MAV_FUEL_STATUS_FLAGS}


def _timeline_event(d, i: int) -> list[TimelineEvent]:
    where = f"timeline[{i}]"
    if not isinstance(d, dict):
        raise ScenarioError(f"{where}: must be a mapping")
    _check_keys(d, {'at', 'set', 'repeat', 'description'}, where)
    if 'at' not in d or 'set' not in d:
        raise ScenarioError(f"{where}: 'at' and 'set' are required")
    at = parse_duration(d['at'], f"{where}.at")
    repeat = None if d.get('repeat') is None else parse_duration(d['repeat'], f"{where}.repeat")
    sets = d['set'] if isinstance(d['set'], list) else [d['set']]
    events = []
    for j, s in enumerate(sets):
        w = f"{where}.set[{j}]"
        targets = [k for k in ('es', 'battery', 'fuel') if k in s]
        if len(targets) != 1:
            raise ScenarioError(f"{w}: must name exactly one of es/battery/fuel")
        target = targets[0]
        component = s.get('component')
        changes = {k: v for k, v in s.items() if k not in (target, 'component')}
        _check_keys(changes, _TIMELINE_KEYS[target], w)
        table = _TIMELINE_FLAG_TABLES[target]
        for k in ('status_flags', 'add_flags', 'clear_flags'):
            if k in changes:
                changes[k] = _flags(table, changes[k], f"{w}.{k}")
        if 'profile' in changes:
            changes['profile'] = _profile(changes['profile'], f"{w}.profile")
        if 'endurance' in changes:
            changes['endurance'] = (None if changes['endurance'] is None
                                    else parse_duration(changes['endurance'], f"{w}.endurance"))
        events.append(TimelineEvent(at=at, target=target, instance=int(s[target]), component=component,
                                    changes=changes, repeat=repeat, description=str(d.get('description', ''))))
    return events


def parse_scenario(data: dict, path: Path | None = None) -> Scenario:
    if not isinstance(data, dict):
        raise ScenarioError("scenario file must contain a mapping")
    _check_keys(data, {'name', 'description', 'behaviour', 'energy_systems', 'sources', 'timeline',
                       'components'}, 'scenario')
    name = str(data.get('name') or (path.stem if path else 'scenario'))
    behaviour = _behaviour(data.get('behaviour'))
    systems = [_energy_system(d, i) for i, d in enumerate(data.get('energy_systems') or [])]
    sources = data.get('sources') or {}
    _check_keys(sources, {'batteries', 'fuel'}, 'sources')
    batteries = [_battery(d, i) for i, d in enumerate(sources.get('batteries') or [])]
    fuel = [_fuel(d, i) for i, d in enumerate(sources.get('fuel') or [])]
    timeline = [e for i, d in enumerate(data.get('timeline') or []) for e in _timeline_event(d, i)]

    # Cross-checks
    # Instance ids are unique per component (a smart battery may reuse the autopilot's battery id)
    for kind, items, key in (('energy_systems', systems, 'index'), ('batteries', batteries, 'id'),
                             ('fuel', fuel, 'id')):
        seen = set()
        for it in items:
            v = getattr(it, key)
            if not 0 <= v <= 255:
                raise ScenarioError(f"{kind}: {key} {v} out of range 0-255")
            if it.key in seen:
                raise ScenarioError(f"{kind}: duplicate {key} {v} on component {it.component}")
            seen.add(it.key)
    if behaviour.energy_systems == 'supported' and not systems:
        raise ScenarioError("no energy_systems defined (use behaviour.energy_systems: unsupported "
                            "for a legacy-only vehicle)")
    for es in systems:
        if es.aggregate != 'explicit':
            matches = [s for s in batteries + fuel if s.energy_systems & es.flags]
            if not matches:
                raise ScenarioError(f"energy system {es.index}: aggregate '{es.aggregate}' but no "
                                    "source shares any energy_systems bit; use aggregate: explicit "
                                    "with a profile")
    keys = {'es': [e.key for e in systems], 'battery': [b.key for b in batteries], 'fuel': [f.key for f in fuel]}
    for ev in timeline:
        matches = [k for k in keys[ev.target]
                   if k[1] == ev.instance and (ev.component is None or k[0] == ev.component)]
        if not matches:
            raise ScenarioError(f"timeline: {ev.target} {ev.instance}"
                                f"{'' if ev.component is None else f' on component {ev.component}'} is not defined")
        if len(matches) > 1:
            raise ScenarioError(f"timeline: {ev.target} {ev.instance} exists on several components; "
                                "add 'component:'")
        ev.component = matches[0][0]
    if not [b for b in batteries if b.component == FLOW_COMPONENT] and behaviour.legacy_battery_status != 'off':
        raise ScenarioError("legacy_battery_status needs at least one battery to report")

    # Components other than the flow component: defaults, plus overrides from `components:`
    components = {item.component: Component(item.component)
                  for item in systems + batteries + fuel if item.component != FLOW_COMPONENT}
    for comp_id, cfg in (data.get('components') or {}).items():
        where = f"components.{comp_id}"
        if not isinstance(comp_id, int) or comp_id not in components:
            raise ScenarioError(f"{where}: not a component used by any energy system or source "
                                f"(used: {sorted(components)})")
        cfg = cfg or {}
        _check_keys(cfg, {'heartbeat', 'stream_status'}, where)
        components[comp_id] = Component(comp_id, heartbeat=bool(cfg.get('heartbeat', False)),
                                        stream_status=bool(cfg.get('stream_status', True)))

    return Scenario(name=name, description=str(data.get('description', '')).strip(),
                    behaviour=behaviour, energy_systems=systems, batteries=batteries, fuel=fuel,
                    timeline=timeline, components=components, path=path)


def load_scenario(path: str | Path) -> Scenario:
    path = Path(path)
    with open(path, encoding='utf-8') as f:
        data = yaml.safe_load(f)
    try:
        return parse_scenario(data, path)
    except ScenarioError as exc:
        raise ScenarioError(f"{path}: {exc}") from None


def is_nan(v) -> bool:
    return isinstance(v, float) and math.isnan(v)
