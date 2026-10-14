"""
model.py - Simulated energy state for a scenario.

Computes the field values of every energy-system and energy-source message at a
given simulation time, and applies scenario timeline events.

Energy-system status is produced by the emitter's own algorithm (the protocol does
not mandate one). Each energy system picks one with `aggregate`:

- min:      worst case over the matching sources (as PX4 does across batteries):
            lowest level, and so lowest time remaining and worst severity.
- weighted: pooled capacity (as ArduPilot's Sum battery monitor does):
            sum(level * weight) / sum(weight), weight defaulting to capacity.
- explicit: the system has its own profile, independent of any sources (required
            when the vehicle has no source to derive it from).

A source matches an energy system when (source.energy_systems & system.flags) != 0, whichever
component reports either of them.

Energy systems and sources are keyed by (component, index/id): instance ids are only unique
within a component (a smart battery may use the same id as an autopilot power monitor).
"""

from __future__ import annotations

import copy
import math

from scenario import (MAV_ENERGY_STATUS_FLAGS, SEVERITY_MASK, Battery, EnergySystem, Fuel,
                      Profile, Scenario, TimelineEvent)

NAN = float('nan')
UINT16_MAX = 65535
INT16_MAX = 32767

_LOW = MAV_ENERGY_STATUS_FLAGS['LOW']
_CRITICAL = MAV_ENERGY_STATUS_FLAGS['CRITICAL']
_EMERGENCY = MAV_ENERGY_STATUS_FLAGS['EMERGENCY']
_INACTIVE = MAV_ENERGY_STATUS_FLAGS['INACTIVE']


class _Live:
    """Mutable runtime copy of a scenario item; profiles restart when replaced by the timeline."""

    def __init__(self, item):
        self.cfg = copy.deepcopy(item)
        self.profile_t0 = 0.0

    def level(self, t: float, profile: Profile) -> float:
        return profile.level(max(0.0, t - self.profile_t0))


class EnergyModel:
    def __init__(self, scenario: Scenario):
        self.scenario = scenario
        self.systems = {es.key: _Live(es) for es in scenario.energy_systems}
        self.batteries = {b.key: _Live(b) for b in scenario.batteries}
        self.fuel = {f.key: _Live(f) for f in scenario.fuel}
        # (next_fire_time, event) list
        self._pending = sorted(((ev.at, i, ev) for i, ev in enumerate(scenario.timeline)),
                               key=lambda x: (x[0], x[1]))

    # ------------------------------------------------------------------
    # Timeline
    # ------------------------------------------------------------------
    def apply_timeline(self, t: float) -> list[str]:
        """Apply all events due at or before t. Returns log lines."""
        logs = []
        while self._pending and self._pending[0][0] <= t:
            fire_at, order, ev = self._pending.pop(0)
            logs.append(self._apply(ev, fire_at))
            if ev.repeat:
                self._pending.append((fire_at + ev.repeat, order, ev))
                self._pending.sort(key=lambda x: (x[0], x[1]))
        return logs

    def _apply(self, ev: TimelineEvent, t: float) -> str:
        live = {'es': self.systems, 'battery': self.batteries, 'fuel': self.fuel}[ev.target][(ev.component, ev.instance)]
        cfg = live.cfg
        for key, value in ev.changes.items():
            if key == 'status_flags':
                cfg.status_flags = value
            elif key == 'add_flags':
                cfg.status_flags |= value
            elif key == 'clear_flags':
                cfg.status_flags &= ~value
            elif key == 'profile':
                if ev.target == 'es':
                    cfg.aggregate = 'explicit'
                cfg.profile = value
                live.profile_t0 = t
            elif key == 'endurance':
                cfg.endurance_s = value
            elif key == 'auto_severity':
                cfg.auto_severity = bool(value)
            elif key == 'report_percent':
                cfg.report_percent = bool(value)
            elif key == 'current_a':
                cfg.current_a = float(value)
            elif key == 'flow_rate':
                cfg.flow_rate = None if value is None else float(value)
        desc = f" ({ev.description})" if ev.description else ''
        where = '' if ev.component == 1 else f" (component {ev.component})"
        return f"[TIMELINE t={t:.0f}s] {ev.target} {ev.instance}{where}: {_fmt_changes(ev.changes)}{desc}"

    # ------------------------------------------------------------------
    # Sources
    # ------------------------------------------------------------------
    def battery_level(self, bid: tuple, t: float) -> float:
        live = self.batteries[bid]
        return live.level(t, live.cfg.profile)

    def fuel_level(self, fid: tuple, t: float) -> float:
        live = self.fuel[fid]
        return live.level(t, live.cfg.profile)

    def battery_info_fields(self, bid: tuple) -> dict:
        b: Battery = self.batteries[bid].cfg
        fields = {
            'id':                              b.id,
            'battery_function':                0,          # MAV_BATTERY_FUNCTION_UNKNOWN; see energy_systems
            'type':                            1,          # MAV_BATTERY_TYPE_LIPO
            'state_of_health':                 92,
            'cells_in_series':                 b.cells,
            'cycle_count':                     47,
            'weight':                          60 * b.cells,
            'discharge_minimum_voltage':       3.3,
            'charging_minimum_voltage':        3.0,
            'resting_minimum_voltage':         3.5,
            'charging_maximum_voltage':        4.2,
            'charging_maximum_current':        b.capacity_ah * 2,
            'nominal_voltage':                 3.7 * b.cells,
            'discharge_maximum_current':       b.capacity_ah * 20,
            'discharge_maximum_burst_current': b.capacity_ah * 40,
            'design_capacity':                 b.capacity_ah,
            'full_charge_capacity':            round(b.capacity_ah * 0.95, 3),
            'manufacture_date':                '01012025',
            'serial_number':                   f'SN-{1000 + b.id}',
            'name':                            f'Acme_LiPo{b.cells}S{int(b.capacity_ah * 1000)}',
            'energy_systems':                  b.energy_systems,
        }
        fields.update(b.info)
        return fields

    def battery_status_fields(self, bid: tuple, t: float) -> dict:
        b: Battery = self.batteries[bid].cfg
        level = self.battery_level(bid, t)
        voltage = b.cells * (3.5 + 0.7 * level / 100.0)
        return {
            'id':              b.id,
            'temperature':     2800 + 5 * b.id,
            'voltage':         round(voltage, 3),
            'current':         b.current_a,
            'state_of_charge': _pct_u16(level) if b.report_level else UINT16_MAX,
            'status_flags':    b.status_flags,
        }

    def battery_cell_fields(self, bid: tuple, t: float) -> list[dict]:
        """One BATTERY_CELL_STATUS per 6 cells (cells_instance 0, 1, ...)."""
        b: Battery = self.batteries[bid].cfg
        level = self.battery_level(bid, t)
        base_mv = 1000 * (3.5 + 0.7 * level / 100.0)
        cells = [int(base_mv + ((i * 7) % 11) - 5) for i in range(b.cells)]
        msgs = []
        for inst in range(max(1, math.ceil(b.cells / 6))):
            chunk = cells[inst * 6:(inst + 1) * 6]
            chunk += [UINT16_MAX] * (6 - len(chunk))
            msgs.append({'id': b.id, 'cells_instance': inst, 'voltages': chunk})
        return msgs

    def fuel_info_fields(self, fid: tuple) -> dict:
        f: Fuel = self.fuel[fid].cfg
        return {
            'id':             f.id,
            'energy_systems': f.energy_systems,
            'fuel_type':      f.fuel_type,
            'maximum_fuel':   f.maximum_fuel,
        }

    def fuel_status_fields(self, fid: tuple, t: float) -> dict:
        f: Fuel = self.fuel[fid].cfg
        level = self.fuel_level(fid, t)
        if f.flow_rate is not None:
            flow = f.flow_rate
        else:
            p = f.profile
            flow = (p.start - p.end) / 100.0 * f.maximum_fuel / p.period_s if p.period_s > 0 else 0.0
        return {
            'id':                f.id,
            'percent_remaining': _pct_u16(level) if f.report_level else UINT16_MAX,
            'flow_rate':         round(flow, 5),
            'temperature':       f.temperature_k,
            'status_flags':      f.status_flags,
        }

    # ------------------------------------------------------------------
    # Energy systems
    # ------------------------------------------------------------------
    def sources_for(self, index: tuple[int, int]) -> tuple[list, list]:
        """Keys of the batteries and fuel sources (any component) that feed energy system `index` (a key)."""
        mask = self.systems[index].cfg.flags
        return ([bid for bid, b in self.batteries.items() if b.cfg.energy_systems & mask],
                [fid for fid, f in self.fuel.items() if f.cfg.energy_systems & mask])

    def system_level(self, index: tuple, t: float) -> float:
        live = self.systems[index]
        es: EnergySystem = live.cfg
        if es.aggregate == 'explicit':
            return live.level(t, es.profile)
        bids, fids = self.sources_for(index)
        levels = ([(self.battery_level(b, t), self._weight_battery(b)) for b in bids]
                  + [(self.fuel_level(f, t), self._weight_fuel(f)) for f in fids])
        if not levels:
            return NAN
        if es.aggregate == 'min':
            return min(lv for lv, _ in levels)
        total = sum(w for _, w in levels)
        return sum(lv * w for lv, w in levels) / total if total > 0 else NAN

    def _weight_battery(self, bid: tuple) -> float:
        b = self.batteries[bid].cfg
        return b.weight if b.weight is not None else b.capacity_ah

    def _weight_fuel(self, fid: tuple) -> float:
        f = self.fuel[fid].cfg
        return f.weight if f.weight is not None else f.maximum_fuel

    def system_info_fields(self, index: tuple) -> dict:
        es: EnergySystem = self.systems[index].cfg
        return {'index': es.index, 'energy_systems': es.flags, 'energy_system_name': es.name}

    def system_status_fields(self, index: tuple, t: float) -> dict:
        es: EnergySystem = self.systems[index].cfg
        level = self.system_level(index, t)
        flags = es.status_flags
        if es.auto_severity:
            flags &= ~SEVERITY_MASK
            if not (flags & _INACTIVE) and not math.isnan(level):
                if level <= es.thresholds.emergency:
                    flags |= _EMERGENCY
                elif level <= es.thresholds.critical:
                    flags |= _CRITICAL
                elif level <= es.thresholds.low:
                    flags |= _LOW
        remaining_time = NAN
        if es.endurance_s is not None and not math.isnan(level):
            remaining_time = round(es.endurance_s * level / 100.0, 1)
        return {
            'index':              es.index,
            'remaining_capacity': round(level, 2) if es.report_percent and not math.isnan(level) else NAN,
            'remaining_time':     remaining_time,
            'status_flags':       flags,
        }

    # ------------------------------------------------------------------
    # Legacy BATTERY_STATUS (for legacy_battery_status: on | until_disabled)
    # ------------------------------------------------------------------
    def legacy_battery_status_fields(self, bid: tuple, t: float) -> dict:
        b: Battery = self.batteries[bid].cfg
        level = self.battery_level(bid, t)
        cell_mv = int(1000 * (3.5 + 0.7 * level / 100.0))
        cells = [cell_mv] * min(b.cells, 10) + [UINT16_MAX] * (10 - min(b.cells, 10))
        ext = [cell_mv if 10 + i < b.cells else 0 for i in range(4)]
        if level < 10:
            charge_state = 5      # MAV_BATTERY_CHARGE_STATE_EMERGENCY
        elif level < 20:
            charge_state = 4      # CRITICAL
        elif level < 30:
            charge_state = 3      # LOW
        else:
            charge_state = 1      # OK
        return {
            'id':                b.id,
            'battery_function':  0,
            'type':              1,
            'temperature':       2800,
            'voltages':          cells,
            'current_battery':   int(b.current_a * 100),
            'current_consumed':  int(b.capacity_ah * 1000 * (1 - level / 100.0)),
            'energy_consumed':   -1,
            'battery_remaining': int(round(level)),
            'time_remaining':    0,
            'charge_state':      charge_state,
            'voltages_ext':      ext,
            'mode':              0,
            'fault_bitmask':     0,
        }


def _pct_u16(level: float) -> int:
    """Percent -> uint16 with multiplier 2E-3 (0-100 % -> 0-50000)."""
    if math.isnan(level):
        return UINT16_MAX
    return max(0, min(50000, int(round(level / 0.002))))


def _fmt_changes(changes: dict) -> str:
    parts = []
    for k, v in changes.items():
        if isinstance(v, Profile):
            v = f"{v.start:g}%->{v.end:g}% over {v.period_s:g}s"
        elif k.endswith('flags'):
            v = f"0x{v:x}"
        parts.append(f"{k}={v}")
    return ', '.join(parts)
