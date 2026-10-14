"""
protocol.py - Vehicle side of the Energy Systems Protocol connection flow.

Transport independent: the responder thread passes in decoded commands, and the
main loop asks which messages are due to be sent. All state is guarded by a lock
because those two run on different threads.

Behaviour (per the devguide connection flow, overridable by the scenario's
`behaviour:` block):

- MAV_CMD_REQUEST_MESSAGE for an _INFO (or status) message: ACK ACCEPTED, then emit
  one message per instance (param2 = 0 means all instances; any other value is that
  instance id). ACK UNSUPPORTED if the scenario doesn't support that message.
- MAV_CMD_SET_MESSAGE_INTERVAL: start (interval > 0, or 0 = default rate) or stop
  (interval = -1) the stream for all instances of that message.
- Streams listed in behaviour.stream_by_default run from start-up (rejoin case).
- Fallback: if an _INFO message was requested but the matching status stream is not
  requested within behaviour.fallback_timeout (because mavsdk_server swallowed the
  SET_MESSAGE_INTERVAL, for example), the stream starts at the documented rate.

Components: requests are answered by the component they are addressed to. The scenario
`behaviour:` applies to the flow component (the autopilot, FLOW_COMPONENT). Other components
(e.g. a smart battery) act as independent devices: they stream their own status messages from
start-up, answer _INFO requests for their own instances, and ignore everything else. A request
to a component the scenario doesn't have gets no reply at all (as on a real vehicle).
"""

from __future__ import annotations

import threading
import time

from scenario import DEFAULT_STREAM_INTERVAL_S, DOC_REQUEST_INTERVAL_S, FLOW_COMPONENT, Scenario

MAV_CMD_SET_MESSAGE_INTERVAL = 511
MAV_CMD_REQUEST_MESSAGE      = 512

MAV_RESULT_ACCEPTED    = 0
MAV_RESULT_DENIED      = 2
MAV_RESULT_UNSUPPORTED = 3
MAV_RESULT_FAILED      = 4

MSG_BATTERY_STATUS       = 147
MSG_BATTERY_CELL_STATUS  = 364
MSG_ENERGY_SYSTEM_INFO   = 365
MSG_ENERGY_SYSTEM_STATUS = 366
MSG_FUEL_INFORMATION_V2  = 367
MSG_FUEL_STATUS_V2       = 368
MSG_BATTERY_STATUS_V2    = 369
MSG_BATTERY_INFO         = 372

MSG_NAMES = {
    MSG_BATTERY_STATUS:       'BATTERY_STATUS',
    MSG_BATTERY_CELL_STATUS:  'BATTERY_CELL_STATUS',
    MSG_ENERGY_SYSTEM_INFO:   'ENERGY_SYSTEM_INFO',
    MSG_ENERGY_SYSTEM_STATUS: 'ENERGY_SYSTEM_STATUS',
    MSG_FUEL_INFORMATION_V2:  'FUEL_INFORMATION_V2',
    MSG_FUEL_STATUS_V2:       'FUEL_STATUS_V2',
    MSG_BATTERY_STATUS_V2:    'BATTERY_STATUS_V2',
    MSG_BATTERY_INFO:         'BATTERY_INFO',
}
ENERGY_MESSAGES = frozenset(MSG_NAMES)

RESULT_NAMES = {MAV_RESULT_ACCEPTED: 'ACCEPTED', MAV_RESULT_DENIED: 'DENIED',
                MAV_RESULT_UNSUPPORTED: 'UNSUPPORTED', MAV_RESULT_FAILED: 'FAILED'}

# _INFO message -> status streams the GCS is expected to request after it
_INFO_TO_STATUS = {
    MSG_ENERGY_SYSTEM_INFO:  (MSG_ENERGY_SYSTEM_STATUS,),
    MSG_BATTERY_INFO:        (MSG_BATTERY_STATUS_V2, MSG_BATTERY_CELL_STATUS),
    MSG_FUEL_INFORMATION_V2: (MSG_FUEL_STATUS_V2,),
}


def msg_name(msg_id: int) -> str:
    return MSG_NAMES.get(msg_id, f'msg#{msg_id}')


def _log(text: str) -> None:
    print(f"  {text}", flush=True)


class Protocol:
    def __init__(self, scenario: Scenario, clock=time.monotonic):
        self.scenario = scenario
        self.behaviour = scenario.behaviour
        self._clock = clock
        self._lock = threading.Lock()
        self._streams: dict[tuple[int, int], list[float]] = {}         # (msg_id, component) -> [interval_s, next_due]
        self._oneshots: list[tuple[int, int, int | None]] = []          # (msg_id, component, instance or None=all)
        self._info_requested_at: dict[tuple[int, int], float] = {}      # (status msg_id, component) -> time
        self._interval_seen: set[tuple[int, int]] = set()
        now = clock()
        for msg_id, interval in self.behaviour.stream_by_default.items():
            self._streams[(msg_id, FLOW_COMPONENT)] = [interval, now]
        if self.behaviour.legacy_battery_status in ('on', 'until_disabled'):
            self._streams.setdefault((MSG_BATTERY_STATUS, FLOW_COMPONENT),
                                     [DEFAULT_STREAM_INTERVAL_S[MSG_BATTERY_STATUS], now])
        # Other components (e.g. smart batteries) stream their status by themselves
        for component in self.components() - {FLOW_COMPONENT}:
            if not scenario.components[component].stream_status:
                continue
            for msg_id in (MSG_ENERGY_SYSTEM_STATUS, MSG_BATTERY_STATUS_V2, MSG_BATTERY_CELL_STATUS, MSG_FUEL_STATUS_V2):
                if self.instances(msg_id, component):
                    self._streams[(msg_id, component)] = [DEFAULT_STREAM_INTERVAL_S[msg_id], now]

    # ------------------------------------------------------------------
    # Instances / support
    # ------------------------------------------------------------------
    def components(self) -> set[int]:
        s = self.scenario
        return {FLOW_COMPONENT} | {item.component for item in s.energy_systems + s.batteries + s.fuel}

    def instances(self, msg_id: int, component: int | None = None) -> list[tuple[int, int]]:
        """Keys (component, id) of the instances of msg_id, optionally only those of one component."""
        s = self.scenario
        if msg_id in (MSG_ENERGY_SYSTEM_INFO, MSG_ENERGY_SYSTEM_STATUS):
            items = s.energy_systems
        elif msg_id in (MSG_BATTERY_INFO, MSG_BATTERY_STATUS_V2):
            items = s.batteries
        elif msg_id == MSG_BATTERY_STATUS:
            items = [b for b in s.batteries if b.component == FLOW_COMPONENT]   # legacy: autopilot only
        elif msg_id == MSG_BATTERY_CELL_STATUS:
            items = [b for b in s.batteries if b.cell_status]
        elif msg_id in (MSG_FUEL_INFORMATION_V2, MSG_FUEL_STATUS_V2):
            items = s.fuel
        else:
            items = []
        return [it.key for it in items if component is None or it.component == component]

    def support_result(self, msg_id: int, disable: bool = False, component: int = FLOW_COMPONENT) -> int | None:
        """MAV_RESULT the component gives for a request/stream of msg_id, or None for no reply."""
        if component != FLOW_COMPONENT:
            if component not in self.components():
                return None
            return MAV_RESULT_ACCEPTED if self.instances(msg_id, component) else MAV_RESULT_UNSUPPORTED
        b = self.behaviour
        if msg_id in (MSG_ENERGY_SYSTEM_INFO, MSG_ENERGY_SYSTEM_STATUS):
            supported = b.energy_systems == 'supported'
        elif msg_id in (MSG_BATTERY_INFO, MSG_BATTERY_STATUS_V2, MSG_BATTERY_CELL_STATUS):
            supported = (b.batteries == 'supported'
                         or (b.batteries == 'info_only' and msg_id == MSG_BATTERY_INFO))
        elif msg_id in (MSG_FUEL_INFORMATION_V2, MSG_FUEL_STATUS_V2):
            supported = (b.fuel == 'supported'
                         or (b.fuel == 'info_only' and msg_id == MSG_FUEL_INFORMATION_V2))
        elif msg_id == MSG_BATTERY_STATUS:
            if b.legacy_battery_status == 'off':
                return MAV_RESULT_UNSUPPORTED
            if b.legacy_battery_status == 'on' and disable:
                return MAV_RESULT_DENIED      # firmware that always streams BATTERY_STATUS
            return MAV_RESULT_ACCEPTED
        else:
            return MAV_RESULT_UNSUPPORTED
        if not supported or not self.instances(msg_id, component):
            return MAV_RESULT_UNSUPPORTED
        return MAV_RESULT_ACCEPTED

    # ------------------------------------------------------------------
    # Commands (called from the responder thread)
    # ------------------------------------------------------------------
    def handles(self, msg_id: int) -> bool:
        return msg_id in ENERGY_MESSAGES

    @staticmethod
    def _who(component: int) -> str:
        return '' if component == FLOW_COMPONENT else f" [component {component}]"

    def request_message(self, msg_id: int, param2: float, component: int = FLOW_COMPONENT) -> int | None:
        result = self.support_result(msg_id, component=component)
        instance = None if int(param2) == 0 else int(param2)
        if (result == MAV_RESULT_ACCEPTED and instance is not None
                and (component, instance) not in self.instances(msg_id, component)):
            result = MAV_RESULT_FAILED
        inst_str = 'all' if instance is None else str(instance)
        _log(f"[GCS-> REQUEST_MESSAGE]{self._who(component)} {msg_name(msg_id)} (id={msg_id}) instance={inst_str}"
             f"  -> {'no reply' if result is None else RESULT_NAMES.get(result, result)}")
        if result != MAV_RESULT_ACCEPTED:
            return result
        with self._lock:
            self._oneshots.append((msg_id, component, instance))
            now = self._clock()
            if component == FLOW_COMPONENT:
                for status_id in _INFO_TO_STATUS.get(msg_id, ()):
                    self._info_requested_at.setdefault((status_id, component), now)
        return result

    def set_message_interval(self, msg_id: int, interval_us: float, component: int = FLOW_COMPONENT) -> int | None:
        disable = interval_us < 0
        result = self.support_result(msg_id, disable=disable, component=component)
        if interval_us > 0:
            rate = f"{interval_us / 1e6:g} s ({1e6 / interval_us:.3g} Hz)"
        elif interval_us == 0:
            rate = 'default rate'
        else:
            rate = 'disable'
        _log(f"[GCS-> SET_MESSAGE_INTERVAL]{self._who(component)} {msg_name(msg_id)} (id={msg_id}) {rate}"
             f"  -> {'no reply' if result is None else RESULT_NAMES.get(result, result)}")
        with self._lock:
            self._interval_seen.add((msg_id, component))
            if result != MAV_RESULT_ACCEPTED:
                return result
            if disable:
                self._streams.pop((msg_id, component), None)
                if msg_id == MSG_BATTERY_STATUS:
                    _log("[LEGACY] BATTERY_STATUS stream OFF (disabled by GCS)")
            else:
                interval_s = interval_us / 1e6 if interval_us > 0 else DEFAULT_STREAM_INTERVAL_S[msg_id]
                self._streams[(msg_id, component)] = [interval_s, self._clock()]
        return result

    # ------------------------------------------------------------------
    # Main loop queries
    # ------------------------------------------------------------------
    def due(self) -> list[tuple[int, int, int | None, str]]:
        """Messages to send now: (msg_id, component, instance or None=all, reason)."""
        out = []
        with self._lock:
            now = self._clock()
            for msg_id, component, instance in self._oneshots:
                out.append((msg_id, component, instance, 'requested'))
            self._oneshots.clear()
            self._check_fallback(now)
            for (msg_id, component), stream in self._streams.items():
                interval, next_due = stream
                if now >= next_due:
                    out.append((msg_id, component, None, 'stream'))
                    # Don't burst to catch up after a stall
                    stream[1] = max(next_due + interval, now + interval * 0.5)
        return out

    def _check_fallback(self, now: float) -> None:
        timeout = self.behaviour.fallback_timeout_s
        if timeout is None:
            return
        for key, requested_at in list(self._info_requested_at.items()):
            status_id, component = key
            if key in self._interval_seen or key in self._streams:
                del self._info_requested_at[key]
                continue
            if now - requested_at < timeout:
                continue
            del self._info_requested_at[key]
            if self.support_result(status_id, component=component) != MAV_RESULT_ACCEPTED:
                continue
            interval = DOC_REQUEST_INTERVAL_S[status_id]
            self._streams[key] = [interval, now]
            _log(f"[FALLBACK] No SET_MESSAGE_INTERVAL for {msg_name(status_id)} within {timeout:g} s "
                 f"of its _INFO request -> streaming at {interval:g} s. (Either the GCS chose not to "
                 f"request it, or mavsdk_server did not forward the command.)")

    def streams(self) -> dict[tuple[int, int], float]:
        with self._lock:
            return {k: v[0] for k, v in self._streams.items()}
