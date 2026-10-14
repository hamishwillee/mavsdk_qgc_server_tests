"""Offline unit tests for the scenario loader, energy model and protocol logic.

Run from energy_systems_tests/:  python3 -m unittest discover -s tests
"""

import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from model import EnergyModel  # noqa: E402
from protocol import (MAV_RESULT_ACCEPTED, MAV_RESULT_DENIED, MAV_RESULT_FAILED,  # noqa: E402
                      MAV_RESULT_UNSUPPORTED, MSG_BATTERY_CELL_STATUS, MSG_BATTERY_INFO,
                      MSG_BATTERY_STATUS, MSG_BATTERY_STATUS_V2, MSG_ENERGY_SYSTEM_INFO,
                      MSG_ENERGY_SYSTEM_STATUS, MSG_FUEL_INFORMATION_V2, MSG_FUEL_STATUS_V2, Protocol)
from scenario import (MAV_ENERGY_STATUS_FLAGS as ESF, ScenarioError, load_scenario,  # noqa: E402
                      parse_duration, parse_scenario)


K0, K1 = (1, 0), (1, 1)   # (component, index/id) keys on the flow component


def scen(**kw):
    base = {'energy_systems': [{'index': 0, 'flags': ['PROPULSION'], 'endurance': '10m'}],
            'sources': {'batteries': [{'id': 0, 'energy_systems': ['PROPULSION'],
                                       'profile': {'start': 100, 'end': 0, 'period': 100, 'loop': False}}]}}
    base.update(kw)
    return parse_scenario(base)


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


class ScenarioTests(unittest.TestCase):
    def test_all_shipped_scenarios_load(self):
        paths = sorted((ROOT / 'scenarios').glob('*.yaml'))
        self.assertGreaterEqual(len(paths), 20)
        for p in paths:
            with self.subTest(p.name):
                load_scenario(p)

    def test_durations(self):
        self.assertEqual(parse_duration('1m30s', 'x'), 90)
        self.assertEqual(parse_duration(5, 'x'), 5)
        self.assertEqual(parse_duration('500ms', 'x'), 0.5)
        with self.assertRaises(ScenarioError):
            parse_duration('5 minutes', 'x')

    def test_flag_names_and_prefixes(self):
        s = scen(energy_systems=[{'index': 0, 'flags': ['MAV_ENERGY_SYSTEM_FLAGS_PROPULSION', 'avionics'],
                                  'status_flags': ['FLIGHT_CRITICAL']}])
        self.assertEqual(s.energy_systems[0].flags, 3)
        self.assertEqual(s.energy_systems[0].status_flags, ESF['FLIGHT_CRITICAL'])

    def test_errors(self):
        for bad in (
            {'energy_systems': [{'index': 0, 'flags': 0, 'profile': 50}]},                     # zero mask
            {'energy_systems': [{'index': 0, 'flags': ['NOPE'], 'profile': 50}]},              # bad flag
            {'energy_systems': [{'index': 0, 'flags': ['PAYLOAD']}]},                          # no source, not explicit
            {'energy_systems': [{'index': 0, 'flags': ['PAYLOAD'], 'name': 'x' * 26, 'profile': 1}]},  # name too long
            {'energy_systems': [{'index': 0, 'flags': ['PAYLOAD'], 'profile': 1},
                                {'index': 0, 'flags': ['AVIONICS'], 'profile': 1}]},          # duplicate index
            {'energy_systems': [{'index': 0, 'flags': ['PAYLOAD'], 'profile': 1}],
             'timeline': [{'at': 1, 'set': {'es': 3, 'add_flags': ['LOW']}}]},                 # unknown target
        ):
            with self.subTest(bad), self.assertRaises(ScenarioError):
                parse_scenario(bad)

    def test_yaml_on_is_accepted(self):
        s = scen(behaviour={'legacy_battery_status': True})
        self.assertEqual(s.behaviour.legacy_battery_status, 'on')


class ModelTests(unittest.TestCase):
    def test_severity_thresholds(self):
        m = EnergyModel(scen())
        flags = lambda t: m.system_status_fields(K0, t)['status_flags']   # noqa: E731
        self.assertEqual(flags(50), 0)                      # 50 %
        self.assertEqual(flags(80), ESF['LOW'])             # 20 %
        self.assertEqual(flags(90), ESF['CRITICAL'])        # 10 %
        self.assertEqual(flags(97), ESF['EMERGENCY'])       # 3 %

    def test_remaining_time_from_endurance(self):
        m = EnergyModel(scen())
        self.assertAlmostEqual(m.system_status_fields(K0, 50)['remaining_time'], 300.0)

    def test_inactive_has_no_severity(self):
        s = scen(energy_systems=[{'index': 0, 'flags': ['PROPULSION'], 'status_flags': ['INACTIVE']}])
        f = EnergyModel(s).system_status_fields(K0, 97)
        self.assertEqual(f['status_flags'], ESF['INACTIVE'])

    def test_min_and_weighted(self):
        s = parse_scenario({
            'energy_systems': [{'index': 0, 'flags': ['PROPULSION'], 'aggregate': 'min'},
                               {'index': 1, 'flags': ['PROPULSION'], 'aggregate': 'weighted'}],
            'sources': {'batteries': [{'id': 0, 'energy_systems': ['PROPULSION'], 'capacity_ah': 30, 'profile': 80},
                                      {'id': 1, 'energy_systems': ['PROPULSION'], 'capacity_ah': 10, 'profile': 40}]}})
        m = EnergyModel(s)
        self.assertAlmostEqual(m.system_level(K0, 0), 40)
        self.assertAlmostEqual(m.system_level(K1, 0), 70)    # (80*30 + 40*10) / 40

    def test_matching_by_mask_and(self):
        s = parse_scenario({
            'energy_systems': [{'index': 0, 'flags': ['PROPULSION_VTOL_MC']},
                               {'index': 1, 'flags': ['PROPULSION_VTOL_FW']}],
            'sources': {'batteries': [{'id': 0, 'energy_systems': ['PROPULSION_VTOL_MC', 'PROPULSION_VTOL_FW']}],
                        'fuel': [{'id': 0, 'energy_systems': ['PROPULSION_VTOL_FW']}]}})
        m = EnergyModel(s)
        self.assertEqual(m.sources_for(K0), ([K0], []))
        self.assertEqual(m.sources_for(K1), ([K0], [K0]))

    def test_timeline_repeat(self):
        s = scen(timeline=[{'at': 10, 'repeat': 20, 'set': {'es': 0, 'add_flags': ['INACTIVE']}},
                           {'at': 20, 'repeat': 20, 'set': {'es': 0, 'clear_flags': ['INACTIVE']}}])
        m = EnergyModel(s)
        inactive = lambda: bool(m.systems[K0].cfg.status_flags & ESF['INACTIVE'])  # noqa: E731
        m.apply_timeline(9)
        self.assertFalse(inactive())
        m.apply_timeline(10)
        self.assertTrue(inactive())
        m.apply_timeline(25)
        self.assertFalse(inactive())
        m.apply_timeline(31)
        self.assertTrue(inactive())

    def test_encodings(self):
        s = parse_scenario({
            'energy_systems': [{'index': 0, 'flags': ['PROPULSION'], 'report_percent': False, 'profile': 50}],
            'sources': {'batteries': [{'id': 0, 'energy_systems': ['PROPULSION'], 'cells': 12, 'profile': 50},
                                      {'id': 1, 'energy_systems': ['PROPULSION'], 'report_level': False}]}})
        m = EnergyModel(s)
        self.assertTrue(math.isnan(m.system_status_fields(K0, 0)['remaining_capacity']))
        self.assertTrue(math.isnan(m.system_status_fields(K0, 0)['remaining_time']))
        self.assertEqual(m.battery_status_fields(K0, 0)['state_of_charge'], 25000)   # 50 % / 2E-3
        self.assertEqual(m.battery_status_fields(K1, 0)['state_of_charge'], 65535)
        cells = m.battery_cell_fields(K0, 0)
        self.assertEqual([c['cells_instance'] for c in cells], [0, 1])
        self.assertEqual(m.battery_info_fields(K0)['energy_systems'], 1)


class ProtocolTests(unittest.TestCase):
    def test_documented_flow(self):
        clock = FakeClock()
        p = Protocol(load_scenario(ROOT / 'scenarios' / 'vtol_mc_fw.yaml'), clock)
        self.assertEqual(p.due(), [])                                   # nothing streams by default
        self.assertEqual(p.request_message(MSG_ENERGY_SYSTEM_INFO, 0), MAV_RESULT_ACCEPTED)
        self.assertEqual(p.due(), [(MSG_ENERGY_SYSTEM_INFO, 1, None, 'requested')])
        self.assertEqual(p.set_message_interval(MSG_ENERGY_SYSTEM_STATUS, 2e6), MAV_RESULT_ACCEPTED)
        self.assertEqual(p.due(), [(MSG_ENERGY_SYSTEM_STATUS, 1, None, 'stream')])
        clock.t = 1.0
        self.assertEqual(p.due(), [])
        clock.t = 2.0
        self.assertEqual(p.due(), [(MSG_ENERGY_SYSTEM_STATUS, 1, None, 'stream')])
        self.assertEqual(p.set_message_interval(MSG_ENERGY_SYSTEM_STATUS, -1), MAV_RESULT_ACCEPTED)
        clock.t = 10.0
        self.assertEqual(p.due(), [])
        self.assertEqual(p.instances(MSG_FUEL_INFORMATION_V2), [K0])
        self.assertEqual(p.request_message(MSG_BATTERY_INFO, 5), MAV_RESULT_FAILED)   # no battery 5
        self.assertEqual(p.request_message(MSG_BATTERY_CELL_STATUS, 0), MAV_RESULT_UNSUPPORTED)  # none

    def test_unsupported_and_info_only(self):
        p = Protocol(load_scenario(ROOT / 'scenarios' / 'legacy_only.yaml'), FakeClock())
        self.assertEqual(p.request_message(MSG_ENERGY_SYSTEM_INFO, 0), MAV_RESULT_UNSUPPORTED)
        self.assertEqual(p.set_message_interval(MSG_BATTERY_STATUS, -1), MAV_RESULT_DENIED)  # legacy 'on'
        p = Protocol(load_scenario(ROOT / 'scenarios' / 'info_only.yaml'), FakeClock())
        self.assertEqual(p.request_message(MSG_BATTERY_INFO, 0), MAV_RESULT_ACCEPTED)
        self.assertEqual(p.set_message_interval(MSG_BATTERY_STATUS_V2, 8e6), MAV_RESULT_UNSUPPORTED)
        self.assertEqual(p.set_message_interval(MSG_FUEL_STATUS_V2, 8e6), MAV_RESULT_UNSUPPORTED)

    def test_fallback_and_legacy_disable(self):
        clock = FakeClock()
        p = Protocol(load_scenario(ROOT / 'scenarios' / 'legacy_until_disabled.yaml'), clock)
        self.assertEqual(p.due(), [(MSG_BATTERY_STATUS, 1, None, 'stream')])
        self.assertEqual(p.set_message_interval(MSG_BATTERY_STATUS, -1), MAV_RESULT_ACCEPTED)
        p.request_message(MSG_ENERGY_SYSTEM_INFO, 0)
        p.due()
        clock.t = 4.9
        self.assertEqual(p.due(), [])
        clock.t = 5.0
        self.assertEqual(p.due(), [(MSG_ENERGY_SYSTEM_STATUS, 1, None, 'stream')])   # fallback at 2 s
        self.assertNotIn((MSG_BATTERY_STATUS, 1), p.streams())

    def test_stream_by_default(self):
        p = Protocol(load_scenario(ROOT / 'scenarios' / 'rejoin.yaml'), FakeClock())
        self.assertEqual(set(p.streams()), {(MSG_ENERGY_SYSTEM_STATUS, 1), (MSG_BATTERY_STATUS_V2, 1), (MSG_FUEL_STATUS_V2, 1)})


class ComponentTests(unittest.TestCase):
    def test_same_id_on_two_components(self):
        s = load_scenario(ROOT / 'scenarios' / 'smart_battery.yaml')
        m = EnergyModel(s)
        self.assertEqual(sorted(m.batteries), [(1, 0), (180, 0)])
        self.assertEqual(sorted(m.sources_for(K0)[0]), [(1, 0), (180, 0)])
        with self.assertRaises(ScenarioError):   # duplicate on the same component is still an error
            scen(sources={'batteries': [{'id': 0, 'energy_systems': ['PROPULSION']},
                                        {'id': 0, 'energy_systems': ['PROPULSION']}]})

    def test_other_component_behaves_as_device(self):
        clock = FakeClock()
        p = Protocol(load_scenario(ROOT / 'scenarios' / 'smart_battery.yaml'), clock)
        # Smart battery streams its own status from start-up; the autopilot streams nothing yet
        self.assertEqual(set(p.streams()), {(MSG_BATTERY_STATUS_V2, 180), (MSG_BATTERY_CELL_STATUS, 180)})
        # Requests are answered per component, with that component's instances only
        self.assertEqual(p.request_message(MSG_BATTERY_INFO, 0, 180), MAV_RESULT_ACCEPTED)
        self.assertEqual(p.due()[0], (MSG_BATTERY_INFO, 180, None, 'requested'))
        self.assertEqual(p.instances(MSG_BATTERY_INFO, 180), [(180, 0)])
        self.assertEqual(p.request_message(MSG_ENERGY_SYSTEM_INFO, 0, 180), MAV_RESULT_UNSUPPORTED)
        self.assertIsNone(p.request_message(MSG_BATTERY_INFO, 0, 99))   # no such component: no reply
        # Legacy BATTERY_STATUS is only the autopilot's
        self.assertEqual(p.instances(MSG_BATTERY_STATUS), [(1, 0)])

    def test_component_config(self):
        s = load_scenario(ROOT / 'scenarios' / 'smart_battery_heartbeat.yaml')
        self.assertTrue(s.components[181].heartbeat)
        self.assertEqual(Protocol(s, FakeClock()).streams(), {})   # stream_status: false
        with self.assertRaises(ScenarioError):
            parse_scenario({'components': {50: {'heartbeat': True}},
                            'energy_systems': [{'index': 0, 'flags': ['PAYLOAD'], 'profile': 1}]})

    def test_timeline_component_resolution(self):
        base = {'energy_systems': [{'index': 0, 'flags': ['PROPULSION']}],
                'sources': {'batteries': [{'id': 0, 'energy_systems': ['PROPULSION']},
                                          {'id': 0, 'component': 180, 'energy_systems': ['PROPULSION']}]}}
        with self.assertRaises(ScenarioError):   # ambiguous
            parse_scenario(dict(base, timeline=[{'at': 1, 'set': {'battery': 0, 'add_flags': ['CHARGING']}}]))
        s = parse_scenario(dict(base, timeline=[{'at': 1, 'set': {'battery': 0, 'component': 180,
                                                                  'add_flags': ['CHARGING']}}]))
        m = EnergyModel(s)
        m.apply_timeline(1)
        self.assertEqual(m.batteries[(180, 0)].cfg.status_flags, 2)
        self.assertEqual(m.batteries[(1, 0)].cfg.status_flags, 0)


if __name__ == '__main__':
    unittest.main()
