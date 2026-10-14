# mavsdk_qgc_server_tests

Repo for MAVSDK tests thatwork with QGC

This is a repo of MAVSDK server plugins to test various MAVLink features and their interaction with QGC.
This is not quite the same as an ordinary server plugin, because:

- It also contains the logic to work with QGC, which requires some parameters be sent etc in the initialisation
- We're testing specific QGC functionality. This is less general that tests for a whole protocol.

## Tests

- [battery_tests](battery_tests/): BATTERY_STATUS / BATTERY_STATUS_V2 / BATTERY_INFO display and V1→V2 negotiation.
- [condition_gate_tests](condition_gate_tests/): upload a mission containing MAV_CMD_CONDITION_GATE.
- [energy_systems_tests](energy_systems_tests/): scenario-driven vehicle simulator for the Energy Systems Protocol (ENERGY_SYSTEM_*, BATTERY_*, FUEL_*_V2).
