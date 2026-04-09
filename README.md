# mavsdk_qgc_server_tests

Repo for MAVSDK tests thatwork with QGC

This is a repo of MAVSDK server plugins to test various MAVLink features and their interaction with QGC.
This is not quite the same as an ordinary server plugin, because:

- It also contains the logic to work with QGC, which requires some parameters be sent etc in the initialisation
- We're testing specific QGC functionality. This is less general that tests for a whole protocol.
