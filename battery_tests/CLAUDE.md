# battery_test_mavsdk.py — Claude context

## Purpose

Simulates a MAVLink vehicle that streams battery telemetry to QGroundControl (QGC).
Used to test QGC's battery display (V1 legacy, V2 development dialect, and the automatic V1→V2 upgrade negotiation that QGC initiates after `initialConnectComplete`).

## Architecture overview

```
QGC (port 14550)
    ↕  UDP
mavsdk_server  ←——— spawned by MAVSDK-Python System.connect()
    ↕  gRPC
Python script
    |
    ├── asyncio main loop      — sends battery MAVLink frames via MavlinkDirect
    └── _responder_loop thread — keep-alive socket (ka)
            • sends HEARTBEAT to QGC (sysid=1, compid=1, type=QUADROTOR) every 1 s
            • receives forwarded commands from QGC (via mavsdk_server routing)
            • handles: COMMAND_LONG, PARAM_REQUEST_READ/LIST, MISSION_REQUEST_LIST
            • replies: COMMAND_ACK, AUTOPILOT_VERSION, PARAM_VALUE, MISSION_COUNT
```

## Why two sockets / two paths

mavsdk_server is a routing proxy. It listens on port 14550 and forwards packets from QGC addressed to sysid=1 to the most recent source of sysid=1 heartbeats.
After bootstrap, our `ka` socket is that source.

QGC's `requestMessage()` infrastructure correlates responses to the link/port that the request was sent on.
Replying via MAVSDK's gRPC-connected socket (a different UDP port) causes QGC to silently ignore the response. All handshake frames (COMMAND_ACK, AUTOPILOT_VERSION, PARAM_VALUE, MISSION_COUNT) MUST be sent from the same `ka` socket.

Battery telemetry (BATTERY_STATUS, BATTERY_STATUS_V2, BATTERY_INFO) is sent via MAVSDK's `MavlinkDirect` API, which goes through the gRPC→mavsdk_server path.
These messages do not need the correlate-to-link behaviour.

## Bootstrap

mavsdk_server starts listening on an ephemeral port when it receives its first MAVLink frame. 
`_bootstrap_grpc()` sniffs that port by binding a SO_REUSEPORT socket alongside mavsdk_server on port 14550, then sends 10 heartbeat frames to wake up gRPC.
The spy socket is closed afterwards; the `ka` socket takes over heartbeat duty.

## QGC InitialConnectStateMachine

After QGC sees sysid=1 heartbeats it starts an 8-state machine:

```
AutopilotVersion → StandardModes → CompInfo → Parameters →
Mission → GeoFence → RallyPoints → Complete
```

Our responder handles each request:

| QGC request                                | Our response                                   |
| ------------------------------------------ | ---------------------------------------------- |
| MAV_CMD_REQUEST_MESSAGE(AUTOPILOT_VERSION) | ACK ACCEPTED + AUTOPILOT_VERSION frame         |
| MAV_CMD_REQUEST_MESSAGE(any other)         | ACK UNSUPPORTED (lets QGC advance immediately) |
| PARAM_REQUEST_LIST / PARAM_REQUEST_READ    | PARAM_VALUE (dummy, 1 parameter)               |
| MISSION_REQUEST_LIST                       | MISSION_COUNT (count=0)                        |
| Other COMMAND_LONG                         | ACK ACCEPTED                                   |

UNSUPPORTED is preferred over ACCEPTED + no response for StandardModes, ComponentMetadata, etc. because it lets QGC skip the 5–30 s wait timeout.

## V1→V2 negotiation (--auto mode)

After `initialConnectComplete` fires, QGC calls `BatteryFactGroupListModel::startV2Negotiation()`:

1. Sends `SET_MESSAGE_INTERVAL(369, 2000000 µs)` (BATTERY_STATUS_V2 at 0.5 Hz)
2. Waits for first BATTERY_STATUS_V2 frame → calls `_activateV2()`
3. `_activateV2` sends `SET_MESSAGE_INTERVAL(147, -1)` (disable BATTERY_STATUS)
4. QGC independently sends `SET_MESSAGE_INTERVAL(372, 10000000 µs)` (BATTERY_INFO at 0.1 Hz)

In `--auto` mode the script:

- Starts streaming V1 only
- On `SET_MESSAGE_INTERVAL(369, >0)` → sets `state.v2 = True`, sends BATTERY_INFO immediately
- On `SET_MESSAGE_INTERVAL(147, <0)` → sets `state.v1 = False`

**15-second fallback**: mavsdk_server may intercept `SET_MESSAGE_INTERVAL` commands and respond ACCEPTED internally without forwarding to the `ka` socket.
In that case `state.v2` never becomes True via the normal path. As a fallback, `state.v2` is forced True 15 seconds after `state.handshake_complete_at` is set (when AUTOPILOT_VERSION was sent).
QGC is in `V2NegotiationRequesting` state at this
point, so the first BATTERY_STATUS_V2 frame triggers `_activateV2`.

## Development dialect messages

BATTERY_STATUS_V2 (id=369) and BATTERY_INFO (id=372) are not in MAVLink's `common.xml`; they are in `development.xml`. mavsdk_server ships with only `common.xml`, so these message IDs are unknown to it. The script works around this by calling `MavlinkDirect.load_custom_xml()` with inline XML definitions that give mavsdk_server the correct field layout and CRC_EXTRA.

The handshake frames (COMMAND_ACK etc.) are sent raw via the `ka` socket, bypassing mavsdk_server entirely, so they also handle development dialect messages without any custom XML.

## Key classes / functions

| Symbol                           | Purpose                                                                                                    |
| -------------------------------- | ---------------------------------------------------------------------------------------------------------- |
| `StreamState`                    | Per-battery mutable flags for `--auto` mode (`v1`, `v2`, `info`, `send_info_now`, `handshake_complete_at`) |
| `send_v1()`                      | Sends BATTERY_STATUS via MavlinkDirect                                                                     |
| `send_v2()`                      | Sends BATTERY_STATUS_V2 via MavlinkDirect; alternates `capacity_consumed`/`capacity_remaining` NaN         |
| `send_battery_info()`            | Sends BATTERY_INFO via MavlinkDirect                                                                       |
| `_bootstrap_grpc()`              | Sniffs mavsdk_server's ephemeral port; sends bootstrap heartbeats                                          |
| `_responder_loop()`              | Background thread: heartbeats + handshake responses via `ka` socket                                        |
| `_handle_set_message_interval()` | Parses SET_MESSAGE_INTERVAL, updates `StreamState`                                                         |
| `_mavlink2_frame()`              | Builds a raw MAVLink2 frame (no signing)                                                                   |
| `_raw_autopilot_version()`       | Wire payload for AUTOPILOT_VERSION (60 bytes, CRC_EXTRA=178)                                               |
| `_raw_command_ack()`             | Wire payload for COMMAND_ACK (3 bytes base fields only)                                                    |

## Wire format notes

- MAVLink2 frame: `0xFD | payload_len | incompat | compat | seq | sysid | compid | msg_id[3] | payload | CRC16`
- COMMAND_ACK base fields: `command(u16), result(u8)` = 3 bytes. Extension fields (`progress`, `result_param2`, `target_system`, `target_component`) are absent.
- AUTOPILOT_VERSION: 60-byte payload, fields in descending-size wire order.
- CRC_EXTRA values used: HEARTBEAT=50, COMMAND_ACK=143, AUTOPILOT_VERSION=178, PARAM_VALUE=220, MISSION_COUNT=221.

## Connection topology

```
QGC listens on udp://0.0.0.0:14550  (default)
    ↕ UDP 14550
mavsdk_server (spawned by MAVSDK-Python, binds 0.0.0.0:14550 via SO_REUSEPORT)
    ↕ gRPC localhost:50051 (ephemeral)
System(sysid=1, compid=191) in battery_test_mavsdk.py

ka socket (unbound, ephemeral port)
    → sends HEARTBEAT to 127.0.0.1:14550 (QGC + mavsdk_server)
    ← receives forwarded COMMAND_LONG etc. from mavsdk_server
    → sends COMMAND_ACK, AUTOPILOT_VERSION, PARAM_VALUE, MISSION_COUNT
```

## Known limitations

- `--auto` mode: V1 continues to stream alongside V2 in routing-proxy mode (Path 2) because `SET_MESSAGE_INTERVAL(147, -1)` is intercepted by mavsdk_server and never reaches the responder.\
  V1 keeps flowing but QGC's V2 display still activates.
- SO_REUSEPORT is required (Linux ≥ 3.9). Does not work on native Windows; run under WSL2.
- Only tested with QGC and `mavsdk` Python package ≥ 3.15.3.
