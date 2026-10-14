# energy_systems_tests — Claude context

Simulated vehicle (energy_sim.py) for testing QGC's Energy Systems Protocol implementation.
User-facing docs: README.md. Transport architecture is inherited from `../battery_tests`
(see its CLAUDE.md: mavsdk_server bootstrap, `ka` responder socket, why replies must come from `ka`).

## Files

| File | Role |
| --- | --- |
| `energy_sim.py` | CLI, transport (bootstrap, responder thread, raw pymavlink emitter on the `ka` socket), main loop |
| `scenario.py` | YAML schema → dataclasses, enum/flag tables, validation (`ScenarioError`) |
| `model.py` | Field values per message at sim time `t`; aggregation (min/weighted/explicit); timeline |
| `protocol.py` | REQUEST_MESSAGE / SET_MESSAGE_INTERVAL handling, stream registry, fallback. Thread-safe, transport-free |
| `dialect.py` | Generates/imports a pymavlink dialect from `xml/energy_messages.xml` into `.generated/` |
| `xml/energy_messages.xml` | Verbatim enums+messages from mavlink#2526 head `2d4dd92` |
| `tools/fake_gcs.py` | QGC stand-in: runs connection flow, decodes independently, checks vs scenario (incl. sender compid 1) |
| `tools/update_xml.py` | Refresh `xml/energy_messages.xml` verbatim from a mavlink git ref |
| `tools/check_scenarios.py` | Runs sim + fake GCS for each scenario (own process group per sim) |
| `tests/test_simulator.py` | Offline unittest for scenario/model/protocol |

## Verified facts (don't re-derive)

- `pip install mavsdk` now gives MAVSDK 4.x (native ctypes binding, no gRPC/mavsdk_server, different API).
  Use `mavsdk-grpc` (import `mavsdk_grpc`) or pin `mavsdk<4`. energy_sim.py tries both.
- mavsdk_server sends MavlinkDirect messages from ITS OWN compid (191), ignoring MavlinkMessage.component_id.
  So all telemetry is sent raw (pymavlink `Emitter._send_raw`) from the `ka` socket with the item's
  `component` (default 1), sharing `_SEQ` with heartbeats/ACKs. `--mavlink-direct` keeps the old path for
  comparison only.
- Design (agreed with user): energy systems and sources may come from ANY component (e.g. smart battery on 180
  sends only BATTERY_INFO/STATUS_V2; the autopilot aggregates). Instances keyed (component, id) everywhere:
  model dict keys, Protocol streams `(msg_id, component)`, fake_gcs `by_instance[(comp, id)]`. Component 1 =
  flow component (behaviour applies); others: stream status unprompted, answer own _INFO requests (ACK from
  that compid), optional MAV_TYPE_BATTERY heartbeat (`components:` block). GCS discovers them on first sight
  or from the heartbeat (QGC and fake_gcs both do this).
- mavsdk_server's built-in BATTERY_INFO (common.xml) wins over `load_custom_xml`; libmav can't set the
  new `energy_systems` extension → BATTERY_INFO is always raw (`RAW_MESSAGES`).
- Verified with real QGC (branch energy-systems): its REQUEST_MESSAGE and SET_MESSAGE_INTERVAL reach the
  `ka` responder; the [FALLBACK] path did not fire.
- With a GCS that sends to every address it has heard sysid 1 from (QGC UDPLink behaviour), both
  REQUEST_MESSAGE and SET_MESSAGE_INTERVAL reach the `ka` responder. Frames can arrive twice (direct and
  via mavsdk_server) → responder dedupes identical frames for 2 s.
- mavsdk_server (sysid 1, compid 191) sends its own commands and echoes our telemetry to `ka`;
  the responder ignores all sysid 1 frames.
- A GCS-side socket must use SO_REUSEADDR only (like Qt). With SO_REUSEPORT the kernel hashes
  mavsdk_server's packets away from the bootstrap spy and System.connect() never returns.
- pymavlink: `msg.pack(mav)` does not advance `mav.seq`; char[] fields are decoded as ASCII
  (use `m._<field>_raw` for UTF-8).
- Killing energy_sim.py's Python process alone leaves mavsdk_server holding gRPC :50051;
  the next run then silently talks to the stale server. Kill the process group.
- YAML 1.1: bare `on`/`off` are booleans — scenario.py normalises `legacy_battery_status`.

## Conventions

- New behaviour knobs go in `scenario.Behaviour` + `_behaviour()` + `Protocol.support_result()`, and get
  a check in `fake_gcs.check()`.
- New scenario: add YAML with a `description` that states the expected QGC display; run
  `tools/check_scenarios.py <name>` and the unit tests.
- When mavlink#2526 changes: refresh `xml/energy_messages.xml` from the PR head (keep verbatim), update
  enum tables in scenario.py and field dicts in model.py; the fake GCS will flag any mismatch.
