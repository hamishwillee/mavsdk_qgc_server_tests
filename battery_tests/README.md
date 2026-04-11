# Battery Tests

Tests for QGC's battery display, covering BATTERY_STATUS (legacy V1), BATTERY_STATUS_V2, BATTERY_INFO, and the automatic V1→V2 negotiation that QGC performs when it connects to a vehicle.

## What is tested

| Mode                   | Messages sent                                       | What QGC should show                                                                                      |
| ---------------------- | --------------------------------------------------- | --------------------------------------------------------------------------------------------------------- |
| `--v1`                 | BATTERY_STATUS (id=147)                             | Legacy battery widget: %, voltage, current, mAh                                                           |
| `--v2`                 | BATTERY_STATUS_V2 (id=369) + BATTERY_INFO (id=372)  | V2 widget: %, voltage, current, capacity remaining/consumed, status flags, serial number, etc.            |
| `--v2 --fault`         | BATTERY_STATUS_V2 with fault flags set              | Battery widget shows FAILED charge state / fault indication                                               |
| `--v2 --no-info`       | BATTERY_STATUS_V2 only, no BATTERY_INFO             | V2 widget without static info (capacity remaining is inferred from percent, not capacity_remaining field) |
| `--auto`               | Starts V1; switches to V2+INFO when QGC requests it | Widget transitions from V1 → V2 after ~5–15 s                                                             |
| `--v1 --v2`            | Both simultaneously                                 | QGC prefers V2 once negotiation completes                                                                 |
| `--count 2`            | Two battery instances (id=0 and id=1)               | Two battery entries in the widget                                                                         |
| `--v2 --fast`          | BATTERY_STATUS_V2 cycling 95%→5% in ~30 s          | LOW voice alert at ~25% (~24 s in), CRITICAL at ~10% (~29 s in)                                          |

## Prerequisites

- Python 3.10+
- MAVSDK-Python (see `requirements.txt`)
- QGroundControl running and listening on UDP port 14550 (default)

Install dependencies:

```bash
pip install -r requirements.txt
```

`mavsdk` bundles `mavsdk_server` — no separate install is needed.

## How to run

```bash
# Auto-negotiate: start V1, switch to V2+INFO when QGC requests
python3 battery_test_mavsdk.py --auto

# V1 only (legacy battery)
python3 battery_test_mavsdk.py --v1

# V2 + BATTERY_INFO (development dialect)
python3 battery_test_mavsdk.py --v2

# V2 with fault flags set
python3 battery_test_mavsdk.py --v2 --fault

# V2 without BATTERY_INFO (capacity inferred from percent)
python3 battery_test_mavsdk.py --v2 --no-info

# Two batteries
python3 battery_test_mavsdk.py --v2 --count 2

# Fast cycle to test LOW/CRITICAL voice alerts (95%→5% in ~30 s)
python3 battery_test_mavsdk.py --v2 --fast

# Custom QGC port
python3 battery_test_mavsdk.py --v1 --port 14551
```

Press **Ctrl+C** to stop.

## Success criteria

**`--v1`**

- QGC displays a battery entry with %, voltage, current, and mAh consumed.
- Values cycle (95 % → ~5 %) over ~3 minutes then repeat.

**`--v2`**

- QGC displays the V2 battery widget with:
  - Voltage, current, % remaining
  - `capacityRemaining` (Ah) — alternates between provided and inferred every 4 ticks
  - Static info from BATTERY_INFO: serial number `SN-1000`, name `Acme_LiPo4S5000`, 4 cells, SOH 92 %, cycle count 47
  - `chargeState` = OK (no fault flags) while battery > 25 %
- BATTERY_STATUS (V1) is still shown if QGC hasn't disabled it yet (race condition on first connect).
- As battery decreases: voice alert "battery level low" when % drops to 25 %, and "battery level critical" when % drops to 10 %.
- Use `--fast` to cycle battery quickly (~30 s) and verify both alerts fire: `python3 battery_test_mavsdk.py --v2 --fast`.

**`--v2 --fault`**

- QGC battery widget shows `chargeState` = FAILED / fault indication.
- `status_flags` has bit 11 set (FLAG_FAULT_OVER_TEMP = 0x800).

**`--auto`**

- First ~5 s: QGC shows V1 data.
- QGC sends `SET_MESSAGE_INTERVAL(369, 2000000 µs)` → script logs `[AUTO] BATTERY_STATUS_V2 stream ON`.
- Within 2 s: V2 widget appears; QGC then sends `SET_MESSAGE_INTERVAL(147, -1)` → script logs `[AUTO] BATTERY_STATUS stream OFF`.
- If `SET_MESSAGE_INTERVAL` is intercepted by `mavsdk_server` (routing proxy mode), the 15 s fallback fires instead: script logs `[AUTO] Activating V2 (15 s after handshake)`.

**`--count 2`**

- QGC shows two battery entries (Battery 0 and Battery 1) with slightly different voltages (+0.05 V per id).

## Known test coverage gaps

These scenarios require manual verification or a more complex test harness:

- **Component-ID scoping of V2 activation**: The script acts as a single autopilot component. It cannot simulate a camera peripheral (different `compid`) sending BATTERY_STATUS_V2 at the same time as the autopilot sends BATTERY_STATUS. To verify this manually: attach a real camera that emits BATTERY_STATUS_V2 while the autopilot only emits BATTERY_STATUS and confirm QGC still shows the autopilot battery via V1.

- **MAVLink string field boundary**: All names used by this script are well under the field limit (`name` ≤ 50 B, `serial_number` ≤ 32 B, `manufacture_date` ≤ 9 B). A sender that fills an entire field without a null terminator is not exercised. To verify: use a flight-stack simulator that sends a 50-character battery name and confirm QGC displays it correctly without garbage characters.

## Failure modes

- **QGC shows no battery at all**: `mavsdk_server` did not connect.
  Check that QGC is listening on the correct port.
  The script prints `[bootstrap] mavsdk_server at port <N>` when the connection is established.
- **`--auto` never switches to V2**: `handshake_complete_at` never set → AUTOPILOT_VERSION request not received from QGC.
  Ensure QGC is in "vehicle connected" state, not just "link up".
- **V2 widget shows NaN for capacity**: `--no-info` was used, or QGC has not yet received a BATTERY_INFO frame.
  BATTERY_INFO is sent every 10 ticks (~10 s); wait longer.
- **`ImportError: cannot import name 'MavlinkDirect'`**: MAVSDK version too old. Run `pip install --upgrade mavsdk`.
