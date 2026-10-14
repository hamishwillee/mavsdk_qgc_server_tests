#!/usr/bin/env python3
"""
check_scenarios.py - Run energy_sim.py against tools/fake_gcs.py for each scenario
and report which pass. This checks the simulator itself (wire format, protocol
behaviour, scenario validity), not QGC.

QGC must not be running on the chosen port.

Usage:
  python3 tools/check_scenarios.py                       # all scenarios
  python3 tools/check_scenarios.py single_battery faults  # selected
  python3 tools/check_scenarios.py --keep-logs            # write logs to .generated/logs/
"""

import argparse
import os
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PY = sys.executable


def _killpg(pgid: int, sig) -> None:
    try:
        os.killpg(pgid, sig)
    except ProcessLookupError:
        pass


def _wait_port_free(port: int, timeout: float = 10.0) -> None:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        with socket.socket() as s:
            if s.connect_ex(('127.0.0.1', port)) != 0:
                return
        time.sleep(0.2)


def run_one(name: str, port: int, duration: float, log_dir: Path | None) -> tuple[bool, str]:
    scen = ROOT / 'scenarios' / f'{name}.yaml'
    gcs = subprocess.Popen([PY, '-W', 'ignore', str(ROOT / 'tools' / 'fake_gcs.py'), '-s', str(scen),
                            '--port', str(port), '--duration', str(duration)],
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, cwd=ROOT)
    time.sleep(0.5)
    # Own process group, so that mavsdk_server (a child of the simulator) is stopped too:
    # a leftover mavsdk_server keeps gRPC port 50051 and the next run talks to it.
    sim = subprocess.Popen([PY, '-W', 'ignore', str(ROOT / 'energy_sim.py'), '-s', str(scen),
                            '--port', str(port), '--speed', '5'],
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, cwd=ROOT,
                           start_new_session=True)
    try:
        gcs_out, _ = gcs.communicate(timeout=duration + 60)
    except subprocess.TimeoutExpired:
        gcs.kill()
        gcs_out, _ = gcs.communicate()
    _killpg(sim.pid, signal.SIGINT)
    try:
        sim_out, _ = sim.communicate(timeout=10)
    except subprocess.TimeoutExpired:
        _killpg(sim.pid, signal.SIGKILL)
        sim_out, _ = sim.communicate()
    _killpg(sim.pid, signal.SIGKILL)     # anything left in the group (mavsdk_server)
    _wait_port_free(50051)
    if log_dir:
        log_dir.mkdir(parents=True, exist_ok=True)
        (log_dir / f'{name}.gcs.log').write_text(gcs_out)
        (log_dir / f'{name}.sim.log').write_text(sim_out)
    errors = [ln for ln in sim_out.splitlines() if '[ERROR]' in ln or 'Failed to set' in ln or 'Traceback' in ln]
    passed = gcs.returncode == 0 and not errors
    detail = '\n'.join([ln for ln in gcs_out.splitlines() if 'FAIL' in ln] + errors)
    return passed, detail


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('scenarios', nargs='*')
    parser.add_argument('--port', type=int, default=14560)
    parser.add_argument('--duration', type=float, default=10.0)
    parser.add_argument('--keep-logs', action='store_true')
    args = parser.parse_args()
    names = args.scenarios or sorted(p.stem for p in (ROOT / 'scenarios').glob('*.yaml'))
    log_dir = ROOT / '.generated' / 'logs' if args.keep_logs else None
    results = []
    for name in names:
        print(f"--- {name} ...", flush=True)
        ok, detail = run_one(name, args.port, args.duration, log_dir)
        results.append((name, ok))
        print(f"    {'PASS' if ok else 'FAIL'}", flush=True)
        if detail:
            print('\n'.join(f"    {ln}" for ln in detail.splitlines()), flush=True)
    failed = [n for n, ok in results if not ok]
    print(f"\n{len(results) - len(failed)}/{len(results)} scenarios passed"
          + (f"; failed: {', '.join(failed)}" if failed else ''))
    sys.exit(1 if failed else 0)


if __name__ == '__main__':
    main()
