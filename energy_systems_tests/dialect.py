"""
dialect.py - pymavlink dialect generated from xml/energy_messages.xml.

Used by tools/fake_gcs.py to decode the simulator's output independently of
mavsdk_server, and by energy_sim.py to pack messages that mavsdk_server cannot
encode (see RAW_MESSAGES in energy_sim.py).

The module is regenerated into .generated/ whenever the XML is newer.
"""

import contextlib
import importlib.util
import io
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
XML_PATH = ROOT / 'xml' / 'energy_messages.xml'
GEN_DIR = ROOT / '.generated'
GEN_PATH = GEN_DIR / 'energy_dialect.py'

_module = None


def load_energy_dialect():
    global _module
    if _module is not None:
        return _module
    if not GEN_PATH.exists() or GEN_PATH.stat().st_mtime < XML_PATH.stat().st_mtime:
        from pymavlink.generator import mavgen
        GEN_DIR.mkdir(exist_ok=True)
        opts = mavgen.Opts(str(GEN_PATH), wire_protocol='2.0', language='Python', validate=True)
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            ok = mavgen.mavgen(opts, [str(XML_PATH)])
        if not ok:
            sys.exit(f"mavgen failed for {XML_PATH}:\n{out.getvalue()}")
    spec = importlib.util.spec_from_file_location('energy_dialect', GEN_PATH)
    _module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(_module)
    return _module
