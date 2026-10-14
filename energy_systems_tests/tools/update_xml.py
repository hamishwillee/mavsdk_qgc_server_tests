#!/usr/bin/env python3
"""
update_xml.py - Refresh xml/energy_messages.xml from a mavlink/mavlink git ref
(e.g. the head of PR #2526), copying the definitions verbatim.

Usage:
  git -C ~/github/mavlink/mavlink fetch upstream pull/2526/head:pr-2526-energy
  python3 tools/update_xml.py ~/github/mavlink/mavlink pr-2526-energy
"""

import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / 'xml' / 'energy_messages.xml'

ENUMS = [('common', 'MAV_BATTERY_FUNCTION'), ('common', 'MAV_BATTERY_TYPE'),
         ('common', 'MAV_ENERGY_SYSTEM_FLAGS'), ('common', 'MAV_FUEL_TYPE'),
         ('development', 'MAV_BATTERY_STATUS_FLAGS'), ('development', 'MAV_FUEL_STATUS_FLAGS'),
         ('development', 'MAV_ENERGY_STATUS_FLAGS')]
MESSAGES = [('common', 'BATTERY_INFO'), ('development', 'BATTERY_STATUS_V2'),
            ('development', 'BATTERY_CELL_STATUS'), ('development', 'ENERGY_SYSTEM_INFO'),
            ('development', 'ENERGY_SYSTEM_STATUS'), ('development', 'FUEL_INFORMATION_V2'),
            ('development', 'FUEL_STATUS_V2')]


def main() -> None:
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    repo, ref = sys.argv[1], sys.argv[2]
    sha = subprocess.check_output(['git', '-C', repo, 'rev-parse', ref], text=True).strip()
    src = {name: subprocess.check_output(['git', '-C', repo, 'show', f'{ref}:message_definitions/v1.0/{name}.xml'],
                                         text=True) for name in ('common', 'development')}

    def grab(dialect: str, tag: str, name: str) -> str:
        m = re.search(r'( *)<%s (?:id="\d+" )?name="%s"[^>]*>.*?</%s>' % (tag, name, tag), src[dialect], re.S)
        if not m:
            sys.exit(f"{tag} {name} not found in {dialect}.xml at {ref}")
        return m.group(0)

    out = ['<?xml version="1.0"?>',
           '<!-- Energy Systems Protocol definitions, copied verbatim from',
           f'     mavlink/mavlink#2526 (head {sha}).',
           '     Loaded into mavsdk_server via MavlinkDirect.load_custom_xml() (and used by',
           '     tools/fake_gcs.py to generate an independent pymavlink decoder). Field order and',
           '     types must match upstream exactly so that CRC_EXTRA matches what QGC compiles.',
           '     Regenerate with tools/update_xml.py when the PR changes. -->',
           '<mavlink>', '  <enums>']
    out += [grab(d, 'enum', n) for d, n in ENUMS]
    out += ['  </enums>', '  <messages>']
    out += [grab(d, 'message', n) for d, n in MESSAGES]
    out += ['  </messages>', '</mavlink>', '']
    OUT.write_text('\n'.join(out), encoding='utf-8')
    print(f"Wrote {OUT} from {ref} ({sha})")


if __name__ == '__main__':
    main()
