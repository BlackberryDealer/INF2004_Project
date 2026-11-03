#!/usr/bin/env python3
"""
gen_tables.py - write the dashboard's lookup tables (telem_tables.js).

Owner:  Buddy 1 (communications)
Spec:   CSC28 AI spec, B1.4 (telemetry and commands), Part D.11 (layout)

Usage (from the repo root):

    python3 tools/dashboard/gen_tables.py           # (re)write telem_tables.js
    python3 tools/dashboard/gen_tables.py --check   # exit 1 if it is stale

The dashboard must decode and encode exactly like tools/telem_decode.py
and tools/telem_cmd.py, so it does not keep tables of its own. This script
imports those two tools (and the generated tools/telem_fields.py), reads
the enum names, wheel constants and topics from app_program/app_types.h
and app_program/subsystem1/cfg_telem.h, and writes them as one JavaScript
object. Rerun it after a layout.csv or app_types.h change; the host test
S1-U44 fails while the committed file is stale.

Python 3 standard library only. Same inputs, byte-identical output.
"""

import json
import os
import re
import sys

DASH_DIR = os.path.dirname(os.path.abspath(__file__))
TOOLS_DIR = os.path.dirname(DASH_DIR)
REPO_DIR = os.path.dirname(TOOLS_DIR)
sys.path.insert(0, TOOLS_DIR)

import telem_cmd     # noqa: E402  (tools/, standard library only)
import telem_decode  # noqa: E402
from telem_fields import GROUPS  # noqa: E402

APP_TYPES_H = os.path.join(REPO_DIR, "app_program", "app_types.h")
CFG_TELEM_H = os.path.join(REPO_DIR, "app_program", "subsystem1",
                           "cfg_telem.h")
OUT_JS = os.path.join(DASH_DIR, "telem_tables.js")

# Enums read from app_types.h: table name -> #define prefix. Each one must
# run 0, 1, 2, ... in file order; the first gap ends it (APP_SCAN_* is
# followed by APP_SCAN_ANGLE_MIN_DEG = 30, which is not a scan state).
ENUM_PREFIXES = (
    ("MOT", "APP_MOT_"),
    ("LINE", "APP_LINE_"),
    ("JCT", "APP_JCT_"),
    ("BAR_CMD", "APP_BAR_CMD_"),
    ("MCLASS", "APP_MCLASS_"),
    ("AVOID", "APP_AVOID_"),
    ("END", "APP_END_"),
    ("SCAN", "APP_SCAN_"),
    ("SIDE", "APP_SIDE_"),
    ("CAL_SRC", "APP_CAL_SRC_"),
)

# Only the payload groups the laptop sees.
GROUP_PREFIXES = ("MQ_", "EVT_", "MSG_EVT")


def read_defines(path):
    """(name, value) of every '#define NAME (123u)' / '(0x1Fu)' line."""
    out = []
    with open(path, encoding="utf-8") as fh:
        for line in fh:
            m = re.match(r"#define\s+(\w+)\s+\((0x[0-9A-Fa-f]+|\d+)u?\)",
                         line)
            if m:
                out.append((m.group(1), int(m.group(2), 0)))
    return out


def read_strings(path):
    """name -> value of every '#define NAME "text"' line."""
    out = {}
    with open(path, encoding="utf-8") as fh:
        for line in fh:
            m = re.match(r'#define\s+(\w+)\s+"([^"]*)"', line)
            if m:
                out[m.group(1)] = m.group(2)
    return out


def enum_names(defines, prefix):
    """Names of the 0, 1, 2, ... run of defines starting with prefix."""
    names = []
    for name, value in defines:
        if not name.startswith(prefix):
            continue
        if value != len(names):
            break
        names.append(name[len(prefix):])
    if not names:
        raise SystemExit("gen_tables: no %s* enum in app_types.h" % prefix)
    return names


def build():
    """The table object written to telem_tables.js."""
    defines = read_defines(APP_TYPES_H)
    consts = dict(defines)
    telem = dict(read_defines(CFG_TELEM_H))
    topics = read_strings(CFG_TELEM_H)
    evt_names, um_per_count = telem_decode.read_app_types()

    groups = {}
    for group, fields in GROUPS.items():
        if group.startswith(GROUP_PREFIXES):
            groups[group] = [list(f) for f in fields]

    enums = {
        "SYS": list(telem_cmd.SYS_NAMES),
        "RUN": list(telem_cmd.RUN_NAMES),
        "FAULT": list(telem_cmd.FAULT_NAMES),
    }
    for table, prefix in ENUM_PREFIXES:
        enums[table] = enum_names(defines, prefix)

    return {
        "MQ_VERSION": telem_decode.MQ_VERSION,
        "GROUPS": groups,
        "TYPE_GROUP": {str(k): list(v) for k, v in
                       sorted(telem_decode.TYPE_GROUP.items())},
        "EVT_NAMES": {str(k): v for k, v in sorted(evt_names.items())},
        "EVT_GROUP_ALIAS": dict(sorted(telem_decode.EVT_GROUP_ALIAS.items())),
        "UM_PER_COUNT": um_per_count,
        "SPEED_WINDOW_MS": telem_decode.SPEED_WINDOW_MS,
        "CMD_TYPES": dict(telem_cmd.CMD_TYPES),
        "BAR_CMDS": dict(telem_cmd.BAR_CMDS),
        "CODE39_CHARS": telem_cmd.CODE39_CHARS,
        "ENUMS": enums,
        "FAULT_STOP_MASK": consts["APP_FAULT_STOP_MASK"],
        "PROFILE_CLEAR_MM": consts["APP_PROFILE_CLEAR_MM"],
        "SCAN_ANGLE_MIN_DEG": consts["APP_SCAN_ANGLE_MIN_DEG"],
        "SCAN_ANGLE_STEP_DEG": consts["APP_SCAN_ANGLE_STEP_DEG"],
        "STATE_PERIOD_MS": telem["TELEM_STATE_PERIOD_MS"],
        "HB_EVERY_N_STATE": telem["TELEM_HB_EVERY_N_STATE"],
        "CMD_MIN_GAP_MS": telem["TELEM_CMD_MIN_GAP_MS"],
        "TOPICS": {
            "STATE": topics["TELEM_TOPIC_STATE"],
            "EVENT": topics["TELEM_TOPIC_EVENT"],
            "HB": topics["TELEM_TOPIC_HB"],
            "AVAIL": topics["TELEM_TOPIC_AVAIL"],
            "CMD": topics["TELEM_TOPIC_CMD"],
        },
    }


def render(tables):
    body = json.dumps(tables, indent=1, sort_keys=False)
    return (
        "/*\n"
        " * telem_tables.js - lookup tables for the telemetry dashboard.\n"
        " *\n"
        " * GENERATED by tools/dashboard/gen_tables.py from "
        "tools/telem_fields.py,\n"
        " * tools/telem_decode.py, tools/telem_cmd.py, "
        "app_program/app_types.h and\n"
        " * app_program/subsystem1/cfg_telem.h. Do not edit.\n"
        " */\n"
        "(function (root) {\n"
        "    \"use strict\";\n"
        "    var T = " + body.replace("\n", "\n    ") + ";\n"
        "    if (typeof module !== \"undefined\" && module.exports) {\n"
        "        module.exports = T;\n"
        "    } else {\n"
        "        root.TELEM_TABLES = T;\n"
        "    }\n"
        "}(this));\n"
    )


def main(argv):
    text = render(build())
    if "--check" in argv:
        try:
            with open(OUT_JS, encoding="utf-8", newline="") as fh:
                current = fh.read()
        except OSError:
            current = None
        if current != text:
            print("gen_tables: %s is stale; run python3 "
                  "tools/dashboard/gen_tables.py" % os.path.relpath(OUT_JS))
            return 1
        print("gen_tables: telem_tables.js is up to date")
        return 0
    with open(OUT_JS, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(text)
    print("gen_tables: wrote %s" % os.path.relpath(OUT_JS))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
