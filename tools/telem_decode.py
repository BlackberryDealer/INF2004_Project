#!/usr/bin/env python3
"""
telem_decode.py - laptop decoder of the car's packed MQTT payloads.

Owner:  Buddy 1 (communications)
Spec:   CSC28 AI spec, B1.4 step 9 and 10, Part D.4 (LSB-first writer)

Usage (from the repo root):

    python3 tools/telem_decode.py                     # live, broker localhost
    python3 tools/telem_decode.py --host 192.168.1.20 --port 1883
    python3 tools/telem_decode.py --golden testing/subsystem1/host/golden

Live mode subscribes to c28/# and prints one line per message. Every
payload must be at least 1 byte long and carry version 1 in its header;
it is then decoded by its header type with the field table in
tools/telem_fields.py (generated from tools/layout.csv), offsets are
subtracted and units converted (speed counts/50 ms -> mm/s, 4 mm fields
x 4, 0.1 deg / 10).

--golden DIR decodes every *.bin in DIR and compares the stored
(offset-removed) value of every field with the matching *.csv written by
the host test S1-U10. Exit 1 on any mismatch (S1-U10), 0 when all match.

Live mode needs paho-mqtt (pip install paho-mqtt); --golden needs only the
Python 3 standard library.
"""

import argparse
import csv
import glob
import os
import re
import sys
import time

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, TOOLS_DIR)
from telem_fields import GROUPS  # noqa: E402  (generated, next to this file)

APP_TYPES_H = os.path.join(TOOLS_DIR, "..", "app_program", "app_types.h")

MQ_VERSION = 1
TOPIC_ALL = "c28/#"

# APP_MQ_TYPE_* (app_types.h) -> payload group in layout.csv. None: header
# only (availability messages and the 1-byte commands).
TYPE_GROUP = {
    0: ("STATE", "MQ_STATE"),
    1: ("EVENT", "MQ_EVT"),
    2: ("PROFILE", "MQ_PROF"),
    3: ("SUMMARY", "MQ_SUMM"),
    4: ("HEARTBEAT", "MQ_HB"),
    5: ("ONLINE", None),
    6: ("OFFLINE", None),
    16: ("CMD_START", None),
    17: ("CMD_STOP", None),
    18: ("CMD_RESET", None),
    19: ("CMD_MAP", "MQ_MAP"),
}

# APP_EVT_* names whose EVT_ group in layout.csv is spelled differently.
EVT_GROUP_ALIAS = {
    "MOTION_CLASS": "EVT_MCLASS",
    "SENSOR_FAULT": "EVT_SFAULT",
    "STATE_CHANGE": "EVT_STATE",
}

SPEED_WINDOW_MS = 50
WHEEL_CIRC_UM_DEFAULT = 204000
COUNTS_PER_REV_DEFAULT = 585


class BitReader:
    """LSB-first bit reader; mirrors app_bits_read (Part D.4)."""

    def __init__(self, data):
        self.data = data
        self.pos = 0

    def read(self, bits):
        value = 0
        for i in range(bits):
            byte = self.pos >> 3
            if byte >= len(self.data):
                raise ValueError("payload too short")
            if (self.data[byte] >> (self.pos & 7)) & 1:
                value |= 1 << i
            self.pos += 1
        return value


def payload_bytes(group):
    """Bytes of a group's payload after the header byte."""
    return (sum(f[1] for f in GROUPS[group]) + 7) // 8


def decode_stored(group, reader):
    """Stored (offset-removed) value of every non-PAD field, in CSV order."""
    out = []
    for name, bits, offset, _lsb, _unit in GROUPS[group]:
        raw = reader.read(bits)
        if name != "PAD":
            out.append((name, raw - offset))
    return out


def decode_header(data):
    """Return (version, type). Raises ValueError on a bad header."""
    if len(data) < 1:
        raise ValueError("empty payload")
    reader = BitReader(data)
    version = reader.read(GROUPS["MQ_HDR"][0][1])
    mq_type = reader.read(GROUPS["MQ_HDR"][1][1])
    if version != MQ_VERSION:
        raise ValueError("version %d, expected %d" % (version, MQ_VERSION))
    return version, mq_type, reader


def decode(data):
    """Return (type_name, group, [(field, stored value)]) for one payload."""
    _version, mq_type, reader = decode_header(data)
    if mq_type not in TYPE_GROUP:
        raise ValueError("unknown type %d" % mq_type)
    type_name, group = TYPE_GROUP[mq_type]
    if group is None:
        if len(data) != 1:
            raise ValueError("%s is %d bytes, expected 1"
                             % (type_name, len(data)))
        return type_name, None, []
    want = 1 + payload_bytes(group)
    if len(data) != want:
        raise ValueError("%s is %d bytes, expected %d"
                         % (type_name, len(data), want))
    return type_name, group, decode_stored(group, reader)


def read_app_types():
    """APP_EVT_* names and the wheel constants from app_types.h."""
    evt_names = {}
    consts = {"APP_WHEEL_CIRC_UM": WHEEL_CIRC_UM_DEFAULT,
              "APP_COUNTS_PER_REV": COUNTS_PER_REV_DEFAULT}
    try:
        with open(APP_TYPES_H, encoding="utf-8") as fh:
            for line in fh:
                m = re.match(r"#define\s+APP_EVT_(\w+)\s+\((\d+)u\)", line)
                if m and m.group(1) not in ("COUNT", "TYPE_BITS"):
                    evt_names[int(m.group(2))] = m.group(1)
                m = re.match(r"#define\s+(APP_WHEEL_CIRC_UM|"
                             r"APP_COUNTS_PER_REV)\s+\((\d+)u\)", line)
                if m:
                    consts[m.group(1)] = int(m.group(2))
    except OSError:
        pass
    circ = consts["APP_WHEEL_CIRC_UM"]
    cpr = consts["APP_COUNTS_PER_REV"]
    um_per_count = (circ + cpr // 2) // cpr     # same rounding as app_types.h
    return evt_names, um_per_count


def unit_value(group, name, stored, um_per_count):
    """Stored value converted to a printable engineering value."""
    for fname, _bits, _offset, lsb, unit in GROUPS[group]:
        if fname != name:
            continue
        if unit == "count/50ms":
            return "%.0fmm/s" % (stored * um_per_count / SPEED_WINDOW_MS)
        if unit == "0.1 deg":
            return "%.1fdeg" % (stored / 10.0)
        if unit in ("mm", "s", "us", "deg/s", "permille") or lsb != 1:
            return "%d%s" % (stored * lsb, "" if unit == "permille"
                             else unit.replace("permille", ""))
        return "%d" % stored
    return "%d" % stored


def event_text(word, evt_names):
    """'TYPE field=value ...' for a 16-bit app event word (MSG_EVT)."""
    reader = BitReader(bytes((word & 0xFF, word >> 8)))
    value = reader.read(GROUPS["MSG_EVT"][0][1])
    evt_type = reader.read(GROUPS["MSG_EVT"][1][1])
    name = evt_names.get(evt_type, "EVT%d" % evt_type)
    group = EVT_GROUP_ALIAS.get(name, "EVT_" + name)
    if group not in GROUPS:
        return "%s value=%d" % (name, value)
    reader = BitReader(bytes((value & 0xFF, value >> 8)))
    fields = decode_stored(group, reader)
    return name + "".join(" %s=%d" % f for f in fields)


def line_for(topic, data, evt_names, um_per_count):
    """One printable line for one received message."""
    try:
        type_name, group, fields = decode(data)
    except ValueError as err:
        return "%s REJECT %s (%d B)" % (topic, err, len(data))
    if group == "MQ_EVT":
        body = event_text(fields[0][1], evt_names)
    elif group is None:
        body = ""
    else:
        body = " ".join("%s=%s" % (n, unit_value(group, n, v, um_per_count))
                        for n, v in fields)
    return ("%s %s (%d B) %s" % (topic, type_name, len(data), body)).rstrip()


def read_golden_csv(path):
    with open(path, newline="", encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    return [(r["field"], int(r["value"])) for r in rows]


def golden(directory):
    """S1-U10: every *.bin must decode to its *.csv. Returns exit code."""
    bins = sorted(glob.glob(os.path.join(directory, "*.bin")))
    if not bins:
        print("[TEST] S1-U10 FAIL  no *.bin in %s" % directory)
        return 1
    fails = 0
    for path in bins:
        name = os.path.splitext(os.path.basename(path))[0]
        csv_path = os.path.splitext(path)[0] + ".csv"
        with open(path, "rb") as fh:
            data = fh.read()
        try:
            _type_name, _group, got = decode(data)
            want = read_golden_csv(csv_path)
        except (OSError, ValueError, KeyError) as err:
            print("[TEST] S1-U10 FAIL  %s: %s" % (name, err))
            fails += 1
            continue
        if got == want:
            print("[TEST] S1-U10 PASS  %s decodes to its CSV (%d fields)"
                  % (name, len(got)))
        else:
            diff = [("%s got %s want %s" % (g[0], g[1], w[1]))
                    for g, w in zip(got, want) if g != w]
            if len(got) != len(want):
                diff.append("%d fields, CSV has %d" % (len(got), len(want)))
            print("[TEST] S1-U10 FAIL  %s: %s" % (name, "; ".join(diff)))
            fails += 1
    print("[TEST] telem_decode golden: %d PASS %d FAIL"
          % (len(bins) - fails, fails))
    return 1 if fails else 0


def live(host, port):
    try:
        import paho.mqtt.client as mqtt
    except ImportError:
        print("telem_decode: live mode needs paho-mqtt "
              "(pip install paho-mqtt)", file=sys.stderr)
        return 2
    evt_names, um_per_count = read_app_types()

    def on_connect(client, _userdata, _flags, *_rest):
        client.subscribe(TOPIC_ALL, qos=1)

    def on_message(_client, _userdata, msg):
        stamp = time.strftime("%H:%M:%S")
        print(stamp, line_for(msg.topic, bytes(msg.payload), evt_names,
                              um_per_count), flush=True)

    if hasattr(mqtt, "CallbackAPIVersion"):              # paho-mqtt >= 2.0
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    else:
        client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(host, port, keepalive=30)
    try:
        client.loop_forever()
    except KeyboardInterrupt:
        pass
    return 0


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--golden", metavar="DIR",
                    help="compare DIR/*.bin with DIR/*.csv and exit")
    args = ap.parse_args(argv)
    if args.golden:
        return golden(args.golden)
    return live(args.host, args.port)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
