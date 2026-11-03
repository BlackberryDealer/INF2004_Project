#!/usr/bin/env python3
"""
telem_cmd.py - send one command to the car over MQTT (topic c28/c).

Owner:  Buddy 1 (communications)
Spec:   CSC28 AI spec, B1.4 (commands), B1.7 (new command = new
        APP_MQ_TYPE_CMD_*); FR11 remote START / STOP, FR12 two-way link

Usage (from the repo root; broker on this laptop unless --host is given):

    py tools/telem_cmd.py start            # same as pressing RUN in READY
    py tools/telem_cmd.py stop             # ends a run in FINISHED
    py tools/telem_cmd.py reset            # same as holding RESET for 2 s
    py tools/telem_cmd.py summary          # "send me a summary now"
    py tools/telem_cmd.py map W left       # barcode W now means LEFT
    py tools/telem_cmd.py --host 10.14.95.64 summary

Commands are single binary bytes (header = version 1 | type << 3):
START 0x81, STOP 0x89, RESET 0x91, SNAP 0xA1; MAP is 0x99 followed by
one byte: Code 39 character index in bits 0-5, command in bits 6-7.

"summary" waits up to --wait seconds for the car's SNAPSHOT reply on
c28/e and prints it decoded (state, distance and time of the run so far,
highest hump, faults). Every command is published at QoS 1. The car
accepts one command of each type per 100 ms; extra ones are dropped and
counted.

Needs paho-mqtt (pip install paho-mqtt). --print-only shows the bytes
without connecting (no paho-mqtt needed). --self-test (host test S1-U43)
checks the tables here against app_program/app_types.h and decodes a
known SNAPSHOT payload; it needs only the standard library.
"""

import argparse
import os
import re
import sys
import threading

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, TOOLS_DIR)

import telem_decode  # noqa: E402  (same folder; standard library only)

TOPIC_CMD = "c28/c"
TOPIC_EVENT = "c28/e"

MQ_VERSION = 1

# APP_MQ_TYPE_CMD_* in app_program/app_types.h
CMD_TYPES = {
    "start": 16,
    "stop": 17,
    "reset": 18,
    "map": 19,
    "summary": 20,
}
MQ_TYPE_SNAPSHOT = 7

# APP_BAR_CMD_* in app_program/app_types.h
BAR_CMDS = {"left": 0, "right": 1, "straight": 2, "uturn": 3}

# Code 39 character index order (app_program/subsystem3/code39.c);
# MAP accepts 0..42 ('*' is the start/stop character and cannot be mapped).
CODE39_CHARS = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%"

SYS_NAMES = ["BOOT", "CAL_IMU", "CAL_IR", "READY", "RUN", "FINISHED",
             "LINE_LOST_STOP", "FAULT"]
RUN_NAMES = ["FOLLOW", "ARMED", "EXECUTE_TURN", "SEARCH_LINE", "AVOID"]
FAULT_NAMES = ["MOTOR_STALL", "ENCODER", "IMU", "NO_ECHO", "WIFI_LOSS",
               "QUEUE_OVERFLOW", "RANGE_CLAMP", "CAL_TIMEOUT",
               "REACQUIRE_FAIL", "PIO_LOAD"]


def header(mq_type):
    """Byte 0 of a payload: version in bits 0-2, type in bits 3-7."""
    return MQ_VERSION | (mq_type << 3)


def command_bytes(name, char=None, bar_cmd=None):
    """The payload for one command name ('map' needs char and bar_cmd)."""
    if name != "map":
        return bytes([header(CMD_TYPES[name])])
    char_up = char.upper() if len(char) == 1 else char
    if len(char_up) != 1 or char_up not in CODE39_CHARS:
        raise ValueError("MAP character must be one of %r" % CODE39_CHARS)
    if bar_cmd.lower() not in BAR_CMDS:
        raise ValueError("MAP command must be one of %s"
                         % ", ".join(BAR_CMDS))
    value = CODE39_CHARS.index(char_up) | (BAR_CMDS[bar_cmd.lower()] << 6)
    return bytes([header(CMD_TYPES["map"]), value])


def snapshot_text(data):
    """Readable lines for a SNAPSHOT payload, or None if it is not one."""
    try:
        type_name, _group, fields = telem_decode.decode(data)
    except ValueError:
        return None
    if type_name != "SNAPSHOT":
        return None
    val = dict(fields)
    sys_name = SYS_NAMES[val["SYS"]] if val["SYS"] < len(SYS_NAMES) else "?"
    run_name = RUN_NAMES[val["RUN"]] if val["RUN"] < len(RUN_NAMES) else "?"
    faults = [n for i, n in enumerate(FAULT_NAMES) if val["FAULTS"] & (1 << i)]
    lines = [
        "state:     %s%s" % (sys_name,
                             (" / " + run_name) if sys_name == "RUN" else ""),
        "distance:  %d mm this run" % val["ODO_MM"],
        "run time:  %d s" % val["RUN_S"],
        "hump peak: %d mm" % val["HUMP_PEAK"],
        "faults:    %s" % (", ".join(faults) if faults else "none"),
    ]
    return "\n".join(lines)


def self_test():
    """S1-U43: command tables agree with app_types.h; known bytes."""
    fails = 0

    def check(ok, text):
        nonlocal fails
        print("[TEST] S1-U43 %s  %s" % ("PASS" if ok else "FAIL", text))
        fails += 0 if ok else 1

    consts = {}
    with open(telem_decode.APP_TYPES_H, encoding="utf-8") as fh:
        for line in fh:
            m = re.match(r"#define\s+(APP_\w+)\s+\((\d+)u?\)", line)
            if m:
                consts[m.group(1)] = int(m.group(2))
    names = {"start": "START", "stop": "STOP", "reset": "RESET",
             "map": "MAP", "summary": "SNAP"}
    check(all(consts.get("APP_MQ_TYPE_CMD_" + names[k]) == v
              for k, v in CMD_TYPES.items()),
          "command types match APP_MQ_TYPE_CMD_* in app_types.h")
    check(consts.get("APP_MQ_TYPE_SNAPSHOT") == MQ_TYPE_SNAPSHOT
          and consts.get("APP_MQ_VERSION") == MQ_VERSION,
          "SNAPSHOT type and version match app_types.h")
    check(all(consts.get("APP_BAR_CMD_" + k.upper()) == v
              for k, v in BAR_CMDS.items()),
          "barcode commands match APP_BAR_CMD_* in app_types.h")
    got = [command_bytes(c).hex() for c in ("start", "stop", "reset",
                                             "summary")]
    check(got == ["81", "89", "91", "a1"],
          "START 81, STOP 89, RESET 91, SNAP A1")
    check(command_bytes("map", "w", "left") == bytes([0x99, 0x20])
          and command_bytes("map", "%", "uturn") == bytes([0x99, 0xEA]),
          "MAP W left = 99 20; MAP % uturn = 99 EA")
    try:
        command_bytes("map", "*", "left")
        check(False, "MAP of '*' (start/stop character) is refused")
    except ValueError:
        check(True, "MAP of '*' (start/stop character) is refused")
    text = snapshot_text(bytes.fromhex("395C869D008401")) or ""
    check("RUN / SEARCH_LINE" in text and "1260 mm" in text
          and "6 s" in text and "RANGE_CLAMP" in text,
          "S1-U42 bytes decode to RUN / SEARCH_LINE, 1260 mm, 6 s")
    check(snapshot_text(bytes.fromhex("1965102700EC01")) is None,
          "a SUMMARY payload is not taken for a SNAPSHOT")
    print("[TEST] telem_cmd SUMMARY %d passed, %d failed"
          % (8 - fails, fails))
    return 1 if fails else 0


def send(host, port, payload, wait_reply_s):
    """Publishes payload on c28/c (QoS 1). With wait_reply_s > 0, waits for
    a SNAPSHOT on c28/e and prints it. Returns the process exit code."""
    try:
        import paho.mqtt.client as mqtt
    except ImportError:
        print("telem_cmd: needs paho-mqtt (pip install paho-mqtt)",
              file=sys.stderr)
        return 2

    subscribed = threading.Event()
    got_reply = threading.Event()

    def on_connect(client, _userdata, _flags, *_rest):
        if wait_reply_s > 0:
            client.subscribe(TOPIC_EVENT, qos=1)
        else:
            subscribed.set()

    def on_subscribe(_client, _userdata, *_rest):
        subscribed.set()

    def on_message(_client, _userdata, msg):
        text = snapshot_text(bytes(msg.payload))
        if text is not None and not got_reply.is_set():
            print(text, flush=True)
            got_reply.set()

    if hasattr(mqtt, "CallbackAPIVersion"):              # paho-mqtt >= 2.0
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    else:
        client = mqtt.Client()
    client.on_connect = on_connect
    client.on_subscribe = on_subscribe
    client.on_message = on_message
    try:
        client.connect(host, port, keepalive=30)
    except OSError as err:
        print("telem_cmd: cannot reach the broker at %s:%d (%s)"
              % (host, port, err), file=sys.stderr)
        return 1
    client.loop_start()
    rc = 0
    try:
        if not subscribed.wait(5.0):
            print("telem_cmd: no answer from the broker", file=sys.stderr)
            return 1
        info = client.publish(TOPIC_CMD, payload, qos=1)
        info.wait_for_publish(5.0)
        print("sent %s to %s" % (payload.hex(" ").upper(), TOPIC_CMD))
        if wait_reply_s > 0 and not got_reply.wait(wait_reply_s):
            print("telem_cmd: no SNAPSHOT reply within %.0f s (is the car "
                  "online? watch c28/a)" % wait_reply_s, file=sys.stderr)
            rc = 1
    finally:
        client.loop_stop()
        client.disconnect()
    return rc


def main(argv):
    ap = argparse.ArgumentParser(
        description="Send one command to the car over MQTT.")
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--wait", type=float, default=3.0, metavar="S",
                    help="seconds to wait for the summary reply (default 3)")
    ap.add_argument("--print-only", action="store_true",
                    help="print the command bytes, do not connect")
    ap.add_argument("--self-test", action="store_true",
                    help="host test S1-U43, then exit")
    ap.add_argument("command", nargs="?", type=str.lower,
                    choices=sorted(CMD_TYPES))
    ap.add_argument("char", nargs="?",
                    help="map only: Code 39 character, e.g. W")
    ap.add_argument("bar_cmd", nargs="?",
                    help="map only: left, right, straight or uturn")
    args = ap.parse_args(argv)
    if args.self_test:
        return self_test()
    if args.command is None:
        ap.error("give a command: " + ", ".join(sorted(CMD_TYPES)))

    if args.command == "map" and (args.char is None or args.bar_cmd is None):
        ap.error("map needs a character and a command, e.g. map W left")
    if args.command != "map" and args.char is not None:
        ap.error("only map takes extra arguments")
    try:
        payload = command_bytes(args.command, args.char, args.bar_cmd)
    except ValueError as err:
        ap.error(str(err))

    if args.print_only:
        print(payload.hex(" ").upper())
        return 0
    wait_s = args.wait if args.command == "summary" else 0.0
    return send(args.host, args.port, payload, wait_s)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
