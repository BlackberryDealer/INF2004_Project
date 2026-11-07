#!/usr/bin/env python3
"""
fake_car.py - a pretend car for trying the dashboard without hardware.

Owner:  Buddy 1 (communications)

Usage (from the repo root, broker already running):

    py tools/dashboard/fake_car.py                    # broker on localhost
    py tools/dashboard/fake_car.py --host 10.14.95.64

Publishes the same packed payloads as the firmware, on the same topics and
QoS (layout from tools/telem_fields.py, so a layout.csv change follows
automatically): ONLINE retained on c28/a with OFFLINE as the will, STATE
on c28/s at 5 Hz, HEARTBEAT on c28/h at 1 Hz, events, the obstacle
profile, SUMMARY and SNAPSHOT on c28/e. It obeys c28/c like the mission
statechart does for the remote buttons:

    BOOT -> CAL_IMU -> CAL_IR -> READY      (about 4 s after start)
    READY   + START  -> 1 s warning -> RUN
    RUN     + STOP   -> FINISHED (SUMMARY sent)
    FINISHED / LINE_LOST_STOP + RESET -> READY;  FAULT + RESET -> BOOT
    any state + SNAP -> SNAPSHOT reply, state unchanged

A run also ends by itself at the end marker after --run-s seconds, passes
one obstacle on the way, and with --fault-at S raises a NO_ECHO fault S
seconds into the run (to try the FAULT -> RESET path). It is a demo aid,
not a model of the car: numbers are plausible, not measured.

Needs paho-mqtt (pip install paho-mqtt).
"""

import argparse
import math
import os
import random
import sys
import time

DASH_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(DASH_DIR))

import telem_cmd     # noqa: E402
import telem_decode  # noqa: E402
from telem_fields import GROUPS  # noqa: E402

T_STATE, T_EVENT, T_HB, T_AVAIL, T_CMD = "c28/s", "c28/e", "c28/h", \
    "c28/a", "c28/c"
T_SENS, T_LOG = "c28/d", "c28/l"        # debug: raw sensors, console lines

SYS = {n: i for i, n in enumerate(telem_cmd.SYS_NAMES)}
RUN = {n: i for i, n in enumerate(telem_cmd.RUN_NAMES)}
FAULT_BIT = {n: 1 << i for i, n in enumerate(telem_cmd.FAULT_NAMES)}
TYPE = {name: t for t, (name, _g) in telem_decode.TYPE_GROUP.items()}
EVT, UM_PER_COUNT = telem_decode.read_app_types()
EVT = {name: code for code, name in EVT.items()}

MOT_FOLLOW, MOT_BRAKE, MOT_TURN_L = 1, 0, 4
END_STOP, END_MARKER, END_FAULT = 0, 1, 3
STOP_MASK = 0x3AF


class BitWriter:
    """LSB-first bit writer; mirrors app_bits_write (Part D.4)."""

    def __init__(self):
        self.bits = []

    def write(self, value, width):
        for i in range(width):
            self.bits.append((value >> i) & 1)

    def to_bytes(self):
        out = bytearray((len(self.bits) + 7) // 8)
        for i, bit in enumerate(self.bits):
            out[i >> 3] |= bit << (i & 7)
        return bytes(out)


def pack(group, values, mq_type):
    """Header byte + the group's fields (stored values, offset added)."""
    w = BitWriter()
    w.write(telem_decode.MQ_VERSION, GROUPS["MQ_HDR"][0][1])
    w.write(mq_type, GROUPS["MQ_HDR"][1][1])
    for name, bits, offset, _lsb, _unit in GROUPS[group]:
        raw = 0 if name == "PAD" else int(values.get(name, 0)) + offset
        raw = max(0, min(raw, (1 << bits) - 1))
        w.write(raw, bits)
    return w.to_bytes()


def event_payload(name, fields=None):
    """MQ_EVT payload for one app event (value 11 bits, type 5 bits)."""
    group = telem_decode.EVT_GROUP_ALIAS.get(name, "EVT_" + name)
    value = 0
    if group in GROUPS:
        w = BitWriter()
        for fname, bits, _o, _l, _u in GROUPS[group]:
            w.write((fields or {}).get(fname, 0), bits)
        value = int.from_bytes(w.to_bytes() + b"\0\0", "little") & 0x7FF
    word = value | (EVT[name] << 11)
    return pack("MQ_EVT", {"WORD": word}, TYPE["EVENT"])


class FakeCar:
    def __init__(self, client, run_s, fault_at):
        self.client = client
        self.run_s = run_s
        self.fault_at = fault_at
        self.t0 = time.monotonic()
        self.sys = SYS["BOOT"]
        self.run = RUN["FOLLOW"]
        self.entered = self.t0
        self.warn_until = None
        self.faults = 0
        self.odo_mm = 0.0
        self.pos = [0.0, 0.0]
        self.speed = [0.0, 0.0]
        self.hump_peak = 0
        self.pitch = 0.0
        self.end_reason = END_STOP
        self.run_start = None
        self.run_end = None
        self.drops = 0
        self.lat_max = 0
        self.n_state = 0
        self.avoided = False
        self.humped = False
        self.barcode_done = False

    # ---- helpers ------------------------------------------------------
    def pub(self, topic, payload, qos=1, retain=False):
        self.client.publish(topic, payload, qos=qos, retain=retain)

    def evt(self, name, fields=None):
        self.pub(T_EVENT, event_payload(name, fields))

    def log(self, text):
        """One console line on c28/l, as the car's mirror sends it."""
        self.pub(T_LOG, bytes([telem_cmd.header(TYPE["LOG"])])
                 + text.encode("ascii", "replace")[:120], qos=0)

    def enter(self, sys_name, run_name="FOLLOW"):
        self.sys = SYS[sys_name]
        self.run = RUN[run_name]
        self.entered = time.monotonic()
        self.evt("STATE_CHANGE", {"SYS": self.sys, "RUN": self.run})
        self.log("STATE %s" % sys_name)
        print("fake_car: %s%s" % (sys_name, (" / " + run_name)
                                  if sys_name == "RUN" else ""), flush=True)

    def run_time(self):
        if self.run_start is None:
            return 0
        end = self.run_end if self.run_end is not None else time.monotonic()
        return int(end - self.run_start)

    def end_run(self, sys_name, reason):
        self.run_end = time.monotonic()
        self.end_reason = reason
        self.speed = [0.0, 0.0]
        self.enter(sys_name)
        self.pub(T_EVENT, pack("MQ_SUMM", {
            "END_REASON": reason, "HUMP_PEAK": self.hump_peak,
            "ODO_MM": int(self.odo_mm), "FAULTS": self.faults,
            "RUN_S": self.run_time()}, TYPE["SUMMARY"]))

    # ---- commands -------------------------------------------------------
    def on_command(self, data):
        try:
            type_name, _group, fields = telem_decode.decode(data)
        except ValueError as err:
            print("fake_car: command rejected: %s" % err, flush=True)
            return
        print("fake_car: got %s (%s)" % (type_name, data.hex(" ").upper()),
              flush=True)
        if type_name == "CMD_START" and self.sys == SYS["READY"] \
                and self.warn_until is None:
            self.warn_until = time.monotonic() + 1.0
            self.evt("START")
        elif type_name == "CMD_STOP" and self.sys == SYS["RUN"]:
            self.evt("STOP")
            self.end_run("FINISHED", END_STOP)
        elif type_name == "CMD_RESET" and self.sys in (
                SYS["FINISHED"], SYS["LINE_LOST_STOP"], SYS["FAULT"]):
            self.evt("RESET")
            was_fault = self.sys == SYS["FAULT"]
            self.faults &= FAULT_BIT["WIFI_LOSS"]
            self.run_start = self.run_end = None
            self.odo_mm, self.hump_peak = 0.0, 0
            self.enter("BOOT" if was_fault else "READY")
        elif type_name == "CMD_SNAP":
            self.pub(T_EVENT, pack("MQ_SNAP", {
                "SYS": self.sys, "RUN": self.run,
                "HUMP_PEAK": self.hump_peak, "ODO_MM": int(self.odo_mm),
                "FAULTS": self.faults, "RUN_S": self.run_time()},
                TYPE["SNAPSHOT"]))
        elif type_name == "CMD_MAP":
            val = dict(fields)
            self.evt("MAP", {"CHAR": val["CHAR"], "CMD": val["CMD"]})
        else:
            print("fake_car: ignored in this state (as on the car)",
                  flush=True)

    # ---- 200 ms step ----------------------------------------------------
    def step(self, now):
        age = now - self.entered
        if self.sys == SYS["BOOT"] and age > 1.0:
            self.enter("CAL_IMU")
        elif self.sys == SYS["CAL_IMU"] and age > 2.0:
            self.evt("CAL_DONE", {"SOURCE": 0})
            self.enter("CAL_IR")
        elif self.sys == SYS["CAL_IR"] and age > 1.5:
            self.evt("CAL_DONE", {"SOURCE": 1})
            self.enter("READY")
        elif self.sys == SYS["READY"] and self.warn_until is not None \
                and now >= self.warn_until:
            self.warn_until = None
            self.run_start, self.run_end = now, None
            self.odo_mm, self.pos = 0.0, [0.0, 0.0]
            self.hump_peak = 0
            self.avoided = self.humped = self.barcode_done = False
            self.enter("RUN")
        elif self.sys == SYS["RUN"]:
            self.drive(now, now - self.run_start)

        period = 0.2
        for side in (0, 1):
            self.pos[side] += self.speed[side] * period * 1000.0 \
                / UM_PER_COUNT
        self.odo_mm += (self.speed[0] + self.speed[1]) / 2.0 * period
        self.publish_state(now)
        self.publish_sensors(now)

    def publish_sensors(self, now):
        """Raw sensors on c28/d (stored values: 4 mm / 4 mg / 250 us)."""
        moving = self.sys == SYS["RUN"]
        steer = getattr(self, "steer", 0.0) if moving else 0.0
        ir_l = int(1500 + 1.6 * steer + random.uniform(-60, 60))
        ir_r = int(1500 - 1.6 * steer + random.uniform(-60, 60))
        dist = 2044
        t = now - self.run_start if (moving and self.run_start) else 0.0
        if moving and 15.0 < t < 18.0:
            dist = int(600 - (t - 15.0) * 130)
        self.edges = getattr(self, "edges", 0) + \
            (random.randint(2, 5) if moving and 10.8 < t < 11.4 else 0)
        gap_us = random.randint(10300, 12400)
        self.pub(T_SENS, pack("MQ_SENS", {
            "IR_L": max(0, min(ir_l, 4095)), "IR_R": max(0, min(ir_r, 4095)),
            "US_MM": dist // 4, "BAR_DO": 1 if 10.8 < t < 11.4 else 0,
            "BAR_EDGES": self.edges & 0xFF,
            "AX": int((random.uniform(-15, 15) + self.pitch * 1.7) / 4),
            "AY": int(random.uniform(-15, 15) / 4),
            "AZ": int((1000 + random.uniform(-20, 20)) / 4),
            "WDT_GAP": gap_us // 250, "SCAN": 1 if dist < 600 else 0},
            TYPE["SENSORS"]), qos=0)
        if not moving and self.n_state % 3 == 0:
            self.log("IR raw L %d R %d" % (ir_l, ir_r))
            self.log("US dist %d mm, lost %d" % (dist, 1 if dist >= 2044
                                                 else 0))
        if self.n_state % 10 == 0:
            self.log("WDT longest feed gap %d us in the last 2 s (watchdog "
                     "fires at 30000)" % gap_us)

    def drive(self, now, t):
        base = 180.0
        steer = 220.0 * math.sin(t * 0.9) + random.uniform(-40, 40)
        self.pitch = 0.0
        self.run = RUN["FOLLOW"]
        if 6.0 < t < 8.0:                       # hump
            self.pitch = 120.0 * math.sin((t - 6.0) * math.pi / 2.0)
            if not self.humped and t > 7.0:
                self.humped = True
                self.hump_peak = 42
                self.evt("HUMP", {"PEAK_MM": 42})
        if 11.0 < t < 11.3 and not self.barcode_done:
            self.barcode_done = True
            self.evt("BARCODE", {"CHAR": 32, "CMD": 0})     # W -> LEFT
            self.log("BARCODE value 32: armed, slow until the next junction")
        if 12.0 < t < 14.0:
            self.run = RUN["EXECUTE_TURN"]
            steer = -700.0
        if 18.0 < t < 22.0:
            self.run = RUN["AVOID"]
            if not self.avoided:
                self.avoided = True
                self.evt("OBSTACLE_AHEAD")
                self.log("OBSTACLE AHEAD: distance 560 mm, slowing down")
                self.log("OBSTACLE width 160 mm, nearest 220 mm ahead, "
                         "pass side 0 (0 = left, 1 = right; 1020 = clear)")
                self.pub(T_EVENT, pack("MQ_PROF", {
                    "ANGLE_STEP": 12, "WIDTH": 40, "NEAREST": 55,
                    "CLEAR_L": 120, "CLEAR_R": 70, "SIDE": 0},
                    TYPE["PROFILE"]))
                self.evt("PLAN", {"SIDE": 0})
            steer = 400.0 * math.sin((t - 18.0) * math.pi / 2.0)
        diff = steer / 1000.0 * base * 0.6
        self.speed = [base + diff + random.uniform(-8, 8),
                      base - diff + random.uniform(-8, 8)]
        self.steer = steer
        if self.fault_at is not None and t >= self.fault_at:
            self.faults |= FAULT_BIT["NO_ECHO"]
            self.evt("SENSOR_FAULT", {"FAULT_IDX": 3})
            self.end_run("FAULT", END_FAULT)
            self.fault_at = None
        elif t >= self.run_s:
            self.evt("STOP")
            self.end_run("FINISHED", END_MARKER)

    def publish_state(self, now):
        moving = self.sys == SYS["RUN"]
        steer = getattr(self, "steer", 0.0) if moving else 0.0
        turn = (self.speed[0] - self.speed[1]) / 120.0 * 57.3 \
            if moving else 0.0
        if moving and self.pitch > 30:
            mclass = 3
        elif moving and abs(turn) > 40:
            mclass = 2
        elif moving:
            mclass = 6
        else:
            mclass = 0
        cps = [s * 50.0 / UM_PER_COUNT for s in self.speed]  # count/50 ms
        self.pub(T_STATE, pack("MQ_STATE", {
            "SYS": self.sys, "RUN": self.run,
            "MOT": MOT_FOLLOW if moving else MOT_BRAKE,
            "SPEED_L": round(cps[0]), "SPEED_R": round(cps[1]),
            "POS_L": round(self.pos[0]), "POS_R": round(self.pos[1]),
            "LINE": 0 if moving or self.sys == SYS["READY"] else 1,
            "STEER": round(steer), "ODO_MM": int(self.odo_mm),
            "PITCH": round(self.pitch), "HUMP_PEAK": self.hump_peak,
            "TURN": round(turn), "MCLASS": mclass,
            "FAULTS": self.faults}, TYPE["STATE"]), qos=0)
        self.n_state += 1
        if self.n_state % 5 == 0:
            if random.random() < 0.05:
                self.drops = min(255, self.drops + 1)
            self.lat_max = max(self.lat_max, random.randint(300, 900))
            self.pub(T_HB, pack("MQ_HB", {
                "UPTIME_S": int(now - self.t0), "DROPS": self.drops,
                "LAT_MAX_US": self.lat_max}, TYPE["HEARTBEAT"]), qos=0)


def main(argv):
    ap = argparse.ArgumentParser(description="Pretend car for the dashboard.")
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--run-s", type=float, default=30.0,
                    help="run ends at the marker after S seconds (30)")
    ap.add_argument("--fault-at", type=float, default=None, metavar="S",
                    help="raise a NO_ECHO fault S seconds into the run")
    args = ap.parse_args(argv)
    try:
        import paho.mqtt.client as mqtt
    except ImportError:
        print("fake_car: needs paho-mqtt (pip install paho-mqtt)",
              file=sys.stderr)
        return 2

    if hasattr(mqtt, "CallbackAPIVersion"):              # paho-mqtt >= 2.0
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                             client_id="c28-fake-car")
    else:
        client = mqtt.Client(client_id="c28-fake-car")
    offline = bytes([telem_cmd.header(TYPE["OFFLINE"])])     # 0x31
    online = bytes([telem_cmd.header(TYPE["ONLINE"])])       # 0x29
    client.will_set(T_AVAIL, offline, qos=1, retain=True)
    car = FakeCar(client, args.run_s, args.fault_at)

    def on_connect(c, _userdata, _flags, *_rest):
        c.subscribe(T_CMD, qos=1)
        c.publish(T_AVAIL, online, qos=1, retain=True)
        print("fake_car: online at %s:%d" % (args.host, args.port),
              flush=True)

    def on_message(_c, _userdata, msg):
        car.on_command(bytes(msg.payload))

    client.on_connect = on_connect
    client.on_message = on_message
    try:
        client.connect(args.host, args.port, keepalive=10)
    except OSError as err:
        print("fake_car: cannot reach the broker at %s:%d (%s)"
              % (args.host, args.port, err), file=sys.stderr)
        return 1
    client.loop_start()
    nxt = time.monotonic()
    try:
        while True:
            nxt += 0.2
            car.step(time.monotonic())
            time.sleep(max(0.0, nxt - time.monotonic()))
    except KeyboardInterrupt:
        pass
    finally:
        client.publish(T_AVAIL, offline, qos=1, retain=True).wait_for_publish(2)
        client.loop_stop()
        client.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
