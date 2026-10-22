#!/usr/bin/env python3
"""
mqtt_size_log.py - log the size of every MQTT message seen on the broker.

Owner:  Buddy 1 (communications)
Spec:   CSC28 AI spec, B1.4 step 9, B1.5 (docs/message_sizes.md), S1-I10

Usage (from the repo root, broker on this laptop):

    python3 tools/mqtt_size_log.py --out testing/subsystem1/logs/sizes_w10.csv
    python3 tools/mqtt_size_log.py --host 192.168.1.20 --topic '#' \
        --out sizes.csv --seconds 60

Subscribes to TOPIC (default '#') and writes one CSV row per message:

    time, topic, topic_len, payload_len, qos, packet_len

packet_len is the size of the MQTT PUBLISH packet the broker received:
    rem        = 2 + topic_len + (2 if qos > 0 else 0) + payload_len
    packet_len = 1 + (1 if rem < 128 else 2) + rem
(fixed header byte, remaining-length field, then the variable header and
payload). Needs paho-mqtt (pip install paho-mqtt).
"""

import argparse
import csv
import sys
import time

COLUMNS = ("time", "topic", "topic_len", "payload_len", "qos", "packet_len")
REM_ONE_BYTE_MAX = 127


def packet_len(topic_len, payload_len, qos):
    """MQTT PUBLISH packet bytes for one message (see the module doc)."""
    rem = 2 + topic_len + (2 if qos > 0 else 0) + payload_len
    return 1 + (1 if rem <= REM_ONE_BYTE_MAX else 2) + rem


def size_row(topic, payload, qos, stamp):
    """One CSV row (a tuple in COLUMNS order)."""
    topic_len = len(topic.encode("utf-8"))
    return ("%.3f" % stamp, topic, topic_len, len(payload), qos,
            packet_len(topic_len, len(payload), qos))


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--topic", default="#")
    ap.add_argument("--out", default="-", help="CSV file (default stdout)")
    ap.add_argument("--seconds", type=float, default=0.0,
                    help="stop after this many seconds (0 = until Ctrl-C)")
    args = ap.parse_args(argv)

    try:
        import paho.mqtt.client as mqtt
    except ImportError:
        print("mqtt_size_log: needs paho-mqtt (pip install paho-mqtt)",
              file=sys.stderr)
        return 2

    out = sys.stdout if args.out == "-" else open(args.out, "w", newline="",
                                                  encoding="utf-8")
    writer = csv.writer(out)
    writer.writerow(COLUMNS)

    def on_connect(client, _userdata, _flags, *_rest):
        client.subscribe(args.topic, qos=1)

    def on_message(_client, _userdata, msg):
        writer.writerow(size_row(msg.topic, msg.payload, msg.qos,
                                 time.time()))
        out.flush()

    if hasattr(mqtt, "CallbackAPIVersion"):              # paho-mqtt >= 2.0
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    else:
        client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(args.host, args.port, keepalive=30)
    try:
        if args.seconds > 0.0:
            client.loop_start()
            time.sleep(args.seconds)
            client.loop_stop()
        else:
            client.loop_forever()
    except KeyboardInterrupt:
        pass
    finally:
        client.disconnect()
        if out is not sys.stdout:
            out.close()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
