"""Capture on-car MQTT evidence with mosquitto_sub and the project decoder."""
import argparse
import csv
import datetime as dt
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tools"))
import telem_decode


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tester", required=True)
    parser.add_argument("--firmware", required=True, help="commit of the flashed UF2")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=1883)
    parser.add_argument("--count", type=int, help="finish after this many messages")
    parser.add_argument("--output", type=Path, help="new CSV path (default: testing/system/logs)")
    args = parser.parse_args()
    if args.count is not None and args.count < 1:
        parser.error("--count must be positive")
    sub = shutil.which("mosquitto_sub")
    if sub is None:
        fallback = Path(os.environ.get("ProgramFiles", "C:/Program Files")) / "Mosquitto/mosquitto_sub.exe"
        if not fallback.is_file():
            parser.error("mosquitto_sub was not found. Use the Mosquitto client installation.")
        sub = str(fallback)
    stamp = dt.datetime.now().strftime("%Y-%m-%d_%H%M%S_%f")
    path = args.output or ROOT / "testing/system/logs" / f"{stamp}_mqtt.csv"
    names, units = telem_decode.read_app_types()
    command = [sub, "-h", args.host, "-p", str(args.port), "-q", "1", "-t", "c28/#",
               "-F", "%I\t%t\t%r\t%x"]
    if args.count is not None:
        command.extend(["-C", str(args.count)])
    count = 0
    with path.open("x", encoding="utf-8", newline="") as output:
        writer = csv.writer(output)
        writer.writerow(["received_at", "topic", "retained", "payload_hex", "decoded",
                         "tester", "firmware_commit"])
        output.flush()
        print(f"Saving {path}\nWaiting for c28/# at {args.host}:{args.port}. Ctrl+C finishes.", flush=True)
        with subprocess.Popen(command, stdout=subprocess.PIPE, text=True, encoding="utf-8") as client:
            try:
                for line in client.stdout:
                    received, topic, retained, payload_hex = line.rstrip("\r\n").split("\t", 3)
                    decoded = telem_decode.line_for(topic, bytes.fromhex(payload_hex), names, units)
                    writer.writerow([received, topic, retained, payload_hex, decoded,
                                     args.tester, args.firmware])
                    output.flush()
                    count += 1
                    if count == 1:
                        print(f"Receiving: {decoded}", flush=True)
            except KeyboardInterrupt:
                if client.poll() is None:
                    client.terminate()
            except BaseException:
                if client.poll() is None:
                    client.terminate()
                raise
            else:
                if client.wait() != 0:
                    return 1
    print(f"Saved {count} messages to {path}")
    return 0 if count else 1


if __name__ == "__main__":
    sys.exit(main())
