#!/usr/bin/env python3
"""
car_link.py - tell the car where the broker is, and serve the dashboard.

Owner:  Buddy 1 (communications)

Usage (from the repo root, on the laptop that runs Mosquitto):

    py tools\\dashboard\\car_link.py            # beacon + dashboard on :8000
    py tools\\dashboard\\car_link.py --no-http  # beacon only
    py tools\\dashboard\\car_link.py --once     # print the addresses, exit

Every second it broadcasts the 9-byte UDP datagram "C28B 1883" to port
18830 from every IPv4 address of this laptop (limited broadcast
255.255.255.255 and the /24 directed broadcast). The car (MQTT image,
since 2026-11-06) takes the sender of the first such datagram as its
broker address, so a new hotspot or a new laptop IP needs no rebuild of
wifi_secrets.h. Without this script the car falls back to MQTT_BROKER_IP.

It also serves tools/dashboard/ over HTTP (default port 8000), so the
dashboard opens at http://localhost:8000 here or http://<laptop IP>:8000 on
a phone on the same hotspot, and it checks every 10 s that Mosquitto is
listening on 1883 (MQTT) and 9001 (WebSocket for the page).

Python 3 standard library only.
"""

import argparse
import functools
import http.server
import os
import socket
import sys
import threading
import time

BEACON_PORT = 18830
BEACON = b"C28B 1883"
BROKER_PORTS = (("MQTT for the car", 1883), ("WebSocket for the page", 9001))
DASH_DIR = os.path.dirname(os.path.abspath(__file__))


def local_ipv4():
    """This laptop's IPv4 addresses, without loopback; default route first."""
    addrs = []
    try:                                  # the address of the default route;
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.connect(("10.255.255.255", 1))   # UDP connect sends nothing
        addrs.append(probe.getsockname()[0])
        probe.close()
    except OSError:
        pass
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None,
                                       socket.AF_INET):
            addrs.append(info[4][0])
    except OSError:
        pass
    out = []
    for addr in addrs:
        if not addr.startswith("127.") and addr not in out:
            out.append(addr)
    return out


def directed_broadcast(addr):
    """x.y.z.255 for x.y.z.w (phone hotspots use /24 networks)."""
    parts = addr.split(".")
    return ".".join(parts[:3] + ["255"])


def send_beacons(addrs):
    """One beacon from each address; returns the addresses that worked."""
    ok = []
    for addr in addrs:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            sock.bind((addr, 0))          # leave through this interface
            for dest in ("255.255.255.255", directed_broadcast(addr)):
                sock.sendto(BEACON, (dest, BEACON_PORT))
            ok.append(addr)
        except OSError:
            pass
        finally:
            sock.close()
    return ok


def port_open(port):
    try:
        with socket.create_connection(("127.0.0.1", port), timeout=0.5):
            return True
    except OSError:
        return False


def check_broker():
    for name, port in BROKER_PORTS:
        if not port_open(port):
            print("car_link: WARNING nothing listens on port %d (%s): start "
                  "Mosquitto with C:\\mosq\\car.conf (README)" % (port, name),
                  flush=True)


def serve_dashboard(port):
    handler = functools.partial(http.server.SimpleHTTPRequestHandler,
                                directory=DASH_DIR)
    handler.log_message = lambda *args: None          # keep the console quiet
    try:
        httpd = http.server.ThreadingHTTPServer(("0.0.0.0", port), handler)
    except OSError as err:
        print("car_link: cannot serve the dashboard on port %d (%s); open "
              "tools\\dashboard\\index.html directly instead" % (port, err),
              flush=True)
        return None
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    return httpd


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--http", type=int, default=8000, metavar="PORT",
                    help="serve the dashboard on this port (default 8000)")
    ap.add_argument("--no-http", action="store_true",
                    help="beacon only, do not serve the dashboard")
    ap.add_argument("--once", action="store_true",
                    help="send one beacon, print the addresses and exit")
    args = ap.parse_args(argv)

    addrs = local_ipv4()
    if not addrs:
        print("car_link: no IPv4 address: is the laptop on the hotspot?",
              file=sys.stderr)
    for addr in addrs:
        print("car_link: laptop address %s (beacon to %s and "
              "255.255.255.255, UDP %d)" % (addr, directed_broadcast(addr),
                                           BEACON_PORT), flush=True)
    send_beacons(addrs)
    if args.once:
        return 0

    if not args.no_http and serve_dashboard(args.http) is not None:
        print("car_link: dashboard at http://localhost:%d  (phone: "
              "http://%s:%d)" % (args.http, addrs[0] if addrs else "<ip>",
                                 args.http), flush=True)
        print("car_link: in the page, broker = this laptop's address (or "
              "localhost), port 9001", flush=True)
    check_broker()
    print("car_link: broadcasting the broker beacon every second; Ctrl+C "
          "to stop", flush=True)

    last_addrs = addrs
    next_check = time.monotonic() + 10.0
    try:
        while True:
            time.sleep(1.0)
            if time.monotonic() >= next_check:
                next_check = time.monotonic() + 10.0
                addrs = local_ipv4()
                if addrs != last_addrs:
                    print("car_link: laptop addresses now %s"
                          % (", ".join(addrs) or "none"), flush=True)
                    last_addrs = addrs
                check_broker()
            send_beacons(addrs)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
