# PicoCar Live: telemetry and command dashboard

A web page for testing the car: live state, wheel speeds, line error, pitch,
faults, link health and the event log, plus START / STOP / RESET / SUMMARY
buttons that follow the mission statechart. Since 2026-11-06 it also shows
the car's **raw sensors** and its **console lines** over Wi-Fi, so the car
can be debugged without the USB cable (see "Debugging over Wi-Fi"). It decodes exactly like
`tools/telem_decode.py` and sends exactly the bytes of `tools/telem_cmd.py`
(host test S1-U44 checks both).

```
 car (Pico W) --MQTT 1883--> Mosquitto on the laptop <--WebSocket 9001-- browser
```

A browser cannot open a raw MQTT (TCP) connection, so Mosquitto gets a
second listener that speaks MQTT over WebSocket. The page carries its own
small MQTT client (`mqtt_ws.js`), so it needs no internet, no npm and no
install: just open `index.html`.

## Files

| File | What it is |
|---|---|
| `index.html` | the page (open this) |
| `dashboard.js` | page logic: state card, command buttons, tiles, charts, log |
| `decode.js` | port of `telem_decode.py` / `telem_cmd.py` (decoder, command bytes) |
| `mqtt_ws.js` | MQTT 3.1.1 over WebSocket: connect, subscribe, QoS 0/1, keep-alive |
| `telem_tables.js` | **generated** by `gen_tables.py`; do not edit |
| `gen_tables.py` | writes `telem_tables.js` from `telem_fields.py`, `telem_decode.py`, `telem_cmd.py`, `app_types.h`, `cfg_telem.h` |
| `car_link.py` | run on the laptop: UDP beacon that tells the car the broker's address, serves this page on :8000, checks Mosquitto |
| `fake_car.py` | pretend car on the broker, for trying the page without hardware |
| `test_dashboard.py`, `test_decode.js` | host test S1-U44 |

## Setup on Windows (once)

1. **Add the WebSocket listener to the broker config.** Open
   `C:\mosq\car.conf` in Notepad and make it look like this (keep any other
   lines you already have). `protocol` applies to the `listener` line just
   above it, so keep this order:

   ```
   listener 1883 0.0.0.0
   listener 9001 0.0.0.0
   protocol websockets
   allow_anonymous true
   ```

2. **Restart Mosquitto** with that config. In PowerShell (stop the old one
   first if it is still running):

   ```powershell
   Stop-Process -Name mosquitto -ErrorAction SilentlyContinue
   & "C:\Program Files\mosquitto\mosquitto.exe" -c C:\mosq\car.conf -v
   ```

   The log must show both lines:

   ```
   Opening ipv4 listen socket on port 1883.
   Opening websockets listen socket on port 9001.
   ```

   If it says websockets are not supported, the Mosquitto build has no
   WebSocket support; install the current Windows build from
   mosquitto.org (it includes it).

3. **(Only for phones or other laptops)** allow port 9001 through the
   firewall. In an **administrator** PowerShell:

   ```powershell
   New-NetFirewallRule -DisplayName "Mosquitto WebSocket 9001" -Direction Inbound -Protocol TCP -LocalPort 9001 -Action Allow
   ```

   The page on the laptop itself (`localhost`) does not need this.

   **The car needs inbound TCP 1883.** Windows treats a newly joined
   hotspot as a *Public* network, where a rule made for *Private* does not
   apply. Allow it on every profile once (administrator PowerShell):

   ```powershell
   New-NetFirewallRule -DisplayName "Mosquitto MQTT 1883" -Direction Inbound -Protocol TCP -LocalPort 1883 -Action Allow -Profile Any
   ```

   (or set the hotspot to *Private* in Settings > Network). Symptom when
   blocked: the car prints `mqtt: connection status 256` / `retry in ...`
   although `car_link.py` runs.

4. **(Once)** allow the beacon and the page through the firewall if
   Windows asks the first time `car_link.py` runs (Python, private
   network). The beacon is outgoing UDP 18830; the page is TCP 8000.

## Every session

1. Connect the laptop to the hotspot the car joins (`WIFI_SSID` in
   `config/wifi_secrets.h`).
2. Start Mosquitto (step 2 above).
3. Start the link helper and leave it running:

   ```powershell
   py tools\dashboard\car_link.py
   ```

   It prints the laptop's address, broadcasts the broker beacon every
   second and serves the page. **The car takes the broker address from this
   beacon**, so a new hotspot or a new laptop IP needs no rebuild; without
   it the car falls back to `MQTT_BROKER_IP` in `wifi_secrets.h`.
4. Power the car (image built with `NET=lwip MQTT=1`). Its console shows
   `wifi: link up, ip=...`, `mqtt: broker beacon from <laptop IP>` and
   `mqtt: connected to <laptop IP> port 1883`.
5. Open `http://localhost:8000` (or double-click `index.html`). Broker
   address `localhost`, port `9001`, press **Connect**. The page
   remembers them and reconnects by itself if the broker restarts.
4. The three pills show where a problem is: **Broker** (page to Mosquitto),
   **Car** (ONLINE / OFFLINE on `c28/a`, OFFLINE is the car's last will) and
   **Telemetry** (STATE messages at 5 Hz; "stale" after 1.5 s without one).

### Opening it on a phone

Serve the folder from the laptop and open it by the laptop's IP. The page
then uses that IP as the broker address automatically:

`car_link.py` already serves the folder on port 8000 (without it:
`py -m http.server 8000 --directory tools\dashboard`). On the phone (same
hotspot) open `http://<laptop IP>:8000`, the address `car_link.py` prints.
Allow TCP 8000 through the firewall as in step 3, plus 9001.

## Using it

**Car state** shows the system state in the car LED's colour (blinking
like the LED), the run sub-state, motor and line status, run time and
distance, and one line saying what to do next. The map under it shows the
three paths through the statechart and which command moves the car on:

```
Start-up   BOOT -> CAL_IMU -> CAL_IR -> READY
Run cycle  READY --START--> RUN --STOP--> FINISHED --RESET--> READY
Stops      LINE_LOST_STOP --RESET--> READY      FAULT --RESET--> BOOT
```

**Commands** are published to `c28/c` at QoS 1, the same bytes as
`telem_cmd.py`: START `81`, STOP `89`, RESET `91`, SUMMARY `A1`, MAP `99 xx`.

- A button is only enabled when the car will act on it: START in READY,
  RESET in FINISHED / LINE_LOST_STOP / FAULT. **STOP and SUMMARY are always
  enabled.** The button that moves the car on is outlined.
- After STOP the car is in FINISHED; **press RESET before the next START**
  (same as the buttons on the car).
- The feedback box follows each command: delivered to the broker, then
  "✓ Car went READY → RUN in 1.2 s", or "✗ No effect after 3.5 s" with the
  reason (wrong state, car offline, no telemetry).
- Keys: **Space** STOP, **S** START, **R** RESET, **Q** SUMMARY.
- "Unlock all buttons" enables every button whatever the state (when
  telemetry is missing but the car still listens).
- "Remap a barcode" sends MAP; the car answers with a MAP event.

**Live readings** are the latest STATE: wheel speeds in mm/s, line error
(‰, + = line to the left), odometer, line sensor, motor state, pitch, turn
rate, motion class, hump peak and encoder counts. **Faults** light red when
they stop the car (cleared by RESET) and amber when informational only
(WIFI_LOSS, RANGE_CLAMP).

**Link health** counts STATE gaps over 400 ms (NFR8a), shows the car's own
DROPS counter and the radio latency max from the heartbeat against the
NFR17 limit of 1000 µs.

**Trends** plot the last 30 s / 60 s / 2 min of wheel speeds, line error,
pitch and turn rate; hover a chart for exact values.

**Run summary** shows the SUMMARY the car sends when a run ends (distance,
time, average speed, end reason, highest hump, faults) and the latest
SNAPSHOT reply. **Obstacle profile** draws the last PROFILE (bearing,
width, nearest point, gaps, chosen side).

**Sensors (raw)** is the SENSORS message on `c28/d` (5 Hz, 12 bytes): raw
IR ADC left / right (with a rough white / edge / black hint), ultrasonic
distance (slow and stop zones marked; "none" = 2044 mm, nothing in range),
obstacle scanner state, barcode DO level and edge count, the calibrated
accelerometer (X / Y / Z in mg) and the watchdog's longest feed gap (limit
30 ms, warning above 15 ms). **Export sensors CSV** saves every sample of
the session (up to about an hour) with the car state, for a test record.
Four more trend charts plot IR raw, ultrasonic distance, accelerometer and
feed gap.

**Car console** shows every console line of the car (`app_log()`: the same
text as the USB serial, e.g. `OBSTACLE STOP`, `SCAN nearest`, `LINE LOST`,
`REJOIN`), sent as LOG messages on `c28/l`. Faults are red, obstacle / line /
barcode lines amber, state changes blue. "hide periodic status lines" hides
the `US dist` / `IR raw` / `ENC pos` / `BAR` / `WDT` prints; **Export .txt**
saves the whole console. If the car's queue overflowed it says so (`(N
console lines dropped: queue full)`).

**Message log** lists every event, command and result with its time;
hovering a row shows the exact `telem_decode.py` line. Tick "include STATE
/ HEARTBEAT" for everything, "show bytes" for the raw payloads, and
**Export CSV** to save the whole session (up to 5000 messages) for a test
record.

## Debugging over Wi-Fi (2026-11-06)

| Topic | Payload | Rate | What |
|---|---|---|---|
| `c28/d` | SENSORS, 12 B binary (`MQ_SENS` in `tools/layout.csv`) | 5 Hz, QoS 0 | raw IR, ultrasonic, scanner, barcode, accelerometer, feed gap |
| `c28/l` | LOG: header byte `0x49` + one console line (ASCII) | as printed, QoS 0 | every `app_log()` line |

- Both are bring-up diagnostics like the console prints. They are on in
  images built with `MQTT=1` while `TELEM_DEBUG_MQTT` is 1
  (`app_program/subsystem1/cfg_telem.h`); set it to 0 for the Week 13
  message-size evidence, which then shows only the original payloads.
- Console lines never block the code that prints them: `app_log()` queues
  the format and arguments (K12, 48 lines) and the radio task formats and
  publishes them, at most 2 per 50 ms, and only while the MQTT output
  buffer is at most a quarter full, so telemetry always goes first.
- `py tools/telem_decode.py` prints both (SENSORS fields, LOG text).
- The broker beacon is the UDP datagram `C28B 1883` to port 18830
  (broadcast). The car connects to its sender; while it is connected a new
  beacon is only noted, and it is used at the next reconnect.

## Without the car

```powershell
py -m pip install paho-mqtt        # once
py tools\dashboard\fake_car.py     # broker on this laptop
```

The pretend car boots, calibrates, waits in READY and obeys the page's
commands like the statechart does (it also sends SENSORS and console
lines); a run passes a hump, a barcode and an
obstacle and ends at the marker after 30 s. `--fault-at 5` raises a
NO_ECHO fault 5 s into a run to try the FAULT → RESET path. Its numbers are
plausible, not measured. Do not run it while the real car is on the same
broker: both would publish on `c28/...`.

## After a layout or app_types.h change

```bash
python3 tools/dashboard/gen_tables.py                  # rewrite telem_tables.js
make -C testing/subsystem1/host dashboard              # S1-U44
```

S1-U44 fails while `telem_tables.js` is stale. The JS half needs Node.js;
without it those checks print SKIP.
