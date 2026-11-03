# PicoCar Live: telemetry and command dashboard

A web page for testing the car: live state, wheel speeds, line error, pitch,
faults, link health and the event log, plus START / STOP / RESET / SUMMARY
buttons that follow the mission statechart. It decodes exactly like
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

## Every session

1. Start Mosquitto (step 2 above) and power the car (built with
   `NET=lwip MQTT=1`, as for `telem_decode.py`).
2. Double-click `tools\dashboard\index.html` (Chrome or Edge).
3. Broker address `localhost`, port `9001`, press **Connect**. The page
   remembers them and reconnects by itself if the broker restarts.
4. The three pills show where a problem is: **Broker** (page to Mosquitto),
   **Car** (ONLINE / OFFLINE on `c28/a`, OFFLINE is the car's last will) and
   **Telemetry** (STATE messages at 5 Hz; "stale" after 1.5 s without one).

### Opening it on a phone

Serve the folder from the laptop and open it by the laptop's IP. The page
then uses that IP as the broker address automatically:

```powershell
py -m http.server 8000 --directory tools\dashboard
```

On the phone (same hotspot): `http://10.14.95.64:8000`. Allow TCP 8000
through the firewall as in step 3, plus 9001.

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

**Message log** lists every event, command and result with its time;
hovering a row shows the exact `telem_decode.py` line. Tick "include STATE
/ HEARTBEAT" for everything, "show bytes" for the raw payloads, and
**Export CSV** to save the whole session (up to 5000 messages) for a test
record.

## Without the car

```powershell
py -m pip install paho-mqtt        # once
py tools\dashboard\fake_car.py     # broker on this laptop
```

The pretend car boots, calibrates, waits in READY and obeys the page's
commands like the statechart does; a run passes a hump, a barcode and an
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
