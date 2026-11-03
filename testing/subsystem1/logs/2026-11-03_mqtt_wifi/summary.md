# MQTT telemetry diagnostics, 2026-11-03

Car `csc28-car` (Pico W, 10.14.95.206) on phone hotspot `Hpi~`, Mosquitto 2.1.2
broker on the laptop (10.14.95.64:1883, `listener 1883 0.0.0.0`,
`allow_anonymous true`). The car was idle in READY (`SYS=3`) for every run;
no command was sent to `c28/c` during any capture.

Each run was captured for 120 s at the same time:

- `decode.txt`: `tools/telem_decode.py --host 127.0.0.1`, every line prefixed
  with the laptop's arrival time (HH:MM:SS.mmm, UTC+8)
- `ping.txt`: `ping -n 120 10.14.95.206`
- `console.txt`: car USB console on COM5 at 115200, arrival-stamped. Lines
  stamped at the capture start are the backlog already buffered on the car
- `broker.txt`: the broker's verbose log (stderr) for the same window, each
  line as `HH:MM:SS (unix) message`

## Builds and capture windows (local time, UTC+8)

| Run | Firmware | Capture window |
|---|---|---|
| 1_before | `PicoCar_mqtt_013ca60.uf2` (main @ 013ca60) | 19:23:31 to 19:25:31 |
| 2_pmoff | `PicoCar_mqtt_pmoff.uf2`: Wi-Fi power save off on link up, lwIP MQTT 24 request slots, 1 KB output ring | 19:36:03 to 19:38:03 |
| 3_nodelay | `PicoCar_mqtt_nodelay.uf2`: as pmoff plus TCP Nagle off on the MQTT connection | 20:01:29 to 20:03:29 |

## Results

| Metric | 1_before | 2_pmoff | 3_nodelay |
|---|---|---|---|
| Ping min / median / p90 / max | 6 / 43 / 143 / 510 ms | 8 / 40 / 95 / 187 ms | 4 / 38 / 88 / 114 ms |
| Ping loss | 0 / 120 | 4 / 120 | 0 / 120 |
| STATE rate | 4.93 /s | 4.74 /s | 5.01 /s (5 in every second) |
| Inter-STATE median / max | 192 / 1694 ms | 180 / 7800 ms | 182 / 324 ms |
| STATE gaps > 400 ms | 14 | 17 | 0 |
| Heartbeats lost | 3 (UPTIME_S 453, 464, 465) | 6 (UPTIME_S 95 to 100) | 0 (120 of 120) |
| DROPS in window | unknown (saturated at 255) | 110 to 123 (+13) | 37 to 37 (+0) |
| LAT_MAX_US first / last | 1991 / 2263 us | 1519 / 1815 us | 1555 / 1684 us |
| WDT feed gap avg / max | 11942 / 15928 us | 11374 / 12168 us | 11765 / 12559 us |
| Broker keepalive disconnects | 2 (19:19:44, 19:20:06, just before the window) | 1 (19:37:09) | 0 |
| STATE per second at broker | 1 to 8 /s, 5 /s in 100 of 123 s | 1 to 14 /s, 13 s at 10 /s or more | 5 /s in all 123 s |

Baseline for the hotspot itself, laptop to phone gateway 10.14.95.19, 30 pings
during run 2: median 14 ms, p90 71 ms, max 169 ms, 0 lost.

### Gaps > 400 ms

- 1_before: 19:23:31.539 (1368 ms), 19:23:33.399 (639), 19:23:35.561 (716),
  19:23:44.179 (1694), 19:23:45.873 (406), 19:23:47.561 (649),
  19:23:54.592 (477), 19:23:59.973 (416), 19:24:19.250 (587),
  19:24:44.993 (459), 19:24:49.386 (585), 19:24:57.773 (580),
  19:25:03.017 (445), 19:25:19.770 (506)
- 2_pmoff: 19:36:10.864 (1363 ms), 19:36:14.650 (1752), 19:36:17.460 (1621),
  19:36:20.818 (1857), 19:36:42.067 (1366), 19:36:46.018 (1579),
  19:36:57.658 (1924), 19:37:02.129 (7800), 19:37:15.874 (1423),
  19:37:17.838 (2108), 19:37:19.954 (1436), 19:37:23.066 (1989),
  19:37:25.239 (1350), 19:37:27.942 (1434), 19:37:48.257 (1753),
  19:37:52.516 (1632), 19:38:00.859 (2137)
- 3_nodelay: none

## Notes

- DROPS in the heartbeat is K7 + K11 + publish drops since boot, saturating
  at 255 (`telem_drops()`). The DROPS value at the start of runs 2 and 3 comes
  from boot and from earlier link loss, not from the capture window.
- Run 2: in the 1.4 to 2.1 s STATE stalls, the car kept answering ping in
  17 to 68 ms, so the radio link was up and only the MQTT TCP stream held
  back. That fits Nagle holding small segments behind unacknowledged ones on
  a slow link, which run 3 removes.
- Run 2 console: the first join failed (`authentication rejected (-3)`) and
  the second one succeeded. In the 7.8 s outage the car logged
  `mqtt: connection status 257` and `FAULT bit 4 WIFI_LOSS (info only)`, and
  reconnected at 19:37:09.794.
- Run 3 console boot backlog confirms the change is active:
  `wifi: link up, ip=10.14.95.206`, `wifi: power save off (0)`,
  `mqtt: connected to 10.14.95.64`. The only FAULT line is
  `FAULT bit 6 RANGE_CLAMP (info only)`.
- Still open: the WDT feed gap is about 12 ms of the 30 ms limit, with the
  sensor debug prints (about 10 lines/s on the console) still enabled.
