# Run the PicoCar tests

**Start with the bench UF2, then switch to the course UF2.** Both use single-core USB CDC, Wi-Fi and MQTT. The local images in `build_make/` use the team's existing Wi-Fi configuration. Git contains the rebuildable sources. Keep locally configured UF2 files off GitHub because the radio configuration is compiled in.

## 1. Bench · USB cable and Windows PowerShell

Open PowerShell in this checkout. List COM ports, then start logging with the Pico's port number and your name. Close other serial monitors first. If the board is currently in BOOTSEL, supply the COM number it uses in normal operation. The script waits for it.

```powershell
[System.IO.Ports.SerialPort]::GetPortNames()
powershell -ExecutionPolicy Bypass -File .\testing\Capture-Pico.ps1 -Image bench -Port COM5 -Tester Dylan
```

Hold BOOTSEL while plugging in the Pico. Copy **`build_make/mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt_test.uf2`** to RPI-RP2. The Pico reboots. Confirm **single-core build** in the saved console text and both `[TEST] === test run start/end ===` markers. Leave capture running in READY for at least 30 seconds after calibration for the 1000-cycle windows. Ctrl+C closes the file. The script shows its dated path under `testing/system/logs/`. If the USB boot banner was missed, keep the capture open and reset. A full capture needs the banner, both markers and the timed measurements.

| Check first | What to record |
|---|---|
| S1-U03 | All queue assertions, including eight full-sized STATE payloads. |
| S5-U06 | Before/after latency counter and all three verdicts. |
| S3-I01 | Period range (9–11 ms) and maximum cycle time (≤300 µs). |
| S4-I09 | Worst read time and applicable 550/750/800 µs workload budget. Identify which limit applies; uncertain workload is inconclusive. |

For a quick readout, run the following on this Windows laptop. It selects the newest bench capture. On another machine, use its Python 3 interpreter. It reviews the last boot and reports the two unit verdicts, the line timing range and the maximum IMU read duration. Exit 1 means a failure was observed and exit 2 means the capture needs more evidence. Keep the raw capture.

```powershell
$capture = Get-ChildItem .\testing\system\logs\*_bench_*.txt | Sort-Object LastWriteTime -Descending | Select-Object -First 1
& "C:\msys64\ucrt64\bin\python3.exe" .\testing\review-bench.py $capture.FullName
```

Save a scope capture where a firmware diagnostic cannot establish the timing. Keep all failing lines. For a missing banner, incomplete test or missing physical measurement, record the observed portion and mark the rest pending.

## 2. Course · normal firmware and broker capture

Flash **`build_make/mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt.uf2`**. Start your usual broker and [dashboard](../tools/dashboard/README.md). In the team's Bash build shell, start the capture below (Python needs `paho-mqtt` for this command). It timestamps incoming messages through the project decoder. Use the broker address in place of `localhost` if it runs on another machine. The dashboard guide includes the discovery beacon setup.

```bash
bash testing/run-demo.sh mqtt Dylan localhost
```

Copy `testing/system/track_trials.csv` into `testing/system/logs/` under a dated filename. Enter the tested commit and tester, the measured inputs, actual observations and evidence filename/time. Blank trials have no outcome. Set the four assigned barcode actions in READY and wait for each MAP acknowledgement. Run SYS-02 through SYS-05 on the course, then SYS-06 and SYS-07 as separate bench/track runs. A second observer can measure clearance and film the actual turn or obstacle pass. The worksheet covers repeat trials; the [system cases](system/TEST_LOG.md) list the other observations. Use USB capture or an independent observation for the interval when the hotspot is off.

## 3. Save and submit

Append one row per completed case or explicitly scoped partial result to the subsystem, [integration](integration/TEST_LOG.md) or [system](system/TEST_LOG.md) log. Include date, ID, flashed commit, tester, verdict, actual values and a relative link to the raw evidence. Have both owners sign the integration result. A short unlisted clip can serve multiple cases; enter it in [the video index](../videos/README.md) and link it with a timestamp wherever used.

```bash
git status --short
git add -- testing videos/README.md
git diff --cached --stat
git commit -m "Record single-core bench and course results"
git push -u origin codex/demo-submission
```

Open the branch's pull request into `main` and verify the logs open on the submitted revision. Use the push command after the team is ready to publish.

## Rebuild or repeat on another laptop

In the configured Bash build shell with GNU Make, host GCC, Python 3 and Node, from this checkout:

```bash
bash testing/run-demo.sh host Dylan
bash testing/run-demo.sh bench
bash testing/run-demo.sh course
```

Firmware builds also require the ARM compiler, `PICO_SDK_PATH` (with TinyUSB, CYW43 and lwIP), and this laptop's ignored `config/wifi_secrets.h`. The build script puts a commit and checksum record next to each UF2. Run bench and course builds sequentially because the Makefile shares objects. A host run saves the complete console output under `testing/subsystem1/logs/` and returns the suite's exit status. Use the header's commit when recording the result.
