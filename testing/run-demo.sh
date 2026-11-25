#!/usr/bin/env bash
# Run from any directory with Bash, GNU Make and the project's toolchain.
set -euo pipefail
cd "$(dirname "$0")/.."
case "${1:-}" in
  host|mqtt) [[ -n "${2:-}" ]] || { echo "Supply the tester's name as argument 2." >&2; exit 2; } ;;
  bench|course) ;;
  *) echo "Usage: bash testing/run-demo.sh host TESTER | bench | course | mqtt TESTER [BROKER]" >&2; exit 2 ;;
esac
mode=$1
commit=$(git rev-parse HEAD)
stamp=$(date +%Y-%m-%d_%H%M%S)
if [[ $mode == bench || $mode == course ]]; then
  test_flag=0
  suffix=
  if [[ $mode == bench ]]; then test_flag=1; suffix=_test; fi
  image="build_make/mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt${suffix}.uf2"
  make -C build_make -j8 SMP=0 CONSOLE=usb_cdc WIFI=cyw43 NET=lwip MQTT=1 TEST="$test_flag" DEBUG_MQTT=0 PROBE=0
  [[ -s "$image" ]]
  {
    echo "commit=$commit"
    echo "profile=$mode"
    if git diff --quiet HEAD -- app_program config device include kernel lib build_make; then
      echo "tracked_firmware_edits=no"
    else
      echo "tracked_firmware_edits=yes"
    fi
    echo "sha256=$(sha256sum "$image" | cut -d ' ' -f 1)"
  } > "$image.info"
  echo "$image"
  exit 0
fi

dest=testing/system/logs
if [[ $mode == host ]]; then dest=testing/subsystem1/logs; fi
log=$(mktemp --suffix=.txt "$dest/${stamp}_${mode}_${commit:0:7}_XXXXXX")
echo "Saving $log"
{
  echo "Date: $(date -Iseconds)"
  echo "Tester: $2"
  echo "Checkout commit: $commit"
  echo "Mode: $mode"
  git status --short -- app_program tools testing/host 'testing/subsystem*/host' lib
  if [[ $mode == host ]]; then
    command -v node >/dev/null || { echo "Node is required for the dashboard checks."; exit 2; }
    echo "Command: make -B -C testing/host"
    make -B -C testing/host
  else
    echo "Broker: ${3:-localhost}; record the flashed course commit in the results row."
    python3 -u tools/telem_decode.py --host "${3:-localhost}"
  fi
} 2>&1 | tee "$log"

