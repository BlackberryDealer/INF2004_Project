"""Summarise one Pico boot's priority checks. Usage: python review-bench.py CAPTURE.txt"""
import argparse
import re
from pathlib import Path


def review(text):
    boots = list(re.finditer(r"^\s*(single-core build|dual-core \(SMP\) build)\s*$", text, re.M))
    if not boots:
        print("INCOMPLETE: no firmware core-count banner in capture")
        return 2
    boot = boots[-1]
    print(f"Reviewing boot {len(boots)}: {boot.group(1)}")
    body = text[boot.end():]
    begin = "[TEST] === test run start ==="
    end = "[TEST] === test run end ==="
    pre, start, after = body.partition(begin)
    checks, finish, _ = after.partition(end)
    incomplete = boot.group(1) != "single-core build" or not (start and finish)
    if incomplete:
        print("Single-core bench evidence requires the single-core banner and both test markers.")
    failures = re.findall(r"^\[TEST\] (\S+) FAIL\s+(.+)$", body, re.M)
    for test_id, detail in failures:
        print(f"FAIL {test_id}: {detail}")
    failed = bool(failures)
    for test_id, expected in (("S1-U03", 4), ("S5-U06", 3)):
        found = re.findall(rf"^\[TEST\] {test_id} (PASS|FAIL)\s+(.+)$", checks, re.M)
        verdicts = [v for v, _ in found]
        print(f"{test_id}: {verdicts.count('PASS')} passed, {verdicts.count('FAIL')} failed, expected {expected} assertions")
        incomplete |= len(found) != expected or len({detail for _, detail in found}) != expected
    line = [tuple(map(int, values)) for values in re.findall(
        r"\[S3-I01\] line 1000-cyc: dt (\d+)\.\.(\d+) us, wcet (\d+) us", body)]
    imu = [int(value) for value in re.findall(r"\[S4-I09\] imu_read 1000-cyc wcet (\d+) us", body)]
    if line:
        low, high, wcet = min(v[0] for v in line), max(v[1] for v in line), max(v[2] for v in line)
        ok = low >= 9000 and high <= 11000 and wcet <= 300
        print(f"S3-I01 {'PASS' if ok else 'FAIL'} over {len(line)} windows: period {low}..{high} us, max WCET {wcet} us")
        failed |= not ok
    if imu:
        maximum = max(imu)
        if maximum > 800:
            print(f"S4-I09 FAIL: {maximum} us exceeds every workload budget (550/750/800 us)")
            failed = True
        elif maximum > 550:
            print(f"S4-I09 REVIEW: peak {maximum} us. Match the window to the 550/750/800 us workload budget.")
            incomplete = True
        else:
            print(f"S4-I09: all {len(imu)} captured windows within 550 us; peak {maximum} us. Record workloads exercised.")
    if not line or not imu:
        print("INCOMPLETE: missing " + ", ".join(label for label, data in (("S3-I01", line), ("S4-I09", imu)) if not data))
        incomplete = True
    if failed:
        return 1
    return 2 if incomplete else 0


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    args = parser.parse_args()
    raise SystemExit(review(args.capture.read_text(encoding="utf-8-sig", errors="replace")))
