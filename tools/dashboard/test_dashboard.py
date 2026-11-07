#!/usr/bin/env python3
"""
test_dashboard.py - host test S1-U44: the dashboard decodes and encodes
exactly like tools/telem_decode.py and tools/telem_cmd.py.

Owner:  Buddy 1 (communications)

Usage (from the repo root; the golden vectors come from the S1-U10 host
test, so run "make -C testing/subsystem1/host telem_bin" first):

    python3 tools/dashboard/test_dashboard.py testing/subsystem1/host/golden

Checks:
  1. telem_tables.js is up to date with its sources (gen_tables --check).
  2. decode.js reproduces every golden *.csv (same as S1-U10).
  3. decode.js gives the S1-U43 command bytes and S1-U42 snapshot text.
  4. decode.js prints the same line as telem_decode.line_for() for every
     valid payload size of every type plus random bytes, lengths and
     versions (fixed seed): 4000+ payloads, every one must match.

Needs Node.js (any version >= 12) for 2-4. Without it, 2-4 print SKIP and
the test still fails if 1 fails. Exit 1 on any FAIL.
"""

import os
import random
import shutil
import subprocess
import sys

DASH_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(DASH_DIR))
sys.path.insert(0, DASH_DIR)

import gen_tables    # noqa: E402
import telem_decode  # noqa: E402

TEST_JS = os.path.join(DASH_DIR, "test_decode.js")
SEED = 2004
RANDOM_PER_TYPE = 250


def fuzz_cases():
    """(topic, payload) list: every type at its right size plus noise."""
    rng = random.Random(SEED)
    cases = []
    for mq_type, (_name, group) in sorted(telem_decode.TYPE_GROUP.items()):
        if group == "TEXT":                      # LOG: any length
            size = 1 + rng.randrange(0, 60)
        else:
            size = (1 if group is None
                    else 1 + telem_decode.payload_bytes(group))
        head = telem_decode.MQ_VERSION | (mq_type << 3)
        for _ in range(RANDOM_PER_TYPE):
            if group == "TEXT":
                size = 1 + rng.randrange(0, 60)
            body = bytes(rng.randrange(256) for _ in range(size - 1))
            cases.append(("c28/s", bytes([head]) + body))
        cases.append(("c28/e", bytes([head]) + b"\xff" * (size - 1)))
        cases.append(("c28/e", bytes([head]) + b"\x00" * (size - 1)))
        cases.append(("c28/e", bytes([head]) + b"\x00" * size))  # too long
    for _ in range(500):                         # junk: any length, version
        n = rng.randrange(0, 10)
        cases.append(("c28/x", bytes(rng.randrange(256) for _ in range(n))))
    # every MQ_EVT word with each event type, random values
    head = telem_decode.MQ_VERSION | (1 << 3)
    for evt_type in range(32):
        for _ in range(20):
            word = rng.randrange(2048) | (evt_type << 11)
            cases.append(("c28/e", bytes([head, word & 0xFF, word >> 8])))
    return cases


def main(argv):
    fails = 0

    def check(ok, text):
        nonlocal fails
        print("[TEST] S1-U44 %s  %s" % ("PASS" if ok else "FAIL", text))
        fails += 0 if ok else 1

    expected = gen_tables.render(gen_tables.build())
    with open(gen_tables.OUT_JS, encoding="utf-8", newline="") as fh:
        check(fh.read() == expected,
              "telem_tables.js is up to date (else run "
              "tools/dashboard/gen_tables.py)")

    node = shutil.which("node")
    if node is None:
        print("[TEST] S1-U44 SKIP  JS checks: Node.js not installed")
        print("[TEST] dashboard SUMMARY %d failed (JS skipped)" % fails)
        return 1 if fails else 0

    golden_dir = argv[0] if argv else os.path.join(
        os.path.dirname(os.path.dirname(DASH_DIR)),
        "testing", "subsystem1", "host", "golden")
    for args in (["golden", golden_dir], ["vectors"]):
        res = subprocess.run([node, TEST_JS] + args, capture_output=True,
                             text=True)
        sys.stdout.write(res.stdout)
        sys.stderr.write(res.stderr)
        if res.returncode != 0:
            fails += 1

    cases = fuzz_cases()
    stdin = "".join("%s %s\n" % (t, p.hex()) for t, p in cases)
    res = subprocess.run([node, TEST_JS, "lines"], input=stdin,
                         capture_output=True, text=True)
    got = res.stdout.split("\n")[:-1]
    evt_names, um_per_count = telem_decode.read_app_types()
    want = [telem_decode.line_for(t, p, evt_names, um_per_count)
            for t, p in cases]
    diffs = [(w, g) for w, g in zip(want, got) if w != g]
    check(res.returncode == 0 and len(got) == len(want) and not diffs,
          "JS line == Python line_for() for %d payloads (%d differ)"
          % (len(cases), len(diffs) + abs(len(got) - len(want))))
    for w, g in diffs[:5]:
        print("        python: %s\n        js:     %s" % (w, g))

    print("[TEST] dashboard SUMMARY %d failed" % fails)
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
