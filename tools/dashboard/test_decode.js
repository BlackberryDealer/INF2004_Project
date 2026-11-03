#!/usr/bin/env node
/*
 * test_decode.js - Node side of host test S1-U44 (run it through
 * tools/dashboard/test_dashboard.py, which also does the Python side).
 *
 *   node test_decode.js golden DIR   decode DIR/*.bin, compare with *.csv
 *   node test_decode.js vectors      command bytes and the S1-U42 snapshot
 *   node test_decode.js lines        stdin: "topic hex" per line;
 *                                    stdout: decode.lineFor() per line
 */
"use strict";

var fs = require("fs");
var path = require("path");
var D = require("./decode.js");

var fails = 0;

function check(ok, text) {
    console.log("[TEST] S1-U44 " + (ok ? "PASS" : "FAIL") + "  " + text);
    if (!ok) {
        fails += 1;
    }
}

function golden(dir) {
    var bins = fs.readdirSync(dir).filter(function (n) {
        return n.endsWith(".bin");
    }).sort();
    check(bins.length > 0, "golden vectors found in " + dir);
    bins.forEach(function (bin) {
        var name = bin.slice(0, -4);
        var data = fs.readFileSync(path.join(dir, bin));
        var rows = fs.readFileSync(path.join(dir, name + ".csv"), "utf8")
            .trim().split(/\r?\n/);
        var head = rows.shift().split(",");
        var fi = head.indexOf("field");
        var vi = head.indexOf("value");
        var want = rows.map(function (r) {
            var c = r.split(",");
            return [c[fi], parseInt(c[vi], 10)];
        });
        var got;
        try {
            got = D.decode(data).fields;
        } catch (err) {
            check(false, "JS decodes " + name + ": " + err.message);
            return;
        }
        check(JSON.stringify(got) === JSON.stringify(want),
              "JS decodes " + name + " to its CSV (" + got.length +
              " fields)");
    });
}

function vectors() {
    var got = ["start", "stop", "reset", "summary"].map(function (c) {
        return D.hex(D.commandBytes(c));
    });
    var text;
    var threw = false;
    check(got.join(",") === "81,89,91,A1",
          "JS commands START 81, STOP 89, RESET 91, SNAP A1");
    check(D.hex(D.commandBytes("map", "w", "left")) === "99 20" &&
          D.hex(D.commandBytes("map", "%", "uturn")) === "99 EA",
          "JS MAP W left = 99 20; MAP % uturn = 99 EA");
    try {
        D.commandBytes("map", "*", "left");
    } catch (err) {
        threw = true;
    }
    check(threw, "JS refuses MAP of '*'");
    text = D.snapshotText(Buffer.from("395C869D008401", "hex")) || "";
    check(text.indexOf("RUN / SEARCH_LINE") >= 0 &&
          text.indexOf("1260 mm") >= 0 && text.indexOf("6 s") >= 0 &&
          text.indexOf("RANGE_CLAMP") >= 0,
          "JS S1-U42 bytes decode to RUN / SEARCH_LINE, 1260 mm, 6 s");
    check(D.snapshotText(Buffer.from("1965102700EC01", "hex")) === null,
          "JS does not take a SUMMARY payload for a SNAPSHOT");
}

function lines() {
    var input = fs.readFileSync(0, "utf8").split("\n");
    var out = [];
    input.forEach(function (row) {
        var parts;
        if (row.length === 0) {
            return;
        }
        parts = row.split(" ");
        out.push(D.lineFor(parts[0],
                           Buffer.from(parts.length > 1 ? parts[1] : "",
                                       "hex")));
    });
    fs.writeSync(1, out.join("\n") + "\n");   /* sync: exit follows */
}

var mode = process.argv[2];
if (mode === "golden") {
    golden(process.argv[3]);
} else if (mode === "vectors") {
    vectors();
} else if (mode === "lines") {
    lines();
} else {
    console.error("usage: node test_decode.js golden DIR | vectors | lines");
    process.exit(2);
}
process.exit(fails ? 1 : 0);
