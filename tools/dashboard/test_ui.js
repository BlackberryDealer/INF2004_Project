/*
 * test_ui.js - host test S1-U50: top measurement cards and per-run evidence
 * history in tools/dashboard/index.html and tools/dashboard/dashboard.js.
 *
 * Verifies that a single recorded track run preserves multiple barcode reads,
 * cross-junction turns, hump heights, obstacle widths (cm and mm), and FR9
 * bypass rejoin timings on the top dashboard cards without overwriting prior
 * events, then resets the run ledger cleanly when a new RUN begins.
 */
"use strict";

var fs = require("fs");
var path = require("path");
var vm = require("vm");

var DASH_DIR = __dirname;
var fails = 0;

function check(ok, desc) {
    console.log("[TEST] S1-U50 " + (ok ? "PASS" : "FAIL") + "  " + desc);
    if (!ok) {
        fails += 1;
    }
}

function packBits(pairs) {
    var bytes = [];
    var cur = 0;
    var bit = 0;
    pairs.forEach(function (pair) {
        var val = pair[0];
        var bits = pair[1];
        for (var i = 0; i < bits; i += 1) {
            if ((val >> i) & 1) {
                cur |= (1 << bit);
            }
            bit += 1;
            if (bit === 8) {
                bytes.push(cur);
                cur = 0;
                bit = 0;
            }
        }
    });
    if (bit > 0) {
        bytes.push(cur);
    }
    return new Uint8Array(bytes);
}

function evtBytes(type, value) {
    var word = (value & 0x7ff) | ((type & 0x1f) << 11);
    return new Uint8Array([1 | (1 << 3), word & 0xff, (word >> 8) & 0xff]);
}

function logBytes(text) {
    var head = 1 | (9 << 3);
    var buf = Buffer.from(text, "utf8");
    var out = new Uint8Array(1 + buf.length);
    out[0] = head;
    for (var i = 0; i < buf.length; i += 1) {
        out[1 + i] = buf[i];
    }
    return out;
}

var htmlPath = path.join(DASH_DIR, "index.html");
var jsPath = path.join(DASH_DIR, "dashboard.js");
var html = fs.readFileSync(htmlPath, "utf8");
var js = fs.readFileSync(jsPath, "utf8");

/* 1. Every $("id") referenced in dashboard.js exists in index.html. */
var htmlIds = new Set(Array.from(html.matchAll(/id="([^"]+)"/g), function (m) { return m[1]; }));
var jsIds = Array.from(new Set(Array.from(js.matchAll(/\$\("([^"]+)"\)/g), function (m) { return m[1]; })));
var missingIds = jsIds.filter(function (id) { return !htmlIds.has(id); });
check(missingIds.length === 0, "every DOM id in dashboard.js exists in index.html (" + missingIds.join(", ") + ")");

/* 2. Top three cards appear above stateCard in Hump -> Obstacle -> Barcode order. */
var idxHump = html.indexOf("id=\"sHumpHeight\"");
var idxObs = html.indexOf("id=\"sObsWidth\"");
var idxBar = html.indexOf("id=\"sBarDecoded\"");
var idxState = html.indexOf("id=\"stateCard\"");
check(idxHump > 0 && idxObs > idxHump && idxBar > idxObs && idxState > idxBar,
      "top course measurement cards appear above stateCard");

/* 3. Boot dashboard.js in a minimal DOM sandbox and replay a multi-feature run. */
var elements = {};
function makeEl(id) {
    return {
        id: id,
        textContent: "",
        innerHTML: "",
        value: "",
        checked: false,
        disabled: false,
        className: "",
        clientWidth: 300,
        clientHeight: 150,
        scrollHeight: 0,
        scrollTop: 0,
        offsetWidth: 50,
        offsetTop: 0,
        style: { setProperty: function () {} },
        classList: { toggle: function () {}, add: function () {}, remove: function () {} },
        addEventListener: function () {},
        getAttribute: function (a) { return a === "data-cmd" ? "start" : "30"; },
        querySelector: function () { return makeEl("child"); },
        appendChild: function () {},
        getContext: function () {
            return new Proxy({}, { get: function () { return function () {}; } });
        }
    };
}
htmlIds.forEach(function (id) { elements[id] = makeEl(id); });

var now = 1000000;
var mqttInstance = null;
var sandbox = {
    window: null,
    document: {
        hidden: true,
        documentElement: { setAttribute: function () {}, getAttribute: function () { return "light"; } },
        body: { appendChild: function () {} },
        getElementById: function (id) {
            if (!elements[id]) { throw new Error("Unknown element ID: " + id); }
            return elements[id];
        },
        querySelector: function () { return makeEl("q"); },
        querySelectorAll: function () { return []; },
        createElement: function () { return makeEl("created"); },
        addEventListener: function (ev, fn) { if (ev === "DOMContentLoaded") { sandbox._init = fn; } }
    },
    location: { protocol: "http:", hostname: "localhost" },
    getComputedStyle: function () { return { getPropertyValue: function () { return "#000"; } }; },
    setInterval: function () { return 1; },
    setTimeout: function () { return 1; },
    clearTimeout: function () {},
    requestAnimationFrame: function () { return 1; },
    Date: Object.assign(function (t) { return new Date(t === undefined ? now : t); }, { now: function () { return now; } }),
    Math: Math,
    Number: Number,
    String: String,
    Array: Array,
    Object: Object,
    RegExp: RegExp,
    Error: Error,
    Uint8Array: Uint8Array,
    parseInt: parseInt,
    console: console
};
sandbox.window = sandbox;
sandbox.window.localStorage = {
    getItem: function (k) { return k.endsWith("auto") ? "1" : (k.endsWith("host") ? "localhost" : null); },
    setItem: function () {}
};
sandbox.window.matchMedia = function () { return { matches: false }; };
sandbox.window.addEventListener = function () {};
sandbox.window.MqttWs = function (opts) {
    this.opts = opts;
    this.connect = function () { if (this.onConnect) { this.onConnect(); } };
    this.subscribe = function () { return Promise.resolve(); };
    this.disconnect = function () {};
    mqttInstance = this;
};

vm.createContext(sandbox);
vm.runInContext(fs.readFileSync(path.join(DASH_DIR, "telem_tables.js"), "utf8"), sandbox);
vm.runInContext(fs.readFileSync(path.join(DASH_DIR, "decode.js"), "utf8"), sandbox);
vm.runInContext(js, sandbox);
sandbox._init();

/* Enter RUN / FOLLOW (SYS=4, RUN=0 => STATE_CHANGE value = 4 | (0 << 3), type=21). */
mqttInstance.onMessage("c28/e", evtBytes(21, 4 | (0 << 3)), false);

/* Scan full-scale barcode 'A' (10) -> LEFT (0), then execute cross turn (RUN=2). */
now += 1000;
mqttInstance.onMessage("c28/e", evtBytes(6, 10 | (0 << 6)), false);
now += 800;
mqttInstance.onMessage("c28/e", evtBytes(21, 4 | (2 << 3)), false);
mqttInstance.onMessage("c28/e", evtBytes(21, 4 | (0 << 3)), false);

/* Scan half-scale barcode 'B' (11) -> RIGHT (1), then execute second cross turn. */
now += 1500;
mqttInstance.onMessage("c28/e", evtBytes(6, 11 | (1 << 6)), false);
now += 700;
mqttInstance.onMessage("c28/e", evtBytes(21, 4 | (2 << 3)), false);
mqttInstance.onMessage("c28/e", evtBytes(21, 4 | (0 << 3)), false);

check(elements.sBarDecoded.innerHTML === "B<small>→ RIGHT</small>",
      "barcode card shows latest decode B -> RIGHT");
check(elements.sBarHist.textContent === "Run reads (2): A→LEFT, B→RIGHT · Cross: A→LEFT, Cross: B→RIGHT",
      "barcode history keeps both full- and half-scale reads and cross turns (" + elements.sBarHist.textContent + ")");

/* Two humps in one run: 12 mm then 8 mm. */
now += 1000;
mqttInstance.onMessage("c28/e", evtBytes(17, 12), false);
now += 1200;
mqttInstance.onMessage("c28/e", evtBytes(17, 8), false);
check(elements.sHumpHeight.innerHTML === "8<small>mm</small>" &&
      elements.sHumpDetail.textContent.indexOf("run peak 12 mm") >= 0 &&
      elements.sHumpHist.textContent === "Run humps (2): 12 mm, 8 mm",
      "hump card keeps run peak and per-hump history (" + elements.sHumpHist.textContent + ")");

/* Obstacle bypass: STATE_CHANGE RUN/AVOID (RUN=4), PROFILE 124 mm RIGHT, back to RUN/FOLLOW, then FR9 2840 ms. */
now += 500;
mqttInstance.onMessage("c28/e", evtBytes(21, 4 | (4 << 3)), false);
var profBytes = packBits([[1, 3], [2, 5], [12, 5], [31, 8], [45, 8], [255, 8], [255, 8], [1, 1], [0, 2]]);
mqttInstance.onMessage("c28/e", profBytes, false);
now += 4200;
mqttInstance.onMessage("c28/e", evtBytes(21, 4 | (0 << 3)), false);
check(elements.sObsHist.textContent === "Run bypass (1): 12.4 cm RIGHT · rejoined (4.2 s)",
      "obstacle card tracks AVOID -> FOLLOW bypass duration even without DEBUG_MQTT (" + elements.sObsHist.textContent + ")");

mqttInstance.onMessage("c28/l", logBytes("FR9 2840 ms"), false);
check(elements.sObsWidth.innerHTML === "12.4<small>cm (124 mm)</small>" &&
      elements.sObsHist.textContent === "Run bypass (1): 12.4 cm RIGHT · FR9 2.84 s",
      "obstacle card shows cm + mm width and exact FR9 rejoin timing (" + elements.sObsHist.textContent + ")");

/* End run (SYS=5 FINISHED): run evidence stays visible on screen! */
now += 500;
mqttInstance.onMessage("c28/e", evtBytes(21, 5 | (0 << 3)), false);
check(elements.sBarHist.textContent.indexOf("A→LEFT, B→RIGHT") >= 0 &&
      elements.sHumpHist.textContent === "Run humps (2): 12 mm, 8 mm" &&
      elements.sObsHist.textContent.indexOf("FR9 2.84 s") >= 0,
      "run evidence remains visible in FINISHED for video inspection");

/* Start a second RUN: per-run ledgers reset cleanly. */
now += 1000;
mqttInstance.onMessage("c28/e", evtBytes(21, 4 | (0 << 3)), false);
check(elements.sBarHist.textContent === "" &&
      elements.sHumpHist.textContent === "" &&
      elements.sObsHist.textContent === "",
      "starting a new RUN resets per-run history ledgers");

process.exit(fails ? 1 : 0);
