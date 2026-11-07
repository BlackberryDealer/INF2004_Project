/*
 * dashboard.js - PicoCar Live: the page logic.
 *
 * Owner:  Buddy 1 (communications)
 *
 * Since 2026-11-06 it also shows the car's raw sensors (SENSORS on c28/d)
 * and its console lines (LOG on c28/l), so the car can be debugged
 * without the USB cable.
 *
 * Subscribes to c28/# through Mosquitto's WebSocket listener, decodes every
 * payload with decode.js (a tested port of tools/telem_decode.py) and
 * shows it; the command buttons publish the same bytes as
 * tools/telem_cmd.py on c28/c at QoS 1, then watch the car's state to say
 * whether the command took effect.
 */
(function () {
    "use strict";

    var D = window.TelemDecode;
    var T = D.T;
    var SYS = T.ENUMS.SYS;
    var TOPIC = T.TOPICS;

    var STALE_MS = 1500;          /* no STATE for this long -> stale */
    var GAP_MS = 400;             /* NFR8a 5 Hz: a gap this long is a miss */
    var LAT_LIMIT_US = 1000;      /* NFR17 */
    var LOG_MAX = 5000;           /* kept for export */
    var LOG_SHOWN = 400;          /* rows in the table */
    var HIST_MAX_S = 130;
    var CON_MAX = 5000;           /* console lines kept for export */
    var CON_SHOWN = 600;          /* console lines drawn */
    var SENS_ROWS_MAX = 20000;    /* sensor samples kept for CSV (~66 min) */
    var SENS_STALE_MS = 1500;
    var US_NONE_MM = 2044;        /* APP_DIST_NONE_MM: nothing in range */
    var US_SLOW_MM = 600;         /* OBS_SLOW_MM */
    var US_STOP_MM = 401;         /* OBS_STOP_MM (placeholder geometry) */
    var GAP_LIMIT_MS = 30;        /* watchdog window */
    var GAP_WARN_MS = 15;
    var QUIET_RE = /^(US dist|IR raw|ENC pos|BAR DO|BAR edges|WDT longest)/;
    var BAD_RE = /FAULT|ERROR|FAIL|ESTOP|TOO CLOSE|impossible/;
    var WARN_RE = /OBSTACLE|LINE LOST|LINE END|RAMP|REJOIN|SCAN|AVOID|BARCODE|BAR capture|JUNCTION|IMPACT|MOVE FAILED|dropped/;

    /* What each state looks like (car LED) and what to do next. */
    var STATE_UI = {
        BOOT: { led: "var(--led-boot)", blink: false,
                next: "Booting: waiting for task check-ins and the radio." },
        CAL_IMU: { led: "var(--warning)", blink: true,
                   next: "<strong>Keep the car still</strong>: the IMU is calibrating (about 2 s)." },
        CAL_IR: { led: "var(--warning)", blink: true,
                  next: "Line sensor calibrating: the car sweeps over the line." },
        READY: { led: "var(--good)", blink: false,
                 next: "<strong>Ready.</strong> Put the car on the line and press <strong>START</strong>. It beeps for 1 s, then runs." },
        RUN: { led: "var(--run)", blink: false,
               next: "Running. Press <strong>STOP</strong> (or Space) to end the run." },
        FINISHED: { led: "var(--good)", blink: true,
                    next: "Run finished. Press <strong>RESET</strong> to go back to READY; START only works from READY." },
        LINE_LOST_STOP: { led: "var(--run)", blink: true,
                          next: "Stopped: <strong>line lost</strong>. Put the car back on the line, then press <strong>RESET</strong>." },
        FAULT: { led: "var(--critical)", blink: true,
                 next: "<strong>FAULT</strong>. Fix the cause, then press <strong>RESET</strong>: the car recalibrates from BOOT." }
    };

    /* Which buttons make sense in which state (mission statechart). */
    var ALLOWED = {
        start: ["READY"],
        stop: null,                                 /* always */
        reset: ["FINISHED", "LINE_LOST_STOP", "FAULT"],
        summary: null
    };

    /* What a command should lead to, and how long to wait for it. */
    var EXPECT = {
        start: { from: ["READY"], to: ["RUN"], ms: 3500,
                 wrong: "START only works in READY. After a run, press RESET first." },
        stop: { from: ["RUN"], to: ["FINISHED"], ms: 2000,
                wrong: "STOP only ends a run; the car is not in RUN." },
        reset: { from: ["FINISHED", "LINE_LOST_STOP", "FAULT"],
                 to: ["READY", "BOOT"], ms: 2500,
                 wrong: "RESET only acts in FINISHED, LINE_LOST_STOP or FAULT." },
        summary: { ms: 3500 },
        map: { ms: 2500 }
    };

    var $ = function (id) { return document.getElementById(id); };

    /* ---- storage (per-viewer conveniences only) -------------------------- */
    function load(key, dflt) {
        try {
            var v = window.localStorage.getItem("c28dash." + key);
            return v === null ? dflt : v;
        } catch (e) {
            return dflt;
        }
    }
    function save(key, value) {
        try { window.localStorage.setItem("c28dash." + key, value); } catch (e) { /* private mode */ }
    }

    /* ---- state ----------------------------------------------------------- */
    var client = null;
    var wantConnected = false;
    var retryTimer = null;
    var retryMs = 1000;

    var S = {
        brokerUp: false,
        avail: null,                 /* "ONLINE" | "OFFLINE" | null */
        state: null,                 /* last decoded STATE values */
        stateAt: 0,
        sysName: null,
        prevSys: null,
        runStartedAt: null,
        hb: null,
        hbAt: 0,
        stateTimes: [],
        gaps: 0,
        maxGap: 0,
        msgs: 0,
        rejects: 0,
        pendingCmd: null,
        lastSend: {}
    };
    var hist = [];                   /* {t, sl, sr, steer, pitch, turn} */
    var shist = [];                  /* {t, irl, irr, us, ax, ay, azd, gap, edges} */
    var sensRows = [];               /* every SENSORS sample, for CSV */
    var conLines = [];               /* {t, text} from c28/l */
    var sensLast = null;             /* {t, v: engineering values} */
    var sensTimes = [];
    var logRows = [];
    var winS = parseInt(load("win", "30"), 10) || 30;

    /* ---- helpers --------------------------------------------------------- */
    function nowMs() { return Date.now(); }

    function clock(t) {
        var d = new Date(t);
        function p(n, w) { return String(n).padStart(w || 2, "0"); }
        return p(d.getHours()) + ":" + p(d.getMinutes()) + ":" + p(d.getSeconds()) + "." + p(d.getMilliseconds(), 3);
    }

    function esc(s) {
        return String(s).replace(/[&<>"]/g, function (c) {
            return { "&": "&amp;", "<": "&lt;", ">": "&gt;", "\"": "&quot;" }[c];
        });
    }

    function fmtDur(s) {
        s = Math.max(0, Math.floor(s));
        var m = Math.floor(s / 60);
        return m > 0 ? m + ":" + String(s % 60).padStart(2, "0") : s + " s";
    }

    function fmtDist(mm) {
        return mm >= 1000 ? (mm / 1000).toFixed(2) + " m" : Math.round(mm) + " mm";
    }

    function pill(id, cls, text) {
        var el = $(id);
        el.className = "pill " + (cls || "");
        el.querySelector("b").textContent = text;
    }

    function c39(code) {
        return code < T.CODE39_CHARS.length ? T.CODE39_CHARS[code] : "?";
    }

    function barName(code) {
        return (T.ENUMS.BAR_CMD[code] || "?");
    }

    /* ---- connection ------------------------------------------------------ */
    function defaultHost() {
        if (location.protocol === "http:" || location.protocol === "https:") {
            return location.hostname;
        }
        return "localhost";
    }

    function setConnUi() {
        var btn = $("connBtn");
        if (S.brokerUp) {
            btn.textContent = "Disconnect";
            btn.classList.remove("primary");
            pill("pBroker", "good", "connected");
        } else if (wantConnected) {
            btn.textContent = "Cancel";
            btn.classList.remove("primary");
            pill("pBroker", "warn", "connecting…");
        } else {
            btn.textContent = "Connect";
            btn.classList.add("primary");
            pill("pBroker", "bad", "off");
        }
    }

    function connect() {
        var host = $("host").value.trim() || defaultHost();
        var port = parseInt($("port").value, 10) || 9001;
        save("host", host);
        save("port", String(port));
        clearTimeout(retryTimer);
        if (client) {
            client.onClose = function () {};
            client.disconnect();
        }
        wantConnected = true;
        setConnUi();
        client = new window.MqttWs({ url: "ws://" + host + ":" + port, keepalive: 20 });
        client.onConnect = function () {
            S.brokerUp = true;
            retryMs = 1000;
            hideBanner();
            setConnUi();
            client.subscribe("c28/#", 1).catch(function (err) {
                showBanner("Subscribe failed: " + esc(err.message));
            });
            addLog({ kind: "info", topic: "", type: "BROKER", text: "Connected to ws://" + host + ":" + port });
        };
        client.onMessage = onMessage;
        client.onClose = function (reason) {
            var was = S.brokerUp;
            S.brokerUp = false;
            setConnUi();
            if (was) {
                addLog({ kind: "reject", topic: "", type: "BROKER", text: reason });
            }
            if (wantConnected) {
                showBanner("<strong>Broker not reachable.</strong> " + esc(reason) +
                    ". Retrying in " + Math.round(retryMs / 1000) + " s. Check that Mosquitto runs with the WebSocket listener " +
                    "(<code>listener 9001</code> + <code>protocol websockets</code>) and that the address is the laptop's IP.");
                retryTimer = setTimeout(connect, retryMs);
                retryMs = Math.min(retryMs * 2, 10000);
            }
            render();
        };
        client.connect();
    }

    function disconnect() {
        wantConnected = false;
        clearTimeout(retryTimer);
        if (client) {
            client.disconnect();
        }
        client = null;
        S.brokerUp = false;
        hideBanner();
        setConnUi();
        render();
    }

    function showBanner(html) {
        var b = $("banner");
        b.innerHTML = html;
        b.classList.add("show");
    }
    function hideBanner() { $("banner").classList.remove("show"); }

    /* ---- incoming -------------------------------------------------------- */
    function onMessage(topic, bytes, retained) {
        var t = nowMs();
        var d;
        S.msgs += 1;
        try {
            d = D.decode(bytes);
        } catch (err) {
            if (topic === TOPIC.CMD) {
                return;              /* someone else's malformed command */
            }
            S.rejects += 1;
            addLog({ kind: "reject", topic: topic, type: "REJECT", text: err.message + " (" + bytes.length + " B)", bytes: bytes, t: t });
            return;
        }
        switch (d.typeName) {
        case "STATE": onState(d.values, t); break;
        case "HEARTBEAT":
            S.hb = d.values;
            S.hbAt = t;
            addLog({ kind: "hb", topic: topic, type: "HEARTBEAT", text: D.lineFor(topic, bytes).split(") ")[1] || "", bytes: bytes, t: t, bulk: true });
            break;
        case "ONLINE":
        case "OFFLINE":
            S.avail = d.typeName;
            addLog({ kind: d.typeName === "ONLINE" ? "state" : "fault", topic: topic, type: d.typeName,
                     text: d.typeName === "ONLINE" ? (retained ? "Car is online (retained)" : "Car came online")
                                                   : (retained ? "Car is offline (retained)" : "Car went offline"),
                     bytes: bytes, t: t });
            break;
        case "EVENT": onEvent(topic, bytes, d.values.WORD, t); break;
        case "SUMMARY": onSummary(d.values, t, bytes, topic); break;
        case "SNAPSHOT": onSnapshot(d.values, t, bytes, topic); break;
        case "PROFILE": onProfile(d.values, t, bytes, topic); break;
        case "SENSORS": onSensors(d.values, t); return;   /* no render storm */
        case "LOG": onConsole(d.values.TEXT, t); return;
        default:
            if (d.typeName.indexOf("CMD_") === 0) {
                onCommandEcho(topic, d, bytes, t);
            } else {
                addLog({ kind: "info", topic: topic, type: d.typeName, text: D.lineFor(topic, bytes), bytes: bytes, t: t });
            }
        }
        render();
    }

    /* ---- raw sensors (c28/d) and console (c28/l) ------------------------ */
    function onSensors(v, t) {
        var e = {
            irl: v.IR_L,
            irr: v.IR_R,
            us: D.engValue("MQ_SENS", "US_MM", v.US_MM),
            scan: v.SCAN,
            bar: v.BAR_DO,
            edges: v.BAR_EDGES,
            ax: D.engValue("MQ_SENS", "AX", v.AX),
            ay: D.engValue("MQ_SENS", "AY", v.AY),
            az: D.engValue("MQ_SENS", "AZ", v.AZ),
            gap: D.engValue("MQ_SENS", "WDT_GAP", v.WDT_GAP) / 1000
        };
        sensLast = { t: t, v: e };
        sensTimes.push(t);
        while (sensTimes.length && sensTimes[0] < t - 5000) { sensTimes.shift(); }
        shist.push({ t: t, irl: e.irl, irr: e.irr, us: e.us, ax: e.ax, ay: e.ay,
                     azd: e.az - 1000, gap: e.gap, edges: e.edges });
        while (shist.length && shist[0].t < t - HIST_MAX_S * 1000) { shist.shift(); }
        sensRows.push([t, S.sysName || "", e.irl, e.irr, e.us, D.enumName("SCAN", e.scan),
                       e.bar, e.edges, e.ax, e.ay, e.az, e.gap.toFixed(2)]);
        if (sensRows.length > SENS_ROWS_MAX) { sensRows.splice(0, sensRows.length - SENS_ROWS_MAX); }
    }

    /* DO edges in the last ~2 s (the car sends the low 8 bits: wraps) */
    function recentEdges(t) {
        var old = null;
        for (var i = shist.length - 1; i >= 0; i -= 1) {
            if (shist[i].t <= t - 2000) { old = shist[i]; break; }
        }
        if (!old || !sensLast) { return null; }
        return (sensLast.v.edges - old.edges + 256) % 256;
    }

    function irTag(raw) {
        /* rough reading of the raw ADC (the car normalises with its own
           CAL_IR spans): white ~200, black ~3200 on the 2026-11 track */
        if (raw < 600) { return "white"; }
        if (raw > 2600) { return "black"; }
        return "edge";
    }

    function renderSensors() {
        var t = nowMs();
        if (!sensLast) {
            $("sensAge").textContent = "c28/d: waiting (needs the MQTT image)";
            return;
        }
        var e = sensLast.v;
        var age = t - sensLast.t;
        var hz = sensTimes.filter(function (x) { return x >= t - 5000; }).length / 5;
        $("sensAge").textContent = "c28/d · " + hz.toFixed(1) + " Hz" + (age > SENS_STALE_MS ? " · stale " + (age / 1000).toFixed(1) + " s" : "");
        $("sensAge").style.color = age > SENS_STALE_MS ? "var(--serious)" : "";
        $("sIrL").textContent = e.irl;
        $("sIrR").textContent = e.irr;
        $("sIrLTag").textContent = irTag(e.irl);
        $("sIrRTag").textContent = irTag(e.irr);
        setBar("bIrL", e.irl / 4095);
        setBar("bIrR", e.irr / 4095);
        if (e.us >= US_NONE_MM) {
            $("sUs").innerHTML = "none<small>in range</small>";
            $("sUsTag").textContent = "";
            setBar("bUs", 0);
        } else {
            $("sUs").innerHTML = e.us + "<small>mm</small>";
            $("sUsTag").textContent = e.us < US_STOP_MM ? "stop zone" : (e.us < US_SLOW_MM ? "slow zone" : "");
            $("sUsTag").style.color = e.us < US_STOP_MM ? "var(--critical)" : "var(--serious)";
            setBar("bUs", 1 - Math.min(e.us, 1000) / 1000);
        }
        $("bUs").style.background = e.us < US_STOP_MM ? "var(--critical)" : (e.us < US_SLOW_MM ? "var(--warning)" : "");
        $("sScan").textContent = D.enumName("SCAN", e.scan).replace(/_/g, " ");
        $("sBar").textContent = e.bar ? "BLACK" : "white";
        var rec = recentEdges(t);
        $("sEdges").textContent = "edges " + e.edges + " (mod 256)" + (rec === null ? "" : ", +" + rec + " in 2 s");
        $("sAxy").textContent = e.ax + " / " + e.ay;
        $("sAz").innerHTML = e.az + "<small>mg</small>";
        $("sGap").innerHTML = e.gap.toFixed(1) + "<small>ms</small>";
        $("sGapTag").textContent = e.gap > GAP_WARN_MS ? "CPU starved?" : "";
        $("sGapTag").style.color = "var(--critical)";
        setBar("bGap", e.gap / GAP_LIMIT_MS);
        $("bGap").style.background = e.gap > GAP_WARN_MS ? "var(--critical)" : "";
        $("sensCount").textContent = sensRows.length + " samples kept";
    }

    function exportSensors() {
        var lines = ["time,state,ir_l,ir_r,us_mm,scanner,bar_do,bar_edges,ax_mg,ay_mg,az_mg,wdt_gap_ms"];
        sensRows.forEach(function (r) {
            lines.push([new Date(r[0]).toISOString()].concat(r.slice(1)).join(","));
        });
        download(lines.join("\r\n") + "\r\n", "text/csv", "c28_sensors_", ".csv");
    }

    function onConsole(text, t) {
        conLines.push({ t: t, text: text });
        if (conLines.length > CON_MAX) { conLines.splice(0, conLines.length - CON_MAX); }
        if (!$("conPause").checked) { scheduleConsole(); }
    }

    var conQueued = false;
    function scheduleConsole() {
        if (conQueued) { return; }
        conQueued = true;
        setTimeout(function () { conQueued = false; renderConsole(); }, 150);
    }

    function lineClass(text) {
        if (BAD_RE.test(text)) { return "bad"; }
        if (/^STATE /.test(text)) { return "st"; }
        if (WARN_RE.test(text)) { return "warn"; }
        if (QUIET_RE.test(text) || /^(wifi|mqtt):/.test(text)) { return "dim"; }
        return "";
    }

    function renderConsole() {
        var box = $("conBox");
        var q = $("conFilter").value.trim().toLowerCase();
        var quiet = $("conQuiet").checked;
        var atEnd = box.scrollHeight - box.scrollTop - box.clientHeight < 30;
        var out = [];
        for (var i = conLines.length - 1; i >= 0 && out.length < CON_SHOWN; i -= 1) {
            var l = conLines[i];
            if (quiet && QUIET_RE.test(l.text)) { continue; }
            if (q && l.text.toLowerCase().indexOf(q) < 0) { continue; }
            out.push("<div class=\"ln " + lineClass(l.text) + "\"><span class=\"ts\">" + clock(l.t) + "</span>" + esc(l.text) + "</div>");
        }
        if (!conLines.length) { return; }
        box.innerHTML = out.reverse().join("") || "<div class=\"empty\">No line matches the filter.</div>";
        if (atEnd) { box.scrollTop = box.scrollHeight; }
        $("conCount").textContent = "c28/l · " + conLines.length + " lines kept";
    }

    function exportConsole() {
        var text = conLines.map(function (l) {
            return new Date(l.t).toISOString() + "  " + l.text;
        }).join("\r\n") + "\r\n";
        download(text, "text/plain", "c28_console_", ".txt");
    }

    function download(content, type, prefix, ext) {
        var blob = new Blob([content], { type: type });
        var a = document.createElement("a");
        a.href = URL.createObjectURL(blob);
        a.download = prefix + new Date().toISOString().replace(/[:.]/g, "-").slice(0, 19) + ext;
        document.body.appendChild(a);
        a.click();
        setTimeout(function () { URL.revokeObjectURL(a.href); a.remove(); }, 1000);
    }

    function onState(v, t) {
        var name = SYS[v.SYS] || "?";
        if (S.stateAt) {
            var gap = t - S.stateAt;
            if (gap > GAP_MS) { S.gaps += 1; }
            if (gap > S.maxGap) { S.maxGap = gap; }
        }
        S.stateTimes.push(t);
        while (S.stateTimes.length && S.stateTimes[0] < t - 5000) { S.stateTimes.shift(); }
        if (name !== S.sysName) {
            S.prevSys = S.sysName;
            if (name === "RUN") { S.runStartedAt = t; }
            if (S.sysName !== null) {
                addLog({ kind: "state", topic: TOPIC.STATE, type: "STATE", text: "State " + S.sysName + " → " + name, t: t });
            }
        }
        S.sysName = name;
        S.state = v;
        S.stateAt = t;
        if (S.avail === null) { S.avail = "ONLINE"; }
        hist.push({
            t: t,
            sl: D.engValue("MQ_STATE", "SPEED_L", v.SPEED_L),
            sr: D.engValue("MQ_STATE", "SPEED_R", v.SPEED_R),
            steer: v.STEER,
            pitch: D.engValue("MQ_STATE", "PITCH", v.PITCH),
            turn: v.TURN
        });
        while (hist.length && hist[0].t < t - HIST_MAX_S * 1000) { hist.shift(); }
        checkPending();
    }

    function eventFriendly(evt) {
        var v = evt.values;
        switch (evt.name) {
        case "STATE_CHANGE":
            return "State → " + (SYS[v.SYS] || "?") + (SYS[v.SYS] === "RUN" ? " / " + D.enumName("RUN", v.RUN) : "");
        case "BARCODE": return "Barcode '" + c39(v.CHAR) + "' read → " + barName(v.CMD);
        case "UNMAPPED": return "Barcode '" + c39(v.CHAR) + "' read, no command mapped";
        case "INVALID": return "Barcode read failed (invalid)";
        case "MAP": return "Barcode '" + c39(v.CHAR) + "' now means " + barName(v.CMD);
        case "JUNCTION": return "Junction: " + D.enumName("JCT", v.CLASS);
        case "LINE_LOST": return "Line lost";
        case "LINE_FOUND": return "Line found (crossed " + v.CROSS_MM + " mm)";
        case "HUMP": return "Hump, peak " + v.PEAK_MM + " mm";
        case "MOTION_CLASS": return "Motion: " + D.enumName("MCLASS", v.CLASS);
        case "CAL_DONE": return "Calibration done: " + D.enumName("CAL_SRC", v.SOURCE);
        case "SENSOR_FAULT": return "Fault raised: " + D.enumName("FAULT", v.FAULT_IDX);
        case "PLAN": return "Obstacle plan: pass on the " + D.enumName("SIDE", v.SIDE);
        case "STALL": return "Wheel stall:" + (v.LEFT ? " left" : "") + (v.RIGHT ? " right" : "");
        case "MOVE_DONE": return "Move done (tag " + v.TAG + ")";
        case "OBSTACLE_AHEAD": return "Obstacle ahead";
        case "OBSTACLE": return "Obstacle stop";
        case "NO_PATH": return "No path around the obstacle";
        case "SIDE_TOO_CLOSE": return "Side too close";
        case "CONTINUE": return "Obstacle cleared, continuing";
        case "START": return "START accepted";
        case "STOP": return "STOP accepted";
        case "RESET": return "RESET accepted";
        default: return D.eventText(evt.word);
        }
    }

    function onEvent(topic, bytes, word, t) {
        var evt = D.decodeEvent(word);
        evt.word = word;
        var kind = (evt.name === "SENSOR_FAULT" || evt.name === "STALL") ? "fault" :
                   (evt.name === "STATE_CHANGE" ? "state" : "event");
        addLog({ kind: kind, topic: topic, type: evt.name, text: eventFriendly(evt), exact: D.lineFor(topic, bytes), bytes: bytes, t: t });
        if (evt.name === "MAP" && S.pendingCmd && S.pendingCmd.name === "map") {
            resolvePending(true, "Car confirmed: " + eventFriendly(evt) + ".");
        }
    }

    function onCommandEcho(topic, d, bytes, t) {
        /* Our own command (or another laptop's) coming back from c28/#. */
        var mine = S.lastEcho && S.lastEcho.hex === D.hex(bytes) && t - S.lastEcho.t < 3000;
        if (mine) {
            S.lastEcho = null;
            return;                                /* already logged as sent */
        }
        addLog({ kind: "cmd", topic: topic, type: d.typeName, text: "Command from another client: " + D.lineFor(topic, bytes), bytes: bytes, t: t });
    }

    var lastSummary = null;
    var lastSnap = null;
    var lastProfile = null;

    function onSummary(v, t, bytes, topic) {
        lastSummary = { v: v, t: t };
        addLog({ kind: "state", topic: topic, type: "SUMMARY", text: "Run ended (" + D.enumName("END", v.END_REASON) + "): " +
                 fmtDist(v.ODO_MM) + " in " + v.RUN_S + " s", exact: D.lineFor(topic, bytes), bytes: bytes, t: t });
        renderSummary();
    }

    function onSnapshot(v, t, bytes, topic) {
        lastSnap = { v: v, t: t };
        addLog({ kind: "event", topic: topic, type: "SNAPSHOT", text: D.snapshotText(bytes).replace(/\n/g, " · ").replace(/\s{2,}/g, " "),
                 bytes: bytes, t: t });
        if (S.pendingCmd && S.pendingCmd.name === "summary") {
            resolvePending(true, "Snapshot received after " + ((t - S.pendingCmd.t) / 1000).toFixed(1) + " s.");
        }
        renderSummary();
    }

    function onProfile(v, t, bytes, topic) {
        lastProfile = { v: v, t: t };
        addLog({ kind: "event", topic: topic, type: "PROFILE", text: "Obstacle scanned: width " + v.WIDTH * 4 + " mm, nearest " +
                 v.NEAREST * 4 + " mm, pass " + D.enumName("SIDE", v.SIDE), exact: D.lineFor(topic, bytes), bytes: bytes, t: t });
        renderProfile();
    }

    /* ---- commands -------------------------------------------------------- */
    function commandAllowed(name) {
        var list = ALLOWED[name];
        if ($("unlock").checked || list === null || list === undefined) { return true; }
        if (!isLive()) { return false; }
        return list.indexOf(S.sysName) >= 0;
    }

    function isLive() {
        return S.state !== null && nowMs() - S.stateAt < STALE_MS;
    }

    function send(name, ch, bar) {
        var bytes;
        var t = nowMs();
        var label = name === "summary" ? "SUMMARY" : name.toUpperCase();
        if (!S.brokerUp) {
            setFeedback("bad", "Not connected to the broker.");
            return;
        }
        if (S.lastSend[name] && t - S.lastSend[name] < 250) {
            return;                       /* the car drops repeats < 100 ms anyway */
        }
        S.lastSend[name] = t;
        try {
            bytes = D.commandBytes(name, ch, bar);
        } catch (err) {
            setFeedback("bad", esc(err.message));
            return;
        }
        var exp = EXPECT[name];
        var warnWrong = exp.from && isLive() && exp.from.indexOf(S.sysName) < 0;
        S.pendingCmd = { name: name, label: label, t: t, from: S.sysName, exp: exp, acked: false };
        S.lastEcho = { hex: D.hex(bytes), t: t };
        addLog({ kind: "cmd", topic: TOPIC.CMD, type: "CMD_" + (name === "summary" ? "SNAP" : label),
                 text: "Sent " + label + (name === "map" ? " '" + ch + "' → " + bar.toUpperCase() : ""), bytes: bytes, t: t });
        setFeedback("wait", "<strong>" + label + "</strong> sent (" + D.hex(bytes) + ")… waiting for the broker." +
                    (warnWrong ? "<br>Note: " + exp.wrong : ""));
        client.publish(TOPIC.CMD, bytes, 1).then(function () {
            if (S.pendingCmd && S.pendingCmd.t === t) {
                S.pendingCmd.acked = true;
                setFeedback("wait", "<strong>" + label + "</strong> delivered to the broker. Waiting for the car" +
                    (exp.to ? " to reach " + exp.to.join(" or ") : " to reply") + "…" +
                    (warnWrong ? "<br>Note: " + exp.wrong : ""));
            }
        }).catch(function (err) {
            if (S.pendingCmd && S.pendingCmd.t === t) {
                S.pendingCmd = null;
                setFeedback("bad", "<strong>" + label + "</strong> not delivered: " + esc(err.message));
            }
        });
        S.pendingCmd.timer = setTimeout(function () {
            if (!S.pendingCmd || S.pendingCmd.t !== t) { return; }
            var why;
            if (S.avail === "OFFLINE") {
                why = "the car is OFFLINE.";
            } else if (!isLive()) {
                why = "no telemetry from the car (is it powered and on the hotspot?).";
            } else if (exp.from && exp.from.indexOf(S.pendingCmd.from) < 0) {
                why = "it was in " + S.pendingCmd.from + ". " + exp.wrong;
            } else {
                why = "the car is still in " + S.sysName + ".";
            }
            resolvePending(false, "No effect after " + (exp.ms / 1000).toFixed(1) + " s: " + why);
        }, exp.ms);
        render();
    }

    function checkPending() {
        var p = S.pendingCmd;
        if (!p || !p.exp.to) { return; }
        if (p.exp.to.indexOf(S.sysName) >= 0 && S.sysName !== p.from) {
            resolvePending(true, "Car went " + (p.from || "?") + " → <strong>" + S.sysName + "</strong> in " +
                           ((nowMs() - p.t) / 1000).toFixed(1) + " s.");
        }
    }

    function resolvePending(ok, msg) {
        var p = S.pendingCmd;
        if (!p) { return; }
        clearTimeout(p.timer);
        S.pendingCmd = null;
        setFeedback(ok ? "ok" : "bad", "<strong>" + p.label + "</strong> " + (ok ? "✓ " : "✗ ") + msg);
        addLog({ kind: ok ? "state" : "reject", topic: "", type: p.label, text: (ok ? "Took effect: " : "No effect: ") + msg.replace(/<[^>]+>/g, "") });
    }

    function setFeedback(cls, html) {
        var f = $("feedback");
        f.className = "feedback " + cls;
        f.innerHTML = html;
    }

    /* ---- log ------------------------------------------------------------- */
    function addLog(row) {
        row.t = row.t || nowMs();
        logRows.push(row);
        if (logRows.length > LOG_MAX) { logRows.splice(0, logRows.length - LOG_MAX); }
        if (!$("logPause").checked) { scheduleLog(); }
    }

    var logQueued = false;
    function scheduleLog() {
        if (logQueued) { return; }
        logQueued = true;
        setTimeout(function () { logQueued = false; renderLog(); }, 150);
    }

    function rowVisible(r, q, bulk) {
        if (r.bulk && !bulk) { return false; }
        if (!q) { return true; }
        return (r.topic + " " + r.type + " " + r.text + " " + (r.exact || "")).toLowerCase().indexOf(q) >= 0;
    }

    function renderLog() {
        var q = $("logFilter").value.trim().toLowerCase();
        var bulk = $("logState").checked;
        var out = [];
        var shown = 0;
        for (var i = logRows.length - 1; i >= 0 && shown < LOG_SHOWN; i -= 1) {
            var r = logRows[i];
            if (!rowVisible(r, q, bulk)) { continue; }
            shown += 1;
            var tagCls = { cmd: "cmd", state: "state", fault: "fault", reject: "reject" }[r.kind] || "";
            out.push("<tr title=\"" + esc(r.exact || "") + "\"><td class=\"time num\">" + clock(r.t) + "</td><td class=\"mono\">" +
                     esc(r.topic) + "</td><td class=\"kind\"><span class=\"tag " + tagCls + "\">" + esc(r.type) + "</span></td><td>" +
                     esc(r.text) + "</td><td class=\"hex mono\">" + (r.bytes ? D.hex(r.bytes) : "") + "</td></tr>");
        }
        $("logBody").innerHTML = out.join("");
        $("logCount").textContent = logRows.length + " messages kept" + (shown < logRows.length ? ", " + shown + " shown" : "");
    }

    function exportCsv() {
        var lines = ["time,topic,type,text,decoded,bytes"];
        logRows.forEach(function (r) {
            lines.push([new Date(r.t).toISOString(), r.topic, r.type, r.text, r.exact || "", r.bytes ? D.hex(r.bytes) : ""]
                .map(function (c) { return "\"" + String(c).replace(/"/g, "\"\"") + "\""; }).join(","));
        });
        var blob = new Blob([lines.join("\r\n") + "\r\n"], { type: "text/csv" });
        var a = document.createElement("a");
        a.href = URL.createObjectURL(blob);
        a.download = "c28_log_" + new Date().toISOString().replace(/[:.]/g, "-").slice(0, 19) + ".csv";
        document.body.appendChild(a);
        a.click();
        setTimeout(function () { URL.revokeObjectURL(a.href); a.remove(); }, 1000);
    }

    /* ---- render ---------------------------------------------------------- */
    function render() {
        renderStatus();
        renderState();
        renderButtons();
        renderTiles();
        renderHealth();
        renderSensors();
    }

    function renderStatus() {
        var t = nowMs();
        if (!S.brokerUp) {
            pill("pCar", "", "unknown");
            pill("pTele", "", "none");
            return;
        }
        if (S.avail === "ONLINE") { pill("pCar", "good", "online"); }
        else if (S.avail === "OFFLINE") { pill("pCar", "bad", "offline"); }
        else { pill("pCar", "warn", "not seen"); }
        if (!S.state) {
            pill("pTele", "warn", "waiting");
        } else if (t - S.stateAt > STALE_MS) {
            pill("pTele", "bad", "stale " + ((t - S.stateAt) / 1000).toFixed(1) + " s");
        } else {
            pill("pTele", "good", "live " + rate().toFixed(1) + " Hz");
        }
    }

    function rate() {
        var t = nowMs();
        var n = S.stateTimes.filter(function (x) { return x >= t - 5000; }).length;
        return n / 5;
    }

    function renderState() {
        var card = $("stateCard");
        var live = isLive();
        var name = S.sysName;
        var ui = name ? STATE_UI[name] : null;
        var led = $("led");
        var v = S.state;
        var t = nowMs();
        var next;

        if (!S.brokerUp) {
            next = "Connect to the broker (top right) to see the car.";
        } else if (!v) {
            next = S.avail === "OFFLINE" ? "The car is <strong>offline</strong>: power it on and check it joins the hotspot."
                                         : "Waiting for the car's first STATE message on c28/s…";
        } else if (!live) {
            next = "<strong>Telemetry stopped</strong> " + ((t - S.stateAt) / 1000).toFixed(0) + " s ago. Values shown are the last ones received." +
                   (S.avail === "OFFLINE" ? " The car is offline." : "");
        } else {
            next = ui ? ui.next : "Unknown state " + v.SYS;
            if (name === "FAULT" && v.FAULTS) {
                next += "<br>Active: <strong>" + D.faultNames(v.FAULTS).join(", ") + "</strong>";
            }
            if (name === "RUN") {
                next = "Running: <strong>" + D.enumName("RUN", v.RUN).replace(/_/g, " ") + "</strong>. Press <strong>STOP</strong> (or Space) to end the run.";
            }
        }
        $("next").innerHTML = next;

        card.style.setProperty("--led", ui && v ? ui.led : "var(--text-3)");
        led.classList.toggle("blink", !!(ui && ui.blink && live));
        led.style.opacity = live ? "1" : ".35";
        $("stateName").textContent = name ? name.replace(/_/g, " ") : "NO DATA";
        $("stateAge").textContent = v ? (live ? "live" : "last seen " + clock(S.stateAt)) : "";

        var sub = [];
        if (v) {
            if (name === "RUN") { sub.push("<span class=\"chip\">mode <b>" + D.enumName("RUN", v.RUN) + "</b></span>"); }
            sub.push("<span class=\"chip\">motor <b>" + D.enumName("MOT", v.MOT) + "</b></span>");
            sub.push("<span class=\"chip\">line <b>" + D.enumName("LINE", v.LINE) + "</b></span>");
            if (v.FAULTS) { sub.push("<span class=\"chip\" style=\"border-color:var(--critical)\">faults <b>" + D.faultNames(v.FAULTS).length + "</b></span>"); }
        }
        $("stateSub").innerHTML = sub.join("");

        var runS = null;
        if (name === "RUN" && S.runStartedAt) { runS = (t - S.runStartedAt) / 1000; }
        else if (["FINISHED", "LINE_LOST_STOP", "FAULT"].indexOf(name) >= 0 && lastSummary) { runS = lastSummary.v.RUN_S; }
        $("heroTime").textContent = runS === null ? "–" : fmtDur(runS);
        $("heroDist").textContent = v ? fmtDist(v.ODO_MM) : "–";

        /* Flow map: the three paths through the mission statechart, with
           the command that moves the car on (current state lit). */
        var st = function (s) {
            return "<span class=\"st" + (s === name ? " on" : "") + "\">" + s.replace(/_/g, " ") + "</span>";
        };
        var go = function (cmd) {
            return cmd ? "<span class=\"arr cmd\">" + cmd + " →</span>" : "<span class=\"arr\">→</span>";
        };
        var rows = [
            ["Start-up", [st("BOOT"), go(), st("CAL_IMU"), go(), st("CAL_IR"), go(), st("READY")]],
            ["Run cycle", [st("READY"), go("START"), st("RUN"), go("STOP"), st("FINISHED"), go("RESET"), st("READY")]],
            ["Stops", [st("LINE_LOST_STOP"), go("RESET"), st("READY"), "<span class=\"sep\"></span>",
                       st("FAULT"), go("RESET"), st("BOOT")]]
        ];
        var parts = rows.map(function (r) {
            return "<div class=\"flow-row\"><span class=\"flow-l\">" + r[0] + "</span>" + r[1].join("") + "</div>";
        });
        $("flow").innerHTML = parts.join("");
    }

    function renderButtons() {
        var live = isLive();
        document.querySelectorAll(".cmdbtn").forEach(function (b) {
            var name = b.getAttribute("data-cmd");
            b.disabled = !S.brokerUp || !commandAllowed(name);
            var suggest = live && S.brokerUp && (
                (name === "start" && S.sysName === "READY") ||
                (name === "stop" && S.sysName === "RUN") ||
                (name === "reset" && ["FINISHED", "LINE_LOST_STOP", "FAULT"].indexOf(S.sysName) >= 0));
            b.classList.toggle("suggest", suggest);
        });
        $("mapSend").disabled = !S.brokerUp;
    }

    function setBar(id, frac, center) {
        var el = $(id);
        frac = Math.max(-1, Math.min(1, frac));
        if (center) {
            el.style.left = (frac >= 0 ? 50 : 50 + frac * 50) + "%";
            el.style.width = Math.abs(frac) * 50 + "%";
        } else {
            el.style.left = "0";
            el.style.width = Math.max(0, frac) * 100 + "%";
        }
    }

    function renderTiles() {
        var v = S.state;
        if (!v) { return; }
        var sl = D.engValue("MQ_STATE", "SPEED_L", v.SPEED_L);
        var sr = D.engValue("MQ_STATE", "SPEED_R", v.SPEED_R);
        $("tSpeedL").innerHTML = Math.round(sl) + "<small>mm/s</small>";
        $("tSpeedR").innerHTML = Math.round(sr) + "<small>mm/s</small>";
        setBar("bSpeedL", Math.abs(sl) / 400);
        setBar("bSpeedR", Math.abs(sr) / 400);
        $("tSteer").innerHTML = v.STEER + "<small>‰</small>";
        $("tSteerDir").textContent = v.STEER > 30 ? "line left" : (v.STEER < -30 ? "line right" : "centred");
        setBar("bSteer", -v.STEER / 1000, true);
        $("tOdo").innerHTML = fmtDist(v.ODO_MM).replace(/ (m|mm)$/, "<small>$1</small>");
        $("tLine").textContent = D.enumName("LINE", v.LINE);
        $("tLine").style.color = v.LINE === 1 ? "var(--critical)" : "";
        $("tMot").textContent = D.enumName("MOT", v.MOT).replace(/_/g, " ");
        $("tMot").style.color = (v.MOT >= 6) ? "var(--critical)" : "";
        $("tPitch").innerHTML = D.engValue("MQ_STATE", "PITCH", v.PITCH).toFixed(1) + "<small>°</small>";
        $("tTurn").innerHTML = v.TURN + "<small>°/s</small>";
        $("tMclass").textContent = D.enumName("MCLASS", v.MCLASS);
        $("tMclass").style.color = v.MCLASS === 5 ? "var(--critical)" : "";
        $("tHump").innerHTML = v.HUMP_PEAK + "<small>mm</small>";
        $("tPos").textContent = v.POS_L + " / " + v.POS_R;

        var html = T.ENUMS.FAULT.map(function (f, i) {
            var set = (v.FAULTS >> i) & 1;
            var stop = (T.FAULT_STOP_MASK >> i) & 1;
            return "<span class=\"fault" + (set ? " set " + (stop ? "stop" : "info") : "") + "\">" + (set ? "● " : "") + f.replace(/_/g, " ") + "</span>";
        }).join("");
        $("faults").innerHTML = html;
    }

    function renderHealth() {
        var t = nowMs();
        var hb = S.hb;
        $("hRate").textContent = S.state ? rate().toFixed(1) + " Hz" : "–";
        $("hRate").className = S.state && rate() < 4 ? "warn" : "";
        $("hGaps").textContent = S.gaps;
        $("hGaps").className = S.gaps ? "warn" : "";
        $("hMaxGap").textContent = S.maxGap ? S.maxGap + " ms" : "–";
        $("hMaxGap").className = S.maxGap > 1000 ? "bad" : (S.maxGap > GAP_MS ? "warn" : "");
        if (hb) {
            $("hUptime").textContent = fmtDur(hb.UPTIME_S);
            $("hDrops").textContent = hb.DROPS + (hb.DROPS >= 255 ? " (max)" : "");
            $("hDrops").className = hb.DROPS ? "warn" : "";
            $("hLat").textContent = hb.LAT_MAX_US + " µs";
            $("hLat").className = hb.LAT_MAX_US > LAT_LIMIT_US ? "bad" : "";
            $("hAge").textContent = ((t - S.hbAt) / 1000).toFixed(1) + " s ago";
            $("hAge").className = t - S.hbAt > 3000 ? "bad" : "";
        }
        $("hCount").textContent = S.msgs + " / " + S.rejects;
        $("hCount").className = S.rejects ? "warn" : "";
    }

    function renderSummary() {
        var html = "";
        if (lastSummary) {
            var v = lastSummary.v;
            var avg = v.RUN_S > 0 ? Math.round(v.ODO_MM / v.RUN_S) : 0;
            var faults = D.faultNames(v.FAULTS);
            html += "<div class=\"summary-big num\">" +
                "<div><div class=\"v\">" + fmtDist(v.ODO_MM) + "</div><div class=\"l\">distance</div></div>" +
                "<div><div class=\"v\">" + fmtDur(v.RUN_S) + "</div><div class=\"l\">run time</div></div>" +
                "<div><div class=\"v\">" + avg + " mm/s</div><div class=\"l\">average speed</div></div></div>" +
                "<dl class=\"kv\"><dt>Ended by</dt><dd>" + D.enumName("END", v.END_REASON).replace(/_/g, " ") + "</dd>" +
                "<dt>Highest hump</dt><dd>" + v.HUMP_PEAK + " mm</dd>" +
                "<dt>Faults</dt><dd class=\"" + (faults.length ? "bad" : "") + "\">" + (faults.length ? faults.join(", ") : "none") + "</dd>" +
                "<dt>Received</dt><dd>" + clock(lastSummary.t) + "</dd></dl>";
            $("sumWhen").textContent = "last run";
        }
        if (lastSnap) {
            var s = lastSnap.v;
            var sys = D.enumName("SYS", s.SYS);
            html += "<h2 style=\"margin-top:" + (lastSummary ? "14px" : "0") + "\">Snapshot · " + clock(lastSnap.t) + "</h2>" +
                "<dl class=\"kv\"><dt>State</dt><dd>" + sys + (sys === "RUN" ? " / " + D.enumName("RUN", s.RUN) : "") + "</dd>" +
                "<dt>Distance this run</dt><dd>" + fmtDist(s.ODO_MM) + "</dd>" +
                "<dt>Run time</dt><dd>" + fmtDur(s.RUN_S) + "</dd>" +
                "<dt>Hump peak</dt><dd>" + s.HUMP_PEAK + " mm</dd>" +
                "<dt>Faults</dt><dd>" + (D.faultNames(s.FAULTS).join(", ") || "none") + "</dd></dl>";
        }
        if (html) { $("summary").innerHTML = html; }
    }

    function renderProfile() {
        if (!lastProfile) { return; }
        var v = lastProfile.v;
        var clear = T.PROFILE_CLEAR_MM;
        var mm = function (x) { return x * 4; };
        var width = mm(v.WIDTH);
        var nearest = mm(v.NEAREST);
        var cl = mm(v.CLEAR_L);
        var cr = mm(v.CLEAR_R);
        var angle = T.SCAN_ANGLE_MIN_DEG + v.ANGLE_STEP * T.SCAN_ANGLE_STEP_DEG;
        var side = D.enumName("SIDE", v.SIDE);
        var txt = function (x) { return x >= clear ? "clear" : x + " mm"; };
        /* Top view, car at the bottom centre, 1020 mm -> 150 px. The scan
           runs 30 deg (robot's right) to 150 deg (left); NEAREST is the
           forward distance, so the obstacle sits at y = NEAREST on its
           bearing. */
        var k = 150 / clear;
        var cx = 200;
        var cy = 200;
        var fwd = Math.min(nearest, clear);
        var a = angle * Math.PI / 180;
        var lat = Math.max(-150, Math.min(150, fwd * Math.cos(a) / Math.max(Math.sin(a), 0.2) * k));
        var bx = cx + lat;
        var by = cy - fwd * k;
        var ow = Math.max(8, Math.min(width, clear) * k);
        var svg = "<svg viewBox=\"0 0 400 220\" role=\"img\" aria-label=\"Obstacle profile, top view\">" +
            "<path d=\"M " + (cx - 150) + " " + cy + " A 150 150 0 0 1 " + (cx + 150) + " " + cy + "\" fill=\"none\" stroke=\"var(--grid)\" stroke-dasharray=\"3 4\"/>" +
            "<path d=\"M " + (cx - 75) + " " + cy + " A 75 75 0 0 1 " + (cx + 75) + " " + cy + "\" fill=\"none\" stroke=\"var(--grid)\" stroke-dasharray=\"3 4\"/>" +
            "<text x=\"" + (cx + 79) + "\" y=\"" + (cy - 4) + "\" font-size=\"10\" fill=\"var(--text-3)\">0.5 m</text>" +
            "<line x1=\"" + cx + "\" y1=\"" + cy + "\" x2=\"" + bx.toFixed(1) + "\" y2=\"" + by.toFixed(1) + "\" stroke=\"var(--text-3)\" stroke-width=\"1.5\"/>" +
            "<path d=\"M " + cx + " " + (cy - 20) + " q " + (side === "LEFT" ? -70 : 70) + " -50 0 -120\" fill=\"none\" stroke=\"var(--good)\" stroke-width=\"2\" stroke-dasharray=\"5 4\"/>" +
            "<rect x=\"" + (bx - ow / 2).toFixed(1) + "\" y=\"" + (by - 7).toFixed(1) + "\" width=\"" + ow.toFixed(1) + "\" height=\"14\" rx=\"3\" fill=\"var(--critical)\"/>" +
            "<rect x=\"" + (cx - 12) + "\" y=\"" + (cy - 4) + "\" width=\"24\" height=\"16\" rx=\"4\" fill=\"var(--series-1)\"/>" +
            "<text x=\"" + (cx - 150) + "\" y=\"214\" font-size=\"11\" fill=\"var(--text-2)\">left gap " + txt(cl) + "</text>" +
            "<text x=\"" + (cx + 150) + "\" y=\"214\" font-size=\"11\" fill=\"var(--text-2)\" text-anchor=\"end\">right gap " + txt(cr) + "</text>" +
            "<text x=\"" + cx + "\" y=\"14\" font-size=\"12\" fill=\"var(--text)\" text-anchor=\"middle\" font-weight=\"600\">plan: pass on the " + side + "</text>" +
            "</svg>";
        $("profile").innerHTML = svg + "<dl class=\"kv num\" style=\"margin-top:8px\">" +
            "<dt>Bearing</dt><dd>" + angle + "° (90° ahead, 30° right, 150° left)</dd>" +
            "<dt>Width / nearest</dt><dd>" + txt(width) + " / " + txt(nearest) + "</dd>" +
            "<dt>Clearance left / right</dt><dd>" + txt(cl) + " / " + txt(cr) + "</dd></dl>";
        $("profWhen").textContent = clock(lastProfile.t);
    }

    /* ---- charts (canvas, no library) ------------------------------------- */
    var charts = [];

    function cssVar(name) {
        return getComputedStyle(document.documentElement).getPropertyValue(name).trim();
    }

    function makeChart(el, series, fixed, src) {
        var canvas = el.querySelector("canvas");
        var tip = document.createElement("div");
        tip.className = "tip";
        el.appendChild(tip);
        var ch = { el: el, canvas: canvas, tip: tip, series: series, fixed: fixed, hoverX: null,
                   src: src || function () { return hist; } };
        canvas.addEventListener("mousemove", function (e) {
            var r = canvas.getBoundingClientRect();
            ch.hoverX = e.clientX - r.left;
            drawChart(ch);
        });
        canvas.addEventListener("mouseleave", function () {
            ch.hoverX = null;
            tip.style.display = "none";
            drawChart(ch);
        });
        charts.push(ch);
        return ch;
    }

    function niceStep(range) {
        var raw = range / 4;
        var p = Math.pow(10, Math.floor(Math.log10(raw)));
        var n = raw / p;
        return (n < 1.5 ? 1 : n < 3 ? 2 : n < 7 ? 5 : 10) * p;
    }

    function drawChart(ch) {
        var c = ch.canvas;
        var dpr = window.devicePixelRatio || 1;
        var w = c.clientWidth;
        var h = c.clientHeight;
        if (!w || !h) { return; }
        if (c.width !== Math.round(w * dpr) || c.height !== Math.round(h * dpr)) {
            c.width = Math.round(w * dpr);
            c.height = Math.round(h * dpr);
        }
        var g = c.getContext("2d");
        g.setTransform(dpr, 0, 0, dpr, 0, 0);
        g.clearRect(0, 0, w, h);
        var now = nowMs();
        var t0 = now - winS * 1000;
        var pts = ch.src().filter(function (p) { return p.t >= t0 - 1000; });
        var padL = 40;
        var padR = 8;
        var padT = 6;
        var padB = 24;
        var lo = ch.fixed[0];
        var hi = ch.fixed[1];
        pts.forEach(function (p) {
            ch.series.forEach(function (s) {
                lo = Math.min(lo, p[s.key]);
                hi = Math.max(hi, p[s.key]);
            });
        });
        var step = niceStep(hi - lo || 1);
        lo = Math.floor(lo / step) * step;
        hi = Math.ceil(hi / step) * step;
        if (hi === lo) { hi = lo + step; }
        var X = function (t) { return padL + (t - t0) / (winS * 1000) * (w - padL - padR); };
        var Y = function (v) { return padT + (hi - v) / (hi - lo) * (h - padT - padB); };

        var gridC = cssVar("--grid");
        var textC = cssVar("--text-3");
        g.font = "11px system-ui, sans-serif";
        g.lineWidth = 1;
        g.fillStyle = textC;
        g.textAlign = "right";
        g.textBaseline = "middle";
        for (var v = lo; v <= hi + 1e-9; v += step) {
            var y = Math.round(Y(v)) + 0.5;
            g.strokeStyle = v === 0 ? textC : gridC;
            g.globalAlpha = v === 0 ? 0.6 : 1;
            g.beginPath(); g.moveTo(padL, y); g.lineTo(w - padR, y); g.stroke();
            g.globalAlpha = 1;
            g.fillText(String(Math.round(v * 10) / 10), padL - 6, y);
        }
        g.textAlign = "center";
        g.textBaseline = "alphabetic";
        var tickS = winS <= 30 ? 5 : (winS <= 60 ? 10 : 20);
        for (var s = 0; s <= winS; s += tickS) {
            g.fillText(s === 0 ? "now" : "−" + s + " s", X(now - s * 1000), h - 4);
        }

        g.save();
        g.beginPath();
        g.rect(padL, 0, w - padL - padR, h);
        g.clip();
        ch.series.forEach(function (sr) {
            g.strokeStyle = cssVar(sr.color);
            g.lineWidth = 2;
            g.lineJoin = "round";
            g.beginPath();
            var prev = null;
            pts.forEach(function (p) {
                var x = X(p.t);
                var yy = Y(p[sr.key]);
                if (prev === null || p.t - prev > 1000) { g.moveTo(x, yy); } else { g.lineTo(x, yy); }
                prev = p.t;
            });
            g.stroke();
        });
        g.restore();

        if (ch.hoverX !== null && pts.length) {
            var tx = t0 + (ch.hoverX - padL) / (w - padL - padR) * winS * 1000;
            var best = null;
            pts.forEach(function (p) { if (!best || Math.abs(p.t - tx) < Math.abs(best.t - tx)) { best = p; } });
            if (best && Math.abs(best.t - tx) < 1500) {
                var bx = X(best.t);
                g.strokeStyle = textC;
                g.lineWidth = 1;
                g.beginPath(); g.moveTo(bx + 0.5, padT); g.lineTo(bx + 0.5, h - padB); g.stroke();
                ch.series.forEach(function (sr) {
                    g.fillStyle = cssVar(sr.color);
                    g.strokeStyle = cssVar("--surface");
                    g.lineWidth = 2;
                    g.beginPath(); g.arc(bx, Y(best[sr.key]), 4, 0, Math.PI * 2); g.fill(); g.stroke();
                });
                ch.tip.innerHTML = clock(best.t).slice(0, 8) + " · " + ch.series.map(function (sr) {
                    return (sr.label ? sr.label + " " : "") + "<b>" + (Math.round(best[sr.key] * 10) / 10) + "</b>";
                }).join(" · ");
                ch.tip.style.display = "block";
                var left = Math.min(Math.max(bx + 10, 0), w - ch.tip.offsetWidth);
                ch.tip.style.left = left + "px";
                ch.tip.style.top = (ch.canvas.offsetTop + 6) + "px";
            } else {
                ch.tip.style.display = "none";
            }
        }
    }

    function drawAll() {
        charts.forEach(drawChart);
    }

    /* ---- wiring ---------------------------------------------------------- */
    function initTheme() {
        var th = load("theme", "");
        if (th) { document.documentElement.setAttribute("data-theme", th); }
        $("themeBtn").addEventListener("click", function () {
            var cur = document.documentElement.getAttribute("data-theme") ||
                (window.matchMedia("(prefers-color-scheme: dark)").matches ? "dark" : "light");
            var nxt = cur === "dark" ? "light" : "dark";
            document.documentElement.setAttribute("data-theme", nxt);
            save("theme", nxt);
            drawAll();
        });
    }

    function initMap() {
        $("mapChar").innerHTML = T.CODE39_CHARS.split("").map(function (ch) {
            return "<option value=\"" + esc(ch) + "\">" + (ch === " " ? "(space)" : esc(ch)) + "</option>";
        }).join("");
        $("mapCmd").innerHTML = Object.keys(T.BAR_CMDS).map(function (k) {
            return "<option value=\"" + k + "\">" + k.toUpperCase() + "</option>";
        }).join("");
        $("mapSend").addEventListener("click", function () {
            send("map", $("mapChar").value, $("mapCmd").value);
        });
    }

    function init() {
        initTheme();
        initMap();
        $("host").value = load("host", defaultHost());
        $("port").value = load("port", "9001");
        $("connForm").addEventListener("submit", function (e) {
            e.preventDefault();
            if (wantConnected) { disconnect(); } else { connect(); }
        });
        document.querySelectorAll(".cmdbtn").forEach(function (b) {
            b.addEventListener("click", function () { send(b.getAttribute("data-cmd")); });
        });
        $("unlock").addEventListener("change", renderButtons);
        document.addEventListener("keydown", function (e) {
            var tag = (e.target && e.target.tagName) || "";
            if (e.ctrlKey || e.metaKey || e.altKey || tag === "INPUT" || tag === "SELECT" || tag === "TEXTAREA") { return; }
            var map = { " ": "stop", s: "start", r: "reset", q: "summary" };
            var name = map[e.key.toLowerCase()];
            if (!name) { return; }
            e.preventDefault();
            var btn = document.querySelector(".cmdbtn[data-cmd=\"" + name + "\"]");
            if (btn && !btn.disabled) { btn.click(); }
        });
        ["logFilter", "logState", "logHex", "logPause"].forEach(function (id) {
            $(id).addEventListener("input", function () {
                $("logBox").classList.toggle("hide-hex", !$("logHex").checked);
                renderLog();
            });
        });
        $("logClear").addEventListener("click", function () { logRows = []; renderLog(); });
        $("logExport").addEventListener("click", exportCsv);
        $("resetStats").addEventListener("click", function () {
            S.gaps = 0; S.maxGap = 0; S.msgs = 0; S.rejects = 0;
            renderHealth();
        });

        makeChart(document.querySelector("[data-chart=speed]"),
                  [{ key: "sl", color: "--series-1", label: "L" }, { key: "sr", color: "--series-2", label: "R" }], [0, 100]);
        makeChart(document.querySelector("[data-chart=steer]"), [{ key: "steer", color: "--series-1" }], [-200, 200]);
        makeChart(document.querySelector("[data-chart=pitch]"), [{ key: "pitch", color: "--series-1" }], [-5, 5]);
        makeChart(document.querySelector("[data-chart=turn]"), [{ key: "turn", color: "--series-1" }], [-30, 30]);
        var sh = function () { return shist; };
        makeChart(document.querySelector("[data-chart=ir]"),
                  [{ key: "irl", color: "--series-1" }, { key: "irr", color: "--series-2" }], [0, 4095], sh);
        makeChart(document.querySelector("[data-chart=us]"), [{ key: "us", color: "--series-1" }], [0, 1000], sh);
        makeChart(document.querySelector("[data-chart=acc]"),
                  [{ key: "ax", color: "--series-1" }, { key: "ay", color: "--series-2" }, { key: "azd", color: "--good" }], [-100, 100], sh);
        makeChart(document.querySelector("[data-chart=gap]"), [{ key: "gap", color: "--series-1" }], [0, 30], sh);
        ["conFilter", "conQuiet", "conPause"].forEach(function (id) {
            $(id).addEventListener("input", renderConsole);
        });
        $("conClear").addEventListener("click", function () {
            conLines = [];
            $("conBox").innerHTML = "<div class=\"empty\">Cleared.</div>";
            $("conCount").textContent = "c28/l";
        });
        $("conExport").addEventListener("click", exportConsole);
        $("sensExport").addEventListener("click", exportSensors);
        document.querySelectorAll("#win button").forEach(function (b) {
            b.classList.toggle("on", parseInt(b.getAttribute("data-s"), 10) === winS);
            b.addEventListener("click", function () {
                winS = parseInt(b.getAttribute("data-s"), 10);
                save("win", String(winS));
                document.querySelectorAll("#win button").forEach(function (o) { o.classList.toggle("on", o === b); });
                drawAll();
            });
        });
        window.addEventListener("resize", drawAll);

        setConnUi();
        render();
        renderLog();
        setInterval(function () { render(); }, 500);
        (function loop() {
            if (!document.hidden) { drawAll(); }
            setTimeout(function () { requestAnimationFrame(loop); }, 100);
        }());

        if (load("auto", "1") === "1" && load("host", "") !== "") {
            connect();
        }
    }

    document.addEventListener("DOMContentLoaded", init);
}());
