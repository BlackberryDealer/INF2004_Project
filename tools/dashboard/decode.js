/*
 * decode.js - packed-payload decoder and command encoder for the dashboard.
 *
 * Owner:  Buddy 1 (communications)
 * Spec:   CSC28 AI spec, B1.4 step 9 and 10, Part D.4 (LSB-first writer)
 *
 * A line-for-line port of tools/telem_decode.py (decode, unit_value,
 * event_text, line_for) and tools/telem_cmd.py (command_bytes,
 * snapshot_text). Same field tables (telem_tables.js is generated from
 * those tools), same checks, same error texts, same printed lines; host
 * test S1-U44 (tools/dashboard/test_dashboard.py) compares the two on the
 * golden vectors and on thousands of random payloads.
 *
 * Works in a browser (window.TelemDecode, needs telem_tables.js loaded
 * first) and in Node (require("./decode.js")).
 */
(function (root, factory) {
    "use strict";
    if (typeof module !== "undefined" && module.exports) {
        module.exports = factory(require("./telem_tables.js"));
    } else {
        root.TelemDecode = factory(root.TELEM_TABLES);
    }
}(this, function (T) {
    "use strict";

    var GROUPS = T.GROUPS;

    /* LSB-first bit reader; mirrors app_bits_read (Part D.4). */
    function BitReader(data) {
        this.data = data;
        this.pos = 0;
    }

    BitReader.prototype.read = function (bits) {
        var value = 0;
        var i;
        var byte;
        for (i = 0; i < bits; i += 1) {
            byte = this.pos >> 3;
            if (byte >= this.data.length) {
                throw new Error("payload too short");
            }
            if ((this.data[byte] >> (this.pos & 7)) & 1) {
                value += Math.pow(2, i);
            }
            this.pos += 1;
        }
        return value;
    };

    function toBytes(data) {
        if (data instanceof Uint8Array) {
            return data;
        }
        return new Uint8Array(data);
    }

    /* Bytes of a group's payload after the header byte. */
    function payloadBytes(group) {
        var bits = 0;
        GROUPS[group].forEach(function (f) { bits += f[1]; });
        return Math.floor((bits + 7) / 8);
    }

    /* Stored (offset-removed) value of every non-PAD field, in CSV order. */
    function decodeStored(group, reader) {
        var out = [];
        GROUPS[group].forEach(function (f) {
            var raw = reader.read(f[1]);
            if (f[0] !== "PAD") {
                out.push([f[0], raw - f[2]]);
            }
        });
        return out;
    }

    function decodeHeader(data) {
        var reader;
        var version;
        var mqType;
        if (data.length < 1) {
            throw new Error("empty payload");
        }
        reader = new BitReader(data);
        version = reader.read(GROUPS.MQ_HDR[0][1]);
        mqType = reader.read(GROUPS.MQ_HDR[1][1]);
        if (version !== T.MQ_VERSION) {
            throw new Error("version " + version + ", expected " +
                            T.MQ_VERSION);
        }
        return { version: version, type: mqType, reader: reader };
    }

    /*
     * {typeName, group, fields: [[name, stored], ...], values: {name:
     * stored}} for one payload. Throws Error (same text as Python's
     * ValueError) on a bad payload.
     */
    function decode(data) {
        var bytes = toBytes(data);
        var hdr = decodeHeader(bytes);
        var entry = T.TYPE_GROUP[String(hdr.type)];
        var want;
        var fields;
        var values = {};
        if (entry === undefined) {
            throw new Error("unknown type " + hdr.type);
        }
        if (entry[1] === "TEXT") {
            values.TEXT = logText(bytes.subarray(1));
            return { typeName: entry[0], group: "TEXT",
                     fields: [["TEXT", values.TEXT]], values: values };
        }
        if (entry[1] === null) {
            if (bytes.length !== 1) {
                throw new Error(entry[0] + " is " + bytes.length +
                                " bytes, expected 1");
            }
            return { typeName: entry[0], group: null, fields: [],
                     values: values };
        }
        want = 1 + payloadBytes(entry[1]);
        if (bytes.length !== want) {
            throw new Error(entry[0] + " is " + bytes.length +
                            " bytes, expected " + want);
        }
        fields = decodeStored(entry[1], hdr.reader);
        fields.forEach(function (f) { values[f[0]] = f[1]; });
        return { typeName: entry[0], group: entry[1], fields: fields,
                 values: values };
    }

    /* LOG text exactly as telem_decode.log_text(): printable ASCII kept,
       any other byte shown as "?". */
    function logText(body) {
        var out = "";
        var i;
        for (i = 0; i < body.length; i += 1) {
            out += (body[i] >= 32 && body[i] < 127)
                ? String.fromCharCode(body[i]) : "?";
        }
        return out;
    }

    function fieldInfo(group, name) {
        var list = GROUPS[group] || [];
        var i;
        for (i = 0; i < list.length; i += 1) {
            if (list[i][0] === name) {
                return list[i];
            }
        }
        return null;
    }

    /* Speed field (counts per 50 ms window) in mm/s, as a number. */
    function speedMmS(stored) {
        return stored * T.UM_PER_COUNT / T.SPEED_WINDOW_MS;
    }

    /* Python "%.0f": correctly rounded, ties to even. */
    function fmt0(x) {
        var r = Math.round(x);
        if (Math.abs(x - Math.trunc(x)) === 0.5 && (r % 2) !== 0) {
            r -= 1;
        }
        if (r === 0 && (x < 0 || Object.is(x, -0))) {
            return "-0";
        }
        return String(r);
    }

    /* Python "%.1f" (no ties are possible for n / 10). */
    function fmt1(x) {
        return x.toFixed(1);
    }

    /* Stored value converted to the printable text of telem_decode.py. */
    function unitText(group, name, stored) {
        var f = fieldInfo(group, name);
        var unit;
        var lsb;
        if (f === null) {
            return String(stored);
        }
        lsb = f[3];
        unit = f[4];
        if (unit === "count/50ms") {
            return fmt0(speedMmS(stored)) + "mm/s";
        }
        if (unit === "0.1 deg") {
            return fmt1(stored / 10.0) + "deg";
        }
        if (["mm", "s", "us", "mg", "deg/s", "permille"].indexOf(unit) >= 0 ||
                lsb !== 1) {
            return String(stored * lsb) +
                   (unit === "permille" ? "" : unit.split("permille").join(""));
        }
        return String(stored);
    }

    /*
     * Engineering value as a number: speed in mm/s, 0.1 deg in deg, x lsb
     * otherwise (4 mm fields x 4).
     */
    function engValue(group, name, stored) {
        var f = fieldInfo(group, name);
        if (f === null) {
            return stored;
        }
        if (f[4] === "count/50ms") {
            return speedMmS(stored);
        }
        if (f[4] === "0.1 deg") {
            return stored / 10.0;
        }
        return stored * f[3];
    }

    /* {name, group, fields, values} for a 16-bit app event word (MSG_EVT). */
    function decodeEvent(word) {
        var reader = new BitReader(new Uint8Array([word & 0xFF, word >> 8]));
        var value = reader.read(GROUPS.MSG_EVT[0][1]);
        var evtType = reader.read(GROUPS.MSG_EVT[1][1]);
        var name = T.EVT_NAMES[String(evtType)];
        var group;
        var out = { name: name, type: evtType, value: value, group: null,
                    fields: [], values: {} };
        if (name === undefined) {
            out.name = "EVT" + evtType;
        }
        group = T.EVT_GROUP_ALIAS[out.name] || ("EVT_" + out.name);
        if (GROUPS[group] === undefined) {
            return out;
        }
        out.group = group;
        out.fields = decodeStored(group,
            new BitReader(new Uint8Array([value & 0xFF, value >> 8])));
        out.fields.forEach(function (f) { out.values[f[0]] = f[1]; });
        return out;
    }

    /* 'TYPE field=value ...' exactly as telem_decode.event_text(). */
    function eventText(word) {
        var evt = decodeEvent(word);
        if (evt.group === null) {
            return evt.name + " value=" + evt.value;
        }
        return evt.name + evt.fields.map(function (f) {
            return " " + f[0] + "=" + f[1];
        }).join("");
    }

    /* One printable line, exactly as telem_decode.line_for(). */
    function lineFor(topic, data) {
        var bytes = toBytes(data);
        var d;
        var body;
        try {
            d = decode(bytes);
        } catch (err) {
            return topic + " REJECT " + err.message + " (" + bytes.length +
                   " B)";
        }
        if (d.group === "MQ_EVT") {
            body = eventText(d.fields[0][1]);
        } else if (d.group === "TEXT") {
            body = d.values.TEXT;
        } else if (d.group === null) {
            body = "";
        } else {
            body = d.fields.map(function (f) {
                return f[0] + "=" + unitText(d.group, f[0], f[1]);
            }).join(" ");
        }
        return (topic + " " + d.typeName + " (" + bytes.length + " B) " +
                body).replace(/\s+$/, "");
    }

    /* Byte 0 of a payload: version in bits 0-2, type in bits 3-7. */
    function header(mqType) {
        return T.MQ_VERSION | (mqType << 3);
    }

    /* The payload for one command name ('map' needs ch and barCmd). */
    function commandBytes(name, ch, barCmd) {
        var up;
        var value;
        if (T.CMD_TYPES[name] === undefined) {
            throw new Error("unknown command " + name);
        }
        if (name !== "map") {
            return new Uint8Array([header(T.CMD_TYPES[name])]);
        }
        up = (typeof ch === "string" && ch.length === 1) ? ch.toUpperCase()
                                                         : ch;
        if (typeof up !== "string" || up.length !== 1 ||
                T.CODE39_CHARS.indexOf(up) < 0) {
            throw new Error("MAP character must be one of '" +
                            T.CODE39_CHARS + "'");
        }
        if (typeof barCmd !== "string" ||
                T.BAR_CMDS[barCmd.toLowerCase()] === undefined) {
            throw new Error("MAP command must be one of " +
                            Object.keys(T.BAR_CMDS).join(", "));
        }
        value = T.CODE39_CHARS.indexOf(up) |
                (T.BAR_CMDS[barCmd.toLowerCase()] << 6);
        return new Uint8Array([header(T.CMD_TYPES.map), value]);
    }

    function enumName(table, code) {
        var names = T.ENUMS[table];
        return (names && code < names.length) ? names[code] : "?";
    }

    function faultNames(mask) {
        return T.ENUMS.FAULT.filter(function (_n, i) {
            return (mask & (1 << i)) !== 0;
        });
    }

    /* Readable lines for a SNAPSHOT payload, or null if it is not one. */
    function snapshotText(data) {
        var d;
        var v;
        var sysName;
        var faults;
        try {
            d = decode(data);
        } catch (err) {
            return null;
        }
        if (d.typeName !== "SNAPSHOT") {
            return null;
        }
        v = d.values;
        sysName = enumName("SYS", v.SYS);
        faults = faultNames(v.FAULTS);
        return [
            "state:     " + sysName +
                (sysName === "RUN" ? " / " + enumName("RUN", v.RUN) : ""),
            "distance:  " + v.ODO_MM + " mm this run",
            "run time:  " + v.RUN_S + " s",
            "hump peak: " + v.HUMP_PEAK + " mm",
            "faults:    " + (faults.length ? faults.join(", ") : "none")
        ].join("\n");
    }

    function hex(data) {
        return Array.prototype.map.call(toBytes(data), function (b) {
            return (b < 16 ? "0" : "") + b.toString(16).toUpperCase();
        }).join(" ");
    }

    return {
        T: T,
        BitReader: BitReader,
        payloadBytes: payloadBytes,
        decode: decode,
        decodeEvent: decodeEvent,
        eventText: eventText,
        unitText: unitText,
        engValue: engValue,
        speedMmS: speedMmS,
        lineFor: lineFor,
        header: header,
        commandBytes: commandBytes,
        snapshotText: snapshotText,
        enumName: enumName,
        faultNames: faultNames,
        hex: hex
    };
}));
