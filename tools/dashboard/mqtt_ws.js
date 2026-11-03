/*
 * mqtt_ws.js - a small MQTT 3.1.1 client over WebSocket for the dashboard.
 *
 * Owner:  Buddy 1 (communications)
 *
 * A browser cannot open the broker's plain TCP port 1883, so the dashboard
 * talks to Mosquitto's WebSocket listener (see README.md). This client
 * covers what the dashboard needs and nothing more, so the page works with
 * no internet connection and no library download:
 *
 *   CONNECT (clean session, keep-alive)   CONNACK
 *   SUBSCRIBE (QoS 0/1)                    SUBACK
 *   PUBLISH in, QoS 0 and 1 (PUBACK sent)  PUBLISH out, QoS 0 and 1 (PUBACK
 *   PINGREQ every keepalive / 2            awaited, so the page can show
 *   DISCONNECT                             "broker accepted the command")
 *
 * Retained messages arrive like any other PUBLISH, with retain = true.
 *
 * Usage:
 *   var c = new MqttWs({ url: "ws://10.14.95.64:9001", clientId: "dash" });
 *   c.onConnect = function () { c.subscribe("c28/#", 1); };
 *   c.onMessage = function (topic, bytes, retain) { ... };
 *   c.onClose   = function (reason) { ... };
 *   c.connect();
 *   c.publish("c28/c", new Uint8Array([0x81]), 1).then(...);
 */
(function (root, factory) {
    "use strict";
    if (typeof module !== "undefined" && module.exports) {
        module.exports = factory();
    } else {
        root.MqttWs = factory();
    }
}(this, function () {
    "use strict";

    var CONNECT = 1;
    var CONNACK = 2;
    var PUBLISH = 3;
    var PUBACK = 4;
    var SUBSCRIBE = 8;
    var SUBACK = 9;
    var PINGREQ = 12;
    var PINGRESP = 13;
    var DISCONNECT = 14;

    var CONNACK_TEXT = [
        "accepted",
        "unacceptable protocol version",
        "client id rejected",
        "server unavailable",
        "bad user name or password",
        "not authorised"
    ];

    function utf8(text) {
        return new TextEncoder().encode(text);
    }

    function mqttString(text) {
        var b = utf8(text);
        var out = new Uint8Array(2 + b.length);
        out[0] = b.length >> 8;
        out[1] = b.length & 0xFF;
        out.set(b, 2);
        return out;
    }

    function concat(parts) {
        var len = 0;
        var out;
        var pos = 0;
        parts.forEach(function (p) { len += p.length; });
        out = new Uint8Array(len);
        parts.forEach(function (p) { out.set(p, pos); pos += p.length; });
        return out;
    }

    /* Fixed header + remaining-length varint + body. */
    function packet(firstByte, body) {
        var len = body.length;
        var lenBytes = [];
        do {
            var digit = len % 128;
            len = Math.floor(len / 128);
            if (len > 0) {
                digit |= 0x80;
            }
            lenBytes.push(digit);
        } while (len > 0);
        return concat([new Uint8Array([firstByte]), new Uint8Array(lenBytes),
                       body]);
    }

    function u16(n) {
        return new Uint8Array([(n >> 8) & 0xFF, n & 0xFF]);
    }

    function MqttWs(opts) {
        this.url = opts.url;
        this.clientId = opts.clientId ||
            ("c28dash-" + Math.random().toString(16).slice(2, 10));
        this.keepalive = opts.keepalive || 20;
        this.connectTimeoutMs = opts.connectTimeoutMs || 5000;
        this.WebSocketImpl = opts.WebSocket ||
            (typeof WebSocket !== "undefined" ? WebSocket : null);
        this.ws = null;
        this.rx = new Uint8Array(0);
        this.nextId = 1;
        this.pending = {};       /* packet id -> {resolve, reject, timer} */
        this.connected = false;
        this.pingTimer = null;
        this.pingOutstanding = false;
        this.connectTimer = null;
        this.closedByUser = false;
        this.onConnect = function () {};
        this.onMessage = function () {};
        this.onClose = function () {};
    }

    MqttWs.prototype.connect = function () {
        var self = this;
        var ws;
        this.closedByUser = false;
        try {
            ws = new this.WebSocketImpl(this.url, ["mqtt"]);
        } catch (err) {
            setTimeout(function () { self.onClose(String(err.message)); }, 0);
            return;
        }
        this.ws = ws;
        ws.binaryType = "arraybuffer";
        ws.onopen = function () {
            var flags = 0x02;    /* clean session */
            var body = concat([mqttString("MQTT"),
                               new Uint8Array([4, flags]),
                               u16(self.keepalive),
                               mqttString(self.clientId)]);
            self.send(packet(CONNECT << 4, body));
        };
        ws.onmessage = function (ev) {
            self.feed(new Uint8Array(ev.data));
        };
        ws.onerror = function () { /* onclose follows with the reason */ };
        ws.onclose = function (ev) {
            var why = self.closedByUser ? "disconnected" :
                (self.connected ? "connection lost" :
                 "cannot reach the broker WebSocket at " + self.url);
            if (ev && ev.code && ev.code !== 1000 && ev.code !== 1005 &&
                    !self.closedByUser) {
                why += " (code " + ev.code + ")";
            }
            self.teardown(why);
        };
        this.connectTimer = setTimeout(function () {
            if (!self.connected) {
                self.teardown("no CONNACK from " + self.url + " within " +
                              (self.connectTimeoutMs / 1000) + " s");
            }
        }, this.connectTimeoutMs);
    };

    MqttWs.prototype.teardown = function (reason) {
        var ids = Object.keys(this.pending);
        var self = this;
        var wasOpen = this.ws !== null;
        clearInterval(this.pingTimer);
        clearTimeout(this.connectTimer);
        this.pingTimer = null;
        this.connected = false;
        ids.forEach(function (id) {
            clearTimeout(self.pending[id].timer);
            self.pending[id].reject(new Error(reason));
        });
        this.pending = {};
        if (this.ws) {
            this.ws.onclose = null;
            this.ws.onmessage = null;
            try { this.ws.close(); } catch (e) { /* already closed */ }
            this.ws = null;
        }
        if (wasOpen) {
            this.onClose(reason);
        }
    };

    MqttWs.prototype.disconnect = function () {
        this.closedByUser = true;
        if (this.ws && this.connected) {
            this.send(packet(DISCONNECT << 4, new Uint8Array(0)));
        }
        this.teardown("disconnected");
    };

    MqttWs.prototype.send = function (bytes) {
        if (this.ws && this.ws.readyState === 1) {
            this.ws.send(bytes);
            return true;
        }
        return false;
    };

    MqttWs.prototype.allocId = function () {
        var id = this.nextId;
        this.nextId = (this.nextId % 65535) + 1;
        return id;
    };

    MqttWs.prototype.waitAck = function (id, what) {
        var self = this;
        return new Promise(function (resolve, reject) {
            var timer = setTimeout(function () {
                delete self.pending[id];
                reject(new Error("no " + what + " from the broker"));
            }, 5000);
            self.pending[id] = { resolve: resolve, reject: reject,
                                 timer: timer };
        });
    };

    MqttWs.prototype.subscribe = function (filter, qos) {
        var id = this.allocId();
        var body = concat([u16(id), mqttString(filter),
                           new Uint8Array([qos || 0])]);
        var p = this.waitAck(id, "SUBACK");
        if (!this.send(packet((SUBSCRIBE << 4) | 0x02, body))) {
            return Promise.reject(new Error("not connected"));
        }
        return p;
    };

    /* Resolves when the broker has the message (PUBACK for QoS 1). */
    MqttWs.prototype.publish = function (topic, payload, qos, retain) {
        var q = qos ? 1 : 0;
        var first = (PUBLISH << 4) | (q << 1) | (retain ? 1 : 0);
        var id = 0;
        var parts = [mqttString(topic)];
        var p = Promise.resolve();
        if (!this.connected) {
            return Promise.reject(new Error("not connected to the broker"));
        }
        if (q === 1) {
            id = this.allocId();
            parts.push(u16(id));
            p = this.waitAck(id, "PUBACK");
        }
        parts.push(payload);
        if (!this.send(packet(first, concat(parts)))) {
            return Promise.reject(new Error("not connected to the broker"));
        }
        return p;
    };

    MqttWs.prototype.feed = function (chunk) {
        var buf = concat([this.rx, chunk]);
        var pos = 0;
        while (buf.length - pos >= 2) {
            var mult = 1;
            var len = 0;
            var i = pos + 1;
            var digit;
            var complete = false;
            do {
                if (i >= buf.length) {
                    break;
                }
                digit = buf[i];
                len += (digit & 0x7F) * mult;
                mult *= 128;
                i += 1;
                complete = (digit & 0x80) === 0;
            } while (!complete && (i - pos) <= 4);
            if (!complete || buf.length < i + len) {
                break;                      /* wait for the rest */
            }
            this.handle(buf[pos], buf.subarray(i, i + len));
            pos = i + len;
        }
        this.rx = buf.slice(pos);
    };

    MqttWs.prototype.handle = function (first, body) {
        var type = first >> 4;
        var self = this;
        var id;
        switch (type) {
        case CONNACK:
            clearTimeout(this.connectTimer);
            if (body[1] !== 0) {
                this.teardown("broker refused: " +
                              (CONNACK_TEXT[body[1]] || ("code " + body[1])));
                return;
            }
            this.connected = true;
            this.pingOutstanding = false;
            this.pingTimer = setInterval(function () {
                if (self.pingOutstanding) {
                    self.teardown("broker stopped answering (no PINGRESP)");
                    return;
                }
                self.pingOutstanding = true;
                self.send(packet(PINGREQ << 4, new Uint8Array(0)));
            }, this.keepalive * 500);
            this.onConnect();
            break;
        case PUBLISH: {
            var qos = (first >> 1) & 3;
            var retain = (first & 1) === 1;
            var tlen = (body[0] << 8) | body[1];
            var topic = new TextDecoder().decode(body.subarray(2, 2 + tlen));
            var pos = 2 + tlen;
            if (qos > 0) {
                id = (body[pos] << 8) | body[pos + 1];
                pos += 2;
                this.send(packet(PUBACK << 4, u16(id)));
            }
            this.onMessage(topic, body.slice(pos), retain);
            break;
        }
        case PUBACK:
        case SUBACK:
            id = (body[0] << 8) | body[1];
            if (this.pending[id]) {
                clearTimeout(this.pending[id].timer);
                if (type === SUBACK && body[2] === 0x80) {
                    this.pending[id].reject(new Error("subscribe refused"));
                } else {
                    this.pending[id].resolve();
                }
                delete this.pending[id];
            }
            break;
        case PINGRESP:
            this.pingOutstanding = false;
            break;
        default:
            break;                          /* not used by the dashboard */
        }
    };

    return MqttWs;
}));
