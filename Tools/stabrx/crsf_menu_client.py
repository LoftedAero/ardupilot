#!/usr/bin/env python3
"""
AP_FLAKE8_CLEAN

Minimal CRSF receiver + radio emulator for testing the gyro setup menu in
SITL. It connects to a SITL serial port exposed over TCP (for example
--serial5=tcp:5765 with SERIAL5_PROTOCOL 23), streams RC channel frames so
ArduPilot detects a CRSF receiver, and talks the CRSF parameter protocol
the way the ExpressLRS Lua script does: ping for devices, read the field
count from the device info, then load fields 1..count one by one,
requesting extra chunks when an entry does not fit in one frame.
"""
import socket
import struct
import threading
import time

SYNC = 0xC8
ADDR_FC = 0xC8
ADDR_RADIO = 0xEA

T_RC = 0x16
T_PING = 0x28
T_DEVICE_INFO = 0x29
T_ENTRY = 0x2B
T_READ = 0x2C
T_WRITE = 0x2D

TYPE_NAMES = {0: "UINT8", 1: "INT8", 2: "UINT16", 3: "INT16", 8: "FLOAT", 9: "SELECT", 10: "STRING",
              11: "FOLDER", 12: "INFO", 13: "COMMAND"}


def crc8_dvb_s2(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def frame(ftype, payload):
    body = bytes([ftype]) + bytes(payload)
    return bytes([SYNC, len(body) + 1]) + body + bytes([crc8_dvb_s2(body)])


def rc_frame(channels_us):
    # 16 channels, 11 bits each, CRSF scale: 988us = 172, 2012us = 1811
    bits = 0
    for i, us in enumerate(channels_us[:16] + [1500] * (16 - len(channels_us))):
        v = int(round((us - 1500) * 1.6 + 992))
        v = max(0, min(2047, v))
        bits |= v << (11 * i)
    return frame(T_RC, bits.to_bytes(22, "little"))


def cstr(data, offset):
    end = data.index(0, offset)
    return data[offset:end].decode(errors="replace"), end + 1


class CrsfClient:
    def __init__(self, host="127.0.0.1", port=5765):
        self.sock = socket.create_connection((host, port), timeout=10)
        self.sock.settimeout(0.05)
        self.channels = [1500, 1500, 988, 1500, 988, 988, 988, 1500] + [1500] * 8
        self.frames = []
        self.lock = threading.Lock()
        self.running = True
        self.buf = b""
        # (arrival time, frame type) of every frame, when record is set
        self.record = False
        self.arrivals = []
        threading.Thread(target=self._rc_loop, daemon=True).start()
        threading.Thread(target=self._rx_loop, daemon=True).start()

    def close(self):
        self.running = False
        self.sock.close()

    def send(self, ftype, payload):
        with self.lock:
            self.sock.sendall(frame(ftype, payload))

    def _rc_loop(self):
        while self.running:
            try:
                with self.lock:
                    self.sock.sendall(rc_frame(self.channels))
            except OSError:
                return
            time.sleep(0.01)

    def _rx_loop(self):
        while self.running:
            try:
                chunk = self.sock.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                return
            if not chunk:
                return
            self.buf += chunk
            while len(self.buf) >= 2:
                length = self.buf[1]
                if length < 2 or length > 62:
                    self.buf = self.buf[1:]
                    continue
                if len(self.buf) < length + 2:
                    break
                body = self.buf[2:2 + length - 1]
                crc = self.buf[1 + length]
                self.buf = self.buf[2 + length:]
                if crc8_dvb_s2(body) != crc:
                    continue
                with self.lock:
                    self.frames.append((body[0], bytes(body[1:])))
                    if self.record:
                        self.arrivals.append((time.time(), body[0]))

    def wait_frame(self, match, timeout=0.5):
        t0 = time.time()
        while time.time() - t0 < timeout:
            with self.lock:
                for i, (ftype, payload) in enumerate(self.frames):
                    if match(ftype, payload):
                        del self.frames[i]
                        return ftype, payload
                # drop telemetry we are not interested in
                if len(self.frames) > 200:
                    self.frames = self.frames[-50:]
            time.sleep(0.005)
        return None

    def pop_frame(self):
        """oldest received frame (type, payload) or None, for consumers that
        process every frame in order, like a radio script"""
        with self.lock:
            if self.frames:
                return self.frames.pop(0)
        return None

    def purge(self, param_id):
        """drop late replies for an entry, e.g. to polls sent while the
        flight controller was busy, so the next answer is the fresh one"""
        with self.lock:
            self.frames = [(t, p) for (t, p) in self.frames
                           if not (t in (T_ENTRY, T_WRITE) and len(p) > 2 and p[2] == param_id)]

    def ping(self, timeout=5):
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.send(T_PING, [0x00, ADDR_RADIO])
            r = self.wait_frame(lambda t, p: t == T_DEVICE_INFO and p[1] == ADDR_FC, 0.5)
            if r:
                p = r[1]
                name, off = cstr(p, 2)
                return {"name": name, "field_count": p[off + 12], "version": p[off + 13]}
        return None

    def read_entry(self, param_id, retries=6, timeout=0.5):
        """read one entry, all chunks; returns the raw entry bytes (after
        parent id and type) or None if the device never answered"""
        data = b""
        chunk = 0
        attempts = 0
        while True:
            self.purge(param_id)
            self.send(T_READ, [ADDR_FC, ADDR_RADIO, param_id, chunk])
            r = self.wait_frame(lambda t, p: t == T_ENTRY and p[2] == param_id, timeout)
            if r is None:
                attempts += 1
                if attempts >= retries:
                    return None
                continue
            payload = r[1]
            chunks_left = payload[3]
            data += payload[4:]
            if chunks_left == 0:
                return data
            chunk += 1

    def write_entry(self, param_id, value_bytes, timeout=0.5):
        self.purge(param_id)
        self.send(T_WRITE, [ADDR_FC, ADDR_RADIO, param_id] + list(value_bytes))
        return self.wait_frame(lambda t, p: t in (T_ENTRY, T_WRITE) and len(p) > 2 and p[2] == param_id, timeout)


def parse_entry(param_id, data):
    """decode an entry the way ELRS Lua does"""
    parent = data[0]
    ftype = data[1] & 0x7F
    hidden = bool(data[1] & 0x80)
    name, off = cstr(data, 2)
    e = {"id": param_id, "parent": parent, "type": TYPE_NAMES.get(ftype, ftype), "hidden": hidden, "name": name}
    if ftype == 9:
        options, off = cstr(data, off)
        e["options"] = options.split(";")
        e["value"] = data[off]
        e["shown"] = e["options"][e["value"]] if e["value"] < len(e["options"]) else "?"
    elif ftype == 8:
        value, vmin, vmax, vdef = struct.unpack(">iiii", data[off:off + 16])
        dec = data[off + 16]
        e["decimals"] = dec
        e["value"] = value / (10 ** dec)
        e["shown"] = "%g" % e["value"]
        e["min"], e["max"] = vmin / (10 ** dec), vmax / (10 ** dec)
    elif ftype == 12:
        e["shown"], off = cstr(data, off)
    elif ftype == 13:
        e["status"] = data[off]
        e["info"], off = cstr(data, off + 2)
        e["shown"] = "[cmd]"
    elif ftype == 11:
        e["shown"] = ">"
    return e


def load_device(client, field_count):
    """load entries 1..field_count; returns (entries dict, list of ids that never answered)"""
    entries = {}
    missing = []
    for pid in range(1, field_count + 1):
        data = client.read_entry(pid)
        if data is None:
            missing.append(pid)
            continue
        entries[pid] = parse_entry(pid, data)
    return entries, missing


def print_tree(entries, parent=None, depth=0, root_parent=0):
    parent = root_parent if parent is None else parent
    for pid in sorted(entries):
        e = entries[pid]
        if e["parent"] != parent or (depth > 0 and pid == parent):
            continue
        flag = " (hidden)" if e["hidden"] else ""
        print("  " * depth + "%3d %s: %s%s" % (pid, e["name"], e.get("shown", ""), flag))
        if e["type"] == "FOLDER":
            print_tree(entries, pid, depth + 1)
