#!/usr/bin/env python3
"""
AP_FLAKE8_CLEAN

Connect a real CRSF receiver to SITL, so a radio can fly SITL and use the
transmitter setup menu over a real ExpressLRS link.

The receiver reaches the PC through a serial port: a gyro board in
ArduPilot serial passthrough (SERIAL_PASS2 set to the receiver's SERIALn:
the board's USB then carries the raw receiver stream), or a receiver on a
USB-UART adapter. This script copies bytes both ways between that port
and SITL's CRSF port:

    sim_vehicle / arduplane ... --serial5=tcp:5765      (SERIAL5_PROTOCOL 23)
    python crsf_serial_bridge.py --port COM53 --tcp 127.0.0.1:5765

On a USB passthrough the PC's baud rate becomes the receiver port's baud
rate, so the default 420000 matches an ExpressLRS receiver at 420 kbaud.
Under WSL2, run this with Windows Python (WSL cannot open COM ports);
Windows reaches a SITL running in WSL at 127.0.0.1. The TCP side
reconnects when SITL restarts. Needs pyserial.
"""
import argparse
import socket
import sys
import threading
import time

import serial

FRAME_NAMES = {0x14: "link", 0x16: "rc", 0x02: "gps", 0x08: "battery", 0x1E: "attitude", 0x21: "mode",
               0x28: "ping", 0x29: "devinfo", 0x2B: "entry", 0x2C: "read", 0x2D: "write", 0x7F: "passthrough"}


class FrameCounter:
    """counts CRSF frame types in one direction, for the status line"""

    def __init__(self):
        self.buf = b""
        self.counts = {}
        self.bytes = 0

    def feed(self, data):
        self.bytes += len(data)
        self.buf += data
        while len(self.buf) >= 3:
            length = self.buf[1]
            if length < 2 or length > 62:
                self.buf = self.buf[1:]
                continue
            if len(self.buf) < length + 2:
                break
            ftype = self.buf[2]
            name = FRAME_NAMES.get(ftype, "0x%02x" % ftype)
            self.counts[name] = self.counts.get(name, 0) + 1
            self.buf = self.buf[length + 2:]

    def take(self):
        out = (self.bytes, self.counts)
        self.bytes, self.counts = 0, {}
        return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[1], formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--port", required=True, help="serial port of the receiver, e.g. COM53 or /dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=420000)
    ap.add_argument("--tcp", default="127.0.0.1:5765", help="SITL CRSF port, host:port")
    ap.add_argument("--stats", type=float, default=5.0, help="seconds between status lines, 0 for none")
    args = ap.parse_args()
    host, port = args.tcp.rsplit(":", 1)

    ser = serial.Serial(args.port, args.baud, timeout=0.005)
    print("serial %s open at %u baud" % (args.port, args.baud), flush=True)
    to_sitl, to_rx = FrameCounter(), FrameCounter()
    state = {"sock": None}
    lock = threading.Lock()

    def serial_to_tcp():
        while True:
            data = ser.read(512)
            if not data:
                continue
            with lock:
                to_sitl.feed(data)
                sock = state["sock"]
            if sock is not None:
                try:
                    sock.sendall(data)
                except OSError:
                    pass

    threading.Thread(target=serial_to_tcp, daemon=True).start()

    def stats_loop():
        while args.stats > 0:
            time.sleep(args.stats)
            with lock:
                (b1, c1), (b2, c2) = to_sitl.take(), to_rx.take()
                connected = state["sock"] is not None
            print("%s  rx->sitl %6u B %s  sitl->rx %6u B %s" % (
                "connected" if connected else "waiting  ", b1, dict(sorted(c1.items())), b2,
                dict(sorted(c2.items()))), flush=True)

    threading.Thread(target=stats_loop, daemon=True).start()

    while True:
        try:
            sock = socket.create_connection((host, int(port)), timeout=2)
        except OSError:
            time.sleep(1)
            continue
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.settimeout(None)
        print("connected to SITL at %s" % args.tcp, flush=True)
        with lock:
            state["sock"] = sock
        try:
            while True:
                data = sock.recv(4096)
                if not data:
                    break
                with lock:
                    to_rx.feed(data)
                ser.write(data)
        except OSError:
            pass
        with lock:
            state["sock"] = None
        sock.close()
        print("SITL disconnected, waiting for it", flush=True)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
