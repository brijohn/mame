#!/usr/bin/env python3
# TCP <-> serial bridge for MAME NABU instances.
#
# Owns a serial device and exposes it as a listening TCP socket
# that a MAME instance can connect to:
#
#   ./nabu-serial-bridge.py /dev/ttyUSB0 --baud 111860
#   ./nabu nabuna ... -npc null_modem -bitb socket.127.0.0.1:5816
#
import argparse
import collections
import fcntl
import functools
import os
import select
import socket
import struct
import sys
import termios
import time
import tty

print = functools.partial(print, flush=True)

STANDARD_BAUD = {
    300: termios.B300, 1200: termios.B1200, 2400: termios.B2400,
    4800: termios.B4800, 9600: termios.B9600, 19200: termios.B19200,
    38400: termios.B38400, 57600: termios.B57600, 115200: termios.B115200,
    230400: termios.B230400,
}

# Linux struct termios2 ioctls
TCGETS2 = 0x802C542A
TCSETS2 = 0x402C542B
BOTHER = 0o010000
CBAUD = 0o010017
TERMIOS2_FMT = "<IIIIB19sII"


def open_serial(path, baud, stopbits):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)

    tty.setraw(fd)
    attrs = termios.tcgetattr(fd)
    attrs[2] |= termios.CLOCAL | termios.CREAD
    attrs[2] &= ~termios.CRTSCTS
    if stopbits == 2:
        attrs[2] |= termios.CSTOPB
    else:
        attrs[2] &= ~termios.CSTOPB
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    if baud in STANDARD_BAUD:
        attrs[4] = attrs[5] = STANDARD_BAUD[baud]
    termios.tcsetattr(fd, termios.TCSANOW, attrs)

    if baud not in STANDARD_BAUD:
        buf = fcntl.ioctl(fd, TCGETS2, bytes(struct.calcsize(TERMIOS2_FMT)))
        iflag, oflag, cflag, lflag, line, cc, _, _ = struct.unpack(TERMIOS2_FMT, buf)
        cflag = (cflag & ~CBAUD) | BOTHER
        buf = struct.pack(TERMIOS2_FMT, iflag, oflag, cflag, lflag, line, cc, baud, baud)
        fcntl.ioctl(fd, TCSETS2, buf)

    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def serial_write_all(fd, data):
    while data:
        try:
            n = os.write(fd, data)
            data = data[n:]
        except BlockingIOError:
            select.select([], [fd], [], 0.05)
        except OSError as e:
            print(f"serial device error: {e}")
            return False
    return True


def main():
    ap = argparse.ArgumentParser(description="bridge a serial device to a TCP socket for MAME -bitb socket. use")
    ap.add_argument("device", help="serial device, e.g. /dev/ttyUSB0")
    ap.add_argument("-p", "--port", type=int, default=5816, help="TCP port to listen on (default 5816)")
    ap.add_argument("-b", "--baud", type=int, default=111860, help="baud rate (default 111860; non-standard rates ok on Linux)")
    ap.add_argument("--stop", type=int, choices=(1, 2), default=1, help="stop bits (default 1)")
    ap.add_argument("--pace-ms", type=float, default=2.0, help="inter-byte gap for serial->TCP bytes, 0 disables (default 2)")
    ap.add_argument("--log", metavar="FILE", help="append a timestamped hex dump of both directions to FILE")
    args = ap.parse_args()

    logf = open(args.log, "a", buffering=1) if args.log else None
    t0 = time.monotonic()

    def log(text):
        if logf:
            logf.write(f"{time.monotonic() - t0:10.6f}  {text}\n")

    ser = open_serial(args.device, args.baud, args.stop)
    print(f"serial {args.device}: {args.baud} baud, 8N{args.stop}, raw")

    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind(("127.0.0.1", args.port))
    listener.listen(1)
    print(f"listening on 127.0.0.1:{args.port} (ctrl-C to stop)")

    conn = None
    pace = args.pace_ms / 1000.0
    txq = collections.deque()
    last_release = 0.0

    while True:
        rlist = [listener, ser] + ([conn] if conn else [])
        timeout = pace if (txq and pace > 0) else None
        readable, _, _ = select.select(rlist, [], [], timeout)

        now = time.monotonic()
        for s in readable:
            if s is listener:
                newconn, addr = listener.accept()
                newconn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                if conn is not None:
                    conn.close()
                conn = newconn
                txq.clear()
                print("client connected")
                log("* client connected")
            elif s is ser:
                try:
                    data = os.read(ser, 4096)
                except OSError as e:
                    print(f"serial device error: {e}")
                    return 1
                if data:
                    log("ser> " + data.hex(" ") + ("" if conn else "  (no client)"))
                if conn is None:
                    continue
                if pace > 0:
                    txq.extend(data)
                else:
                    try:
                        conn.sendall(data)
                    except OSError:
                        pass
            else:
                try:
                    data = s.recv(4096)
                except OSError:
                    data = b""
                if not data:
                    s.close()
                    conn = None
                    txq.clear()
                    print("client disconnected")
                    log("* client disconnected")
                    continue
                log("tcp> " + data.hex(" "))
                if not serial_write_all(ser, data):
                    return 1

        while txq and conn is not None and (time.monotonic() - last_release) >= pace:
            try:
                conn.sendall(bytes([txq.popleft()]))
            except OSError:
                break
            last_release = time.monotonic()


try:
    sys.exit(main())
except KeyboardInterrupt:
    print()
    sys.exit(0)
