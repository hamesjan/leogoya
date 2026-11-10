#!/usr/bin/env python3
"""Talk to the STM32 controller over its serial port.

    python3 scripts/send.py                       interactive: type commands, see replies
    python3 scripts/send.py build/sim/out/painting.gcode   stream a file, one line per "ok"
    python3 scripts/send.py -p /dev/cu.usbmodem1103 ...

Needs pyserial:  python3 -m pip install pyserial
Protocol: every non-empty, non-comment line gets info lines then exactly one
final "ok" or "error: ..." (see firmware/src/cmd.h).
"""
import argparse
import glob
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial missing: python3 -m pip install pyserial")


def find_port():
    ports = sorted(glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/ttyACM*"))
    if not ports:
        sys.exit("no Nucleo serial port found; pass -p PORT")
    return ports[0]


def command(ser, line, quiet=False):
    """Send one line, print info lines, return (ok, final_line)."""
    ser.write((line + "\n").encode())
    while True:
        raw = ser.readline()
        if not raw:
            return False, "timeout"
        reply = raw.decode(errors="replace").strip()
        if reply == "ok":
            return True, reply
        if reply.startswith("error"):
            return False, reply
        if reply and not quiet:
            print(reply)


def strip(line):
    return line.split(";", 1)[0].strip()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file", nargs="?", help="G-code / command file to stream")
    ap.add_argument("-p", "--port", help="serial port (default: first /dev/cu.usbmodem*)")
    ap.add_argument("-b", "--baud", type=int, default=115200)
    ap.add_argument("--keep-going", action="store_true", help="don't stop on the first error")
    args = ap.parse_args()

    port = args.port or find_port()
    with serial.Serial(port, args.baud, timeout=5) as ser:
        time.sleep(0.2)
        ser.reset_input_buffer()
        print(f"connected to {port}")

        if not args.file:
            print("type commands (HELP for a list), Ctrl-D to quit")
            for line in sys.stdin:
                if strip(line):
                    ok, final = command(ser, strip(line))
                    print(final)
            return

        lines = [strip(l) for l in open(args.file)]
        lines = [l for l in lines if l]
        t0 = time.time()
        for i, line in enumerate(lines, 1):
            ok, final = command(ser, line, quiet=True)
            if not ok:
                print(f"\nline {i}: {line!r} -> {final}")
                if not args.keep_going:
                    sys.exit(1)
            if i % 200 == 0 or i == len(lines):
                rate = i / max(time.time() - t0, 1e-6)
                print(f"\r{i}/{len(lines)} lines  {rate:.0f} lines/s", end="", flush=True)
        print()


if __name__ == "__main__":
    main()
