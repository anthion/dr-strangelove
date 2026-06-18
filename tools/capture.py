#!/usr/bin/env python3
"""
Dr. Strangelove Telemetry Capture Script
Captures CSV telemetry from the Dr. Strangelove laser alignment controller
over USB serial and writes it to a timestamped file.

Usage:
    python3 capture.py              # auto-detect Teensy
    python3 capture.py --port /dev/cu.usbmodem117996301
    python3 capture.py --port COM3  # Windows

Press Ctrl+C to stop capture.
"""

import argparse
import csv
import signal
import sys
from datetime import datetime, timezone
from pathlib import Path

import serial
import serial.tools.list_ports

# Teensy 4.0 USB serial VID/PID (PJRC)
TEENSY_VID = 0x16C0
TEENSY_PID = 0x0483

BAUD_RATE = 115200

# micros() rolls over at 2^32 microseconds (~71.6 minutes)
MICROS_ROLLOVER = 2 ** 32


def find_teensy_port():
    """Scan serial ports and return the device path for a connected Teensy."""
    matches = [
        p for p in serial.tools.list_ports.comports()
        if p.vid == TEENSY_VID and p.pid == TEENSY_PID
    ]
    if len(matches) == 1:
        return matches[0].device
    elif len(matches) == 0:
        print("ERROR: No Teensy found (VID=0x16C0, PID=0x0483).")
        print("Available ports:")
        for p in serial.tools.list_ports.comports():
            print(f"  {p.device}  VID={hex(p.vid) if p.vid else None}  "
                  f"PID={hex(p.pid) if p.pid else None}  {p.description}")
        print("\nSpecify a port manually with --port.")
        sys.exit(1)
    else:
        print("ERROR: Multiple Teensy devices found:")
        for p in matches:
            print(f"  {p.device}")
        print("Specify a port manually with --port.")
        sys.exit(1)


def make_output_filename():
    """Generate a timestamped output filename."""
    now = datetime.now()
    return f"strangelove_{now.strftime('%Y%m%d_%H%M%S')}.csv"


def main():
    parser = argparse.ArgumentParser(description="Dr. Strangelove telemetry capture")
    parser.add_argument(
        "--port", type=str, default=None,
        help="Serial port to use (default: auto-detect Teensy by VID/PID)"
    )
    parser.add_argument(
        "--output", type=str, default=None,
        help="Output file path (default: auto-named strangelove_YYYYMMDD_HHMMSS.csv)"
    )
    args = parser.parse_args()

    port = args.port if args.port else find_teensy_port()
    output_path = Path(args.output) if args.output else Path(make_output_filename())

    print(f"Port:   {port}")
    print(f"Output: {output_path}")
    print("Press Ctrl+C to stop.\n")

    # Capture wall-clock time at session start (both UTC and local)
    session_start_utc = datetime.now(timezone.utc)
    session_start_local = datetime.now().astimezone()

    # Rollover tracking
    last_timestamp_us = None
    rollover_offset = 0
    first_elapsed_us = None
    sample_count = 0
    firmware_header_lines = []
    header_written = False

    def handle_sigint(sig, frame):
        print(f"\nCapture stopped. {sample_count} samples written to {output_path}")
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_sigint)

    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=None)
    except serial.SerialException as e:
        print(f"ERROR: Could not open port {port}: {e}")
        sys.exit(1)

    with open(output_path, "w", newline="") as outfile:
        writer = None  # will be initialised once we see the data header line

        print("Waiting for telemetry stream... (toggle telemetry on the unit now)")
        
        for raw_line in ser:
            try:
                line = raw_line.decode("utf-8", errors="replace").rstrip()
            except Exception:
                continue

            # --- Firmware comment/header lines (start with #) ---
            if line.startswith("#"):
                firmware_header_lines.append(line)
                continue

            # --- First non-comment line: write the full header block then open CSV ---
            if not header_written:
                # Write PC-side session metadata
                outfile.write("# Dr. Strangelove telemetry capture\n")
                outfile.write(f"# capture_start_utc={session_start_utc.strftime('%Y-%m-%dT%H:%M:%S.%f')[:-3]}Z\n")
                outfile.write(f"# capture_start_local={session_start_local.strftime('%Y-%m-%d %H:%M:%S %Z')}\n")
                outfile.write(f"# port={port}\n")
                outfile.write(f"# output_file={output_path.name}\n")

                # Pass through all firmware header lines
                for fline in firmware_header_lines:
                    outfile.write(fline + "\n")

                # Extra column documentation
                outfile.write("# t_sec=elapsed seconds from first sample (rollover-corrected)\n")

                header_written = True

            # --- Parse CSV data line ---
            # Expected columns from firmware:
            # timestamp_us,sample,q1,q2,q3,q4,errorX,errorY,totalPower,cmd_steps_x,cmd_steps_y

            parts = line.split(",")

            # Skip the firmware's own column header line if present
            if parts[0].strip() == "timestamp_us":
                # Write augmented column header to file
                outfile.write(
                    "timestamp_us,sample,q1,q2,q3,q4,errorX,errorY,"
                    "totalPower,cmd_steps_x,cmd_steps_y,t_sec\n"
                )
                outfile.flush()
                writer = True  # flag that we've written the header
                continue

            if len(parts) < 11:
                # Malformed line — skip silently
                continue

            try:
                timestamp_us = int(parts[0])
            except ValueError:
                continue

            # --- Rollover correction ---
            if last_timestamp_us is not None and timestamp_us < last_timestamp_us:
                rollover_offset += MICROS_ROLLOVER
            last_timestamp_us = timestamp_us

            elapsed_us = timestamp_us + rollover_offset
            if first_elapsed_us is None:
                first_elapsed_us = elapsed_us

            t_sec = (elapsed_us - first_elapsed_us) / 1_000_000.0

            # Write augmented data line
            outfile.write(f"{line},{t_sec:.6f}\n")

            sample_count += 1

            # Flush periodically so data isn't lost if process is killed
            if sample_count % 100 == 0:
                outfile.flush()
                print(f"  {sample_count} samples  t={t_sec:.1f}s", end="\r")


if __name__ == "__main__":
    main()
