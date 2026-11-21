#!/usr/bin/env python3
"""Summarize GyroGuard serial telemetry for supported bench commissioning.

This is an offline diagnostic, NOT evidence of closed-loop or hardware safety.
Capture serial logs first, e.g. pio device monitor -b 115200 > bench.log.
"""
import argparse
import json
import math
import sys
from collections import Counter
from pathlib import Path

REQUIRED = {"arm", "imu", "pitch", "scan", "look", "front", "left", "right"}


def parse_sample(line):
    fields = {}
    for token in line.split():
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        fields[key] = value
    if not REQUIRED.issubset(fields):
        return None
    try:
        if fields["arm"] not in ("0", "1") or fields["imu"] not in ("0", "1"):
            return None
        if fields["scan"] not in ("0", "1"):
            return None
        if fields["look"] not in ("front", "left", "right"):
            return None
        pitch = float(fields["pitch"])
        if not math.isfinite(pitch):
            return None
        ranges = {}
        for direction in ("front", "left", "right"):
            value = fields[direction]
            if value.startswith("?"):
                ranges[direction] = None
            else:
                cm = float(value)
                ranges[direction] = cm if math.isfinite(cm) and 2 <= cm <= 260 else None
    except ValueError:
        return None
    return {
        "armed": fields["arm"] == "1",
        "imu_ok": fields["imu"] == "1",
        "scanning": fields["scan"] == "1",
        "look": fields["look"],
        "pitch": pitch,
        "ranges": ranges,
    }


def analyze_lines(lines):
    result = {
        "samples": 0,
        "armed_samples": 0,
        "imu_fault_samples": 0,
        "scan_samples": 0,
        "max_abs_pitch_deg": 0.0,
        "valid_range_samples": {key: 0 for key in ("front", "left", "right")},
        "disarm_reasons": {},
    }
    reasons = Counter()
    for raw_line in lines:
        line = raw_line.strip()
        if "DISARM: " in line:
            reasons[line.split("DISARM: ", 1)[1]] += 1
        sample = parse_sample(line)
        if sample is None:
            continue
        result["samples"] += 1
        result["armed_samples"] += int(sample["armed"])
        result["imu_fault_samples"] += int(not sample["imu_ok"])
        result["scan_samples"] += int(sample["scanning"])
        result["max_abs_pitch_deg"] = max(result["max_abs_pitch_deg"], abs(sample["pitch"]))
        for direction, value in sample["ranges"].items():
            result["valid_range_samples"][direction] += int(value is not None)
    result["disarm_reasons"] = dict(sorted(reasons.items()))
    return result


def has_faults(summary):
    return summary["samples"] == 0 or summary["imu_fault_samples"] > 0 or bool(summary["disarm_reasons"])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", nargs="?", default="-", help="Serial log path, or - for standard input")
    parser.add_argument("--json", action="store_true", help="Machine-readable report")
    parser.add_argument("--strict", action="store_true", help="Exit 2 on missing telemetry, IMU fault or disarm")
    args = parser.parse_args(argv)
    if args.log == "-":
        summary = analyze_lines(sys.stdin)
    else:
        try:
            with Path(args.log).open(encoding="utf-8", errors="replace") as stream:
                summary = analyze_lines(stream)
        except OSError as exc:
            parser.error(str(exc))
    if args.json:
        print(json.dumps(summary, indent=2, sort_keys=True))
    else:
        print("GyroGuard bench telemetry (diagnostic only; hardware not certified)")
        print(f"Telemetry samples: {summary['samples']} | armed: {summary['armed_samples']} | scanning: {summary['scan_samples']}")
        print(f"IMU fault samples: {summary['imu_fault_samples']} | max |pitch|: {summary['max_abs_pitch_deg']:.2f} deg")
        print("Valid range samples: " + ", ".join(f"{k}={v}" for k, v in summary["valid_range_samples"].items()))
        print(f"Disarm reasons: {summary['disarm_reasons'] or 'none recorded'}")
    return 2 if args.strict and has_faults(summary) else 0


if __name__ == "__main__":
    sys.exit(main())
