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
# All timing fields are optional *as a group* for compatibility with V0.4 logs.
TIMING_KEYS = {
    "t_ms", "ctl_n", "ctl_min_us", "ctl_max_us", "ctl_misses",
    "rate_dps", "cmd_l", "cmd_r",
}
UINT32_MAX = 2**32 - 1


def uint32_field(fields, key):
    raw = fields[key]
    if not raw.isascii() or not raw.isdecimal():
        raise ValueError(f"{key} must be a nonnegative decimal integer")
    result = int(raw)
    if result > UINT32_MAX:
        raise ValueError(f"{key} outside uint32 range")
    return result




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
        if fields.get("arm_ready") not in (None, "0", "1"):
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
        timing = None
        if TIMING_KEYS.intersection(fields):
            if not TIMING_KEYS.issubset(fields):
                return None
            timing = {
                key: uint32_field(fields, key)
                for key in ("t_ms", "ctl_n", "ctl_min_us",
                            "ctl_max_us", "ctl_misses")
            }
            for key in ("rate_dps", "cmd_l", "cmd_r"):
                timing[key] = float(fields[key])
                if not math.isfinite(timing[key]):
                    return None
            count = timing["ctl_n"]
            minimum = timing["ctl_min_us"]
            maximum = timing["ctl_max_us"]
            if (count == 0 and (minimum != 0 or maximum != 0)) or (
                count > 0 and (minimum == 0 or minimum > maximum)
            ):
                return None
    except ValueError:
        return None
    return {
        "armed": fields["arm"] == "1",
        "arm_ready": None if "arm_ready" not in fields else fields["arm_ready"] == "1",
        "imu_ok": fields["imu"] == "1",
        "scanning": fields["scan"] == "1",
        "look": fields["look"],
        "pitch": pitch,
        "ranges": ranges,
        "timing": timing,
    }


def analyze_lines(lines):
    result = {
        "samples": 0,
        "armed_samples": 0,
        "arm_ready_samples": 0,
        "imu_fault_samples": 0,
        "scan_samples": 0,
        "max_abs_pitch_deg": 0.0,
        "valid_range_samples": {key: 0 for key in ("front", "left", "right")},
        "disarm_reasons": {},
        "timing_windows": 0,
        "control_cycles_reported": 0,
        "control_period_min_us": None,
        "control_period_max_us": None,
        "controller_deadline_misses_reported": 0,
        "peak_abs_pitch_rate_dps": None,
        "peak_abs_motor_command": None,
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
        result["arm_ready_samples"] += int(sample["arm_ready"] is True)
        result["imu_fault_samples"] += int(not sample["imu_ok"])
        result["scan_samples"] += int(sample["scanning"])
        result["max_abs_pitch_deg"] = max(result["max_abs_pitch_deg"], abs(sample["pitch"]))
        for direction, value in sample["ranges"].items():
            result["valid_range_samples"][direction] += int(value is not None)
        timing = sample["timing"]
        if timing is not None:
            result["timing_windows"] += 1
            result["control_cycles_reported"] += timing["ctl_n"]
            result["controller_deadline_misses_reported"] = max(
                result["controller_deadline_misses_reported"], timing["ctl_misses"]
            )
            if timing["ctl_n"]:
                lo = result["control_period_min_us"]
                hi = result["control_period_max_us"]
                result["control_period_min_us"] = (
                    timing["ctl_min_us"] if lo is None else min(lo, timing["ctl_min_us"])
                )
                result["control_period_max_us"] = (
                    timing["ctl_max_us"] if hi is None else max(hi, timing["ctl_max_us"])
                )
            rate = abs(timing["rate_dps"])
            motor = max(abs(timing["cmd_l"]), abs(timing["cmd_r"]))
            old_rate = result["peak_abs_pitch_rate_dps"]
            old_motor = result["peak_abs_motor_command"]
            result["peak_abs_pitch_rate_dps"] = rate if old_rate is None else max(old_rate, rate)
            result["peak_abs_motor_command"] = motor if old_motor is None else max(old_motor, motor)
    result["disarm_reasons"] = dict(sorted(reasons.items()))
    return result


def has_faults(summary):
    return (
        summary["samples"] == 0
        or summary["imu_fault_samples"] > 0
        or bool(summary["disarm_reasons"])
        or summary["controller_deadline_misses_reported"] > 0
    )


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", nargs="?", default="-", help="Serial log path, or - for standard input")
    parser.add_argument("--json", action="store_true", help="Machine-readable report")
    parser.add_argument("--strict", action="store_true", help="Exit 2 on missing telemetry, IMU fault, disarm or reported control deadline miss")
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
        print(f"Telemetry samples: {summary['samples']} | armed: {summary['armed_samples']} | arm-ready: {summary['arm_ready_samples']} | scanning: {summary['scan_samples']}")
        print(f"IMU fault samples: {summary['imu_fault_samples']} | max |pitch|: {summary['max_abs_pitch_deg']:.2f} deg")
        print("Valid range samples: " + ", ".join(f"{k}={v}" for k, v in summary["valid_range_samples"].items()))
        print(f"Disarm reasons: {summary['disarm_reasons'] or 'none recorded'}")
        if summary["timing_windows"]:
            print(
                "Control scheduling: "
                f"windows={summary['timing_windows']}, "
                f"cycles={summary['control_cycles_reported']}, "
                f"min={summary['control_period_min_us']} us, "
                f"max={summary['control_period_max_us']} us, "
                f"deadline misses={summary['controller_deadline_misses_reported']}"
            )
            print(
                f"Max sampled |pitch rate|: {summary['peak_abs_pitch_rate_dps']:.2f} deg/s; "
                f"max sampled |motor command|: {summary['peak_abs_motor_command']:.3f}"
            )
        else:
            print("Controller timing and command telemetry: unavailable (legacy log)")
    return 2 if args.strict and has_faults(summary) else 0


if __name__ == "__main__":
    sys.exit(main())
