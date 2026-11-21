import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from bench_report import analyze_lines, has_faults, parse_sample


class BenchReportTests(unittest.TestCase):
    def test_valid_and_missing_echo(self):
        sample = parse_sample("arm=1 imu=1 pitch=-3.20 scan=1 look=left front=?0.0 left=25.0 right=?0.0")
        self.assertTrue(sample["armed"])
        self.assertIsNone(sample["arm_ready"])  # older telemetry remains parseable
        ready = parse_sample("arm=0 imu=1 pitch=0.1 arm_ready=1 scan=0 look=front front=75 left=?0.0 right=?0.0")
        self.assertTrue(ready["arm_ready"])
        self.assertEqual(sample["ranges"]["left"], 25.0)
        self.assertIsNone(sample["ranges"]["front"])

    def test_reject_malformed_and_nonfinite(self):
        self.assertIsNone(parse_sample("booting..."))
        self.assertIsNone(parse_sample("arm=0 imu=1 pitch=nan scan=0 look=front front=10 left=20 right=30"))
        self.assertIsNone(parse_sample("arm=2 imu=1 pitch=0 scan=0 look=front front=10 left=20 right=30"))
        sample = parse_sample("arm=0 imu=1 pitch=0 scan=0 look=front front=inf left=20 right=300")
        self.assertIsNone(sample["ranges"]["front"])
        self.assertIsNone(sample["ranges"]["right"])

    def test_summary_and_fault_detection(self):
        lines = [
            "arm=0 imu=1 pitch=0.5 scan=0 look=front front=150.0 left=?0.0 right=?0.0",
            "arm=1 imu=0 pitch=7.25 scan=1 look=right front=30.0 left=45.0 right=40.0",
            "DISARM: IMU communication lost",
            "random unparseable banner",
        ]
        summary = analyze_lines(lines)
        self.assertEqual(summary["samples"], 2)
        self.assertEqual(summary["armed_samples"], 1)
        self.assertEqual(summary["imu_fault_samples"], 1)
        self.assertEqual(summary["max_abs_pitch_deg"], 7.25)
        self.assertEqual(summary["valid_range_samples"], {"front": 2, "left": 1, "right": 1})
        self.assertEqual(summary["disarm_reasons"], {"IMU communication lost": 1})
        self.assertEqual(summary["arm_ready_samples"], 0)
        self.assertTrue(has_faults(summary))
        self.assertTrue(has_faults(analyze_lines([])))
        self.assertFalse(has_faults(analyze_lines(lines[:1])))

    def test_v05_control_timing_windows_and_missed_deadline(self):
        lines = [
            "arm=1 imu=1 pitch=2.00 arm_ready=1 scan=0 look=front "
            "front=100.0 left=?0.0 right=?0.0 t_ms=200 ctl_n=39 "
            "ctl_min_us=5000 ctl_max_us=5200 ctl_misses=0 "
            "rate_dps=4.20 cmd_l=0.100 cmd_r=-0.200",
            "arm=0 imu=1 pitch=0.50 arm_ready=0 scan=0 look=front "
            "front=80.0 left=?0.0 right=?0.0 t_ms=400 ctl_n=31 "
            "ctl_min_us=5001 ctl_max_us=35000 ctl_misses=1 "
            "rate_dps=-8.50 cmd_l=0.000 cmd_r=0.000",
        ]
        first = parse_sample(lines[0])
        self.assertEqual(first["timing"]["ctl_n"], 39)
        self.assertAlmostEqual(first["timing"]["cmd_r"], -0.2)
        summary = analyze_lines(lines)
        self.assertEqual(summary["timing_windows"], 2)
        self.assertEqual(summary["control_cycles_reported"], 70)
        self.assertEqual(summary["control_period_min_us"], 5000)
        self.assertEqual(summary["control_period_max_us"], 35000)
        self.assertEqual(summary["controller_deadline_misses_reported"], 1)
        self.assertAlmostEqual(summary["peak_abs_pitch_rate_dps"], 8.5)
        self.assertAlmostEqual(summary["peak_abs_motor_command"], 0.2)
        self.assertTrue(has_faults(summary))

    def test_v05_faulty_or_incomplete_timing_fields(self):
        stem = ("arm=0 imu=1 pitch=0.00 arm_ready=0 scan=0 look=front "
                "front=100 left=?0 right=?0 ")
        good = ("t_ms=100 ctl_n=10 ctl_min_us=5000 ctl_max_us=5500 "
                "ctl_misses=0 rate_dps=0.10 cmd_l=0 cmd_r=0")
        self.assertIsNotNone(parse_sample(stem + good))
        for bad in (
            good.replace("ctl_max_us=5500 ", ""),
            good.replace("ctl_n=10", "ctl_n=-10"),
            good.replace("ctl_n=10", "ctl_n=bogus"),
            good.replace("ctl_n=10", "ctl_n=4294967296"),
            good.replace("ctl_min_us=5000", "ctl_min_us=8000"),
            good.replace("rate_dps=0.10", "rate_dps=nan"),
            good.replace("cmd_l=0", "cmd_l=inf"),
            good.replace("ctl_n=10", "ctl_n=0"),
        ):
            self.assertIsNone(parse_sample(stem + bad), bad)
        legacy = parse_sample(stem.rstrip())
        self.assertIsNone(legacy["timing"])
        self.assertIsNone(analyze_lines([stem])["control_period_min_us"])
        self.assertFalse(has_faults(analyze_lines([stem])))


if __name__ == "__main__":
    unittest.main()
