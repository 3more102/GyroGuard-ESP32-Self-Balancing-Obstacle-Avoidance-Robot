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


if __name__ == "__main__":
    unittest.main()
