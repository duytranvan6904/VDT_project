import unittest
from simulation.control.touchdown_detector import TouchdownDetector, TouchdownParams, TouchdownResult


class TestTouchdownDetector(unittest.TestCase):
    def setUp(self):
        self.params = TouchdownParams(
            optical_height_threshold_m=0.40,
            descent_cmd_threshold_mps=-0.10,
            stoppage_vz_max_mps=0.08,
            stoppage_dz_dt_max_mps=0.05,
            confirmation_duration_s=0.35,
            altitude_ceiling_m=0.80,
        )
        self.detector = TouchdownDetector(self.params)

    def test_in_flight_no_touchdown(self):
        """While descending freely mid-air, touchdown must not trigger."""
        t = 0.0
        for _ in range(10):
            res = self.detector.step(
                actual_vz=-0.30,         # Actively descending at 0.3 m/s
                commanded_vz=-0.30,
                current_alt_z=1.50 - t * 0.30,
                optical_z=1.50 - t * 0.30,
                current_time=t,
            )
            t += 0.05
            self.assertFalse(res.touchdown_confirmed)
            self.assertFalse(res.kinematic_stopped)

    def test_touchdown_confirmed_with_landing_gear_and_sensor_drift(self):
        """Touchdown confirmed despite sensor drift (e.g. altitude reads 0.38m on ground)."""
        # Step 1: Descending towards pad
        res1 = self.detector.step(
            actual_vz=-0.20,
            commanded_vz=-0.20,
            current_alt_z=0.60,
            optical_z=0.55,
            current_time=0.0,
        )
        self.assertFalse(res1.touchdown_confirmed)

        # Step 2: Chân chạm sàn tại z_optical = 0.30m (chiều cao chân đáp),
        # sensor đo độ cao trôi thành 0.38m, vận tốc đứng bị triệt tiêu do mặt sàn giữ lại.
        t = 0.10
        # Cung cấp liên tục các frame dừng đứng trong 0.55s (> 0.35s confirmation duration)
        for _ in range(12):
            res = self.detector.step(
                actual_vz=0.01,          # Vận tốc đứng triệt tiêu (~0 m/s)
                commanded_vz=-0.25,      # Đang ép hạ cánh (-0.25 m/s)
                current_alt_z=0.38,      # Độ cao thế giới trôi ở 0.38m!
                optical_z=0.30,          # Khoảng cách quang học camera tới bãi = 0.30m
                current_time=t,
            )
            t += 0.05

        self.assertTrue(res.touchdown_confirmed)
        self.assertTrue(res.kinematic_stopped)
        self.assertTrue(res.optical_near)

    def test_optical_near_prevents_false_detection_in_hover(self):
        """Momentary zero vertical speed mid-air must not trigger touchdown."""
        t = 0.0
        for _ in range(10):
            res = self.detector.step(
                actual_vz=0.01,          # Vz = 0 (tạm thời lơ lửng)
                commanded_vz=-0.15,
                current_alt_z=1.20,      # Độ cao còn cao (1.2m)
                optical_z=1.15,          # Optical z > 0.40m
                current_time=t,
            )
            t += 0.05
            self.assertFalse(res.touchdown_confirmed)

    def test_latching_behavior(self):
        """Once confirmed, touchdown must remain latched."""
        # Force confirmed
        t = 0.0
        for _ in range(10):
            self.detector.step(
                actual_vz=0.0,
                commanded_vz=-0.20,
                current_alt_z=0.30,
                optical_z=0.28,
                current_time=t,
            )
            t += 0.05

        self.assertTrue(self.detector.is_latched)
        # Next frame
        res = self.detector.step(
            actual_vz=0.0,
            commanded_vz=0.0,
            current_alt_z=0.30,
            optical_z=0.28,
            current_time=1.0,
        )
        self.assertTrue(res.touchdown_confirmed)
        self.assertEqual(res.diagnostic_reason, "LATCHED_TOUCHDOWN")

    def test_px4_firmware_flag_override(self):
        """PX4 landed flag triggers touchdown confirmation."""
        res = self.detector.step(
            actual_vz=0.0,
            commanded_vz=-0.20,
            current_alt_z=0.35,
            optical_z=0.32,
            px4_landed=True,
            current_time=0.0,
        )
        # Next frame after confirmation duration
        res2 = self.detector.step(
            actual_vz=0.0,
            commanded_vz=-0.20,
            current_alt_z=0.35,
            optical_z=0.32,
            px4_landed=True,
            current_time=0.40,
        )
        self.assertTrue(res2.touchdown_confirmed)


if __name__ == '__main__':
    unittest.main()
