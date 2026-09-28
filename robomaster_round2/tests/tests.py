"""
==============================================================================
Module: tests.py
Description: Unit test suite verifying all modular components:
             PID controller, angle error, motion vectors, A*, obstacles,
             walls, target sequencer, and driver state tracking.
==============================================================================
"""

import unittest
from robomaster_round2.config import get_motion_vector, TILE_SIZE_M
from robomaster_round2.robot.pid_controller import PIDController, ChassisHeadingPID, calculate_angle_error
from robomaster_round2.navigation.grid_map import GridMap
from robomaster_round2.navigation.pathfinder import PathFinder
from robomaster_round2.mission.target_sequence import TargetSequencer
from robomaster_round2.robot.chassis_driver import SimulatedChassisDriver


class TestRoboMasterModularNavigation(unittest.TestCase):

    def setUp(self):
        self.arena = GridMap(rows=7, cols=7)
        self.pathfinder = PathFinder(self.arena)
        self.sequencer = TargetSequencer(self.pathfinder)

    # --------------------------------------------------------------------------
    # 1. PID Controller Unit Tests
    # --------------------------------------------------------------------------
    def test_pid_controller_compute(self):
        """Test PIDController computation, clamping, and reset behavior."""
        pid = PIDController(kp=1.0, ki=0.5, kd=0.1, min_output=-10.0, max_output=10.0)
        # Iteration 1: setpoint=10, measurement=0, error=10, dt=1.0
        # P = 1.0 * 10 = 10
        # I = 0.5 * 10 * 1 = 5
        # D = 0.1 * (10 - 0) / 1 = 1
        # Total = 16 -> Clamped to max_output = 10.0
        output = pid.compute(setpoint=10.0, measurement=0.0, dt=1.0)
        self.assertEqual(output, 10.0)

        # Test reset
        pid.reset()
        self.assertEqual(pid.integral, 0.0)
        self.assertEqual(pid.prev_error, 0.0)

    def test_calculate_angle_error(self):
        """Test shortest angular difference and [-180, 180] angle wrapping."""
        # Aligned angles: error = 0
        self.assertEqual(calculate_angle_error(0.0, 0.0), 0.0)
        # Setpoint 10 deg, actual 0 deg -> error = 10 deg
        self.assertEqual(calculate_angle_error(10.0, 0.0), 10.0)
        # Setpoint 0 deg, actual 350 deg (-10 deg) -> error = +10 deg
        self.assertAlmostEqual(calculate_angle_error(0.0, 350.0), 10.0)
        # Setpoint 0 deg, actual 10 deg -> error = -10 deg
        self.assertAlmostEqual(calculate_angle_error(0.0, 10.0), -10.0)

    def test_heading_pid_lock(self):
        """Test ChassisHeadingPID yaw correction speed to restore 0 deg heading."""
        heading_pid = ChassisHeadingPID(kp=1.0, ki=0.0, kd=0.0, max_yaw_dps=30.0)
        # Robot deviated clockwise to 10 deg -> must produce negative yaw correction speed
        z_speed = heading_pid.compute_yaw_speed(current_yaw_deg=10.0, dt=0.1)
        self.assertLess(z_speed, 0.0)

    # --------------------------------------------------------------------------
    # 2. Kinematics & Motion Vector Conversion Tests
    # --------------------------------------------------------------------------
    def test_motion_vector_conversion(self):
        """Test conversion of grid steps (dr, dc) into relative 0.60m step distances."""
        dx, dy, action = get_motion_vector(1, 0)
        self.assertEqual(action, "FORWARD")
        self.assertAlmostEqual(dx, TILE_SIZE_M)
        self.assertAlmostEqual(dy, 0.0)

        dx, dy, action = get_motion_vector(-1, 0)
        self.assertEqual(action, "BACKWARD")
        self.assertAlmostEqual(dx, -TILE_SIZE_M)
        self.assertAlmostEqual(dy, 0.0)

        dx, dy, action = get_motion_vector(0, -1)
        self.assertEqual(action, "STRAFE_LEFT")
        self.assertAlmostEqual(dx, 0.0)
        self.assertAlmostEqual(dy, TILE_SIZE_M)

        dx, dy, action = get_motion_vector(0, 1)
        self.assertEqual(action, "STRAFE_RIGHT")
        self.assertAlmostEqual(dx, 0.0)
        self.assertAlmostEqual(dy, -TILE_SIZE_M)

    # --------------------------------------------------------------------------
    # 3. A* Search, Obstacles, and Barrier Wall Tests
    # --------------------------------------------------------------------------
    def test_astar_obstacle_avoidance(self):
        """Test that A* detours around blocked obstacle cells."""
        self.arena.add_blocked_cell(0, 1)
        path = self.pathfinder.find_path((0, 0), (0, 2))
        self.assertIsNotNone(path)
        self.assertNotIn((0, 1), path)
        self.assertEqual(path[0], (0, 0))
        self.assertEqual(path[-1], (0, 2))

    def test_astar_wall_avoidance(self):
        """Test that A* routes around edge walls between cells."""
        self.arena.add_wall((2, 2), (2, 3))
        self.assertFalse(self.arena.can_move((2, 2), (2, 3)))
        path = self.pathfinder.find_path((2, 2), (2, 3))
        self.assertIsNotNone(path)
        # Ensure path does not step directly between (2, 2) and (2, 3)
        for i in range(len(path) - 1):
            pair = frozenset([path[i], path[i + 1]])
            self.assertNotEqual(pair, frozenset([(2, 2), (2, 3)]))

    # --------------------------------------------------------------------------
    # 4. Target Sequencer Unit Tests
    # --------------------------------------------------------------------------
    def test_target_sequencer(self):
        """Test Greedy Nearest Neighbor target ordering based on shortest distance."""
        start = (0, 0)
        targets = [(0, 5), (0, 1), (0, 3)]
        ordered, steps = self.sequencer.sequence_targets(start, targets)
        self.assertEqual(ordered, [(0, 1), (0, 3), (0, 5)])
        self.assertEqual(steps, 5.0)

    # --------------------------------------------------------------------------
    # 5. Simulated Driver Unit Tests
    # --------------------------------------------------------------------------
    def test_simulated_driver(self):
        """Test SimulatedChassisDriver position updates and distance telemetry."""
        driver = SimulatedChassisDriver(start_pos=(0, 0))
        driver.connect()
        self.assertTrue(driver.is_connected)
        driver.move_step(1, 0)
        self.assertEqual(driver.position, (1, 0))
        self.assertAlmostEqual(driver.total_distance_m, TILE_SIZE_M)
        self.assertEqual(driver.total_steps, 1)
        driver.disconnect()
        self.assertFalse(driver.is_connected)


if __name__ == "__main__":
    unittest.main()
