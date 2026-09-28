"""Contracts between exploration and hardware, without a robot connection."""
import unittest
import sys
from pathlib import Path
from itertools import count
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock, patch

from robomaster_round2.Slam.main import main
from robomaster_round2.Slam.setting import maze_setting as settings
from robomaster_round2.Slam.workflow import ExplorationWorkflow


class TestWorkflow(unittest.TestCase):
    def setUp(self):
        self.scanner = Mock()
        self.tracker = Mock()
        self.tracker.read.return_value = (0, 0, 12)
        self.mover = Mock(return_value=True)
        self.mover.reference_yaw = 5
        self.alignment = Mock()
        self.workflow = ExplorationWorkflow(
            self.scanner, self.tracker, self.mover, self.alignment)
        self.walls = dict(N=True, E=False, S=False, W=True)

    def test_scan_does_not_adjust_chassis(self):
        self.assertIs(self.workflow.scan(), self.scanner.scan.return_value)
        self.mover.assert_not_called()
        self.scanner.scan.assert_called_once()

    def test_skipped_alignment_keeps_initial_scan(self):
        self.alignment.align_in_cell.return_value = False
        self.assertIs(self.workflow.correct((1, 2), self.walls), self.walls)
        self.scanner.invalidate_cache.assert_not_called()
        self.scanner.scan.assert_not_called()
        self.assertEqual(self.mover.reference_yaw, 5)
        self.tracker.read.assert_not_called()

    def test_success_updates_heading_only_with_left_wall(self):
        self.alignment.align_in_cell.return_value = True
        self.workflow.correct((1, 2), self.walls)
        self.assertEqual(self.mover.reference_yaw, 12)
        self.mover.reference_yaw = 5
        self.workflow.correct((1, 2), dict(self.walls, W=False))
        self.assertEqual(self.mover.reference_yaw, 5)

    def test_disabled_alignment_passes_no_callback(self):
        workflow = ExplorationWorkflow(self.scanner, self.tracker, self.mover)
        session = Mock()
        workflow.run(session, 2)
        self.assertEqual(session.max_steps, 2)
        session.explore.assert_called_once_with(
            workflow.scan, self.mover, None, workflow.verify_move)

    def test_callback_uses_current_cell_after_move(self):
        session = SimpleNamespace(cell=(1, 2))
        def explore(scan, move, correct, verify_move):
            session.cell = (2, 2)
            correct(self.walls)
        session.explore = explore
        self.workflow.run(session)
        self.alignment.align_in_cell.assert_called_once_with((2, 2), self.walls, 'N')

    def test_rechecks_planned_direction_after_pose_change(self):
        self.workflow.scan_pose = (0, 0, 12)
        self.tracker.read.return_value = (0.03, 0, 15)
        self.scanner.verify_direction.return_value = False
        self.assertTrue(self.workflow.verify_move('E', (2, 0), (2, 1)))
        self.scanner.verify_direction.assert_called_once_with('E')

    def test_stable_pose_uses_original_scan(self):
        self.workflow.scan_pose = (0, 0, 12)
        self.tracker.read.return_value = (0.005, 0, 12.5)
        self.assertTrue(self.workflow.verify_move('E', (2, 0), (2, 1)))
        self.scanner.verify_direction.assert_not_called()

    def test_rechecked_wall_rejects_move(self):
        self.workflow.scan_pose = (0, 0, 12)
        self.tracker.read.return_value = (0.03, 0, 15)
        self.scanner.verify_direction.return_value = True
        self.assertFalse(self.workflow.verify_move('E', (2, 0), (2, 1)))

    def test_recheck_refuses_if_gimbal_turns_chassis(self):
        self.workflow.scan_pose = (0, 0, 12)
        self.tracker.read.side_effect = [(0.03, 0, 15), (0.03, 0, 20)]
        self.scanner.verify_direction.return_value = False
        with self.assertRaisesRegex(RuntimeError, 'during ToF direction verification'):
            self.workflow.verify_move('E', (2, 0), (2, 1))

    def test_cli_dispatches_without_hardware(self):
        with patch('robomaster_round2.Slam.main.run_real') as run:
            main(['--mode', 'real', '--max-steps', '1', '--output', 'test.json'])
        run.assert_called_once_with('test.json', False, 1)

    def test_cli_alignment_dispatches_without_exploration(self):
        with patch('robomaster_round2.Slam.main.run_real') as run:
            main(['--mode', 'real', '--alignment'])
        run.assert_called_once_with('maps/maze_map.json', alignment_only=True)

    def test_cli_sharp_check_dispatches_without_exploration(self):
        with patch('robomaster_round2.Slam.main.run_real') as run:
            main(['--mode', 'real', '--sharp-check'])
        run.assert_called_once_with('maps/maze_map.json', sharp_check=True)

    def test_live_map_window_uses_selected_file_without_blocking(self):
        from robomaster_round2.Slam.runtime import launch_map_window
        with patch.dict('os.environ', {'DISPLAY': ':0'}, clear=True), patch(
                'robomaster_round2.Slam.runtime.subprocess.Popen') as popen:
            launch_map_window('maps/custom.json')
        args, kwargs = popen.call_args
        self.assertEqual(args[0][:3], [sys.executable, '-m',
                                        'robomaster_round2.Slam.map_gui'])
        self.assertEqual(args[0][3], str(Path('maps/custom.json').resolve()))
        self.assertTrue(kwargs['start_new_session'])

    def test_live_map_window_skips_when_no_display(self):
        from robomaster_round2.Slam.runtime import launch_map_window
        with patch.dict('os.environ', {}, clear=True), patch(
                'robomaster_round2.Slam.runtime.subprocess.Popen') as popen:
            launch_map_window('maps/custom.json')
        popen.assert_not_called()

    def test_alignment_only_scans_once_and_cleans_up(self):
        from robomaster_round2.Slam.runtime import run_real
        fake_sdk = ModuleType('robomaster')
        ep = Mock()
        fake_sdk.robot = SimpleNamespace(Robot=Mock(return_value=ep), FREE='free')
        walls = dict(N=True, E=False, S=True, W=True)
        with patch.dict(sys.modules, {'robomaster': fake_sdk}), patch(
                'robomaster_round2.Slam.sensors.GimbalToFScanner') as scanner_class, patch(
                'robomaster_round2.Slam.correction.RoboMasterAlignment') as alignment_class, patch(
                'robomaster_round2.Slam.sharp_sensor.sharp_sensor') as sharp_class, patch(
                'robomaster_round2.Slam.pose_tracker.ChassisPoseTracker') as tracker_class, patch(
                'robomaster_round2.Slam.movement.stop_chassis') as stop:
            scanner_class.return_value.scan.return_value = walls
            alignment_class.return_value.align_in_cell.return_value = True
            self.assertTrue(run_real('unused.json', alignment_only=True))
        scanner_class.return_value.scan.assert_called_once()
        alignment_class.return_value.align_in_cell.assert_called_once_with(
            settings.START_CELL, walls, settings.START_DIRECTION)
        tracker_class.assert_not_called()
        scanner_class.return_value.stop.assert_called_once()
        stop.assert_called_once_with(ep.chassis)
        ep.close.assert_called_once()

    def test_alignment_only_ctrl_c_stops_chassis(self):
        from robomaster_round2.Slam.runtime import run_real
        fake_sdk = ModuleType('robomaster')
        ep = Mock()
        fake_sdk.robot = SimpleNamespace(Robot=Mock(return_value=ep), FREE='free')
        with patch.dict(sys.modules, {'robomaster': fake_sdk}), patch(
                'robomaster_round2.Slam.sensors.GimbalToFScanner') as scanner_class, patch(
                'robomaster_round2.Slam.movement.stop_chassis') as stop:
            scanner_class.return_value.scan.side_effect = KeyboardInterrupt
            with self.assertRaises(KeyboardInterrupt):
                run_real('unused.json', alignment_only=True)
        scanner_class.return_value.stop.assert_called_once()
        stop.assert_called_once_with(ep.chassis)
        ep.close.assert_called_once()

    def test_sharp_check_reads_samples_without_scanning_or_moving(self):
        from robomaster_round2.Slam.runtime import run_real
        fake_sdk = ModuleType('robomaster')
        ep = Mock()
        fake_sdk.robot = SimpleNamespace(Robot=Mock(return_value=ep), FREE='free')
        with patch.dict(sys.modules, {'robomaster': fake_sdk}), patch(
                'robomaster_round2.Slam.sensors.GimbalToFScanner') as scanner_class, patch(
                'robomaster_round2.Slam.sharp_sensor.sharp_sensor') as sharp_class, patch(
                'robomaster_round2.Slam.movement.stop_chassis') as stop, patch(
                'time.sleep', side_effect=KeyboardInterrupt):
            sharp_class.return_value.read_sample.side_effect = [
                (512, 1.65, 7.54), (510, 1.64, 7.51)]
            run_real('unused.json', sharp_check=True)
        self.assertEqual(sharp_class.return_value.read_sample.call_count, 2)
        scanner_class.assert_not_called()
        stop.assert_called_once_with(ep.chassis)
        ep.close.assert_called_once()


class TestManualStop(unittest.TestCase):
    def test_movement_has_no_front_stop_or_deadline_and_ctrl_c_stops(self):
        from robomaster_round2.Slam.movement import RoboMasterChassisController
        chassis = SimpleNamespace(drive_speed=Mock())
        tracker = Mock()
        tracker.read.side_effect = [
            (0, 0, 0), (0, 0, 0), KeyboardInterrupt()]
        sensors = Mock()
        controller = RoboMasterChassisController(
            SimpleNamespace(chassis=chassis), sensors, tracker)
        with patch('time.monotonic', side_effect=count(0, 100)), patch('time.sleep'):
            with self.assertRaises(KeyboardInterrupt):
                controller(0.61, 0)
        sensors.front_distance_cm.assert_not_called()
        self.assertEqual(chassis.drive_speed.call_args_list[0].kwargs['timeout'], None)
        chassis.drive_speed.assert_called_with(x=0.0, y=0.0, z=0.0)

    def test_move_never_adjusts_yaw_and_rejects_bad_final_pose(self):
        from robomaster_round2.Slam.movement import RoboMasterChassisController
        chassis = SimpleNamespace(drive_speed=Mock())
        tracker = Mock()
        tracker.read.side_effect = [
            (0, 0, 4),
            (0, 0, 4),
            (0.61, 0.1, 4),
            (0.61, 0.1, 4),
        ]
        controller = RoboMasterChassisController(
            SimpleNamespace(chassis=chassis), Mock(), tracker)
        controller.reference_yaw = 0
        with patch('time.sleep'):
            with self.assertRaisesRegex(RuntimeError, 'outside cell'):
                controller(0.61, 0)
        commands = [call.kwargs for call in chassis.drive_speed.call_args_list]
        self.assertTrue(any(command['x'] != 0 for command in commands))
        self.assertTrue(all(command['y'] == 0 and command['z'] == 0
                            for command in commands))

    def test_slide_commands_only_y_and_rejects_forward_drift(self):
        from robomaster_round2.Slam.movement import RoboMasterChassisController
        chassis = SimpleNamespace(drive_speed=Mock())
        tracker = Mock()
        tracker.read.side_effect = [
            (0, 0, 4),
            (0, 0, 4),
            (0.1, 0.61, 4),
            (0.1, 0.61, 4),
        ]
        controller = RoboMasterChassisController(
            SimpleNamespace(chassis=chassis), Mock(), tracker)
        controller.reference_yaw = 0
        with patch('time.sleep'):
            with self.assertRaisesRegex(RuntimeError, 'outside cell'):
                controller(0, 0.61)
        commands = [call.kwargs for call in chassis.drive_speed.call_args_list]
        self.assertTrue(any(command['y'] != 0 for command in commands))
        self.assertTrue(all(command['x'] == 0 and command['z'] == 0
                            for command in commands))

    def test_diagonal_request_runs_x_then_y(self):
        from robomaster_round2.Slam.movement import RoboMasterChassisController
        chassis = SimpleNamespace(drive_speed=Mock())
        tracker = Mock()
        tracker.read.side_effect = [
            (0, 0, 0), (0, 0, 0), (0.2, 0, 0), (0.2, 0, 0),
            (0.2, 0, 0), (0.2, 0, 0), (0.2, 0.3, 0), (0.2, 0.3, 0),
        ]
        controller = RoboMasterChassisController(
            SimpleNamespace(chassis=chassis), Mock(), tracker)
        controller.reference_yaw = 0
        with patch('time.sleep'):
            self.assertTrue(controller(0.2, 0.3))
        commands = [call.kwargs for call in chassis.drive_speed.call_args_list]
        self.assertTrue(all(command['x'] == 0 or command['y'] == 0
                            for command in commands))
        first_x = next(i for i, command in enumerate(commands) if command['x'] != 0)
        first_y = next(i for i, command in enumerate(commands) if command['y'] != 0)
        self.assertLess(first_x, first_y)

    def test_small_cross_track_and_yaw_error_still_confirm_arrival(self):
        from robomaster_round2.Slam.movement import RoboMasterChassisController
        chassis = SimpleNamespace(drive_speed=Mock())
        tracker = Mock()
        tracker.read.side_effect = [
            (0, 0, 0), (0, 0, 0), (0.61, 0.01, 2), (0.61, 0.01, 2)]
        controller = RoboMasterChassisController(
            SimpleNamespace(chassis=chassis), Mock(), tracker)
        controller.reference_yaw = 0
        with patch('time.sleep'):
            self.assertTrue(controller(0.61, 0))

    def test_bad_heading_refuses_before_motion(self):
        from robomaster_round2.Slam.movement import RoboMasterChassisController
        chassis = SimpleNamespace(drive_speed=Mock())
        tracker = Mock()
        tracker.read.return_value = (0, 0, 10)
        controller = RoboMasterChassisController(
            SimpleNamespace(chassis=chassis), Mock(), tracker)
        controller.reference_yaw = 0
        with self.assertRaisesRegex(RuntimeError, 'heading differs'):
            controller(0.61, 0)
        chassis.drive_speed.assert_not_called()

    def test_drift_after_settling_does_not_confirm_cell(self):
        from robomaster_round2.Slam.movement import RoboMasterChassisController
        chassis = SimpleNamespace(drive_speed=Mock())
        tracker = Mock()
        tracker.read.side_effect = [
            (0, 0, 0), (0, 0, 0), (0.61, 0, 0), (0.61, 0.05, 0)]
        controller = RoboMasterChassisController(
            SimpleNamespace(chassis=chassis), Mock(), tracker)
        controller.reference_yaw = 0
        with patch('time.sleep'):
            with self.assertRaisesRegex(RuntimeError, 'outside cell'):
                controller(0.61, 0)
        chassis.drive_speed.assert_called_with(x=0.0, y=0.0, z=0.0)


if __name__ == '__main__':
    unittest.main()
