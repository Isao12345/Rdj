"""Hardware-free exploration checks: python3 -m unittest robomaster_round2.tests.test_slam_explore"""
import json
import io
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from robomaster_round2.Slam.slam import SlamSession, run_simulation
from robomaster_round2.Slam.sensors import GimbalToFScanner
from robomaster_round2.Slam.correction import RoboMasterAlignment
from robomaster_round2.Slam.setting import maze_setting as settings


class TestAlignmentSkip(unittest.TestCase):
    def setUp(self):
        self.chassis = SimpleNamespace(drive_speed=Mock())
        self.sharp = Mock()
        self.tof = Mock()
        self.alignment = RoboMasterAlignment(
            SimpleNamespace(chassis=self.chassis), self.sharp, self.tof)

    def test_invalid_sharp_skips_alignment_and_stops(self):
        self.sharp.get_distance.return_value = None
        self.assertFalse(self.alignment.align_in_cell(
            (0, 0), {'N': False, 'W': True}, 'N'))
        self.chassis.drive_speed.assert_called_once_with(x=0.0, y=0.0, z=0.0)

    def test_alignment_continues_past_old_deadline(self):
        self.sharp.get_distance.return_value = 7.5
        with patch('time.monotonic', side_effect=[0, 100, 100, 100]), patch('time.sleep') as sleep:
            self.assertTrue(self.alignment.align_in_cell(
                (0, 0), {'N': False, 'W': True}, 'N'))
        self.chassis.drive_speed.assert_called_once_with(x=0.0, y=0.0, z=0.0)
        sleep.assert_called_once_with(settings.ALIGNMENT_SETTLE_S)

    def test_left_alignment_only_rotates_until_sharp_is_parallel(self):
        self.sharp.get_distance.side_effect = [7.5, 8.5, 7.5, 7.5]
        with patch('time.sleep'):
            self.assertTrue(self.alignment.align_in_cell(
                (0, 0), {'N': False, 'W': True}, 'N'))
        commands = [call.kwargs for call in self.chassis.drive_speed.call_args_list]
        self.assertTrue(any(command['z'] != 0 for command in commands))
        self.assertTrue(all(command['x'] == 0 and command['y'] == 0
                            for command in commands))

    def test_small_sharp_difference_needs_no_rotation(self):
        self.sharp.get_distance.side_effect = [7.54, 7.51]
        with patch.object(settings, 'PARALLEL_TOLERANCE_CM', 0.5), patch('time.sleep'):
            self.assertTrue(self.alignment.align_in_cell(
                (0, 0), {'N': False, 'W': True}, 'N'))
        self.chassis.drive_speed.assert_called_once_with(x=0.0, y=0.0, z=0.0)

    def test_sharp_front_farther_turns_left_back_farther_turns_right(self):
        for front, back, expected_sign in ((8.5, 7.5, -1), (7.5, 8.5, 1)):
            with self.subTest(front=front, back=back):
                self.chassis.drive_speed.reset_mock()
                self.sharp.get_distance.side_effect = [front, back, 7.5, 7.5]
                with patch('time.sleep'):
                    self.assertTrue(self.alignment.align_in_cell(
                        (0, 0), {'N': False, 'W': True}, 'N'))
                turns = [call.kwargs['z'] for call in self.chassis.drive_speed.call_args_list
                         if call.kwargs['z'] != 0]
                self.assertTrue(turns)
                self.assertTrue(all(speed * expected_sign > 0 for speed in turns))

    def test_sharp_yaw_finishes_before_tof_front_adjustment(self):
        self.sharp.get_distance.side_effect = [7.5, 8.5, 7.5, 7.5]
        self.tof.front_distance_cm.side_effect = [22.0, 15.0]
        with patch.object(settings, 'FRONT_WALL_THRESHOLD_CM', 30.0), patch('time.sleep'):
            self.assertTrue(self.alignment.align_in_cell(
                (0, 0), {'N': True, 'W': True}, 'N'))
        commands = [call.kwargs for call in self.chassis.drive_speed.call_args_list]
        yaw_indices = [i for i, command in enumerate(commands) if command['z'] != 0]
        forward_indices = [i for i, command in enumerate(commands) if command['x'] != 0]
        self.assertTrue(yaw_indices)
        self.assertTrue(forward_indices)
        self.assertLess(max(yaw_indices), min(forward_indices))
        self.assertTrue(all(command['y'] == 0 for command in commands))
        self.assertTrue(all(command['x'] == 0 or command['z'] == 0
                            for command in commands))
        self.assertEqual(self.sharp.get_distance.call_count, 4)
        self.assertEqual(self.tof.front_distance_cm.call_count, 2)

    def test_axis_speed_preserves_direction_minimum_and_reset(self):
        from robomaster_round2.src.PID import PIDController
        pid = PIDController(0.025, 0.01, 0, min_output=-0.4, max_output=0.4)
        speed = self.alignment._axis_speed
        self.assertEqual(speed(pid, 18, 17, 0.05, 0.2, 0.04), 0.04)
        self.assertEqual(speed(pid, 18, 19, 0.05, 0.2, 0.04), -0.04)
        self.assertEqual(speed(pid, 18, 18.1, 0.05, 0.2, 0.04), 0.0)
        self.assertEqual(pid.integral, 0.0)
        self.assertEqual(speed(pid, 18, 0, 0.05, 0.2, 0.04), 0.4)

    def test_invalid_tof_skips_alignment_and_stops(self):
        self.tof.front_distance_cm.return_value = None
        self.assertFalse(self.alignment.align_in_cell(
            (0, 0), {'N': True, 'W': False}, 'N'))
        self.chassis.drive_speed.assert_called_once_with(x=0.0, y=0.0, z=0.0)

    def test_front_tof_over_30_skips_alignment(self):
        self.tof.front_distance_cm.side_effect = [30.1, settings.TARGET_FRONT_WALL_CM]
        self.assertFalse(self.alignment.align_in_cell(
            (0, 0), {'N': True, 'W': False}, 'N'))
        self.tof.front_distance_cm.assert_called_once()
        self.chassis.drive_speed.assert_called_once_with(x=0.0, y=0.0, z=0.0)


class ConfiguredArenaTest(unittest.TestCase):
    def setUp(self):
        # สนามทดสอบคงที่ เพื่อให้การจูน config หุ่นจริงไม่ทำให้ test เปลี่ยนตาม
        config = patch.multiple(
            settings, MAZE_ROWS=6, MAZE_COLS=5, START_CELL=(5, 0),
            CORRECTION_TARGET_CM=dict.fromkeys(('N', 'E', 'S', 'W'), 30.5),
            PRE_MOVE_DELAY_S=0)
        config.start()
        self.addCleanup(config.stop)


class TestExploration(ConfiguredArenaTest):
    def setUp(self):
        super().setUp()
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.output = Path(self.directory.name) / 'map.json'

    def test_full_arena_shared_edges_and_return(self):
        run_simulation(self.output)
        data = json.loads(self.output.read_text())
        self.assertEqual(data['status'], 'exploration_complete')
        self.assertEqual(len(data['visited_cells']), 30)
        self.assertEqual(data['path'][0], data['path'][-1])
        self.assertEqual(len(data['moves']), 58)
        for move in data['moves']:
            r, c = move['from']
            dr, dc = move['to'][0] - r, move['to'][1] - c
            direction = {(-1, 0): 'N', (1, 0): 'S', (0, 1): 'E', (0, -1): 'W'}[dr, dc]
            self.assertIs(data['maze'][r][c][direction], False)
        self.assertIs(data['maze'][4][0]['E'], True)
        self.assertIs(data['maze'][4][1]['W'], True)

    def test_enclosed_start_finishes_without_moving(self):
        session = SlamSession(lambda *args: True, self.output)
        move = Mock()
        session.explore(lambda: None, move)
        move.assert_not_called()
        self.assertEqual(len(session.visited), 1)

    def test_map_window_callback_runs_after_initial_file_before_scan(self):
        events = []
        def map_ready():
            data = json.loads(self.output.read_text())
            self.assertEqual(data['status'], 'running')
            events.append('map_ready')
        def scan():
            events.append('scan')
            return dict(N=True, E=True, S=True, W=True)
        session = SlamSession(Mock(), self.output, on_map_ready=map_ready)
        session.explore(scan, Mock())
        self.assertEqual(events, ['map_ready', 'scan'])

    def test_failed_move_does_not_advance_cell(self):
        session = SlamSession(lambda r, c, d: d != 'N', self.output)
        with self.assertRaises(RuntimeError):
            session.explore(lambda: None, lambda x, y: False)
        data = json.loads(self.output.read_text())
        self.assertEqual(data['status'], 'failed')
        self.assertEqual(data['path'], [[5, 0]])
        self.assertEqual(data['moves'], [])

    def test_one_scan_controls_alignment_and_first_move(self):
        reader = Mock(side_effect=AssertionError('unexpected second read'))
        session = SlamSession(reader, self.output)
        walls = dict(N=True, E=False, S=True, W=True)
        scan = Mock(return_value=walls)
        move = Mock(side_effect=KeyboardInterrupt)
        correct = Mock(return_value=walls)
        with self.assertRaises(KeyboardInterrupt):
            session.explore(scan, move, correct)
        scan.assert_called_once()
        correct.assert_called_once_with(walls)
        reader.assert_not_called()
        move.assert_called_once_with(0.0, settings.CELL_DISTANCE_M)

    def test_blocked_recheck_does_not_move_or_advance_cell(self):
        session = SlamSession(Mock(), self.output)
        walls = dict(N=False, E=True, S=True, W=True)
        move = Mock()
        verify = Mock(return_value=False)
        with self.assertRaisesRegex(RuntimeError, 'blocked after alignment'):
            session.explore(lambda: walls, move, verify_move=verify)
        verify.assert_called_once_with('N', settings.START_CELL,
                                       (settings.START_CELL[0] - 1, 0))
        move.assert_not_called()
        self.assertEqual(session.cell, settings.START_CELL)
        self.assertEqual(json.loads(self.output.read_text())['status'], 'failed')

    def test_scanner_classifies_and_returns_forward(self):
        scanner = GimbalToFScanner(SimpleNamespace(gimbal=Mock(), sensor=Mock()))
        with patch.object(scanner, '_point') as point, patch.object(
                scanner, '_fresh_distance', side_effect=[20, 90, 30, 80, 20]):
            self.assertEqual(scanner.scan(), {'N': True, 'E': False, 'S': True, 'W': False})
        self.assertEqual([call.args[0] for call in point.call_args_list], [0, 90, 180, -90, 0])

    def test_uncertain_tof_retries_then_accepts_open(self):
        scanner = GimbalToFScanner(SimpleNamespace(gimbal=Mock(), sensor=Mock()))
        with patch.object(settings, 'SCAN_WALL_THRESHOLD_CM', 30), patch.object(
                settings, 'SCAN_OPEN_THRESHOLD_CM', 45), patch.object(
                settings, 'SCAN_UNCERTAIN_RETRIES', 2), patch.object(
                scanner, '_fresh_distance', side_effect=[35, 40, 90]) as read:
            self.assertEqual(scanner._read_wall('E'), (90, False))
        self.assertEqual(read.call_count, 3)

    def test_persistent_uncertain_tof_refuses_to_move(self):
        scanner = GimbalToFScanner(SimpleNamespace(gimbal=Mock(), sensor=Mock()))
        with patch.object(settings, 'SCAN_WALL_THRESHOLD_CM', 30), patch.object(
                settings, 'SCAN_OPEN_THRESHOLD_CM', 45), patch.object(
                settings, 'SCAN_UNCERTAIN_RETRIES', 2), patch.object(
                scanner, '_fresh_distance', side_effect=[35, 40, 41]):
            with self.assertRaisesRegex(RuntimeError, 'remains uncertain'):
                scanner._read_wall('E')

    def test_direction_recheck_points_once_and_returns_front(self):
        scanner = GimbalToFScanner(SimpleNamespace(gimbal=Mock(), sensor=Mock()))
        with patch.object(scanner, '_point') as point, patch.object(
                scanner, '_read_wall', return_value=(90, False)):
            self.assertFalse(scanner.verify_direction('E'))
        self.assertEqual([call.args[0] for call in point.call_args_list], [90, 0])

    def test_tof_samples_are_logged_as_they_arrive(self):
        scanner = GimbalToFScanner(SimpleNamespace(gimbal=Mock(), sensor=Mock()))
        readings = [(distance, index + 1) for index, distance in
                    enumerate((20, 21, 100, 19, 22))]
        output = io.StringIO()
        with patch.object(settings, 'SCAN_SAMPLES', 5), patch(
                'time.monotonic', return_value=0), patch.object(
                scanner.tof, 'read', side_effect=readings), patch('time.sleep'):
            with redirect_stdout(output):
                self.assertEqual(scanner._fresh_distance('N'), 21)
        self.assertIn('[ToF sample] direction=N count=1/5 distance=20.00cm',
                      output.getvalue())
        self.assertIn('[ToF sample] direction=N count=5/5 distance=22.00cm',
                      output.getvalue())

    def test_scanner_failure_returns_forward_and_keeps_map_unset(self):
        scanner = GimbalToFScanner(SimpleNamespace(gimbal=Mock(), sensor=Mock()))
        with patch.object(scanner, '_point') as point, patch.object(
                scanner, '_fresh_distance', side_effect=RuntimeError('missing ToF')):
            with self.assertRaises(RuntimeError):
                scanner.scan()
        self.assertEqual(point.call_args_list[-1].args, (0,))
        self.assertIsNone(scanner._walls)


    def test_interrupted_gimbal_does_not_dispatch_cleanup_action(self):
        gimbal = Mock()
        interrupted = KeyboardInterrupt()
        gimbal.moveto.return_value.wait_for_completed.side_effect = interrupted
        scanner = GimbalToFScanner(SimpleNamespace(gimbal=gimbal, sensor=Mock()))
        with self.assertRaises(KeyboardInterrupt) as caught:
            scanner.scan()
        self.assertIs(caught.exception, interrupted)
        gimbal.moveto.assert_called_once()
        self.assertIsNone(scanner._walls)

    def test_timed_out_gimbal_prevents_cleanup_and_retry_actions(self):
        gimbal = Mock()
        gimbal.moveto.return_value.wait_for_completed.return_value = False
        scanner = GimbalToFScanner(SimpleNamespace(gimbal=gimbal, sensor=Mock()))
        with self.assertRaisesRegex(RuntimeError, 'did not reach'):
            scanner.scan()
        with self.assertRaisesRegex(RuntimeError, 'has not completed'):
            scanner.scan()
        gimbal.moveto.assert_called_once()
        self.assertIsNone(scanner._walls)


class TestCorrection(ConfiguredArenaTest):
    def test_axis_signs_and_no_reference_wall(self):
        from robomaster_round2.Slam.correction import WallCorrection
        walls = {'N': True, 'E': False, 'S': False, 'W': True}
        distances = {'N': 35.5, 'E': 100, 'S': 100, 'W': 25.5}
        self.assertEqual(WallCorrection._axis_error(walls, distances, 'N', 'S'), 5)
        self.assertEqual(WallCorrection._axis_error(walls, distances, 'E', 'W'), 5)
        walls = dict.fromkeys(walls, False)
        self.assertEqual(WallCorrection._axis_error(walls, distances, 'N', 'S'), 0)

    def test_correction_moves_then_rescans(self):
        from robomaster_round2.Slam.correction import WallCorrection
        walls = {'N': True, 'E': False, 'S': False, 'W': False}
        scanner = Mock()
        scanner.last_measurements.side_effect = [
            {'distance_cm': dict(N=35.5, E=100, S=100, W=100)},
            {'distance_cm': dict(N=30.5, E=100, S=100, W=100)},
        ]
        scanner.scan.return_value = walls
        move = Mock(return_value=True)
        self.assertEqual(WallCorrection(scanner, move)(walls), walls)
        move.assert_called_once_with(0.05, 0.0)
        scanner.scan.assert_called_once()

    def test_large_correction_refuses_to_move(self):
        from robomaster_round2.Slam.correction import WallCorrection
        scanner = Mock()
        scanner.last_measurements.return_value = {
            'distance_cm': dict(N=10, E=100, S=100, W=100)}
        move = Mock()
        with self.assertRaises(RuntimeError):
            WallCorrection(scanner, move)(dict(N=True, E=False, S=False, W=False))
        move.assert_not_called()

    def test_step_limit_records_final_cell(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'map.json'
            # Open north and south, wall east/west, so one northward move is possible.
            session = SlamSession(lambda r, c, d: d in ('E', 'W'), output)
            session.max_steps = 1
            session.explore(lambda: None, lambda x, y: True)
            data = json.loads(output.read_text())
            self.assertEqual(data['status'], 'step_limit')
            self.assertEqual(data['path'], [[5, 0], [4, 0]])
            self.assertIn('4,0', data['cells'])

    def test_pre_move_waits_before_command(self):
        with tempfile.TemporaryDirectory() as directory:
            session = SlamSession(
                lambda r, c, d: d in ('E', 'W'), Path(directory) / 'map.json')
            session.max_steps = 1
            events = []

            def move(x, y):
                events.append(('move', x, y))
                return True

            with patch.object(settings, 'PRE_MOVE_DELAY_S', 3.0), patch(
                    'robomaster_round2.Slam.slam.time.sleep',
                    side_effect=lambda seconds: events.append(('wait', seconds))):
                session.explore(lambda: None, move)
            self.assertEqual(events, [('wait', 3.0), ('move', 0.61, 0.0)])

    def test_exit_recheck_runs_after_delay_before_move(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        session = SlamSession(Mock(), Path(directory.name) / 'map.json')
        session.max_steps = 1
        scans = iter((dict(N=False, E=True, S=True, W=True),
                      dict(N=True, E=True, S=False, W=True)))
        events = []
        def verify(direction, source, target):
            events.append('verify')
            return True
        def move(x, y):
            events.append('move')
            return True
        with patch.object(settings, 'PRE_MOVE_DELAY_S', 3.0), patch(
                'robomaster_round2.Slam.slam.time.sleep',
                side_effect=lambda seconds: events.append('wait')):
            session.explore(lambda: next(scans), move, verify_move=verify)
        self.assertEqual(events, ['wait', 'verify', 'move'])

    def test_conflicting_scan_preserves_known_wall(self):
        with tempfile.TemporaryDirectory() as directory:
            session = SlamSession(lambda *args: True, Path(directory) / 'map.json')
            session.scan_current_cell()
            session.wall_reader = lambda *args: False
            with self.assertRaises(RuntimeError):
                session.scan_current_cell()
            self.assertIs(session.maze[5][0]['N'], True)


if __name__ == '__main__':
    unittest.main()
