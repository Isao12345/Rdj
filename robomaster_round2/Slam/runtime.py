"""ประกอบฮาร์ดแวร์และดูแลการเปิด/ปิดทรัพยากรของโหมด real."""

import os
from pathlib import Path
import subprocess
import sys

from .slam import SlamSession
from .setting import maze_setting as settings
from .workflow import ExplorationWorkflow


def launch_map_window(output):
    """Show the live JSON map without blocking robot control."""
    if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        print('[Map] live window unavailable: no graphical display', flush=True)
        return
    path = Path(output).resolve()
    try:
        subprocess.Popen(
            [sys.executable, '-m', 'robomaster_round2.Slam.map_gui', str(path)],
            start_new_session=True,
        )
    except OSError as error:
        print(f'[Map] could not open live window: {error}', flush=True)
        return
    print(f'[Map] live window opening for {path}', flush=True)

def run_real(output, scan_only=False, max_steps=None, alignment_only=False,
             sharp_check=False):
    from contextlib import ExitStack
    from robomaster import robot
    from .correction import RoboMasterAlignment
    from .sharp_sensor import sharp_sensor
    from .sensors import GimbalToFScanner
    from .pose_tracker import ChassisPoseTracker
    from .movement import RoboMasterChassisController, stop_chassis

    if settings.START_DIRECTION != "N":
        raise ValueError("Current movement mapping requires START_DIRECTION=N")
    with ExitStack() as cleanup:
        ep = robot.Robot()
        cleanup.callback(ep.close)
        ep.initialize(conn_type="ap")
        ep.set_robot_mode(mode=robot.FREE)
        cleanup.callback(stop_chassis, ep.chassis)
        if sharp_check:
            import time
            sensor = sharp_sensor(ep)
            print('[Sharp check] chassis stationary; press Ctrl+C to finish', flush=True)
            try:
                while True:
                    front_adc, front_v, front_cm = sensor.read_sample(
                        settings.LEFT_FRONT_BOARD, settings.LEFT_FRONT_PORT)
                    back_adc, back_v, back_cm = sensor.read_sample(
                        settings.LEFT_BACK_BOARD, settings.LEFT_BACK_PORT)
                    front_text = 'N/A' if front_cm is None else f'{front_cm:.2f}'
                    back_text = 'N/A' if back_cm is None else f'{back_cm:.2f}'
                    difference = ('N/A' if front_cm is None or back_cm is None
                                  else f'{front_cm - back_cm:.2f}')
                    print(
                        f'[Sharp check] front={front_text}cm adc={front_adc} '
                        f'voltage={front_v:.3f}V | back={back_text}cm '
                        f'adc={back_adc} voltage={back_v:.3f}V | '
                        f'front_minus_back={difference}cm', flush=True)
                    time.sleep(0.5)
            except KeyboardInterrupt:
                print('[Sharp check] stopped', flush=True)
            return
        scanner = GimbalToFScanner(ep)
        cleanup.callback(scanner.stop)
        scanner.start()
        if alignment_only:
            print(f'[Alignment test] scanning at cell={settings.START_CELL}', flush=True)
            walls = scanner.scan()
            print(f'[Alignment test] walls={walls}', flush=True)
            alignment = RoboMasterAlignment(ep, sharp_sensor(ep), scanner)
            aligned = alignment.align_in_cell(
                settings.START_CELL, walls, settings.START_DIRECTION)
            print(f'[Alignment test] result={"aligned" if aligned else "skipped"}', flush=True)
            return aligned
        if scan_only:
            session = SlamSession(scanner, output,
                                  measurement_reader=scanner.last_measurements,
                                  on_map_ready=lambda: launch_map_window(output))
            print(session.run_once())
            return
        tracker = ChassisPoseTracker(ep)
        cleanup.callback(tracker.stop)
        tracker.start()
        mover = RoboMasterChassisController(ep, scanner, tracker)
        session = SlamSession(scanner, output, tracker.read, scanner.last_measurements,
                              on_map_ready=lambda: launch_map_window(output))
        mover.reference_yaw = tracker.read()[2]
        alignment = None
        if settings.ALIGNMENT_ENABLED:
            alignment = RoboMasterAlignment(ep, sharp_sensor(ep), scanner)
        workflow = ExplorationWorkflow(scanner, tracker, mover, alignment)
        workflow.run(session, max_steps)
