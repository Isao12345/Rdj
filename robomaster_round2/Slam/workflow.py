"""ลำดับสแกน/ปรับแนว แยกจาก SDK เพื่อทดสอบด้วยอุปกรณ์จำลองได้."""

import math

from .setting import maze_setting as settings


class ExplorationWorkflow:
    def __init__(self, scanner, tracker, mover, alignment=None):
        self.scanner = scanner
        self.tracker = tracker
        self.mover = mover
        self.alignment = alignment
        self.scan_pose = None

    def scan(self):
        """Scan walls without moving or correcting chassis heading."""
        print('[Workflow] scanning walls', flush=True)
        self.scan_pose = self.tracker.read()
        return self.scanner.scan()

    def correct(self, cell, walls):
        if self.alignment is None:
            return walls
        print(f'[Workflow] aligning cell={cell}', flush=True)
        aligned = self.alignment.align_in_cell(cell, walls, settings.START_DIRECTION)
        print(f'[Workflow] alignment {"complete" if aligned else "skipped"} cell={cell}', flush=True)
        if aligned and walls["W"] is True:
            self.mover.reference_yaw = self.tracker.read()[2]
        return walls

    def verify_move(self, direction, source, target):
        """Recheck the chosen exit only when pose changed since the wall scan."""
        if self.scan_pose is None:
            raise RuntimeError('Cannot verify move before scanning')
        x, y, yaw = self.tracker.read()
        sx, sy, syaw = self.scan_pose
        translation = math.hypot(x - sx, y - sy)
        yaw_change = abs((yaw - syaw + 180.0) % 360.0 - 180.0)
        if (translation <= settings.SCAN_RECHECK_TRANSLATION_M
                and yaw_change <= settings.SCAN_RECHECK_YAW_DEG):
            return True
        print(f'[Workflow] pose changed since scan: '
              f'distance={translation:.3f}m yaw={yaw_change:.2f}deg; '
              f'rechecking {direction} toward {target}', flush=True)
        is_wall = self.scanner.verify_direction(direction)
        after_x, after_y, after_yaw = self.tracker.read()
        verification_shift = math.hypot(after_x - x, after_y - y)
        verification_yaw = abs((after_yaw - yaw + 180.0) % 360.0 - 180.0)
        if (verification_shift > settings.SCAN_RECHECK_TRANSLATION_M
                or verification_yaw > settings.SCAN_RECHECK_YAW_DEG):
            raise RuntimeError('Robot pose changed during ToF direction verification; '
                               'refusing to move')
        return not is_wall

    def run(self, session, max_steps=None):
        session.max_steps = max_steps
        correction = None
        if self.alignment is not None:
            correction = lambda walls: self.correct(session.cell, walls)
        session.explore(self.scan, self.mover, correction, self.verify_move)
