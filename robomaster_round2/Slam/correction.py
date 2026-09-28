"""Alignment ด้านหน้าด้วย ToF และด้านซ้าย/มุมด้วย Sharp."""

from .setting import maze_setting as settings


class WallCorrection:
    """ใช้เฉพาะด้านที่พบผนัง; แกนที่ไม่มีผนังอ้างอิงจะไม่ปรับ.

    target distances ต้องวัดจาก ToF ขณะหุ่นอยู่กลางช่องจริง
    ไม่ใช้ค่า 6/18 cm ของชุดเซนเซอร์ซ้ายเดิมกับ ToF บน gimbal
    """

    def __init__(self, scanner, move):
        self.scanner = scanner
        self.move = move

    @staticmethod
    def _axis_error(walls, distances, positive, negative):
        errors = []
        targets = settings.CORRECTION_TARGET_CM
        if walls[positive]:
            errors.append(distances[positive] - targets[positive])
        if walls[negative]:
            errors.append(targets[negative] - distances[negative])
        return sum(errors) / len(errors) if errors else 0.0

    def __call__(self, walls):
        for attempt in range(settings.CORRECTION_MAX_ATTEMPTS + 1):
            distances = self.scanner.last_measurements()['distance_cm']
            x_cm = self._axis_error(walls, distances, 'N', 'S')
            y_cm = self._axis_error(walls, distances, 'E', 'W')
            error = max(abs(x_cm), abs(y_cm))
            if error <= settings.CORRECTION_TOLERANCE_CM:
                return walls
            if error > settings.CORRECTION_MAX_OFFSET_CM:
                raise RuntimeError('Correction too large; check pose and ToF calibration')
            if attempt == settings.CORRECTION_MAX_ATTEMPTS:
                raise RuntimeError('Wall correction did not converge')
            if self.move(x_cm / 100, y_cm / 100 * settings.RIGHT_SLIDE_SIGN) is not True:
                raise RuntimeError('Wall correction movement failed')
            walls = self.scanner.scan()
        return walls


class RoboMasterAlignment:
    """ToF ปรับระยะหน้า; Sharp ซ้ายหน้า/หลังปรับมุมให้ขนานผนัง.

    left_sensor ต้องมี get_distance(board, port) คืนระยะเซนติเมตร
    ผู้เรียกใช้ผล True เพื่อยืนยัน alignment ก่อนปรับ reference yaw
    """

    def __init__(self, ep_robot, left_sensor, front_sensor):
        from ..src.PID import PIDController
        self.chassis = ep_robot.chassis
        self.left_sensor = left_sensor
        # front_sensor.front_distance_cm() ต้องอ่าน ToF ที่หันด้านหน้าแล้ว
        self.front_sensor = front_sensor
        self.front_position_pid = PIDController(
            *settings.ALIGNMENT_FRONT_PID,
            min_output=-settings.MAX_ALIGNMENT_SPEED_MPS,
            max_output=settings.MAX_ALIGNMENT_SPEED_MPS,
        )
        self.parallel_pid = PIDController(
            *settings.ALIGNMENT_YAW_PID,
            min_output=-settings.MAX_ALIGNMENT_YAW_DPS,
            max_output=settings.MAX_ALIGNMENT_YAW_DPS,
        )

    @staticmethod
    def _axis_speed(pid, target, measured, dt, tolerance, minimum):
        """แกนที่ถึงเป้าให้หยุด; แกนอื่นใช้ PID พร้อมความเร็วขั้นต่ำ."""
        if abs(target - measured) <= tolerance:
            pid.reset()
            return 0.0
        speed = pid.compute(target, measured, dt)
        if 0 < abs(speed) < minimum:
            return minimum if speed > 0 else -minimum
        return speed

    def align_in_cell(self, cell, walls, heading):
        import math
        import time
        from .movement import stop_chassis

        directions = ('N', 'E', 'S', 'W')
        left_direction = directions[(directions.index(heading) - 1) % 4]
        align_front = walls[heading] is True
        align_left = walls[left_direction] is True
        if not align_front and not align_left:
            print(f'[Alignment] cell={cell} skipped: no front/left wall')
            return False

        print(
            f'[Alignment] cell={cell} adjusting '
            f'front={align_front} left={align_left}', flush=True)
        if not align_left:
            print(f'[Sharp] cell={cell} skipped: no left wall', flush=True)

        if align_left:
            reset_filter = getattr(self.left_sensor, 'reset_filter', None)
            if reset_filter is not None:
                reset_filter()
        self.front_position_pid.reset()
        self.parallel_pid.reset()
        started = previous = time.monotonic()
        next_sharp_log = started
        next_command_log = started
        period = 1.0 / settings.ALIGNMENT_FREQUENCY_HZ
        phase = 'sharp' if align_left else 'tof'
        aligned = False
        try:
            while True:
                now = time.monotonic()
                if phase == 'sharp':
                    front = self.left_sensor.get_distance(
                        settings.LEFT_FRONT_BOARD, settings.LEFT_FRONT_PORT)
                    back = self.left_sensor.get_distance(
                        settings.LEFT_BACK_BOARD, settings.LEFT_BACK_PORT)
                    invalid_sharp = any(
                        v is None or not math.isfinite(v) or v <= 0
                        for v in (front, back))
                    if now >= next_sharp_log or invalid_sharp:
                        lf = 'N/A' if front is None else f'{front:.3f}'
                        lb = 'N/A' if back is None else f'{back:.3f}'
                        difference = 'N/A' if invalid_sharp else f'{front - back:.3f}'
                        print(
                            f'[Sharp] cell={cell} left_front={lf}cm '
                            f'left_back={lb}cm '
                            f'parallel_error={difference}cm '
                            f'filter=median{settings.SHARP_FILTER_WINDOW}',
                            flush=True)
                        next_sharp_log = now + 0.5
                    if invalid_sharp:
                        print(f'[Alignment] cell={cell} skipped: invalid Sharp reading; continuing exploration', flush=True)
                        return False
                    parallel_error = front - back
                    if abs(parallel_error) <= settings.PARALLEL_TOLERANCE_CM:
                        print(
                            f'[Alignment] cell={cell} Sharp parallel: '
                            f'difference={abs(parallel_error):.3f}cm <= '
                            f'tolerance={settings.PARALLEL_TOLERANCE_CM:.3f}cm; '
                            'no yaw movement needed', flush=True)
                        if not align_front:
                            aligned = True
                            return True
                        stop_chassis(self.chassis)
                        phase = 'tof'
                        previous = time.monotonic()
                        time.sleep(settings.ALIGNMENT_SETTLE_S)
                        print(f'[Alignment] cell={cell} adjusting front distance with ToF', flush=True)
                        continue
                    dt = max(time.monotonic() - previous, 0.001)
                    previous = time.monotonic()
                    # RoboMaster: z < 0 turns left, z > 0 turns right.
                    # Sharp front farther than back -> left; reverse -> right.
                    yaw_speed = self._axis_speed(
                        self.parallel_pid, 0, parallel_error, dt,
                        settings.PARALLEL_TOLERANCE_CM, settings.MIN_ALIGNMENT_YAW_DPS)
                    if now >= next_command_log:
                        turn = 'left' if yaw_speed < 0 else 'right'
                        print(
                            f'[Alignment cmd] cell={cell} phase=Sharp '
                            f'x=0.000m/s y=0.000m/s z={yaw_speed:.1f}deg/s '
                            f'turn={turn} parallel_error={parallel_error:.3f}cm',
                            flush=True)
                        next_command_log = now + 0.5
                    self.chassis.drive_speed(
                        x=0.0, y=0.0, z=yaw_speed, timeout=None)
                else:
                    front_cm = self.front_sensor.front_distance_cm()
                    if (front_cm is None or not math.isfinite(front_cm)
                            or front_cm <= 0
                            or front_cm > settings.FRONT_WALL_THRESHOLD_CM):
                        print(
                            f'[Alignment] cell={cell} skipped: front ToF={front_cm}cm '
                            f'(requires <= {settings.FRONT_WALL_THRESHOLD_CM:.1f}cm); '
                            'continuing exploration', flush=True)
                        return False
                    if abs(front_cm - settings.TARGET_FRONT_WALL_CM) <= settings.POSITION_TOLERANCE_CM:
                        print(
                            f'[Alignment] cell={cell} front ToF={front_cm:.2f}cm '
                            f'within target={settings.TARGET_FRONT_WALL_CM:.2f} '
                            f'+/-{settings.POSITION_TOLERANCE_CM:.2f}cm; '
                            'no front movement needed', flush=True)
                        aligned = True
                        return True
                    dt = max(time.monotonic() - previous, 0.001)
                    previous = time.monotonic()
                    x_speed = -self._axis_speed(
                        self.front_position_pid, settings.TARGET_FRONT_WALL_CM,
                        front_cm, dt, settings.POSITION_TOLERANCE_CM,
                        settings.MIN_ALIGNMENT_SPEED_MPS)
                    if now >= next_command_log:
                        print(
                            f'[Alignment cmd] cell={cell} phase=ToF '
                            f'x={x_speed:.3f}m/s y=0.000m/s z=0.0deg/s '
                            f'front_error={settings.TARGET_FRONT_WALL_CM - front_cm:.2f}cm',
                            flush=True)
                        next_command_log = now + 0.5
                    self.chassis.drive_speed(
                        x=x_speed, y=0.0, z=0.0, timeout=None)
                time.sleep(max(0, period - (time.monotonic() - now)))
        finally:
            stop_chassis(self.chassis)
            if aligned:
                print(f'[Alignment] cell={cell} aligned; settling {settings.ALIGNMENT_SETTLE_S:.2f}s', flush=True)
                time.sleep(settings.ALIGNMENT_SETTLE_S)
                print(f'[Alignment] cell={cell} ready', flush=True)
