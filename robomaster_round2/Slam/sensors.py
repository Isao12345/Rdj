import threading
import time

from .setting import maze_setting as settings


DIRECTIONS = ("N", "E", "S", "W")
class FrontToF:
    """เก็บค่า ToF ล่าสุด โดยแปลงจากมิลลิเมตรเป็นเซนติเมตร"""

    def __init__(self):
        self._distance_cm = None
        self._updated_at = None
        self._lock = threading.Lock()
        self.ready = threading.Event()

    def update(self, distances):
        if not distances:
            return
        with self._lock:
            self._distance_cm = distances[0] / 10.0
            self._updated_at = time.monotonic()
            self.ready.set()

    def read(self):
        with self._lock:
            return self._distance_cm, self._updated_at


class RoboMasterMazeSensors:
    """แปลงเซนเซอร์รอบตัวหุ่นยนต์เป็นกำแพงทิศ N/E/S/W"""

    def __init__(self, ep_robot, left_sensor, start_direction):
        self.distance_sensor = ep_robot.sensor
        self.sensor_adaptor = ep_robot.sensor_adaptor
        self.left_sensor = left_sensor
        self.front_tof = FrontToF()
        self.heading = start_direction
        self._subscribed = False
        self._cached_cell = None
        self._cached_walls = None
        self._last_measurements = None

    def start(self):
        self.distance_sensor.sub_distance(
            freq=settings.TOF_FREQUENCY_HZ,
            callback=self.front_tof.update,
        )
        self._subscribed = True
        if not self.front_tof.ready.wait(3.0):
            raise RuntimeError("ไม่ได้รับข้อมูลจาก ToF ด้านหน้า")

    def stop(self):
        if self._subscribed:
            self.distance_sensor.unsub_distance()
            self._subscribed = False

    def set_heading(self, direction):
        self.heading = direction
        self.invalidate_cache()

    def invalidate_cache(self):
        self._cached_cell = None
        self._cached_walls = None

    def _read_ir_wall(self, board, port):
        value = self.sensor_adaptor.get_io(id=board, port=port)
        if value not in (0, 1):
            raise RuntimeError(
                f"IR sensor board={board}, port={port} คืนค่าผิดปกติ: {value}"
            )
        return value == settings.IR_WALL_VALUE, value

    def read_measurements(self):
        """อ่านค่าดิบที่ใช้ทั้งตรวจผนังและจัดตำแหน่งกลางช่อง"""
        front_cm = self.front_distance_cm()

        left_front_cm = self.left_sensor.get_distance(
            settings.LEFT_FRONT_BOARD,
            settings.LEFT_FRONT_PORT,
        )
        left_back_cm = self.left_sensor.get_distance(
            settings.LEFT_BACK_BOARD,
            settings.LEFT_BACK_PORT,
        )
        right_wall, right_io = self._read_ir_wall(
            settings.RIGHT_IR_BOARD,
            settings.RIGHT_IR_PORT,
        )
        back_wall, back_io = self._read_ir_wall(
            settings.BACK_IR_BOARD,
            settings.BACK_IR_PORT,
        )

        measurements = {
            "front_cm": front_cm,
            "left_front_cm": left_front_cm,
            "left_back_cm": left_back_cm,
            "right_wall": right_wall,
            "back_wall": back_wall,
            "right_io": right_io,
            "back_io": back_io,
        }
        self._last_measurements = dict(measurements)
        return measurements

    def front_distance_cm(self):
        """คืน ToF ด้านหน้าล่าสุด และไม่ยอมใช้ข้อมูลที่เก่าเกินกำหนด"""
        front_cm, updated_at = self.front_tof.read()
        now = time.monotonic()
        if (
            updated_at is None
            or now - updated_at > settings.SENSOR_TIMEOUT_S
        ):
            raise RuntimeError("ข้อมูล ToF ด้านหน้าขาดหายเกิน 1 วินาที")
        return front_cm

    def last_measurements(self):
        """คืน snapshot ล่าสุดโดยไม่สั่งอ่านฮาร์ดแวร์ซ้ำ"""
        if self._last_measurements is None:
            return None
        return dict(self._last_measurements)

    def _read_relative_walls(self):
        measurements = self.read_measurements()
        front_cm = measurements["front_cm"]
        left_front_cm = measurements["left_front_cm"]
        left_back_cm = measurements["left_back_cm"]

        relative_walls = {
            "front": front_cm <= settings.FRONT_WALL_THRESHOLD_CM,
            "right": measurements["right_wall"],
            "back": measurements["back_wall"],
            "left": left_front_cm is not None and left_back_cm is not None,
        }

        lf_text = "N/A" if left_front_cm is None else f"{left_front_cm:.1f}"
        lb_text = "N/A" if left_back_cm is None else f"{left_back_cm:.1f}"
        print(
            f"[Sensors] ToF={front_cm:.1f}cm lf={lf_text}cm lb={lb_text}cm "
            f"right_io={measurements['right_io']} "
            f"back_io={measurements['back_io']} walls={relative_walls}"
        )
        return relative_walls

    def _read_absolute_walls(self):
        relative = self._read_relative_walls()
        heading_index = DIRECTIONS.index(self.heading)
        relative_by_offset = (
            relative["front"],
            relative["right"],
            relative["back"],
            relative["left"],
        )
        return {
            DIRECTIONS[(heading_index + offset) % 4]: is_wall
            for offset, is_wall in enumerate(relative_by_offset)
        }

    def __call__(self, row, col, direction):
        cell = (row, col)
        if self._cached_cell != cell:
            self._cached_walls = self._read_absolute_walls()
            self._cached_cell = cell
        return self._cached_walls[direction]





class GimbalToFScanner:
    """Scan four chassis-relative directions while the chassis remains still."""

    def __init__(self, ep_robot):
        self.gimbal = ep_robot.gimbal
        self.sensor = ep_robot.sensor
        self.tof = FrontToF()
        self._subscribed = False
        self._walls = None
        self._measurements = {}
        self._gimbal_action_pending = False

    def start(self):
        self.sensor.sub_distance(freq=settings.TOF_FREQUENCY_HZ,
                                 callback=self.tof.update)
        self._subscribed = True

    def stop(self):
        if self._subscribed:
            self.sensor.unsub_distance()
            self._subscribed = False

    def invalidate_cache(self):
        self._walls = None

    def _point(self, yaw):
        if self._gimbal_action_pending:
            raise RuntimeError("Previous gimbal action has not completed")
        # A timeout or interrupt does not cancel the SDK's running action.
        # Mark pending before dispatch, since dispatch itself can be interrupted.
        self._gimbal_action_pending = True
        action = self.gimbal.moveto(pitch=0, yaw=yaw,
                                   yaw_speed=settings.SCAN_YAW_SPEED_DPS)
        if not action.wait_for_completed(timeout=None):
            raise RuntimeError("Gimbal did not reach scan angle")
        self._gimbal_action_pending = False
        time.sleep(settings.SCAN_SETTLE_S)

    def _fresh_distance(self, direction):
        # Only accept a callback received AFTER the gimbal has settled.
        started = since = time.monotonic()
        next_wait_log = started + 1.0
        samples = []
        while True:
            distance, timestamp = self.tof.read()
            if timestamp is not None and timestamp > since:
                since = timestamp
                if distance is None or not (0 < distance <= settings.TOF_MAX_VALID_CM):
                    time.sleep(0.005)
                    continue
                samples.append(distance)
                print(
                    f'[ToF sample] direction={direction} '
                    f'count={len(samples)}/{settings.SCAN_SAMPLES} '
                    f'distance={distance:.2f}cm', flush=True)
                if len(samples) == settings.SCAN_SAMPLES:
                    return sorted(samples)[len(samples) // 2]
            now = time.monotonic()
            if now >= next_wait_log:
                print(
                    f'[ToF scan] direction={direction} waiting for fresh ToF samples '
                    f'{len(samples)}/{settings.SCAN_SAMPLES} '
                    f'elapsed={now - started:.1f}s', flush=True)
                next_wait_log = now + 1.0
            time.sleep(0.005)

    def scan(self):
        self._walls = None
        distances = {}
        walls = {}
        scan_started = time.monotonic()
        print('[ToF scan] scanning N E S W', flush=True)
        try:
            for direction, yaw in settings.SCAN_YAWS.items():
                direction_started = time.monotonic()
                print(f'[ToF scan] direction={direction} turning gimbal to {yaw}deg', flush=True)
                self._point(yaw)
                print(f'[ToF scan] direction={direction} gimbal ready; '
                      f'collecting {settings.SCAN_SAMPLES} ToF samples', flush=True)
                distance, is_wall = self._read_wall(direction)
                distances[direction] = distance
                walls[direction] = is_wall
                result = 'WALL' if is_wall else 'OPEN'
                print(
                    f'[ToF scan] direction={direction} yaw_target={yaw}deg '
                    f'median_distance={distance:.2f}cm '
                    f'wall<={settings.SCAN_WALL_THRESHOLD_CM:.2f}cm '
                    f'open>={settings.SCAN_OPEN_THRESHOLD_CM:.2f}cm '
                    f'result={result} elapsed={time.monotonic() - direction_started:.2f}s',
                    flush=True)
        finally:
            # The existing movement controller expects ToF to face forward.
            # Do not overlap an interrupted/timed-out turn or mask its error.
            if not self._gimbal_action_pending:
                print('[ToF scan] returning gimbal to front', flush=True)
                self._point(0)
        print('[ToF scan] gimbal front; refreshing ToF', flush=True)
        self._fresh_distance('front')
        self._measurements = {"distance_cm": distances}
        self._walls = walls
        print(f'[ToF scan] complete walls={self._walls} '
              f'elapsed={time.monotonic() - scan_started:.2f}s', flush=True)
        return dict(self._walls)

    def _read_wall(self, direction):
        """Classify a fresh median; a borderline result never becomes OPEN."""
        if settings.SCAN_OPEN_THRESHOLD_CM <= settings.SCAN_WALL_THRESHOLD_CM:
            raise ValueError('SCAN_OPEN_THRESHOLD_CM must exceed SCAN_WALL_THRESHOLD_CM')
        for attempt in range(settings.SCAN_UNCERTAIN_RETRIES + 1):
            distance = self._fresh_distance(direction)
            if distance <= settings.SCAN_WALL_THRESHOLD_CM:
                return distance, True
            if distance >= settings.SCAN_OPEN_THRESHOLD_CM:
                return distance, False
            print(
                f'[ToF scan] direction={direction} distance={distance:.2f}cm '
                f'result=UNCERTAIN attempt={attempt + 1}/'
                f'{settings.SCAN_UNCERTAIN_RETRIES + 1}', flush=True)
        raise RuntimeError(
            f'ToF direction={direction} remains uncertain after '
            f'{settings.SCAN_UNCERTAIN_RETRIES + 1} readings; refusing to move')

    def verify_direction(self, direction):
        """Recheck one planned exit after alignment without rescanning the maze."""
        if direction not in settings.SCAN_YAWS:
            raise ValueError(f'Unknown scan direction: {direction}')
        print(f'[ToF verify] checking direction={direction} before move', flush=True)
        try:
            self._point(settings.SCAN_YAWS[direction])
            distance, is_wall = self._read_wall(direction)
            print(f'[ToF verify] direction={direction} distance={distance:.2f}cm '
                  f'result={"WALL" if is_wall else "OPEN"}', flush=True)
            return is_wall
        finally:
            if not self._gimbal_action_pending:
                self._point(0)

    def __call__(self, row, col, direction):
        if self._walls is None:
            self.scan()
        return self._walls[direction]

    def last_measurements(self):
        return dict(self._measurements)

    def front_distance_cm(self):
        while True:
            distance, timestamp = self.tof.read()
            if (timestamp is not None
                    and time.monotonic() - timestamp <= settings.SENSOR_TIMEOUT_S
                    and distance is not None
                    and 0 < distance <= settings.TOF_MAX_VALID_CM):
                return distance
            time.sleep(0.005)
