import threading
import time

from .setting import maze_setting as settings


class ChassisPoseTracker:
    """อ่าน position และ attitude เพื่อป้อนกลับให้ PID ขณะเดิน"""

    def __init__(self, ep_robot):
        self.chassis = ep_robot.chassis
        self._x = None
        self._y = None
        self._yaw = None
        self._position_updated_at = None
        self._attitude_updated_at = None
        self._lock = threading.Lock()
        self._position_ready = threading.Event()
        self._attitude_ready = threading.Event()
        self._position_subscribed = False
        self._attitude_subscribed = False

    def _position_callback(self, position):
        x, y, _ = position
        with self._lock:
            self._x = x
            self._y = y
            self._position_updated_at = time.monotonic()
            self._position_ready.set()

    def _attitude_callback(self, attitude):
        yaw, _, _ = attitude
        with self._lock:
            self._yaw = yaw
            self._attitude_updated_at = time.monotonic()
            self._attitude_ready.set()

    def start(self):
        self.chassis.sub_position(
            freq=settings.MOVE_PID_FREQUENCY_HZ,
            callback=self._position_callback,
        )
        self._position_subscribed = True
        self.chassis.sub_attitude(
            freq=settings.MOVE_PID_FREQUENCY_HZ,
            callback=self._attitude_callback,
        )
        self._attitude_subscribed = True

        while not self._position_ready.wait(0.1):
            pass
        while not self._attitude_ready.wait(0.1):
            pass

    def stop(self):
        if self._position_subscribed:
            self.chassis.unsub_position()
            self._position_subscribed = False
        if self._attitude_subscribed:
            self.chassis.unsub_attitude()
            self._attitude_subscribed = False

    def read(self):
        # Wait for usable feedback instead of terminating the run on a timeout.
        while True:
            with self._lock:
                now = time.monotonic()
                if (self._position_updated_at is not None
                        and self._attitude_updated_at is not None
                        and now - self._position_updated_at <= settings.POSE_STALE_TIMEOUT_S
                        and now - self._attitude_updated_at <= settings.POSE_STALE_TIMEOUT_S):
                    return self._x, self._y, self._yaw
            time.sleep(0.01)
