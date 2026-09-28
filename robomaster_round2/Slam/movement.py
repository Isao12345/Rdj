import math
import threading
import time

from .setting import maze_setting as settings
from ..src.PID import PIDController
from ..src.pid_telementry import PIDTelemetryRecorder


def stop_chassis(chassis):
    """หยุด chassis และยกเลิก SDK auto-stop timer ที่อาจชนกับ arm action"""
    auto_timer = getattr(chassis, "_auto_timer", None)
    if auto_timer is not None:
        auto_timer.cancel()
        if (
            auto_timer is not threading.current_thread()
            and auto_timer.is_alive()
        ):
            auto_timer.join(timeout=1.0)
        chassis._auto_timer = None
    return chassis.drive_speed(x=0.0, y=0.0, z=0.0)


class RoboMasterChassisController:
    """รับ target x/y และขับ chassis ด้วย PID position x/y และ yaw z"""

    def __init__(self, ep_robot, maze_sensors, pose_tracker):
        self.chassis = ep_robot.chassis
        self.maze_sensors = maze_sensors
        self.pose_tracker = pose_tracker
        self.yaw_records = []
        self.reference_yaw = None
        self.telemetry_started_at = time.monotonic()

        self.position_x_pid = PIDController(
            kp=settings.MOVE_X_KP,
            ki=settings.MOVE_X_KI,
            kd=settings.MOVE_X_KD,
            min_output=None,
            max_output=None,
        )
        self.position_y_pid = PIDController(
            kp=settings.MOVE_Y_KP,
            ki=settings.MOVE_Y_KI,
            kd=settings.MOVE_Y_KD,
            min_output=None,
            max_output=None,
        )

        self.pid_telemetry = None
        if settings.PID_TELEMETRY_ENABLED:
            self.pid_telemetry = PIDTelemetryRecorder(
                settings.PID_TELEMETRY_DIR,
                metadata={
                    "x_pid": {
                        "kp": settings.MOVE_X_KP,
                        "ki": settings.MOVE_X_KI,
                        "kd": settings.MOVE_X_KD,
                    },
                    "y_pid": {
                        "kp": settings.MOVE_Y_KP,
                        "ki": settings.MOVE_Y_KI,
                        "kd": settings.MOVE_Y_KD,
                    },
                    "move_speed_mps": settings.MOVE_SPEED_MPS,
                    "xy_output_limit_mode": "vector_magnitude",
                    "minimum_move_speed_mps": settings.MIN_MOVE_SPEED_MPS,
                    "position_tolerance_m": (
                        settings.MOVE_POSITION_TOLERANCE_M
                    ),
                    "frequency_hz": settings.MOVE_PID_FREQUENCY_HZ,
                },
            )

    @staticmethod
    def _angle_error(setpoint, measurement):
        return (setpoint - measurement + 180.0) % 360.0 - 180.0

    def __call__(self, target_x, target_y):
        """เดินตาม target x/y ซึ่งเป็นระยะสัมพัทธ์ในแกน chassis (เมตร)"""
        if not math.isfinite(target_x) or not math.isfinite(target_y):
            raise ValueError("target_x และ target_y ต้องเป็นตัวเลขที่มีค่าจำกัด")
        if target_x != 0.0 and target_y != 0.0:
            print('[Chassis] splitting X and Y into separate moves', flush=True)
            return self(target_x, 0.0) is True and self(0.0, target_y) is True
        print(f"[Chassis] target x={target_x:.2f}, y={target_y:.2f}")

        # Alignment เป็นขั้นตอนเดียวที่ปรับ yaw; move ใช้มุมอ้างอิงเพื่อแปลงแกนเท่านั้น
        start_x, start_y, yaw_before_move = self.pose_tracker.read()
        if self.reference_yaw is None:
            self.reference_yaw = yaw_before_move
            print(
                f"[Chassis PID] locked reference_yaw="
                f"{self.reference_yaw:.2f} deg"
            )
        yaw_setpoint = self.reference_yaw
        if (abs(self._angle_error(yaw_setpoint, yaw_before_move))
                > settings.MOVE_ARRIVAL_YAW_TOLERANCE_DEG):
            raise RuntimeError('Robot heading differs from reference before move; '
                               'refusing to move')
        yaw_radians = math.radians(yaw_setpoint)

        # x/y ระยะสั่งเป็นแกนตัวหุ่น แต่ pose position เป็นแกนโลก
        world_x_distance = (
            math.cos(yaw_radians) * target_x
            - math.sin(yaw_radians) * target_y
        )
        world_y_distance = (
            math.sin(yaw_radians) * target_x
            + math.cos(yaw_radians) * target_y
        )
        world_target_x = start_x + world_x_distance
        world_target_y = start_y + world_y_distance

        self.yaw_records.append(
            {
                "target_x": target_x,
                "target_y": target_y,
                "yaw_before_move": yaw_before_move,
                "yaw_setpoint": yaw_setpoint,
            }
        )
        print(
            f"[Chassis PID] start=({start_x:.3f},{start_y:.3f}) "
            f"target=({world_target_x:.3f},{world_target_y:.3f}) "
            f"yaw_before={yaw_before_move:.2f} deg "
            f"yaw_setpoint={yaw_setpoint:.2f} deg"
        )

        self.position_x_pid.reset()
        self.position_y_pid.reset()
        self.position_x_pid.prev_error = target_x
        self.position_y_pid.prev_error = target_y

        started_at = time.monotonic()
        previous_time = started_at
        period = 1.0 / settings.MOVE_PID_FREQUENCY_HZ
        move_status = "error"
        move_completed = False
        if self.pid_telemetry is not None:
            self.pid_telemetry.start_move(started_at)

        try:
            while True:
                loop_started = time.monotonic()
                current_x, current_y, current_yaw = self.pose_tracker.read()
                dt = max(loop_started - previous_time, 0.001)
                previous_time = loop_started

                x_error = world_target_x - current_x
                y_error = world_target_y - current_y
                yaw_error = self._angle_error(yaw_setpoint, current_yaw)
                delta_x = current_x - start_x
                delta_y = current_y - start_y
                reference_x = math.cos(yaw_radians) * delta_x + math.sin(yaw_radians) * delta_y
                reference_y = -math.sin(yaw_radians) * delta_x + math.cos(yaw_radians) * delta_y
                axis_x_error = target_x - reference_x
                axis_y_error = target_y - reference_y

                x_arrived = target_x == 0.0 or abs(axis_x_error) <= settings.MOVE_POSITION_TOLERANCE_M
                y_arrived = target_y == 0.0 or abs(axis_y_error) <= settings.MOVE_POSITION_TOLERANCE_M
                if x_arrived and y_arrived:
                    if self.pid_telemetry is not None:
                        self.pid_telemetry.record(
                            {
                                "time_s": loop_started
                                - self.telemetry_started_at,
                                "move_time_s": loop_started - started_at,
                                "dt_s": dt,
                                "relative_target_x_m": target_x,
                                "relative_target_y_m": target_y,
                                "start_x_m": start_x,
                                "start_y_m": start_y,
                                "target_x_m": world_target_x,
                                "target_y_m": world_target_y,
                                "position_x_m": current_x,
                                "position_y_m": current_y,
                                "error_x_m": x_error,
                                "error_y_m": y_error,
                                "yaw_before_move_deg": yaw_before_move,
                                "yaw_setpoint_deg": yaw_setpoint,
                                "yaw_deg": current_yaw,
                                "yaw_error_deg": yaw_error,
                                "world_command_x_mps": 0.0,
                                "world_command_y_mps": 0.0,
                                "chassis_command_x_mps": 0.0,
                                "chassis_command_y_mps": 0.0,
                                "chassis_command_z_dps": 0.0,
                                "sample_state": "reached",
                            }
                        )
                    move_status = "reached"
                    move_completed = True
                    print(
                        f"\n[Chassis PID] reached ({current_x:.3f},"
                        f"{current_y:.3f}) yaw={current_yaw:.2f}"
                    )
                    break

                if x_arrived:
                    x_speed = 0.0
                    self.position_x_pid.reset()
                else:
                    x_speed = self.position_x_pid.compute(
                        target_x,
                        reference_x,
                        dt,
                    )

                if y_arrived:
                    y_speed = 0.0
                    self.position_y_pid.reset()
                else:
                    y_speed = self.position_y_pid.compute(
                        target_y,
                        reference_y,
                        dt,
                    )

                translation_speed = math.hypot(x_speed, y_speed)
                if translation_speed > settings.MOVE_SPEED_MPS:
                    scale = settings.MOVE_SPEED_MPS / translation_speed
                    x_speed *= scale
                    y_speed *= scale
                elif 0.0 < translation_speed < settings.MIN_MOVE_SPEED_MPS:
                    scale = settings.MIN_MOVE_SPEED_MPS / translation_speed
                    x_speed *= scale
                    y_speed *= scale

                yaw_speed = 0.0
                current_yaw_radians = math.radians(current_yaw)
                world_x_speed = (math.cos(current_yaw_radians) * x_speed
                                 - math.sin(current_yaw_radians) * y_speed)
                world_y_speed = (math.sin(current_yaw_radians) * x_speed
                                 + math.cos(current_yaw_radians) * y_speed)

                if self.pid_telemetry is not None:
                    self.pid_telemetry.record(
                        {
                            "time_s": loop_started
                            - self.telemetry_started_at,
                            "move_time_s": loop_started - started_at,
                            "dt_s": dt,
                            "relative_target_x_m": target_x,
                            "relative_target_y_m": target_y,
                            "start_x_m": start_x,
                            "start_y_m": start_y,
                            "target_x_m": world_target_x,
                            "target_y_m": world_target_y,
                            "position_x_m": current_x,
                            "position_y_m": current_y,
                            "error_x_m": x_error,
                            "error_y_m": y_error,
                            "yaw_before_move_deg": yaw_before_move,
                            "yaw_setpoint_deg": yaw_setpoint,
                            "yaw_deg": current_yaw,
                            "yaw_error_deg": yaw_error,
                            "world_command_x_mps": world_x_speed,
                            "world_command_y_mps": world_y_speed,
                            "chassis_command_x_mps": x_speed,
                            "chassis_command_y_mps": y_speed,
                            "chassis_command_z_dps": yaw_speed,
                            "sample_state": "running",
                        }
                    )

                self.chassis.drive_speed(
                    x=x_speed,
                    y=y_speed,
                    z=yaw_speed,
                    timeout=None,
                )
                print(
                    f"\r[Chassis PID] error x={axis_x_error:.3f} "
                    f"y={axis_y_error:.3f} yaw={yaw_error:.2f} | "
                    f"cmd x={x_speed:.3f} y={y_speed:.3f} z={yaw_speed:.1f}",
                    end="",
                    flush=True,
                )

                elapsed = time.monotonic() - loop_started
                if elapsed < period:
                    time.sleep(period - elapsed)
        finally:
            try:
                stop_chassis(self.chassis)
            finally:
                self.maze_sensors.invalidate_cache()
                if self.pid_telemetry is not None:
                    self.pid_telemetry.finish_move(move_status)

        if move_completed:
            print(f'[Chassis] move complete; settling {settings.MOVE_SETTLE_S:.2f}s', flush=True)
            time.sleep(settings.MOVE_SETTLE_S)
            final_x, final_y, final_yaw = self.pose_tracker.read()
            final_delta_x = final_x - start_x
            final_delta_y = final_y - start_y
            final_reference_x = (math.cos(yaw_radians) * final_delta_x
                                 + math.sin(yaw_radians) * final_delta_y)
            final_reference_y = (-math.sin(yaw_radians) * final_delta_x
                                 + math.cos(yaw_radians) * final_delta_y)
            final_x_error = target_x - final_reference_x
            final_y_error = target_y - final_reference_y
            final_yaw_error = self._angle_error(yaw_setpoint, final_yaw)
            x_tolerance = (settings.MOVE_POSITION_TOLERANCE_M if target_x != 0
                           else settings.MOVE_CROSS_TRACK_TOLERANCE_M)
            y_tolerance = (settings.MOVE_POSITION_TOLERANCE_M if target_y != 0
                           else settings.MOVE_CROSS_TRACK_TOLERANCE_M)
            print(f'[Chassis verify] error x={final_x_error:.3f}m '
                  f'y={final_y_error:.3f}m yaw={final_yaw_error:.2f}deg', flush=True)
            if (abs(final_x_error) > x_tolerance
                    or abs(final_y_error) > y_tolerance
                    or abs(final_yaw_error) > settings.MOVE_ARRIVAL_YAW_TOLERANCE_DEG):
                raise RuntimeError('Move ended outside cell position/yaw tolerance; '
                                   'cell position is not confirmed')
            print('[Chassis] ready for next command', flush=True)
        return move_completed
