"""
==============================================================================
Module: chassis_driver.py
Description: Hardware Abstraction Layer (HAL) for the Mecanum chassis.
             Supports both software simulation and physical RoboMaster EP robot,
             incorporating PID control for locked yaw heading (0 degrees).
==============================================================================
"""

from abc import ABC, abstractmethod
from typing import Tuple, Dict, Any
import time

from . import config
from .pid_controller import PIDController, ChassisHeadingPID, calculate_angle_error


class BaseChassisDriver(ABC):
    """
    Abstract Base Class defining the standard interface for chassis drivers.
    """

    def __init__(self, start_pos: Tuple[int, int] = config.START_POSITION):
        self.current_grid_pos: Tuple[int, int] = start_pos
        self.real_x_m: float = start_pos[0] * config.TILE_SIZE_M
        self.real_y_m: float = start_pos[1] * config.TILE_SIZE_M
        self.total_distance_m: float = 0.0
        self.total_steps: int = 0
        self.is_connected: bool = False

    @abstractmethod
    def connect(self) -> bool:
        """Establish connection to the chassis hardware or simulation."""
        pass

    @abstractmethod
    def disconnect(self) -> None:
        """Safely disconnect and stop actuator commands."""
        pass

    @abstractmethod
    def move_step(self, dr: int, dc: int) -> bool:
        """
        Execute an omnidirectional step of 1 tile (0.60 m) along (dr, dc)
        while strictly locking heading at 0 degrees.
        """
        pass

    @abstractmethod
    def stop(self) -> None:
        """Immediately halt chassis motion."""
        pass

    @property
    def position(self) -> Tuple[int, int]:
        """Return the current grid coordinates (row, col)."""
        return self.current_grid_pos

    @property
    def telemetry(self) -> Dict[str, Any]:
        """Return a snapshot of current telemetry data."""
        return {
            "grid_pos": self.current_grid_pos,
            "real_x_m": round(self.real_x_m, 3),
            "real_y_m": round(self.real_y_m, 3),
            "total_distance_m": round(self.total_distance_m, 2),
            "total_steps": self.total_steps,
            "heading_deg": config.LOCKED_HEADING_DEG,
        }


# ==============================================================================
# Simulated Chassis Driver for Terminal Testing
# ==============================================================================
class SimulatedChassisDriver(BaseChassisDriver):
    """
    Software simulated chassis driver for testing on the terminal without real robot hardware.
    """

    def __init__(self, start_pos: Tuple[int, int] = config.START_POSITION):
        super().__init__(start_pos)
        self.heading_pid = ChassisHeadingPID(
            kp=config.PID_YAW_KP,
            ki=config.PID_YAW_KI,
            kd=config.PID_YAW_KD,
            max_yaw_dps=config.MAX_YAW_SPEED_DPS,
        )

    def connect(self) -> bool:
        self.is_connected = True
        print(f"[SIM DRIVER] Initialized at Grid {self.current_grid_pos} | Heading locked at {config.LOCKED_HEADING_DEG} deg")
        return True

    def disconnect(self) -> None:
        self.stop()
        self.is_connected = False
        print("[SIM DRIVER] Disconnected cleanly.")

    def move_step(self, dr: int, dc: int) -> bool:
        if not self.is_connected:
            raise RuntimeError("Driver is not connected. Call connect() first.")

        dx, dy, action = config.get_motion_vector(dr, dc)
        step_dist = config.TILE_SIZE_M

        new_r = self.current_grid_pos[0] + dr
        new_c = self.current_grid_pos[1] + dc

        # Simulation animation delay
        if config.SIM_STEP_DELAY > 0:
            time.sleep(config.SIM_STEP_DELAY)

        self.current_grid_pos = (new_r, new_c)
        self.real_x_m += dx
        self.real_y_m += dy
        self.total_distance_m += step_dist
        self.total_steps += 1

        print(
            f"[SIM DRIVER] Step #{self.total_steps}: {action:12s} "
            f"| d(r,c)=({dr:+d},{dc:+d}) -> Cmd(dx={dx:+.2f}m, dy={dy:+.2f}m, z=0 deg) "
            f"| Pos: {self.current_grid_pos}"
        )
        return True

    def stop(self) -> None:
        pass


# ==============================================================================
# Physical DJI RoboMaster EP Chassis Driver
# ==============================================================================
class RoboMasterChassisDriver(BaseChassisDriver):
    """
    Hardware driver communicating with the physical robot via the official robomaster SDK.
    Uses chassis.move() with relative 0.60m step translations and zero yaw rotation (z=0).
    """

    def __init__(self, start_pos: Tuple[int, int] = config.START_POSITION):
        super().__init__(start_pos)
        self.ep_robot = None
        self.chassis = None

        # PID controllers for position and yaw correction
        self.pid_x = PIDController(config.PID_POS_KP, config.PID_POS_KI, config.PID_POS_KD)
        self.pid_y = PIDController(config.PID_POS_KP, config.PID_POS_KI, config.PID_POS_KD)
        self.heading_pid = ChassisHeadingPID(
            kp=config.PID_YAW_KP,
            ki=config.PID_YAW_KI,
            kd=config.PID_YAW_KD,
            max_yaw_dps=config.MAX_YAW_SPEED_DPS,
        )

    def connect(self) -> bool:
        print("[ROBOT DRIVER] Connecting to DJI RoboMaster EP...")
        try:
            import robomaster
            from robomaster import robot
        except ImportError as e:
            raise ImportError(
                "Missing 'robomaster' library.\n"
                "Please install via: pip install robomaster\n"
                "Or enable simulation mode in config.py: SIMULATION_MODE = True"
            ) from e

        try:
            robomaster.config.DEFAULT_CONN_TYPE = config.ROBOT_CONN_TYPE
            self.ep_robot = robot.Robot()
            self.ep_robot.initialize(conn_type=config.ROBOT_CONN_TYPE)
            self.chassis = self.ep_robot.chassis
            self.is_connected = True
            print(
                f"[ROBOT DRIVER] Connected successfully! (mode: {config.ROBOT_CONN_TYPE}) "
                f"Locked heading at {config.LOCKED_HEADING_DEG} deg"
            )
            return True
        except Exception as e:
            print(f"[ROBOT DRIVER ERROR] Connection failed: {e}")
            self.is_connected = False
            raise

    def disconnect(self) -> None:
        print("[ROBOT DRIVER] Disconnecting...")
        self.stop()
        if self.ep_robot is not None:
            try:
                self.ep_robot.close()
            except Exception as e:
                print(f"[ROBOT DRIVER WARNING] Error during close: {e}")
            self.ep_robot = None
            self.chassis = None
        self.is_connected = False
        print("[ROBOT DRIVER] Disconnected cleanly.")

    def move_step(self, dr: int, dc: int) -> bool:
        """
        Command the robot to translate 1 grid step (0.60 m).
        Maintains heading strictly at z = 0 degrees (no yaw turning).
        """
        if not self.is_connected or self.chassis is None:
            raise RuntimeError("Robot chassis is not connected.")

        dx, dy, action = config.get_motion_vector(dr, dc)
        step_dist = config.TILE_SIZE_M

        print(
            f"[ROBOT DRIVER] Executing: {action} "
            f"| dx={dx:+.2f}m, dy={dy:+.2f}m, z=0 deg @ speed {config.CHASSIS_SPEED} m/s"
        )

        try:
            action_handler = self.chassis.move(
                x=dx,
                y=dy,
                z=0.0,
                xy_speed=config.CHASSIS_SPEED,
            )
            action_handler.wait_for_completed(timeout=config.CHASSIS_TIMEOUT)

            self.current_grid_pos = (self.current_grid_pos[0] + dr, self.current_grid_pos[1] + dc)
            self.real_x_m += dx
            self.real_y_m += dy
            self.total_distance_m += step_dist
            self.total_steps += 1
            return True
        except Exception as e:
            print(f"[ROBOT DRIVER ERROR] Step execution error: {e}")
            self.stop()
            return False

    def stop(self) -> None:
        """Stop chassis translation immediately."""
        if self.chassis is not None:
            try:
                self.chassis.drive_speed(x=0.0, y=0.0, z=0.0)
            except Exception as e:
                print(f"[ROBOT DRIVER WARNING] Stop error: {e}")


def create_chassis_driver(
    simulation_mode: bool = config.SIMULATION_MODE,
    start_pos: Tuple[int, int] = config.START_POSITION,
) -> BaseChassisDriver:
    """Factory function instantiating the appropriate ChassisDriver instance."""
    if simulation_mode:
        return SimulatedChassisDriver(start_pos=start_pos)
    else:
        return RoboMasterChassisDriver(start_pos=start_pos)
