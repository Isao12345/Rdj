"""
==============================================================================
Module: pid_controller.py
Description: Proportional-Integral-Derivative (PID) controller implementations
             for position tracking (X/Y) and yaw heading lock (0 degrees).
==============================================================================
"""

import math
from typing import Optional


class PIDController:
    """
    Standard discrete-time PID Controller.
    Formula: Output = (Kp * e) + (Ki * ∫e dt) + (Kd * de/dt)
    """

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        min_output: Optional[float] = None,
        max_output: Optional[float] = None,
    ):
        """
        Initialize PID Controller parameters.
        
        Args:
            kp: Proportional gain (responds proportionally to current error)
            ki: Integral gain (eliminates residual steady-state error)
            kd: Derivative gain (dampens rate of change and reduces overshoot)
            min_output: Lower saturation limit for output clamping
            max_output: Upper saturation limit for output clamping
        """
        self.kp: float = float(kp)
        self.ki: float = float(ki)
        self.kd: float = float(kd)
        self.min_output: Optional[float] = min_output
        self.max_output: Optional[float] = max_output

        self.prev_error: float = 0.0
        self.integral: float = 0.0

    def compute(self, setpoint: float, measurement: float, dt: float) -> float:
        """
        Compute the PID control signal given the setpoint, current measurement, and dt.
        
        Args:
            setpoint: Desired target value
            measurement: Current measured sensor value
            dt: Time elapsed since the previous update (seconds)
            
        Returns:
            output: Clamped control output value
        """
        error = setpoint - measurement

        # 1. Proportional term
        p_term = self.kp * error

        # 2. Integral term
        if dt > 0:
            self.integral += error * dt
        i_term = self.ki * self.integral

        # 3. Derivative term
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        d_term = self.kd * derivative

        self.prev_error = error
        output = p_term + i_term + d_term

        # Clamp output within defined limits
        if self.min_output is not None:
            output = max(self.min_output, output)
        if self.max_output is not None:
            output = min(self.max_output, output)

        return output

    def reset(self) -> None:
        """Reset internal error and integral accumulator."""
        self.prev_error = 0.0
        self.integral = 0.0


def calculate_angle_error(setpoint_deg: float, measurement_deg: float) -> float:
    """
    Calculate the shortest angular difference (error) wrapped in [-180.0, 180.0] degrees.
    Prevents discontinuity issues around 0/360 boundary.
    """
    return (setpoint_deg - measurement_deg + 180.0) % 360.0 - 180.0


class ChassisHeadingPID:
    """
    Controller dedicated to locking the chassis heading at 0 degrees.
    """

    def __init__(self, kp: float = 0.8, ki: float = 0.0, kd: float = 0.05, max_yaw_dps: float = 30.0):
        self.pid = PIDController(
            kp=kp,
            ki=ki,
            kd=kd,
            min_output=-max_yaw_dps,
            max_output=max_yaw_dps,
        )
        self.target_yaw_deg: float = 0.0

    def compute_yaw_speed(self, current_yaw_deg: float, dt: float) -> float:
        """
        Compute angular velocity along Z-axis (deg/s) to correct heading toward 0 degrees.
        """
        yaw_error = calculate_angle_error(self.target_yaw_deg, current_yaw_deg)
        yaw_speed = self.pid.compute(setpoint=0.0, measurement=-yaw_error, dt=dt)
        return yaw_speed

    def reset(self) -> None:
        """Reset heading PID history."""
        self.pid.reset()
