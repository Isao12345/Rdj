"""
==============================================================================
Module: config.py
Description: Centralized system configuration for the RoboMaster EP Round 2
             navigation system (grid dimensions, speed, PID gains, modes).
==============================================================================
"""

from typing import Tuple

# ==============================================================================
# 1. Operation Mode Configuration
# ==============================================================================
# True  : Terminal ASCII simulation mode (no physical robot hardware required)
# False : Physical DJI RoboMaster EP robot mode via the robomaster SDK
SIMULATION_MODE: bool = True

# ==============================================================================
# 2. Arena and Grid System Specifications
# ==============================================================================
# Grid dimensions: 7 rows x 7 columns (Row 0..6, Col 0..6, total 49 cells)
GRID_ROWS: int = 7
GRID_COLS: int = 7

# Tile dimension: square tiles of 0.60 meters (60 cm) each
TILE_SIZE_M: float = 0.60

# Starting coordinate for the robot (row, col)
START_POSITION: Tuple[int, int] = (0, 0)

# ==============================================================================
# 3. Mecanum Locomotion Kinematics
# ==============================================================================
# Linear translation speed (meters per second)
CHASSIS_SPEED: float = 0.5

# Fixed heading angle in degrees (0.0 deg: locked orientation, zero rotation)
LOCKED_HEADING_DEG: float = 0.0

# Command execution timeout per grid step (seconds)
CHASSIS_TIMEOUT: float = 5.0

# ==============================================================================
# 4. PID Controller Gains
# ==============================================================================
# PID gains for position tracking (X / Y translation axes)
PID_POS_KP: float = 1.2
PID_POS_KI: float = 0.0
PID_POS_KD: float = 0.05

# PID gains for heading lock (Yaw axis keeping heading at 0 degrees)
PID_YAW_KP: float = 0.8
PID_YAW_KI: float = 0.0
PID_YAW_KD: float = 0.05
MAX_YAW_SPEED_DPS: float = 30.0  # Maximum angular correction speed (deg/s)

# ==============================================================================
# 5. RoboMaster EP Hardware Connection
# ==============================================================================
# "ap"  : Robot Wi-Fi Direct mode (computer connects to robot's Wi-Fi network)
# "sta" : Router mode (both computer and robot connect to same Wi-Fi router)
ROBOT_CONN_TYPE: str = "ap"

# ==============================================================================
# 6. Timing and Delays
# ==============================================================================
# Animation delay between steps in simulation mode (seconds)
SIM_STEP_DELAY: float = 0.20

# Dwell / signal transmission pause time at each target (seconds)
TARGET_WAIT_TIME: float = 1.0


# ==============================================================================
# Grid Direction Delta to Chassis Displacement Vector Mapping
# ==============================================================================
def get_motion_vector(dr: int, dc: int) -> Tuple[float, float, str]:
    """
    Converts grid displacement (dr, dc) into chassis relative translation (dx, dy)
    and an action description string.
    
    RoboMaster SDK chassis coordinate convention:
      dx > 0: Forward
      dx < 0: Backward
      dy > 0: Strafe Left
      dy < 0: Strafe Right
      
    Assuming the robot faces +Row (looking down the grid):
      dr = +1, dc =  0 -> Forward      (dx = +0.60m, dy =  0.00m)
      dr = -1, dc =  0 -> Backward     (dx = -0.60m, dy =  0.00m)
      dr =  0, dc = -1 -> Strafe Left  (dx =  0.00m, dy = +0.60m)
      dr =  0, dc = +1 -> Strafe Right (dx =  0.00m, dy = -0.60m)
    """
    if dr == 1 and dc == 0:
        return (TILE_SIZE_M, 0.0, "FORWARD")
    elif dr == -1 and dc == 0:
        return (-TILE_SIZE_M, 0.0, "BACKWARD")
    elif dr == 0 and dc == -1:
        return (0.0, TILE_SIZE_M, "STRAFE_LEFT")
    elif dr == 0 and dc == 1:
        return (0.0, -TILE_SIZE_M, "STRAFE_RIGHT")
    else:
        raise ValueError(f"Invalid grid step delta: dr={dr}, dc={dc} (only 4-cardinal steps supported)")
