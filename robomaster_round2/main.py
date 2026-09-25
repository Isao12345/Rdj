"""
==============================================================================
Module: main.py (robomaster_round2/main.py)
Description: Main execution entry point for the robomaster_round2 package.
             Supports direct script execution, command-line arguments,
             and dual simulation/real robot modes.
==============================================================================
"""

import os
import sys
import argparse
from typing import Tuple, Dict, Any

# Ensure project root is in sys.path for direct script execution
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

try:
    from . import config
    from .grid_map import GridMap
    from .pathfinder import PathFinder
    from .target_sequencer import TargetSequencer
    from .chassis_driver import create_chassis_driver
    from .mission_controller import MissionController
except ImportError:
    from robomaster_round2 import config
    from robomaster_round2.grid_map import GridMap
    from robomaster_round2.pathfinder import PathFinder
    from robomaster_round2.target_sequencer import TargetSequencer
    from robomaster_round2.chassis_driver import create_chassis_driver
    from robomaster_round2.mission_controller import MissionController


# ==============================================================================
# Example Handshake Callback connecting to external modules
# ==============================================================================
def external_handshake_callback(
    target_idx: int,
    target_coord: Tuple[int, int],
    telemetry: Dict[str, Any],
) -> Dict[str, Any]:
    """
    Demonstration handshake callback triggered when the robot reaches each target.
    
    In competition scenarios, this function interfaces with:
      1. Computer Vision module for tag / target recognition
      2. RF / Optical beacon transmission to the base station
      3. Gimbal aiming or projectile firing subsystem
    """
    print("\n   +-------------------------------------------------------------+")
    print(f"   | [EXTERNAL HANDSHAKE] Arrived at Target #{target_idx} @ {target_coord}        |")
    print(f"   | Robot Pose : X={telemetry['real_x_m']:.2f}m, Y={telemetry['real_y_m']:.2f}m, Steps={telemetry['total_steps']:2d}           |")
    print("   | Action     : Transmitting RF / Optical Beacon Signal...     |")
    print("   | Response   : Signal ACK (200 OK) received from Base Station!|")
    print("   +-------------------------------------------------------------+\n")

    return {
        "status": "ACK_SUCCESS",
        "target_idx": target_idx,
        "coord": target_coord,
    }


def run():
    """Main setup and execution flow."""
    parser = argparse.ArgumentParser(description="RoboMaster EP Round 2: Autonomous Grid Navigation")
    parser.add_argument(
        "--mode",
        choices=["sim", "real"],
        default="sim" if config.SIMULATION_MODE else "real",
        help="Run mode: 'sim' (terminal simulation) or 'real' (hardware connection)",
    )
    parser.add_argument(
        "--step-delay",
        type=float,
        default=config.SIM_STEP_DELAY,
        help="Delay between steps in simulation mode (seconds)",
    )
    args = parser.parse_args()

    # Apply command-line arguments
    config.SIMULATION_MODE = (args.mode == "sim")
    config.SIM_STEP_DELAY = args.step_delay

    # 1. Initialize 7x7 Grid Map
    arena_map = GridMap(rows=config.GRID_ROWS, cols=config.GRID_COLS)

    # 2. Add sample obstacles and barrier walls
    arena_map.add_blocked_cells([
        (1, 2), (2, 2),  # Vertical obstacle column
        (4, 3), (4, 4),  # Horizontal obstacle block
        (2, 5),          # Single obstacle
    ])
    arena_map.add_wall((5, 1), (5, 2))  # Edge wall between (5, 1) and (5, 2)

    # 3. Define Dynamic Target List (unordered N waypoints)
    dynamic_targets = [
        (1, 5),
        (5, 5),
        (6, 1),
        (3, 3),
        (0, 4),
    ]

    # 4. Instantiate subsystems
    pathfinder = PathFinder(grid_map=arena_map)
    sequencer = TargetSequencer(pathfinder=pathfinder)
    driver = create_chassis_driver(
        simulation_mode=config.SIMULATION_MODE,
        start_pos=config.START_POSITION,
    )

    # 5. Create Mission Controller and inject handshake callback
    controller = MissionController(
        grid_map=arena_map,
        pathfinder=pathfinder,
        sequencer=sequencer,
        driver=driver,
        target_callback=external_handshake_callback,
    )

    # 6. Execute mission
    return controller.run_mission(
        dynamic_targets=dynamic_targets,
        start_pos=config.START_POSITION,
    )


if __name__ == "__main__":
    run()
