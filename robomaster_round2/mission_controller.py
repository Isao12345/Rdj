"""
==============================================================================
Module: mission_controller.py
Description: Main mission orchestrator for Round 2.
             Executes dynamic target sequencing, navigates along A* paths,
             and triggers external handshake callbacks upon arriving at targets.
==============================================================================
"""

from typing import List, Tuple, Callable, Optional, Dict, Any, Set
import time

from . import config
from .grid_map import GridMap
from .pathfinder import PathFinder
from .target_sequencer import TargetSequencer
from .chassis_driver import BaseChassisDriver

# Handshake callback signature: callback(target_index, target_coordinate, telemetry_dict) -> any
TargetCallback = Callable[[int, Tuple[int, int], Dict[str, Any]], Any]


class MissionController:
    """
    Coordinates the autonomous navigation mission across the 7x7 grid.
    """

    def __init__(
        self,
        grid_map: GridMap,
        pathfinder: PathFinder,
        sequencer: TargetSequencer,
        driver: BaseChassisDriver,
        target_callback: Optional[TargetCallback] = None,
    ):
        self.grid_map: GridMap = grid_map
        self.pathfinder: PathFinder = pathfinder
        self.sequencer: TargetSequencer = sequencer
        self.driver: BaseChassisDriver = driver
        self.target_callback: Optional[TargetCallback] = target_callback

        self.visited_targets: Set[Tuple[int, int]] = set()

    def run_mission(
        self,
        dynamic_targets: List[Tuple[int, int]],
        start_pos: Tuple[int, int] = config.START_POSITION,
    ) -> Dict[str, Any]:
        """
        Execute the full mission:
          1. Connect chassis driver.
          2. Sequence target points with Greedy Nearest Neighbor.
          3. Calculate shortest A* paths and translate step by step (locked heading 0 deg).
          4. Invoke handshake callback at each destination target.
          5. Report mission summary and disconnect cleanly.
        """
        start_time = time.time()
        print("\n" + "=" * 65)
        print("   ROBOMASTER EP: ROUND 2 AUTONOMOUS GRID NAVIGATION")
        print("   (Mecanum Omnidirectional / Fixed Heading = 0 deg)")
        print("=" * 65)
        print(f"Arena Size       : {self.grid_map.rows}x{self.grid_map.cols} (Grid cell = {config.TILE_SIZE_M} m)")
        print(f"Start Position   : {start_pos}")
        print(f"Dynamic Targets  : {dynamic_targets} ({len(dynamic_targets)} points)")
        print(f"Simulation Mode  : {config.SIMULATION_MODE}")
        print(f"Obstacles/Walls  : {len(self.grid_map.blocked_cells)} cells, {len(self.grid_map.walls)} walls")
        print("=" * 65)

        # 1. Connect chassis driver
        self.driver.connect()

        # 2. Sequence targets using Greedy Nearest Neighbor
        print("\n[MISSION] Sequencing targets using Greedy Nearest Neighbor...")
        ordered_targets, est_steps = self.sequencer.sequence_targets(start_pos, dynamic_targets)
        print(f"[MISSION] Optimized Target Sequence: {ordered_targets}")
        print(f"[MISSION] Estimated Total Steps    : {int(est_steps)} steps ({est_steps * config.TILE_SIZE_M:.2f} m)")

        # Render initial arena map
        print("\n--- Initial Arena Map ---")
        print(self.grid_map.render_ascii(
            robot_pos=self.driver.position,
            targets=ordered_targets,
            visited_targets=self.visited_targets,
        ))
        print("Legend: [@] Robot, [T1..TN] Targets, [###] Obstacle, [:] Wall, [ ] Empty\n")

        # 3. Main navigation loop over targets
        try:
            for t_idx, target in enumerate(ordered_targets, start=1):
                curr_pos = self.driver.position
                print("\n" + "-" * 55)
                print(f"[*] Navigating to Target #{t_idx}/{len(ordered_targets)}: {target} from {curr_pos}")
                print("-" * 55)

                if curr_pos == target:
                    print(f"[MISSION] Robot is already at target {target}!")
                else:
                    # Calculate shortest path via A*
                    path = self.pathfinder.find_path(curr_pos, target)
                    if path is None:
                        print(f"[MISSION ERROR] Path to {target} is blocked!")
                        continue

                    print(f"[MISSION] Planned Path ({len(path) - 1} steps): {path}")

                    # Step along path (skip index 0 as it is the current position)
                    for step_idx in range(1, len(path)):
                        next_cell = path[step_idx]
                        now_cell = self.driver.position
                        dr = next_cell[0] - now_cell[0]
                        dc = next_cell[1] - now_cell[1]

                        # Command driver for one step
                        success = self.driver.move_step(dr, dc)
                        if not success:
                            raise RuntimeError(f"Step to {next_cell} failed!")

                        # Update simulation map display
                        if config.SIMULATION_MODE:
                            remaining_path = path[step_idx:]
                            print(self.grid_map.render_ascii(
                                robot_pos=self.driver.position,
                                targets=ordered_targets,
                                visited_targets=self.visited_targets,
                                planned_path=remaining_path,
                            ))

                # Arrived at target: halt chassis
                self.driver.stop()
                print(f"\n>>> ARRIVED AT TARGET #{t_idx}: {target} <<<")

                # Trigger handshake callback for external modules
                if self.target_callback:
                    print(f"[MISSION] Triggering external handshake callback for Target #{t_idx}...")
                    telemetry = self.driver.telemetry
                    try:
                        ack = self.target_callback(t_idx, target, telemetry)
                        print(f"[MISSION] Handshake response: {ack}")
                    except Exception as e:
                        print(f"[MISSION ERROR] External callback exception: {e}")

                # Dwell time at target
                if config.TARGET_WAIT_TIME > 0:
                    print(f"[MISSION] Pausing for {config.TARGET_WAIT_TIME}s at target...")
                    time.sleep(config.TARGET_WAIT_TIME)

                self.visited_targets.add(target)

            # 4. Mission completed summary
            elapsed = round(time.time() - start_time, 2)
            print("\n" + "=" * 65)
            print("                 ALL TARGETS VISITED!")
            print("=" * 65)
            print(f"Total Targets Visited : {len(self.visited_targets)} / {len(ordered_targets)}")
            print(f"Total Steps Taken     : {self.driver.total_steps} steps")
            print(f"Total Distance Moved  : {self.driver.total_distance_m:.2f} meters")
            print(f"Elapsed Mission Time  : {elapsed} seconds")
            print("=" * 65)

            # Render final completed map
            print("\n--- Final Completed Arena Map ---")
            print(self.grid_map.render_ascii(
                robot_pos=self.driver.position,
                targets=ordered_targets,
                visited_targets=self.visited_targets,
            ))
            print("Legend: [*] Completed Targets\n")

            return {
                "status": "COMPLETED",
                "targets_visited": list(self.visited_targets),
                "total_steps": self.driver.total_steps,
                "total_distance_m": self.driver.total_distance_m,
                "elapsed_time_s": elapsed,
            }

        finally:
            self.driver.disconnect()
