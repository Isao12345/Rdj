"""
==============================================================================
Module: target_sequencer.py
Description: Dynamic target list sequencing (N targets) using the Greedy
             Nearest Neighbor algorithm based on actual obstacle-aware A* distances.
==============================================================================
"""

from typing import Tuple, List, Optional
from .pathfinder import PathFinder


class TargetSequencer:
    """
    Sequences N dynamic target waypoints to optimize visitation order.
    """

    def __init__(self, pathfinder: PathFinder):
        self.pathfinder: PathFinder = pathfinder

    def sequence_targets(
        self,
        start_pos: Tuple[int, int],
        targets: List[Tuple[int, int]],
    ) -> Tuple[List[Tuple[int, int]], float]:
        """
        Sequence target waypoints using the Greedy Nearest Neighbor heuristic.
        
        Algorithm Steps:
          1. Begin at the robot's current location.
          2. Find the unvisited target with the shortest obstacle-aware A* distance.
          3. Append that target to the sequence and update the current reference position.
          4. Repeat until all reachable targets have been visited.
          
        Args:
            start_pos: Initial position of the robot (row, col)
            targets: List of target coordinates [(r1, c1), (r2, c2), ...]
            
        Returns:
            Tuple containing:
              - ordered_targets: List of targets in sequenced visit order
              - total_steps: Estimated total grid steps required for the full tour
        """
        if not targets:
            return [], 0.0

        remaining = list(targets)
        ordered_targets: List[Tuple[int, int]] = []
        current_pos = start_pos
        total_steps = 0.0

        while remaining:
            nearest_target: Optional[Tuple[int, int]] = None
            shortest_dist = float("inf")

            # Measure true A* path distance to each remaining unvisited target
            for target in remaining:
                dist = self.pathfinder.get_path_distance(current_pos, target)
                if dist < shortest_dist:
                    shortest_dist = dist
                    nearest_target = target

            # Handle unreachable targets blocked by obstacles
            if nearest_target is None or shortest_dist == float("inf"):
                print(f"[TargetSequencer WARNING] Unreachable targets remaining: {remaining}")
                ordered_targets.extend(remaining)
                break

            # Select the nearest target
            ordered_targets.append(nearest_target)
            total_steps += shortest_dist
            current_pos = nearest_target
            remaining.remove(nearest_target)

        return ordered_targets, total_steps
