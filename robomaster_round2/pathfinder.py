"""
==============================================================================
Module: pathfinder.py
Description: Optimal shortest path finding on the 7x7 grid using the
             A* (A-Star) search algorithm with Manhattan distance heuristic.
==============================================================================
"""

import heapq
from typing import Tuple, List, Optional, Dict, Set
from .grid_map import GridMap


class PathFinder:
    """
    Finds the shortest collision-free path across the 7x7 GridMap
    using the A* search algorithm with Manhattan Distance Heuristic.
    """

    def __init__(self, grid_map: GridMap):
        self.grid_map: GridMap = grid_map

    @staticmethod
    def heuristic(cell_a: Tuple[int, int], cell_b: Tuple[int, int]) -> int:
        """
        Manhattan Distance: |r1 - r2| + |c1 - c2|
        Admissible and consistent heuristic for 4-directional grid movements.
        """
        return abs(cell_a[0] - cell_b[0]) + abs(cell_a[1] - cell_b[1])

    def find_path(
        self,
        start: Tuple[int, int],
        goal: Tuple[int, int],
    ) -> Optional[List[Tuple[int, int]]]:
        """
        Search for the optimal path from start to goal.
        
        Args:
            start: Starting grid coordinate (row, col)
            goal: Target grid coordinate (row, col)
            
        Returns:
            List of coordinates along the shortest path [(r0, c0), (r1, c1), ..., (rg, cg)],
            or None if no path exists.
        """
        # Validate coordinates
        if not self.grid_map.is_in_bounds(*start) or not self.grid_map.is_in_bounds(*goal):
            return None
        if self.grid_map.is_blocked(*start) or self.grid_map.is_blocked(*goal):
            return None
        if start == goal:
            return [start]

        # Priority Queue stores tuples: (f_score, g_score, cell)
        open_set = []
        heapq.heappush(open_set, (self.heuristic(start, goal), 0, start))

        # Maps each node to the predecessor on the cheapest path found
        came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}

        # g_score: Cost of the cheapest path from start to node
        g_score: Dict[Tuple[int, int], int] = {start: 0}

        # Set of evaluated nodes
        closed_set: Set[Tuple[int, int]] = set()

        while open_set:
            _, current_g, current = heapq.heappop(open_set)

            # Goal reached: reconstruct path
            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path

            if current in closed_set:
                continue
            closed_set.add(current)

            # Evaluate accessible orthogonal neighbors
            for neighbor in self.grid_map.get_valid_neighbors(current):
                if neighbor in closed_set:
                    continue

                tentative_g = current_g + 1  # Uniform step cost = 1
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, tentative_g, neighbor))

        return None  # Path is blocked or unreachable

    def get_path_distance(self, start: Tuple[int, int], goal: Tuple[int, int]) -> float:
        """
        Compute the exact step distance between two cells based on A* path.
        Returns float('inf') if goal is unreachable.
        """
        path = self.find_path(start, goal)
        if path is None:
            return float("inf")
        return float(len(path) - 1)
