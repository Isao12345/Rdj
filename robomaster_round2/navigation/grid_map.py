"""
==============================================================================
Module: grid_map.py
Description: GridMap representation for the 7x7 competition arena.
             Handles boundary checking, cell obstacles, edge walls, and
             ASCII terminal map rendering.
==============================================================================
"""

from typing import Tuple, List, Set, FrozenSet, Optional, Dict, Iterable
from .. import config


class GridMap:
    """
    Manages the 7x7 competition grid map:
    - Stores blocked obstacle cells (Blocked Cells)
    - Stores barrier walls between adjacent cells (Edge Walls)
    - Validates orthogonal 4-directional transitions (Orthogonal Neighbors)
    """

    def __init__(self, rows: int = config.GRID_ROWS, cols: int = config.GRID_COLS):
        self.rows: int = rows
        self.cols: int = cols
        
        # Blocked cells: set of (row, col) coordinates
        self.blocked_cells: Set[Tuple[int, int]] = set()
        
        # Edge walls between two adjacent cells: set of frozenset({(r1, c1), (r2, c2)})
        self.walls: Set[FrozenSet[Tuple[int, int]]] = set()

    def is_in_bounds(self, r: int, c: int) -> bool:
        """Check if coordinates (r, c) reside within grid boundaries."""
        return 0 <= r < self.rows and 0 <= c < self.cols

    def is_blocked(self, r: int, c: int) -> bool:
        """Check if cell (r, c) is an impassable obstacle or out of bounds."""
        if not self.is_in_bounds(r, c):
            return True
        return (r, c) in self.blocked_cells

    def has_wall(self, cell_a: Tuple[int, int], cell_b: Tuple[int, int]) -> bool:
        """Check if a barrier wall exists between two adjacent cells."""
        return frozenset([cell_a, cell_b]) in self.walls

    def can_move(self, cell_a: Tuple[int, int], cell_b: Tuple[int, int]) -> bool:
        """
        Determine if the robot can translate directly from cell_a to cell_b.
        Conditions:
          - Both cells must be in bounds
          - Neither cell is an obstacle
          - Must be orthogonal neighbors (Manhattan distance == 1)
          - No barrier wall exists between them
        """
        r1, c1 = cell_a
        r2, c2 = cell_b

        # 1. Bounds verification
        if not self.is_in_bounds(r1, c1) or not self.is_in_bounds(r2, c2):
            return False

        # 2. Obstacle verification
        if self.is_blocked(r1, c1) or self.is_blocked(r2, c2):
            return False

        # 3. Orthogonal neighbor verification (strictly non-diagonal)
        if abs(r1 - r2) + abs(c1 - c2) != 1:
            return False

        # 4. Wall barrier verification
        if self.has_wall(cell_a, cell_b):
            return False

        return True

    def get_valid_neighbors(self, cell: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        Retrieve accessible orthogonal neighbors (Up, Down, Left, Right).
        Diagonal transitions are excluded to maintain straight Mecanum translation.
        """
        r, c = cell
        candidates = [
            (r - 1, c),  # Up
            (r + 1, c),  # Down
            (r, c - 1),  # Left
            (r, c + 1),  # Right
        ]
        return [nbr for nbr in candidates if self.can_move(cell, nbr)]

    def add_blocked_cell(self, r: int, c: int) -> None:
        """Register a single cell as an obstacle."""
        if self.is_in_bounds(r, c):
            self.blocked_cells.add((r, c))

    def add_blocked_cells(self, cells: Iterable[Tuple[int, int]]) -> None:
        """Register multiple cells as obstacles."""
        for r, c in cells:
            self.add_blocked_cell(r, c)

    def add_wall(self, cell_a: Tuple[int, int], cell_b: Tuple[int, int]) -> None:
        """Register an edge wall barrier between two adjacent cells."""
        self.walls.add(frozenset([cell_a, cell_b]))

    def render_ascii(
        self,
        robot_pos: Optional[Tuple[int, int]] = None,
        targets: Optional[List[Tuple[int, int]]] = None,
        visited_targets: Optional[Set[Tuple[int, int]]] = None,
        planned_path: Optional[List[Tuple[int, int]]] = None,
    ) -> str:
        """
        Render a clean ASCII diagram of the 7x7 grid.
        Symbols:
            [@] : Current robot position
            [T1], [T2] : Pending targets
            [*] : Completed/visited targets
            [###] : Obstacle cell
            [.] : Planned path waypoints
            [ ] : Free navigable cell
        """
        targets = targets or []
        visited = visited_targets or set()
        path_set = set(planned_path) if planned_path else set()

        target_map: Dict[Tuple[int, int], str] = {}
        for idx, t in enumerate(targets):
            target_map[t] = "*" if t in visited else f"T{idx + 1}"

        lines = []
        # Column header labels C0 .. C6
        col_header = "     " + "   ".join(f"C{c}" for c in range(self.cols))
        lines.append(col_header)
        lines.append("   +" + "---+" * self.cols)

        for r in range(self.rows):
            row_str = f"R{r} |"
            for c in range(self.cols):
                cell = (r, c)

                if cell == robot_pos:
                    content = " @ "
                elif cell in target_map:
                    label = target_map[cell]
                    content = f" {label} " if len(label) == 1 else f"{label} "
                elif cell in self.blocked_cells:
                    content = "###"
                elif cell in path_set:
                    content = " . "
                else:
                    content = "   "

                # Check vertical wall on the right side of the cell
                has_v_wall = self.has_wall(cell, (r, c + 1)) if c < self.cols - 1 else False
                wall_char = "|" if has_v_wall or c == self.cols - 1 else ":"
                row_str += content + wall_char

            lines.append(row_str)

            # Check horizontal wall on the bottom side of the cell
            if r < self.rows - 1:
                boundary = "   +"
                for c in range(self.cols):
                    has_h_wall = self.has_wall((r, c), (r + 1, c))
                    boundary += "===+" if has_h_wall else "---+"
                lines.append(boundary)
            else:
                lines.append("   +" + "---+" * self.cols)

        return "\n".join(lines)
