"""Grid exploration using a four-direction ToF scan. Entry point: Slam/main.py.

Rows increase southward, columns eastward. Walls use None (unknown),
True (wall), False (open).
"""

import time

from .map_recorder import MazeMapRecorder
from .setting import maze_setting as settings

DIRECTIONS = {"N": (-1, 0, "S"), "E": (0, 1, "W"),
              "S": (1, 0, "N"), "W": (0, -1, "E")}


class SlamSession:
    """Scan the start cell using wall_reader(row, col, direction).

    The caller owns sensor startup/shutdown and heading configuration.
    run_once scans one cell; explore scans every reachable cell and returns home.
    """

    def __init__(self, wall_reader, output_path, pose_reader=None,
                 measurement_reader=None, on_map_ready=None):
        self.wall_reader = wall_reader
        self.on_map_ready = on_map_ready
        self.cell = settings.START_CELL
        self.maze = [
            [dict.fromkeys(DIRECTIONS) for _ in range(settings.MAZE_COLS)]
            for _ in range(settings.MAZE_ROWS)
        ]
        self.visited = set()
        self.path = [self.cell]
        self.max_steps = None
        self.recorder = MazeMapRecorder(
            output_path, settings.MAZE_ROWS, settings.MAZE_COLS,
            settings.START_CELL, settings.GOAL_CELL,
            settings.START_DIRECTION, settings.CELL_DISTANCE_M,
            pose_reader=pose_reader, measurement_reader=measurement_reader,
        )

    def scan_current_cell(self, observed=None):
        row, col = self.cell
        # Read and validate the full snapshot before changing the map.
        walls = ({d: self.wall_reader(row, col, d) for d in DIRECTIONS}
                 if observed is None else {d: observed[d] for d in DIRECTIONS})
        if any(value is not None and type(value) is not bool
               for value in walls.values()):
            raise ValueError("wall_reader must return True, False or None")
        for direction, value in walls.items():
            known = self.maze[row][col][direction]
            if value is not None and known is not None and value != known:
                raise RuntimeError(f"Conflicting wall at {self.cell} direction {direction}")
        for direction, value in walls.items():
            if value is None:
                continue
            self.maze[row][col][direction] = value
            dr, dc, opposite = DIRECTIONS[direction]
            nr, nc = row + dr, col + dc
            if 0 <= nr < settings.MAZE_ROWS and 0 <= nc < settings.MAZE_COLS:
                self.maze[nr][nc][opposite] = value
        self.visited.add(self.cell)
        self.recorder.record_cell(
            self.cell, self.maze[row][col], self.maze, [], self.visited,
            self.path, settings.START_DIRECTION, "stationary_scan",
        )
        return dict(self.maze[row][col])

    def run_once(self):
        self.recorder.start_run()
        try:
            if self.on_map_ready is not None:
                self.on_map_ready()
            walls = self.scan_current_cell()
        except BaseException as error:
            self.recorder.finish("failed", self.path, str(error) or type(error).__name__)
            raise
        self.recorder.finish("scan_complete", self.path)
        return walls


    def explore(self, scan, move, correct=None, verify_move=None):
        """scan() refreshes readings; move(x, y) must confirm arrival with True."""
        from .maze_traversal import MazeTraversal
        traversal = MazeTraversal(self.cell)
        self.recorder.start_run()
        try:
            if self.on_map_ready is not None:
                self.on_map_ready()
            while True:
                observed = scan()
                if correct is not None:
                    corrected = correct(observed)
                    if corrected is not None:
                        observed = corrected
                walls = self.scan_current_cell(observed)
                if any(value is None for value in walls.values()):
                    raise RuntimeError("Incomplete scan; cannot declare exploration complete")
                if self.max_steps is not None and len(self.path) - 1 >= self.max_steps:
                    self.recorder.finish("step_limit", self.path)
                    return
                target = traversal.next_cell(self.maze, self.visited)
                if target is None:
                    break
                dr, dc = target[0] - self.cell[0], target[1] - self.cell[1]
                x = -dr * settings.CELL_DISTANCE_M
                y = dc * settings.CELL_DISTANCE_M * settings.RIGHT_SLIDE_SIGN
                direction = next(d for d, (r, c, _) in DIRECTIONS.items()
                                 if (r, c) == (dr, dc))
                purpose = 'backtrack' if target in self.visited else 'explore'
                print(f'[Workflow] next cell={target} direction={direction} '
                      f'purpose={purpose}', flush=True)
                print(
                    f'[Pre-move] from={self.cell} to={target} '
                    f'direction={direction} x={x:.2f}m y={y:.2f}m',
                    flush=True,
                )
                if settings.PRE_MOVE_DELAY_S > 0:
                    print(
                        f'[Pre-move] waiting {settings.PRE_MOVE_DELAY_S:.1f}s '
                        'before moving', flush=True)
                    time.sleep(settings.PRE_MOVE_DELAY_S)
                if verify_move is not None and not verify_move(direction, self.cell, target):
                    raise RuntimeError(
                        f'Planned exit {direction} from {self.cell} to {target} '
                        'is blocked after alignment; refusing to move')
                if move(x, y) is not True:
                    raise RuntimeError("Move did not reach target; cell position is uncertain")
                previous = self.cell
                self.cell = target
                self.path.append(target)
                traversal.arrived(target)
                self.recorder.record_move(previous, target, direction, x, y, self.path)
        except BaseException as error:
            self.recorder.finish("failed", self.path, str(error) or type(error).__name__)
            raise
        self.recorder.finish("exploration_complete", self.path)


# Compatibility for existing callers; CLI and setup now live in main.py.
def run_simulation(output):
    from .main import run_simulation as run
    return run(output)


def run_real(output, scan_only=False, max_steps=None):
    from .main import run_real as run
    return run(output, scan_only, max_steps)


def main():
    from .main import main as run
    return run()


if __name__ == "__main__":
    main()
