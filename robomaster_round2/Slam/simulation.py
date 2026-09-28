"""สนามจำลอง ไม่ต้องเชื่อมต่อ RoboMaster SDK."""

from .slam import DIRECTIONS, SlamSession
from .setting import maze_setting as settings

def run_simulation(output):
    # Arena size follows config; the sample wall applies when inside the arena.
    blocked = frozenset(((4, 0), (4, 1)))
    def read_wall(row, col, direction):
        dr, dc, _ = DIRECTIONS[direction]
        target = row + dr, col + dc
        return (not (0 <= target[0] < settings.MAZE_ROWS
                     and 0 <= target[1] < settings.MAZE_COLS)
                or frozenset(((row, col), target)) == blocked)
    session = SlamSession(read_wall, output,
                          measurement_reader=lambda: {"source": "simulation"})
    session.explore(scan=lambda: None, move=lambda x, y: True)
    print(f"[Simulation] explored {len(session.visited)} cells; returned to {session.cell}")
