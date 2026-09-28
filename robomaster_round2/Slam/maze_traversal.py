"""DFS exploration; traverse only edges explicitly observed as open."""

DIRECTIONS = {"N": (-1, 0), "E": (0, 1), "S": (1, 0), "W": (0, -1)}


class MazeTraversal:
    def __init__(self, start):
        self.stack = [start]

    def next_cell(self, maze, visited):
        row, col = self.stack[-1]
        for direction, (dr, dc) in DIRECTIONS.items():
            target = row + dr, col + dc
            if (0 <= target[0] < len(maze) and 0 <= target[1] < len(maze[0])
                    and maze[row][col][direction] is False and target not in visited):
                return target
        if len(self.stack) > 1:
            target = self.stack[-2]
            direction = next(d for d, (dr, dc) in DIRECTIONS.items()
                             if (row + dr, col + dc) == target)
            if maze[row][col][direction] is not False:
                raise RuntimeError("Return path is no longer confirmed open")
            return target
        return None

    def arrived(self, target):
        if len(self.stack) > 1 and target == self.stack[-2]:
            self.stack.pop()
        else:
            self.stack.append(target)
