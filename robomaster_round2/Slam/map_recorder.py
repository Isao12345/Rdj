import json
import math
from datetime import datetime, timezone
from pathlib import Path


class MazeMapRecorder:
    """บันทึก logical maze, เส้นทาง, pose และค่าเซนเซอร์เป็น JSON"""

    def __init__(
        self,
        output_path,
        rows,
        cols,
        start,
        goal,
        start_direction,
        cell_distance_m,
        right_slide_sign=1.0,
        pose_reader=None,
        measurement_reader=None,
    ):
        self.output_path = Path(output_path)
        self.pose_reader = pose_reader
        self.measurement_reader = measurement_reader
        self.data = {
            "version": 1,
            "status": "ready",
            "started_at": None,
            "updated_at": None,
            "finished_at": None,
            "error": None,
            "rows": rows,
            "cols": cols,
            "start": list(start),
            "goal": list(goal),
            "start_direction": start_direction,
            "cell_distance_m": cell_distance_m,
            "right_slide_sign": right_slide_sign,
            "maze": None,
            "flood": None,
            "visited_cells": [],
            "path": [],
            "cells": {},
            "moves": [],
        }

    @staticmethod
    def _now():
        return datetime.now(timezone.utc).isoformat()

    @staticmethod
    def _cell_key(cell):
        return f"{cell[0]},{cell[1]}"

    @staticmethod
    def _copy_maze(maze):
        return [
            [dict(walls) for walls in row]
            for row in maze
        ]

    @staticmethod
    def _copy_flood(flood):
        return [
            [None if math.isinf(value) else value for value in row]
            for row in flood
        ]

    def _read_pose(self):
        if self.pose_reader is None:
            return None
        x, y, yaw = self.pose_reader()
        return {"x": x, "y": y, "yaw": yaw}

    def _read_measurements(self):
        if self.measurement_reader is None:
            return None
        measurements = self.measurement_reader()
        return None if measurements is None else dict(measurements)

    def start_run(self):
        now = self._now()
        self.data["status"] = "running"
        self.data["started_at"] = now
        self.data["updated_at"] = now
        self.data["finished_at"] = None
        self.data["error"] = None
        self.data["path"] = []
        self.data["visited_cells"] = []
        self.data["cells"] = {}
        self.data["moves"] = []
        self.save()
        print(f"[Map] recording to {self.output_path}")

    def record_cell(
        self,
        cell,
        walls,
        maze,
        flood,
        visited_cells,
        path,
        heading,
        navigation_mode,
    ):
        key = self._cell_key(cell)
        self.data["cells"][key] = {
            "row": cell[0],
            "col": cell[1],
            "walls": dict(walls),
            "heading": heading,
            "navigation_mode": navigation_mode,
            "pose": self._read_pose(),
            "measurements": self._read_measurements(),
            "sensed_at": self._now(),
        }
        self.data["maze"] = self._copy_maze(maze)
        self.data["flood"] = self._copy_flood(flood)
        self.data["visited_cells"] = [
            list(visited_cell)
            for visited_cell in sorted(visited_cells)
        ]
        self.data["path"] = [list(path_cell) for path_cell in path]
        self.save()

    def record_move(
        self,
        current_cell,
        target_cell,
        direction,
        target_x,
        target_y,
        path,
    ):
        self.data["moves"].append(
            {
                "from": list(current_cell),
                "to": list(target_cell),
                "direction": direction,
                "target_x": target_x,
                "target_y": target_y,
                "pose_after": self._read_pose(),
                "completed_at": self._now(),
            }
        )
        self.data["path"] = [list(path_cell) for path_cell in path]
        self.save()

    def finish(self, status, path, error=None):
        self.data["status"] = status
        self.data["path"] = [list(path_cell) for path_cell in path]
        self.data["error"] = error
        self.data["finished_at"] = self._now()
        self.save()
        print(f"[Map] saved {self.output_path} status={status}")

    def save(self):
        self.data["updated_at"] = self._now()
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        temporary_path = self.output_path.with_suffix(
            self.output_path.suffix + ".tmp"
        )
        with temporary_path.open("w", encoding="utf-8") as output_file:
            json.dump(
                self.data,
                output_file,
                ensure_ascii=False,
                indent=2,
                allow_nan=False,
            )
            output_file.write("\n")
        temporary_path.replace(self.output_path)
