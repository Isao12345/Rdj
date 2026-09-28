import csv
import json
from datetime import datetime
from pathlib import Path


FIELDNAMES = (
    "move_index",
    "sample_index",
    "time_s",
    "move_time_s",
    "dt_s",
    "relative_target_x_m",
    "relative_target_y_m",
    "start_x_m",
    "start_y_m",
    "target_x_m",
    "target_y_m",
    "position_x_m",
    "position_y_m",
    "error_x_m",
    "error_y_m",
    "yaw_before_move_deg",
    "yaw_setpoint_deg",
    "yaw_deg",
    "yaw_error_deg",
    "world_command_x_mps",
    "world_command_y_mps",
    "chassis_command_x_mps",
    "chassis_command_y_mps",
    "chassis_command_z_dps",
    "sample_state",
    "move_status",
)


class PIDTelemetryRecorder:
    """เก็บข้อมูลลูป PID ในหน่วยความจำและเขียน CSV เมื่อจบแต่ละก้าว"""

    def __init__(self, base_directory, metadata):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        self.run_directory = Path(base_directory) / f"run_{timestamp}"
        self.csv_path = self.run_directory / "pid_telemetry.csv"
        self.metadata_path = self.run_directory / "pid_metadata.json"
        self.run_directory.mkdir(parents=True, exist_ok=False)

        self._move_index = 0
        self._samples = []
        self._run_started_at = None

        with self.metadata_path.open("w", encoding="utf-8") as metadata_file:
            json.dump(
                metadata,
                metadata_file,
                ensure_ascii=False,
                indent=2,
                allow_nan=False,
            )
            metadata_file.write("\n")

        print(f"[PID Log] recording to {self.csv_path}")

    def start_move(self, run_started_at):
        if self._run_started_at is None:
            self._run_started_at = run_started_at
        self._move_index += 1
        self._samples = []

    def record(self, sample):
        row = {field: "" for field in FIELDNAMES}
        row.update(sample)
        row["move_index"] = self._move_index
        row["sample_index"] = len(self._samples)
        self._samples.append(row)

    def finish_move(self, status):
        if not self._samples:
            return

        file_exists = self.csv_path.exists()
        with self.csv_path.open("a", newline="", encoding="utf-8") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=FIELDNAMES)
            if not file_exists:
                writer.writeheader()
            for row in self._samples:
                row["move_status"] = status
                writer.writerow(row)

        self._samples = []
        print(f"[PID Log] move={self._move_index} status={status}")
