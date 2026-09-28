"""ค่าตั้งทั้งหมดสำหรับการเดินเขาวงกตและฮาร์ดแวร์เซนเซอร์"""

# ------------------------- เขาวงกต -------------------------
MAZE_ROWS = 5
MAZE_COLS = 4
START_CELL = (4, 0)
GOAL_CELL = (0, 3)

# หุ่นไม่หมุน 90 องศา: N=หน้า, E=ขวา, S=หลัง, W=ซ้าย
START_DIRECTION = "N"

# เขียนทับไฟล์นี้ระหว่างเดิน เพื่อให้เปิดดูแผนที่ล่าสุดได้เสมอ
MAP_OUTPUT_PATH = "maps/maze_map.json"

# ------------------------- PID เดินข้ามช่อง -------------------------
CELL_DISTANCE_M = 0.61
PRE_MOVE_DELAY_S = 3.0

# หยุดฉุกเฉินด้วย Ctrl+C; ไม่มี front-stop หรือ timeout การเดิน

# ค่าบวกของแกน y คือสไลด์ขวา จากผลทดสอบ chassis ตัวจริง
RIGHT_SLIDE_SIGN = 1.0

# ------------------------- PID จัดกลางช่อง -------------------------
ALIGNMENT_ENABLED = True
# PID gain เรียง (Kp, Ki, Kd)
ALIGNMENT_FRONT_PID = (0.025 , 0.01, 0.0)
ALIGNMENT_YAW_PID = (2, 0.1, 0.5)
FRONT_WALL_THRESHOLD_CM = 20.0
TARGET_FRONT_WALL_CM = 15.0
POSITION_TOLERANCE_CM = 0.5
PARALLEL_TOLERANCE_CM = 0.005
MAX_ALIGNMENT_SPEED_MPS = 0.40
MIN_ALIGNMENT_SPEED_MPS = 0.04
MAX_ALIGNMENT_YAW_DPS = 20.0
MIN_ALIGNMENT_YAW_DPS = 8.0
ALIGNMENT_FREQUENCY_HZ = 20
# พักหลังหยุด chassis ก่อนสแกนหรือสั่งงานถัดไป (วินาที)
ALIGNMENT_SETTLE_S = 1

# ------------------------- เซนเซอร์ -------------------------
TOF_FREQUENCY_HZ = 20
SENSOR_TIMEOUT_S = 1.0

# Sharp calibration: ADC -> voltage -> ระยะ cm
SHARP_ADC_MAX = 1023
SHARP_ADC_REFERENCE_V = 3.3
SHARP_VALID_MIN_CM = 4.0
SHARP_VALID_MAX_CM = 30.0
# Median ของค่าล่าสุดต่อเซนเซอร์ ลดค่ากระโดดโดยไม่รอเก็บเป็นชุด
SHARP_FILTER_WINDOW = 3
SHARP_REGRESSION_SLOPE = 12.1967
SHARP_REGRESSION_INTERCEPT = 0.0705
SHARP_DISTANCE_OFFSET_CM = 0.42

# Sharp IR แบบ ADC ที่ติดด้านซ้าย
LEFT_FRONT_BOARD = 1
LEFT_FRONT_PORT = 1
LEFT_BACK_BOARD = 2
LEFT_BACK_PORT = 1

# IR digital ด้านขวาและด้านหลัง
RIGHT_IR_BOARD = 3
RIGHT_IR_PORT = 2
BACK_IR_BOARD = 4
BACK_IR_PORT = 1

# IR sensor คืน 0 เมื่อพบกำแพง
IR_WALL_VALUE = 0


# ค่าเริ่มต้นสำหรับ movement/pose_tracker ยังไม่ได้จูนหรือยืนยันทิศ yaw กับหุ่นจริง
MOVE_X_KP = 1.5
MOVE_X_KI = 0.0
MOVE_X_KD = 0.05    
MOVE_Y_KP = 1.5
MOVE_Y_KI = 0.0
MOVE_Y_KD = 0.05
MOVE_SPEED_MPS = 0.30
MIN_MOVE_SPEED_MPS = 0.06
MOVE_POSITION_TOLERANCE_M = 0.005
MOVE_CROSS_TRACK_TOLERANCE_M = 0.02
MOVE_ARRIVAL_YAW_TOLERANCE_DEG = 5.0
MOVE_PID_FREQUENCY_HZ = 20
MOVE_SETTLE_S = 0.5
POSE_STALE_TIMEOUT_S = 1.0
PID_TELEMETRY_ENABLED = False
PID_TELEMETRY_DIR = "logs/pid"


# ToF บน gimbal: หุ่นคงทิศเดิม, 0=หน้า, +90=ขวา (ตรวจทิศกับชุดติดตั้งจริง)
SCAN_YAWS = {"N": 0, "E": 90, "S": 180, "W": -90}
SCAN_YAW_SPEED_DPS = 90
SCAN_SETTLE_S = 0.2
SCAN_SAMPLES = 5
TOF_MAX_VALID_CM = 1000.0
# ค่าตั้งต้นสำหรับสแกนจากกลางช่อง 61 cm ต้องปรับตามตำแหน่งติดตั้ง ToF
SCAN_WALL_THRESHOLD_CM = 30.0
SCAN_OPEN_THRESHOLD_CM = 45.0
SCAN_UNCERTAIN_RETRIES = 2
# ตรวจทางที่จะเดินซ้ำเฉพาะเมื่อ alignment/การสแกนทำให้ pose เปลี่ยน
SCAN_RECHECK_YAW_DEG = 2.0
SCAN_RECHECK_TRANSLATION_M = 0.02


# ToF correction: ค่าเริ่มต้นสมมุติ ToF อยู่ตรงกลางหุ่น ต้องวัดจริงแต่ละทิศ
CORRECTION_TARGET_CM = {d: CELL_DISTANCE_M * 100 / 2 for d in SCAN_YAWS}
# ระยะด้านหน้าใช้ค่าเดียวกับ TARGET_FRONT_WALL_CM
CORRECTION_TARGET_CM["N"] = TARGET_FRONT_WALL_CM
CORRECTION_TOLERANCE_CM = 2.5
CORRECTION_MAX_OFFSET_CM = 17.0
CORRECTION_MAX_ATTEMPTS = 2
