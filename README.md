# RoboMaster: สำรวจเขาวงกต

รันจากโฟลเดอร์ `RDJ-FINAL-ROUND-2`:

```bash
# จำลองตามขนาดสนามและจุดเริ่มใน config
python3 -m robomaster_round2.Slam.main --mode sim
# หุ่นจริง: สแกนอย่างเดียว
python3 -m robomaster_round2.Slam.main --mode real --scan-only
# หุ่นจริง: สแกนหนึ่งรอบแล้วทดสอบ alignment อย่างเดียว
python3 -m robomaster_round2.Slam.main --mode real --alignment
# หุ่นจริง: ดู Sharp ซ้ายหน้า/หลังแบบไม่สั่งหุ่นขยับ กด Ctrl+C เพื่อจบ
python3 -m robomaster_round2.Slam.main --mode real --sharp-check
# เดินหนึ่งช่อง
python3 -m robomaster_round2.Slam.main --mode real --max-steps 1
# สำรวจทุกช่องที่เข้าถึงได้ แล้วกลับจุดเริ่ม
python3 -m robomaster_round2.Slam.main --mode real
# ทดสอบ SLAM โดยไม่ต่อหุ่น
python3 -m unittest discover -s robomaster_round2/tests -p 'test_*.py'
```

## แก้ config ที่เดียว

ไฟล์ `robomaster_round2/Slam/setting/maze_setting.py` ใช้กับ SLAM
แก้ค่าแล้วเริ่มโปรแกรมใหม่ หน่วยระบุในชื่อ: `_CM`, `_M`, `_S`, `_MPS`, `_DPS`

| ต้องการปรับ | ค่า config |
|---|---|
| ขนาดสนาม / จุดเริ่ม | `MAZE_ROWS`, `MAZE_COLS`, `START_CELL` (แถว, คอลัมน์ เริ่มที่ 0) |
| ระยะหนึ่งช่อง / ความเร็วเดิน | `CELL_DISTANCE_M`, `MOVE_SPEED_MPS` |
| PID เดิน | `MOVE_X_KP/KI/KD`, `MOVE_Y_KP/KI/KD` |
| เปิดการปรับแนว | `ALIGNMENT_ENABLED` |
| ระยะผนังหน้าเป้าหมาย / ระยะต่าง Sharp ที่ยอมรับ | `TARGET_FRONT_WALL_CM`, `PARALLEL_TOLERANCE_CM` |
| PID ปรับแนว (Kp, Ki, Kd) | `ALIGNMENT_FRONT_PID`, `ALIGNMENT_YAW_PID` |
| เวลาพักก่อนเดิน / หลังเดิน / หลัง alignment | `PRE_MOVE_DELAY_S`, `MOVE_SETTLE_S`, `ALIGNMENT_SETTLE_S` |
| พอร์ต Sharp | `LEFT_FRONT_BOARD/PORT`, `LEFT_BACK_BOARD/PORT` |
| คาลิเบรต Sharp | ค่าที่ขึ้นต้น `SHARP_` |
| ตัวกรอง Sharp | `SHARP_FILTER_WINDOW` (median ย้อนหลังแยกแต่ละเซนเซอร์) |
| มุมสแกน / ระยะผนังและทางเปิด | `SCAN_YAWS`, `SCAN_WALL_THRESHOLD_CM`, `SCAN_OPEN_THRESHOLD_CM` |
| ตรวจทางหลัง alignment | `SCAN_UNCERTAIN_RETRIES`, `SCAN_RECHECK_YAW_DEG`, `SCAN_RECHECK_TRANSLATION_M` |
| ยืนยันว่าถึงช่อง | `MOVE_POSITION_TOLERANCE_M`, `MOVE_CROSS_TRACK_TOLERANCE_M`, `MOVE_ARRIVAL_YAW_TOLERANCE_DEG` |
| ข้อมูลตำแหน่งเก่า | `POSE_STALE_TIMEOUT_S` |
| บันทึก PID | `PID_TELEMETRY_ENABLED`, `PID_TELEMETRY_DIR` |

Sharp อ่านระยะไม่ได้ หรือ ToF ด้านหน้าเกิน `FRONT_WALL_THRESHOLD_CM` จะข้าม
alignment ของช่องนั้นแล้วสำรวจต่อ การเดินไม่มี timeout หรือระยะหยุดหน้าอัตโนมัติ

## ลำดับการทำงาน

สแกน ToF สี่ทิศหนึ่งรอบ → ใช้ Sharp ซ้ายปรับ yaw ให้ขนานผนังก่อน
→ หยุดและพัก → ใช้ ToF หน้าปรับระยะถ้ามีผนังหน้า
→ บันทึกแผนที่จากผลสแกนรอบแรก → เลือกทางด้วย DFS → รอ 3 วินาที
→ ถ้า pose เปลี่ยนหลังสแกน ให้ตรวจ ToF เฉพาะทิศที่จะเดินอีกครั้ง → เดินหนึ่งช่อง
คำสั่งเดินหน้า/ถอยหลังใช้เฉพาะ X และคำสั่งสไลด์ใช้เฉพาะ Y; ระหว่างเดินไม่สั่งหมุน yaw
ToF ระยะมากกว่า 30 แต่น้อยกว่า 45 ซม. เป็นค่าก้ำกึ่งและวัดซ้ำได้อีก 2 ชุด; ถ้ายังไม่ชัดจะไม่เดิน
เมื่อเดินเสร็จ จะยืนยันทั้งแกนหลัก แกนข้าง และ yaw ก่อนบันทึกช่องใหม่
ยังไม่มีการตรวจสิ่งกีดขวางระหว่างเดิน
`GOAL_CELL` ไม่ใช่จุดหยุดของการสำรวจนี้

แผนที่หุ่นจริงบันทึกที่ `maps/maze_map.json`; โหมดจำลองที่ `maps/slam_demo.json`
ใช้ `--output` เพื่อเปลี่ยนปลายทาง ค่า wall เป็น true/false/null
(กำแพง/ทางโล่ง/ยังไม่ทราบ) และ `step_limit` หมายถึงครบจำนวนก้าวที่กำหนด

## หน้าที่ไฟล์

- `Slam/main.py`: รับ argument และเลือกโหมด
- `Slam/runtime.py`: เชื่อมฮาร์ดแวร์และปิดทรัพยากรเมื่อจบงาน
- `Slam/simulation.py`: สนามจำลอง ไม่ต้องต่อหุ่น
- `Slam/workflow.py`: รักษามุม → สแกน → ปรับแนว → สแกนซ้ำ
- `Slam/slam.py`: สำรวจและบันทึกความคืบหน้า
- `Slam/sensors.py`: สแกน ToF ด้วย gimbal
- `Slam/sharp_sensor.py`: แปลง ADC เป็นระยะ
- `Slam/correction.py`: `RoboMasterAlignment` ปรับระยะหน้า/ซ้ายและมุม
- `Slam/movement.py`: PID เดินและหยุด chassis
- `Slam/pose_tracker.py`: อ่านตำแหน่งและ yaw
- `Slam/maze_traversal.py`: เลือกทาง DFS
- `Slam/map_recorder.py`: เขียนแผนที่ JSON

`WallCorrection` และค่า `CORRECTION_*` เป็นตัวปรับระยะ ToF อีกแบบที่ยังเก็บไว้
แต่ไม่ได้ใช้ในเส้นทาง `real` ปัจจุบัน เช่นเดียวกับ IR digital ใน `RoboMasterMazeSensors`

ก่อนรันหุ่นจริง ต้องตรวจพอร์ต ทิศแกน มุม gimbal และคาลิเบรตระยะให้ตรงกับชุดติดตั้ง
การทดสอบอัตโนมัติใช้ข้อมูลจำลอง ยังไม่ยืนยันผลบนหุ่นจริง

## แนวทางพัฒนาต่อ

อ่าน [DEVELOPMENT.md](DEVELOPMENT.md) สำหรับจุดต่อโมดูลและวิธีทดสอบ

## เปิดดูแผนที่ที่บันทึกไว้

รันจาก `RDJ-FINAL-ROUND-2`:

```bash
# อ่าน maps/maze_map.json แล้วสร้าง maps/maze_map.html พร้อมเปิดเบราว์เซอร์
python3 -m robomaster_round2.Slam.map_viewer --open
# เลือกไฟล์แผนที่อื่น
python3 -m robomaster_round2.Slam.map_viewer maps/slam_demo.json --open
# กำหนดปลายทาง HTML (ไม่เปิดเบราว์เซอร์)
python3 -m robomaster_round2.Slam.map_viewer maps/maze_map.json --output maps/view.html
```

ไม่ต้องติดตั้งแพ็กเกจเพิ่ม HTML เปิดดูหรือส่งต่อได้โดยไม่ต้องมีไฟล์ JSON อยู่ข้างกัน
แสดงกำแพง ช่องที่สำรวจ เส้นทาง จุดเริ่ม เป้าหมาย สถานะ และข้อความผิดพลาด
เส้นประหมายถึงยังไม่ทราบ; จุดชมพูคือช่องล่าสุดที่ยืนยัน ไม่ใช่ตำแหน่งสด
หาก JSON เปลี่ยน ให้รันคำสั่งอีกครั้งและรีเฟรชเบราว์เซอร์
ตัวดูแผนที่อ่านข้อมูลจาก JSON โดยไม่เชื่อมต่อหรือสั่งงานหุ่น

## ดูแผนที่ด้วยหน้าต่าง Python (.py)

เมื่อรันโหมดหุ่นจริง โปรแกรมจะเปิดหน้าต่างแผนที่นี้อัตโนมัติหลังสร้างไฟล์ JSON
และหน้าต่างจะอ่านไฟล์ซ้ำทุก 1 วินาที การปิดหน้าต่างไม่หยุดการเดินของหุ่น
โหมด `--alignment` และ `--sharp-check` ไม่เปิดแผนที่ เพราะไม่ได้บันทึกแผนที่

```bash
# รันจาก RDJ-FINAL-ROUND-2
python3 show_map.py
# เลือกไฟล์อื่น หรือปรับรอบอ่านไฟล์ (มิลลิวินาที)
python3 show_map.py maps/slam_demo.json --refresh-ms 1000
```

มีปุ่มเลือก JSON, โหลดใหม่ และเปิด/ปิดการอ่านไฟล์อัตโนมัติ
วาดด้วย Tkinter ไม่ต้องสร้าง HTML หรือเปิดเบราว์เซอร์
ต้องรันใน desktop ที่เปิดหน้าต่างได้ และมี Tkinter (Ubuntu/Debian: `python3-tk`)
ค่าเริ่มต้นอ่าน `maps/maze_map.json` ภายในโปรเจกต์ แม้เรียกไฟล์จากโฟลเดอร์อื่น
โค้ด UI อยู่ที่ `robomaster_round2/Slam/map_gui.py`; `show_map.py` เป็นจุดรัน
เมื่ออ่านไฟล์ไม่ได้จะแจ้งข้อความและเก็บภาพก่อนหน้าไว้
