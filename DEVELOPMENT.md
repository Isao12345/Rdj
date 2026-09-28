# แนวทางพัฒนาต่อ

## โครงสร้าง

`main.py` รับ CLI → `runtime.py` ประกอบอุปกรณ์ → `workflow.py` จัดลำดับงาน
→ `SlamSession` สำรวจ → `MazeMapRecorder` บันทึก
โหมดจำลองอยู่ใน `simulation.py` และใช้ `SlamSession` เดียวกัน

## เปลี่ยนอะไร แก้ที่ไหน

| งาน | จุดแก้ |
|---|---|
| จูนค่าหุ่นและพอร์ต | `robomaster_round2/Slam/setting/maze_setting.py` |
| เพิ่ม CLI option | `Slam/main.py` แล้วส่งค่าเข้า runtime |
| เปลี่ยนเซนเซอร์ / การเชื่อมต่อ | `Slam/runtime.py`, `Slam/sensors.py` |
| เปลี่ยนลำดับสแกนหรือปรับแนว | `Slam/workflow.py` |
| เปลี่ยนวิธีเลือกช่องถัดไป | `Slam/maze_traversal.py` |
| เปลี่ยน PID เดิน | `Slam/movement.py` |
| เปลี่ยนรูปแบบข้อมูลแผนที่ | `Slam/map_recorder.py` |

## ข้อตกลงระหว่างโมดูล

- Scanner: `scan()` คืนกำแพง N/E/S/W; `invalidate_cache()` ล้างผลเก่า;
  เรียก `scanner(row, col, direction)` เพื่ออ่านกำแพงจากผลสแกน
- Mover: `mover(x, y)` รับเมตรและคืน `True` เมื่อถือว่าถึงช่อง;
  `False` ทำให้การสำรวจหยุด ไม่เลื่อนตำแหน่งช่อง
- Alignment: `align_in_cell(cell, walls, heading)` คืน `True` เมื่อปรับสำเร็จ;
  `False` เมื่อข้าม ต้องหยุดมอเตอร์ก่อนคืนค่า และสแกนใหม่เพราะหุ่นอาจขยับแล้ว
- Tracker: `read()` คืน `(x, y, yaw)`; yaw ใช้องศา
- Session: รับฟังก์ชันสแกน/เดิน/ปรับแนว ไม่เปิดหรือปิดฮาร์ดแวร์เอง
- Runtime: เป็นเจ้าของอายุอุปกรณ์ ใช้ `ExitStack` ลงทะเบียน cleanup ก่อนเริ่มใช้งาน

หากเปลี่ยนกติกาคืนค่า ต้องปรับทั้งผู้เรียกและ test ที่เกี่ยวข้อง
อย่า import RoboMaster SDK ในโมดูลที่ใช้จำลอง; import เมื่อเข้า `run_real`
CLI เดิมผ่าน `robomaster_round2.main`, `Slam.main`, `Slam.slam` ยังใช้ได้

## ทดสอบก่อนใช้กับหุ่น

รันจากโฟลเดอร์ `RDJ-FINAL-ROUND-2`:

```bash
python3 -m unittest discover -s robomaster_round2/tests -p 'test_*.py'
python3 -m robomaster_round2.main --mode sim --output /tmp/rdj-sim.json
```

เพิ่มชุดทดสอบชื่อ `test_*.py` ใช้ Mock สำหรับอุปกรณ์
ทดสอบลำดับงานใน `test_workflow.py`; ทดสอบสำรวจ/ปรับแนวใน `test_slam_explore.py`
กำหนดสนามและค่าที่ test ต้องใช้ผ่าน `patch` เพื่อไม่ให้การจูนหุ่นเปลี่ยนผลทดสอบ

`tests/tests.py` เป็นชุดเก่าที่อ้างโมดูลซึ่งไม่มีแล้ว เก็บไว้เป็นข้อมูลอ้างอิง
คำสั่งข้างต้นเลือกเฉพาะชุดปัจจุบันด้วย pattern `test_*.py`
การผ่าน test และ simulation ไม่ได้ยืนยันพฤติกรรมของฮาร์ดแวร์จริง
