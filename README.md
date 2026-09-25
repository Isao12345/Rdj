# RoboMaster EP Round 2: Autonomous Grid Navigation

ระบบนำทางอัตโนมัติบน Grid Map 7x7 สำหรับหุ่นยนต์ **DJI RoboMaster EP (รอบที่ 2)**
ขับเคลื่อนด้วยล้อ Mecanum แบบ Omnidirectional รักษาระดับมุมหันคงที่ตลอดเวลา (Heading = 0 องศา) ก้าวเดินช่องละ 0.60 เมตร พร้อมระบบจัดลำดับเป้าหมายแบบไดนามิก (Greedy Nearest Neighbor), ระบบ PID Controller และ Handshake Callback

---

## โครงสร้างโปรเจกต์ (Multi-file Modular Architecture)

```text
STATE2MOVE_finalproject/
├── main.py                     # [ROOT ENTRY] จุดรันหลักของโปรเจกต์ สั่งรันได้ทันที
├── README.md                   # คู่มือการใช้งานระบบ
└── robomaster_round2/
    ├── __init__.py             # Package Exports
    ├── config.py               # ค่าคงที่ระบบ (กริด 7x7, ขนาดช่อง 0.6m, PID gains, ความเร็ว)
    ├── pid_controller.py       # [PID MODULE] ควบคุม PID, Angle Error และ Heading Yaw Lock
    ├── grid_map.py             # คลาสจัดการแผนที่ สิ่งกีดขวาง กำแพง และ ASCII Map Visualizer
    ├── pathfinder.py           # คลาส A* Pathfinding ค้นหาเส้นทางที่สั้นที่สุด
    ├── target_sequencer.py     # อัลกอริทึม Greedy Nearest Neighbor จัดลำดับ N เป้าหมาย
    ├── chassis_driver.py       # Hardware Abstraction Layer (HAL) รองรับทั้ง Sim และ หุ่นจริง
    ├── mission_controller.py   # Flow การทำงานหลัก ลูปตาม Path และเรียก Handshake Callback
    ├── main.py                 # จุดรันภายในแพ็กเกจ (รองรับการรันตรง)
    └── tests.py                # ชุด Unit Tests ครบถ้วน (รวมทดสอบ PID)
```

---

## วิธีการใช้งาน (รันผ่าน main)

### 1. รันในโหมดจำลอง (Simulation Mode - ไม่ต้องต่อหุ่น)
สามารถรันผ่าน Terminal เพื่อดูภาพ ASCII Map จำลองการเดินแบบ Real-time:

```bash
# รันผ่าน Root main.py
python main.py

# หรือกำหนดความเร็วหน่วงเวลาต่อก้าว
python main.py --step-delay 0.05

# หรือรันผ่านโฟลเดอร์โมดูล
python robomaster_round2/main.py --step-delay 0.05
```

### 2. รันกับหุ่นยนต์จริง (Physical RoboMaster EP)
1. ติดตั้งไลบรารี RoboMaster SDK (หากยังไม่มี):
   ```bash
   pip install robomaster
   ```
2. เปิดสวิตช์หุ่นยนต์ และเชื่อมต่อ Wi-Fi กับตัวหุ่น (Direct Connection / AP Mode)
3. รันผ่านคำสั่ง:
   ```bash
   python main.py --mode real
   ```
   *(หรือแก้ไข `SIMULATION_MODE = False` ใน `robomaster_round2/config.py`)*

---

## รายละเอียดโมดูล PID Controller (`pid_controller.py`)

โมดูลนี้ถูกแยกออกมาอย่างชัดเจนเพื่อควบคุมการเคลื่อนที่และมุมหัน:
- **`PIDController`**: คลาสคำนวณ Proportional-Integral-Derivative พร้อมระบบ Clamping และ Reset
- **`calculate_angle_error(setpoint, measurement)`**: คำนวณ Error ของมุม Yaw ในช่วง `[-180.0, 180.0]` ป้องกัน Angle Wrap-around
- **`ChassisHeadingPID`**: คลาสล็อกมุมหันให้อยู่ที่ 0 องศาคงที่ตลอดเวลา เพื่อไม่ให้โครงหุ่นยนต์ฟาดกำแพง

---

## การเชื่อมต่อ Handshake Callback กับโมดูลภายนอก

```python
def my_handshake_callback(target_idx: int, target_coord: tuple, telemetry: dict):
    print(f"ถึงเป้าหมายที่ #{target_idx} พิกัด {target_coord}")
    # ใส่โค้ดเชื่อมต่อ เช่น ยิงเลเซอร์, เปิดกล้อง, ตรวจจับภาพ ฯลฯ
    return {"status": "SUCCESS"}
```

---

## การรัน Unit Tests

```bash
python -m unittest robomaster_round2.tests
```
