"""Tkinter viewer for saved maps; no robot connection or HTML required."""

import argparse
import json
from pathlib import Path

from .map_viewer import validate_map

DEFAULT_MAP = Path(__file__).resolve().parents[2] / 'maps' / 'maze_map.json'


def load_map(path):
    data = json.loads(Path(path).read_text(encoding='utf-8'))
    if not isinstance(data, dict):
        raise ValueError('map JSON must be an object')
    validate_map(data)
    return data


def draw_map(canvas, data, width, height):
    """Draw on a Tk Canvas; coordinates follow the recorded row/column grid."""
    canvas.delete('all')
    rows, cols = data['rows'], data['cols']
    size = max(1, min((width - 40) / cols, (height - 40) / rows))
    left, top = (width - cols * size) / 2, (height - rows * size) / 2
    maze = data.get('maze') or [[dict.fromkeys('NESW') for _ in range(cols)] for _ in range(rows)]
    visited = {tuple(cell) for cell in data.get('visited_cells', [])}
    edges = {}
    for r, row in enumerate(maze):
        for c, walls in enumerate(row):
            x, y = left + c * size, top + r * size
            canvas.create_rectangle(x, y, x+size, y+size, outline='',
                                    fill='#dbeafe' if (r, c) in visited else '#f1f5f9')
            canvas.create_text(x+5, y+5, anchor='nw', text=f'{r},{c}', fill='#64748b')
            for d, edge in {
                'N': (x, y, x+size, y), 'S': (x, y+size, x+size, y+size),
                'W': (x, y, x, y+size), 'E': (x+size, y, x+size, y+size),
            }.items():
                # Round to merge shared boundaries despite floating-point arithmetic.
                edge = tuple(round(n, 6) for n in edge)
                edges.setdefault(edge, set()).add(walls[d])
    for edge, values in edges.items():
        known = values - {None}
        if len(known) > 1:
            canvas.create_line(*edge, fill='#ef4444', width=4)
        elif True in known:
            canvas.create_line(*edge, fill='#0f172a', width=3)
        elif not known:
            canvas.create_line(*edge, fill='#94a3b8', dash=(5, 5))

    def center(cell):
        return left+(cell[1]+0.5)*size, top+(cell[0]+0.5)*size

    path = data.get('path', [])
    if len(path) > 1:
        canvas.create_line(*(n for cell in path for n in center(cell)), fill='#2563eb', width=2)
    for key, label, offset, color in [('start', 'S', -10, '#15803d'), ('goal', 'G', 10, '#b45309')]:
        x, y = center(data[key])
        canvas.create_text(x+offset, y+18, text=label, fill=color, font=('TkDefaultFont', 12, 'bold'))
    if path:
        x, y = center(path[-1])
        canvas.create_oval(x-6, y-6, x+6, y+6, fill='#e11d48', outline='')


class MapWindow:
    def __init__(self, root, path, refresh_ms=1000):
        import tkinter as tk
        from tkinter import ttk
        self.root, self.path = root, Path(path)
        self.refresh_ms, self.data = refresh_ms, None
        root.title('RoboMaster — Saved Map')
        root.geometry('800x760')
        root.minsize(480, 420)
        bar = ttk.Frame(root, padding=8)
        bar.pack(fill='x')
        ttk.Button(bar, text='เปิด JSON', command=self.choose_file).pack(side='left')
        ttk.Button(bar, text='โหลดใหม่', command=self.reload).pack(side='left', padx=8)
        self.auto = tk.BooleanVar(value=True)
        ttk.Checkbutton(bar, text='อัปเดตจากไฟล์อัตโนมัติ', variable=self.auto).pack(side='left')
        self.status = tk.StringVar()
        ttk.Label(root, textvariable=self.status, padding=8, wraplength=750).pack(fill='x')
        self.canvas = tk.Canvas(root, background='white', highlightthickness=0)
        self.canvas.pack(fill='both', expand=True)
        self.canvas.bind('<Configure>', lambda event: self.redraw())
        ttk.Label(root, padding=8, text=(
            'N ↑  E → | ดำ: กำแพง · ประ: ยังไม่ทราบ · แดง: ข้อมูลขัดกัน\n'
            'พื้นฟ้า: สำรวจแล้ว · น้ำเงิน: เส้นทาง · S: เริ่ม · G: เป้าหมาย\n'
            'จุดชมพู: ช่องล่าสุดที่ยืนยันในไฟล์ ไม่ใช่ตำแหน่งหุ่นแบบสด')).pack(fill='x')
        self.reload()
        root.after(self.refresh_ms, self.tick)

    def choose_file(self):
        from tkinter import filedialog
        path = filedialog.askopenfilename(filetypes=[('Map JSON', '*.json')])
        if path:
            self.path = Path(path)
            self.data = None
            self.canvas.delete('all')
            self.reload()

    def reload(self):
        try:
            data = load_map(self.path)
        except (OSError, ValueError) as error:
            self.status.set(f'อ่าน {self.path.name} ไม่ได้: {error}\nภาพที่คงอยู่เป็นข้อมูลก่อนหน้า')
            return
        changed = data != self.data
        self.data = data
        self.status.set(
            f'{self.path}\nสถานะ: {data.get("status", "unknown")} | '
            f'สำรวจ {len(data.get("visited_cells", []))}/{data["rows"]*data["cols"]} ช่อง | '
            f'ก้าว: {max(0, len(data.get("path", []))-1)}\n'
            f'อัปเดต: {data.get("updated_at") or "—"}'
            + (f'\nError: {data["error"]}' if data.get('error') else ''))
        if changed:
            self.redraw()

    def redraw(self):
        if self.data:
            draw_map(self.canvas, self.data, self.canvas.winfo_width(), self.canvas.winfo_height())

    def tick(self):
        if self.auto.get():
            self.reload()
        self.root.after(self.refresh_ms, self.tick)


def main(argv=None):
    parser = argparse.ArgumentParser(description='Show a saved map in a Python window')
    parser.add_argument('map', nargs='?', type=Path, default=DEFAULT_MAP)
    parser.add_argument('--refresh-ms', type=int, default=1000)
    args = parser.parse_args(argv)
    if args.refresh_ms < 100:
        parser.error('--refresh-ms must be at least 100')
    try:
        import tkinter as tk
    except ImportError:
        parser.error('Tkinter is required (Ubuntu/Debian package: python3-tk)')
    try:
        root = tk.Tk()
    except tk.TclError as error:
        parser.error(f'Cannot open a graphical window: {error}')
    MapWindow(root, args.map, args.refresh_ms)
    root.mainloop()


if __name__ == '__main__':
    main()
