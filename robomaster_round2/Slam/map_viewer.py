"""เปิดแผนที่ JSON ที่บันทึกไว้เป็น HTML โดยไม่ต้องติดตั้งไลบรารีเพิ่ม."""

import argparse
import html
import json
from pathlib import Path
import webbrowser

from .setting import maze_setting as settings

CELL_SIZE = 80
MARGIN = 30


def validate_map(data):
    """ตรวจโครงสร้างก่อนวาด รวมถึงไฟล์ที่ยังไม่สแกนช่องแรก."""
    rows, cols = data.get('rows'), data.get('cols')
    if any(type(n) is not int or n <= 0 for n in (rows, cols)):
        raise ValueError('rows and cols must be positive integers')
    maze = data.get('maze')
    if maze is not None:
        if not isinstance(maze, list) or len(maze) != rows:
            raise ValueError('maze row count does not match rows')
        for row in maze:
            if not isinstance(row, list) or len(row) != cols:
                raise ValueError('maze column count does not match cols')
            for cell in row:
                if not isinstance(cell, dict) or any(
                    d not in cell or (cell[d] is not None and type(cell[d]) is not bool)
                    for d in 'NESW'
                ):
                    raise ValueError('each cell needs N/E/S/W with true, false or null')
    for key in ('start', 'goal', 'path', 'visited_cells'):
        points = [data.get(key)] if key in ('start', 'goal') else data.get(key, [])
        if not isinstance(points, list):
            raise ValueError(f'{key} must be a list')
        for point in points:
            if (not isinstance(point, (list, tuple)) or len(point) != 2
                    or any(type(n) is not int for n in point)
                    or not (0 <= point[0] < rows and 0 <= point[1] < cols)):
                raise ValueError(f'invalid cell in {key}: {point}')


def render_map(data, title='Saved maze'):
    """คืน HTML แบบ standalone; แถวเพิ่มลงล่าง คอลัมน์เพิ่มไปขวา."""
    validate_map(data)
    rows, cols = data['rows'], data['cols']
    maze = data.get('maze') or [[dict.fromkeys('NESW') for _ in range(cols)]
                              for _ in range(rows)]
    visited = {tuple(cell) for cell in data.get('visited_cells', [])}
    width, height = cols * CELL_SIZE + 2 * MARGIN, rows * CELL_SIZE + 2 * MARGIN
    svg = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}" '
           'role="img" aria-label="Saved maze map">']
    edges = {}
    for r, row in enumerate(maze):
        for c, walls in enumerate(row):
            x, y = MARGIN + c * CELL_SIZE, MARGIN + r * CELL_SIZE
            fill = '#dbeafe' if (r, c) in visited else '#f1f5f9'
            svg.append(f'<rect x="{x}" y="{y}" width="{CELL_SIZE}" height="{CELL_SIZE}" fill="{fill}"/>')
            svg.append(f'<text x="{x+8}" y="{y+17}" font-size="11" fill="#64748b">{r},{c}</text>')
            for direction, edge in {
                'N': (x, y, x+CELL_SIZE, y),
                'S': (x, y+CELL_SIZE, x+CELL_SIZE, y+CELL_SIZE),
                'W': (x, y, x, y+CELL_SIZE),
                'E': (x+CELL_SIZE, y, x+CELL_SIZE, y+CELL_SIZE),
            }.items():
                edges.setdefault(edge, []).append(walls[direction])
    for (x1, y1, x2, y2), values in edges.items():
        known = {value for value in values if value is not None}
        if len(known) > 1:
            color, thickness, dash = '#ef4444', 5, ''
        elif True in known:
            color, thickness, dash = '#0f172a', 4, ''
        elif False in known:
            continue
        else:
            color, thickness, dash = '#94a3b8', 1, ' stroke-dasharray="5 5"'
        svg.append(f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" '
                   f'stroke="{color}" stroke-width="{thickness}"{dash}/>')

    def center(cell):
        return MARGIN + (cell[1] + 0.5) * CELL_SIZE, MARGIN + (cell[0] + 0.5) * CELL_SIZE

    path = data.get('path', [])
    if path:
        points = ' '.join(f'{x},{y}' for x, y in map(center, path))
        svg.append(f'<polyline points="{points}" fill="none" stroke="#2563eb" '
                   'stroke-width="3" stroke-linejoin="round" opacity="0.65"/>')
    for key, label, color, offset in [('start', 'S', '#15803d', -15), ('goal', 'G', '#b45309', 15)]:
        x, y = center(data[key])
        svg.append(f'<text x="{x+offset}" y="{y+24}" text-anchor="middle" '
                   f'font-size="16" font-weight="bold" fill="{color}">{label}</text>')
    if path:
        x, y = center(path[-1])
        svg.append(f'<circle cx="{x}" cy="{y}" r="8" fill="#e11d48"><title>Last confirmed cell</title></circle>')
    svg.append('</svg>')
    escape = lambda value: html.escape(str(value))
    error = f'<p class="error">{escape(data["error"])}</p>' if data.get('error') else ''
    return f'''<!doctype html>
<html lang="th"><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>{escape(title)}</title>
<style>
body {{font:16px system-ui,sans-serif;background:#f8fafc;color:#0f172a;max-width:900px;margin:32px auto;padding:0 20px}}
svg {{display:block;width:100%;max-height:75vh;margin:20px auto;background:white;border-radius:12px}}
p {{line-height:1.7}} .error {{color:#b91c1c}} small {{color:#64748b}}
</style>
<h1>{escape(title)}</h1>
<p>สถานะ: <b>{escape(data.get('status', 'unknown'))}</b> · สำรวจ {len(visited)}/{rows*cols} ช่อง
 · เดิน {max(0, len(path)-1)} ก้าว · N ↑ E →</p>
<small>บันทึกล่าสุด: {escape(data.get('updated_at') or '—')}</small>
{error}{''.join(svg)}
<p>เส้นดำ: กำแพง · เส้นประ: ยังไม่ทราบ · ไม่มีเส้น: ทางโล่ง · เส้นแดง: ข้อมูลกำแพงขัดกัน<br>
พื้นฟ้า: สำรวจแล้ว · เส้นน้ำเงิน: เส้นทาง · S: จุดเริ่ม · G: เป้าหมาย<br>
จุดชมพู: ช่องล่าสุดที่ยืนยันในไฟล์ (ไม่ใช่ตำแหน่งหุ่นแบบสด)</p>
<p><small>ภาพจากไฟล์บันทึก ณ ตอนสร้าง HTML; รันคำสั่งเดิมอีกครั้งเพื่ออ่านข้อมูลล่าสุด</small></p>
</html>'''


def main(argv=None):
    parser = argparse.ArgumentParser(description='Render a recorded maze JSON as HTML')
    parser.add_argument('map', nargs='?', default=settings.MAP_OUTPUT_PATH)
    parser.add_argument('--output', help='HTML output path; default: next to JSON')
    parser.add_argument('--open', action='store_true', help='Open the result in a browser')
    args = parser.parse_args(argv)
    source = Path(args.map)
    output = Path(args.output) if args.output else source.with_suffix('.html')
    if output.resolve() == source.resolve():
        parser.error('HTML output must not overwrite the source JSON')
    try:
        data = json.loads(source.read_text(encoding='utf-8'))
        if not isinstance(data, dict):
            raise ValueError('map JSON must be an object')
        document = render_map(data, source.name)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(document, encoding='utf-8')
    except (OSError, ValueError) as error:
        parser.error(str(error))
    print(f'[Map viewer] {output.resolve()}')
    if args.open:
        webbrowser.open(output.resolve().as_uri())


if __name__ == '__main__':
    main()
