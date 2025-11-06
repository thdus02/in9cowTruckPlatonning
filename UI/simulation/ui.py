# ui.py
import tkinter as tk
import math
from .config import KMH_LIMIT

def _polar(cx, cy, r, angle_deg):
    rad = math.radians(angle_deg)
    return cx + r * math.cos(rad), cy - r * math.sin(rad)

def draw_scale(canvas, cx, cy, radius, max_speed_kmh=160, step=20, font_px=9):
    label_r = radius * 0.82
    for v in range(0, max_speed_kmh + 1, step):
        angle = 225 - (v / max_speed_kmh) * 270
        x, y = _polar(cx, cy, label_r, angle)
        canvas.create_text(x, y, text=str(v), font=("Arial", font_px))

def _draw_needle(canvas, needle_id, speed_mps, max_speed_kmh=KMH_LIMIT):
    meta = getattr(canvas, "_needle_meta", None)
    if not meta:
        return
    cx, cy, needle_len = meta
    kmh = min(speed_mps * 3.6, max_speed_kmh)
    angle = 225 - (kmh / max_speed_kmh) * 270
    x, y = _polar(cx, cy, needle_len, angle)
    canvas.coords(needle_id, cx, cy, x, y)

def build_speedometer(root, title, col, needle_color, size=220):
    #size만 바꾸면 전체가 비율 유지.

    canvas = tk.Canvas(root, width=size, height=size, bg="white")

    pad_x = max(30, size // 6) 
    pad_y = max(10, size // 18)
    canvas.grid(row=0, column=col, padx=pad_x, pady=pad_y)

    cx = cy = size // 2
    margin = int(size * 0.10)
    radius = cx - margin

    tick_font = max(8, int(size * 0.042))
    label_font = max(12, int(size * 0.065))

    canvas.create_oval(cx - radius, cy - radius, cx + radius, cy + radius, width=3)
    draw_scale(canvas, cx, cy, radius, max_speed_kmh=int(KMH_LIMIT), step=20, font_px=tick_font)

    needle_len = int(radius * 0.90)
    needle_id = canvas.create_line(
        cx, cy, *_polar(cx, cy, needle_len, 225), width=3, fill=needle_color
    )
    canvas._needle_meta = (cx, cy, needle_len)
    _draw_needle(canvas, needle_id, 0.0)

    # 라벨을 게이지에서 더 떨어뜨리기
    label_gap = max(4, size // 40) 
    label = tk.Label(root, text=f"{title}: 0.00 km/h", font=("Arial", label_font))
    label.grid(row=1, column=col, pady=(label_gap, 0))

    return canvas, needle_id, label

def update_vehicle(traci, veh_id, canvas, needle_id, label):
    try:
        vehicles = traci.vehicle.getIDList()
        if veh_id in vehicles:
            v = traci.vehicle.getSpeed(veh_id)
            # 차량 타입 가져오기
            try:
                vtype = traci.vehicle.getTypeID(veh_id)
            except:
                vtype = "unknown"
            _draw_needle(canvas, needle_id, v)
            label.config(text=f"{veh_id} ({vtype.split('@')[0]}): {v*3.6:.2f} km/h")
        else:
            _draw_needle(canvas, needle_id, 0.0)
            label.config(text=f"{veh_id}: -")
    except Exception:
        _draw_needle(canvas, needle_id, 0.0)
        label.config(text=f"{veh_id}: -")

def build_gap_labels(root):
    gap1 = tk.Label(root, text="Gap L→F1: — m", font=("Arial", 13))
    gap1.grid(row=2, column=0, columnspan=3, pady=(6, 0))
    gap2 = tk.Label(root, text="Gap F1→F2: — m", font=("Arial", 13))
    gap2.grid(row=3, column=0, columnspan=3)
    gap3 = tk.Label(root, text="Gap F2→F3: — m", font=("Arial", 13))
    gap3.grid(row=4, column=0, columnspan=3)
    return gap1, gap2, gap3
