import tkinter as tk
import math
from .config import KMH_LIMIT, NEEDLE_RADIUS, CENTER_X, CENTER_Y

def draw_scale(canvas, max_speed_kmh=160, step=20):
    """속도계 눈금 숫자 표시"""
    radius = 100
    for v in range(0, max_speed_kmh+1, step):
        angle = 225 - (v / max_speed_kmh) * 270  # 0km/h=225°, max=-45°
        rad = math.radians(angle)
        x = CENTER_X + (radius + 25) * math.cos(rad)
        y = CENTER_Y - (radius + 25) * math.sin(rad)
        canvas.create_text(x, y, text=str(v), font=("Arial", 10))

def draw_needle(canvas, needle, speed_mps):
    """속도(m/s)를 계기판 각도로 변환해 바늘 갱신"""
    kmh = min(speed_mps*3.6, KMH_LIMIT)
    angle = 225 - (kmh / KMH_LIMIT) * 270
    rad = math.radians(angle)
    x = CENTER_X + NEEDLE_RADIUS * math.cos(rad)
    y = CENTER_Y - NEEDLE_RADIUS * math.sin(rad)
    canvas.coords(needle, CENTER_X, CENTER_Y, x, y)

def build_speedometer(root, title, col, needle_color):
    canvas = tk.Canvas(root, width=300, height=300, bg="white")
    canvas.grid(row=0, column=col, padx=20, pady=20)
    canvas.create_oval(50, 50, 250, 250, width=3)
    draw_scale(canvas, max_speed_kmh=int(KMH_LIMIT), step=20)
    needle = canvas.create_line(CENTER_X, CENTER_Y, CENTER_X, 60, width=3, fill=needle_color)
    draw_needle(canvas, needle, 0.0)
    label = tk.Label(root, text=f"{title}: 0.00 km/h", font=("Arial", 14))
    label.grid(row=1, column=col)
    return canvas, needle, label

def update_vehicle(traci, veh_id, canvas, needle, label):
    vehicles = traci.vehicle.getIDList()
    if veh_id in vehicles:
        v = traci.vehicle.getSpeed(veh_id)
        draw_needle(canvas, needle, v)
        label.config(text=f"{veh_id}: {v*3.6:.2f} km/h")
    else:
        draw_needle(canvas, needle, 0.0)
        label.config(text=f"{veh_id}: -")

def build_gap_labels(root):
    gap1 = tk.Label(root, text="Gap L→F1: — m", font=("Arial", 14)); gap1.grid(row=2, column=0, columnspan=3, pady=(5,0))
    gap2 = tk.Label(root, text="Gap F1→F2: — m", font=("Arial", 14)); gap2.grid(row=3, column=0, columnspan=3)
    gap3 = tk.Label(root, text="Gap F2→F3: — m", font=("Arial", 14)); gap3.grid(row=3, column=0, columnspan=3)
    return gap1, gap2, gap3
