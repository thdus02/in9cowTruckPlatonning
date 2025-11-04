import itertools
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
    gap3 = tk.Label(root, text="Gap F2→F3: — m", font=("Arial", 14)); gap3.grid(row=4, column=0, columnspan=3)
    return gap1, gap2, gap3

def veh_distance(traci, a: str, b: str):
    """두 차량의 유클리드 거리(m). 차량이 없거나 좌표 비정상이면 None."""
    try:
        ids = set(traci.vehicle.getIDList())
        if a not in ids or b not in ids:
            return None
        xa, ya = traci.vehicle.getPosition(a)
        xb, yb = traci.vehicle.getPosition(b)
        if xa <= -1e9 or ya <= -1e9 or xb <= -1e9 or yb <= -1e9:
            return None
        import math
        return math.hypot(xa - xb, ya - yb)
    except:
        return None

def build_distance_sections(root, chain, all_ids, row_start=7):
    """
    체인 차량과 비체인 차량을 구분해 두 섹션의 거리 라벨을 만든다.
    return: (frame_platoon, labels_platoon, frame_others, labels_others)
      - labels_* : {("VehA","VehB"): tk.Label}
    """
    # 섹션 A: 플래툰 차량들끼리 (chain 전체의 모든 2-쌍)
    frame_p = tk.LabelFrame(root, text="Platooning vehicles — distances (m)")
    frame_p.grid(row=row_start, column=0, columnspan=6, sticky="w", padx=6, pady=(8, 4))
    labels_p = {}

    pairs_p = list(itertools.combinations(chain, 2))
    if pairs_p:
        tk.Label(frame_p, text="Pair  |  Distance (m)", font=("Segoe UI", 9, "bold")).grid(
            row=0, column=0, sticky="w", padx=4, pady=2
        )
        for i, (a, b) in enumerate(pairs_p, start=1):
            lbl = tk.Label(frame_p, text=f"{a}–{b}: —")
            lbl.grid(row=i, column=0, sticky="w", padx=6, pady=1)
            labels_p[(a, b)] = lbl
    else:
        tk.Label(frame_p, text="(no platooning pair)").grid(row=0, column=0, sticky="w", padx=6, pady=2)

    # 섹션 B: 비플래툰 차량 vs 플래툰 맨 뒤 차량
    # 비플래투닝 차량 목록을 안정적으로 분리 (중복 제거 + 원래 순서 유지)
    chain_set = set(chain)
    ordered_all = list(dict.fromkeys(all_ids))
    others = [vid for vid in ordered_all if vid not in chain_set]
    frame_o = tk.LabelFrame(root, text="Non-platooning → platoon tail — distances (m)")
    frame_o.grid(row=row_start+1, column=0, columnspan=6, sticky="w", padx=6, pady=(4, 8))
    labels_o = {}

    tail = chain[-1] if len(chain) >= 1 else None
    pairs_o = [(tail, o) for o in others] if tail else []
    if pairs_o:
        tk.Label(frame_o, text="Pair  |  Distance (m)", font=("Segoe UI", 9, "bold")).grid(
            row=0, column=0, sticky="w", padx=4, pady=2
        )
        for i, (a, b) in enumerate(pairs_o, start=1):
            lbl = tk.Label(frame_o, text=f"{a}–{b}: —")
            lbl.grid(row=i, column=0, sticky="w", padx=6, pady=1)
            labels_o[(a, b)] = lbl
    else:
        msg = "(no platoon tail or no non-platooning vehicles)" if tail is None else "(no non-platooning vehicles)"
        tk.Label(frame_o, text=msg).grid(row=0, column=0, sticky="w", padx=6, pady=2)

    return frame_p, labels_p, frame_o, labels_o


def update_distance_sections(traci, labels_platoon, labels_others):
    """build_distance_sections()로 만든 두 섹션 라벨들을 매 스텝 갱신."""
    for labels in (labels_platoon, labels_others):
        for (a, b), lbl in labels.items():
            d = veh_distance(traci, a, b)
            if d is None:
                lbl.config(text=f"{a}–{b}: —")
            else:
                lbl.config(text=f"{a}–{b}: {d:,.1f} m")
