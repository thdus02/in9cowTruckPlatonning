# Step 1: Add modules to provide access to specific libraries and functions
import os
import sys
import math
import tkinter as tk

# Step 2: Establish path to SUMO (SUMO_HOME)
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

# Step 3: Add TraCI module to provide access to specific libraries and functions
import traci  # SUMO <-> Python bridge (TraCI)

# Step 4: Define SUMO configuration
Sumo_config = [
    'sumo-gui',
    '-c', 'First.sumocfg',
    '--step-length', '0.05',          # 더 부드럽게(원하면 0.1로)
    '--delay', '500',                # GUI 느리게(원하면 조절/삭제)
    '--lateral-resolution', '0.1'
]

# Step 5: Open connection between SUMO and TraCI
traci.start(Sumo_config)

# Step 6: Define variables (state & constants)
KMH_LIMIT = 160.0   # 계기판 최대 표기 (km/h)
NEEDLE_RADIUS = 90  # 바늘 길이 (px)
CENTER_X, CENTER_Y = 150, 150

# Step 7: Define functions (UI helpers & update loop)
def draw_scale(canvas, max_speed_kmh=160, step=20):
    """속도계 눈금 숫자 표시"""
    radius = 100
    for v in range(0, max_speed_kmh + 1, step):
        angle = 225 - (v / max_speed_kmh) * 270  # 0km/h=225°, max=-45°
        rad = math.radians(angle)
        x = CENTER_X + (radius + 25) * math.cos(rad)
        y = CENTER_Y - (radius + 25) * math.sin(rad)
        canvas.create_text(x, y, text=str(v), font=("Arial", 10))

def draw_needle(canvas, needle, speed_mps):
    """속도(m/s)를 계기판 각도로 변환해 바늘 갱신"""
    kmh = speed_mps * 3.6
    if kmh > KMH_LIMIT:
        kmh = KMH_LIMIT
    angle = 225 - (kmh / KMH_LIMIT) * 270
    rad = math.radians(angle)
    x = CENTER_X + NEEDLE_RADIUS * math.cos(rad)
    y = CENTER_Y - NEEDLE_RADIUS * math.sin(rad)
    canvas.coords(needle, CENTER_X, CENTER_Y, x, y)

def update_data():
    """한 번의 시뮬레이션 스텝 후 UI를 갱신"""
    try:
        traci.simulationStep()
    except traci.exceptions.TraCIException:
        # 시뮬이 끝나거나 연결이 닫혔으면.. 등 안전하게 종료
        root.quit() #Tkinter 루프 종료
        return # update_data() 종료 (더 이상 반복x)

    vehicles = set(traci.vehicle.getIDList())

    # Leader 갱신
    if "Leader" in vehicles:
        vL = traci.vehicle.getSpeed("Leader")
        draw_needle(canvas_leader, leader_needle, vL)
        leader_label.config(text=f"Leader: {vL*3.6:.2f} km/h")
    else:
        # 없으면 0으로 떨어뜨리거나 유지하고 싶으면 이 블록 지우기. 지우는게 좋을듯.
        draw_needle(canvas_leader, leader_needle, 0.0)
        leader_label.config(text="Leader: —")

    # Follower 갱신
    vF = 0.0
    if "Follower" in vehicles:
        vF = traci.vehicle.getSpeed("Follower")
        draw_needle(canvas_follower, follower_needle, vF)
        follower_label.config(text=f"Follower: {vF*3.6:.2f} km/h")
    else:
        # 없으면 0으로 떨어뜨리거나 유지하고 싶으면 이 블록 지우기. 지우는게 좋을듯.
        draw_needle(canvas_follower, follower_needle, 0.0)
        follower_label.config(text="Follower: —")

    # Gap / Time Headway 계산 (Follower 기준)
    gap_text = "—"
    thw_text = "—"
    if "Follower" in vehicles:
        lead_info = traci.vehicle.getLeader("Follower")  # (leaderID, gap_m) or None
        if lead_info is not None and lead_info[0] == "Leader":
            gap_m = lead_info[1]  # Follower 앞범퍼 ~ Leader 뒤범퍼
            gap_text = f"{gap_m:,.1f} m"
            if vF > 1e-3:
                thw_text = f"{gap_m / vF:.2f} s"
            else:
                thw_text = "∞ s"

    gap_label.config(text=f"Gap: {gap_text}")
    thw_label.config(text=f"Headway: {thw_text}")

    # 100ms 후 반복
    root.after(100, update_data)

def on_close():
    """창 닫기 시 TraCI 안전 종료"""
    try:
        traci.close(False)
    except Exception:
        pass
    root.destroy()

# Step 8: Take simulation steps (GUI main loop instead of while-loop)
# UI 구성
root = tk.Tk()
root.title("Truck Platooning Speedometers")

# Leader speedometer
canvas_leader = tk.Canvas(root, width=300, height=300, bg="white")
canvas_leader.grid(row=0, column=0, padx=20, pady=20)
canvas_leader.create_oval(50, 50, 250, 250, width=3)
draw_scale(canvas_leader, max_speed_kmh=int(KMH_LIMIT), step=20)
leader_needle = canvas_leader.create_line(CENTER_X, CENTER_Y, CENTER_X, 60, width=3, fill="red")
draw_needle(canvas_leader, leader_needle, 0.0)
leader_label = tk.Label(root, text="Leader: 0.00 km/h", font=("Arial", 14))
leader_label.grid(row=1, column=0)

# Follower speedometer
canvas_follower = tk.Canvas(root, width=300, height=300, bg="white")
canvas_follower.grid(row=0, column=1, padx=20, pady=20)
canvas_follower.create_oval(50, 50, 250, 250, width=3)
draw_scale(canvas_follower, max_speed_kmh=int(KMH_LIMIT), step=20)
follower_needle = canvas_follower.create_line(CENTER_X, CENTER_Y, CENTER_X, 60, width=3, fill="blue")
draw_needle(canvas_follower, follower_needle, 0.0)
follower_label = tk.Label(root, text="Follower: 0.00 km/h", font=("Arial", 14))
follower_label.grid(row=1, column=1)

# Gap / Headway labels (공통 하단)
gap_label = tk.Label(root, text="Gap: — m", font=("Arial", 14))
gap_label.grid(row=2, column=0, columnspan=2, pady=(5, 0))

thw_label = tk.Label(root, text="Headway: — s", font=("Arial", 12), fg="gray")
thw_label.grid(row=3, column=0, columnspan=2)

root.protocol("WM_DELETE_WINDOW", on_close)

# 첫 갱신 스케줄 후 GUI 실행
root.after(100, update_data)
try:
    root.mainloop()
finally:
    # Step 9: Close connection between SUMO and TraCI
    try:
        traci.close(False)
    except Exception:
        pass
