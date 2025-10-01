import tkinter as tk
import math
import os, sys, traci

# === SUMO 설정 ===
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

Sumo_config = [
    'sumo-gui',
    '-c', 'First.sumocfg',
    '--step-length', '0.1'
]

root = tk.Tk()
root.title("Truck Platooning Speedometers")

def draw_scale(canvas, max_speed=160, step=20):
    """속도계 눈금 숫자 표시"""
    radius = 100
    for v in range(0, max_speed+1, step):
        # 각도 매핑: 0 = 225°, max_speed = -45°
        angle = 225 - (v / max_speed) * 270
        rad = math.radians(angle)
        x = 150 + (radius+25) * math.cos(rad)
        y = 150 - (radius+25) * math.sin(rad)
        canvas.create_text(x, y, text=str(v), font=("Arial", 10))

def draw_needle(canvas, needle, speed, max_speed=33):
    """속도를 자동차 계기판 각도로 변환 후 바늘 그리기"""
    # SUMO 속도(m/s)를 km/h로 변환
    kmh = speed * 3.6
    if kmh > 160: kmh = 160  # 계기판 최대치 제한

    # 0km/h = 225°, 160km/h = -45° (시계방향)
    angle = 225 - (kmh / 160) * 270
    rad = math.radians(angle)
    x = 150 + 90 * math.cos(rad)
    y = 150 - 90 * math.sin(rad)
    canvas.coords(needle, 150, 150, x, y)

def update_data():
    traci.simulationStep()

    vehicles = traci.vehicle.getIDList()
    if "Leader" in vehicles and "Follower" in vehicles:
        leader_speed = traci.vehicle.getSpeed("Leader")
        follower_speed = traci.vehicle.getSpeed("Follower")

        # Leader 업데이트
        draw_needle(canvas_leader, leader_needle, leader_speed)
        leader_label.config(text=f"Leader: {leader_speed*3.6:.2f} km/h")

        # Follower 업데이트
        draw_needle(canvas_follower, follower_needle, follower_speed)
        follower_label.config(text=f"Follower: {follower_speed*3.6:.2f} km/h")

    root.after(100, update_data)
    
# Leader 속도계
canvas_leader = tk.Canvas(root, width=300, height=300, bg="white")
canvas_leader.grid(row=0, column=0, padx=20, pady=20)
canvas_leader.create_oval(50, 50, 250, 250, width=3)
draw_scale(canvas_leader, max_speed=160, step=20)
leader_needle = canvas_leader.create_line(150, 150, 150, 60, width=3, fill="red")
draw_needle(canvas_leader, leader_needle, 0.0)
leader_label = tk.Label(root, text="Leader: 0.00 m/s", font=("Arial", 14))
leader_label.grid(row=1, column=0)

# Follower 속도계
canvas_follower = tk.Canvas(root, width=300, height=300, bg="white")
canvas_follower.grid(row=0, column=1, padx=20, pady=20)
canvas_follower.create_oval(50, 50, 250, 250, width=3)
draw_scale(canvas_follower, max_speed=160, step=20)
follower_needle = canvas_follower.create_line(150, 150, 150, 60, width=3, fill="blue")
draw_needle(canvas_follower, follower_needle, 0.0)
follower_label = tk.Label(root, text="Follower: 0.00 m/s", font=("Arial", 14))
follower_label.grid(row=1, column=1)

# === SUMO 실행 ===
traci.start(Sumo_config)

update_data()
root.mainloop()
traci.close()
