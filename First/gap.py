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
    '-c', 'osm.sumocfg',
    '--route-files', 'First.rou.xml',
    '--step-length', '0.05',          # 더 부드럽게(원하면 0.1로)
    '--delay', '100',                # GUI 느리게(원하면 조절/삭제)
    '--lateral-resolution', '0.1'
]

# Step 5: Open connection between SUMO and TraCI
traci.start(Sumo_config)


# 브레이크제어 (클릭형: 클릭마다 감속량 누적, 이후 자동 복귀)
class LeaderController:
    def __init__(self, traci_mod, sim_dt=0.05,
                 ramp_down_per_s=1.5,   # 누르는 동안 factor 감소 속도 (초당)
                 ramp_up_per_s=0.8,     # 떼고 나서 factor 회복 속도 (초당)
                 min_factor=0.0):         # 최저 factor (0.0이면 정지까지 허용)
        self.traci = traci_mod
        self.SIM_DT = sim_dt
        self.ramp_down = ramp_down_per_s
        self.ramp_up = ramp_up_per_s
        self.min_factor = min_factor

        self.factor = 1.0  # 현재 speedFactor
        self.braking = False

    def _apply(self):
        if "Leader" not in self.traci.vehicle.getIDList():
            return
        # 혹시 남아있는 수동 속도 지시 해제
        self.traci.vehicle.setSpeed("Leader", -1)
        # factor 반영 (CACC 제어는 그대로 유지)
        self.traci.vehicle.setSpeedFactor("Leader", self.factor)

    def on_brake_press(self, event=None):
        """버튼을 꾹 누르는 순간"""
        self.braking = True

    def on_brake_release(self, event=None):
        """버튼을 떼는 순간"""
        self.braking = False

    def update(self):
        """매 step 회복(브레이크를 누르지 않아도 자동 복귀)"""
        if "Leader" not in self.traci.vehicle.getIDList():
            return
        if self.braking:
            # 누르는 동안 감속
            self.factor = max(self.min_factor, self.factor - self.ramp_down * self.SIM_DT)
        else:
            # 떼면 천천히 복귀
            self.factor = min(1.0, self.factor + self.ramp_up * self.SIM_DT)

        self._apply()

            
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

def update_vehicle(veh_id, canvas, needle, label):
    # 개별 차량 업데이트로 변경 (기존: 2개의 차량이 도로 위에 올라가면 표시 됨)
    vehicles = traci.vehicle.getIDList()
    if veh_id in vehicles:
        #도로 위에 있을 때 속도 무조건 보여주기
        speed = traci.vehicle.getSpeed(veh_id)
        draw_needle(canvas, needle, speed)
        label.config(text=f"{veh_id}: {speed*3.6:.2f} km/h")
    else:
        draw_needle(canvas, needle, 0.0)
        label.config(text=f"{veh_id}: -")


def update_data():
    """한 번의 시뮬레이션 스텝 후 UI를 갱신"""
    try:
        traci.simulationStep()
    except traci.exceptions.TraCIException:
        # 시뮬이 끝나거나 연결이 닫혔으면.. 등 안전하게 종료
        root.quit() #Tkinter 루프 종료
        return # update_data() 종료 (더 이상 반복x)
    
    vehicles = set(traci.vehicle.getIDList())

    #여기서 함수 호출 개별 업데이트 하는
    update_vehicle("Leader", canvas_leader, leader_needle, leader_label)
    update_vehicle("Follower", canvas_follower, follower_needle, follower_label)

    vF = traci.vehicle.getSpeed("Follower") if "Follower" in vehicles else 0.0
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

    ctrl.update()
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

# === 브레이크 버튼 패널 ===
panel = tk.Frame(root)
panel.grid(row=4, column=0, columnspan=2, pady=(10, 5))
btn_brake = tk.Button(panel, text="Brake (click)", width=14)
btn_brake.grid(row=0, column=0, padx=5, pady=5)

# 컨트롤러 생성 및 바인딩
ctrl = LeaderController(traci_mod=traci, sim_dt=0.05,
                        ramp_down_per_s=1.5,  # 1클릭 감속량
                        ramp_up_per_s=0.8,        # 자동 복귀 속도
                        min_factor=0.0)

# 버튼을 꾹 누르고 있는 동안만 브레이크 효과
btn_brake.bind("<ButtonPress-1>", ctrl.on_brake_press)
btn_brake.bind("<ButtonRelease-1>", ctrl.on_brake_release)

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
