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
    '--step-length', '0.05',          # 더 부드럽게(원하면 0.1로)
    '--delay', '100',                # GUI 느리게(원하면 조절/삭제)
    '--lateral-resolution', '0.1',
     '--collision.action', 'warn',           # ★ 충돌 시 제거/텔레포트 대신 경고
    '--collision.mingap-factor', '1.0'      # ★ 안전거리 계산에 minGap 반영(보수적)
]

# Step 5: Open connection between SUMO and TraCI
traci.start(Sumo_config)

# === Safety defaults (충돌 회피 및 급제동 성능 보정) ===
try:
    # (선택) 안전 모드 보장 – 기본값이지만 혹시라도 변경된 상태를 초기화
    traci.vehicle.setSpeedMode("Leader", 31)
    traci.vehicle.setSpeedMode("Follower", 31)

    # vType의 emergencyDecel 을 충분히 크게 (리더 급정지 대응력 향상)
    # rou 파일에 id가 'leadtruck', 'truckCACC' 라고 했으니 그 타입에 적용
    traci.vehicletype.setEmergencyDecel("leadtruck", 6.0)   # m/s^2
    traci.vehicletype.setEmergencyDecel("truckCACC", 6.0)

except traci.exceptions.TraCIException:
    pass


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
# === Platooningcontrol constants ===
MIN_GAP_M    = 20.0      # 최소 간격 (Follower 앞범퍼 ~ Leader 뒷범퍼)
MAX_GAP_M    = 20.0      # 최대 간격
T_HEADWAY_S  = 0.8       # 목표 시간 헤드웨이 (s)

CATCH_GAIN   = 0.20      # 멀어질 때 추종 가속 이득
BRAKE_GAIN   = 0.50      # 가까워질 때 감속 이득
V_MAX_FOLLOW = 33.0      # Follower 속도 상한 (m/s) - 필요시 조정

# === 속도계기판 ===
KMH_LIMIT = 160.0   # 계기판 최대 표기 (km/h)
NEEDLE_RADIUS = 90  # 바늘 길이 (px)
CENTER_X, CENTER_Y = 150, 150

# Step 7: Define functions (UI helpers & update loop)
# === 최대,최소 gap설정 및 추월 금지
def ensure_no_overtake(veh_id):
    """차선 변경을 완전히 막아 추월 금지."""
    try:
        traci.vehicle.setLaneChangeMode(veh_id, 0)  # 모든 lane-change 의도 off
    except traci.exceptions.TraCIException:
        pass

def control_follower_speed():
    """Leader 기준으로 Follower가 MIN_GAP~MAX_GAP 범위에 머물도록 제어."""
    vehicles = set(traci.vehicle.getIDList())
    if not {"Leader", "Follower"}.issubset(vehicles):
        return

    # 추월 금지 (보수적으로 매 스텝 보장)
    ensure_no_overtake("Leader")
    ensure_no_overtake("Follower")

    vL = traci.vehicle.getSpeed("Leader")
    vF = traci.vehicle.getSpeed("Follower")

    lead_info = traci.vehicle.getLeader("Follower", 1000.0)  # (leaderID, gap_m) or None
    if not lead_info or lead_info[0] != "Leader":
        # 리더를 못 보면 -> vL보다 조금 빠르게 달려서 추격
        target_v = min(vL + 2.0, V_MAX_FOLLOW) # +2 m/s 정도 여유
        traci.vehicle.setSpeed("Follower", max(0.0, target_v))
        return

    gap_m = max(0.0, lead_info[1])  # 앞범퍼~뒷범퍼 간격
    closing = vF - vL  # (+)면 좁혀지는 중
     # 정지거리 추정: s = v^2 / (2*a_em)
    try:
        a_em_L = traci.vehicletype.getEmergencyDecel(traci.vehicle.getTypeID("Leader"))
    except:
        a_em_L = 6.0
    try:
        a_em_F = traci.vehicletype.getEmergencyDecel(traci.vehicle.getTypeID("Follower"))
    except:
        a_em_F = 6.0

    sL = (vL * vL) / (2.0 * max(1e-6, a_em_L))
    sF = (vF * vF) / (2.0 * max(1e-6, a_em_F))
    BUFFER = 5.0  # m, 추가 안전 여유

    # 2) 정지거리 기준 ‘이미 위험’ 판단
    imminent = (sF > gap_m + sL - BUFFER)

    # 3) TTC(Time-To-Collision) 기반 판단(양수 closing에서만 유효)
    TTC_MIN = 1.3  # s
    ttc_danger = False
    if closing > 0.0:
        ttc = gap_m / closing
        ttc_danger = (ttc < TTC_MIN)
    else:
        ttc = float('inf')

    if imminent or ttc_danger:
        # ★ 강한 감속 명령: 리더 속도보다 확실히 낮추고, slowDown으로 급제동 유도
        safe_v = max(0.0, min(vF - 3.0, vL - 2.0))  # ↓ 더 강하게 감속
        traci.vehicle.slowDown("Follower", safe_v, int(1000 * 0.5))  # 0.3s 동안 급감속, 0.3을 0.4~0.6으로 늘리면 감속 시간을 늘려 보다 부드럽게 감속.
        traci.vehicle.setSpeed("Follower", safe_v)
        return
    # ====== 안전 급제동 블록 끝 ======

    # 시간 헤드웨이 기반 목표 간격 + 상/하한 클램프
    desired_gap = MIN_GAP_M + T_HEADWAY_S * vF
    desired_gap = max(MIN_GAP_M, min(desired_gap, MAX_GAP_M))

    error = gap_m - desired_gap  # (+) 멀다 → 가속 / (-) 가깝다 → 감속

    if error < 0:
        # 너무 가까움 → Leader보다 느리게 (감속)
        target_v = min(vF, vL) + error * BRAKE_GAIN  # error<0 → 감속
        target_v = min(target_v, vL - 0.5)  # Leader보다 약간 느리게(여유 0.5 m/s)
    else:
        # 너무 멀다 → Leader보다 약간 빠르게 (상한 제한)
        target_v = vL + min(3.0, error * CATCH_GAIN)  # 최대 +3 m/s까지만 추종 가속

    target_v = max(0.0, min(target_v, V_MAX_FOLLOW))
    traci.vehicle.setSpeed("Follower", target_v)

def _boost_follower_once():
    """Follower가 출발한 직후 최고속도 제한을 풀어주는 함수"""
    for vid in traci.simulation.getDepartedIDList():
        if vid == "Follower":
            try:
                # 40 m/s ≈ 144 km/h (테스트용 넉넉히)
                traci.vehicle.setMaxSpeed("Follower", 40.0)
            except traci.exceptions.TraCIException:
                pass

# === 속도계기판 ===
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
    
    _boost_follower_once()  # ★ 출발 순간 한 번만 상한 해제

    vehicles = set(traci.vehicle.getIDList())

    #여기서 함수 호출 개별 업데이트 하는
    update_vehicle("Leader", canvas_leader, leader_needle, leader_label)
    update_vehicle("Follower", canvas_follower, follower_needle, follower_label)

    vF = traci.vehicle.getSpeed("Follower") if "Follower" in vehicles else 0.0
    # Gap / Time Headway 계산 (Follower 기준)
    gap_text = "—"
    thw_text = "—"
    if "Follower" in vehicles:
        lead_info = traci.vehicle.getLeader("Follower", 1000.0)  # (leaderID, gap_m) or None
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
    control_follower_speed()   # ★ 팔로워 추종/감속/간격/추월금지 제어
    
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
