# Step 1~3: import & SUMO_HOME 경로 처리 + TraCI Start + 메인 루프
import os, sys, tkinter as tk, traci
from simulation.config import Sumo_config, FOLLOW_PAIRS
from simulation.safety import init_safety_defaults
from simulation.leader_controller import LeaderController
from simulation.platoon import (
    ensure_initial_gap_lock,
    maintain_or_release_lock,
    control_follower_speed,
    boost_followers_once,
)
from simulation.ui import build_speedometer, update_vehicle
def iter_pairs(pairs):
    """FOLLOW_PAIRS에서 (follower, leader) 페어만 안전하게 꺼냄"""
    for i, p in enumerate(pairs):
        if not isinstance(p, (tuple, list)) or len(p) < 2:
            raise ValueError(f"FOLLOW_PAIRS[{i}] 형식 오류: {p}")
        yield p[0], p[1]  # follower, leader

def order_chain(pairs):
    """(f,l) 리스트 → [Leader, F1, F2, ...] 순서로 정렬"""
    pairs = list(pairs)
    followers = {f for f, _ in pairs}
    leaders   = {l for _, l in pairs}
    roots = list(leaders - followers)   # 팔로워가 아닌 리더(체인 시작 후보)
    start = roots[0] if roots else "Leader"  # 없으면 기본 'Leader' 가정

    order = [start]
    while True:
        nxt = next((f for f, l in pairs if l == order[-1] and f not in order), None)
        if not nxt:
            break
        order.append(nxt)
    return order

def build_gap_labels_dyn(root, n_followers, base_row=5):
    """팔로워 수(n)만큼 Gap/THW 라벨 생성해서 리스트로 반환"""
    gap_labels = []
    for i in range(n_followers):
        g = tk.Label(root, text="Gap —")
        # 보기 좋게 두 줄로 나눠 배치(원하면 위치 바꿔도 됨)
        g.grid(row=base_row + i, column=0, columnspan=2, sticky="w", padx=4)
        gap_labels.append(g)
    return gap_labels

def _gap_text(traci_mod, follower_id, leader_id):
    #존재하는 차량 ID 들을 set으로 바꿈
    #만약 차량 중에 팔로워가 없으면 - 반환
    vehicles = set(traci_mod.vehicle.getIDList())
    if follower_id not in vehicles:
        return "—"
    #
    lead_info = traci_mod.vehicle.getLeader(follower_id, 1000.0)
    if not lead_info or lead_info[0] != leader_id:
        return "—"
    gap_m = max(0.0, lead_info[1])
    return f"{gap_m:,.1f} m"

def safe_position(traci_mod, veh_id):
    """차량이 네트워크에 없거나 특수 좌표면 None 반환."""
    try:
        if veh_id not in traci_mod.vehicle.getIDList():
            return None
        x, y = traci_mod.vehicle.getPosition(veh_id)
        # SUMO가 '미존재'를 나타낼 때 쓰는 큰 음수 좌표 가드
        if x <= -1e9 or y <= -1e9:
            return None
        return (x, y)
    except traci.exceptions.TraCIException:
        return None

def fmt_pos(pos):
    """콘솔/라벨용 출력 포맷: None -> '—'"""
    return "—" if pos is None else f"({pos[0]:.1f},{pos[1]:.1f})"

def run():
    # Step 2: Establish path to SUMO (SUMO_HOME)
    if 'SUMO_HOME' in os.environ:
        tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
        sys.path.append(tools)
    else:
        sys.exit("Please declare environment variable 'SUMO_HOME'")

    # Step 5: Open connection between SUMO and TraCI
    traci.start(Sumo_config)
    init_safety_defaults()

    # Step 8: Tk GUI & bindings
    root = tk.Tk(); root.title("Truck Platooning Speedometers")

    
    # --- FOLLOW_PAIRS → (f,l) 페어/체인 구성 ---
    pairs = list(iter_pairs(FOLLOW_PAIRS))      # ex) [("Follower1","Leader"), ("Follower2","Follower1"), ...]
    chain = order_chain(pairs)                  # ex) ["Leader","Follower1","Follower2",...]
    n_followers = max(0, len(chain) - 1)        # 리더 제외 수

    # --- 스피도미터(체인 순서대로 생성) ---
    colors = ["red", "blue", "green", "pink", "purple", "orange"]
    meters = {}  # veh_id -> (canvas, needle, label)
    for idx, vid in enumerate(chain):
        canv, needle, lab = build_speedometer(root, vid, col=idx, needle_color=colors[idx % len(colors)])
        meters[vid] = (canv, needle, lab)

    # --- 갭 라벨(팔로워 수만큼) ---
    gap_labels = build_gap_labels_dyn(root, n_followers)

    # Brake button panel
    panel = tk.Frame(root); panel.grid(row=6, column=0, columnspan=3, pady=(10,5))
    btn_brake = tk.Button(panel, text="Brake (click)", width=14); btn_brake.grid(row=0, column=0, padx=5, pady=5)
    ctrl = LeaderController(traci_mod=traci, sim_dt=0.05, ramp_down_per_s=1.5, ramp_up_per_s=0.8, min_factor=0.0)
    btn_brake.bind("<ButtonPress-1>",  ctrl.on_brake_press)
    btn_brake.bind("<ButtonRelease-1>", ctrl.on_brake_release)

    def update_loop():
        # Step 7: Define functions (update loop)
        try:
            traci.simulationStep()
        except traci.exceptions.TraCIException:
            root.quit(); return

        # 초기 락 / 해제
        for fid, lid in iter_pairs(FOLLOW_PAIRS):
            maintain_or_release_lock(fid, lid)

        boost_followers_once()

        # UI: speed
        for vid in chain:
            canv, needle, lab = meters[vid]
            update_vehicle(traci, vid, canv, needle, lab)

        pos_strs = [f"{vid}={fmt_pos(safe_position(traci, vid))}" for vid in chain]
        print(f"[t={traci.simulation.getTime():.1f}s] " + ", ".join(pos_strs))
        
        # UI: gaps
        for i in range(1, len(chain)):
            fid, lid = chain[i], chain[i-1]
            g = _gap_text(traci, fid, lid)
            gap_labels[i-1].config(text=f"Gap {lid}→{fid}: {g}")
            

        ctrl.update()
        for fid, lid in iter_pairs(FOLLOW_PAIRS):
            control_follower_speed(fid, lid)

        root.after(100, update_loop)

    def on_close():
        try: traci.close(False)
        except: pass
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.after(100, update_loop)
    try:
        root.mainloop()
    finally:
        try: traci.close(False)
        except: pass
