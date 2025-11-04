# Step 1~3: import & SUMO_HOME 경로 처리 + TraCI Start + 메인 루프
import os, sys, tkinter as tk, traci
import simulation.config as cfg
from simulation.config import Sumo_config
from simulation.safety import init_safety_defaults
from simulation.leader_controller import LeaderController
from simulation.platoon import (
    maintain_or_release_lock,
    control_follower_speed,
    boost_followers_once,
)
from simulation.ui import build_speedometer, update_vehicle

# ★ 추가: 거리 계산용
import math
from itertools import combinations


def iter_pairs(pairs):
    """FOLLOW_PAIRS에서 (follower, leader) 페어만 안전하게 꺼냄"""
    for i, p in enumerate(pairs):
        if not isinstance(p, (tuple, list)) or len(p) < 2:
            raise ValueError(f"FOLLOW_PAIRS[{i}] 형식 오류: {p}")
        yield p[0], p[1]  # follower, leader

def order_chain(pairs):
    pairs = list(pairs)
    if not pairs:
        return []  # FOLLOW_PAIRS가 비어있을 경우 빈 체인 반환

    followers = {f for f, _ in pairs}
    leaders   = {l for _, l in pairs}
    roots = list(leaders - followers)   # 팔로워가 아닌 리더(시작점 후보)

    # 루트가 있으면 첫 번째 리더, 없으면 첫 페어의 리더로 시작
    start = roots[0] if roots else pairs[0][1]

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
        g.grid(row=base_row + i, column=0, columnspan=2, sticky="w", padx=4)
        gap_labels.append(g)
    return gap_labels

def _gap_text(traci_mod, follower_id, leader_id):
    try:
        vehicles = set(traci_mod.vehicle.getIDList())
        if follower_id not in vehicles or leader_id not in vehicles:
            return "—"

        lead_info = traci_mod.vehicle.getLeader(follower_id, 1000.0)
        if not lead_info or lead_info[0] != leader_id:
            return "—"

        gap_m = max(0.0, lead_info[1])
        return f"{gap_m:,.1f} m"

    except traci.exceptions.TraCIException:
        return "—"

def safe_position(traci_mod, veh_id):
    """차량이 네트워크에 없거나 특수 좌표면 None 반환."""
    try:
        if veh_id not in traci_mod.vehicle.getIDList():
            return None
        x, y = traci_mod.vehicle.getPosition(veh_id)
        if x <= -1e9 or y <= -1e9:
            return None
        return (x, y)
    except traci.exceptions.TraCIException:
        return None

def fmt_pos(pos):
    """콘솔/라벨용 출력 포맷: None -> '—'"""
    return "—" if pos is None else f"({pos[0]:.1f},{pos[1]:.1f})"

# ★ 추가: 두 차량 간 유클리드 거리
def veh_distance(traci_mod, a: str, b: str):
    try:
        ids = set(traci_mod.vehicle.getIDList())
        if a not in ids or b not in ids:
            return None
        xa, ya = traci_mod.vehicle.getPosition(a)
        xb, yb = traci_mod.vehicle.getPosition(b)
        if xa <= -1e9 or ya <= -1e9 or xb <= -1e9 or yb <= -1e9:
            return None
        return math.hypot(xa - xb, ya - yb)
    except traci.exceptions.TraCIException:
        return None

def build_distance_sections(root, chain, non_platoon, row_start=7):
    """
    return: frame_p, labels_p, frame_o, labels_o
    labels_*: {("VehA","VehB"): tk.Label}
    """
    # 섹션 A: 플래투닝(체인) 차량들 간 거리
    frame_p = tk.LabelFrame(root, text="Platooning vehicles — distances (m)")
    frame_p.grid(row=row_start, column=0, columnspan=6, sticky="w", padx=6, pady=(8, 4))
    labels_p = {}
    pairs_p = list(combinations(chain, 2))
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

    # 섹션 B: 비플래투닝 차량 vs 플래투닝 꼬리 차량 거리
    frame_o = tk.LabelFrame(root, text="Non-platooning → platoon tail — distances (m)")
    frame_o.grid(row=row_start+1, column=0, columnspan=6, sticky="w", padx=6, pady=(4, 8))
    labels_o = {}
    tail = chain[-1] if len(chain) >= 1 else None
    pairs_o = [(tail, o) for o in non_platoon] if tail else []
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


# ★ 추가: 섹션 라벨 갱신
def update_distance_sections(traci_mod, labels_p, labels_o):
    for labels in (labels_p, labels_o):
        for (a, b), lbl in labels.items():
            d = veh_distance(traci_mod, a, b)
            if d is None:
                lbl.config(text=f"{a}–{b}: —")
            else:
                lbl.config(text=f"{a}–{b}: {d:,.1f} m")


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
    root = tk.Tk(); 
    root.title("Truck Platooning Speedometers")

    # --- 스피도미터(체인 순서대로 생성) ---
    colors = ["red", "blue", "green", "pink", "purple", "orange"]
    meters= {}
    gap_labels = []
    prev_chain = None

    # ★ 추가: 거리 섹션 상태
    frame_p = frame_o = None
    labels_p, labels_o = {}, {}

    def rebuild_ui_for_chain(chain):
        """체인이 바뀌면 속도계/갭라벨 재구성 + 거리 섹션 재생성"""
        nonlocal meters, gap_labels, frame_p, labels_p, frame_o, labels_o
        # 기존 위젯 제거
        for canv, needle, lab in meters.values():
            try:
                canv.destroy(); lab.destroy()
            except:
                pass
        for g in gap_labels:
            try:
                g.destroy()
            except:
                pass
        if frame_p:
            try: frame_p.destroy()
            except: pass
        if frame_o:
            try: frame_o.destroy()
            except: pass
        meters.clear()
        gap_labels.clear()
        labels_p.clear()
        labels_o.clear()

        # 새 속도계
        for idx, vid in enumerate(chain):
            canv, needle, lab = build_speedometer(root, vid, col=idx, needle_color=colors[idx % len(colors)])
            meters[vid] = (canv, needle, lab)

        # 새 갭 라벨(리더 제외 수만큼)
        n_followers = max(0, len(chain) - 1)
        gap_labels.extend(build_gap_labels_dyn(root, n_followers))

        # ★ 추가: 플래투닝/비플래투닝 거리 섹션 생성
        # ★ UI 선택 결과 기반 구분 (없으면 안전한 대체)
        ui_chain  = getattr(cfg, "PLATOON_CHAIN", chain) or []
        ui_others = getattr(cfg, "NON_PLATOON", None)
        if ui_others is None:
            # fallback: SELECTABLE_VEHICLES가 있다면 거기서 비체인 계산, 그마저 없으면 현재 활성 차량에서 계산
            selectable = getattr(cfg, "SELECTABLE_VEHICLES", list(traci.vehicle.getIDList()))
            ui_others = [v for v in selectable if v not in set(ui_chain)]

        frame_p, labels_p, frame_o, labels_o = build_distance_sections(
            root, ui_chain, ui_others, row_start=7
        )

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

        try:
            pairs = list(iter_pairs(cfg.FOLLOW_PAIRS))  # [(f,l), ...]
        except ValueError as e:
            # 형식 오류 시 안전하게 스킵
            print("FOLLOW_PAIRS 형식 오류:", e)
            pairs = []

        chain = order_chain(pairs)  # [Leader, F1, F2, ...]

        # 2) 체인이 바뀌면 UI 재구성 + 리더 id 반영
        nonlocal prev_chain
        if chain != prev_chain:
            rebuild_ui_for_chain(chain)
            leader_id = chain[0] if chain else None
            if leader_id:
                ctrl.set_leader(leader_id)
            prev_chain = chain

            # ★ 선택된 차량만 출발 (triggered 해제) — 기존 로직 그대로
            selected = set(chain)
            for vid in selected:
                try:
                    traci.vehicle.resume(vid)
                except traci.exceptions.TraCIException:
                    pass

        # --- 리더가 아직 지정되지 않았다면 update 스킵 ---
        if not ctrl.leader_id:
            root.after(100, update_loop)
            return
        
        # 3) 초기 락/해제 & 출발부스트
        for fid, lid in pairs:
            maintain_or_release_lock(fid, lid)
        boost_followers_once()

        # 4) UI: speedometer 업데이트
        for vid in chain:
            canv, needle, lab = meters[vid]
            update_vehicle(traci, vid, canv, needle, lab)

        pos_strs = [f"{vid}={fmt_pos(safe_position(traci, vid))}" for vid in chain]
        print(f"[t={traci.simulation.getTime():.1f}s] " + ", ".join(pos_strs))
        
        # UI: gaps (플래투닝 체인 순서만)
        for i in range(1, len(chain)):
            fid, lid = chain[i], chain[i-1]
            g = _gap_text(traci, fid, lid)
            gap_labels[i-1].config(text=f"Gap {lid}→{fid}: {g}")

        # ★ 추가: 거리 섹션 갱신 (플래투닝/비플래투닝 모두)
        update_distance_sections(traci, labels_p, labels_o)

        ctrl.update()
        for fid, lid in pairs:
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
