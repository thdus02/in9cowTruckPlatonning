# simulation/app.py
import math
import time, tkinter as tk, traci
import simulation.config as cfg
from simulation.config import Sumo_config
from simulation.safety import init_safety_defaults
from simulation.BrakeController import BrakeController
from simulation.platoon import (
    maintain_or_release_lock,
    boost_followers_once,
    control_follower_speed,
    ensure_initial_gap_lock,
    switch_to_cacc,         # ← 추가
    switch_to_basic,        # ← 추가 (이탈 시 사용)
)

from simulation.ui import build_speedometer, update_vehicle
from simulation.startui import open_selector_and_wait

# ==== 출발 게이트 설정 (pa_0 출구 위치 기준) ====
# pa_0이 lane="E0_0"에 있다면 EDGE는 "E0" 입니다.
START_GATE_EDGE = "E0"        # 출발 게이트가 위치한 엣지 ID
PA0_END_POS     = 30        # pa_0 endPos = 17.02 + 2m 정도로 설정(당신 맵에 맞게 수정)
START_SPACING   = 10.0        # 앞차가 게이트 통과 후 최소 이 거리(m) 이상 벌어졌을 때 다음 차 출발

# ==== 플래튜닝 참여 거리 임계값 ====
PLATOON_JOIN_DISTANCE = 300.0  # 플래튜닝 참여 버튼 활성화 거리 (m)

# ======= 모든 차량이 주차될 때까지 대기 =======
def wait_until_all_parked(traci_mod, timeout=180.0):
    """
    시뮬레이터에 등장한 모든 차량이 정차(주차) 상태가 될 때까지 대기.
    주차 완료 시 True 반환, 시간 초과 시 False.
    """
    t0 = time.time()
    while time.time() - t0 < timeout:
        traci_mod.simulationStep()
        ids = list(traci_mod.vehicle.getIDList())
        if ids and all(traci_mod.vehicle.isStopped(vid) for vid in ids):
            print("[INFO] 모든 차량 주차 완료.")
            return True
    print("[WARN] 일부 차량이 여전히 이동 중입니다. (timeout)")
    return False


def run():
    # 1) SUMO 시작 + 기본값
    traci.start(Sumo_config)
    init_safety_defaults()
    print("[INFO] SUMO 시작 - 모든 차량 주차 완료 대기 중...")

    ok = wait_until_all_parked(traci, timeout=180.0)
    print("[INFO] 주차 완료 상태:", ok)

    # 2) 주차 이후 선택창: 리더/팔로워 선택 → 체인
    chain = open_selector_and_wait(traci)  # ['Veh0','Veh1', ...]
    print("[DEBUG] 선택 결과 chain =", chain)
    if not chain:
        print("[WARN] 선택이 취소되거나 비어 있습니다. 종료.")
        traci.close(False)
        return

    # FOLLOW_PAIRS 구성 (선택 차량만)
    pairs = [(chain[i], chain[i-1]) for i in range(1, len(chain))]  # (follower, leader)
    cfg.FOLLOW_PAIRS = pairs
    cfg.FOLLOWERS    = [f for f, _ in pairs]
    print("[INFO] FOLLOW_PAIRS:", cfg.FOLLOW_PAIRS)

    leader_id = chain[0]
    try:
        traci.vehicle.setType(leader_id, "truckBASIC")
    except:
        pass

    for f, _ in cfg.FOLLOW_PAIRS:
        switch_to_cacc(f)

    # 3) UI 구성 (선택 차량만 계기판 띄우기)
    root = tk.Tk()
    root.title("Truck Platooning Simulation (Gate-based Release)")
    colors = ["red", "blue", "green", "purple", "orange", "pink"]

    all_vehicles = list(traci.vehicle.getIDList())
    meters = {}  # vid -> (canvas, needle, label)
    for idx, vid in enumerate(all_vehicles):
        try:
            canv, needle, lab = build_speedometer(root, vid, col=idx, needle_color=colors[idx % len(colors)])
            meters[vid] = (canv, needle, lab)
        except Exception as e:
            print(f"[WARN] {vid} 계기판 생성 실패: {e}")

    # (중략) meters 생성까지 끝난 직후에 ↓ 추가
    from simulation.vehicle_ui import open_vehicle_viewer
    open_vehicle_viewer(root, traci, chain)   # 드롭다운 뷰어 창 1개 띄움

    # 4) 리더 컨트롤러
    ctrl = BrakeController(
        traci_mod=traci, sim_dt=0.05,
        ramp_down_per_s=1.5, ramp_up_per_s=0.8, min_factor=0.0
    )
    ctrl.set_leader(chain[0])

    # 5) “게이트 + 간격” 조건으로 순차 출발
    release_index = 0                # chain[release_index]가 다음 출발 대상
    released = []                    # 이미 출발한 차량 목록
    gate_cross_dist = {}             # {vid: gate 통과 직후의 누적 거리}
    started_vehicles = set()         # 출발한 차량 집합 (버튼 비활성화용)

    def ready_to_release_next():
        """다음 차량을 출발시켜도 되는지 판단."""
        nonlocal gate_cross_dist

        # 리더는 바로 출발
        if release_index == 0:
            return True

        # 앞차 조건 확인
        prev_id = chain[release_index - 1]
        try:
            # 앞차의 현재 엣지/엣지 내 위치/속도
            road     = traci.vehicle.getRoadID(prev_id)          # 예: 'E0'
            lane_pos = traci.vehicle.getLanePosition(prev_id)    # 해당 엣지 내 s (m)
            v        = traci.vehicle.getSpeed(prev_id)           # m/s

            # (A) 게이트 통과 여부: 아직 pa_0 출구 이전이면 대기
            if road == START_GATE_EDGE and lane_pos < PA0_END_POS:
                return False

            # (B) 게이트 통과 순간의 누적거리(distance)를 기준점으로 기록
            if prev_id not in gate_cross_dist:
                gate_cross_dist[prev_id] = traci.vehicle.getDistance(prev_id)

            # (C) 간격 조건: 게이트 통과 기준점 대비 START_SPACING 이상 이동했는가
            d_from_gate = traci.vehicle.getDistance(prev_id) - gate_cross_dist[prev_id]
            return d_from_gate >= START_SPACING

        except traci.exceptions.TraCIException:
            return False

    # ==== 메인 루프 ====
    def update_loop():
        nonlocal release_index
        try:
            traci.simulationStep()
        except traci.exceptions.TraCIException:
            root.quit(); return
        
        # --- 동적으로 플래튜닝 체인 업데이트 및 계기판 관리 ---
        from simulation.vehicle_ui import _order_chain
        current_chain = _order_chain(cfg.FOLLOW_PAIRS)

        # --- 출발 조건 충족 시에만 다음 차량 release ---
        if release_index < len(chain) and ready_to_release_next():
            vid = chain[release_index]
            try:
                if traci.vehicle.isStopped(vid):
                    traci.vehicle.resume(vid)
                    print(f"[START] {vid} 출발")
                    released.append(vid)

                    # 팔로워 출발 직후 초기 락
                    for f, l in cfg.FOLLOW_PAIRS:
                        if vid == f:
                            ensure_initial_gap_lock(f, l)
            except traci.exceptions.TraCIException:
                pass
            release_index += 1

        # --- UI 갱신 ---
        all_existing_vehicles = set(traci.vehicle.getIDList())
        for vid in list(meters.keys()):
            if vid in all_existing_vehicles:
                try:
                    canv, needle, lab = meters[vid]
                    update_vehicle(traci, vid, canv, needle, lab)
                except Exception as e:
                    print(f"[WARN] {vid} UI 갱신 실패: {e}")
            else:
                # 차량이 사라진 경우 계기판 숨기기 (제거하지 않음)
                try:
                    canv, needle, lab = meters[vid]
                    from simulation.ui import draw_needle
                    draw_needle(canv, needle, 0.0)
                    lab.config(text=f"{vid}: - (제거됨)")
                except:
                    pass

        # 새로 추가된 차량의 계기판 생성
        for vid in all_existing_vehicles:
            if vid not in meters:
                try:
                    idx = len(meters)
                    canv, needle, lab = build_speedometer(root, vid, col=idx, needle_color=colors[idx % len(colors)])
                    meters[vid] = (canv, needle, lab)
                    print(f"[UI] 계기판 추가: {vid}")
                except Exception as e:
                    print(f"[WARN] {vid} 계기판 추가 실패: {e}")

        # --- 제어 로직 ---
        ctrl.update()
        boost_followers_once()
        for f, l in cfg.FOLLOW_PAIRS:
            maintain_or_release_lock(f, l)
            control_follower_speed(f, l)  

        # --- 플래튜닝 맨 뒷 차량과 비플래튜닝 차량 간 유클리드 거리 계산 및 버튼 활성화 상태 업데이트 ---
        try:
            if len(chain) > 0:
                last_platoon_veh = chain[-1]  # 플래튜닝 맨 뒷 차량
                
                # 비플래튜닝 차량 찾기 (출발한 차량 포함)
                try:
                    all_vehicles = set(traci.vehicle.getIDList())
                except:
                    all_vehicles = set()
                
                platoon_vehicles = set(chain)
                non_platoon_vehicles = all_vehicles - platoon_vehicles
                
                if last_platoon_veh in all_vehicles and non_platoon_vehicles:
                    try:
                        last_lane_id = traci.vehicle.getLaneID(last_platoon_veh)
                        last_road = traci.vehicle.getRoadID(last_platoon_veh)
                        
                        # 주차장에 있는지 확인
                        is_last_in_parking = (not last_lane_id or last_lane_id.startswith("pa_") or 
                                            last_road.startswith("pa_") or 
                                            traci.vehicle.isStopped(last_platoon_veh))
                        
                        # 주차장에 있으면 거리 계산 스킵
                        if not is_last_in_parking:
                            # 플래튜닝 맨 뒷 차량 위치
                            last_pos = traci.vehicle.getPosition(last_platoon_veh)
                            
                            # 각 비플래튜닝 차량과의 거리 계산
                            for non_platoon_veh in non_platoon_vehicles:
                                try:
                                    # 비플래튜닝 차량 정보
                                    non_lane_id = traci.vehicle.getLaneID(non_platoon_veh)
                                    non_road = traci.vehicle.getRoadID(non_platoon_veh)
                                    is_non_in_parking = (not non_lane_id or non_lane_id.startswith("pa_") or 
                                                        non_road.startswith("pa_") or
                                                        traci.vehicle.isStopped(non_platoon_veh))
                                    
                                    # 주차장에 있으면 거리 계산 스킵
                                    if is_non_in_parking:
                                        # 주차장에 있으면 거리 무한대로 설정 (버튼 비활성화)
                                        cfg.VEHICLE_DISTANCES[non_platoon_veh] = float('inf')
                                        continue
                                    
                                    # 유클리드 거리 계산
                                    non_pos = traci.vehicle.getPosition(non_platoon_veh)
                                    dx = last_pos[0] - non_pos[0]
                                    dy = last_pos[1] - non_pos[1]
                                    distance_diff = math.sqrt(dx*dx + dy*dy)
                                    
                                    # 거리 정보 저장 (vehicle_ui에서 사용)
                                    cfg.VEHICLE_DISTANCES[non_platoon_veh] = distance_diff
                                    
                                    # 거리 출력
                                    if distance_diff < 0.01:
                                        print(f"[거리차이] 플래튜닝 맨 뒷 차량({last_platoon_veh}) <-> 비플래튜닝 차량({non_platoon_veh}): 0.00m")
                                    elif distance_diff <= 5000:
                                        print(f"[거리차이] 플래튜닝 맨 뒷 차량({last_platoon_veh}) <-> 비플래튜닝 차량({non_platoon_veh}): {distance_diff:.2f}m")
                                except traci.exceptions.TraCIException:
                                    # 에러 발생 시 거리 무한대로 설정
                                    cfg.VEHICLE_DISTANCES[non_platoon_veh] = float('inf')
                                    pass
                    except traci.exceptions.TraCIException:
                        pass
        except Exception as e:
            pass

        # --- 종료 처리 ---
        if traci.simulation.getMinExpectedNumber() <= 0:
            try: traci.close()
            except: pass
            root.quit(); return

        root.after(50, update_loop)  # 20Hz

    def on_close():
        try: traci.close(False)
        except: pass
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.after(50, update_loop)
    root.mainloop()