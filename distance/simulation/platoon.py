import math
import traci
import simulation.config as cfg
from .config import DESIRED_GAP, CATCH_GAIN, BRAKE_GAIN, V_MAX_FOLLOW

# --- 전역 상태(팔로워별 초기 락) ---
startup_lock_done  = {}  # follower_id -> bool
startup_lock_until = {}  # follower_id -> float

def _ensure_lock_keys(fid):
    if fid not in startup_lock_done:
        startup_lock_done[fid] = False
    if fid not in startup_lock_until:
        startup_lock_until[fid] = 0.0

#정렬
def ensure_initial_gap_lock(follower_id, leader_id):
    """출발 직후 follower를 leader 뒤 DESIRED_GAP로 '한번만' 정렬"""
    try:
        sim_t = traci.simulation.getTime()
        vehicles = set(traci.vehicle.getIDList())
        _ensure_lock_keys(follower_id)

        if startup_lock_done.get(follower_id, False):
            return
        if not {follower_id, leader_id}.issubset(vehicles):
            return

        lane_id = traci.vehicle.getLaneID(leader_id)
        posL    = traci.vehicle.getLanePosition(leader_id)
        lenL    = traci.vehicle.getLength(leader_id)
        target_posF = max(0.0, posL - lenL - DESIRED_GAP)

        # 일시적으로 follower의 속도모드 해제 후 위치/속도 강제
        traci.vehicle.setSpeedMode(follower_id, 0)
        traci.vehicle.moveTo(follower_id, lane_id, target_posF)

        vL = traci.vehicle.getSpeed(leader_id)
        traci.vehicle.setSpeed(follower_id, vL)

        startup_lock_until[follower_id] = sim_t + 1.0  # 1초 유지
        startup_lock_done[follower_id]  = True
    except traci.exceptions.TraCIException:
        pass

def maintain_or_release_lock(follower_id, leader_id):
    """락 유지 중엔 setSpeed로 강제 동기화, 시간이 지나면 SpeedMode 복구"""
    try:
        sim_t = traci.simulation.getTime()
        vehicles = set(traci.vehicle.getIDList())
        _ensure_lock_keys(follower_id)

        if not {follower_id, leader_id}.issubset(vehicles): return
        if sim_t < startup_lock_until.get(follower_id, 0.0):
            vL = traci.vehicle.getSpeed(leader_id)
            traci.vehicle.setSpeed(follower_id, vL)
        else:
            traci.vehicle.setSpeedMode(follower_id, 31)
    except traci.exceptions.TraCIException:
        pass

def ensure_no_overtake(veh_id: str):
    """차선 변경을 완전히 막아 추월 금지."""
    try:
        traci.vehicle.setLaneChangeMode(veh_id, 0)
    except traci.exceptions.TraCIException:
        pass

def control_follower_speed(follower_id, leader_id):
    """leader_id 기준으로 follower_id 간격을 DESIRED_GAP에 유지"""
    try:
        vehicles = set(traci.vehicle.getIDList())
        if not {follower_id, leader_id}.issubset(vehicles):
            return

        ensure_no_overtake(follower_id)
        ensure_no_overtake(leader_id)

        vL = traci.vehicle.getSpeed(leader_id)
        vF = traci.vehicle.getSpeed(follower_id)

        # 리더 인식 (길어도 1000m 범위 내)
        lead_info = traci.vehicle.getLeader(follower_id, 1000.0)
        if not lead_info or lead_info[0] != leader_id:
            # 리더를 직접 못 보면 과도 가속 방지: vL/V_MAX 따라 안전 추종
            traci.vehicle.setSpeed(follower_id, max(0.0, min(vL, V_MAX_FOLLOW)))
            return

        gap_m = max(0.0, lead_info[1])
        close = vF - vL

        # 안전 급제동(정지거리 + TTC)
        a_em_L = _safe_em_decel(traci.vehicle.getTypeID(leader_id))
        a_em_F = _safe_em_decel(traci.vehicle.getTypeID(follower_id))
        sL = (vL*vL) / (2.0*max(1e-6, a_em_L))
        sF = (vF*vF) / (2.0*max(1e-6, a_em_F))
        BUFFER = 5.0

        imminent   = (sF > gap_m + sL - BUFFER)
        ttc = (gap_m/close) if close>0.0 else math.inf
        ttc_danger = (ttc < 1.3)

        if imminent or ttc_danger:
            safe_v = max(0.0, min(vF - 3.0, vL - 2.0))
            traci.vehicle.slowDown(follower_id, safe_v, int(1000*0.5))
            traci.vehicle.setSpeed(follower_id, safe_v)
            return

        # 간격 고정 제어(시간헤드웨이 항 제외)
        err = gap_m - DESIRED_GAP
        if err < 0:
            #너무 가까움 -> 감속 위주
            target_v = min(vF, vL) + err * BRAKE_GAIN
            target_v = min(target_v, vL - 0.5) #리더보다 살짝 느리게
        else:
            #너무 멀다 -> 리더 속도 + 보정
            target_v = vL + min(3.0, err * CATCH_GAIN)

        traci.vehicle.setSpeed(follower_id, max(0.0, min(target_v, V_MAX_FOLLOW)))
    except traci.exceptions.TraCIException:
        pass

def _safe_em_decel(vtype_id: str, default: float = 6.0) -> float:
    try:
        return traci.vehicletype.getEmergencyDecel(vtype_id)
    except:
        return default

_boosted = set()

def boost_followers_once():
    """출발 직후 상한 풀기(모든 팔로워 대상)"""
    try:
        current_followers = set(cfg.FOLLOWERS)  # 최신 값 스냅샷
        for vid in traci.simulation.getDepartedIDList():
            if vid in current_followers and vid not in _boosted:
                try:
                    traci.vehicle.setMaxSpeed(vid, 40.0)     # 충분한 상한
                    traci.vehicle.setSpeedMode(vid, 31)      # 기본 모드로
                except traci.exceptions.TraCIException:
                    pass
                _boosted.add(vid)
    except traci.exceptions.TraCIException:
        pass

def enforce_no_lane_change():
    """플래튜닝 차량의 차선 변경 금지"""
    try:
        try:
            vehicles = set(traci.vehicle.getIDList())
        except:
            vehicles = set()
        
        # 모든 플래튜닝 차량에 대해 차선 변경 금지
        all_platoon_vehicles = set()
        for follower_id, leader_id in cfg.FOLLOW_PAIRS:
            all_platoon_vehicles.add(follower_id)
            all_platoon_vehicles.add(leader_id)
        
        for veh_id in all_platoon_vehicles:
            if veh_id in vehicles:
                try:
                    ensure_no_overtake(veh_id)
                except:
                    pass  # 차량이 없으면 무시
                
    except Exception:
        pass