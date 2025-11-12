# platoon.py
import traci
import simulation.config as cfg
from .config import DESIRED_GAP, CATCH_GAIN, V_MAX_FOLLOW, CUT_IN_EXPAND_GAP, CUT_IN_APPROACH_DISTANCE, CUT_IN_DECELERATION

# --- 전역 상태(팔로워별 초기 락) ---
startup_lock_done  = {}  # follower_id -> bool
startup_lock_until = {}  # follower_id -> float
_boosted = set()         # 출발 1회 상한 풀기 마킹

def _ensure_lock_keys(fid):
    if fid not in startup_lock_done:
        startup_lock_done[fid] = False
    if fid not in startup_lock_until:
        startup_lock_until[fid] = 0.0

def _get_leader_info(follower_id: str, max_dist=1000.0):
    """(leader_id, gap[m]) 또는 (None, None)"""
    try:
        info = traci.vehicle.getLeader(follower_id, max_dist)
        if not info:
            return None, None
        lid, gap = info[0], float(info[1])
        return lid, gap
    except traci.exceptions.TraCIException:
        return None, None

# ===== 끼어들기 대응: 실제 앞차를 우선 타겟으로 =====
def _pick_front_target(follower_id: str, designated_leader_id: str, lookahead: float = 250.0):
    """
    follower 바로 앞 차량을 우선 타겟으로 선택.
    - 같은 차선에서 바로 앞차가 있으면 그 차량을 '임시 리더'로 반환
    - 없으면 기존 지정 리더(designated_leader_id)를 사용
    return: (target_id, measured_gap_m)
    """
    try:
        info = traci.vehicle.getLeader(follower_id, lookahead)  # (vehID, gap[m])
    except traci.exceptions.TraCIException:
        info = None

    if info and info[0]:
        front_id, gap = info
        return front_id, float(gap)

    # 앞차 정보가 없으면 지정 리더 기준으로 거리 추정
    try:
        if designated_leader_id in traci.vehicle.getIDList():
            roadL = traci.vehicle.getRoadID(designated_leader_id)
            posL  = traci.vehicle.getLanePosition(designated_leader_id)
            d_road = traci.vehicle.getDrivingDistance(follower_id, roadL, posL)
            if d_road is not None and d_road > 0:
                return designated_leader_id, float(d_road)
    except traci.exceptions.TraCIException:
        pass
    return designated_leader_id, None

# -----------------------------------------------------------------------------
# ① 출발 직후 '초기 락' 지정 (0.5~1.0초 권장)
def ensure_initial_gap_lock(follower_id: str, leader_id: str, lock_duration: float = 0.7):
    """팔로워가 방금 출발했을 때 1회만 호출: 리더 뒤 안정화 구간 확보"""
    try:
        _ensure_lock_keys(follower_id)
        t_now = traci.simulation.getTime()
        startup_lock_done[follower_id]  = True
        startup_lock_until[follower_id] = t_now + lock_duration

        # 안전한 기본 모드 유지, 속도는 리더에 동기화
        if follower_id in traci.vehicle.getIDList() and leader_id in traci.vehicle.getIDList():
            vL = traci.vehicle.getSpeed(leader_id)
            traci.vehicle.setSpeed(follower_id, vL)   # 즉시 동기화
            traci.vehicle.setMaxSpeed(follower_id, max(V_MAX_FOLLOW, vL + 5.0))
    
    except traci.exceptions.TraCIException as e:
        print(f"[LOCK] init failed for {follower_id}: {e}")
    pass

# -----------------------------------------------------------------------------
# ② 락 유지/해제 (매 스텝 호출)
def maintain_or_release_lock(follower_id: str, leader_id: str):
    """락 시간 동안은 리더 속도에 바짝 동기화 + 간단한 거리 보정
       (주의) 끼어들기 동안(타겟이 지정 리더가 아님)에는 락 개입하지 않음
    """
    try:
        _ensure_lock_keys(follower_id)
        t_now = traci.simulation.getTime()
        if not startup_lock_done.get(follower_id, False):
            return

        if t_now <= startup_lock_until.get(follower_id, 0.0):
            if follower_id not in traci.vehicle.getIDList() or leader_id not in traci.vehicle.getIDList():
                return

            # 끼어들기 중이면 락 개입 금지
            target_id, _ = _pick_front_target(follower_id, leader_id, lookahead=250.0)
            if target_id != leader_id:
                return

            vL = traci.vehicle.getSpeed(leader_id)
            lid, gap = _get_leader_info(follower_id)
            if lid != leader_id or gap is None:
                # 리더 인식이 틀어지면 보수적 감속
                try:
                    traci.vehicle.setSpeed(follower_id, max(0.0, vL - 2.0))
                except traci.exceptions.TraCIException:
                    pass
                return

            # 간단 보정: 너무 멀면 +2, 너무 가까우면 -2
            if gap > DESIRED_GAP + 1.0:
                v_cmd = vL + 2.0
            elif gap < DESIRED_GAP - 1.0:
                v_cmd = max(0.0, vL - 2.0)
            else:
                v_cmd = vL

            v_cmd = max(0.0, min(V_MAX_FOLLOW, v_cmd))
            traci.vehicle.setSpeed(follower_id, v_cmd)
        else:
            # 락 기간 종료 → 다음부터는 정상 추종 제어가 담당
            startup_lock_done[follower_id] = False
    except traci.exceptions.TraCIException:
        pass

# -----------------------------------------------------------------------------
# ③ 정상 추종 제어 (매 스텝 호출)
#    끼어든 차량 포함 '실제 앞차'를 타겟으로 15m 유지 (PD + catch-up)
_KP = 0.8
_KD = 0.4
_DT = 0.05   # config의 --step-length와 일치(여기선 0.05s)

def control_follower_speed(follower_id, leader_id):
    try:
        if follower_id not in traci.vehicle.getIDList():
            return

        # 1) 실제 타겟(끼어든 일반차 포함) 선택
        target_id, gap_m = _pick_front_target(follower_id, leader_id, lookahead=250.0)

        # 속도 취득
        vF = traci.vehicle.getSpeed(follower_id)
        vT = traci.vehicle.getSpeed(target_id) if target_id in traci.vehicle.getIDList() else vF

        # 2) gap이 없으면(리더를 못 잡았지만 앞에 있을 수 있음) 보수적 추종
        if gap_m is None:
            v_cmd = min(vT + 1.5, V_MAX_FOLLOW) if vT > 0 else max(0.0, vF - 1.0)
            traci.vehicle.setSpeed(follower_id, v_cmd)
            return

        # 3) 끼어들기 접근 중인지 확인하여 목표 간격 결정
        from simulation.config import CUT_IN_EXPAND_GAP, CUT_IN_ACTIVE_PAIRS
        pair_key = (leader_id, follower_id)
        is_cutin_approaching = pair_key in CUT_IN_ACTIVE_PAIRS
        
        # 접근 중이면 확장된 간격 사용, 아니면 기본 간격 사용
        target_gap = CUT_IN_EXPAND_GAP if is_cutin_approaching else DESIRED_GAP
        
        # PD 제어 (gap → target_gap)
        err  = gap_m - target_gap
        vrel = vT - vF
        a_cmd = _KP * err + _KD * vrel
        v_cmd = vF + a_cmd * _DT

        # 4) Catch-up: 너무 멀면 상한 풀고 보너스
        if err > 10.0:
            try:
                traci.vehicle.setSpeedMode(follower_id, 29)      # 제한속도 검사만 해제
                traci.vehicle.setMaxSpeed(follower_id, 40.0)     # 넉넉한 상한
            except traci.exceptions.TraCIException:
                pass

            catch_bonus = min(6.0, err * CATCH_GAIN * 0.25)      # 최대 +6 m/s
            v_cmd = max(v_cmd, vT + catch_bonus)
            v_cap = 40.0
            v_cmd = max(0.0, min(v_cmd, v_cap))

        elif err < 2.0:
            # 수렴: 기본 안전 모드 복구
            try:
                traci.vehicle.setSpeedMode(follower_id, 31)
            except traci.exceptions.TraCIException:
                pass
            v_cmd = max(0.0, min(v_cmd, V_MAX_FOLLOW))
        else:
            v_cmd = max(0.0, min(v_cmd, V_MAX_FOLLOW))

        # 너무 가까우면 앞차보다 살짝 느리게
        if err < -1.5:
            v_cmd = min(v_cmd, vT - 0.5)

        # 5) 명령 반영 (부드럽게 가고 싶으면 slowDown으로 바꿔도 됨)
        traci.vehicle.setSpeed(follower_id, v_cmd)

    except traci.exceptions.TraCIException:
        pass

# -----------------------------------------------------------------------------
def boost_followers_once():
    """출발 직후 상한 풀기(모든 팔로워 대상) — 끼어들기 중이면 스킵"""
    try:
        current_followers = set(cfg.FOLLOWERS)  # 최신 값 스냅샷
        for vid in traci.simulation.getDepartedIDList():
            if vid in current_followers and vid not in _boosted:
                # 끼어들기 중이면 부스트 금지 (안전)
                # FOLLOW_PAIRS에서 내 지정 리더를 찾아 타겟 비교
                try:
                    designated = next((l for (f, l) in cfg.FOLLOW_PAIRS if f == vid), None)
                except Exception:
                    designated = None
                if designated:
                    tgt, _ = _pick_front_target(vid, designated, lookahead=250.0)
                    if tgt != designated:
                        continue

                try:
                    traci.vehicle.setMaxSpeed(vid, 40.0)     # 충분한 상한
                    traci.vehicle.setSpeedMode(vid, 31)      # 기본 모드로
                except traci.exceptions.TraCIException:
                    pass
                _boosted.add(vid)
    except traci.exceptions.TraCIException:
        pass

# 차량 타입 switch 함수 ---------------------------------------------------------
def switch_to_cacc(veh_id: str) -> bool:
    """팔로워 진입 시 CACC 타입으로 전환 + 추월 금지 + 기본 파라미터 안정화"""
    try:
        if veh_id not in traci.vehicle.getIDList():
            return False

        # 이미 CACC면 스킵
        if traci.vehicle.getTypeID(veh_id) == "truckCACC":
            # 그래도 추월 금지는 보장
            traci.vehicle.setLaneChangeMode(veh_id, 0)
            return True

        # vType 전환
        traci.vehicle.setType(veh_id, "truckCACC")

        # 안전 규칙 유지(31) + 추월 금지(0)
        traci.vehicle.setSpeedMode(veh_id, 31)
        traci.vehicle.setLaneChangeMode(veh_id, 0)

        # 수렴 안정화: 랜덤 속도계수/τ/minGap 정렬
        try:
            traci.vehicle.setSpeedFactor(veh_id, 1.0)
            traci.vehicle.setTau(veh_id, 0.6)
            traci.vehicle.setMinGap(veh_id, 3.0)
        except traci.exceptions.TraCIException:
            pass
        print(f"[DEBUG] {veh_id} -> {traci.vehicle.getTypeID(veh_id)} 전환 완료")
        return True
    except traci.exceptions.TraCIException:
        return False

def switch_to_basic(veh_id: str) -> bool:
    """플래투닝 이탈 시 BASIC으로 복귀 + 기본 차선변경 복구"""
    try:
        if veh_id not in traci.vehicle.getIDList():
            return False

        # 이미 BASIC이면 스킵
        if traci.vehicle.getTypeID(veh_id) == "truckBASIC":
            traci.vehicle.setLaneChangeMode(veh_id, 1621)
            traci.vehicle.setSpeed(veh_id, -1)  # 외부 속도 명령 해제
            return True

        traci.vehicle.setType(veh_id, "truckBASIC")

        # 외부 속도 제어 해제 + 기본 안전 규칙 + 기본 LCMODE
        traci.vehicle.setSpeed(veh_id, -1)
        traci.vehicle.setSpeedMode(veh_id, 31)
        traci.vehicle.setLaneChangeMode(veh_id, 1621)

        try:
            traci.vehicle.setSpeedFactor(veh_id, 1.0)
        except traci.exceptions.TraCIException:
            pass

        print(f"[CACC 이탈] {veh_id} 차량 타입 복귀 완료 → {traci.vehicle.getTypeID(veh_id)}")
        return True
    except traci.exceptions.TraCIException:
        return False

# -----------------------------------------------------------------------------
# 끼어들기 대응: 플래투닝 차량이 거리를 벌려주는 함수
def expand_gap_for_cutin(leader_id: str, follower_id: str, cutin_car_id: str = None):
    """
    일반 차량이 끼어들려고 접근할 때 플래투닝 차량이 거리를 벌려주는 함수.
    리더와 첫 번째 팔로워 사이의 간격을 늘려서 끼어들 공간을 만들어줍니다.
    
    Args:
        leader_id: 플래투닝 리더 차량 ID
        follower_id: 플래투닝 첫 번째 팔로워 차량 ID
        cutin_car_id: 접근 중인 일반 차량 ID (선택)
    """
    try:
        # 차량 존재 확인
        if leader_id not in traci.vehicle.getIDList() or follower_id not in traci.vehicle.getIDList():
            return False

        # 현재 간격 확인
        lid, current_gap = _get_leader_info(follower_id, max_dist=100.0)
        if lid != leader_id or current_gap is None:
            return False

        # 끼어들기 접근 차량이 있으면 거리 확인
        should_expand = False
        if cutin_car_id and cutin_car_id in traci.vehicle.getIDList():
            try:
                # 일반 차량의 위치와 플래투닝 차량들의 위치 비교
                cutin_pos = traci.vehicle.getPosition(cutin_car_id)
                leader_pos = traci.vehicle.getPosition(leader_id)
                follower_pos = traci.vehicle.getPosition(follower_id)
                
                # 유클리드 거리 계산
                import math
                dist_to_leader = math.sqrt((cutin_pos[0] - leader_pos[0])**2 + (cutin_pos[1] - leader_pos[1])**2)
                dist_to_follower = math.sqrt((cutin_pos[0] - follower_pos[0])**2 + (cutin_pos[1] - follower_pos[1])**2)
                
                # 접근 거리 내에 있고, 리더와 팔로워 사이 영역에 있으면 확장
                if dist_to_leader < CUT_IN_APPROACH_DISTANCE or dist_to_follower < CUT_IN_APPROACH_DISTANCE:
                    # 일반 차량이 리더와 팔로워 사이 영역에 있는지 확인
                    if dist_to_leader < dist_to_follower * 1.5:  # 리더 쪽에 더 가까움
                        should_expand = True
            except traci.exceptions.TraCIException:
                pass
        else:
            # cutin_car_id가 없으면 항상 확장 (접근 중으로 가정)
            should_expand = True

        if not should_expand:
            return False

        # 현재 간격이 목표 간격보다 작으면 확장
        if current_gap < CUT_IN_EXPAND_GAP:
            vL = traci.vehicle.getSpeed(leader_id)
            vF = traci.vehicle.getSpeed(follower_id)
            
            # 리더는 약간 감속 (안전하게)
            target_leader_speed = max(0.0, vL - CUT_IN_DECELERATION * 0.3)
            
            # 팔로워는 리더보다 더 감속하여 거리를 벌림
            gap_error = CUT_IN_EXPAND_GAP - current_gap
            # 거리 오차가 클수록 더 많이 감속
            decel_amount = min(CUT_IN_DECELERATION, gap_error * 0.2)
            target_follower_speed = max(0.0, vF - decel_amount)
            
            # 리더 속도 조정 (너무 많이 감속하지 않도록)
            try:
                if vL > 5.0:  # 최소 속도 유지
                    traci.vehicle.slowDown(leader_id, target_leader_speed, 1.5)
            except traci.exceptions.TraCIException:
                pass
            
            # 팔로워 속도 조정 (거리 벌리기)
            try:
                if vF > 5.0:  # 최소 속도 유지
                    traci.vehicle.slowDown(follower_id, target_follower_speed, 1.2)
                    # 일시적으로 DESIRED_GAP을 늘려서 제어 로직이 거리를 유지하도록
                    # control_follower_speed에서 사용할 수 있도록 전역 상태에 표시
            except traci.exceptions.TraCIException:
                pass
            
            return True
        else:
            # 이미 충분한 간격이 있으면 정상 제어로 복귀
            return False

    except traci.exceptions.TraCIException:
        return False

def control_follower_speed_with_cutin_expansion(follower_id, leader_id, expanded_gap_target=None):
    """
    끼어들기 접근 시 확장된 간격을 유지하도록 하는 제어 함수.
    expanded_gap_target이 None이면 기본 DESIRED_GAP 사용.
    """
    try:
        if follower_id not in traci.vehicle.getIDList():
            return

        # 목표 간격 설정
        target_gap = expanded_gap_target if expanded_gap_target is not None else DESIRED_GAP

        # 실제 타겟 선택
        target_id, gap_m = _pick_front_target(follower_id, leader_id, lookahead=250.0)

        # 속도 취득
        vF = traci.vehicle.getSpeed(follower_id)
        vT = traci.vehicle.getSpeed(target_id) if target_id in traci.vehicle.getIDList() else vF

        # gap이 없으면 보수적 추종
        if gap_m is None:
            v_cmd = min(vT + 1.5, V_MAX_FOLLOW) if vT > 0 else max(0.0, vF - 1.0)
            traci.vehicle.setSpeed(follower_id, v_cmd)
            return

        # PD 제어 (gap → target_gap)
        err = gap_m - target_gap
        vrel = vT - vF
        a_cmd = _KP * err + _KD * vrel
        v_cmd = vF + a_cmd * _DT

        # Catch-up: 너무 멀면 상한 풀고 보너스
        if err > 10.0:
            try:
                traci.vehicle.setSpeedMode(follower_id, 29)
                traci.vehicle.setMaxSpeed(follower_id, 40.0)
            except traci.exceptions.TraCIException:
                pass

            catch_bonus = min(6.0, err * CATCH_GAIN * 0.25)
            v_cmd = max(v_cmd, vT + catch_bonus)
            v_cap = 40.0
            v_cmd = max(0.0, min(v_cmd, v_cap))

        elif err < 2.0:
            # 수렴: 기본 안전 모드 복구
            try:
                traci.vehicle.setSpeedMode(follower_id, 31)
            except traci.exceptions.TraCIException:
                pass
            v_cmd = max(0.0, min(v_cmd, V_MAX_FOLLOW))
        else:
            v_cmd = max(0.0, min(v_cmd, V_MAX_FOLLOW))

        # 너무 가까우면 앞차보다 살짝 느리게
        if err < -1.5:
            v_cmd = min(v_cmd, vT - 0.5)

        # 명령 반영
        traci.vehicle.setSpeed(follower_id, v_cmd)

    except traci.exceptions.TraCIException:
        pass