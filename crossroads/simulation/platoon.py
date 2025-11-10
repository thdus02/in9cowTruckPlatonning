# platoon.py
import traci
import simulation.config as cfg
from .config import DESIRED_GAP, CATCH_GAIN, V_MAX_FOLLOW

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
        print(f"[LOCK] moveTo failed for {follower_id}: {e}")
    pass

# -----------------------------------------------------------------------------
# ② 락 유지/해제 (매 스텝 호출)
def maintain_or_release_lock(follower_id: str, leader_id: str):
    """락 시간 동안은 리더 속도에 바짝 동기화 + 간단한 거리 보정"""
    try:
        _ensure_lock_keys(follower_id)
        t_now = traci.simulation.getTime()
        if not startup_lock_done.get(follower_id, False):
            return

        if t_now <= startup_lock_until.get(follower_id, 0.0):
            if follower_id not in traci.vehicle.getIDList() or leader_id not in traci.vehicle.getIDList():
                return
            vL = traci.vehicle.getSpeed(leader_id)
            lid, gap = _get_leader_info(follower_id)
            # 리더가 원하는 리더인지 확인 (끼어듦 대비)
            if lid != leader_id or gap is None:
                # 리더 인식이 틀어지면 보수적 감속
                try: traci.vehicle.setSpeed(follower_id, max(0.0, vL - 2.0))
                except: pass
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
#    출발 직후 튐을 막기 위해 PD 기반으로 gap 오차와 상대속도(vrel)를 사용
_KP = 0.8
_KD = 0.4
_DT = 0.05   # config의 --step-length와 일치(여기선 0.05s)

def control_follower_speed(follower_id, leader_id):
    try:
        if follower_id not in traci.vehicle.getIDList():
            return

        vF = traci.vehicle.getSpeed(follower_id)
        vL = traci.vehicle.getSpeed(leader_id) if leader_id in traci.vehicle.getIDList() else 0.0

        # 0) 지정 리더의 "도로 기준" 거리 계산(리더 인식 실패 대비)
        d_road = None
        try:
            if leader_id in traci.vehicle.getIDList():
                roadL   = traci.vehicle.getRoadID(leader_id)
                posL    = traci.vehicle.getLanePosition(leader_id)
                # follower → (leader의 edge,pos)까지 주행거리(앞이면 +)
                d_road  = traci.vehicle.getDrivingDistance(follower_id, roadL, posL)
        except traci.exceptions.TraCIException:
            pass

        # 1) SUMO가 바로 앞차로 잡아준 leader 정보
        lid_gap = traci.vehicle.getLeader(follower_id, 1000.0)
        has_lock_leader = bool(lid_gap and lid_gap[0] == leader_id)
        gap_m = float(lid_gap[1]) if has_lock_leader else (d_road if d_road is not None and d_road > 0 else None)

        # 2) gap이 없으면 (리더 뒤라는 확신이 없으면) 보수적이나 "조금은" 추종 가속 허용
        if gap_m is None:
            # 리더를 못 잡았지만 지정 리더가 앞에 있을 수 있음 → 살짝 추종 가속 허용
            v_cmd = min(vL + 1.5, V_MAX_FOLLOW) if vL > 0 else max(0.0, vF - 1.0)
            traci.vehicle.setSpeed(follower_id, v_cmd)
            return

        # 3) PD + 캐치업
        err  = gap_m - DESIRED_GAP
        vrel = vL - vF

        a_cmd = 0.8 * err + 0.4 * vrel
        v_cmd = vF + a_cmd * 0.05

        if err > 10.0:
            # ---------- Catch-up 모드: 제한속도 검사 해제 + 상한 올림 ----------
            try:
                traci.vehicle.setSpeedMode(follower_id, 29)      # 31에서 maxSpeed 검사만 끈 모드
                traci.vehicle.setMaxSpeed(follower_id, 40.0)     # catch-up 상한(필요하면 더)
                # (선택) 조금 더 여유 주고 싶으면:
                # traci.vehicle.setSpeedFactor(follower_id, 1.15)
            except traci.exceptions.TraCIException:
                pass

            catch_bonus = min(6.0, err * CATCH_GAIN * 0.25)      # 최대 +6 m/s 보너스
            v_cmd = max(v_cmd, vL + catch_bonus)

            # ★ 여기서는 V_MAX_FOLLOW로 캡하지 말고 setMaxSpeed 범위로만 제한
            v_cap = 40.0
            v_cmd = max(0.0, min(v_cmd, v_cap))

        elif err < 2.0:
            # ---------- 수렴: 원래 안전 모드로 복구 ----------
            try:
                traci.vehicle.setSpeedMode(follower_id, 31)
                # (선택) speedFactor도 원복
                # traci.vehicle.setSpeedFactor(follower_id, 1.0)
            except traci.exceptions.TraCIException:
                pass

            # 수렴 구간에선 기존 캡 사용
            v_cmd = max(0.0, min(v_cmd, V_MAX_FOLLOW))
        else:
            # 일반 구간: 기존 캡
            v_cmd = max(0.0, min(v_cmd, V_MAX_FOLLOW))

        # 너무 가까우면 리더보다 살짝 느리게
        if err < -1.5:
            v_cmd = min(v_cmd, vL - 0.5)

        traci.vehicle.setSpeed(follower_id, v_cmd)

    except traci.exceptions.TraCIException:
        pass

# -----------------------------------------------------------------------------
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
        # (truckCACC vType에 이미 맞게 정의돼 있다면 아래는 생략 가능)
        try:
            traci.vehicle.setSpeedFactor(veh_id, 1.0)  # 랜덤성 제거
            traci.vehicle.setTau(veh_id, 0.6)          # CACC에 맞는 반응지연
            traci.vehicle.setMinGap(veh_id, 3.0)
        except traci.exceptions.TraCIException:
            pass
        vtype = traci.vehicle.getTypeID(veh_id)
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
            # 기본 차선 변경 모드만 보장
            traci.vehicle.setLaneChangeMode(veh_id, 1621)
            traci.vehicle.setSpeed(veh_id, -1)  # 외부 속도 명령 해제
            return True

        traci.vehicle.setType(veh_id, "truckBASIC")

        # 외부 속도 제어 해제 + 기본 안전 규칙 + 기본 LCMODE
        traci.vehicle.setSpeed(veh_id, -1)
        traci.vehicle.setSpeedMode(veh_id, 31)
        traci.vehicle.setLaneChangeMode(veh_id, 1621)

        # (선택) 기본 주행 랜덤성 복원
        try:
            traci.vehicle.setSpeedFactor(veh_id, 1.0)
        except traci.exceptions.TraCIException:
            pass

        vtype = traci.vehicle.getTypeID(veh_id)
        print(f"[CACC 이탈] {veh_id} 차량 타입 복귀 완료 → {vtype}")

        return True
    except traci.exceptions.TraCIException:
        return False
