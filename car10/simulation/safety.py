# simulation/safety.py
import traci
import simulation.config as cfg  # ← 항상 모듈로 읽어서 최신 FOLLOW_PAIRS/FOLLOWERS 사용

def init_safety_defaults():
    """
    traci.start() 직후 1회 호출:
    - 현재 존재하는 모든 차량에 SpeedMode 기본값(31) 적용
    - vType의 emergencyDecel 표준화
    """
    try:
        vehs = set(traci.vehicle.getIDList())

        # SpeedMode 기본값 적용(전체 차량에 적용해도 안전)
        for vid in vehs:
            try:
                traci.vehicle.setSpeedMode(vid, 31)
            except traci.exceptions.TraCIException:
                pass

        # vType emergency decel 정규화 (없는 타입은 건너뜀)
        for vtype in ("truckCACC", "truckBASIC"):
            try:
                traci.vehicletype.setEmergencyDecel(vtype, 6.0)
            except traci.exceptions.TraCIException:
                pass

    except traci.exceptions.TraCIException:
        # SUMO가 아직 준비 안 됐거나 일시적 예외
        pass