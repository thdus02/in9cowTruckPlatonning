# simulation/safety.py
import traci

def init_safety_defaults():
    """
    traci.start() 직후 1회 호출:
    - 현재 존재하는 모든 차량에 SpeedMode 기본값(31) 적용
    """
    try:
        vehs = set(traci.vehicle.getIDList())

        # SpeedMode 기본값 적용(전체 차량에 적용해도 안전)
        for vid in vehs:
            try:
                traci.vehicle.setSpeedMode(vid, 31)
            except traci.exceptions.TraCIException:
                pass

        for vtype in ("truckCACC", "truckBASIC"):
            try:
                traci.vehicletype.setEmergencyDecel(vtype, 6.0)
            except traci.exceptions.TraCIException:
                pass

    except traci.exceptions.TraCIException:
        pass