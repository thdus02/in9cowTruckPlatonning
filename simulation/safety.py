import traci
from .config import FOLLOWERS

def init_safety_defaults():
    """Step 5 이후: SpeedMode/비상감속 등 안전 초기화"""
    try:
        traci.vehicle.setSpeedMode("Leader", 31)
        for fid in FOLLOWERS:
            traci.vehicle.setSpeedMode(fid, 31)

        # rou의 vType id 기준
        traci.vehicletype.setEmergencyDecel("leadtruck", 6.0)
        traci.vehicletype.setEmergencyDecel("truckCACC", 6.0)
    except traci.exceptions.TraCIException:
        pass
