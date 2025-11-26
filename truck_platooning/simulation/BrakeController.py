# simulation/BrakeController.py
# 모든 차량 브레이크 제어 (클릭마다 감속량 누적, 이후 자동 복귀)
class BrakeController:
    def __init__(self, traci_mod, sim_dt=0.05,
                 ramp_down_per_s=1.5,   # 버튼 누르는 동안 factor 감소 속도 (초당)
                 ramp_up_per_s=0.8,     # 버튼 떼고 나서 factor 회복 속도 (초당)
                 min_factor=0.0):       # 최저 factor (0.0이면 정지까지 허용)
        self.traci = traci_mod
        self.SIM_DT = sim_dt
        self.ramp_down = ramp_down_per_s
        self.ramp_up = ramp_up_per_s
        self.min_factor = min_factor
        self.factor = 1.0
        self.braking = False
        self.leader_id = None 

    def set_leader(self, leader_id: str):
        """리더 차량 ID를 동적으로 설정"""
        self.leader_id = leader_id

    def _apply(self):
        lid = self.leader_id
        if not lid or lid not in self.traci.vehicle.getIDList():
            return
        self.traci.vehicle.setSpeed(lid, -1) # 잔여 명령 해제
        self.traci.vehicle.setSpeedFactor(lid, self.factor)

    def on_brake_press(self, _=None):
        """버튼을 꾹 누르는 순간"""
        self.braking = True

    def on_brake_release(self, _=None):
        """버튼을 떼는 순간"""
        self.braking = False

    def update(self):
        """매 step 회복(브레이크를 누르지 않아도 자동 복귀)"""
        lid = self.leader_id
        if not lid or lid not in self.traci.vehicle.getIDList():
            return
        if self.braking:
            self.factor = max(self.min_factor, self.factor - self.ramp_down * self.SIM_DT)
        else:
            self.factor = min(1.0, self.factor + self.ramp_up * self.SIM_DT)
        self._apply()