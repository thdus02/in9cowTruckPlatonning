# simulation/config.py
# SUMO configuration 
Sumo_config = [
    'sumo-gui',
    '-c', 'map/final.sumocfg',        # 상대경로로 map 폴더 지정
    '--step-length', '0.05',         
    '--delay', '100',                 
    '--lateral-resolution', '0.1',
    '--collision.action', 'warn',   
    '--collision.mingap-factor', '1.0'
]

# === 플래투닝 / 제어 상수 === 
DESIRED_GAP   = 15.0  #리더 - 팔로워 사이 간격
CATCH_GAIN    = 0.45  #멀 때 빨리 따라붙게
BRAKE_GAIN    = 0.35  #가까울 때 살살 떼기 
V_MAX_FOLLOW  = 33.0  #팔로워 최대 속도(≈ 119km/h)

# CACC용: 정지 간격 + 시간 헤드웨이
STANDSTILL_GAP = 5.0   # d0: 완전 정지 시 기본 간격 [m]
TIME_HEADWAY   = 0.5   # Th: 시간 간격 #0.6으로 하면 15.9정도 유지 0.5로하면 14.0~14.2정도 유지함

# 플래투닝 참여 버튼 활성화 거리 (m)
PLATOON_JOIN_DISTANCE = 300.0 

# ===== 전역 상태(런타임 갱신) =====
FOLLOW_PAIRS = []
FOLLOWERS = []
VEHICLE_DISTANCES = {}
STARTED = set()
NEARBY_PLATOON = {} # 300m 참여 후보: {미참여 차량: [(플래투닝 차량, 거리), ...]}
CUT_IN_ACTIVE_PAIRS = {}  # {(leader_id, follower_id): True} - 끼어들기 접근 중인 플래투닝 쌍

# === 끼어들기 대응 상수 ===
CUT_IN_EXPAND_GAP = 38.0  # 끼어들기 접근 시 목표 간격 (기본 15m -> 38m: 차량길이 5m + 앞안전거리 15m + 뒤안전거리 15m + 여유 3m)
CUT_IN_APPROACH_DISTANCE = 50.0  # 끼어들기 접근 감지 거리 (m) - 실제 접근 시에만 감지
CUT_IN_DECELERATION = 2.0  # 끼어들기 대응 시 감속량 (m/s) - 더 적극적으로 감속
CUT_IN_EXPANSION_RATE = 0.3  # 거리 확장 속도 계수 

# ======= 플래투닝 전용 트럭 판별 함수 =======
def is_platoon_truck(vid: str) -> bool:
    """
    VehicleViewer 콤보박스에 보여줄 플래투닝 트럭인지 판별.
    전제: 플래투닝 트럭 ID는 'Veh...'로 시작.
    """
    return vid.startswith("Veh")