# Step 4: Define SUMO configuration 
Sumo_config = [
    'sumo-gui',
    '-c', 'map/final.sumocfg',          # 상대경로로 map 폴더 지정
    '--step-length', '0.05',          # 더 부드럽게(원하면 0.1로)
    '--delay', '100',                 # GUI 느리게(원하면 조절/삭제)
    '--lateral-resolution', '0.1',
    '--collision.action', 'warn',     # ★ 충돌 시 제거/텔레포트 대신 경고
    '--collision.mingap-factor', '1.0'# ★ 안전거리 계산에 minGap 반영(보수적)
]

# === Platooning control constants === 
DESIRED_GAP   = 15.0  #리더 - 팔로워 사이 간격
CATCH_GAIN    = 1.2  #멀 때 빨리 따라붙게(기존보다 ↑)
BRAKE_GAIN    = 0.35  #가까울 때 살살 떼기 
V_MAX_FOLLOW  = 33.0  #팔로워 최대 속도(≈ 119km/h)
MAX_GAP       = 30.0  #허용 최대 갭 (m)
KMH_LIMIT     = 160.0

FOLLOW_PAIRS = []
FOLLOWERS = []

# 전역 상태
VEHICLE_DISTANCES = {}
STARTED = set()

BRAKE_FACTORS = {}  # {veh_id: 0.0~1.0}

# 300m 참여 후보: {미참여 차량: [(플래투닝 차량, 거리), ...]}
NEARBY_PLATOON = {}

# === 끼어들기 대응 상수 ===
CUT_IN_EXPAND_GAP = 38.0  # 끼어들기 접근 시 목표 간격 (기본 15m -> 38m: 차량길이 5m + 앞안전거리 15m + 뒤안전거리 15m + 여유 3m)
CUT_IN_APPROACH_DISTANCE = 50.0  # 끼어들기 접근 감지 거리 (m) - 실제 접근 시에만 감지
CUT_IN_DECELERATION = 2.0  # 끼어들기 대응 시 감속량 (m/s) - 더 적극적으로 감속
CUT_IN_EXPANSION_RATE = 0.3  # 거리 확장 속도 계수 (높을수록 빠르게 확장)

# 끼어들기 접근 중인 플래투닝 쌍 (접근 중일 때 확장된 간격 유지)
CUT_IN_ACTIVE_PAIRS = {}  # {(leader_id, follower_id): True} - 접근 중인 플래투닝 쌍