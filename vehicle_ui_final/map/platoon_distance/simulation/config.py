# Step 4: Define SUMO configuration (원본 주석 보존)
Sumo_config = [
    'sumo-gui',
    '-c', 'map/final.sumocfg',          # 상대경로로 map 폴더 지정
    '--step-length', '0.05',          # 더 부드럽게(원하면 0.1로)
    '--delay', '100',                 # GUI 느리게(원하면 조절/삭제)
    '--lateral-resolution', '0.1',
    '--collision.action', 'warn',     # ★ 충돌 시 제거/텔레포트 대신 경고
    '--collision.mingap-factor', '1.0'# ★ 안전거리 계산에 minGap 반영(보수적)
]

# === Platooning control constants === (Step 6 일부)
DESIRED_GAP   = 15.0  #리더 - 팔로워 사이 간격
CATCH_GAIN    = 1.2  #멀 때 빨리 따라붙게(기존보다 ↑)
BRAKE_GAIN    = 0.35  #가까울 때 살살 떼기 
V_MAX_FOLLOW  = 33.0  #팔로워 최대 속도(≈ 119km/h)
KMH_LIMIT     = 160.0
NEEDLE_RADIUS = 90
CENTER_X, CENTER_Y = 150, 150

FOLLOW_PAIRS = []
FOLLOWERS = []

# 비플래튜닝 차량과 플래튜닝 맨 뒷 차량 간 거리 정보
# {vehicle_id: distance_in_meters}
VEHICLE_DISTANCES = {}

#시작 누른 차량 기록
STARTED = set()