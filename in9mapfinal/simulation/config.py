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
CATCH_GAIN    = 0.30  #앞차와 거리가 멀어질 때 가속 계수
BRAKE_GAIN    = 0.50  #앞차와 너무 가까워졌을 때 감속 계수 
V_MAX_FOLLOW  = 33.0  #팔로워 최대 속도
KMH_LIMIT     = 160.0
NEEDLE_RADIUS = 90
CENTER_X, CENTER_Y = 150, 150

# === 팔로워 구성: 여기만 바꾸면 추가/변경 반영됨
FOLLOW_PAIRS = [("Follower1", "Leader"),
                ("Follower2", "Follower1"),
                ("Follower3", "Follower2")]
FOLLOWERS = [p[0] for p in FOLLOW_PAIRS]
#플래투닝 제어 적용되는 팔로워들 차량만 정리하기 위해