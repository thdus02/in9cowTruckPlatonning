# Step 4: Define SUMO configuration (원본 주석 보존)
Sumo_config = [
    'sumo-gui',
    '-c', 'map/osm.sumocfg',          # 상대경로로 map 폴더 지정
    '--step-length', '0.05',          # 더 부드럽게(원하면 0.1로)
    '--delay', '100',                 # GUI 느리게(원하면 조절/삭제)
    '--lateral-resolution', '0.1',
    '--collision.action', 'warn',     # ★ 충돌 시 제거/텔레포트 대신 경고
    '--collision.mingap-factor', '1.0'# ★ 안전거리 계산에 minGap 반영(보수적)
]

# === Platooning control constants === (Step 6 일부)
DESIRED_GAP   = 15.0
CATCH_GAIN    = 0.20
BRAKE_GAIN    = 0.50
V_MAX_FOLLOW  = 33.0
KMH_LIMIT     = 160.0
NEEDLE_RADIUS = 90
CENTER_X, CENTER_Y = 150, 150

# === 팔로워 구성: 여기만 바꾸면 추가/변경 반영됨
FOLLOW_PAIRS = [("Follower1", "Leader"),
                ("Follower2", "Follower1")]
FOLLOWERS = [f for f, _ in FOLLOW_PAIRS]
