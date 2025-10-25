import os, sys, time, msvcrt, traci

# SUMO_HOME 등록
if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

# SUMO 실행 커맨드 (GUI 켠 상태 유지)
sumo_cmd = [
    "sumo-gui",
    "-c", "parking.sumocfg",
    "--step-length", "0.1",
]

END_TIME = 300.0          # 시뮬 유지 시간
VEHS = ["veh_0", "veh_1"] # 키로 출발시킬 차량들

def resume(vid):
    try:
        traci.vehicle.resume(vid)
        print(f"[OK] {vid} 출발")
    except traci.TraCIException as e:
        print(f"[WARN] {vid} 출발 실패: {e}")

def main():
    traci.start(sumo_cmd)
    print("=== 키 도움말 ===")
    print("  1: veh_0 출발   2: veh_1 출발   a: 모두 출발   q: 종료")
    print("※ 콘솔 창(이 창)에 포커스를 두고 키를 누르세요.")

    try:
        while traci.simulation.getTime() < END_TIME:
            traci.simulationStep()

            # 콘솔 키 입력(논블로킹)
            if msvcrt.kbhit():
                ch = msvcrt.getwch().lower()
                if ch == '1': resume("veh_0")
                elif ch == '2': resume("veh_1")
                elif ch == 'a':
                    for v in VEHS: resume(v)
                elif ch == 'q':
                    print("[INFO] 사용자 종료"); break

            time.sleep(0.01)  # CPU 점유율 완화

    finally:
        traci.close()

if __name__ == "__main__":
    main()
