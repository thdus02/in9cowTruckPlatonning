import os, sys, traci
if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
else:
    sys.exit("Please declare 'SUMO_HOME'")

PARK_EXIT = 80.59   # pa_2 endPos
PASS_BUF  = 10.0    # 통과 인정 버퍼
TAIL_GAP  = 35.0    # 꼬리와 최소 간격
MIN_V     = 3.0     # 꼬리 최소 속도
TRIO = ["t_0", "t_1", "t_2"]

def passed(v):
    edge = traci.vehicle.getRoadID(v)
    pos  = traci.vehicle.getLanePosition(v)
    return (edge != "E0") or (pos >= PARK_EXIT + PASS_BUF)

def tail_of_trio():
    info = []
    for vid in TRIO:
        if vid in traci.vehicle.getIDList():
            info.append((vid, traci.vehicle.getRoadID(vid), traci.vehicle.getLanePosition(vid)))
    same = [x for x in info if x[1] == "E0"]
    return (min(same, key=lambda x: x[2])[0] if same else (min(info, key=lambda x: x[2])[0] if info else None))

def run():
    released = False
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        if released or "t_3" not in traci.vehicle.getIDList(): 
            continue
        if all(v in traci.vehicle.getIDList() and passed(v) for v in TRIO):
            tail = tail_of_trio()
            if not tail: 
                continue
            tail_edge = traci.vehicle.getRoadID(tail)
            tail_pos  = traci.vehicle.getLanePosition(tail)
            tail_v    = traci.vehicle.getSpeed(tail)
            safe_gap  = (tail_edge != "E0") or (tail_pos >= PARK_EXIT + TAIL_GAP)
            if safe_gap and tail_v >= MIN_V:
                traci.vehicle.setType("t_3", "truckCACC")  # CACC로 전환
                traci.vehicle.setSpeedMode("t_3", 31)
                traci.vehicle.setSpeed("t_3", tail_v)      # 초기속도 맞추기(선택)
                traci.vehicle.resume("t_3")                # 트리거 해제 → 출발
                released = True
                print("[INFO] t_3 merged from pa_2 at tail.")
    traci.close()

if __name__ == "__main__":
    traci.start([
        "sumo-gui", "-c", "osm.sumocfg",
        "--step-length", "0.1",   # tau=0.6 쓰면 권장
        "--start"
    ])
    run()
