import os
import sys
import traci
import tkinter as tk

# === SUMO_HOME 설정 ===
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

# === 여기에서 사용자 환경에 맞게 수정 ===
SUMO_CMD = [
    "sumo-gui",
    "-c", "parking.sumocfg",          # 당신의 .sumocfg 파일 경로
    "--step-length", "0.1",
]

# 차량 번호 → 차량 ID 매핑 (routes.xml의 trip id와 일치해야 함)
# 1 → t_0, 2 → t_1, 3 → t_2 (없으면 자동 안내)
VEHICLE_MAP = {
    1: "t_0",
    2: "t_1",
    3: "t_2",
}

# --- Tkinter UI 구성 ---
root = tk.Tk()
root.title("주차장 출발 제어 (1/2/3)")

status = tk.StringVar(value="대기 중")

def get_all_vehicle_ids():
    try:
        return set(traci.vehicle.getIDList())
    except traci.TraCIException:
        return set()

def is_stopped_or_parked(veh_id: str) -> bool:
    """주차장 정차 포함: 정차 상태면 True"""
    try:
        return traci.vehicle.isStopped(veh_id)
    except traci.TraCIException:
        return False

def try_resume(veh_id: str):
    """특정 차량 출발(주차 해제)"""
    all_ids = get_all_vehicle_ids()
    if veh_id not in all_ids:
        status.set(f"[알림] 차량 {veh_id} 가 현재 시뮬레이션에 없음")
        return
    if is_stopped_or_parked(veh_id):
        try:
            traci.vehicle.resume(veh_id)
            status.set(f"[출발] 차량 {veh_id} 출발 명령 보냄")
        except traci.TraCIException as e:
            status.set(f"[오류] {veh_id} resume 실패: {e}")
    else:
        status.set(f"[무시] {veh_id} 는 현재 주차/정지 상태가 아님")

def on_number(num: int):
    veh_id = VEHICLE_MAP.get(num)
    if not veh_id:
        status.set(f"[알림] 번호 {num} 에 매핑된 차량이 없습니다")
        return
    try_resume(veh_id)

# 버튼 UI
btn_frame = tk.Frame(root, padx=8, pady=8)
btn_frame.pack()

for n in (1, 2, 3):
    tk.Button(btn_frame, text=f"{n} 번 차량 출발", width=16,
              command=lambda x=n: on_number(x)).grid(row=0, column=n-1, padx=6)

# 키보드 바인딩 (키 '1','2','3')
root.bind("1", lambda e: on_number(1))
root.bind("2", lambda e: on_number(2))
root.bind("3", lambda e: on_number(3))

tk.Label(root, textvariable=status, anchor="w").pack(fill="x", padx=8, pady=6)

# --- 시뮬레이션 루프를 Tkinter after로 돌리기 ---
def sim_step():
    try:
        # 더 진행할 차량/사람/컨테이너가 있으면 step
        if traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
            root.after(50, sim_step)  # 20Hz(50ms) 정도로 갱신
        else:
            status.set("시뮬레이션 종료")
            traci.close()
    except traci.TraCIException as e:
        status.set(f"시뮬레이션 오류: {e}")
        try:
            traci.close(False)
        except Exception:
            pass

def main():
    traci.start(SUMO_CMD)
    root.after(100, sim_step)
    root.mainloop()

if __name__ == "__main__":
    main()
