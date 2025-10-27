import os, sys, traci
import tkinter as tk

# === SUMO_HOME 등록 ===
if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

# === SUMO 실행 커맨드 (GUI 켠 상태 유지) ===
sumo_cmd = [
    "sumo-gui",
    "-c", "parking.sumocfg",
    "--step-length", "0.1",
]

VEHS = ["veh_0", "veh_1", "veh_2"]  # 버튼으로 제어할 차량들

# --- 간격 제어 파라미터 ---
DESIRED_GAP = 20.0   # m (트럭 플래투닝 목표 간격)
KP          = 0.35   # 간단한 P 제어 이득 (원하면 조정)
V_MIN       = 0.0
V_MAX       = 33.0   # m/s ≈ 119 km/h (vType maxSpeed와 맞추면 좋음)

def clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)

def resume(vid):
    """주차장 stop(triggered)에 묶인 차량을 출발시킴"""
    try:
        traci.vehicle.resume(vid)
        print(f"[OK] {vid} 출발")
    except traci.TraCIException as e:
        print(f"[WARN] {vid} 출발 실패: {e}")

class App:
    def __init__(self, root):
        self.root = root
        self.root.title("SUMO Control (Leader Select + Start)")
        self.root.resizable(False, False)

        # 안내
        tk.Label(root, text="1) 리더 선택  2) 각 차량 '출발' 또는 '모두 출발' 클릭").grid(
            row=0, column=0, columnspan=6, padx=10, pady=(10, 6), sticky="w"
        )

        # 리더 선택 (라디오 버튼)
        tk.Label(root, text="Set Leader:").grid(row=1, column=0, padx=(10,0), pady=6, sticky="w")
        self.leader_var = tk.StringVar(value="")  # 아직 미지정
        for i, vid in enumerate(VEHS, start=1):
            rb = tk.Radiobutton(root, text=vid, value=vid, variable=self.leader_var)
            rb.grid(row=1, column=i, padx=4, pady=6, sticky="w")

        # 차량별 출발 버튼
        for i, vid in enumerate(VEHS):
            tk.Button(root, text=f"{vid} 출발", width=12, command=lambda v=vid: resume(v)).grid(
                row=2, column=i, padx=6, pady=6
            )

        # 모두 출발 / 종료
        tk.Button(root, text="모두 출발", width=12, command=self.resume_all).grid(
            row=2, column=len(VEHS), padx=6, pady=6
        )
        tk.Button(root, text="종료", width=12, command=self.on_quit).grid(
            row=3, column=len(VEHS), padx=6, pady=(6, 10)
        )

        # TraCI 시작
        traci.start(sumo_cmd)
        self._closed = False

        # 시뮬 루프 시작 (시간 제한 없음)
        self.step_loop()

        # 창(X) 닫기 처리
        self.root.protocol("WM_DELETE_WINDOW", self.on_quit)

    def resume_all(self):
        for v in VEHS:
            resume(v)

    def follower_control(self):
        """선택된 리더를 기준으로 나머지 차량 간단 P 제어 (gap ≈ 20 m)"""
        leader_id = self.leader_var.get()
        if leader_id not in VEHS:
            return  # 리더 미지정이면 아무것도 안함

        try:
            vL = traci.vehicle.getSpeed(leader_id)
        except traci.TraCIException:
            return

        for v in VEHS:
            if v == leader_id:
                # 리더는 자유 주행 (사용자가 버튼으로 출발시켜야 함)
                continue

            try:
                # 내 앞의 차량 정보 (id, gap[m])를 얻어 리더인지 확인
                info = traci.vehicle.getLeader(v, 250.0)  # 250m 범위 내
                if info is None:
                    continue
                front_id, gap = info[0], float(info[1])

                if front_id != leader_id:
                    # 바로 앞이 리더가 아니면 강제 속도 명령을 피함 (자연 주행)
                    continue

                # 간단한 P 제어: v_cmd = v_leader + Kp*(gap - D)
                e = gap - DESIRED_GAP
                v_cmd = clamp(vL + KP * e, V_MIN, V_MAX)

                # 속도 명령
                traci.vehicle.setSpeed(v, v_cmd)
            except traci.TraCIException:
                pass

    def step_loop(self):
        if self._closed:
            return
        try:
            traci.simulationStep()
            # 리더 지정되어 있으면 팔로워 간격 제어
            self.follower_control()
        except Exception:
            self.on_quit()
            return
        # 계속 진행
        self.root.after(10, self.step_loop)

    def on_quit(self):
        if not self._closed:
            self._closed = True
            try:
                traci.close(False)
            except Exception:
                pass
            self.root.after(50, self.root.destroy)

def main():
    root = tk.Tk()
    App(root)
    root.mainloop()

if __name__ == "__main__":
    main()
