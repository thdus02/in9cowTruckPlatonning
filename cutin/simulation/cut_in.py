# simulation/cut_in.py
import traci
from collections import deque
import time
_last_debug_print = 0.0  # 파일 상단에 추가

DESIRED_GAP = 15.0
APPROACH_VF = 1.15
HOLD_CHANGE_SEC = 2.0

DEBUG_CUTIN = True

class CutInManager:
    """
    상태:
      idle -> spawn -> approach -> in_main (끼어든 상태) -> cut_out -> done
    - 수동 트리거:
        request_cut_in()  : 옆차선 -> 메인차선 진입
        request_cut_out() : 메인차선 -> 옆차선 복귀
    """
    def __init__(self):
        self.state = "idle"
        self.car_id = None
        self.target_lane = 0
        self.side_lane = 1
        self.leader = None
        self.follower = None
        self._last_msgs = deque(maxlen=10)

        # 수동 트리거 플래그
        self._want_cut_in = False
        self._want_cut_out = False

    def log(self, msg):
        self._last_msgs.append(msg)
        print(f"[CUT-IN] {msg}")

    def ready(self):
        return self.state in ("idle", "done")

    def start(self, leader_id: str, follower_id: str, car_id: str = "VehCut"):
        """일반차 생성 + 옆차선에서 접근 대기 (끼어들기/나가기는 버튼으로 수동)"""
        if not self.ready():
            self.log("이미 시나리오가 진행 중입니다.")
            return False

        self.car_id = car_id
        self.leader = leader_id
        self.follower = follower_id
        self._want_cut_in = False
        self._want_cut_out = False
        self.state = "spawn"
        self.log(f"시작: leader={leader_id}, follower={follower_id}, car={car_id}")
        return True

    # -------- 유틸 --------
    @staticmethod
    def _lane_index(lane_id: str) -> int:
        return int(lane_id.split("_")[-1])

    @staticmethod
    def _edge_id(lane_id: str) -> str:
        return lane_id.split("_")[0]

    def _ensure_dynamic_route(self, leader_id: str):
        rid = traci.vehicle.getRouteID(leader_id)
        if rid:
            edges = traci.route.getEdges(rid)
        else:
            edges = traci.vehicle.getRoute(leader_id)
        new_rid = f"r_cut_{leader_id}"
        try:
            traci.route.add(new_rid, edges)
        except traci.TraCIException:
            pass
        return new_rid

    def _pick_side_lane(self, lane_id: str):
        base_edge = self._edge_id(lane_id)
        num_lanes = traci.edge.getLaneNumber(base_edge)
        self.target_lane = 0
        self.side_lane = 1 if num_lanes >= 2 else 0

    # -------- 수동 트리거 API --------
    def request_cut_in(self):
        """옆차선에서 메인차선으로 끼어들기"""
        if self.state not in ("approach",):
            self.log("지금은 끼어들기 요청을 수행할 수 없는 상태입니다.")
            return False
        self._want_cut_in = True
        self.log("끼어들기 요청됨")
        return True

    def request_cut_out(self):
        """메인차선에서 옆차선으로 복귀"""
        if self.state not in ("in_main",):
            self.log("지금은 나가기 요청을 수행할 수 없는 상태입니다.")
            return False
        self._want_cut_out = True
        self.log("옆차선 복귀 요청됨")
        return True

    # -------- 메인 루프에서 step마다 호출 --------
    def tick(self):
        if self.state in ("idle", "done"):
            return

        # 리더/팔로워 유효성
        for vid in [self.leader, self.follower]:
            if not vid or vid not in traci.vehicle.getIDList():
                self.log("리더/팔로워 사라짐 → 종료")
                self.state = "done"
                return

        if self.state == "spawn":
            laneL = traci.vehicle.getLaneID(self.leader)
            self._pick_side_lane(laneL)

            posL = traci.vehicle.getLanePosition(self.leader)
            lenL = traci.vehicle.getLength(self.leader)
            spawn_pos = max(0.0, posL - lenL - DESIRED_GAP*2)

            route_id = self._ensure_dynamic_route(self.leader)
            car = self.car_id

            if car in traci.vehicle.getIDList():
                try:
                    traci.vehicle.remove(car)
                except traci.TraCIException:
                    pass

            traci.vehicle.add(vehID=car, routeID=route_id, typeID="carCUT", depart="now")
            traci.vehicle.setSpeedMode(car, 0)
            traci.vehicle.setSpeedFactor(car, APPROACH_VF)

            side_lane_id = f"{self._edge_id(laneL)}_{self.side_lane}"
            traci.vehicle.moveTo(car, side_lane_id, spawn_pos)
            self.log(f"일반차 생성: {car} @ {side_lane_id}:{spawn_pos}")
            self.state = "approach"
            return

        if self.state == "approach":
            # 수동 '끼어들기' 버튼을 기다린다.
            if self._want_cut_in:
                self._want_cut_in = False
                steps = int(HOLD_CHANGE_SEC / traci.simulation.getDeltaT())
                traci.vehicle.changeLane(self.car_id, self.target_lane, steps)
                vL = traci.vehicle.getSpeed(self.leader)
                traci.vehicle.slowDown(self.car_id, max(vL, 9.0), 1.2)
                self.log(f"끼어들기 실행 → lane {self.target_lane}")
                self.state = "in_main"
                # ★ 컷인 직후 바로 한 번 체크
                self._debug_follow_recognition()
            return

        if self.state == "in_main":
            # ★ 매 스텝 팔로워가 일반차를 앞차로 인식하는지 체크
            self._debug_follow_recognition()

            if self._want_cut_out:
                self._want_cut_out = False
                steps = int(HOLD_CHANGE_SEC / traci.simulation.getDeltaT())
                traci.vehicle.changeLane(self.car_id, self.side_lane, steps)
                vC = traci.vehicle.getSpeed(self.car_id)
                traci.vehicle.slowDown(self.car_id, vC + 5.0, 1.0)
                self.log("옆차선 복귀 실행")
                self.state = "cut_out"
            return

        if self.state == "cut_out":
            try:
                laneC = traci.vehicle.getLaneID(self.car_id)
                if self._lane_index(laneC) == self.side_lane:
                    try:
                        traci.vehicle.setSpeedMode(self.car_id, 31)
                        traci.vehicle.setSpeedFactor(self.car_id, 1.0)
                    except traci.TraCIException:
                        pass
                    self.log("옆차선 복귀 완료 → 종료")
                    self.state = "done"
            except traci.TraCIException:
                self.state = "done"
            return

        
    def _debug_follow_recognition(self):
        """팔로워가 지금 누구를 앞차로 인식하는지 0.5초마다 한 번씩 로그"""
        global _last_debug_print
        if not DEBUG_CUTIN:
            return
        try:
            now = time.time()
            # 0.5초 이내면 출력 생략
            if now - _last_debug_print < 0.5:
                return
            _last_debug_print = now

            ids = set(traci.vehicle.getIDList())
            if self.follower not in ids:
                self.log(f"DEBUG skip: follower {self.follower} not in sim (state={self.state})")
                return

            # 바로 앞 차량 정보
            info = traci.vehicle.getLeader(self.follower, 250.0)
            tgt_id, gap = (info[0], float(info[1])) if info and info[0] else (None, None)

            # 색상은 제거, 로그만 출력
            if tgt_id == self.car_id:
                self.log(f"follower={self.follower} → target=VehCut gap={gap:.2f}m (인식 OK)")
            elif tgt_id is not None:
                self.log(f"follower={self.follower} → target={tgt_id} gap={gap:.2f}m")
            else:
                self.log(f"follower={self.follower} → target=None")

        except Exception as e:
            self.log(f"DEBUG error: {type(e).__name__}: {e}")


