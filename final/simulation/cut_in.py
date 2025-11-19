import traci
from collections import deque
import time
import math

_last_approach_print = 0.0  # 옆차 접근 감지 로그 레이트 리밋
_last_recognition_print = 0.0  # 인식 확인 로그 레이트 리밋

DESIRED_GAP = 15.0
APPROACH_VF = 1.15
HOLD_CHANGE_SEC = 2.0

DEBUG_CUTIN = False  # 기본 디버그 로그 비활성화


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

        # 차선 변경 감지용
        self._previous_lane_id = None  # 이전 차선 ID 저장
        self._lane_change_detected = False  # 차선 변경 감지 플래그

        # 끼어들기 성공(한 번만 처리) 플래그 + 타임스탬프
        self._recognized_once = False
        self._last_recog_ts = 0.0

    def log(self, msg):
        # 디버그 메시지는 내부 저장만 하고 출력하지 않음
        self._last_msgs.append(msg)
        # print(f"[CUT-IN] {msg}")  # 필요 시 활성화

    def ready(self):
        return self.state in ("idle", "done")

    def start(self, leader_id: str, follower_id: str, car_id: str = "VehCut"):
        """일반차 생성 + 옆차선에서 접근 대기 (끼어들기/나가기는 버튼으로 수동)"""
        if not self.ready():
            return False

        self.car_id = car_id
        self.leader = leader_id
        self.follower = follower_id
        self._want_cut_in = False
        self._want_cut_out = False
        self.state = "spawn"

        # 차선 변경 감지 관련 초기화
        self._previous_lane_id = None
        self._lane_change_detected = False

        # 기존 플래그 제거 (새로 시작할 때)
        import simulation.config as cfg
        pair_key = (leader_id, follower_id)
        if pair_key in cfg.CUT_IN_ACTIVE_PAIRS:
            del cfg.CUT_IN_ACTIVE_PAIRS[pair_key]

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
            return False
        self._want_cut_in = True
        return True

    def request_cut_out(self):
        """메인차선에서 옆차선으로 복귀"""
        if self.state not in ("in_main",):
            return False
        self._want_cut_out = True
        return True

    # -------- 메인 루프에서 step마다 호출 --------
    def tick(self):
        if self.state in ("idle", "done"):
            return

        # 리더/팔로워 유효성
        for vid in [self.leader, self.follower]:
            if not vid or vid not in traci.vehicle.getIDList():
                self.state = "done"
                return

        if self.state == "spawn":
            laneL = traci.vehicle.getLaneID(self.leader)
            self._pick_side_lane(laneL)

            posL = traci.vehicle.getLanePosition(self.leader)
            lenL = traci.vehicle.getLength(self.leader)
            spawn_pos = max(0.0, posL - lenL - DESIRED_GAP * 2)

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
            self.state = "approach"

            try:
                self._previous_lane_id = side_lane_id
            except:
                pass
            self._lane_change_detected = False
            return

        if self.state == "approach":
            # 차선 변경 감지 (실제로 차선 변경이 시작되는지 확인)
            self._detect_lane_change()

            # 차선 변경이 감지되면 간격 확장 시작
            if self._lane_change_detected:
                import simulation.config as cfg
                pair_key = (self.leader, self.follower)
                if pair_key not in cfg.CUT_IN_ACTIVE_PAIRS:
                    cfg.CUT_IN_ACTIVE_PAIRS[pair_key] = True
                    print(f"[플래그 설정] CUT_IN_ACTIVE_PAIRS에 ({self.leader}, {self.follower}) 추가됨")

            # 간격 확장 유지 (차선 변경 감지 후)
            import simulation.config as cfg
            pair_key = (self.leader, self.follower)
            if pair_key in cfg.CUT_IN_ACTIVE_PAIRS:
                self._expand_platoon_gap_for_cutin()
                # 끼어드는 차량도 계속 감속하여 간격 확장에 협조
                self._slow_down_cutin_vehicle()

            # 수동 '끼어들기' 버튼을 기다린다.
            if self._want_cut_in:
                self._want_cut_in = False

                # 차선 변경 명령 실행 (깜빡이 켜기)
                steps = int(HOLD_CHANGE_SEC / traci.simulation.getDeltaT())
                traci.vehicle.changeLane(self.car_id, self.target_lane, steps)
                vL = traci.vehicle.getSpeed(self.leader)
                traci.vehicle.slowDown(self.car_id, max(vL, 9.0), 1.2)

                # 깜빡이를 켰으므로 즉시 플래그 설정 (차선 변경 감지 전에 미리 설정)
                if pair_key not in cfg.CUT_IN_ACTIVE_PAIRS:
                    cfg.CUT_IN_ACTIVE_PAIRS[pair_key] = True
                    print(f"[깜빡이 켜짐] 끼어들기 버튼 클릭 - 플래그 즉시 설정: ({self.leader}, {self.follower})")

                try:
                    self._previous_lane_id = traci.vehicle.getLaneID(self.car_id)
                except traci.exceptions.TraCIException:
                    pass

            # 차선 변경이 완료되면 상태 변경
            try:
                current_lane_id = traci.vehicle.getLaneID(self.car_id)
                if current_lane_id and self._lane_index(current_lane_id) == self.target_lane:
                    self.state = "in_main"
                    # 컷인 직후 한 번 체크
                    self._check_cutin_recognition()
            except traci.exceptions.TraCIException:
                pass
            return

        if self.state == "in_main":
            # 매 스텝 팔로워가 일반차를 앞차로 인식하는지 체크
            self._check_cutin_recognition()

            # 끼어들기 완료 후에도 잠시 간격 확장 유지 (CUT_IN_ACTIVE_PAIRS 플래그는
            # _check_cutin_recognition에서 적절한 시점에 해제)
            self._expand_platoon_gap_for_cutin()

            if self._want_cut_out:
                self._want_cut_out = False
                import simulation.config as cfg
                pair_key = (self.leader, self.follower)
                if pair_key in cfg.CUT_IN_ACTIVE_PAIRS:
                    del cfg.CUT_IN_ACTIVE_PAIRS[pair_key]

                steps = int(HOLD_CHANGE_SEC / traci.simulation.getDeltaT())
                traci.vehicle.changeLane(self.car_id, self.side_lane, steps)
                vC = traci.vehicle.getSpeed(self.car_id)
                traci.vehicle.slowDown(self.car_id, vC + 5.0, 1.0)
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
                    self.state = "done"
            except traci.TraCIException:
                self.state = "done"
            return

    def _detect_lane_change(self):
        """차선 변경(깜빡이) 시작을 감지하는 함수 - 차선 변경이 실제로 시작되는 순간 감지"""
        try:
            if self.car_id not in traci.vehicle.getIDList():
                return

            current_lane_id = traci.vehicle.getLaneID(self.car_id)
            if not current_lane_id:
                return

            current_lane_idx = self._lane_index(current_lane_id)

            if self._previous_lane_id:
                prev_lane_idx = self._lane_index(self._previous_lane_id)

                if prev_lane_idx == self.side_lane and current_lane_idx == self.target_lane:
                    if not self._lane_change_detected:
                        self._lane_change_detected = True
                        print(f"[차선 변경 감지] 옆차({self.car_id})가 차선 변경 시작 - 플래투닝 그룹 간격 확장 시작")
                elif prev_lane_idx == self.side_lane and current_lane_idx != self.side_lane:
                    if not self._lane_change_detected:
                        self._lane_change_detected = True
                        print(f"[차선 변경 감지] 옆차({self.car_id})가 차선 변경 시작 - 플래투닝 그룹 간격 확장 시작")

            self._previous_lane_id = current_lane_id

        except traci.exceptions.TraCIException:
            pass
        except Exception:
            pass

    def _slow_down_cutin_vehicle(self):
        """끼어드는 차량을 감속시켜 간격 확장에 협조하도록 함 (부드러운 감속)"""
        try:
            if self.car_id not in traci.vehicle.getIDList():
                return
            if self.leader not in traci.vehicle.getIDList():
                return

            vL = traci.vehicle.getSpeed(self.leader)
            vC = traci.vehicle.getSpeed(self.car_id)

            target_speed = max(8.0, vL - 3.0)

            if vC > target_speed + 0.5:
                traci.vehicle.slowDown(self.car_id, target_speed, 1.5)
        except Exception:
            pass

    def _expand_platoon_gap_for_cutin(self):
        """CUT_IN_ACTIVE_PAIRS 플래그가 켜졌을 때, 현재 간격을 모니터링만 함.
        실제로는 platoon.control_follower_speed() 쪽에서 제어.
        """
        global _last_approach_print
        try:
            if self.car_id not in traci.vehicle.getIDList():
                import simulation.config as cfg
                pair_key = (self.leader, self.follower)
                if pair_key in cfg.CUT_IN_ACTIVE_PAIRS:
                    del cfg.CUT_IN_ACTIVE_PAIRS[pair_key]
                return

            if self.leader not in traci.vehicle.getIDList() or self.follower not in traci.vehicle.getIDList():
                import simulation.config as cfg
                pair_key = (self.leader, self.follower)
                if pair_key in cfg.CUT_IN_ACTIVE_PAIRS:
                    del cfg.CUT_IN_ACTIVE_PAIRS[pair_key]
                return

            import simulation.config as cfg
            pair_key = (self.leader, self.follower)

            if pair_key not in cfg.CUT_IN_ACTIVE_PAIRS:
                return

            from simulation.config import CUT_IN_EXPAND_GAP

            try:
                info = traci.vehicle.getLeader(self.follower, 150.0)
                if info and info[0] == self.leader:
                    current_gap = float(info[1])

                    now = time.time()
                    if now - _last_approach_print > 2.0:
                        if current_gap < CUT_IN_EXPAND_GAP:
                            print(
                                f"[간격 확장 중] 플래투닝 그룹 간격 확장 중 - "
                                f"현재: {current_gap:.1f}m, 목표: {CUT_IN_EXPAND_GAP:.1f}m "
                                f"(실제 제어는 platoon.py에서 처리)"
                            )
                        else:
                            print(
                                f"[간격 확장 완료] 플래투닝 그룹 간격 확보됨 - "
                                f"현재: {current_gap:.1f}m, 목표: {CUT_IN_EXPAND_GAP:.1f}m"
                            )
                        _last_approach_print = now

            except traci.exceptions.TraCIException:
                pass
        except Exception:
            pass

    def _check_cutin_recognition(self):
        """
        팔로워가 끼어든 일반차(self.car_id)를 앞차로 인식하면
        딱 한 번만 CUT_IN_ACTIVE_PAIRS 플래그를 해제해서
        다시 DESIRED_GAP 기준으로 줄어들 수 있게 한다.
        """
        try:
            if self._recognized_once:
                return

            now = time.time()
            if now - self._last_recog_ts < 1.0:
                return
            self._last_recog_ts = now

            ids = set(traci.vehicle.getIDList())
            if self.follower not in ids or self.car_id not in ids:
                return

            info = traci.vehicle.getLeader(self.follower, 150.0)
            if not info:
                return

            front_id, gap = info[0], float(info[1])

            if front_id == self.car_id:
                print(
                    f"[끼어들기 인식 완료] follower={self.follower}가 "
                    f"{self.car_id}를 앞차로 인식 (gap={gap:.1f}m) → 간격 플래그 해제"
                )

                self._clear_cutin_flag()
                self._recognized_once = True

        except Exception:
            pass

    def _clear_cutin_flag(self):
        """CUT_IN_ACTIVE_PAIRS 플래그 해제 유틸"""
        try:
            import simulation.config as cfg
            pair_key = (self.leader, self.follower)
            if pair_key in cfg.CUT_IN_ACTIVE_PAIRS:
                del cfg.CUT_IN_ACTIVE_PAIRS[pair_key]
        except Exception:
            pass
