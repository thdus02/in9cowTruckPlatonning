# simulation/cut_in.py
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

    def log(self, msg):
        # 디버그 메시지는 내부 저장만 하고 출력하지 않음
        self._last_msgs.append(msg)
        # print(f"[CUT-IN] {msg}")  # 제거됨

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
            self.state = "approach"
            # 초기 차선 ID 저장
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
            
            # 간격 확장 유지 (차선 변경 감지 후)
            import simulation.config as cfg
            pair_key = (self.leader, self.follower)
            if pair_key in cfg.CUT_IN_ACTIVE_PAIRS:
                self._expand_platoon_gap_for_cutin()
            
            # 수동 '끼어들기' 버튼을 기다린다.
            if self._want_cut_in:
                self._want_cut_in = False
                
                # 차선 변경 명령 실행
                steps = int(HOLD_CHANGE_SEC / traci.simulation.getDeltaT())
                traci.vehicle.changeLane(self.car_id, self.target_lane, steps)
                vL = traci.vehicle.getSpeed(self.leader)
                traci.vehicle.slowDown(self.car_id, max(vL, 9.0), 1.2)
                
                # 이전 차선 ID 저장 (차선 변경 감지용)
                try:
                    self._previous_lane_id = traci.vehicle.getLaneID(self.car_id)
                except traci.exceptions.TraCIException:
                    pass
                
            # 차선 변경이 완료되면 상태 변경 (차선 변경 감지 여부와 관계없이)
            try:
                current_lane_id = traci.vehicle.getLaneID(self.car_id)
                if current_lane_id and self._lane_index(current_lane_id) == self.target_lane:
                    # 차선 변경 완료 - 상태 변경
                    self.state = "in_main"
                    # ★ 컷인 직후 바로 한 번 체크
                    self._check_cutin_recognition()
            except traci.exceptions.TraCIException:
                pass
            return

        if self.state == "in_main":
            # ★ 매 스텝 팔로워가 일반차를 앞차로 인식하는지 체크
            self._check_cutin_recognition()
            
            # 끼어들기 완료 후에도 간격 확장 유지 (일반 차량이 안전하게 들어올 수 있도록)
            # 플래그가 설정되어 있으면 계속 확장 유지
            self._expand_platoon_gap_for_cutin()

            if self._want_cut_out:
                self._want_cut_out = False
                # 나가기 시작하면 플래그 제거 (정상 간격으로 복귀)
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
            
            # 차선 변경 명령이 실행된 후 차선 인덱스가 변경되기 시작하는 것을 감지
            if self._previous_lane_id:
                prev_lane_idx = self._lane_index(self._previous_lane_id)
                
                # 차선이 변경되기 시작했는지 확인
                # 옆차선에서 목표 차선으로 변경되는 경우
                if prev_lane_idx == self.side_lane and current_lane_idx == self.target_lane:
                    if not self._lane_change_detected:
                        self._lane_change_detected = True
                        print(f"[차선 변경 감지] 옆차({self.car_id})가 차선 변경 시작 - 플래투닝 그룹 간격 확장 시작")
                # 차선이 변경되고 있는 경우 (옆차선에서 다른 차선으로)
                elif prev_lane_idx == self.side_lane and current_lane_idx != self.side_lane:
                    if not self._lane_change_detected:
                        self._lane_change_detected = True
                        print(f"[차선 변경 감지] 옆차({self.car_id})가 차선 변경 시작 - 플래투닝 그룹 간격 확장 시작")
            
            # 현재 차선 ID 저장
            self._previous_lane_id = current_lane_id
            
        except traci.exceptions.TraCIException:
            pass
        except Exception as e:
            pass

    def _expand_platoon_gap_for_cutin(self):
        """끼어들기 의도가 있을 때 플래투닝 차량이 거리를 벌려주도록 하는 함수 (차선 변경 감지 후에만 작동)"""
        global _last_approach_print
        try:
            # 차량 존재 확인
            if self.car_id not in traci.vehicle.getIDList():
                # 접근 차량이 없으면 플래그 제거
                import simulation.config as cfg
                pair_key = (self.leader, self.follower)
                if pair_key in cfg.CUT_IN_ACTIVE_PAIRS:
                    del cfg.CUT_IN_ACTIVE_PAIRS[pair_key]
                return
            
            if self.leader not in traci.vehicle.getIDList() or self.follower not in traci.vehicle.getIDList():
                # 플래투닝 차량이 없으면 플래그 제거
                import simulation.config as cfg
                pair_key = (self.leader, self.follower)
                if pair_key in cfg.CUT_IN_ACTIVE_PAIRS:
                    del cfg.CUT_IN_ACTIVE_PAIRS[pair_key]
                return

            # 플래그가 설정되어 있을 때만 확장 (차선 변경 감지 후에만)
            import simulation.config as cfg
            pair_key = (self.leader, self.follower)
            
            if pair_key not in cfg.CUT_IN_ACTIVE_PAIRS:
                return  # 차선 변경이 감지되지 않았으면 확장하지 않음
            
            # 현재 간격 확인
            from simulation.config import CUT_IN_EXPAND_GAP, CUT_IN_EXPANSION_RATE
            
            try:
                info = traci.vehicle.getLeader(self.follower, 150.0)
                if info and info[0] == self.leader:
                    current_gap = float(info[1])
                    
                    # 목표 간격보다 작으면 거리 확장
                    if current_gap < CUT_IN_EXPAND_GAP:
                        vL = traci.vehicle.getSpeed(self.leader)
                        vF = traci.vehicle.getSpeed(self.follower)
                        
                        # 거리 오차 계산
                        gap_error = CUT_IN_EXPAND_GAP - current_gap
                        
                        # 확장 속도 계수 적용 (더 빠르게 확장)
                        expansion_factor = 1.0 + (gap_error / CUT_IN_EXPAND_GAP) * CUT_IN_EXPANSION_RATE
                        
                        # 리더는 약간 감속 (거리 오차에 비례, 더 적극적으로)
                        leader_decel = min(1.5, gap_error * 0.12 * expansion_factor)
                        target_leader_speed = max(8.0, vL - leader_decel)
                        
                        # 팔로워는 더 많이 감속하여 거리를 벌림 (더 적극적으로)
                        follower_decel = min(3.5, gap_error * 0.25 * expansion_factor)
                        target_follower_speed = max(8.0, vF - follower_decel)
                        
                        # 속도 조정 (목표 간격까지 빠르게 확보)
                        try:
                            if vL > target_leader_speed + 0.2:
                                traci.vehicle.slowDown(self.leader, target_leader_speed, 1.2)
                            if vF > target_follower_speed + 0.2:
                                traci.vehicle.slowDown(self.follower, target_follower_speed, 1.0)
                        except traci.exceptions.TraCIException:
                            pass
                        
                        # 로그 출력 (1초마다)
                        now = time.time()
                        if now - _last_approach_print > 1.0:
                            print(f"[간격 확장 중] 플래투닝 그룹 간격 확장 중 - 현재: {current_gap:.1f}m, 목표: {CUT_IN_EXPAND_GAP:.1f}m")
                            _last_approach_print = now
                    
            except traci.exceptions.TraCIException:
                pass
        except Exception as e:
            pass

    def _check_cutin_recognition(self):
        """팔로워가 끼어든 옆차를 앞차로 인식하는지 확인하고 출력"""
        global _last_recognition_print
        try:
            now = time.time()
            # 1초마다 출력
            if now - _last_recognition_print < 1.0:
                return
            _last_recognition_print = now

            ids = set(traci.vehicle.getIDList())
            if self.follower not in ids or self.car_id not in ids:
                return

            # 바로 앞 차량 정보
            info = traci.vehicle.getLeader(self.follower, 250.0)
            tgt_id, gap = (info[0], float(info[1])) if info and info[0] else (None, None)

            # 끼어든 옆차를 인식하는지 확인
            if tgt_id == self.car_id:
                print(f"[끼어들기 인식 확인] 플래투닝 팔로워({self.follower})가 옆차({self.car_id})를 앞차로 인식 중 - 간격: {gap:.2f}m")
            elif tgt_id == self.leader:
                print(f"[끼어들기 인식 확인] 플래투닝 팔로워({self.follower})가 리더({self.leader})를 앞차로 인식 중 - 간격: {gap:.2f}m")
            elif tgt_id is not None:
                print(f"[끼어들기 인식 확인] 플래투닝 팔로워({self.follower})가 다른 차량({tgt_id})을 앞차로 인식 중 - 간격: {gap:.2f}m")
            else:
                print(f"[끼어들기 인식 확인] 플래투닝 팔로워({self.follower})가 앞차를 인식하지 못함")

        except Exception as e:
            pass

