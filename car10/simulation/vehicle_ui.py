import tkinter as tk
from tkinter import ttk, messagebox
import math
import simulation.config as cfg
from simulation.brakeController import BrakeController
from simulation.platoon import (
    switch_to_cacc,         # 참여 시 사용
    switch_to_basic,        # 이탈 시 사용
)
from simulation.config import is_platoon_truck, PLATOON_JOIN_DISTANCE

# --- Lane-change hold & pending merge schedulers (전역 상태) ---
LANE_MODE_RESTORE = {}   # vid -> restore_time (sim time)
PENDING_MERGE = {}       # vid -> (front_id, target_lane_idx)

# --- 재합류 안티-오버테이크 가드 (전역 상태) ---
JOIN_COOLDOWN = {}        # vid -> until_time (sim time)
COOLDOWN_MARGIN = 1.5     # m/s, 앞차보다 이만큼 느리게 유지
COOLDOWN_SEC = 5.0        # 재합류 후 n초간 적용

# --- 이탈 보호(Leave Guard): 앞차가 빠질 때 뒤차 감속/고정 (전역 상태) ---
LEAVE_GUARD = {}          # rear_vid -> (until_time, departing_vid)
LEAVE_GUARD_SEC = 4.0     # 앞차 이탈 보장 시간
LEAVE_MARGIN = 2.0        # 앞차(이탈 차량/혹은 새 front)보다 최소 이만큼 느리게

# ===== 거리 계산 유틸 =====
def _euclid_m(traci_mod, a, b):
    """두 차량의 유클리드 거리(m). SUMO 좌표는 미터 단위."""
    try:
        xa, ya = traci_mod.vehicle.getPosition(a)
        xb, yb = traci_mod.vehicle.getPosition(b)
        dx, dy = xa - xb, ya - yb
        return (dx*dx + dy*dy) ** 0.5
    except Exception:
        return float('inf')

def _nearby_fallback(traci_mod, me, chain, limit_m):
    """
    cfg.NEARBY_PLATOON이 비어있는 경우 즉석에서 후보 산출.
    반환: [(vid, dist), ...] (dist 오름차순)
    """
    if not chain:
        return []
    cand = []
    for v in chain:
        if v == me:
            continue
        d = _euclid_m(traci_mod, me, v)
        if d <= float(limit_m) + 1e-6:
            cand.append((v, d))
    cand.sort(key=lambda x: x[1])
    return cand

# ===== 차선 변경 유틸 =====
def _adjacent_lane_or_self(traci_mod, lane_id, cur_idx, prefer_right=True):
    """현재 엣지의 차선 수에 맞춰 인접 차선 인덱스를 고른다."""
    try:
        edge_id = traci_mod.lane.getEdgeID(lane_id)
        nlanes  = traci_mod.edge.getLaneNumber(edge_id)
    except Exception:
        return cur_idx

    # 우측(번호 +1) 우선, 없으면 좌측(번호 -1)
    if prefer_right and cur_idx + 1 < nlanes:
        return cur_idx + 1
    if cur_idx - 1 >= 0:
        return cur_idx - 1
    # 반대 방향도 안 되면 제자리
    if not prefer_right and cur_idx + 1 < nlanes:
        return cur_idx + 1
    return cur_idx

def _smooth_change_lane(traci_mod, vid, target_lane_index, hold_sec=15.0):
    """
    CACC라도 잠깐 laneChange 허용 → changeLane 시도 → hold_sec 뒤에 자동 복구.
    즉시 0으로 잠그면 변경이 실패할 수 있어 '예약 복구' 방식 사용.
    """
    try:
        traci_mod.vehicle.setLaneChangeMode(vid, 1621)  # 잠깐 허용
        traci_mod.vehicle.changeLane(vid, int(target_lane_index), float(hold_sec))
        sim_t = traci_mod.simulation.getTime()
        LANE_MODE_RESTORE[vid] = sim_t + float(hold_sec)
    except Exception:
        pass

def _tick_lane_mode_restore(traci_mod):
    """laneChangeMode 예약 복구: hold_sec 지난 뒤 CACC면 0(추월금지)로 재잠금."""
    try:
        sim_t = traci_mod.simulation.getTime()
        for vid, t_restore in list(LANE_MODE_RESTORE.items()):
            if sim_t >= t_restore:
                try:
                    if traci_mod.vehicle.getTypeID(vid) == "truckCACC":
                        traci_mod.vehicle.setLaneChangeMode(vid, 0)
                except Exception:
                    pass
                LANE_MODE_RESTORE.pop(vid, None)
    except Exception:
        pass

def _tick_pending_merge(traci_mod):
    """
    front와 같은 엣지에 도달하면 그때 lane change 시도.
    (같은 엣지가 아니면 changeLane 자체가 불가)
    """
    try:
        for vid, (front, tgt_idx) in list(PENDING_MERGE.items()):
            try:
                if (vid not in traci_mod.vehicle.getIDList()) or (front not in traci_mod.vehicle.getIDList()):
                    PENDING_MERGE.pop(vid, None)
                    continue
                my_lane_id     = traci_mod.vehicle.getLaneID(vid)
                front_lane_id  = traci_mod.vehicle.getLaneID(front)
                my_edge        = traci_mod.lane.getEdgeID(my_lane_id)
                front_edge     = traci_mod.lane.getEdgeID(front_lane_id)
                if my_edge == front_edge:
                    nlanes = traci_mod.edge.getLaneNumber(my_edge)
                    tgt_i  = max(0, min(int(tgt_idx), int(nlanes) - 1))
                    _smooth_change_lane(traci_mod, vid, tgt_i, hold_sec=4.0)
                    # 보류 합류가 실행된 순간부터 쿨다운 시작
                    try:
                        sim_t = traci_mod.simulation.getTime()
                        JOIN_COOLDOWN[vid] = sim_t + COOLDOWN_SEC
                    except Exception:
                        pass
                    PENDING_MERGE.pop(vid, None)
            except Exception:
                PENDING_MERGE.pop(vid, None)
                continue
    except Exception:
        pass

def _tick_join_cooldown(traci_mod):
    """재합류 직후 일정 시간 동안 추월 금지 + 속도 상한 강제."""
    try:
        sim_t = traci_mod.simulation.getTime()
        pairs = list(getattr(cfg, "FOLLOW_PAIRS", []))
        # 쿨다운 정리만 필요한 경우
        if not pairs:
            for vid, until_t in list(JOIN_COOLDOWN.items()):
                if sim_t >= until_t:
                    JOIN_COOLDOWN.pop(vid, None)
            return

        f2l = {f: l for (f, l) in pairs}  # follower -> leader

        for vid, until_t in list(JOIN_COOLDOWN.items()):
            if (vid not in traci_mod.vehicle.getIDList()):
                JOIN_COOLDOWN.pop(vid, None)
                continue

            # 쿨다운 만료 체크
            if sim_t >= until_t:
                JOIN_COOLDOWN.pop(vid, None)
                try:
                    if traci_mod.vehicle.getTypeID(vid) == "truckCACC":
                        traci_mod.vehicle.setLaneChangeMode(vid, 0)
                except Exception:
                    pass
                continue

            # 내 front(리더) 식별
            front = f2l.get(vid)
            if not front or (front not in traci_mod.vehicle.getIDList()):
                continue

            # 1) 차선변경 금지 유지
            try:
                traci_mod.vehicle.setLaneChangeMode(vid, 0)
            except Exception:
                pass

            # 2) 속도 상한: v_me ≤ v_front - margin
            try:
                vF = traci_mod.vehicle.getSpeed(front)
                vCap = max(4.0, vF - COOLDOWN_MARGIN)
                vNow = traci_mod.vehicle.getSpeed(vid)
                if vNow > vCap:
                    traci_mod.vehicle.setSpeed(vid, vCap)
            except Exception:
                pass

            # 3) 같은 엣지에서 내 pos가 front를 앞섰으면 즉시 더 강하게 감속
            try:
                my_lane  = traci_mod.vehicle.getLaneID(vid)
                fr_lane  = traci_mod.vehicle.getLaneID(front)
                my_edge  = traci_mod.lane.getEdgeID(my_lane)
                fr_edge  = traci_mod.lane.getEdgeID(fr_lane)
                if my_edge == fr_edge:
                    my_pos = traci_mod.vehicle.getLanePosition(vid)
                    fr_pos = traci_mod.vehicle.getLanePosition(front)
                    if my_pos >= fr_pos - 0.5:
                        vF = traci_mod.vehicle.getSpeed(front)
                        traci_mod.vehicle.setSpeed(vid, max(3.0, vF - (COOLDOWN_MARGIN + 1.0)))
            except Exception:
                pass
    except Exception:
        pass

def _tick_leave_guard(traci_mod):
    """
    앞차가 이탈하는 동안 뒤차(rear)가 급가속/추월로 끼어들지 못하도록
    잠깐(LEAVE_GUARD_SEC) 감속 및 차선 변경 금지.
    """
    try:
        sim_t = traci_mod.simulation.getTime()
        for rear, (until_t, departing) in list(LEAVE_GUARD.items()):
            # rear나 departing이 사라졌으면 종료
            if (rear not in traci_mod.vehicle.getIDList()) or (departing not in traci_mod.vehicle.getIDList()):
                LEAVE_GUARD.pop(rear, None)
                continue

            if sim_t >= until_t:
                LEAVE_GUARD.pop(rear, None)
                # rear가 CACC면 차선변경 금지는 기본 0으로 유지
                try:
                    if traci_mod.vehicle.getTypeID(rear) == "truckCACC":
                        traci_mod.vehicle.setLaneChangeMode(rear, 0)
                except Exception:
                    pass
                continue

            # rear는 차선 변경 금지
            try:
                traci_mod.vehicle.setLaneChangeMode(rear, 0)
            except Exception:
                pass

            # rear 속도 캡: departing(이탈 차량) 또는 현재 체인상 front 둘 중 더 낮은 기준을 따라감
            try:
                v_dep = traci_mod.vehicle.getSpeed(departing)
            except Exception:
                v_dep = 6.0

            # rear의 현재 체인상 front(리더)를 찾아 캡을 더 보수적으로
            try:
                pairs = list(getattr(cfg, "FOLLOW_PAIRS", []))
                f2l = {f:l for (f,l) in pairs}
                front = f2l.get(rear)
                v_front = traci_mod.vehicle.getSpeed(front) if front and (front in traci_mod.vehicle.getIDList()) else v_dep
            except Exception:
                v_front = v_dep

            v_cap = max(3.0, min(v_dep - LEAVE_MARGIN, v_front - LEAVE_MARGIN))
            try:
                v_now = traci_mod.vehicle.getSpeed(rear)
                if v_now > v_cap:
                    traci_mod.vehicle.setSpeed(rear, v_cap)
            except Exception:
                pass

            # departing과 rear가 같은 엣지에서 너무 가까우면 더 강하게 감속
            try:
                dep_lane = traci_mod.vehicle.getLaneID(departing)
                rear_lane= traci_mod.vehicle.getLaneID(rear)
                dep_edge = traci_mod.lane.getEdgeID(dep_lane)
                rear_edge= traci_mod.lane.getEdgeID(rear_lane)
                if dep_edge == rear_edge:
                    dep_pos = traci_mod.vehicle.getLanePosition(departing)
                    rear_pos= traci_mod.vehicle.getLanePosition(rear)
                    gap = dep_pos - rear_pos  # rear가 뒤에 있으면 양수여야 정상
                    if gap < 12.0:  # 너무 붙었으면 강제 더 줄이기
                        v_dep2 = traci_mod.vehicle.getSpeed(departing)
                        traci_mod.vehicle.setSpeed(rear, max(2.0, v_dep2 - (LEAVE_MARGIN + 2.0)))
            except Exception:
                pass
    except Exception:
        pass

# ===== 체인 유틸 =====
def _order_chain(pairs):
    pairs = list(pairs)
    if not pairs:
        return []
    followers = {f for f, _ in pairs}
    leaders   = {l for _, l in pairs}
    roots = list(leaders - followers)
    start = roots[0] if roots else pairs[0][1]
    order = [start]
    while True:
        nxt = next((f for f, l in pairs if l == order[-1] and f not in order), None)
        if not nxt:
            break
        order.append(nxt)
    return order

def _neighbors(chain, vid):
    if vid not in chain:
        return (None, None)
    i = chain.index(vid)
    front = chain[i-1] if i-1 >= 0 else None
    rear  = chain[i+1] if i+1 < len(chain) else None
    return front, rear

# ===== 상태/거리 유틸 =====
def _has_started(traci_mod, vid):
    try:
        return (vid in traci_mod.vehicle.getIDList()) and (not traci_mod.vehicle.isStopped(vid))
    except Exception:
        return False

def _euclid_gap(traci_mod, back, front):
    """리더 매칭이 안 될 때를 위한 기하학적 거리 fallback."""
    try:
        if (back not in traci_mod.vehicle.getIDList()) or (front not in traci_mod.vehicle.getIDList()):
            return None
        x1, y1 = traci_mod.vehicle.getPosition(back)
        x2, y2 = traci_mod.vehicle.getPosition(front)
        d = math.hypot(x2 - x1, y2 - y1)
        return d
    except Exception:
        return None

def _gap_between(traci_mod, back, front, lookahead=2000.0):
    """우선 getLeader로 시도, 실패하면 유클리드 거리로 대체."""
    try:
        if (back not in traci_mod.vehicle.getIDList()) or (front not in traci_mod.vehicle.getIDList()):
            return None
        info = traci_mod.vehicle.getLeader(back, lookahead)
        if info and info[0] == front:
            return max(0.0, info[1])
        return _euclid_gap(traci_mod, back, front)
    except Exception:
        return None

# ===== 도로상 순서 기반 front 재선택 =====
def _pick_best_front_for_merge(traci_mod, me, chain, initial_front):
    """
    '가까운 차량' 대신 '내 차보다 도로상 앞에 있는 플래투닝 차량'을 front로 선택.
    1) getLeader(me)로 바로 앞 차량이 체인 멤버면 그걸 front로.
    2) 같은 엣지면 lanePosition 비교로 '나보다 앞'에 있는 체인 멤버 중 가장 가까운 차량을 front로.
    3) 그래도 없으면 initial_front 유지.
    """
    try:
        info = traci_mod.vehicle.getLeader(me, 2000.0)
        if info and info[0] in chain:
            return info[0]
    except Exception:
        pass

    try:
        me_lane = traci_mod.vehicle.getLaneID(me)
        me_edge = traci_mod.lane.getEdgeID(me_lane)
        me_pos  = traci_mod.vehicle.getLanePosition(me)

        candidates = []
        for v in chain:
            if v == me:
                continue
            v_lane = traci_mod.vehicle.getLaneID(v)
            v_edge = traci_mod.lane.getEdgeID(v_lane)
            if v_edge != me_edge:
                continue
            v_pos = traci_mod.vehicle.getLanePosition(v)
            if v_pos > me_pos + 0.5:
                candidates.append((v, v_pos - me_pos))

        if candidates:
            candidates.sort(key=lambda x: x[1])
            return candidates[0][0]
    except Exception:
        pass

    return initial_front

# ===== 단일 뷰어 창 =====
class VehicleViewer(tk.Toplevel):
    """
    드롭다운으로 차량을 선택해 보는 단일 창 뷰어.
    레이아웃: 좌(참여/나가기/브레이크) | 중(이웃 네모 + gap) | 우(체인 리스트)
    """
    # SUMO 차량 색과 동일하게 매핑
    VEH_COLORS = {
        "Veh0": "#ff3b30",  # 빨강
        "Veh1": "#ffaa34",  # 주황
        "Veh2": "#ffee00",  # 노랑
        "Veh3": "#23d750",  # 초록
    }

    @staticmethod
    def _get_destination_str(traci_mod, vid: str) -> str:
        """
        차량 vid의 '목적지'를 보기 좋게 문자열로 반환.
        우선순위:
        1) 경로상 마지막 stop의 parkingArea/busStop/stopID 등
        2) route의 마지막 edge
        3) 알 수 없음
        """
        try:
            if vid not in traci_mod.vehicle.getIDList():
                return "—"

            # 1) 정류/주차장 등 stop 정보
            try:
                stops = traci_mod.vehicle.getStops(vid)  # list of dict-like objects
            except Exception:
                stops = []

            if stops:
                last = stops[-1]
                # parkingArea 우선 표시
                pa = last.get("parkingArea", "") if isinstance(last, dict) else getattr(last, "parkingArea", "")
                if pa:
                    return f"주차장 {pa}"
                # busStop 또는 generic stop id
                bs = last.get("busStop", "") if isinstance(last, dict) else getattr(last, "busStop", "")
                if bs:
                    return f"정류장 {bs}"
                sid = last.get("id", "") if isinstance(last, dict) else getattr(last, "id", "")
                if sid:
                    return f"정차지 {sid}"

            # 2) route 마지막 엣지
            try:
                rte = traci_mod.vehicle.getRoute(vid)
                if rte:
                    return f"엣지 {rte[-1]}"
            except Exception:
                pass

            return "—"
        except Exception:
            return "—"

    #보이기/숨기기 헬퍼 추가
    def _show(self, w):
        try:
            if not w.winfo_ismapped():
                w.pack(anchor="w", pady=(6,0) if w is self.lbl_dest_leader else (2,8))
        except Exception:
            pass

    def _hide(self, w):
        try:
            if w.winfo_ismapped():
                w.pack_forget()
        except Exception:
            pass

    def __init__(self, parent, traci_mod, initial_candidates):
        super().__init__(parent)

        # ★ 창 생성 직후 위치 지정
        self.geometry("+100+400")

        self.title("Truck Platooning – Vehicle Control Panel")
        self.traci = traci_mod

        # --- 상단 바: 차량 선택 ---
        top = ttk.Frame(self); top.pack(fill="x", padx=10, pady=6)
        ttk.Label(top, text="내 차량:", font=("Arial", 11, "bold")).pack(side="left")

        self.candidates = list(initial_candidates) if initial_candidates else ["Veh0", "Veh1", "Veh2", "Veh3"]
        self.selected = tk.StringVar(value=self.candidates[0])
        self.combo = ttk.Combobox(top, textvariable=self.selected, values=self.candidates,
                                  width=10, state="readonly")
        self.combo.pack(side="left", padx=8)

        # --- 상태 라벨 --- 
        self.status_var = tk.StringVar(value="")
        self.status_lbl = ttk.Label(top, textvariable=self.status_var)
        self.status_lbl.pack(side="left", padx=(20, 0))

        # --- 본문 3열 ---
        body = ttk.Frame(self); body.pack(padx=8, pady=8, fill="both", expand=True)
        self.left  = ttk.Frame(body); self.left.grid(row=0, column=0, sticky="nsw", padx=(0,10))
        self.mid   = ttk.Frame(body); self.mid.grid(row=0, column=1, sticky="nsew", padx=(0,10))
        self.right = ttk.Frame(body); self.right.grid(row=0, column=2, sticky="nse")

        body.columnconfigure(1, weight=1)
        body.rowconfigure(0, weight=1)

        # 좌측: 버튼들
        self.btn_join  = tk.Button(self.left, text="참여하기", width=12, command=self._on_join, state="disabled")
        self.btn_leave = tk.Button(self.left, text="나가기",   width=12, command=self._on_leave)
        self.btn_start = tk.Button(self.left, text="출발", width=12, command=self._on_start)
        self.btn_brake = tk.Button(self.left, text="브레이크", width=12)
        self.btn_join.pack(anchor="w", pady=2)
        self.btn_leave.pack(anchor="w", pady=2)
        self.btn_start.pack(anchor="w", pady=2)
        self.btn_brake.pack(anchor="w", pady=(8,2))

        # 목적지 라벨  ▶ 위젯 핸들 보관
        self.dest_leader_var = tk.StringVar(value="리더 목적지: —")
        self.dest_me_var     = tk.StringVar(value="내 목적지: —")

        # 라벨 객체 생성 (핸들 만들기)
        self.lbl_dest_leader = ttk.Label(self.left, textvariable=self.dest_leader_var)
        self.lbl_dest_me     = ttk.Label(self.left, textvariable=self.dest_me_var)

        # 기본은 ‘내 목적지’만 표시, 리더 목적지는 숨김 시작
        self.lbl_dest_me.pack(anchor="w", pady=(6,8))
        # self.lbl_dest_leader 는 pack 하지 않음 (미참여 기본)

        # 브레이크 컨트롤러(선택 바뀌면 타겟 교체)
        self.ctrl = BrakeController(traci_mod=self.traci)
        self.ctrl.set_leader(self.selected.get())
        self.btn_brake.bind("<ButtonPress-1>",  self.ctrl.on_brake_press)
        self.btn_brake.bind("<ButtonRelease-1>", self.ctrl.on_brake_release)

        # 가운데: 이웃 뷰 (캔버스)
        self.canvas = tk.Canvas(self.mid, width=350, height=420, bg="white",
                                highlightthickness=1, relief="solid")
        self.canvas.pack(fill="both", expand=True)

        # 오른쪽: 컨텍스트 패널 (체인 or 참여 후보)
        self.right_title_var = tk.StringVar(value="플래투닝 차량")
        ttk.Label(
            self.right,
            textvariable=self.right_title_var,
            font=("Arial", 11, "bold")
        ).pack(anchor="w", padx=0, pady=(0,4))

        # 공용 리스트박스 (체인 목록/참여 후보 목록 공용)
        self.listbox = tk.Listbox(self.right, width=30, height=18)
        self.listbox.pack(fill="both", expand=True, pady=(0,0))

        # 이벤트
        self.combo.bind("<<ComboboxSelected>>", self._on_select)

        # 주기 갱신 루프 시작
        self._tick()
        
    def _refresh_candidates(self):
        """
        콤보박스 후보를 '플래투닝 트럭(Veh*)'으로만 유지.
        bg_random.rou.xml에서 나오는 일반 차량은 무시.
        """
        try:
            current_ids = list(self.traci.vehicle.getIDList())
        except Exception:
            current_ids = []

        # 플래투닝 트럭만 후보에 추가
        for vid in current_ids:
            if is_platoon_truck(vid) and vid not in self.candidates:
                self.candidates.append(vid)

        # 중복 제거 + 순서 유지
        self.candidates = list(dict.fromkeys(self.candidates))

        # 현재 선택된 값이 후보에 없으면 첫 번째로 보정
        if self.selected.get() not in self.candidates and self.candidates:
            self.selected.set(self.candidates[0])

        # 콤보박스 갱신
        self.combo["values"] = self.candidates

    # ---- 버튼 토글 ----
    def _refresh_buttons(self):
        me = self.selected.get()
        chain = _order_chain(cfg.FOLLOW_PAIRS)
        in_platoon = me in chain

        # 참여 버튼: 미참여 + 300m 이내 + 체인 존재
        if not in_platoon:
            d = cfg.VEHICLE_DISTANCES.get(me, float('inf'))
            self.btn_join.configure(state=("normal" if d <= PLATOON_JOIN_DISTANCE and len(chain)>0 else "disabled"))
        else:
            self.btn_join.configure(state="disabled")

        # 출발 / 나가기
        self.btn_start.configure(state=("normal" if (me not in chain and me not in cfg.STARTED) else "disabled"))
        self.btn_leave.configure(state=("normal" if in_platoon else "disabled"))

    # ---- 콜백들 ----
    def _on_select(self, _evt=None):
        self.ctrl.set_leader(self.selected.get())
        self._refresh_now()  # 즉시 반영
        self._refresh_buttons()

    def _on_join(self):
        chain = _order_chain(cfg.FOLLOW_PAIRS)
        me = self.selected.get()

        # 체인이 없으면 리더 시작 로직은 기존 그대로
        if not chain:
            response = messagebox.askyesno(
                f"플래투닝 리더 시작 - {me}",
                f"차량 {me}가 플래투닝의 리더가 되시겠습니까?",
                parent=self
            )
            if response:
                cfg.FOLLOW_PAIRS = []
                cfg.FOLLOWERS = []
                print(f"[플래투닝 리더] {me}가 플래투닝의 리더가 되었습니다.")
                self._refresh_now(); self._refresh_buttons()
            return

        # 이미 체인에 있으면 무시
        if me in chain:
            return

        # ▶ 300m 이내 ‘참여 대상’들 중 가장 가까운 front 선택 (fallback 포함)
        cand_map = getattr(cfg, "NEARBY_PLATOON", {})
        nearby = cand_map.get(me, [])  # [(platoon_vid, dist), ...] 오름차순
        if not nearby:
            nearby = _nearby_fallback(self.traci, me, chain, PLATOON_JOIN_DISTANCE)

        if not nearby:
            messagebox.showwarning("참여 불가", "300m 내 플래투닝 차량이 없습니다.", parent=self)
            return

        # 체인 멤버만 유지
        nearby = [(v, d) for (v, d) in nearby if v in chain]
        if not nearby:
            messagebox.showwarning("참여 불가", "근처 플래투닝 차량이 없습니다.", parent=self)
            return

        front, distance = nearby[0]  # (유클리드 기준) 가장 가까운 차량
        # ★ 도로상 순서 보정: 내가 이미 그 차량보다 '앞'이면, 내 앞에 있는 체인 차량을 front로 재선택
        front = _pick_best_front_for_merge(self.traci, me, chain, front)

        # 사용자 확인
        response = messagebox.askyesno(
            f"플래투닝 참여 확인 - {front}",
            f"{front} 차량 뒤에 참여하시겠습니까?\n"
            f"(현재 거리: {distance:.1f} m)",
            parent=self
        )
        if not response:
            print(f"[플래투닝 거부] {me} 참여 취소")
            return

        # ▶ 체인 '중간 삽입' 재배선: (rear, front) → (rear, me), 그리고 (me, front) 추가
        pairs = list(cfg.FOLLOW_PAIRS)

        # 1) front 바로 뒤(rear)가 있는지 찾기
        rear = next((f for (f, l) in pairs if l == front), None)

        # 2) me 관련 잔존 페어 제거(안전)
        pairs = [(f, l) for (f, l) in pairs if f != me and l != me]

        # 3) (me, front) 추가
        pairs.append((me, front))

        # 4) rear가 있었다면 rear의 리더를 me로 변경
        if rear:
            pairs = [(f, (me if (f == rear and l == front) else l)) for (f, l) in pairs]

        # 반영
        cfg.FOLLOW_PAIRS = pairs
        cfg.FOLLOWERS = [f for (f, _) in pairs]

        # ▶ CACC 전환
        switch_to_cacc(me)

        # ▶ 재합류 쿨다운 시작: n초 동안 추월 금지 + 속도 상한
        try:
            sim_t = self.traci.simulation.getTime()
            JOIN_COOLDOWN[me] = sim_t + COOLDOWN_SEC
        except Exception:
            pass

        # ▶ 자연스러운 재합류: front와 같은 엣지면 즉시, 아니면 pending 등록
        try:
            lane_front_idx = self.traci.vehicle.getLaneIndex(front)
            my_lane_id     = self.traci.vehicle.getLaneID(me)
            front_lane_id  = self.traci.vehicle.getLaneID(front)
            my_edge        = self.traci.lane.getEdgeID(my_lane_id)
            front_edge     = self.traci.lane.getEdgeID(front_lane_id)

            # 합류 안전성↑: 앞차보다 약간 느리게(뒤로 물러나며 갭 형성)
            vF = self.traci.vehicle.getSpeed(front)
            self.traci.vehicle.setSpeed(me, max(5.0, vF - 1.5))

            if my_edge == front_edge:
                # 같은 엣지면 즉시 시도
                nlanes = self.traci.edge.getLaneNumber(my_edge)
                tgt_i  = max(0, min(int(lane_front_idx), int(nlanes) - 1))
                _smooth_change_lane(self.traci, me, tgt_i, hold_sec=4.0)
            else:
                # 아직 다른 엣지면, 같은 엣지에 들어올 때까지 보류
                PENDING_MERGE[me] = (front, int(lane_front_idx))
        except Exception:
            pass

        self._refresh_now()
        self._refresh_buttons()
        print(f"[플래투닝 참여] {me}가 {front} 뒤에 합류(표시 거리 {distance:.1f} m)")

    def _on_leave(self):
        #플래투닝 떠나기 버튼
        me = self.selected.get()

        pairs = list(cfg.FOLLOW_PAIRS)
        chain = _order_chain(pairs)

        if me not in chain:
            self._refresh_now()
            self._refresh_buttons()
            return

        # 확인 대화상자 표시 (차량 번호 포함)
        last_vehicle = chain[-1] if chain else "없음"
        response = messagebox.askyesno(
            f"플래투닝 나가기 확인 - {me} (맨 뒷 차량: {last_vehicle})",
            f"플래투닝에서 나가시겠습니까?\n\n"
            f"플래투닝 맨 뒷 차량: {last_vehicle}",
            parent=self
        )
        if not response:
            return

        # 현재 위치(앞/뒤) 계산
        i = chain.index(me)
        front = chain[i-1] if i-1 >= 0 else None
        rear  = chain[i+1] if i+1 < len(chain) else None

        # === 이탈 보호 시작: rear가 있으면 rear에 LEAVE_GUARD 걸기 ===
        try:
            if rear and (rear in self.traci.vehicle.getIDList()):
                sim_t = self.traci.simulation.getTime()
                LEAVE_GUARD[rear] = (sim_t + LEAVE_GUARD_SEC, me)
                # 즉시 살짝 속도 낮춰 공간 만들기 + 차선 변경 금지
                try:
                    v_now = self.traci.vehicle.getSpeed(rear)
                    self.traci.vehicle.setSpeed(rear, max(3.0, v_now - 2.0))
                    self.traci.vehicle.setLaneChangeMode(rear, 0)
                except Exception:
                    pass
        except Exception:
            pass

        # me 관련 페어 제거 + 뒤차를 앞차에 재연결(있다면)
        pairs = [(f, l) for (f, l) in pairs if f != me and l != me]
        if front and rear:
            pairs.append((rear, front))

        #config에 차량 업데이트
        cfg.FOLLOW_PAIRS = pairs
        cfg.FOLLOWERS = [f for f, _ in pairs]

        #체인 업데이트
        new_chain = _order_chain(cfg.FOLLOW_PAIRS)

        # me가 체인에서 빠졌다면 리더로 자동 전환 (체인 남아있을 때)
        if me not in new_chain:
            if new_chain:
                self.selected.set(new_chain[0])
                self.ctrl.set_leader(new_chain[0])
            else:
                # 체인 비었으면 오른쪽/가운데 비우기
                self.listbox.delete(0, tk.END)
                self.canvas.delete("all")

        # 주행 모드 기본으로
        switch_to_basic(me)

        # 자연스러운 이탈: 인접 차선으로 이동
        try:
            cur_lane = self.traci.vehicle.getLaneID(me)
            cur_idx  = self.traci.vehicle.getLaneIndex(me)
            tgt_idx  = _adjacent_lane_or_self(self.traci, cur_lane, cur_idx, prefer_right=True)
            _smooth_change_lane(self.traci, me, tgt_idx, hold_sec=3.0)
        except Exception:
            pass

        self._refresh_now()  # 즉시 반영
        self._refresh_buttons()

    def _on_start(self):
        """주차장에 있는 차량을 플래투닝 없이 출발시키기"""
        me = self.selected.get()
        self.btn_start.configure(state="disabled")

        try:
            if me in self.traci.vehicle.getIDList():
                # 주차장에 있는지 확인
                lane_id = self.traci.vehicle.getLaneID(me)
                road_id = self.traci.vehicle.getRoadID(me)
                is_in_parking = (not lane_id or lane_id.startswith("pa_") or 
                                 road_id.startswith("pa_") or
                                 self.traci.vehicle.isStopped(me))
                
                if is_in_parking:
                    # 주차장에 있으면 출발시키기 (플래투닝에 포함하지 않음)
                    self.traci.vehicle.resume(me)
                    print(f"[출발] {me} 플래투닝 없이 출발")
                    # 플래투닝에서 제거 (이미 있다면)
                    pairs = list(cfg.FOLLOW_PAIRS)
                    pairs = [(f, l) for (f, l) in pairs if f != me and l != me]
                    cfg.FOLLOW_PAIRS = pairs
                    cfg.FOLLOWERS = [f for f, _ in pairs]

                    # 출발 기록
                    if hasattr(cfg, "STARTED"):
                        try:
                            cfg.STARTED.add(me)
                        except Exception:
                            pass
        except Exception:
            pass
        self._refresh_now()
        self._refresh_buttons()

    # ---- 그리기 ----
    def _draw_scene(self, me, front, rear, gap_f, gap_r):
        self.canvas.delete("all")
        W = int(self.canvas.winfo_width() or 520)
        H = int(self.canvas.winfo_height() or 360)
        cx = W // 2
        w, h = 160, 50 #40, 100
        y_front, y_me, y_rear = 80, H // 2, H - 80

        def box(xc, yc, label, fill):
            self.canvas.create_rectangle(xc - w // 2, yc - h // 2, xc + w // 2, yc + h // 2,
                                         fill=fill, outline="black")
            self.canvas.create_text(xc, yc, text=label, font=("Arial", 12, "bold"))

        # 플래투닝이 아니면 빈 칸 유지
        if not (front or rear):
            return

        # 내 차
        #box(cx, y_me, me, "#efefef")
        my_color = self.VEH_COLORS.get(me, "#efefef")   # 기본 회색
        box(cx, y_me, me, my_color)

        # 앞차
        if front:
            #box(cx, y_front, front, "#d9efff")
            front_color = self.VEH_COLORS.get(front, "#d9efff")  # 기본 파랑 톤
            box(cx, y_front, front, front_color)
            if gap_f is not None:
                self.canvas.create_text(cx, (y_front + y_me) // 2,
                                        text=f"gap: {gap_f:.1f} m", font=("Arial", 11))
        # 뒷차
        if rear:
            #box(cx, y_rear, rear, "#ffe3c2")
            rear_color = self.VEH_COLORS.get(rear, "#ffe3c2")    # 기본 주황 톤
            box(cx, y_rear, rear, rear_color)
            if gap_r is not None:
                self.canvas.create_text(cx, (y_me + y_rear) // 2,
                                        text=f"gap: {gap_r:.1f} m", font=("Arial", 11))

    # ---- 즉시/주기 갱신 ----
    def _refresh_now(self):
        try:
            me = self.selected.get()
            chain = _order_chain(cfg.FOLLOW_PAIRS)

            # 오른쪽 리스트 갱신 - 상태에 따라 전환(참여 중=체인, 미참여=300m 후보)
            self.listbox.delete(0, tk.END)
            highlight_idx = None

            if me in chain:
                # ▶ 플래투닝 참여 중: 체인 목록 + 내 차량 표시
                for i, v in enumerate(chain):
                    mark = ""
                    if i == 0:
                        mark = " ⚑"     # 리더
                    self.listbox.insert(tk.END, f"{i}. {v}{mark}")
                    if v == me:
                        highlight_idx = i
                try:
                    for i in range(len(chain)):
                        self.listbox.itemconfig(i, {'bg': 'white'})
                    if highlight_idx is not None:
                        self.listbox.itemconfig(highlight_idx, {'bg': "#aeaeae"})
                except Exception:
                    pass

            else:
                # ▶ 플래투닝 미참여: 300m 안의 '참여 가능 대상(플래투닝 차량)' 리스트
                cand_map = getattr(cfg, "NEARBY_PLATOON", {})
                cand = cand_map.get(me, [])

                if not cand:
                    self.listbox.insert(tk.END, "300m 내 진입 시 플래투닝 참여 가능")
                    # tail 거리 표시 (app.py에서 cfg.VEHICLE_DISTANCES 갱신됨)
                    d = getattr(cfg, "VEHICLE_DISTANCES", {}).get(me, float("inf"))
                    if d != float("inf"):
                        self.listbox.insert(tk.END, f"→ 참여 차량까지 거리: {d:.1f} m")
                    else:
                        self.listbox.insert(tk.END, "→ 참여 차량까지 거리: —")
                    
            # 상태 라벨 갱신
            if me in chain:
                self.status_var.set("상태: 플래투닝 참여중")
                self.status_lbl.configure(foreground="#2e7d32")  # 초록색
            else:
                self.status_var.set("상태: 미참여")
                self.status_lbl.configure(foreground="#6b7280")

            # 이웃/갭 계산 및 그리기
            if me and (me in chain):
                front, rear = _neighbors(chain, me)

                # 출발(주행) 상태일 때만 gap 표시
                gap_f = None
                gap_r = None
                if front and _has_started(self.traci, me) and _has_started(self.traci, front):
                    gap_f = _gap_between(self.traci, me, front)
                if rear and _has_started(self.traci, me) and _has_started(self.traci, rear):
                    gap_r = _gap_between(self.traci, rear, me)

                self._draw_scene(me, front, rear, gap_f, gap_r)
            else:
                #플래투닝 미참여: 가운데에 내 차량 박스만
                self.canvas.delete("all")
                W = int(self.canvas.winfo_width() or 520)
                H = int(self.canvas.winfo_height() or 360)
                cx, cy = W // 2, H // 2
                w, h = 160, 50 # 40, 100
                try:
                    my_color = self.VEH_COLORS.get(me, "#f5f5f5")
                except Exception:
                    my_color = "#f5f5f5"
                self.canvas.create_rectangle(cx - w // 2, cy - h // 2,
                                             cx + w // 2, cy + h // 2,
                                             fill=my_color, outline="black")
                self.canvas.create_text(cx, cy, text=me, font=("Arial", 12, "bold"))

            # 브레이크 자동 복귀
            self.ctrl.update()

            # === 목적지 라벨 업데이트 ===
            try:
                # 리더: 체인이 있으면 chain[0], 없으면 None
                leader_id = chain[0] if chain else None
                leader_dest = self._get_destination_str(self.traci, leader_id) if leader_id else "—"
                me_dest     = self._get_destination_str(self.traci, me) if me else "—"

                self.dest_leader_var.set(f"리더 목적지: {leader_dest}")
                self.dest_me_var.set(f"내 목적지: {me_dest}")
                 # 참여 여부로 리더 목적지 라벨 보이기/숨기기
                if me in chain:
                    self._show(self.lbl_dest_leader)
                else:
                    self._hide(self.lbl_dest_leader)
            except Exception:
                # UI가 죽지 않게 안전하게 무시
                pass


            # ▶ 합류 보류/LCM 원복/쿨다운/이탈보호 스케줄링 틱
            _tick_pending_merge(self.traci)
            _tick_lane_mode_restore(self.traci)
            _tick_join_cooldown(self.traci)
            _tick_leave_guard(self.traci)

        except Exception:
            pass

    def _tick(self):
        self._refresh_candidates()
        self._refresh_now()
        self._refresh_buttons()
        self.after(500, self._tick)  # 0.5초마다 갱신

def open_vehicle_viewer(parent, traci_mod, candidates):
    """app.py에서 호출: 단일 뷰어 창 열기"""
    return VehicleViewer(parent, traci_mod, candidates)
