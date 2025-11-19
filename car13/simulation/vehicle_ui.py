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
# PENDING_MERGE는 MERGE_COORDINATOR가 대체하지만, 호환성을 위해 남겨둠
PENDING_MERGE = {}       # vid -> (front_id, target_lane_idx)

# --- [신규] 합류 코디네이터 (Overlap 해결 핵심) ---
# key: merger_id, value: { 'front': str, 'rear': str, 'state': str }
MERGE_COORDINATOR = {}

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
    """cfg.NEARBY_PLATOON이 비어있는 경우 즉석에서 후보 산출."""
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
    """
    try:
        traci_mod.vehicle.setLaneChangeMode(vid, 1621)  # 잠깐 허용
        traci_mod.vehicle.changeLane(vid, int(target_lane_index), float(hold_sec))
        sim_t = traci_mod.simulation.getTime()
        LANE_MODE_RESTORE[vid] = sim_t + float(hold_sec)
    except Exception:
        pass

def _tick_lane_mode_restore(traci_mod):
    """laneChangeMode 예약 복구"""
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
    """기존 단순 합류 로직 (MERGE_COORDINATOR가 주로 처리하므로 보조용)"""
    try:
        for vid, (front, tgt_idx) in list(PENDING_MERGE.items()):
            # MERGE_COORDINATOR에 있으면 여기서 처리하지 않음
            if vid in MERGE_COORDINATOR:
                PENDING_MERGE.pop(vid, None)
                continue
                
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
                    PENDING_MERGE.pop(vid, None)
            except Exception:
                PENDING_MERGE.pop(vid, None)
                continue
    except Exception:
        pass

# ===== [핵심 수정] 합류 코디네이터 함수 =====
def _tick_merge_coordinator(traci_mod):
    """
    합류 시도 차량(Me)과 타겟 차선의 뒷차(Rear) 간의 상호작용을 제어합니다.
    - Rear가 Me와 겹치거나 가까우면, Rear를 강제로 급감속시킵니다 (Active Yield).
    - 공간이 확보되면 Me를 차선 변경시킵니다.
    """
    try:
        if not MERGE_COORDINATOR:
            return

        # config.py에 YIELDING_FOR_MERGE set이 있다고 가정 (없으면 에러 없이 처리)
        yielding_set = getattr(cfg, "YIELDING_FOR_MERGE", set())
        if not hasattr(cfg, "YIELDING_FOR_MERGE"):
            # 만약 config에 없으면 임시로 속성 추가 (런타임 패치)
            cfg.YIELDING_FOR_MERGE = set()
            yielding_set = cfg.YIELDING_FOR_MERGE

        active_mergers = list(MERGE_COORDINATOR.keys())
        for me in active_mergers:
            data = MERGE_COORDINATOR[me]
            front = data['front']
            rear = data['rear'] # None일 수 있음 (맨 뒤 합류)

            # 차량 소멸 체크
            if me not in traci_mod.vehicle.getIDList():
                MERGE_COORDINATOR.pop(me, None)
                continue
            
            # 1. 앞차(Front) 기준 속도 동기화
            #    Me는 Front보다 살짝 느리게 가서 자연스럽게 뒤로 붙게 함
            if front in traci_mod.vehicle.getIDList():
                v_front = traci_mod.vehicle.getSpeed(front)
                # ex-code 방식: 앞차보다 1.0~1.5m/s 느리게 유지하여 갭 형성
                target_v_me = max(1.0, v_front - 1.0)
                traci_mod.vehicle.setSpeed(me, target_v_me)
            else:
                # 앞차가 사라지면 합류 취소
                MERGE_COORDINATOR.pop(me, None)
                continue

            # 2. 뒷차(Rear) 제어 및 합류 가능 여부 판단
            safe_to_merge = True
            
            if rear and rear in traci_mod.vehicle.getIDList():
                pos_me = traci_mod.vehicle.getPosition(me)
                pos_rear = traci_mod.vehicle.getPosition(rear)
                
                # 거리 계산 (단순 유클리드)
                dist = math.sqrt((pos_me[0]-pos_rear[0])**2 + (pos_me[1]-pos_rear[1])**2)
                
                # [판단 기준] 
                # 거리가 25m 이내면 "겹쳐있거나 위험하다"고 판단 -> 강제 양보 필요
                SAFE_GAP = 25.0
                
                if dist < SAFE_GAP:
                    safe_to_merge = False
                    
                    # === Active Yield: 뒷차 강제 감속 ===
                    yielding_set.add(rear)
                    
                    v_me = traci_mod.vehicle.getSpeed(me)
                    v_rear = traci_mod.vehicle.getSpeed(rear)
                    
                    # 뒷차를 내 속도보다 5m/s 느리게 만듦 (급브레이크 효과)
                    # 단, 0 이하로는 안 떨어지게
                    yield_speed = max(0.0, v_me - 5.0)
                    
                    # 너무 급격한 변화 완화 (현재 속도에서 점진적 하강)
                    # 하지만 공간을 만들려면 과감해야 함
                    final_yield = min(v_rear - 0.5, yield_speed)
                    final_yield = max(0.0, final_yield)
                    
                    traci_mod.vehicle.setSpeed(rear, final_yield)
                    # print(f"[Coordinator] {rear} yielding to {me} (Gap: {dist:.1f}m)")
                
                else:
                    # 거리가 충분히 벌어짐 -> 뒷차 제어 해제
                    if rear in yielding_set:
                        yielding_set.discard(rear)
                        traci_mod.vehicle.setSpeed(rear, -1) # 제어권 반환

            # 3. 차선 변경 실행 (안전하다고 판단되면)
            if safe_to_merge:
                try:
                    # 같은 엣지에 있는지 확인
                    lane_me = traci_mod.vehicle.getLaneID(me)
                    lane_front = traci_mod.vehicle.getLaneID(front)
                    edge_me = traci_mod.lane.getEdgeID(lane_me)
                    edge_front = traci_mod.lane.getEdgeID(lane_front)
                    
                    if edge_me == edge_front:
                        tgt_idx = traci_mod.vehicle.getLaneIndex(front)
                        cur_idx = traci_mod.vehicle.getLaneIndex(me)
                        
                        if cur_idx != tgt_idx:
                            # ex-code 방식: 과감하게 차선 변경 시도 (hold 5초)
                            _smooth_change_lane(traci_mod, me, tgt_idx, hold_sec=5.0)
                            print(f"[Merge Execute] {me} merging behind {front}")
                        
                        # 성공적으로 명령 내렸으므로 코디네이터 졸업
                        MERGE_COORDINATOR.pop(me, None)
                        
                        # 뒷차 완전 해방
                        if rear and rear in yielding_set:
                            yielding_set.discard(rear)
                            traci_mod.vehicle.setSpeed(rear, -1)
                            
                        # 쿨다운 시작
                        sim_t = traci_mod.simulation.getTime()
                        JOIN_COOLDOWN[me] = sim_t + COOLDOWN_SEC

                except Exception:
                    pass

    except Exception:
        pass

def _tick_join_cooldown(traci_mod):
    """재합류 직후 일정 시간 동안 추월 금지 + 속도 상한 강제."""
    try:
        sim_t = traci_mod.simulation.getTime()
        pairs = list(getattr(cfg, "FOLLOW_PAIRS", []))
        if not pairs:
            for vid, until_t in list(JOIN_COOLDOWN.items()):
                if sim_t >= until_t:
                    JOIN_COOLDOWN.pop(vid, None)
            return

        f2l = {f: l for (f, l) in pairs}

        for vid, until_t in list(JOIN_COOLDOWN.items()):
            if (vid not in traci_mod.vehicle.getIDList()):
                JOIN_COOLDOWN.pop(vid, None)
                continue

            if sim_t >= until_t:
                JOIN_COOLDOWN.pop(vid, None)
                try:
                    if traci_mod.vehicle.getTypeID(vid) == "truckCACC":
                        traci_mod.vehicle.setLaneChangeMode(vid, 0)
                except Exception:
                    pass
                continue

            front = f2l.get(vid)
            if not front or (front not in traci_mod.vehicle.getIDList()):
                continue

            try:
                traci_mod.vehicle.setLaneChangeMode(vid, 0)
            except Exception:
                pass

            # 쿨다운 중 속도 제한
            try:
                vF = traci_mod.vehicle.getSpeed(front)
                vCap = max(4.0, vF - COOLDOWN_MARGIN)
                vNow = traci_mod.vehicle.getSpeed(vid)
                if vNow > vCap:
                    traci_mod.vehicle.setSpeed(vid, vCap)
            except Exception:
                pass
    except Exception:
        pass

def _tick_leave_guard(traci_mod):
    """앞차가 이탈하는 동안 뒤차 감속."""
    try:
        sim_t = traci_mod.simulation.getTime()
        for rear, (until_t, departing) in list(LEAVE_GUARD.items()):
            if (rear not in traci_mod.vehicle.getIDList()) or (departing not in traci_mod.vehicle.getIDList()):
                LEAVE_GUARD.pop(rear, None)
                continue

            if sim_t >= until_t:
                LEAVE_GUARD.pop(rear, None)
                try:
                    if traci_mod.vehicle.getTypeID(rear) == "truckCACC":
                        traci_mod.vehicle.setLaneChangeMode(rear, 0)
                except Exception:
                    pass
                continue

            try:
                traci_mod.vehicle.setLaneChangeMode(rear, 0)
            except Exception:
                pass

            try:
                v_dep = traci_mod.vehicle.getSpeed(departing)
            except Exception:
                v_dep = 6.0

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
    VEH_COLORS = {
        "Veh0": "#ff3b30", "Veh1": "#ffaa34", "Veh2": "#ffee00", "Veh3": "#23d750",
    }

    @staticmethod
    def _get_destination_str(traci_mod, vid: str) -> str:
        try:
            if vid not in traci_mod.vehicle.getIDList():
                return "—"
            stops = traci_mod.vehicle.getStops(vid)
            if stops:
                last = stops[-1]
                pa = last.get("parkingArea", "") if isinstance(last, dict) else getattr(last, "parkingArea", "")
                if pa: return f"주차장 {pa}"
                bs = last.get("busStop", "") if isinstance(last, dict) else getattr(last, "busStop", "")
                if bs: return f"정류장 {bs}"
                sid = last.get("id", "") if isinstance(last, dict) else getattr(last, "id", "")
                if sid: return f"정차지 {sid}"
            rte = traci_mod.vehicle.getRoute(vid)
            if rte: return f"엣지 {rte[-1]}"
            return "—"
        except Exception:
            return "—"

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
        self.geometry("+100+400")
        self.title("Truck Platooning – Vehicle Control Panel")
        self.traci = traci_mod

        top = ttk.Frame(self); top.pack(fill="x", padx=10, pady=6)
        ttk.Label(top, text="내 차량:", font=("Arial", 11, "bold")).pack(side="left")

        self.candidates = list(initial_candidates) if initial_candidates else ["Veh0", "Veh1", "Veh2", "Veh3"]
        self.selected = tk.StringVar(value=self.candidates[0])
        self.combo = ttk.Combobox(top, textvariable=self.selected, values=self.candidates, width=10, state="readonly")
        self.combo.pack(side="left", padx=8)

        self.status_var = tk.StringVar(value="")
        self.status_lbl = ttk.Label(top, textvariable=self.status_var)
        self.status_lbl.pack(side="left", padx=(20, 0))

        body = ttk.Frame(self); body.pack(padx=8, pady=8, fill="both", expand=True)
        self.left  = ttk.Frame(body); self.left.grid(row=0, column=0, sticky="nsw", padx=(0,10))
        self.mid   = ttk.Frame(body); self.mid.grid(row=0, column=1, sticky="nsew", padx=(0,10))
        self.right = ttk.Frame(body); self.right.grid(row=0, column=2, sticky="nse")
        body.columnconfigure(1, weight=1)
        body.rowconfigure(0, weight=1)

        self.btn_join  = tk.Button(self.left, text="참여하기", width=12, command=self._on_join, state="disabled")
        self.btn_leave = tk.Button(self.left, text="나가기",   width=12, command=self._on_leave)
        self.btn_start = tk.Button(self.left, text="출발", width=12, command=self._on_start)
        self.btn_brake = tk.Button(self.left, text="브레이크", width=12)
        self.btn_join.pack(anchor="w", pady=2)
        self.btn_leave.pack(anchor="w", pady=2)
        self.btn_start.pack(anchor="w", pady=2)
        self.btn_brake.pack(anchor="w", pady=(8,2))

        self.dest_leader_var = tk.StringVar(value="리더 목적지: —")
        self.dest_me_var     = tk.StringVar(value="내 목적지: —")
        self.lbl_dest_leader = ttk.Label(self.left, textvariable=self.dest_leader_var)
        self.lbl_dest_me     = ttk.Label(self.left, textvariable=self.dest_me_var)
        self.lbl_dest_me.pack(anchor="w", pady=(6,8))

        self.ctrl = BrakeController(traci_mod=self.traci)
        self.ctrl.set_leader(self.selected.get())
        self.btn_brake.bind("<ButtonPress-1>",  self.ctrl.on_brake_press)
        self.btn_brake.bind("<ButtonRelease-1>", self.ctrl.on_brake_release)

        self.canvas = tk.Canvas(self.mid, width=350, height=420, bg="white", highlightthickness=1, relief="solid")
        self.canvas.pack(fill="both", expand=True)

        self.right_title_var = tk.StringVar(value="플래투닝 차량")
        ttk.Label(self.right, textvariable=self.right_title_var, font=("Arial", 11, "bold")).pack(anchor="w", padx=0, pady=(0,4))
        self.listbox = tk.Listbox(self.right, width=30, height=18)
        self.listbox.pack(fill="both", expand=True, pady=(0,0))

        self.combo.bind("<<ComboboxSelected>>", self._on_select)
        self._tick()
        
    def _refresh_candidates(self):
        try:
            current_ids = list(self.traci.vehicle.getIDList())
        except Exception:
            current_ids = []
        for vid in current_ids:
            if is_platoon_truck(vid) and vid not in self.candidates:
                self.candidates.append(vid)
        self.candidates = list(dict.fromkeys(self.candidates))
        if self.selected.get() not in self.candidates and self.candidates:
            self.selected.set(self.candidates[0])
        self.combo["values"] = self.candidates

    def _refresh_buttons(self):
        me = self.selected.get()
        chain = _order_chain(cfg.FOLLOW_PAIRS)
        in_platoon = me in chain
        if not in_platoon:
            d = cfg.VEHICLE_DISTANCES.get(me, float('inf'))
            self.btn_join.configure(state=("normal" if d <= PLATOON_JOIN_DISTANCE and len(chain)>0 else "disabled"))
        else:
            self.btn_join.configure(state="disabled")
        self.btn_start.configure(state=("normal" if (me not in chain and me not in cfg.STARTED) else "disabled"))
        self.btn_leave.configure(state=("normal" if in_platoon else "disabled"))

    def _on_select(self, _evt=None):
        self.ctrl.set_leader(self.selected.get())
        self._refresh_now()
        self._refresh_buttons()

    def _on_join(self):
        chain = _order_chain(cfg.FOLLOW_PAIRS)
        me = self.selected.get()

        if not chain:
            response = messagebox.askyesno(f"플래투닝 리더 시작 - {me}", f"{me}가 리더가 되시겠습니까?", parent=self)
            if response:
                cfg.FOLLOW_PAIRS = []
                cfg.FOLLOWERS = []
                switch_to_cacc(me) # 리더도 CACC 타입 전환 권장
                print(f"[플래투닝 리더] {me}가 리더가 되었습니다.")
                self._refresh_now(); self._refresh_buttons()
            return

        if me in chain: return

        cand_map = getattr(cfg, "NEARBY_PLATOON", {})
        nearby = cand_map.get(me, [])
        if not nearby: nearby = _nearby_fallback(self.traci, me, chain, PLATOON_JOIN_DISTANCE)
        nearby = [(v, d) for (v, d) in nearby if v in chain]
        if not nearby:
            messagebox.showwarning("참여 불가", "300m 내 플래투닝 차량 없음.", parent=self)
            return

        front, distance = nearby[0]
        front = _pick_best_front_for_merge(self.traci, me, chain, front)

        response = messagebox.askyesno(f"참여 - {front}", f"{front} 뒤에 합류하시겠습니까?\n거리: {distance:.1f}m", parent=self)
        if not response: return

        pairs = list(cfg.FOLLOW_PAIRS)
        rear = next((f for (f, l) in pairs if l == front), None)
        pairs = [(f, l) for (f, l) in pairs if f != me and l != me]
        pairs.append((me, front))
        if rear:
            pairs = [(f, (me if (f == rear and l == front) else l)) for (f, l) in pairs]

        cfg.FOLLOW_PAIRS = pairs
        cfg.FOLLOWERS = [f for (f, _) in pairs]

        switch_to_cacc(me)

        # [수정] 합류 코디네이터에 등록 (PENDING_MERGE 대신 사용 권장)
        MERGE_COORDINATOR[me] = {
            'front': front,
            'rear': rear,  # rear가 None이면 맨 뒤 합류
            'state': 'aligning'
        }
        print(f"[Merge Start] {me} trying to merge behind {front}, coordinator active.")

        self._refresh_now()
        self._refresh_buttons()

    def _on_leave(self):
        me = self.selected.get()
        pairs = list(cfg.FOLLOW_PAIRS)
        chain = _order_chain(pairs)
        if me not in chain: return

        last_vehicle = chain[-1] if chain else "없음"
        response = messagebox.askyesno(f"나가기 - {me}", f"플래투닝에서 나가시겠습니까?\n맨 뒤: {last_vehicle}", parent=self)
        if not response: return

        i = chain.index(me)
        front = chain[i-1] if i-1 >= 0 else None
        rear  = chain[i+1] if i+1 < len(chain) else None

        try:
            if rear and (rear in self.traci.vehicle.getIDList()):
                sim_t = self.traci.simulation.getTime()
                LEAVE_GUARD[rear] = (sim_t + LEAVE_GUARD_SEC, me)
                try:
                    v_now = self.traci.vehicle.getSpeed(rear)
                    self.traci.vehicle.setSpeed(rear, max(3.0, v_now - 2.0))
                    self.traci.vehicle.setLaneChangeMode(rear, 0)
                except Exception: pass
        except Exception: pass

        pairs = [(f, l) for (f, l) in pairs if f != me and l != me]
        if front and rear: pairs.append((rear, front))
        cfg.FOLLOW_PAIRS = pairs
        cfg.FOLLOWERS = [f for f, _ in pairs]

        new_chain = _order_chain(cfg.FOLLOW_PAIRS)
        if me not in new_chain:
            if new_chain:
                self.selected.set(new_chain[0])
                self.ctrl.set_leader(new_chain[0])
            else:
                self.listbox.delete(0, tk.END)
                self.canvas.delete("all")

        switch_to_basic(me)
        
        # 합류 중이었다면 코디네이터에서 제거
        if me in MERGE_COORDINATOR:
            MERGE_COORDINATOR.pop(me, None)

        try:
            cur_lane = self.traci.vehicle.getLaneID(me)
            cur_idx  = self.traci.vehicle.getLaneIndex(me)
            tgt_idx  = _adjacent_lane_or_self(self.traci, cur_lane, cur_idx, prefer_right=True)
            _smooth_change_lane(self.traci, me, tgt_idx, hold_sec=3.0)
        except Exception: pass

        self._refresh_now()
        self._refresh_buttons()

    def _on_start(self):
        me = self.selected.get()
        self.btn_start.configure(state="disabled")
        try:
            if me in self.traci.vehicle.getIDList():
                lane_id = self.traci.vehicle.getLaneID(me)
                road_id = self.traci.vehicle.getRoadID(me)
                is_in_parking = (not lane_id or lane_id.startswith("pa_") or road_id.startswith("pa_") or self.traci.vehicle.isStopped(me))
                if is_in_parking:
                    self.traci.vehicle.resume(me)
                    print(f"[출발] {me} 출발")
                    pairs = list(cfg.FOLLOW_PAIRS)
                    pairs = [(f, l) for (f, l) in pairs if f != me and l != me]
                    cfg.FOLLOW_PAIRS = pairs
                    cfg.FOLLOWERS = [f for f, _ in pairs]
                    if hasattr(cfg, "STARTED"):
                        try: cfg.STARTED.add(me)
                        except Exception: pass
        except Exception: pass
        self._refresh_now()
        self._refresh_buttons()

    def _draw_scene(self, me, front, rear, gap_f, gap_r):
        self.canvas.delete("all")
        W = int(self.canvas.winfo_width() or 520)
        H = int(self.canvas.winfo_height() or 360)
        cx = W // 2
        w, h = 160, 50
        y_front, y_me, y_rear = 80, H // 2, H - 80

        def box(xc, yc, label, fill):
            self.canvas.create_rectangle(xc - w // 2, yc - h // 2, xc + w // 2, yc + h // 2, fill=fill, outline="black")
            self.canvas.create_text(xc, yc, text=label, font=("Arial", 12, "bold"))

        if not (front or rear): return
        my_color = self.VEH_COLORS.get(me, "#efefef")
        box(cx, y_me, me, my_color)
        if front:
            front_color = self.VEH_COLORS.get(front, "#d9efff")
            box(cx, y_front, front, front_color)
            if gap_f is not None: self.canvas.create_text(cx, (y_front + y_me) // 2, text=f"gap: {gap_f:.1f} m", font=("Arial", 11))
        if rear:
            rear_color = self.VEH_COLORS.get(rear, "#ffe3c2")
            box(cx, y_rear, rear, rear_color)
            if gap_r is not None: self.canvas.create_text(cx, (y_me + y_rear) // 2, text=f"gap: {gap_r:.1f} m", font=("Arial", 11))

    def _refresh_now(self):
        try:
            me = self.selected.get()
            chain = _order_chain(cfg.FOLLOW_PAIRS)
            self.listbox.delete(0, tk.END)
            highlight_idx = None

            if me in chain:
                for i, v in enumerate(chain):
                    mark = ""
                    if i == 0: mark = " ⚑"
                    self.listbox.insert(tk.END, f"{i}. {v}{mark}")
                    if v == me: highlight_idx = i
                try:
                    for i in range(len(chain)): self.listbox.itemconfig(i, {'bg': 'white'})
                    if highlight_idx is not None: self.listbox.itemconfig(highlight_idx, {'bg': "#aeaeae"})
                except Exception: pass
            else:
                cand_map = getattr(cfg, "NEARBY_PLATOON", {})
                cand = cand_map.get(me, [])
                if not cand: cand = _nearby_fallback(self.traci, me, chain, PLATOON_JOIN_DISTANCE)
                if not cand: self.listbox.insert(tk.END, "300m 내 참여 후보 없음")
                else:
                    d = getattr(cfg, "VEHICLE_DISTANCES", {}).get(me, float("inf"))
                    if d != float("inf"): self.listbox.insert(tk.END, f"→ 거리: {d:.1f} m")
                    else: self.listbox.insert(tk.END, "→ 거리: —")

            if me in chain:
                self.status_var.set("상태: 플래투닝 참여중")
                self.status_lbl.configure(foreground="#2e7d32")
            else:
                self.status_var.set("상태: 미참여")
                self.status_lbl.configure(foreground="#6b7280")

            if me and (me in chain):
                front, rear = _neighbors(chain, me)
                gap_f, gap_r = None, None
                if front and _has_started(self.traci, me) and _has_started(self.traci, front):
                    gap_f = _gap_between(self.traci, me, front)
                if rear and _has_started(self.traci, me) and _has_started(self.traci, rear):
                    gap_r = _gap_between(self.traci, rear, me)
                self._draw_scene(me, front, rear, gap_f, gap_r)
            else:
                self.canvas.delete("all")
                W, H = int(self.canvas.winfo_width() or 520), int(self.canvas.winfo_height() or 360)
                cx, cy = W // 2, H // 2
                w, h = 160, 50
                my_color = self.VEH_COLORS.get(me, "#f5f5f5")
                self.canvas.create_rectangle(cx - w // 2, cy - h // 2, cx + w // 2, cy + h // 2, fill=my_color, outline="black")
                self.canvas.create_text(cx, cy, text=me, font=("Arial", 12, "bold"))

            self.ctrl.update()
            try:
                leader_id = chain[0] if chain else None
                leader_dest = self._get_destination_str(self.traci, leader_id) if leader_id else "—"
                me_dest     = self._get_destination_str(self.traci, me) if me else "—"
                self.dest_leader_var.set(f"리더 목적지: {leader_dest}")
                self.dest_me_var.set(f"내 목적지: {me_dest}")
                if me in chain: self._show(self.lbl_dest_leader)
                else: self._hide(self.lbl_dest_leader)
            except Exception: pass

            # [중요] 스케줄러 호출들
            _tick_merge_coordinator(self.traci)  # 합류 코디네이터
            _tick_pending_merge(self.traci)
            _tick_lane_mode_restore(self.traci)
            _tick_join_cooldown(self.traci)
            _tick_leave_guard(self.traci)

        except Exception: pass

    def _tick(self):
        self._refresh_candidates()
        self._refresh_now()
        self._refresh_buttons()
        self.after(500, self._tick)

def open_vehicle_viewer(parent, traci_mod, candidates):
    return VehicleViewer(parent, traci_mod, candidates)