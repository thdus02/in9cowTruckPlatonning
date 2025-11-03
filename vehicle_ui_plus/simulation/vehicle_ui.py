import tkinter as tk
from tkinter import ttk
import math
import simulation.config as cfg
from simulation.leader_controller import LeaderController

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

# ===== 단일 뷰어 창 =====
class VehicleViewer(tk.Toplevel):
    """
    드롭다운으로 차량을 선택해 보는 단일 창 뷰어.
    레이아웃: 좌(참여/나가기/브레이크) | 중(이웃 네모 + gap) | 우(체인 리스트)
    """
    def __init__(self, parent, traci_mod, initial_candidates):
        super().__init__(parent)
        self.title("Truck Platooning – Vehicle UIs")
        self.traci = traci_mod

        # --- 상단 바: 차량 선택 ---
        top = ttk.Frame(self); top.pack(fill="x", padx=10, pady=6)
        ttk.Label(top, text="내 차량:", font=("Arial", 11, "bold")).pack(side="left")

        self.candidates = list(initial_candidates) if initial_candidates else ["Veh0", "Veh1", "Veh2", "Veh3"]
        self.selected = tk.StringVar(value=self.candidates[0])
        self.combo = ttk.Combobox(top, textvariable=self.selected, values=self.candidates,
                                  width=10, state="readonly")
        self.combo.pack(side="left", padx=8)

        # --- 본문 3열 ---
        body = ttk.Frame(self); body.pack(padx=8, pady=8, fill="both", expand=True)
        self.left  = ttk.Frame(body); self.left.grid(row=0, column=0, sticky="nsw", padx=(0,10))
        self.mid   = ttk.Frame(body); self.mid.grid(row=0, column=1, sticky="nsew", padx=(0,10))
        self.right = ttk.Frame(body); self.right.grid(row=0, column=2, sticky="nse")

        body.columnconfigure(1, weight=1)
        body.rowconfigure(0, weight=1)

        # 좌측: 버튼들
        self.btn_join  = tk.Button(self.left, text="참여하기", width=12, command=self._on_join)
        self.btn_leave = tk.Button(self.left, text="나가기",   width=12, command=self._on_leave)
        self.btn_brake = tk.Button(self.left, text="브레이크", width=12)
        self.btn_join.pack(anchor="w", pady=2)
        self.btn_leave.pack(anchor="w", pady=2)
        self.btn_brake.pack(anchor="w", pady=(8,2))

        # 브레이크 컨트롤러(선택 바뀌면 타겟 교체)
        self.ctrl = LeaderController(traci_mod=self.traci)
        self.ctrl.set_leader(self.selected.get())
        self.btn_brake.bind("<ButtonPress-1>",  self.ctrl.on_brake_press)
        self.btn_brake.bind("<ButtonRelease-1>", self.ctrl.on_brake_release)

        # 가운데: 이웃 뷰 (캔버스)
        self.canvas = tk.Canvas(self.mid, width=520, height=360, bg="white",
                                highlightthickness=1, relief="solid")
        self.canvas.pack(fill="both", expand=True)

        # 오른쪽: 체인 리스트
        ttk.Label(self.right, text="플래투닝 차량", font=("Arial", 11, "bold")).pack(anchor="w")
        self.listbox = tk.Listbox(self.right, width=18, height=16)
        self.listbox.pack(pady=(6,0), fill="y")

        # 이벤트
        self.combo.bind("<<ComboboxSelected>>", self._on_select)

        # 주기 갱신 루프 시작
        self._tick()

    # ---- 콜백들 ----
    def _on_select(self, _evt=None):
        self.ctrl.set_leader(self.selected.get())
        self._refresh_now()  # 즉시 반영

    def _on_join(self):
        chain = _order_chain(cfg.FOLLOW_PAIRS)
        me = self.selected.get()
        if not chain:
            cfg.FOLLOW_PAIRS = []
            cfg.FOLLOWERS = []
            self._refresh_now()
            return
        if me not in chain:
            last = chain[-1]
            cfg.FOLLOW_PAIRS.append((me, last))
            cfg.FOLLOWERS = [f for f, _ in cfg.FOLLOW_PAIRS]
        self._refresh_now()  # 즉시 반영

    def _on_leave(self):
        me = self.selected.get()
        pairs = list(cfg.FOLLOW_PAIRS)
        chain = _order_chain(pairs)
        if me not in chain:
            return

        i = chain.index(me)
        front = chain[i-1] if i-1 >= 0 else None
        rear  = chain[i+1] if i+1 < len(chain) else None

        # me 관련 페어 제거
        pairs = [(f, l) for (f, l) in pairs if f != me and l != me]
        # 뒤차를 앞차에 재연결
        if rear and front:
            pairs.append((rear, front))

        cfg.FOLLOW_PAIRS = pairs
        cfg.FOLLOWERS = [f for f, _ in pairs]

        # me가 체인에서 빠졌다면 리더로 자동 전환 (체인 남아있을 때)
        new_chain = _order_chain(cfg.FOLLOW_PAIRS)
        if me not in new_chain:
            if new_chain:
                self.selected.set(new_chain[0])
                self.ctrl.set_leader(new_chain[0])
            else:
                # 체인 비었으면 오른쪽/가운데 비우기
                self.listbox.delete(0, tk.END)
                self.canvas.delete("all")

        self._refresh_now()  # 즉시 반영

    # ---- 그리기 ----
    def _draw_scene(self, me, front, rear, gap_f, gap_r):
        self.canvas.delete("all")
        W = int(self.canvas.winfo_width() or 520)
        H = int(self.canvas.winfo_height() or 360)
        cx = W // 2
        w, h = 160, 50
        y_front, y_me, y_rear = 80, H // 2, H - 80

        def box(xc, yc, label, fill):
            self.canvas.create_rectangle(xc - w // 2, yc - h // 2, xc + w // 2, yc + h // 2,
                                         fill=fill, outline="black")
            self.canvas.create_text(xc, yc, text=label, font=("Arial", 12, "bold"))

        # 플래투닝이 아니면 빈 칸 유지
        if not (front or rear):
            return

        # 내 차
        box(cx, y_me, me, "#efefef")
        # 앞차
        if front:
            box(cx, y_front, front, "#d9efff")
            if gap_f is not None:
                self.canvas.create_text(cx, (y_front + y_me) // 2,
                                        text=f"gap: {gap_f:.1f} m", font=("Arial", 11))
        # 뒷차
        if rear:
            box(cx, y_rear, rear, "#ffe3c2")
            if gap_r is not None:
                self.canvas.create_text(cx, (y_me + y_rear) // 2,
                                        text=f"gap: {gap_r:.1f} m", font=("Arial", 11))

    # ---- 즉시/주기 갱신 ----
    def _refresh_now(self):
        try:
            me = self.selected.get()
            chain = _order_chain(cfg.FOLLOW_PAIRS)

            # 오른쪽 리스트 갱신
            self.listbox.delete(0, tk.END)
            for v in chain:
                self.listbox.insert(tk.END, v)

            # 이웃/갭 계산 및 그리기
            if me and (me in chain):
                front, rear = _neighbors(chain, me)

                # ✅ 출발(주행) 상태일 때만 gap 표시
                gap_f = None
                gap_r = None
                if front and _has_started(self.traci, me) and _has_started(self.traci, front):
                    gap_f = _gap_between(self.traci, me, front)
                if rear and _has_started(self.traci, me) and _has_started(self.traci, rear):
                    gap_r = _gap_between(self.traci, rear, me)

                self._draw_scene(me, front, rear, gap_f, gap_r)
            else:
                # 선택 차량이 체인 밖이면 빈 화면
                self.canvas.delete("all")

            # 브레이크 자동 복귀
            self.ctrl.update()
        except Exception:
            pass

    def _tick(self):
        self._refresh_now()
        self.after(80, self._tick)  # 12.5 Hz 정도로 갱신

def open_vehicle_viewer(parent, traci_mod, candidates):
    """app.py에서 호출: 단일 뷰어 창 열기"""
    return VehicleViewer(parent, traci_mod, candidates)
