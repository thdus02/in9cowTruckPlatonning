# simulation/startui.py
import tkinter as tk
from tkinter import ttk, messagebox

def _get_pa0_ids(traci_mod):
    """pa_0에 실제로 주차 중인 차량 ID만 반환"""
    try:
        return list(traci_mod.parkingarea.getVehicleIDs("pa_0"))
    except Exception:
        return []

class StartUI(tk.Tk):
    def __init__(self, traci_mod):
        super().__init__()

        # 창 생성 직후 위치 지정
        self.geometry("+500+400")
        
        self.title("플래투닝 차량 선택")
        self.resizable(False, False)
        self.traci = traci_mod

        # pa_0 차량만 후보에 포함
        self.candidates = _get_pa0_ids(self.traci)

        # 상태 변수
        self.leader_var = tk.StringVar(value="")
        self.follower_vars = {vid: tk.BooleanVar(value=False) for vid in self.candidates}

        # 위젯 레퍼런스
        self.cb_widgets = {} 

        # 반환 체인 [Leader, F1, F2, ...]
        self._chain = []

        self._build_ui()
        self._bind_state()
        self._follower_state()

    # ---------- UI ----------
    def _build_ui(self):
        main = ttk.Frame(self, padding=10)
        main.pack(fill="both", expand=True)

        L_select = ttk.LabelFrame(main, text="Leader (pa_0만 표시)", padding=10)
        L_select.pack(side=tk.LEFT, fill="both", expand=True)

        ttk.Separator(main, orient="vertical").pack(side=tk.LEFT, fill=tk.Y, padx=8)

        R_select = ttk.LabelFrame(main, text="Follower (pa_0만 표시)", padding=10)
        R_select.pack(side=tk.LEFT, fill="both", expand=True)

        if not self.candidates:
            ttk.Label(L_select, text="(pa_0에 차량이 없습니다)").pack(anchor="w", pady=2)
            ttk.Label(R_select, text="(pa_0에 차량이 없습니다)").pack(anchor="w", pady=2)
        else:
            for vid in self.candidates:
                ttk.Radiobutton(L_select, text=vid, value=vid, variable=self.leader_var)\
                    .pack(anchor="w", pady=2)

            for vid in self.candidates:
                cb = ttk.Checkbutton(R_select, text=vid, variable=self.follower_vars[vid])
                cb.pack(anchor="w", pady=2)
                self.cb_widgets[vid] = cb

        # 하단 버튼
        btns = ttk.Frame(self)
        btns.pack(pady=10)
        ttk.Button(btns, text="확인", width=12, command=self._on_ok).pack(side=tk.LEFT, padx=6)
        ttk.Button(btns, text="취소", width=12, command=self._on_cancel).pack(side=tk.LEFT, padx=6)

    # ---------- 상태 바인딩 ----------
    def _bind_state(self):
        # 리더 변경 시 팔로워에서 본인 비활성화
        self.leader_var.trace_add("write", lambda *_: self._follower_state())

        # 리더를 팔로워로 체크하지 못하도록 방지
        for vid, var in self.follower_vars.items():
            var.trace_add("write", lambda *_, v=vid: self._prevent_leader_in_followers(v))

    def _follower_state(self):
        """리더가 선택되면 같은 ID의 팔로워 체크박스는 비활성화"""
        leader = self.leader_var.get()
        for vid, cb in self.cb_widgets.items():
            if leader and vid == leader:
                # 본인을 팔로워로 체크 못 하게
                self.follower_vars[vid].set(False)
                cb.state(["disabled"])
            else:
                cb.state(["!disabled"])

    def _prevent_leader_in_followers(self, vid):
        if vid == self.leader_var.get() and self.follower_vars[vid].get():
            self.follower_vars[vid].set(False)

    # ---------- 확인/취소 ----------
    def _on_ok(self):
        leader = self.leader_var.get()
        if not leader:
            messagebox.showwarning("선택 오류", "리더를 선택하세요.", parent=self)
            return

        followers = [v for v, var in self.follower_vars.items() if var.get() and v != leader]

        self._chain = [leader] + followers
        self.destroy()

    def _on_cancel(self):
        self._chain = []
        self.destroy()

    def get_chain(self):
        return list(self._chain)

# 외부에서 호출하는 함수
def open_selector_and_wait(traci_mod):
    ui = StartUI(traci_mod)
    ui.mainloop()
    return ui.get_chain()
