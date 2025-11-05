import tkinter as tk
from tkinter import ttk
import traci
import simulation.config as cfg

def _get_stopped_ids(traci_mod):
    out = []
    try:
        for vid in traci_mod.vehicle.getIDList():
            try:
                if traci_mod.vehicle.isStopped(vid):
                    out.append(vid)
            except traci_mod.exceptions.TraCIException:
                pass
    except traci_mod.exceptions.TraCIException:
        pass
    return out

class StartUI(tk.Tk):
    def __init__(self, traci_mod):
        super().__init__()
        self.title("플래투닝 차량 선택 (주차 완료 후)")
        self.resizable(False, False)
        self.traci = traci_mod

        self.candidates = _get_stopped_ids(self.traci)  # 정차/주차 중인 차량만
        self.leader_var = tk.StringVar(value="")
        self.follower_vars = {vid: tk.BooleanVar(value=False) for vid in self.candidates}
        self.cb_widgets = {}
        self._chain = []  # 반환할 [Leader, F1, F2, ...]

        self._build_ui()
        self._bind_state()
        self._follower_state()

    def _build_ui(self):
        main = ttk.Frame(self, padding=10); main.pack(fill="both", expand=True)

        L_select = ttk.LabelFrame(main, text="Leader", padding=10)
        L_select.pack(side=tk.LEFT, fill="both", expand=True)

        ttk.Separator(main, orient="vertical").pack(side=tk.LEFT, fill=tk.Y, padx=8)

        R_select = ttk.LabelFrame(main, text="Follower", padding=10)
        R_select.pack(side=tk.LEFT, fill="both", expand=True)

        if not self.candidates:
            ttk.Label(L_select, text="(정지한 차량 없음)").pack(anchor="w", pady=2)
            ttk.Label(R_select, text="(정지한 차량 없음)").pack(anchor="w", pady=2)
        else:
            for vid in self.candidates:
                ttk.Radiobutton(L_select, text=vid, value=vid, variable=self.leader_var)\
                    .pack(anchor="w", pady=2)
            for vid in self.candidates:
                cb = ttk.Checkbutton(R_select, text=vid, variable=self.follower_vars[vid])
                cb.pack(anchor="w", pady=2)
                self.cb_widgets[vid] = cb

        self.status = tk.StringVar(value="주차(정지) 상태의 차량만 목록에 표시됩니다.")
        ttk.Label(self, textvariable=self.status).pack(padx=10, pady=(4, 0), anchor="w")

        btns = ttk.Frame(self); btns.pack(pady=10)
        ttk.Button(btns, text="확인", width=12, command=self._on_ok).pack(side=tk.LEFT, padx=6)
        ttk.Button(btns, text="취소", width=12, command=self._on_cancel).pack(side=tk.LEFT, padx=6)

    def _bind_state(self):
        self.leader_var.trace_add("write", lambda *_: self._follower_state())
        for vid, var in self.follower_vars.items():
            var.trace_add("write", lambda *_, v=vid: self._prevent_leader_in_followers(v))

    def _follower_state(self):
        leader = self.leader_var.get()
        for vid, cb in self.cb_widgets.items():
            if leader and vid == leader:
                self.follower_vars[vid].set(False)
                cb.state(["disabled"])
            else:
                cb.state(["!disabled"])

    def _prevent_leader_in_followers(self, vid):
        if vid == self.leader_var.get() and self.follower_vars[vid].get():
            self.follower_vars[vid].set(False)

    def _on_ok(self):
        leader = self.leader_var.get()
        if not leader:
            self.status.set("리더를 선택하세요."); return
        followers = [v for v, var in self.follower_vars.items() if var.get() and v != leader]
        self._chain = [leader] + followers
        self.destroy()


    def _on_cancel(self):
        self._chain = []
        self.destroy()

    def get_chain(self):
        return list(self._chain)

def open_selector_and_wait(traci_mod):
    ui = StartUI(traci_mod)
    ui.mainloop()
    return ui.get_chain()