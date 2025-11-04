import tkinter as tk
from tkinter import ttk
import simulation.config as cfg

VEH_LIST = {
    "차량 1":"Veh0",
    "차량 2":"Veh1",
    "차량 3":"Veh2",
    "차량 4":"Veh3",
}

class StartUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Select Platonn")
    
        self.leader_var = tk.StringVar(value="")
        self.follower_vars = {vid: tk.BooleanVar(value=False) for vid in VEH_LIST.values()}
        self.cb_widgets = {}

        self.build_ui()
        self.bind_state()

    def build_ui(self):
        #Leader와 Follower 버튼을 담을 박스
        L_select = ttk.LabelFrame(self, text="Leader", padding=10)
        L_select.pack(side=tk.LEFT, fill="both", expand=True)

        ttk.Separator(self, orient="vertical").pack(side=tk.LEFT, fill=tk.Y, padx=4)

        R_select = ttk.Labelframe(self, text="Follower", padding=10)
        R_select.pack(side=tk.LEFT, fill="both", expand=True)

        #라디오 버튼(Leader)
        for label, vid in VEH_LIST.items():
            ttk.Radiobutton(L_select, text=label, value=vid, variable=self.leader_var).pack(anchor="w", pady=2)
    
        #체크박스 버튼(Follower)
        for label, vid in VEH_LIST.items():
            cb = ttk.Checkbutton(R_select, text=label, variable=self.follower_vars[vid])
            cb.pack(anchor="w", pady=2)
            self.cb_widgets[vid] = cb

        ttk.Button(self, text="플래투닝 시작!", command=self.apply).pack(side=tk.BOTTOM, pady=10)
        self.follower_state()
    
    def bind_state(self):
        self.leader_var.trace_add("write", lambda *_: self.follower_state())

        for vid, var in self.follower_vars.items():
            var.trace_add("write", lambda *_, v=vid: self.mistake_follower(v))
    
    def follower_state(self):
        leader = self.leader_var.get()
        for vid, cb in self.cb_widgets.items():
            if leader and vid == leader:
                self.follower_vars[vid].set(False)
                cb.state(["disabled"])
            else:
                cb.state(["!disabled"])

    def mistake_follower(self, vid):
        if vid == self.leader_var.get() and self.follower_vars[vid].get():
            self.follower_vars[vid].set(False)

    def apply(self):
        leader = self.leader_var.get()
        followers = [v for v, var in self.follower_vars.items() if var.get() and v != leader]

        # FOLLOW_PAIRS 구성
        cfg.FOLLOW_PAIRS = []
        prev = leader
        for f in followers:
            cfg.FOLLOW_PAIRS.append((f, prev))
            prev = f
        cfg.FOLLOWERS = [f for f, _ in cfg.FOLLOW_PAIRS]

        # ✅ 선택 결과를 명시적으로 저장 (여기가 핵심)
        cfg.SELECTABLE_VEHICLES = list(VEH_LIST.values())          # UI에서 보이던 전체 후보
        cfg.PLATOON_CHAIN = ([leader] + followers) if leader else []
        cfg.NON_PLATOON   = [v for v in cfg.SELECTABLE_VEHICLES if v not in cfg.PLATOON_CHAIN]

        print("PLATOON_CHAIN:", cfg.PLATOON_CHAIN)
        print("NON_PLATOON:", cfg.NON_PLATOON)

        self.destroy()



def open_selector_and_wait():
    """main에서 호출: 선택 완료되면 창 닫히고 반환."""
    app = StartUI()
    app.mainloop()