# simulation/cutin_ui.py
import tkinter as tk
from tkinter import ttk, messagebox

def open_cutin_panel(parent, cutin_mgr, get_chain_callable):
    win = tk.Toplevel(parent)
    win.title("Cut-in Scenario")
    win.geometry("340x240+1300+50")  # ★ 위치 지정
    win.lift(); win.attributes("-topmost", True); win.after(200, lambda: win.attributes("-topmost", False))

    frm = ttk.Frame(win, padding=10); frm.pack(fill="both", expand=True)

    ttk.Label(frm, text="Leader (앞차):").pack(anchor="w")
    leader_var = tk.StringVar()
    cmb_leader = ttk.Combobox(frm, textvariable=leader_var, state="readonly"); cmb_leader.pack(fill="x")

    ttk.Label(frm, text="Follower (뒤차):").pack(anchor="w", pady=(8,0))
    follower_var = tk.StringVar()
    cmb_follower = ttk.Combobox(frm, textvariable=follower_var, state="readonly"); cmb_follower.pack(fill="x")

    status_var = tk.StringVar(value="상태: idle")
    ttk.Label(frm, textvariable=status_var).pack(anchor="w", pady=(6,4))

    def refresh_chain():
        try:
            chain = get_chain_callable() or []
        except Exception:
            chain = []
        cmb_leader["values"] = chain
        cmb_follower["values"] = chain
        if chain:
            leader_var.set(chain[0])
            if len(chain) >= 2:
                follower_var.set(chain[1])

    def on_spawn():
        L = leader_var.get(); F = follower_var.get()
        if not L or not F or L == F:
            messagebox.showwarning("선택 오류", "리더/팔로워를 올바르게 선택하세요.")
            return
        if not cutin_mgr.ready():
            messagebox.showinfo("안내", "이미 시나리오가 진행 중입니다.")
            return
        if cutin_mgr.start(L, F, car_id="VehCut"):
            status_var.set("상태: approach (끼어들기 대기)")

    def on_cut_in():
        ok = cutin_mgr.request_cut_in()
        if ok:
            status_var.set("상태: in_main (끼어든 상태)")

    def on_cut_out():
        ok = cutin_mgr.request_cut_out()
        if ok:
            status_var.set("상태: cut_out (복귀 중)")

    row = ttk.Frame(frm); row.pack(fill="x", pady=(10,0))
    ttk.Button(row, text="체인 새로고침", command=refresh_chain).pack(side="left")

    # 버튼 분리: 생성/접근, 끼어들기, 나가기
    btns = ttk.Frame(frm); btns.pack(fill="x", pady=(10,0))
    ttk.Button(btns, text="일반차 생성&접근", command=on_spawn).pack(side="left", expand=True, fill="x", padx=(0,6))
    ttk.Button(btns, text="끼어들기", command=on_cut_in).pack(side="left", expand=True, fill="x", padx=6)
    ttk.Button(btns, text="나가기", command=on_cut_out).pack(side="left", expand=True, fill="x", padx=(6,0))

    refresh_chain()
    return win
