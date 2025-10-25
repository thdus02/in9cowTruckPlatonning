# Step 1~3: import & SUMO_HOME 경로 처리 + TraCI Start + 메인 루프
import os, sys, tkinter as tk, traci
from simulation.config import Sumo_config, FOLLOW_PAIRS
from simulation.safety import init_safety_defaults
from simulation.leader_controller import LeaderController
from simulation.platoon import (
    ensure_initial_gap_lock,
    maintain_or_release_lock,
    control_follower_speed,
    boost_followers_once,
)
from simulation.ui import build_speedometer, update_vehicle, build_gap_labels

def _gap_and_thw_text(traci_mod, follower_id, leader_id):
    vehicles = set(traci_mod.vehicle.getIDList())
    if follower_id not in vehicles:
        return "—", "—"
    lead_info = traci_mod.vehicle.getLeader(follower_id, 1000.0)
    if not lead_info or lead_info[0] != leader_id:
        return "—", "—"
    gap_m = max(0.0, lead_info[1])
    vF = traci_mod.vehicle.getSpeed(follower_id)
    thw = f"{(gap_m / vF):.2f} s" if vF > 1e-3 else "∞ s"
    return f"{gap_m:,.1f} m", thw

def run():
    # Step 2: Establish path to SUMO (SUMO_HOME)
    if 'SUMO_HOME' in os.environ:
        tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
        sys.path.append(tools)
    else:
        sys.exit("Please declare environment variable 'SUMO_HOME'")

    # Step 5: Open connection between SUMO and TraCI
    traci.start(Sumo_config)
    init_safety_defaults()

    # Step 8: Tk GUI & bindings
    root = tk.Tk(); root.title("Truck Platooning Speedometers")

    # Leader / Followers speedometers
    canvL, needleL, labelL = build_speedometer(root, "Leader",    col=0, needle_color="red")
    canvF1, needleF1, labelF1 = build_speedometer(root, "Follower1", col=1, needle_color="blue")
    canvF2, needleF2, labelF2 = build_speedometer(root, "Follower2", col=2, needle_color="green")

    gap1_label, gap2_label, thw1_label, thw2_label = build_gap_labels(root)

    # Brake button panel
    panel = tk.Frame(root); panel.grid(row=6, column=0, columnspan=3, pady=(10,5))
    btn_brake = tk.Button(panel, text="Brake (click)", width=14); btn_brake.grid(row=0, column=0, padx=5, pady=5)
    ctrl = LeaderController(traci_mod=traci, sim_dt=0.05, ramp_down_per_s=1.5, ramp_up_per_s=0.8, min_factor=0.0)
    btn_brake.bind("<ButtonPress-1>",  ctrl.on_brake_press)
    btn_brake.bind("<ButtonRelease-1>", ctrl.on_brake_release)

    def update_loop():
        # Step 7: Define functions (update loop)
        try:
            traci.simulationStep()
        except traci.exceptions.TraCIException:
            root.quit(); return

        # 초기 락 / 해제
        for fid, lid in FOLLOW_PAIRS:
            ensure_initial_gap_lock(fid, lid)
            maintain_or_release_lock(fid, lid)

        boost_followers_once()

        # UI: speed
        update_vehicle(traci, "Leader", canvL, needleL, labelL)
        update_vehicle(traci, "Follower1", canvF1, needleF1, labelF1)
        update_vehicle(traci, "Follower2", canvF2, needleF2, labelF2)

        # UI: gaps
        g1, thw1 = _gap_and_thw_text(traci, "Follower1", "Leader")
        gap1_label.config(text=f"Gap L→F1: {g1}"); thw1_label.config(text=f"Headway F1: {thw1}")
        g2, thw2 = _gap_and_thw_text(traci, "Follower2", "Follower1")
        gap2_label.config(text=f"Gap F1→F2: {g2}"); thw2_label.config(text=f"Headway F2: {thw2}")

        ctrl.update()
        for fid, lid in FOLLOW_PAIRS:
            control_follower_speed(fid, lid)

        root.after(100, update_loop)

    def on_close():
        try: traci.close(False)
        except: pass
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.after(100, update_loop)
    try:
        root.mainloop()
    finally:
        try: traci.close(False)
        except: pass
