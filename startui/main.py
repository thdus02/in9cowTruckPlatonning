
from simulation.startui import open_selector_and_wait
import simulation.config as cfg

# app.run은 cfg를 매 스텝 읽도록 이미 수정된 버전이어야 함!
from simulation.app import run

if __name__ == "__main__":
    # 1) 선택 창 먼저
    open_selector_and_wait()

    # 2) 선택 결과 유효성 체크(선택 안 했을 때 대비)
    if not cfg.FOLLOW_PAIRS:
        print("[WARN] FOLLOW_PAIRS가 비어있습니다. 리더/팔로워를 선택해야 플래투닝이 동작합니다.")

    # 3) SUMO 실행
    run()
