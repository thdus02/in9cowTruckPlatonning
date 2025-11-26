# Capstone_25_2
## SUMO_Simulation_Platooning



**1. 프로젝트명**

- 트럭 플래투닝 (Truck Platooning) 시스템 개발



**2. 팀명: 인구소**
- 팀장 - 컴퓨터공학과 202101750 김진경
- 팀원 - 컴퓨터공학과 202101733 김가람
- 팀원 - 컴퓨터공학과 202101775 안소연
- 팀원 - 컴퓨터공학과 202101809 조하연



**3. 기획 의도**



**4. 작품 설명**

      가. 시스템 구성도 ( 어떤거 썼는지.. sumo 설명 등 넣으면 되나)

      나. 구현 기능
  
    1. 시스템 UI 흐름

    2. 구성 내용

         2-1. CACC 기반 추종 제어 알고리즘

             (1) 기준 간격 Time Headway 모델

             (2) PD 기반 제어식

         2-2. 플래투닝 상황별 이벤트 

    3. 결과 
    
    
    https://github.com/user-attachments/assets/9ff2c064-ebfe-4253-a139-72d3d84c2e87


    [플래투닝 이탈 및 합류 결과 영상]
    
    플래투닝 이탈 → 단독 주행 → 간격 확보 → 자연스러운 재합류를 구현하였다.
    
    
    https://github.com/user-attachments/assets/1100c6df-3e73-4d6b-91f4-87eb9c563ed5

    
    [일반차 중간 합류 결과 영상]
    
    일반 차량 접근 → 대열 간격 확장 → 안전 여유 공간 확보 → 자연스러운 끼어들기를 구현하였다.
      




**5. 개발환경**
   -   Python 3.9.13
   -   SUMO 1.24.0
   -   TraCI API 사용 (SUMO ↔ Python 연동)
   -   Tkinter GUI Toolkit
