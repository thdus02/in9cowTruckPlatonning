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

가. 시스템 구성도 



나. 구현 기능

- 차량 주차 및 초기화
- 리더/팔로워 설정
- 플래투닝 상태 실시간 확인
- 차량별 속도/간격 UI 표시
- 합류·이탈·재합류 기능
- 중간합류 & 꼬리합류 자동 판별
- 출발/브레이크 제어
- 끼어들기 감지 및 갭 확장
- CACC 기반 자동 간격 유지
- 연비/CO₂ 계산 및 시각화
- 전체 시스템 통합 제어


1. 시스템 UI 흐름

2. 구성 내용

   2-1. CACC 기반 추종 제어 알고리즘
            팔로워 차량은 CACC(Cooperative Adaptive Cruise Control) 원리에 따라 리더와의 간격을 유지한다. 
            본 연구에서는 SUMO 내부 자동제어 대신 TraCI 외부 제어 방식을 사용하여 실시간으로 속도 및 차선 명령을 전달한다.
            
             (1) 기준 간격 Time Headway 모델
<img width="281" height="112" alt="image" src="https://github.com/user-attachments/assets/11c45d4e-b43b-45d8-a10e-f09237068458" />
<img width="282" height="40" alt="image" src="https://github.com/user-attachments/assets/65f4e5c7-4bfb-44f2-9ffa-1a97a6a863af" />

             (2) PD 기반 제어식
             현재 간격(d), 목표 간격, 속도 차이(v_L−v_F), 앞차 가속도를 이용해 팔로워의 목표 가속도를 산출한다.
<img width="315" height="39" alt="image" src="https://github.com/user-attachments/assets/db5906fd-994a-48ef-baf2-4a32d85a9865" />
<img width="214" height="72" alt="image" src="https://github.com/user-attachments/assets/d6b7229b-0ad1-48f2-8743-45f9cc8cb7ed" />

            산출된 가속도는 다음 스텝의 속도 명령으로 변환되어 적용된다.

   2-2. 플래투닝 상황별 이벤트 



4. 결과

 
https://github.com/user-attachments/assets/9ff2c064-ebfe-4253-a139-72d3d84c2e87

[플래투닝 이탈 및 중간 합류 결과 영상]
    
플래투닝 이탈 → 단독 주행 → 간격 확보 → 자연스러운 재합류를 구현하였다.
    
    
https://github.com/user-attachments/assets/1100c6df-3e73-4d6b-91f4-87eb9c563ed5

[일반차 중간 합류 결과 영상]
    
일반 차량 접근 → 대열 간격 확장 → 안전 여유 공간 확보 → 자연스러운 끼어들기 합류를 구현하였다.


https://github.com/user-attachments/assets/9a5d35ed-dc30-4ef0-aaa8-d9476d1ad1f1

[PA_1 주차 차량 출발 후 플래투닝 차량 꼬리 합류 결과 영상]
      
다른 출발지에서 출발 → 대열 300m 이내 접근 → 가까운 플래투닝 차량 인식 → 자연스러운 끼어들기 합류를 구현하였다.



**5. 개발환경**
   -   Python 3.9.13
   -   SUMO 1.24.0
   -   TraCI API 사용 (SUMO ↔ Python 연동)
   -   Tkinter GUI Toolkit
