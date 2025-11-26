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

- 트럭 플래투닝은 여러 대의 화물차가 짧은 간격을 유지하며 군집 주행함으로써 연료 절감, 탄소 배출 감소, 물류 효율 향상을 기대할 수 있는 차세대 물류 기술이다. 그러나 실제 도로에서 다양한 상황을 반복 실험하기에는 비용과 위험이 크기 때문에, 본 프로젝트에서는 SUMO 시뮬레이터 기반의 플래투닝 실험 환경을 구축하였다. 이를 통해 CACC 기반 간격·속도 제어를 비롯하여 합류, 이탈, 끼어들기와 같은 동적 주행 시나리오까지 안정적으로 처리하는 제어 로직을 구현하였으며, 플래투닝의 동작 특성과 안전성을 자유롭게 실험·분석할 수 있는 연구용 테스트베드를 제공하는 것을 목표로 한다.


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

   ---
   (1) 기준 간격 Time Headway 모델
    
   <img width="281" height="112" alt="image" src="https://github.com/user-attachments/assets/bec8ffb2-2978-40f3-a7f9-9c3d3e4385ce" />
    
   <img width="366" height="52" alt="image" src="https://github.com/user-attachments/assets/9f5333bd-90c9-4a4f-bf68-4761c6a2b6f2" />

   ---
    
   (2) PD 기반 제어식
   
   현재 간격(d), 목표 간격, 속도 차이(v_L−v_F), 앞차 가속도를 이용해 팔로워의 목표 가속도를 산출한다.
    
   <img width="315" height="44" alt="image" src="https://github.com/user-attachments/assets/205e87c3-1998-4a16-9abe-99d4a2d08974" />

   <img width="244" height="85" alt="image" src="https://github.com/user-attachments/assets/5d712bbd-5481-41a3-9504-7112bbfffcfd" />

   산출된 가속도는 다음 스텝의 속도 명령으로 변환되어 적용된다.

   ---


   2-2. 플래투닝 상황별 이벤트

   <img width="649" height="74" alt="image" src="https://github.com/user-attachments/assets/157ac4d4-7afc-4aea-b558-949ea15af145" />

   <img width="466" height="117" alt="image" src="https://github.com/user-attachments/assets/85c94b41-dc12-4129-a6ac-3c3bac182f62" />

   시뮬레이션을 시작하면 리더/팔로워 구성 및 초기 제어 설정 작업이 시작된다.
   
   <img width="682" height="210" alt="image" src="https://github.com/user-attachments/assets/e430590e-7ac3-4798-b7ca-8403cd9540c7" />

   주행이 시작되면 메인 루프에서 매 스텝마다 다음 작업들이 반복된다.

   1. 중간 합류 – 기존 대열 사이로 들어오는 플래투닝 차량
   2. 일반 차량 끼어들기(Cut-in)
   3. 플래투닝 이탈

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



**4. 개발환경**
   -   Python 3.9.13
   -   SUMO 1.24.0
   -   TraCI API 사용 (SUMO ↔ Python 연동)
   -   Tkinter GUI Toolkit
