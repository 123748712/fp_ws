
 

https://github.com/user-attachments/assets/dc78d335-0dca-4559-957d-9d4529d9f628


https://github.com/user-attachments/assets/819617d0-72ca-4d3a-9966-93a4c5fb1c53


https://github.com/user-attachments/assets/49b76ce1-932d-4eb1-9146-448e1720c6a7



 자율주행 지게차 웹 제어 시스템
 
TurtleBot3(Waffle Pi) 기반의 자율주행 로봇을 웹 브라우저에서 실시간으로 제어하는 풀스택 프로젝트입니다.
SLAM으로 지도를 생성하고, 생성된 지도 위에서 로봇의 위치 설정 및 기능 별 노드를 통한 자율주행 경로를 지정할 수 있습니다.
 
---
 
## 기술 스택
 
| 분류 | 기술 |
|------|------|
| Backend | Python, Flask, Flask-SocketIO |
| Frontend | HTML, JavaScript, jQuery, Bootstrap 4 |
| ROS2 시각화 | ROS2D.js, ROSLIB.js |
| ROS2 | Humble, Nav2, Cartographer, AMCL, rosbridge |
| 데이터베이스 | 관계형 DB (지도, 노드 정보 관리) |
| 로봇 | TurtleBot3 Waffle Pi |
 
---
 
## 시스템 아키텍처
 
```
[브라우저]
    ↕ HTTP (Flask)
[Flask 서버 - app.py]
    ↕ rosbridge WebSocket
[ROS2 노드들]
    - map_server      : 저장된 지도 제공
    - amcl            : 로봇 위치 추정
    - controller_server, planner_server, bt_navigator
    - cartographer    : SLAM 실시간 지도 생성
[TurtleBot3 Waffle Pi]
```
 
---
 
## 주요 기능
 
### 지도 관리
- SLAM(Cartographer)을 이용한 실시간 지도 생성
- 조이스틱으로 로봇을 조작하며 환경 스캔
- 완성된 지도를 yaml/pgm 파일로 저장 및 DB 등록
- 저장된 복수의 지도를 SelectBox로 전환
 
### 로봇 제어
- 조이스틱(nipplejs) 기반 수동 제어
- 지도 위 드래그로 로봇 초기 위치(InitialPose) 설정
- 지도 위 드래그로 목적지 설정 및 단일 목적지 자율주행
- 경유 노드를 지정한 웨이포인트 기반 순차 자율주행
 
### 실시간 시각화
- 브라우저에서 OccupancyGrid 맵 렌더링
- LiDAR 스캔 포인트 실시간 표시 (초록색 점)
- AMCL 위치 추정 기반 로봇 아이콘 실시간 이동
- SLAM 모드에서 지도가 확장될 때 뷰 자동 조정
 
### 노드 관리
- 지도 위에서 경유, 상/하차, 대기, 충전 노드(Waypoint) 생성, 수정, 삭제
- 노드 타입 분류 및 DB 저장
- 지도별 노드 독립 관리

### 상차 기능
ArUco 마커 기반 정밀 정렬 후 지게발을 상승시켜 팔레트 상차
- 카메라로 팔레트에 부착된 ArUco 마커를 탐색
- 마커의 윗선과 카메라 기준 임의의 수평선을 비교하여, 두 선이 동일 선상에 놓이면 지게차가 목표 거리에 도달했다고 판단
- 판단 후 일정 거리만큼 전진하고, 모터를 구동하여 지게발을 들어 올려 팔레트를 상차

### 하차 기능
라인 트레이싱과 ArUco 마커 인식을 결합하여 정확한 위치에 팔레트 하차
- 현재 운반 중인 팔레트의 고유 ID에 매핑된 색상의 라인을 우선적으로 트레이싱하며 하차 위치로 이동
- 이동 중 전방의 ArUco 마커를 인식하여, 마커가 카메라 기준 임계선을 넘어서면 목표 위치에 충분히 도달했다고 판단
- 지게발을 하강시켜 팔레트를 내려놓고, 하차 완료 후 다시 상차 노드로 이동하여 다음 사이클을 시작

### LOCAL LLM 기반 자연어 로봇 제어
Ollama를 통해 로컬에서 구동되는 Qwen 2.5 3B 모델을 활용하여 자연어 명령으로 로봇의 이동 및 작업을 제어합니다.
- 자연어 명령 해석: 사용자의 채팅 명령을 분석하여 로봇이 이해할 수 있는 이동 시퀀스(move, rotate, nav, fork)로 변환하고 순차적으로 실행합니다.
- 복합 명령 지원: "대기 노드로 이동한 후 지게발 들어줘"와 같이 이동과 작업을 하나의 명령으로 연결하여 처리할 수 있으며, 각 동작의 완료를 확인한 후 다음 동작을 수행합니다.
- 노드 기반 목적지 이동: 상차, 하차, 대기, 충전 등 DB에 등록된 노드 이름을 자연어로 호출하면 해당 좌표로 자율 이동합니다.
- 저지연 온디바이스 추론: 외부 클라우드 연결 없이 로컬 GPU 자원을 활용하여 빠른 응답 속도로 명령을 처리합니다.
- 중복 명령 방지: 이전 명령 수행 중 새로운 명령이 입력되면 완료 후 처리되도록 스레드 단위로 실행을 관리합니다.

---

## 핵심 시나리오 설명
핵심 시나리오
자율주행 지게차는 상차 → 경로 주행 → 하차 → 복귀의 사이클을 반복하며 패트롤 작업을 수행합니다.
사용자가 웹 UI에서 상/하차 대상 팔레트를 선택하고 자동 상/하차 시작 버튼을 누르면 전체 시나리오가 시작됩니다.

로봇이 해당 팔레트의 ArUco 마커를 탐색합니다.
상차 작업을 완료한 후 설정된 경유 노드를 따라 하차 장소까지 자율주행합니다.
하차 장소에서 하차 작업을 완료한 후 다시 상차 노드로 복귀하며 패트롤을 반복합니다.

---


## 핵심 컴포넌트 설명
 
### app.py
Flask 서버의 진입점입니다.
- 서버 시작 시 `start_ros_launch()`를 호출하여 ROS2 런치 파일을 자동 실행합니다.
- 런치 실행 후 `MapManager.start_map_server()`를 호출하여 모든 Nav2 노드의 Lifecycle을 활성화합니다.
 
### map_server.launch.py
ROS2 노드들을 실행하는 런치 파일입니다.
- `slam:=false` (기본값): map_server + amcl + Nav2 navigation 노드들을 실행합니다.
- `slam:=true`: Cartographer SLAM을 실행하여 실시간 지도 생성 모드로 전환합니다.
- Nav2 노드들은 `autostart: false`로 설정되어 있어 `MapManager`가 직접 Lifecycle을 관리합니다.
 
### MapManager (map_manager.py)
Nav2 노드들의 Lifecycle 상태를 관리하는 핵심 클래스입니다.
 
ROS2 Lifecycle 노드는 `unconfigured → inactive → active` 순서로 전환해야 동작합니다.
map_server와 amcl은 지도 전환 시 yaml 파일을 교체해야 하므로 별도로 관리하고,
나머지 Nav2 노드들은 의존성 순서에 따라 자동으로 활성화합니다.
