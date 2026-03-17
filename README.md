 자율주행 지게차 웹 제어 시스템
 
TurtleBot3(Waffle Pi) 기반의 자율주행 로봇을 웹 브라우저에서 실시간으로 제어하는 풀스택 프로젝트입니다.
SLAM으로 지도를 생성하고, 생성된 지도 위에서 로봇의 위치 설정 및 자율주행 경로를 지정할 수 있습니다.
 
---
 
## 기술 스택
 
| 분류 | 기술 |
|------|------|
| Backend | Python, Flask, Flask-SocketIO |
| Frontend | HTML, JavaScript, jQuery, Bootstrap 4 |
| ROS2 시각화 | ROS2D.js, ROSLIB.js, EaselJS |
| ROS2 | Humble, Nav2, Cartographer, AMCL, rosbridge |
| 데이터베이스 | 관계형 DB (지도 정보 관리) |
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
    - waypoint_follower : 경유 노드 기반 주행
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
- 지도 위에서 경유 노드(Waypoint) 생성, 수정, 삭제
- 노드 타입 분류 및 DB 저장
- 지도별 노드 독립 관리
 
---

## 핵심 컴포넌트 설명
 
### app.py
Flask 서버의 진입점입니다.
- 서버 시작 시 `start_ros_launch()`를 호출하여 ROS2 런치 파일을 자동 실행합니다.
- 런치 실행 후 `MapManager.start_map_server()`를 호출하여 모든 Nav2 노드의 Lifecycle을 활성화합니다.
- `FollowWayPointNode`를 별도 스레드에서 실행하여 웨이포인트 주행 요청을 처리합니다.
 
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
