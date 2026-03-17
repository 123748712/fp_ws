# 🤖 Industrial All-in-One Robot Management System (ROS2 & Web)

이 프로젝트는 **ROS2 Humble**과 **Flask**를 결합하여, 기존의 RVIZ나 Cartographer 없이도 웹 브라우저에서 로봇을 실시간으로 제어, 모니터링 및 맵핑할 수 있는 **산업용 올인원 관리 시스템**입니다.

---

## 🚀 주요 기능 (Key Features)

### 1. 실시간 웹 제어 인터페이스 (Web UI)
* **Virtual Joystick:** 웹 브라우저에서 마우스/터치로 로봇 이동 제어 (Teleop).
* **Live Mapping:** Cartographer를 연동하여 웹상에서 실시간으로 지도를 생성 및 시각화.
* **Status Monitoring:** 로봇의 센서 데이터 및 연결 상태를 한눈에 파악.

### 2. 자율 주행 및 내비게이션 (Autonomous Navigation)
* **Waypoint Navigation:** 지도상의 특정 지점을 클릭하여 로봇을 자동으로 이동시키는 기능 (CRUD 기반 웨이포인트 관리).
* **A* Algorithm:** 최적의 경로 탐색을 위한 자체 알고리즘 구현 및 테스트.
* **Automated Parking:** 정밀한 센서 데이터를 활용한 자동 주차 시스템.

### 3. 지능형 객체 인식 (Computer Vision)
* **YOLOv8 Integration:** 화살표, 숫자, 장애물 등을 실시간으로 탐색 및 인식.
* **Cargo Verification:** QR 코드를 활용하여 화물 적재 및 하역 거리의 정밀도 검증.

### 4. 강화학습 최적화 (Reinforcement Learning)
* **DQN 기반 주행:** DQN(Deep Q-Network) 알고리즘을 적용하여 자율 주행 성능 최적화.
* **효율성 증대:** 가상 환경 학습을 통해 주행 단계(Step Count)를 약 **38% 단축**하는 성과 달성.

---

## 🛠 기술 스택 (Tech Stack)

### Robotics & Backend
* **OS:** Ubuntu 22.04 LTS
* **Framework:** ROS2 Humble / Foxy
* **Robot:** TurtleBot3 Waffle Pi
* **Language:** Python 3.10
* **Server:** Flask (Web Server), SQL (Data Management)

### Frontend
* **Library:** Roslibjs, Ros2djs (ROS-Web Bridge)
* **Styling:** Bootstrap 4, FontAwesome

---

## 📂 프로젝트 구조 (Project Structure)

```text
fp_ws/
├── src/
│   └── fp_pkg/
│       ├── fp_pkg/            # ROS2 노드 (A*, Drive, Unloading 등)
│       ├── launch/            # 실행용 Launch 파일 (pi_all_in_one 등)
│       ├── static/            # Web UI 자원 (JS, CSS)
│       ├── templates/         # HTML 템플릿 (Main UI)
│       ├── database/          # 미션 및 로그 데이터 관리
│       └── app.py             # Flask 메인 서버
├── .gitignore                 # 빌드/임시 파일 관리
└── README.md
