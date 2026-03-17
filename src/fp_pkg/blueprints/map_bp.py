from flask import Blueprint, request, jsonify
# from database.map_service import MapService
import os
import time
import signal
import subprocess
import yaml
from database.map_service import map_service
map_bp = Blueprint('map_bp', __name__)
# map_service = MapService()

def kill_ros_nodes():
    kill_list = [
        "cartographer",
        "occupancy_grid_node",
        "map_server", "amcl", "nav2_amcl",
        "controller_server", "planner_server", "bt_navigator",
        "behavior_server", "waypoint_follower", "velocity_smoother", "smoother_server"
    ]
    # 1차 종료
    for proc in kill_list:
        os.system(f"pkill -f '{proc}' 2>/dev/null")
    time.sleep(2)

    # 2차 강제 종료
    for proc in kill_list:
        os.system(f"pkill -9 -f '{proc}' 2>/dev/null")

    # ↓↓↓ 추가: 프로세스가 실제로 죽었는지 확인 ↓↓↓
    import psutil
    max_wait = 10  # 최대 10초 대기
    start = time.time()
    while time.time() - start < max_wait:
        still_alive = False
        for proc in psutil.process_iter(['cmdline']):
            try:
                cmdline = ' '.join(proc.info['cmdline'] or [])
                if any(k in cmdline for k in kill_list):
                    still_alive = True
                    break
            except:
                pass
        if not still_alive:
            print("=== 모든 ROS 노드 종료 확인 ===")
            break
        time.sleep(0.5)

# 지도 리스트 조회 (SelectBox용)
@map_bp.route('/getMaps.do', methods=['GET'])
def get_maps():
    maps = map_service.get_maps()
    return jsonify(maps)

# 지도 변경 (SelectBox Change 이벤트)
@map_bp.route('/switchMap.do', methods=['POST'])
def switch_map():
    data = request.get_json()
    print(f"===================== {data}")  # ← 여기서 map_seq가 1인지 2인지 확인
    map = map_service.get_map(data)

    change_map = map_service.change_map_status(map) # 현재 선택된 맵 플래그

    if not change_map:
        print(f"===================== error : 맵 변경 중 에러 발생 =====================") 
        return

    print(f"===================== {map}")
    map = map_service.get_map(data)
    map_path = map['map_file_path']

    # launch 재시작
    # kill_list = ["map_server", "amcl", "controller_server", "planner_server",
    #              "bt_navigator", "behavior_server", "waypoint_follower", "velocity_smoother", "smoother_server"]
    # for proc in kill_list:
    #     os.system(f"pkill -f '{proc}' 2>/dev/null")
    # time.sleep(2)
    # for proc in kill_list:
    #     os.system(f"pkill -9 -f '{proc}' 2>/dev/null")
    kill_ros_nodes()

    setup_cmd = "source /opt/ros/humble/setup.bash && source ~/turtlebot3_ws/install/setup.bash"
    launch_cmd = f"ros2 launch fp_pkg map_server.launch.py map:={map_path}"
    log_file = open("/tmp/ros_launch.log", "w")
    subprocess.Popen(
        ["/bin/bash", "-c", f"{setup_cmd} && {launch_cmd}"],
        preexec_fn=os.setsid,
        stdout=log_file,
        stderr=log_file
    )

    time.sleep(10)  # 노드 올라올 때까지 대기
    
    success = map_service.map_mgr.start_map_server(map_path, is_slam=False)
    
    if success:
        return jsonify({"result": "success", "message": "지도를 교체합니다."})
    else:
        return jsonify({"result": "error", "message": "맵 서버 활성화 타임아웃"}), 500
    
# 지도 그리기
@map_bp.route('/drawMap.do', methods=['POST'])
def draw_map():
    try:
        kill_ros_nodes()
        time.sleep(2)

        setup_cmd = "source /opt/ros/humble/setup.bash && source ~/turtlebot3_ws/install/setup.bash"
        launch_cmd = "ros2 launch fp_pkg map_server.launch.py slam:=True use_sim_time:=False"
        subprocess.Popen(
            ["/bin/bash", "-c", f"{setup_cmd} && {launch_cmd}"],
            start_new_session=True
        )

        return jsonify({"result": "success", "message": "맵 생성을 시작합니다. 로봇을 움직여주세요."})
    except Exception as e:
        return jsonify({"result": "error", "message": str(e)})


# 지도 저장
@map_bp.route('/saveMap.do', methods=['POST'])
def save_map():
    try:
        print("===================== save_map 진입 =====================")
        data = request.get_json()
        map_name = data.get('map_name', 'new_map')
        save_path = f"/home/dev/my_map/{map_name}"

        setup_cmd = "source /opt/ros/humble/setup.bash && source ~/turtlebot3_ws/install/setup.bash"

        print("=== /map 토픽 대기 중 ===")
        map_wait = subprocess.run(
            ["/bin/bash", "-c",
            f"{setup_cmd} && ros2 topic hz /map --window 3 2>&1 | head -5"],
            capture_output=True, text=True, timeout=30
        )
        print(f"=== /map 상태: {map_wait.stdout} ===")

        result = subprocess.run(
            ["/bin/bash", "-c",
            f"{setup_cmd} && ros2 run nav2_map_server map_saver_cli -f {save_path} --ros-args -p save_map_timeout:=10.0"],
            capture_output=True, text=True, timeout=30  # 15 → 30으로 증가
        )
        print("===================== nav2_map_server 저장 완료 =====================")
        if result.returncode != 0:
            return jsonify({"result": "error", "message": "맵 저장 실패: " + result.stderr}), 500


        with open(f"{save_path}.yaml", 'r') as f:
            data = yaml.safe_load(f)

        # 상대 경로를 절대 경로로 변경
        if not data['image'].startswith('/'):
            data['image'] = f"/home/dev/my_map/{data['image']}"

        with open(f"{save_path}.yaml", 'w') as f:
            yaml.dump(data, f)
            
        new_map = map_service.save_map({
            "map_name": map_name,
            "map_file_path": f"{save_path}.yaml"
        })

        # print("=== odom 리셋 중 ===")
        # setup_cmd = "source /opt/ros/humble/setup.bash && source ~/turtlebot3_ws/install/setup.bash"
        # subprocess.run(
        #     ["/bin/bash", "-c",
        #      f"{setup_cmd} && ros2 service call /reset std_srvs/srv/Trigger {{}}"],
        #     capture_output=True, text=True, timeout=5
        # )
        # time.sleep(1)
        # print("=== odom 리셋 완료 ===")

        print("=== ROS 2 Launch 자동 재 실행 중 ===")

        os.system("pkill -9 -f map_server.launch.py")
        
        kill_ros_nodes()
        # kill_list = [
        #     "map_server", "amcl", "nav2_amcl", 
        #     "planner_server", "controller_server", "bt_navigator"
        # ]
        # for proc in kill_list:
        #     os.system(f"pkill -f '{proc}' 2>/dev/null")
            
        # time.sleep(2)
        # for proc in kill_list:
        #     os.system(f"pkill -9 -f '{proc}' 2>/dev/null")

        # 현재 active 상태인 map 조회
        # current_map = map_service.get_active_map()

        setup_cmd = "source /opt/ros/humble/setup.bash && source ~/turtlebot3_ws/install/setup.bash"
        launch_cmd = f"ros2 launch fp_pkg map_server.launch.py map:={save_path}.yaml"
        
        log_file = open("/tmp/ros_launch.log", "w")
        ros_process = subprocess.Popen(
            ["/bin/bash", "-c", f"{setup_cmd} && {launch_cmd}"],
            preexec_fn=os.setsid,
            stdout=log_file,
            stderr=log_file
        )

        time.sleep(10) # 노드들이 'unconfigured' 상태로 올라올 때까지 대기

        print(f"=== Lifecycle 활성화 시작: {save_path}.yaml ===")
        success = map_service.map_mgr.start_map_server(f"{save_path}.yaml", is_slam=False)

        if success:
            print("=== 모든 노드 Lifecycle 활성화 성공 ===")
        else:
            print("=== Lifecycle 활성화 실패 (로그 확인 필요) ===")

    #    
        return jsonify({"result": "success", "map": new_map})
    except Exception as e:
        return jsonify({"result": "error", "message": str(e)})


# 지도 삭제
@map_bp.route('/deleteMap.do', methods=['POST'])
def delete_map():
    try:
        data = request.get_json()
        result = map_service.delete_map(data)

        print("=== ROS 2 Launch 자동 재 실행 중 ===")

        os.system("pkill -9 -f map_server.launch.py")

        # kill_list = [
        #     "map_server", "amcl", "nav2_amcl", 
        #     "planner_server", "controller_server", "bt_navigator"
        # ]
        # for proc in kill_list:
        #     os.system(f"pkill -f '{proc}' 2>/dev/null")
        # time.sleep(2)
        # for proc in kill_list:
        #     os.system(f"pkill -9 -f '{proc}' 2>/dev/null")
        kill_ros_nodes()

        # 현재 active 상태인 map 조회
        current_map = map_service.get_active_map()

        setup_cmd = "source /opt/ros/humble/setup.bash && source ~/turtlebot3_ws/install/setup.bash"
        launch_cmd = f"ros2 launch fp_pkg map_server.launch.py map:={current_map['map_file_path']}"
        
        log_file = open("/tmp/ros_launch.log", "w")
        ros_process = subprocess.Popen(
            ["/bin/bash", "-c", f"{setup_cmd} && {launch_cmd}"],
            preexec_fn=os.setsid,
            stdout=log_file,
            stderr=log_file
        )

        time.sleep(10) # 노드들이 'unconfigured' 상태로 올라올 때까지 대기

        print(f"=== Lifecycle 활성화 시작: {current_map['map_file_path']} ===")
        success = map_service.map_mgr.start_map_server(f"{current_map['map_file_path']}", is_slam=False)

        if success:
            print("=== 모든 노드 Lifecycle 활성화 성공 ===")
        else:
            print("=== Lifecycle 활성화 실패 (로그 확인 필요) ===")

    #    
        return jsonify({"result": "success"})
    except Exception as e:
        return jsonify({"result": "error", "message": str(e)})


# 지도 그리기
@map_bp.route('/getMapYaml.do', methods=['POST'])
def get_mapYaml():
    try:
        current_map = map_service.get_active_map()

        with open(current_map['map_file_path'], 'r') as f:
            mapYaml = yaml.safe_load(f)
        return jsonify({"result": "success", "mapYaml": mapYaml})
    except Exception as e:
        return jsonify({"result": "error", "message": str(e)})
