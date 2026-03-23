import rclpy
from flask import Flask, render_template, jsonify, request
from flask_socketio import SocketIO
import os
from flask_cors import CORS
from threading import Thread
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from fp_pkg.followWayPointNode import FollowWayPointNode
import atexit
import signal
import time
import platform
import subprocess
from blueprints.node_bp import node_bp
from blueprints.map_bp import map_bp
from blueprints.drive_bp import drive_bp
from blueprints.alert_bp import alert_bp
from database.map_service import map_service
from flask_socketio import SocketIO, emit
from utils.llm_handler import LLMController
import threading

app = Flask(__name__)
app.register_blueprint(node_bp, url_prefix='/node')
app.register_blueprint(map_bp, url_prefix='/map')
app.register_blueprint(drive_bp, url_prefix='/drive')
app.register_blueprint(alert_bp, url_prefix='/alert')
CORS(app)

socketio = SocketIO(app, cors_allowed_origins="*")

ros_node = None
ros_process = None

llm = None
llm_running = False

@socketio.on('chat_command')
def handle_chat_command(json_data):
    global llm_running
    if llm_running:
        socketio.emit('chat_response', {'data': "이전 명령 수행 중입니다. 잠시 후 다시 시도하세요."})
        return

    user_text = json_data['data']
    command_data = llm.get_robot_command(user_text)

    if command_data and 'steps' in command_data:
        llm_running = True
        t = threading.Thread(
            target=run_llm_sequence,
            args=(command_data['steps'], ),
            daemon=True
        )
        t.start()
    else:
        socketio.emit('chat_response', {'data': "죄송합니다. 명령을 이해하지 못했습니다."})

def run_llm_sequence(steps):
    global llm_running
    try:
        llm.execute_robot_sequence(steps, socketio)
    finally:
        llm_running = False

def start_ros_launch():
    global ros_process
    print("=== ROS 2 환경 정리 및 실행 시작 ===")

    kill_cmd = "pkill -9 -f 'map_server|amcl|controller|planner|bt_navigator|behavior|waypoint|velocity|lifecycle|map_manager_client|integrated_navigation'"
    print(f"kill_cmd : {kill_cmd}")
    subprocess.run(["/bin/bash", "-c", kill_cmd], stderr=subprocess.DEVNULL)
    print('kill cmd success')
    subprocess.run(["ros2", "daemon", "stop"], stderr=subprocess.DEVNULL)
    print('daemon stop success')
    # 2. Launch 실행
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

    # map_server의 'get_state' 서비스가 나타날 때까지만 기다림
    print("=== 노드 대기 중 (최대 10초) ===")
    start_wait = time.time()
    node_ready = False
    
    while time.time() - start_wait < 10:
        # map_server가 목록에 떴는지 확인
        check = subprocess.run(["ros2", "node", "list"], capture_output=True, text=True)
        if "/map_server" in check.stdout:
            print(f"=== 노드 발견! ({round(time.time()-start_wait, 2)}초 소요) ===")
            node_ready = True
            break
        time.sleep(0.5) # 0.5초 간격으로 체크

    if not node_ready:
        print("=== 경고: 노드가 제한 시간 내에 뜨지 않았습니다. ===")

    # 4. 바로 Lifecycle 활성화 단계로 진입
    print(f"=== Lifecycle 활성화 시작 ===")
    success = map_service.map_mgr.start_map_server(current_map['map_file_path'])

    if success:
        print("=== 모든 과정 완료 ===")
    else:
        print("=== 활성화 실패 ===")

    

def stop_ros_launch():
    global ros_process
    if ros_process and ros_process.poll() is None:
        print("=== 프로세스 전체 종료 중... ===")
        try:
            os.killpg(os.getpgid(ros_process.pid), signal.SIGKILL)
            ros_process = None
        except Exception as e:
            print(f"=== 종료 실패: {e} ===")
        
atexit.register(stop_ros_launch)

@app.route('/main.do')
def main():
    return render_template('main.html')

@app.route('/log.do')
def log():
    return render_template('log.html')

if __name__ == '__main__':
    if not rclpy.ok():
        rclpy.init()
    start_ros_launch()

    llm = LLMController()
    print("=== LLMController 생성 완료 ===")
    socketio.run(app, host='0.0.0.0', port=5000, debug=False, use_reloader=False)
