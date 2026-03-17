import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction, LogInfo

def launch_setup(context, *args, **kwargs):
    pi_ip = "192.168.0.26"
    pi_user = "pi"
    domain_id = "17"
    model = "waffle_pi"
    lds_model = "LDS-02"
    
    remote_env = (
        f"export ROS_DOMAIN_ID={domain_id} && "
        f"export TURTLEBOT3_MODEL={model} && "
        f"export LDS_MODEL={lds_model} && "
        f"source /opt/ros/humble/setup.bash && "
        f"source ~/fp_ws/install/setup.bash"
    )
    ssh_base = f"ssh -t {pi_user}@{pi_ip}"

    # 1. Bringup 실행 (새 창 1)
    bringup_cmd = f"gnome-terminal --title='1. TB3 Bringup' -- bash -c \"{ssh_base} '{remote_env} && ros2 launch turtlebot3_bringup robot.launch.py'; exec bash\""
    os.system(bringup_cmd)

    # --- 사용자 입력 대기 ---
    print("\n" + "="*60)
    print(" [Step 1] 첫 번째 창에 비번을 입력하여 Bringup을 완료하세요.")
    user_input = input(" [Step 2] 준비되었다면 'y'를 눌러 나머지 노드 창을 엽니다: ")
    print("="*60 + "\n")

    if user_input.lower() != 'y':
        return [LogInfo(msg="사용자가 중단했습니다.")]

    # 2. 모든 노드가 환경 변수를 공유하도록 괄호 ( )로 묶어서 실행
    all_nodes_cmd = (
        f"gnome-terminal --title='2. Sensors & Nodes' -- bash -c \""
        f"{ssh_base} '{remote_env} && ("
        f"ros2 run v4l2_camera v4l2_camera_node & "
        f"ros2 run fp_pkg carry_node & "
        f"ros2 run fp_pkg sensor_node & "
        f"wait)'; exec bash\""
    )
    
    os.system(all_nodes_cmd)

    return [LogInfo(msg="모든 노드 환경 설정 및 실행 명령이 재전송되었습니다.")]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])