import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import GetState, ChangeState
from lifecycle_msgs.msg import Transition
import time
from geometry_msgs.msg import PoseWithCovarianceStamped

class MapManager(Node):
    def __init__(self):
        self._ctx = rclpy.Context()
        rclpy.init(context=self._ctx)
        super().__init__('map_manager_client', context=self._ctx)
        self.ordered_nodes = [
            '/controller_server', '/planner_server', '/smoother_server',
            '/behavior_server', '/bt_navigator', '/waypoint_follower', '/velocity_smoother'
        ]

    def _change_state(self, node_name, transition_id):
        client = self.create_client(ChangeState, f'{node_name}/change_state')
        if not client.wait_for_service(timeout_sec=2.0):
            print(f"[{node_name}] 서비스를 찾을 수 없습니다.")
            return False

        req = ChangeState.Request()
        req.transition.id = transition_id
        
        future = client.call_async(req)
        executor = rclpy.executors.SingleThreadedExecutor(context=self._ctx)
        executor.add_node(self)
        executor.spin_until_future_complete(future, timeout_sec=30.0)
        executor.remove_node(self)
        return future.result() is not None

    def _get_state(self, node_name):
        client = self.create_client(GetState, f'{node_name}/get_state')
        if not client.wait_for_service(timeout_sec=1.0):
            return "unknown"

        req = GetState.Request()
        future = client.call_async(req)
        executor = rclpy.executors.SingleThreadedExecutor(context=self._ctx)
        executor.add_node(self)
        executor.spin_until_future_complete(future, timeout_sec=2.0)
        executor.remove_node(self)
        
        if future.result() is not None:
            return future.result().current_state.label
        return "unknown"

    # map_server.launch.py 시작 함수
    def start_map_server(self, map_path, is_slam=False):
        print(f"=== 맵 변경 시작: {map_path} ===")
        
        # subprocess 대신 파라미터 클라이언트 사용 (여기선 일단 기존 subprocess 유지하되 최소화)
        # import subprocess
        # setup = "source /opt/ros/humble/setup.bash && source ~/turtlebot3_ws/install/setup.bash"
        # subprocess.run(f"{setup} && ros2 param set /map_server yaml_filename {map_path}", shell=True, executable='/bin/bash')

        # 2. 모든 노드 순차 활성화
        all_nodes = ['/map_server', '/amcl'] + self.ordered_nodes
        
        for node in all_nodes:
            current = self._get_state(node)
            print(f"[{node}] 현재 상태: {current}")

            if current == 'active': continue

            # 순서: unconfigured -> configure -> inactive -> activate -> active
            if current == 'unconfigured':
                print(f"[{node}] Configuring...")
                self._change_state(node, Transition.TRANSITION_CONFIGURE)
            
            # 잠시 대기 후 Activate
            print(f"[{node}] Activating...")
            self._change_state(node, Transition.TRANSITION_ACTIVATE)
            
            # 상태 확인
            if self._get_state(node) == 'active':
                print(f"[{node}] 활성화 완료")
            else:
                print(f"[{node}] 활성화 지연 중 (다음 노드로 진행)")

        # self._publish_initialpose()
        print("=== 모든 프로세스 완료 ===")
        return True

    def _publish_initialpose(self):
        """토픽 발행도 직접 수행"""
        pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.orientation.w = 1.0
        # 공분산(Covariance) 설정 생략 (필요시 추가)
        
        # Subscriber가 연결될 때까지 아주 짧게 대기 후 발행
        time.sleep(1)
        pub.publish(msg)
        print("=== Initial Pose 발행 완료 ===")
