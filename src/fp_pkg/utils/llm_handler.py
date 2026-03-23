import requests
import json
import time
from rclpy.node import Node 
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
import sys
from database.node_service import NodeService
from database.map_service import map_service
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import math
from std_msgs.msg import String

sys.path.append('/home/dev/fp_ws/src/fp_pkg')

class LLMController(Node):
    def __init__(self, model="qwen2.5:3b"):
        super().__init__('llm_controller')

        reliable_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        fork_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        
        self.url = "http://localhost:11434/api/generate"
        self.model = model
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.llm_goal_pub = self.create_publisher(PoseStamped, '/llm_goal', 10)
        self.fork_pub = self.create_publisher(String, '/fork_cmd', fork_qos)
        self.node_service = NodeService()

        # 로봇 위치 수신 토픽
        self.current_pose = None
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)

    def pose_callback(self, msg):
        self.current_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )


    def get_robot_command(self, user_text):
        try:
            prompt = f"""You are a robot motion command parser.
                Your only job is to convert Korean natural language into a JSON motion sequence.
                Do not include any explanation. Output JSON only.

                ## Output Format
                {{"steps": [...]}}

                ## Step Types

                ### 1. move
                {{"type": "move", "dir": "forward" | "backward", "val": <meters>}}

                ### 2. rotate
                {{"type": "rotate", "dir": "left" | "right", "angle": 1.57}}

                ### 3. nav
                {{"type": "nav", "place": "<node_id>"}}

                ### 4. fork
                지게발 올리기/내리기
                {{"type": "fork", "action": "UP" | "DOWN"}}

                ## Available Nodes
                | 키워드                        | node_id  |
                |-------------------------------|----------|
                | 충전, 충전소                   | CHRG001  |
                | 상차1, 상차 1번                | LOAD001  |
                | 상차2, 상차 2번                | LOAD002  |
                | 상차3, 상차 3번                | LOAD003  |
                | 경유1, 1번 노드                | NODE001  |
                | 경유2, 2번 노드                | NODE002  |
                | 경유3, 3번 노드                | NODE003  |
                | 경유4, 4번 노드                | NODE004  |
                | 하차1, 하차 1번                | UNLD001  |
                | 하차2, 하차 2번                | UNLD002  |
                | 하차3, 하차 3번                | UNLD003  |
                | 대기, 대기 노드, 대기 장소       | WAIT001  |

                ## Unit Conversion
                - cm → m (50cm = 0.5)
                - 미터 / m → use as-is

                ## Critical Rules
                - "앞", "전진" → move forward ONLY. Never add rotate.
                - "뒤", "후진" → move backward ONLY. Never add rotate.
                - "왼쪽" → rotate left (1.57), then move forward.
                - "오른쪽" → rotate right (1.57), then move forward.
                - Place/destination name → nav ONLY. Never add move or rotate.
                - Always map Korean keywords to the correct node_id from the table above.
                - "지게발 올려줘", "지게발 들어줘", "포크 올려" → fork UP ONLY. Never add nav.
                - "지게발 내려줘", "지게발 내려", "포크 내려" → fork DOWN ONLY. Never add nav.
                - fork 명령은 단독으로 사용될 때 절대 nav를 추가하지 않습니다.
                - nav는 오직 장소/노드 이름이 명시된 경우에만 사용합니다.
                - "지게발 올려줘", "지게발 내려줘" 등 단독 액션 명령이 들어오면 오직 `fork` step 하나만 배열에 넣습니다.
                - 사용자가 명시적으로 장소를 말하지 않았다면, 절대 임의로 장소(예: CHRG001, LOAD001 등)를 유추해서 `nav` step을 추가하지 않습니다.

                ## Compound Command Rules
                여러 동작을 순서대로 처리할 수 있습니다.
                - "~한 후", "~하고 나서", "~하고", "그 다음" 등의 표현은 순서를 의미합니다.
                - 각 동작을 steps 배열에 순서대로 추가하세요.

                ## Examples

                Input: 앞으로 1m 가줘
                Output: {{"steps": [{{"type": "move", "dir": "forward", "val": 1.0}}]}}

                Input: 앞으로 50cm 이동해
                Output: {{"steps": [{{"type": "move", "dir": "forward", "val": 0.5}}]}}

                Input: 뒤로 1미터 이동해줘
                Output: {{"steps": [{{"type": "move", "dir": "backward", "val": 1.0}}]}}

                Input: 뒤로 50cm 가
                Output: {{"steps": [{{"type": "move", "dir": "backward", "val": 0.5}}]}}

                Input: 후진 2미터
                Output: {{"steps": [{{"type": "move", "dir": "backward", "val": 2.0}}]}}

                Input: 왼쪽으로 1미터 이동
                Output: {{"steps": [{{"type": "rotate", "dir": "left", "angle": 1.57}}, {{"type": "move", "dir": "forward", "val": 1.0}}]}}

                Input: 오른쪽으로 50cm
                Output: {{"steps": [{{"type": "rotate", "dir": "right", "angle": 1.57}}, {{"type": "move", "dir": "forward", "val": 0.5}}]}}

                Input: 충전소로 가줘
                Output: {{"steps": [{{"type": "nav", "place": "CHRG001"}}]}}

                Input: 상차 위치로 이동
                Output: {{"steps": [{{"type": "nav", "place": "LOAD001"}}]}}

                Input: 상차 2번으로 가줘
                Output: {{"steps": [{{"type": "nav", "place": "LOAD002"}}]}}

                Input: 하차 노드로 이동해줘
                Output: {{"steps": [{{"type": "nav", "place": "UNLD001"}}]}}

                Input: 하차 3번으로 가
                Output: {{"steps": [{{"type": "nav", "place": "UNLD003"}}]}}

                Input: 대기 장소로 이동해줘
                Output: {{"steps": [{{"type": "nav", "place": "WAIT001"}}]}}

                Input: 2번 경유 노드로 가줘
                Output: {{"steps": [{{"type": "nav", "place": "NODE002"}}]}}

                Input: 경유 4번으로 이동
                Output: {{"steps": [{{"type": "nav", "place": "NODE004"}}]}}

                Input: 지게발 올려줘
                Output: {{"steps": [{{"type": "fork", "action": "UP"}}]}}

                Input: 지게발 내려줘
                Output: {{"steps": [{{"type": "fork", "action": "DOWN"}}]}}

                Input: 포크 들어줘
                Output: {{"steps": [{{"type": "fork", "action": "UP"}}]}}
                
                Input: 대기 노드로 이동한 후 지게발 들어줘
                Output: {{"steps": [{{"type": "nav", "place": "WAIT001"}}, {{"type": "fork", "action": "UP"}}]}}

                Input: 상차 1번으로 이동하고 지게발 올려줘
                Output: {{"steps": [{{"type": "nav", "place": "LOAD001"}}, {{"type": "fork", "action": "UP"}}]}}

                Input: 하차 노드로 가서 지게발 내려줘
                Output: {{"steps": [{{"type": "nav", "place": "UNLD001"}}, {{"type": "fork", "action": "DOWN"}}]}}

                Input: 앞으로 1m 가고 왼쪽으로 돌아줘
                Output: {{"steps": [{{"type": "move", "dir": "forward", "val": 1.0}}, {{"type": "rotate", "dir": "left", "angle": 1.57}}]}}

                Input: 충전소로 이동한 후 지게발 내려줘
                Output: {{"steps": [{{"type": "nav", "place": "CHRG001"}}, {{"type": "fork", "action": "DOWN"}}]}}

                ## Anti-Examples (never do this)
                Input: 뒤로 1미터
                WRONG: {{"steps": [{{"type": "rotate", "dir": "left", "angle": 1.57}}, {{"type": "move", "dir": "backward", "val": 1.0}}]}}
                CORRECT: {{"steps": [{{"type": "move", "dir": "backward", "val": 1.0}}]}}

                Input: 충전소로 가줘
                WRONG: {{"steps": [{{"type": "nav", "place": "충전"}}]}}
                CORRECT: {{"steps": [{{"type": "nav", "place": "CHRG001"}}]}}

                Input: 지게발 내려줘
                WRONG: {{"steps": [{{"type": "nav", "place": "UNLD001"}}, {{"type": "fork", "action": "DOWN"}}]}}
                CORRECT: {{"steps": [{{"type": "fork", "action": "DOWN"}}]}}

                Input: 지게발 올려줘
                WRONG: {{"steps": [{{"type": "nav", "place": "LOAD001"}}, {{"type": "fork", "action": "UP"}}]}}
                CORRECT: {{"steps": [{{"type": "fork", "action": "UP"}}]}}

                Input: 지게발 올려줘
                WRONG: {{"steps": [{{"type": "nav", "place": "CHRG001"}}, {{"type": "fork", "action": "UP"}}]}}
                CORRECT: {{"steps": [{{"type": "fork", "action": "UP"}}]}}

                Input: 지게발 내려줘
                WRONG: {{"steps": [{{"type": "nav", "place": "UNLD001"}}, {{"type": "fork", "action": "DOWN"}}]}}
                CORRECT: {{"steps": [{{"type": "fork", "action": "DOWN"}}]}}

                Input: {user_text}
                Output:
            """

            response = requests.post(self.url, json={
                "model": self.model,
                "prompt": prompt,
                "stream": False,
                "format": "json",
                "options": {
                    "temperature": 0,
                    "num_predict": 300,
                }
            })
            return json.loads(response.json()['response'])
        except Exception as e:
            print(f"LLM Error: {e}")
            return None


    def get_coords_from_db(self, place_name: str):
        try:
            keyword_map = {
                '충전':  'CHRG001',
                '상차1': 'LOAD001',
                '상차2': 'LOAD002',
                '상차3': 'LOAD003',
                '경유1': 'NODE001',
                '경유2': 'NODE002',
                '경유3': 'NODE003',
                '경유4': 'NODE004',
                '하차1': 'UNLD001',
                '하차2': 'UNLD002',
                '하차3': 'UNLD003',
                '대기':  'WAIT001',
            }

            target_id = keyword_map.get(place_name, place_name.upper().strip())
            print(target_id)
            active_map = map_service.get_active_map()
            if not active_map:
                print("활성화된 맵이 없습니다.")
                return None

            nodes = self.node_service.get_nodes({'map_id': active_map['map_seq']})
            for node in nodes:
                if node['node_id'] == target_id:
                    return {
                        'x':   float(node['node_x_coord']),
                        'y':   float(node['node_y_coord']),
                        'yaw': 0.0
                    }

            print(f"'{place_name}' → '{target_id}' DB에서 찾지 못했습니다.")
            return None

        except Exception as e:
            print(f"DB 조회 오류: {e}")
            return None

    def _wait_until_arrived(self, target_x, target_y, tolerance=0.3, timeout=60):
        """
        목표 위치에 도달할 때까지 대기
        tolerance: 도착 판정 거리 (미터)
        timeout: 최대 대기 시간 (초)
        """
        start = time.time()
        while time.time() - start < timeout:
            if self.current_pose is not None:
                dx = self.current_pose[0] - target_x
                dy = self.current_pose[1] - target_y
                dist = math.sqrt(dx*dx + dy*dy)
                if dist < tolerance:
                    return True
            time.sleep(0.5)
        return False

    def publish_llm_goal(self, x, y, yaw=0.0):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)

        self.llm_goal_pub.publish(msg)

    def execute_robot_sequence(self, steps, socketio):
        print("================== execute_robot_sequence start ==================")
        msg = Twist()
        PUBLISH_RATE = 0.05

        for step in steps:
            if step['type'] == 'rotate':
                socketio.emit('chat_response', {'data': f"{step['dir']} 방향으로 회전 중..."})

                angular_speed = 0.3
                calibration = 2.0
                duration = (step['angle'] / angular_speed) * calibration
                msg.linear.x = 0.0
                msg.angular.z = -angular_speed if step['dir'] == 'right' else angular_speed

                start = time.time()
                while time.time() - start < duration:
                    self.cmd_pub.publish(msg)
                    time.sleep(PUBLISH_RATE)

                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.cmd_pub.publish(msg)
                time.sleep(0.5)

            elif step['type'] == 'move':
                direction = step['dir']
                socketio.emit('chat_response', {
                    'data': f"{step['val']}m {'전진' if direction == 'forward' else '후진'} 중..."
                })

                linear_speed = 0.2
                duration = step['val'] / linear_speed
                msg.angular.z = 0.0
                msg.linear.x = linear_speed if direction == 'forward' else -linear_speed

                start = time.time()
                while time.time() - start < duration:
                    self.cmd_pub.publish(msg)
                    time.sleep(PUBLISH_RATE)

                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.cmd_pub.publish(msg)
                time.sleep(0.5)

            elif step['type'] == 'nav':
                place_name = step['place']
                socketio.emit('chat_response', {'data': f"'{place_name}' 위치 검색 중..."})

                coords = self.get_coords_from_db(place_name)
                if coords is None:
                    socketio.emit('chat_response', {
                        'data': f"'{place_name}' 을(를) DB에서 찾을 수 없습니다."
                    })
                    continue

                socketio.emit('chat_response', {
                    'data': f"'{place_name}' 으로 이동 중... (x={coords['x']:.2f}, y={coords['y']:.2f})"
                })

                self.publish_llm_goal(coords['x'], coords['y'], coords['yaw'])
                
                # 다음 스텝 유무에 따른 대기
                current_idx = steps.index(step)
                has_next_step = current_idx < len(steps) - 1

                if has_next_step:
                    socketio.emit('chat_response', {'data': "목적지 도착 대기 중..."})
                    arrived = self._wait_until_arrived(
                        coords['x'], coords['y'],
                        tolerance=0.3,
                        timeout=60
                    )
                    if arrived:
                        socketio.emit('chat_response', {'data': f"'{place_name}' 도착 완료! 다음 동작 수행합니다."})
                    else:
                        socketio.emit('chat_response', {'data': f"'{place_name}' 도착 시간 초과. 다음 동작을 수행합니다."})

            elif step['type'] == 'fork':
                action = step['action']
                socketio.emit('chat_response', {
                    'data': f"지게발 {'올리는' if action == 'UP' else '내리는'} 중..."
                })

                fork_msg = String()
                fork_msg.data = action
                self.fork_pub.publish(fork_msg)
                time.sleep(2.0)
                socketio.emit('chat_response', {
                    'data': f"지게발 {'올리기' if action == 'UP' else '내리기'} 완료"
                })

        socketio.emit('chat_response', {'data': "명령 수행 완료 !"})
