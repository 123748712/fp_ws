import requests
import json
import time
from rclpy.node import Node 
from geometry_msgs.msg import Twist, PoseStamped
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
        
        self.url = "http://localhost:11434/api/generate"
        self.model = model
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.llm_goal_pub = self.create_publisher(PoseStamped, '/llm_goal', 10)
        self.fork_pub = self.create_publisher(String, '/fork_cmd', 10)
        self.node_service = NodeService()
    
    def get_robot_command(self, user_text):
        try:
            prompt = f"""You are a robot motion command parser.
                Your only job is to convert Korean natural language into a JSON motion sequence.
                Do not include any explanation. Output JSON only.

                ## Output Format
                {{"steps": [...]}}

                ## Step Types

                ### 1. move
                Direct driving command. No rotation involved.
                {{"type": "move", "dir": "forward" | "backward", "val": <meters>}}

                ### 2. rotate
                Rotate in place. Always followed by a forward move step.
                {{"type": "rotate", "dir": "left" | "right", "angle": 1.57}}

                ### 3. nav
                Used ONLY when a specific place name or node destination is mentioned.
                {{"type": "nav", "place": "<node_id>"}}

                ## Available Nodes (place → node_id)
                | 키워드                        | node_id  |
                |-------------------------------|----------|
                | 충전, 충전소, 충전 노드         | CHRG001  |
                | 상차1, 상차 1번                | LOAD001  |
                | 상차2, 상차 2번                | LOAD002  |
                | 상차3, 상차 3번                | LOAD003  |
                | 경유1, 경유 1번, 1번 노드       | NODE001  |
                | 경유2, 경유 2번, 2번 노드       | NODE002  |
                | 경유3, 경유 3번, 3번 노드       | NODE003  |
                | 경유4, 경유 4번, 4번 노드       | NODE004  |
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

                ## Anti-Examples (never do this)
                Input: 뒤로 1미터
                WRONG: {{"steps": [{{"type": "rotate", "dir": "left", "angle": 1.57}}, {{"type": "move", "dir": "backward", "val": 1.0}}]}}
                CORRECT: {{"steps": [{{"type": "move", "dir": "backward", "val": 1.0}}]}}

                Input: 충전소로 가줘
                WRONG: {{"steps": [{{"type": "nav", "place": "충전"}}]}}
                CORRECT: {{"steps": [{{"type": "nav", "place": "CHRG001"}}]}}

                Input: {user_text}
                Output:"""

            response = requests.post(self.url, json={
                "model": self.model,
                "prompt": prompt,
                "stream": False,
                "format": "json",
                "options": {
                    "temperature": 0,
                    "num_predict": 200,
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
            # ── 회전 (직접 cmd_vel) ──
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

            # ── 직선 이동 (직접 cmd_vel) ──
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

            # ── 노드 주행 (A* + Pure Pursuit) ──
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
                continue
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