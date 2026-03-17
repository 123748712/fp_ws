'''
센서 아두이노 

주행 중일때 D or d 신호 보내고
상~하차중 일때 W or w 신호 
작업이 끝났을때 F or f 신호
비상상황이 발생했을때 E or e 신호
상황 해결했을때 R or r 신호
주행이랑 상하차 부분 분리 
상하차중에도 비상상황 적용
아두이노 버튼 눌렀을때 E 신호 들어옴 이거 맞춰서 비상상황 으로 전환 
웹에서 비상 떳을때 해결 버튼 누르면 신호 받고 아두이노에 R or r 신호 보내줘야함 
작업 종료 버튼은 아마 내려 놓고 돌아갈때 쯤 종료 시킬것이니 그때쯤 인식하게 할지?

상하차 아두이노

상하차는 카메라로 제품 가운데 정렬만 하고 3cm정도 이동후 지게팔 아두이노 쪽에 신호 보낼 것
상차 성공시 팔을 들고 있다 하차 지역에 도착시 팔을 내리고 하차 구역을 찾아 제품 하차 
하차 후 팔을 내리고 상차 지역으로 이동 카메라로 하차 물품 수 확인하고 특정 갯수 이상이면 루프문 끊고 대기장소로 
대기장소에서 제품신호 받으면 다시 이동 

주행 
주행을 제품 3개에 따라 다르게 설정해야함 

'''

import serial
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
from math import pow, atan2, sqrt, sin, cos, pi
import heapq
import numpy as np
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# ser1 = serial.Serial('/dev/ttyACM1', 9600)
# ser2 = serial.Serial('/dev/ttyACM2', 9600)
# time.sleep(2) # 아두이노 리셋 대기

class NodeAStar:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0
    def __eq__(self, other): return self.position == other.position
    def __lt__(self, other): return self.f < other.f

class IntegratedNavigation(Node):
    def __init__(self):
        super().__init__('integrated_navigation')
        
        self.ser = serial.Serial('/dev/ttyACM1', 9600, timeout=0.01) # 타임아웃 짧게 설정
        self.is_emergency_paused = False

        qos_profile = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        map_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # 주행 설정
        self.lookahead_dist = 0.45  
        self.linear_vel = 0.125
        self.stop_tolerance = 0.15
        
        # [미션 좌표 설정]

        # test
        # 0.61, 0.978
        # 2.74, 0.798
        # 1.54 , -0.00363
        
        self.rooms = {
            'right': [0.61, 0.978],
            'left': [2.74, 0.798],
            'standby': [1.54 , -0.00363]
        }

        # [미션 시퀀스 생성]
        # 시작(오른쪽) -> (왼쪽->오른쪽) x 5번 -> 대기실
        self.mission_sequence = []
        self.mission_sequence.append(self.rooms['right']) # 첫 시작은 오른쪽 방
        for _ in range(1):
            self.mission_sequence.append(self.rooms['left'])
            self.mission_sequence.append(self.rooms['right'])
        self.mission_sequence.append(self.rooms['standby']) # 마지막 대기실

        self.mission_idx = 0
        self.is_mission_active = False # RViz 클릭 전까지 대기
        
        # 상태 관리 변수
        self.map_data = None
        self.map_resolution = 0.05
        self.map_origin = [0.0, 0.0]
        self.map_width = 0
        self.map_height = 0
        
        self.current_pose = None
        self.current_yaw = 0.0
        self.global_path = [] 
        self.path_index = 0
        self.is_initial_turn = False
        self.scan_data = None

        # 통신 설정
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_path = self.create_publisher(Path, '/planned_path', 10)
        self.sub_map = self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos_profile)
        self.sub_pose = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.sub_goal = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.scan_subscription = self.create_subscription(LaserScan, "/scan", self.scan_callback, qos_profile)

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Ready for Mission! Click '2D Nav Goal' in RViz to start.")

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        ranges = np.where(np.isinf(ranges), 3.5, ranges)
        ranges = np.where(np.isnan(ranges), 3.5, ranges)
        self.scan_data = ranges 

    def map_callback(self, msg):
        self.map_resolution = msg.info.resolution
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_origin = [msg.info.origin.position.x, msg.info.origin.position.y]
        self.map_data = np.array(msg.data).reshape((self.map_height, self.map_width))

    def pose_callback(self, msg):
        self.current_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        q = msg.pose.pose.orientation
        self.current_yaw = atan2(2.0*(q.w*q.z + q.x*q.y), 1.0-2.0*(q.y*q.y + q.z*q.z))

    def goal_callback(self, msg):
        if self.is_mission_active:
            self.get_logger().warn("Mission is already in progress!")
            return

        if self.map_data is None or self.current_pose is None:
            self.get_logger().error("Map or Pose not ready!")
            return

        self.get_logger().info("🚀 Mission Trigger Received! Starting 5-Lap Routine.")
        
        # [수정] 아두이노에 주행 시작 신호 전송
        try:
            self.ser.write(b'D\n') 
            self.get_logger().info("Sent 'D' to Arduino: LED Green On")
        except Exception as e:
            self.get_logger().error(f"Serial Write Error: {e}")

        self.is_mission_active = True
        self.mission_idx = 0
        
        # 미션의 첫 번째 목적지로 출발
        self.set_next_destination(self.mission_sequence[self.mission_idx])
        
    def set_next_destination(self, goal_pose):
        self.get_logger().info(f"📍 New Target: {goal_pose}")
        
        start_grid = self.world_to_grid(self.current_pose)
        goal_grid = self.world_to_grid(goal_pose)
        
        self.get_logger().info("Calculating Path...")
        path_grid = self.run_astar(start_grid, goal_grid)
        
        if path_grid:
            self.global_path = [self.grid_to_world(p) for p in path_grid]
            self.path_index = 0
            self.is_initial_turn = True
            self.publish_path_viz()
            self.get_logger().info(f"Path Found! Moving to target {self.mission_idx + 1}/{len(self.mission_sequence)}")
        else:
            self.get_logger().error("❌ No Path Found to next goal!")
            self.is_mission_active = False # 경로 실패 시 미션 중단

    def run_astar(self, start, end):
        if self.map_data is None: return None
        if not (0 <= start[0] < self.map_height and 0 <= start[1] < self.map_width): return None
        if not (0 <= end[0] < self.map_height and 0 <= end[1] < self.map_width): return None

        start_node = NodeAStar(None, start)
        end_node = NodeAStar(None, end)
        open_list = []
        heapq.heappush(open_list, start_node)
        visited = set()
        
        # 이동 비용 (대각선 포함)
        moves = [(0,1,1.0), (0,-1,1.0), (1,0,1.0), (-1,0,1.0), (1,1,1.41), (1,-1,1.41), (-1,1,1.41), (-1,-1,1.41)]

        # [핵심 수정] 좁은 길 통과를 위해 마진 최소화
        safety_margin = 2    # 1칸(5cm)까지는 허용 (벽에 거의 붙어서라도 지나감)
        preferred_margin = 8 # 3칸(15cm) 이내일 때만 페널티 부여
        obstacle_threshold = 80 # 장애물 인식 문턱값을 높여 흐릿한 선은 무시

        while open_list:
            current_node = heapq.heappop(open_list)
            if current_node.position in visited: continue
            visited.add(current_node.position)

            if current_node.position == end_node.position:
                path = []
                current = current_node
                while current:
                    path.append(current.position)
                    current = current.parent
                
                # 경로 단순화 (3칸 간격)
                full_path = path[::-1]
                smoothed_path = full_path[::3] 
                if full_path[-1] not in smoothed_path:
                    smoothed_path.append(full_path[-1])
                return smoothed_path

            for dy, dx, move_cost in moves:
                ny, nx = current_node.position[0] + dy, current_node.position[1] + dx
                
                if not (0 <= ny < self.map_height and 0 <= nx < self.map_width): continue
                
                # 1. 장애물 체크 (문턱값 완화)
                if self.map_data[ny][nx] > obstacle_threshold or self.map_data[ny][nx] == -1: continue
                
                # 2. 고속 Inflation 체크
                penalty = 0
                too_close = False
                
                # 십자 검사를 더 짧고 정밀하게 수행
                for r in range(1, preferred_margin + 1):
                    for check_y, check_x in [(ny+r, nx), (ny-r, nx), (ny, nx+r), (ny, nx-r)]:
                        if 0 <= check_y < self.map_height and 0 <= check_x < self.map_width:
                            # 실제 벽(검은 점)인지 확인
                            if self.map_data[check_y][check_x] > obstacle_threshold:
                                if r <= safety_margin:
                                    too_close = True
                                    break
                                penalty += (preferred_margin - r) * 15
                    if too_close: break
                
                if too_close: continue

                new_node = NodeAStar(current_node, (ny, nx))
                # 페널티 비중을 낮춰서 좁은 길도 '길'로 인식하게 함
                new_node.g = current_node.g + move_cost + penalty
                new_node.h = sqrt((ny - end[0])**2 + (nx - end[1])**2)
                new_node.f = new_node.g + new_node.h
                heapq.heappush(open_list, new_node)
                
        return None

    def control_loop(self):
        # [0] 아두이노로부터 신호 읽기 (매 루프마다 확인)
        if self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode().strip()
                if 'E' in line: # 비상 신호 수신 (아두이노 버튼 or 소리)
                    self.is_emergency_paused = True
                    self.get_logger().warn("⚠️ EMERGENCY SIGNAL RECEIVED!")
                elif 'R' in line: # 해결 신호 수신 (웹/시리얼 R 입력)
                    if self.is_emergency_paused:
                        self.is_emergency_paused = False
                        self.is_initial_turn = True # 해제 후 정렬 다시 수행
                        self.ser.write(b'D\n') # 아두이노를 다시 주행 모드로 변경
                        self.get_logger().info("✅ EMERGENCY RESOLVED. Resuming...")
            except Exception as e:
                self.get_logger().error(f"Serial Read Error: {e}")

        # [1] 비상 정지 중이면 로봇을 멈추고 루프 탈출 (나머지 로직 실행 안함)
        if self.is_emergency_paused:
            self.stop_robot()
            return

        # [2] 데이터 유효성 체크
        if self.scan_data is None or not self.global_path or self.current_pose is None:
            return

        # [3, 4] 데이터 전처리 (기존과 동일)
        ranges = np.array(self.scan_data)
        ranges[np.isnan(ranges) | np.isinf(ranges)] = 3.5
        ranges[ranges < 0.15] = 3.5

        num_points = len(ranges)
        idx_per_degree = num_points / 360.0
        f_idx = int(40 * idx_per_degree)
        front_ranges = np.concatenate((ranges[:f_idx], ranges[-f_idx:]))
        left_ranges = ranges[int(45 * idx_per_degree):int(120 * idx_per_degree)]
        right_ranges = ranges[int(240 * idx_per_degree):int(315 * idx_per_degree)]
        back_ranges = ranges[int(150 * idx_per_degree):int(210 * idx_per_degree)]
        f_dist, l_dist, r_dist, b_dist = np.min(front_ranges), np.min(left_ranges), np.min(right_ranges), np.min(back_ranges)

        # [5] 도착 판정 및 미션 관리
        final_goal = self.global_path[-1]
        dist_to_goal = sqrt((final_goal[0] - self.current_pose[0])**2 + (final_goal[1] - self.current_pose[1])**2)
        
        if dist_to_goal < self.stop_tolerance:
            self.stop_robot()
            self.get_logger().info(f"✅ Reached Goal {self.mission_idx + 1}")
            
            if self.is_mission_active:
                self.mission_idx += 1
                if self.mission_idx < len(self.mission_sequence):
                    # 다음 미션지로 자동 전환
                    self.set_next_destination(self.mission_sequence[self.mission_idx])
                    return 
                else:
                    self.get_logger().info("🏁 MISSION COMPLETE! All laps done.")
                    self.is_mission_active = False
                    # [수정] 모든 주행 종료 시 아두이노에 작업 종료 신호 전송
                    self.ser.write(b'F\n') 
            
            self.stop_robot()
            self.global_path = []
            return

        # [6] 초기 제자리 회전 (기존과 동일)
        if self.is_initial_turn:
            target_idx = min(len(self.global_path) - 1, 5)
            target_x, target_y = self.global_path[target_idx]
            alpha_init = atan2(target_y - self.current_pose[1], target_x - self.current_pose[0]) - self.current_yaw
            while alpha_init > pi: alpha_init -= 2*pi
            while alpha_init < -pi: alpha_init += 2*pi

            turn_margin = 0.25 
            cmd = Twist()

            if (f_dist < turn_margin) or (alpha_init > 0 and l_dist < turn_margin) or (alpha_init < 0 and r_dist < turn_margin):
                if b_dist > 0.3:
                    cmd.linear.x = -0.1
                else:
                    self.stop_robot()
                self.pub_cmd.publish(cmd)
                return 

            elif b_dist < turn_margin:
                if f_dist > 0.3:
                    cmd.linear.x = 0.1
                else:
                    self.stop_robot()
                self.pub_cmd.publish(cmd)
                return

            if abs(alpha_init) > 0.15:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.4 if alpha_init > 0 else -0.4
                self.pub_cmd.publish(cmd)
                return
            else:
                self.get_logger().info("★★★ Alignment Done! ★★★")
                self.is_initial_turn = False

        # [7] Pure Pursuit 조향각 계산 (기존과 동일)
        min_dist_to_path = float('inf')
        target_x, target_y = self.global_path[-1]
        for i in range(self.path_index, len(self.global_path)):
            px, py = self.global_path[i]
            dist = sqrt((px - self.current_pose[0])**2 + (py - self.current_pose[1])**2)
            if dist < min_dist_to_path: min_dist_to_path = dist
            if dist >= self.lookahead_dist:
                target_x, target_y = px, py
                self.path_index = i
                break

        alpha = atan2(target_y - self.current_pose[1], target_x - self.current_pose[0]) - self.current_yaw
        while alpha > pi: alpha -= 2*pi
        while alpha < -pi: alpha += 2*pi

        # [8] 상태 결정 (기존과 동일)
        safe_dist = 0.37
        emergency_dist = 0.18 
        cmd = Twist()

        if f_dist < emergency_dist or l_dist < 0.13 or r_dist < 0.13:
            action = "stop"
        elif f_dist < 0.22:
            action = "go_back"
        elif f_dist < safe_dist:
            action = "turn_avoid"
            cmd.linear.x = 0.06
            cmd.angular.z = 0.7 if l_dist >= r_dist else -0.7
        else:
            action = "go_forward"

        # [9] 동작 수행 (기존과 동일)
        if action == "go_forward":
            current_speed = self.linear_vel
            if l_dist < 0.35 or r_dist < 0.35:
                current_speed *= 0.8 
            cmd.linear.x = current_speed
            steering = (1.8 * sin(alpha)) / self.lookahead_dist
            margin = 0.35 
            if l_dist < margin:
                steering -= 0.6 * (margin - l_dist)  
            if r_dist < margin:
                steering += 0.6 * (margin - r_dist)
            cmd.angular.z = cmd.linear.x * steering

        elif action == "go_back":
            cmd.linear.x = -0.15 if b_dist > 0.2 else 0.0
            cmd.angular.z = 0.0

        elif action == "stop":
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.pub_cmd.publish(cmd)
        self.get_logger().info(f"ACT: {action} | F:{f_dist:.2f} L:{l_dist:.2f} R:{r_dist:.2f} Ang:{cmd.angular.z:.2f}")

        
    def world_to_grid(self, world):
        return (int((world[1]-self.map_origin[1])/self.map_resolution), int((world[0]-self.map_origin[0])/self.map_resolution))

    def grid_to_world(self, grid):
        return [(grid[1]*self.map_resolution)+self.map_origin[0], (grid[0]*self.map_resolution)+self.map_origin[1]]

    def publish_path_viz(self):
        msg = Path()
        msg.header.frame_id = 'map'
        for p in self.global_path:
            ps = PoseStamped()
            ps.pose.position.x, ps.pose.position.y = p[0], p[1]
            msg.poses.append(ps)
        self.pub_path.publish(msg)

    def stop_robot(self):
        self.pub_cmd.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = IntegratedNavigation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
