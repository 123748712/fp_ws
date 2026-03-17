import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import String, Bool, Int32
from math import atan2, sqrt, sin, pi
import heapq
import numpy as np
from sensor_msgs.msg import LaserScan, CompressedImage
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
import threading
import sys
import tty
import termios
import os
import cv2
#carry_done , ros2 topic pub --once /carry std_msgs/msg/Int32 "{data: 0}" 0,1,2
sys.path.append('/home/dev/fp_ws/src/fp_pkg')
from database.node_service import NodeService
from database.map_service import MapService

class NodeAStar:
    def __init__(self, parent=None, position=None):
        self.parent   = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0
    def __eq__(self, other): return self.position == other.position
    def __lt__(self, other): return self.f < other.f

class NavigationDBHelper:
    def __init__(self):
        self.node_service = NodeService()
        self.map_service  = MapService()

    def get_active_map_id(self):
        result = self.map_service.get_active_map()
        if result:
            return result['map_seq']
        return None

    def load_rooms(self, zone: int):
        map_id = self.get_active_map_id()
        if map_id is None:
            raise RuntimeError("활성화된 맵이 없습니다!")

        nodes = self.node_service.get_nodes({'map_id': map_id})
        rooms = {}
        for node in nodes:
            node_id = node['node_id']
            x = float(node['node_x_coord'])
            y = float(node['node_y_coord'])

            # ★ left→load, right→unload 로 변경
            if node_id == f'LOAD00{zone}': rooms[f'load_{zone}']   = [x, y]
            if node_id == f'UNLD00{zone}': rooms[f'unload_{zone}'] = [x, y]
            if node_id == 'WAIT001':       rooms['standby']         = [x, y]

            ## 예시
            # waypoints = []
            # if node_id.startswith("NODE"): # NODE~로 시작하면 경유노드
            #     waypoint = {
            #         'node_id' : node.node_id,
            #         'node_x' : node.node_x_coord,
            #         'node_y' : node.node_y_coord
            #     }
            #     waypoints.push(waypoint)

        if f'load_{zone}'   not in rooms: raise RuntimeError(f"LOAD00{zone} 노드가 DB에 없습니다!")
        if f'unload_{zone}' not in rooms: raise RuntimeError(f"UNLD00{zone} 노드가 DB에 없습니다!")
        if 'standby'        not in rooms: raise RuntimeError("WAIT001 노드가 DB에 없습니다!")

        return rooms, map_id

class IntegratedNavigation(Node):
    def __init__(self):
        super().__init__('integrated_navigation')

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

        camera_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ── 대기 관련 변수 ──
        self.is_waiting        = False
        self.wait_duration     = 3.0
        self.wait_start_time   = None
        self.current_room_type = None

        # 비상정지 플래그
        self.is_emergency_paused = False

        # ── 주행 설정 ──
        self.lookahead_dist = 0.5
        self.linear_vel     = 0.15
        self.stop_tolerance = 0.25

        # ── rooms 초기화 (실제 로드는 키 입력 시 trigger_mission에서) ──
        self.rooms              = {}
        self.current_load_key   = 'load_1'    # ★ left_key → load_key
        self.current_unload_key = 'unload_1'  # ★ right_key → unload_key
        self.mission_sequence   = []
        self.mission_idx        = 0
        self.is_mission_active  = False

        # ── 상태 관리 변수 ──
        self.map_data        = None
        self.map_resolution  = 0.05
        self.map_origin      = [0.0, 0.0]
        self.map_width       = 0
        self.map_height      = 0
        self.current_pose    = None
        self.current_yaw     = 0.0
        self.global_path     = []
        self.path_index      = 0
        self.is_initial_turn = False
        self.scan_data       = None

        # ── 라인 정렬 관련 변수 ──
        self.is_line_aligning     = False
        self.line_align_phase     = 0
        self.line_cx_offset       = None
        self.line_bottom_ratio    = None
        self.line_near_end_offset = None
        self.line_angle           = None
        self.image_width          = 640
        self.image_height         = 480
        self.line_lost_count      = 0
        self.LINE_LOST_THRESHOLD  = 5
        self.last_align_cmd_az = 0.0   # 이전 angular.z 저장
        self.CMD_SMOOTH_ALPHA  = 0.4   # 낮을수록 부드러움 (0.3~0.5)

        # ── 통신 설정 ──
        self.pub_cmd  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_path = self.create_publisher(Path,  '/planned_path', 10)

        self.pub_forklift_cmd  = self.create_publisher(String, '/forklift_cmd',  10)
        self.pub_mission_start = self.create_publisher(Bool,   '/mission_start', 10)
        self.pub_mission_done  = self.create_publisher(Bool,   '/mission_done',  10)
        self.pub_emergency_resolve = self.create_publisher(Bool, '/emergency_resolve', 10)
        self.pub_carry = self.create_publisher(Int32, '/carry_done_ack', 10)  # 필요시
        # 디버그용
        self.pub_debug_image = self.create_publisher(CompressedImage, '/debug/line_image/compressed', 10)

        self.sub_map  = self.create_subscription(OccupancyGrid,            '/map',       self.map_callback,  map_qos_profile)
        self.sub_pose = self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose', self.pose_callback, 10)
        self.scan_subscription = self.create_subscription(LaserScan,       '/scan',      self.scan_callback, qos_profile)
        self.camera_sub = self.create_subscription(CompressedImage, '/image_raw/compressed', self.camera_callback, camera_qos)

        self.sub_emergency     = self.create_subscription(
            Bool, '/emergency_stop', self.emergency_callback, 10)
        self.sub_forklift_done = self.create_subscription(
            Bool, '/forklift_done',  self.forklift_done_callback, 10)
        self.sub_trigger = self.create_subscription(
            Int32, '/carry', self.trigger_callback, 10)
        self.sub_carry_done = self.create_subscription(
            Bool, '/carry_done', self.carry_done_callback, 10)
        
        self.current_zone = 1

        self.timer = self.create_timer(0.1, self.control_loop)

        # ── 키보드 입력 스레드 ──
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.keyboard_thread.start()

        self.get_logger().info("🎮 Ready! '1~3' = 구역 선택 | 'S' = 강제 중지 | 'O' = 비상정지 해제")

    # 비상정지 콜백 (파이 → 리모트PC)
    def emergency_callback(self, msg):
        if msg.data:
            self.is_emergency_paused = True
            self.stop_robot()
            self.get_logger().warn("⚠️  비상정지 신호 수신! 로봇 정지.")
        else:
            self.is_emergency_paused = False
            self.is_initial_turn     = True
            self.get_logger().info("✅ 비상정지 해제. 주행 재개합니다.")

    # 작업 완료 콜백 (파이 → 리모트PC)
    # 수정
    def forklift_done_callback(self, msg):
        if msg.data and self.is_waiting and self.current_room_type != 'load':
            self.is_waiting = False
            self.get_logger().info("✅ 작업 완료 신호 수신! 다음 목적지로 출발합니다.")
            if self.is_mission_active:
                self.mission_idx += 1
                if self.mission_idx < len(self.mission_sequence):
                    self.set_next_destination(self.mission_sequence[self.mission_idx])
                else:
                    self.get_logger().info("🏁 MISSION COMPLETE!")
                    self.is_mission_active = False
                    done_msg = Bool()
                    done_msg.data = True
                    self.pub_mission_done.publish(done_msg)
                    self.global_path = []  # ← 미션 완료 시에만 초기화

    # 키보드 리스너
    def keyboard_listener(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while True:
                ch = sys.stdin.read(1)
                if ch == '1':
                    self.trigger_mission(zone=1)
                elif ch == '2':
                    self.trigger_mission(zone=2)
                elif ch == '3':
                    self.trigger_mission(zone=3)
                elif ch in ('s', 'S'):
                    self.get_logger().info("🛑 강제 중지 명령 수신.")
                    self.is_mission_active = False
                    self.is_waiting        = False
                    self.global_path       = []
                    self.stop_robot()
                    done_msg = Bool()
                    done_msg.data = True
                    self.pub_mission_done.publish(done_msg)
                    
                # ★ 추가: O키로 비상정지 해제
                elif ch in ('o', 'O'):
                    if self.is_emergency_paused:
                        self.is_emergency_paused = False
                        self.is_initial_turn     = True
                        # ★ 파이로 해제 신호 전송 → 아두이노 'R'
                        resolve_msg = Bool()
                        resolve_msg.data = True
                        self.pub_emergency_resolve.publish(resolve_msg)
                        self.get_logger().info("✅ [O키] 비상정지 해제. 주행 재개합니다.")
                    else:
                        self.get_logger().info("ℹ️  현재 비상정지 상태가 아닙니다.")

                elif ch == '\x03':
                    break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def trigger_callback(self, msg):
        zone_map = {0: 1, 1: 2, 2: 3}   # 0→1번구역, 1→2번구역, 2→3번구역
        target = msg.data

        if target not in zone_map:
            self.get_logger().error(f"❌ /carry 유효하지 않은 값: {target}. 0, 1, 2만 사용 가능.")
            return

        zone = zone_map[target]
        self.get_logger().info(f"🔔 /carry 수신: {target} → {zone}번 구역 미션 시작")
        self.trigger_mission(zone=zone)

    def trigger_mission(self, zone: int):
        if self.is_mission_active:
            self.get_logger().warn("⚠️  미션이 이미 진행 중입니다!")
            return
        if self.map_data is None or self.current_pose is None:
            self.get_logger().error("❌ 맵 또는 위치 정보가 준비되지 않았습니다!")
            return

        self.current_zone       = zone
        self.current_load_key   = f'load_{zone}'

        try:
            nav_db = NavigationDBHelper()
            self.rooms, active_map_id = nav_db.load_rooms(zone)
            self.get_logger().info(
                f"✅ [{zone}번 구역] DB 노드 로드 완료 (map_id={active_map_id})\n"
                f"   상차(LOAD): {self.rooms[f'load_{zone}']}\n"
                f"   하차(UNLOAD): {self.rooms[f'unload_{zone}']}\n"
                f"   대기: {self.rooms['standby']}"
            )
        except Exception as e:
            self.get_logger().error(f"❌ [{zone}번 구역] DB 노드 로드 실패: {e}")
            return

        self.current_load_key   = f'load_{zone}'
        self.current_unload_key = f'unload_{zone}'

        # ★ 상차(load) 먼저 → 하차(unload) 반복
        self.mission_sequence = []
        for _ in range(5):
            self.mission_sequence.append(self.rooms[self.current_load_key])    # 상차 먼저
            self.mission_sequence.append(self.rooms[self.current_unload_key])  # 하차
        self.mission_sequence.append(self.rooms['standby'])

        self.get_logger().info(f"🚀 미션 시작! [{zone}번 구역] 상차→하차 5회 왕복을 시작합니다.")

        start_msg = Bool()
        start_msg.data = True
        self.pub_mission_start.publish(start_msg)

        self.is_mission_active = True
        self.mission_idx       = 0
        self.set_next_destination(self.mission_sequence[self.mission_idx])

    # 콜백
    def carry_done_callback(self, msg):
        if msg.data and self.is_waiting and self.current_room_type == 'load':
            self.is_waiting = False
            self.get_logger().info("✅ 상차 완료 신호 수신! 다음 목적지로 출발합니다.")
            if self.is_mission_active:
                self.mission_idx += 1
                if self.mission_idx < len(self.mission_sequence):
                    self.set_next_destination(self.mission_sequence[self.mission_idx])
                else:
                    self.get_logger().info("🏁 MISSION COMPLETE!")
                    self.is_mission_active = False
                    done_msg = Bool()
                    done_msg.data = True
                    self.pub_mission_done.publish(done_msg)
                    self.global_path = []

    def camera_callback(self, msg):
        if not self.is_line_aligning:
            return

        np_arr = np.frombuffer(msg.data, np.uint8)
        frame  = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return

        h, w = frame.shape[:2]
        self.image_width  = w
        self.image_height = h

        # ★ ROI: 화면 하단 60%만 검사 (천장/벽 노이즈 원천 차단)
        roi_top  = int(h * 0.4)
        roi      = frame[roi_top:h, :]
        roi_hsv  = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # ★ HSV 범위 재조정: S 하한 올리고 H 좁힘
        lower = np.array([100, 80, 80])
        upper = np.array([130, 255, 255])
        mask  = cv2.inRange(roi_hsv, lower, upper)

        kernel = np.ones((5, 5), np.uint8)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # ── 디버그 이미지 (전체 프레임 기준) ──
        debug_frame = frame.copy()
        mask_full   = np.zeros((h, w), dtype=np.uint8)
        mask_full[roi_top:h, :] = mask          # ROI 위치에 맞게 복원

        mask_colored = cv2.cvtColor(mask_full, cv2.COLOR_GRAY2BGR)
        mask_colored[mask_full > 0] = [0, 255, 0]
        debug_frame = cv2.addWeighted(debug_frame, 0.7, mask_colored, 0.3, 0)

        # ROI 경계선 표시
        cv2.line(debug_frame, (0, roi_top), (w, roi_top), (255, 255, 0), 1)
        # 화면 중앙선
        cv2.line(debug_frame, (w//2, 0), (w//2, h), (0, 255, 255), 1)

        center_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)[h//2, w//2]
        cv2.putText(debug_frame, f"Center HSV: {center_hsv}",
                    (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        self.line_cx_offset       = None
        self.line_bottom_ratio    = None
        self.line_near_end_offset = None
        self.line_angle           = None

        if not contours:
            # valid_contours 없는 경우와 동일하게 처리
            self.line_lost_count += 1
            if self.line_lost_count >= self.LINE_LOST_THRESHOLD:
                cv2.putText(debug_frame, f"NO CONTOUR LOST {self.line_lost_count}f",
                            (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 100, 255), 1)
            else:
                self.line_cx_offset       = getattr(self, '_last_cx_offset',       None)
                self.line_near_end_offset = getattr(self, '_last_near_end_offset', None)
                self.line_angle           = getattr(self, '_last_angle',           None)
                self.line_bottom_ratio    = getattr(self, '_last_bottom_ratio',    None)
            self._publish_debug_image(debug_frame)
            return

        # ★ 면적 + 종횡비로 라인 후보만 필터링
        valid_contours = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 50:          # ★ 300 → 100 (멀리 있는 작은 라인도 감지)
                continue
            rx, ry, rw, rh = cv2.boundingRect(cnt)
            aspect = max(rw, rh) / (min(rw, rh) + 1e-5)
            if aspect < 1.0:        # ★ 1.5 → 1.2 (가늘고 긴 형태 기준 완화)
                continue
            valid_contours.append(cnt)

        if not valid_contours:
            self.line_lost_count += 1
            if self.line_lost_count >= self.LINE_LOST_THRESHOLD:
                cv2.putText(debug_frame, f"LOST {self.line_lost_count}f",
                            (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 100, 255), 1)
            else:
                self.line_cx_offset       = getattr(self, '_last_cx_offset',       None)
                self.line_near_end_offset = getattr(self, '_last_near_end_offset', None)
                self.line_angle           = getattr(self, '_last_angle',           None)
                self.line_bottom_ratio    = getattr(self, '_last_bottom_ratio',    None)
            self._publish_debug_image(debug_frame)
            return

        largest = max(valid_contours, key=cv2.contourArea)
        area    = cv2.contourArea(largest)

        # ROI 좌표 → 전체 프레임 좌표로 변환
        rx, ry, rw, rh = cv2.boundingRect(largest)
        ry_full = ry + roi_top   # ★ 전체 프레임 기준 y

        # 컨투어도 y축 보정해서 그리기
        largest_full = largest.copy()
        largest_full[:, :, 1] += roi_top
        cv2.drawContours(debug_frame, [largest_full], -1, (0, 255, 0), 2)
        cv2.rectangle(debug_frame, (rx, ry_full), (rx+rw, ry_full+rh), (255, 0, 0), 2)

        M = cv2.moments(largest)
        if M['m00'] == 0:
            self._publish_debug_image(debug_frame)
            return
        cx       = int(M['m10'] / M['m00'])
        cy_local = int(M['m01'] / M['m00'])
        cy_full  = cy_local + roi_top

        self.line_cx_offset    = cx - (w // 2)
        self.line_bottom_ratio = (ry_full + rh) / h

        cv2.circle(debug_frame, (cx, cy_full), 6, (0, 0, 255), -1)
        cv2.line(debug_frame, (cx, 0), (cx, h), (0, 0, 255), 1)

        line_left_x  = rx
        line_right_x = rx + rw
        center_x     = w // 2

        left_dist  = abs(line_left_x  - center_x)
        right_dist = abs(line_right_x - center_x)
        near_end_x = line_right_x if right_dist >= left_dist else line_left_x
        self.line_near_end_offset = near_end_x - center_x

        cv2.circle(debug_frame, (near_end_x, ry_full + rh//2), 8, (0, 165, 255), -1)

        rect  = cv2.minAreaRect(largest)
        angle = rect[2]
        if rw < rh:
            angle = angle + 90
        self.line_angle = angle

        # 감지 성공 시 카운트 리셋 + 이전 값 저장 (largest 계산 끝난 직후에 추가)
        self.line_lost_count          = 0
        self._last_cx_offset          = self.line_cx_offset
        self._last_near_end_offset    = self.line_near_end_offset
        self._last_angle              = self.line_angle
        self._last_bottom_ratio       = self.line_bottom_ratio

        # 디버그 텍스트
        cv2.putText(debug_frame,
                    f"Phase:{self.line_align_phase}  Area:{area:.0f}",
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(debug_frame,
                    f"CX_offset:{self.line_cx_offset}  bottom:{self.line_bottom_ratio:.2f}",
                    (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(debug_frame,
                    f"end_offset:{self.line_near_end_offset}  angle:{self.line_angle:.1f}deg",
                    (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        self._publish_debug_image(debug_frame)

    def _publish_debug_image(self, frame):
        """디버그 이미지를 CompressedImage 토픽으로 발행 → rqt_image_view로 확인"""
        ret, buf = cv2.imencode('.jpg', frame)
        if not ret:
            return
        debug_msg      = CompressedImage()
        debug_msg.header.stamp  = self.get_clock().now().to_msg()
        debug_msg.format        = 'jpeg'
        debug_msg.data          = buf.tobytes()
        self.pub_debug_image.publish(debug_msg)

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        ranges = np.where(np.isinf(ranges), 3.5, ranges)
        ranges = np.where(np.isnan(ranges), 3.5, ranges)
        self.scan_data = ranges

    def map_callback(self, msg):
        self.map_resolution = msg.info.resolution
        self.map_width      = msg.info.width
        self.map_height     = msg.info.height
        self.map_origin     = [msg.info.origin.position.x, msg.info.origin.position.y]
        self.map_data       = np.array(msg.data).reshape((self.map_height, self.map_width))

    def pose_callback(self, msg):
        self.current_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        q = msg.pose.pose.orientation
        self.current_yaw = atan2(2.0*(q.w*q.z + q.x*q.y), 1.0-2.0*(q.y*q.y + q.z*q.z))

    # 경로 탐색
    def set_next_destination(self, goal_pose):
        self.get_logger().info(f"📍 다음 목표: {goal_pose}")

        drive_msg = Bool()
        drive_msg.data = True
        self.pub_mission_start.publish(drive_msg)

        start_grid = self.world_to_grid(self.current_pose)
        goal_grid  = self.world_to_grid(goal_pose)
        path_grid  = self.run_astar(start_grid, goal_grid)

        if path_grid:
            self.global_path     = [self.grid_to_world(p) for p in path_grid]
            self.path_index      = 0
            self.is_initial_turn = True
            self.publish_path_viz()
            self.get_logger().info(f"경로 탐색 완료! 목표 {self.mission_idx + 1}/{len(self.mission_sequence)}으로 이동 중")
        else:
            self.get_logger().error("❌ 경로를 찾을 수 없습니다!")
            self.is_mission_active = False

    def run_astar(self, start, end):
        if self.map_data is None: return None
        if not (0 <= start[0] < self.map_height and 0 <= start[1] < self.map_width): return None
        if not (0 <= end[0]   < self.map_height and 0 <= end[1]   < self.map_width): return None

        start_node = NodeAStar(None, start)
        end_node   = NodeAStar(None, end)
        open_list  = []
        heapq.heappush(open_list, start_node)
        visited = set()

        moves = [(0,1,1.0),(0,-1,1.0),(1,0,1.0),(-1,0,1.0),
                 (1,1,1.41),(1,-1,1.41),(-1,1,1.41),(-1,-1,1.41)]
        safety_margin      = 2
        preferred_margin   = 8
        obstacle_threshold = 80

        while open_list:
            current_node = heapq.heappop(open_list)
            if current_node.position in visited: continue
            visited.add(current_node.position)

            if current_node.position == end_node.position:
                path = []
                cur  = current_node
                while cur:
                    path.append(cur.position)
                    cur = cur.parent
                full_path     = path[::-1]
                smoothed_path = full_path[::3]
                if full_path[-1] not in smoothed_path:
                    smoothed_path.append(full_path[-1])
                return smoothed_path

            for dy, dx, move_cost in moves:
                ny, nx = current_node.position[0]+dy, current_node.position[1]+dx
                if not (0 <= ny < self.map_height and 0 <= nx < self.map_width): continue
                if self.map_data[ny][nx] > obstacle_threshold or self.map_data[ny][nx] == -1: continue

                penalty   = 0
                too_close = False
                for r in range(1, preferred_margin+1):
                    for cy, cx in [(ny+r,nx),(ny-r,nx),(ny,nx+r),(ny,nx-r)]:
                        if 0 <= cy < self.map_height and 0 <= cx < self.map_width:
                            if self.map_data[cy][cx] > obstacle_threshold:
                                if r <= safety_margin:
                                    too_close = True; break
                                penalty += (preferred_margin - r) * 15
                    if too_close: break
                if too_close: continue

                new_node   = NodeAStar(current_node, (ny, nx))
                new_node.g = current_node.g + move_cost + penalty
                new_node.h = sqrt((ny-end[0])**2 + (nx-end[1])**2)
                new_node.f = new_node.g + new_node.h
                heapq.heappush(open_list, new_node)
        return None

    # 메인 제어 루프
    def control_loop(self):
        if self.is_emergency_paused:
            self.stop_robot()
            return

        if self.scan_data is None or self.current_pose is None:
            return

        # ── 대기 중 처리 ──
        if self.is_waiting:
            self.stop_robot()

            # ★ 상차 중이면 타이머 체크 안 함 → carry_done_callback만 처리
            if self.current_room_type == 'load':
                elapsed = (self.get_clock().now().nanoseconds - self.wait_start_time) / 1e9
                if int(elapsed) != int(elapsed - 0.1):
                    self.get_logger().info(f"🔼 상차 대기 중... ({elapsed:.0f}초 경과, carry_done 신호 대기)")
                return

            elapsed   = (self.get_clock().now().nanoseconds - self.wait_start_time) / 1e9
            remaining = self.wait_duration - elapsed

            if int(elapsed) != int(elapsed - 0.1):
                if self.current_room_type == 'unload':
                    self.get_logger().info(f"🔽 하차 중... ({remaining:.1f}초 남음)")
                elif self.current_room_type == 'standby':
                    self.get_logger().info(f"🅿️  대기 중... ({remaining:.1f}초 남음)")

            if elapsed >= self.wait_duration:
                self.is_waiting = False
                self.get_logger().info("✅ 작업 완료! 다음 목적지로 출발합니다.")
                if self.is_mission_active:
                    self.mission_idx += 1
                    if self.mission_idx < len(self.mission_sequence):
                        self.set_next_destination(self.mission_sequence[self.mission_idx])
                        return
                    else:
                        self.get_logger().info("🏁 MISSION COMPLETE!")
                        self.is_mission_active = False
                        done_msg = Bool()
                        done_msg.data = True
                        self.pub_mission_done.publish(done_msg)
                self.global_path = []
            return

        # ── 라인 정렬 모드 ──
        if self.is_line_aligning:
            cmd = Twist()

            ranges = np.array(self.scan_data)
            ranges[np.isnan(ranges) | np.isinf(ranges)] = 3.5
            ranges[ranges < 0.15] = 3.5
            num_points  = len(ranges)
            idx_per_deg = num_points / 360.0
            f_idx        = int(30 * idx_per_deg)
            front_ranges = np.concatenate((ranges[:f_idx], ranges[-f_idx:]))
            left_ranges  = ranges[int(45*idx_per_deg):int(120*idx_per_deg)]
            right_ranges = ranges[int(240*idx_per_deg):int(315*idx_per_deg)]
            f_dist = np.min(front_ranges)
            l_dist = np.min(left_ranges)
            r_dist = np.min(right_ranges)

            if f_dist < 0.22 or l_dist < 0.18 or r_dist < 0.18:
                self.get_logger().warn(
                    f"⚠️  정렬 중 장애물! F:{f_dist:.2f} L:{l_dist:.2f} R:{r_dist:.2f}")
                cmd.linear.x  = 0.0
                cmd.angular.z = 0.0
                self.pub_cmd.publish(cmd)
                return

            if self.line_align_phase == 1:
                if self.line_cx_offset is None:
                    cmd.linear.x  = 0.0
                    cmd.angular.z = 0.2
                    self.last_align_cmd_az = 0.0
                    self.get_logger().info("🔍 라인 탐색 중...")
                else:
                    offset_ratio = self.line_cx_offset / (self.image_width / 2)
                    if abs(offset_ratio) > 0.08:
                        raw_az        = -0.25 * offset_ratio
                        cmd.angular.z = (self.CMD_SMOOTH_ALPHA * raw_az +
                                        (1 - self.CMD_SMOOTH_ALPHA) * self.last_align_cmd_az)
                        self.last_align_cmd_az = cmd.angular.z
                        cmd.linear.x  = 0.0
                        self.get_logger().info(f"↔️  Phase1 | offset:{self.line_cx_offset}px az:{cmd.angular.z:.3f}")
                    else:
                        self.get_logger().info("✅ Phase1 완료 → Phase3")
                        self.last_align_cmd_az = 0.0
                        self.line_align_phase  = 3
                self.pub_cmd.publish(cmd)
                return

            if self.line_align_phase == 3:
                if self.line_angle is None:
                    cmd.linear.x  = 0.0
                    cmd.angular.z = 0.15
                    self.pub_cmd.publish(cmd)
                    return
                angle_err = self.line_angle
                if abs(angle_err) > 15.0:
                    cmd.linear.x  = 0.0
                    raw_az        = max(-0.3, min(0.3, 0.3 * (angle_err / 90.0)))
                    cmd.angular.z = (self.CMD_SMOOTH_ALPHA * raw_az +
                                    (1 - self.CMD_SMOOTH_ALPHA) * self.last_align_cmd_az)
                    self.last_align_cmd_az = cmd.angular.z
                    self.get_logger().info(f"🔄 Phase3 | angle:{angle_err:.1f}° az:{cmd.angular.z:.3f}")
                else:
                    cmd.linear.x  = 0.0
                    cmd.angular.z = 0.0
                    self.pub_cmd.publish(cmd)
                    self.get_logger().info("✅ 라인 정렬 완료! 하차 대기 시작.")
                    self.is_line_aligning  = False
                    self.line_align_phase  = 0
                    self.last_align_cmd_az = 0.0
                    self.is_waiting        = True
                    self.wait_start_time   = self.get_clock().now().nanoseconds
                    return
                self.pub_cmd.publish(cmd)
                return

        if not self.global_path:
            return

        # ── 라이다 전처리 ──
        ranges = np.array(self.scan_data)
        ranges[np.isnan(ranges) | np.isinf(ranges)] = 3.5
        ranges[ranges < 0.15] = 3.5

        num_points  = len(ranges)
        idx_per_deg = num_points / 360.0
        f_idx       = int(40 * idx_per_deg)
        front_ranges = np.concatenate((ranges[:f_idx], ranges[-f_idx:]))
        left_ranges  = ranges[int(45*idx_per_deg):int(120*idx_per_deg)]
        right_ranges = ranges[int(240*idx_per_deg):int(315*idx_per_deg)]
        back_ranges  = ranges[int(150*idx_per_deg):int(210*idx_per_deg)]

        front_left_ranges  = ranges[int(20*idx_per_deg):int(60*idx_per_deg)]
        front_right_ranges = ranges[int(300*idx_per_deg):int(340*idx_per_deg)]

        f_dist, l_dist, r_dist, b_dist = (
            np.min(front_ranges), np.min(left_ranges),
            np.min(right_ranges), np.min(back_ranges)
        )
        fl_dist = np.min(front_left_ranges)
        fr_dist = np.min(front_right_ranges)

        # ── 도착 판정 ──
        final_goal   = self.global_path[-1]
        dist_to_goal = sqrt((final_goal[0]-self.current_pose[0])**2 +
                            (final_goal[1]-self.current_pose[1])**2)

        if dist_to_goal < self.stop_tolerance:
            self.stop_robot()
            current_goal = self.mission_sequence[self.mission_idx]
            cmd_msg = String()

            if current_goal == self.rooms[self.current_load_key]:
                self.current_room_type = 'load'
                self.get_logger().info(f"📦 {self.current_load_key} 상차 도착! 상차 대기 중...")
                cmd_msg.data = 'LOAD'
                self.pub_forklift_cmd.publish(cmd_msg)
                # ★ /carry 에 zone 번호 발행 (0,1,2)
                carry_msg = Int32()
                carry_msg.data = self.current_zone - 1   # zone 1→0, 2→1, 3→2
                # self.pub_carry.publish(carry_msg)  # 필요시 활성화
                self.is_waiting      = True
                self.wait_start_time = self.get_clock().now().nanoseconds
                self.global_path     = []
                return                               # ★ pass → return 으로 변경

            elif current_goal == self.rooms[self.current_unload_key]:
                self.current_room_type = 'unload'
                self.get_logger().info(f"📦 {self.current_unload_key} 하차 도착! 라인 정렬 시작...")
                cmd_msg.data = 'UNLD'
                self.pub_forklift_cmd.publish(cmd_msg)
                self.is_line_aligning = True
                self.line_align_phase = 1
                self.last_align_cmd_az = 0.0   # ★ 추가
                self.global_path      = []
                return

            elif current_goal == self.rooms['standby']:
                self.current_room_type = 'standby'
                self.get_logger().info("🅿️  대기실 도착!")

            self.is_waiting      = True
            self.wait_start_time = self.get_clock().now().nanoseconds
            self.global_path     = []
            return

        # ── 초기 제자리 회전 ──
        if self.is_initial_turn:
            target_idx         = min(len(self.global_path)-1, 5)
            target_x, target_y = self.global_path[target_idx]
            alpha_init = atan2(target_y-self.current_pose[1], target_x-self.current_pose[0]) - self.current_yaw
            while alpha_init >  pi: alpha_init -= 2*pi
            while alpha_init < -pi: alpha_init += 2*pi

            turn_margin = 0.25
            cmd = Twist()
            if (f_dist < turn_margin) or (alpha_init > 0 and l_dist < turn_margin) or (alpha_init < 0 and r_dist < turn_margin):
                cmd.linear.x  = -0.1 if b_dist > 0.3 else 0.0
                cmd.angular.z = 0.0
                self.pub_cmd.publish(cmd); return
            elif b_dist < turn_margin:
                cmd.linear.x  = 0.1 if f_dist > 0.3 else 0.0
                cmd.angular.z = 0.0
                self.pub_cmd.publish(cmd); return
            if abs(alpha_init) > 0.15:
                cmd.linear.x  = 0.0
                cmd.angular.z = 0.4 if alpha_init > 0 else -0.4
                self.pub_cmd.publish(cmd); return
            else:
                self.get_logger().info("★★★ 방향 정렬 완료! ★★★")
                self.is_initial_turn = False

        # ── Pure Pursuit ──
        target_x, target_y = self.global_path[-1]
        for i in range(self.path_index, len(self.global_path)):
            px, py = self.global_path[i]
            dist   = sqrt((px-self.current_pose[0])**2 + (py-self.current_pose[1])**2)
            if dist >= self.lookahead_dist:
                target_x, target_y = px, py
                self.path_index = i; break

        alpha = atan2(target_y-self.current_pose[1], target_x-self.current_pose[0]) - self.current_yaw
        while alpha >  pi: alpha -= 2*pi
        while alpha < -pi: alpha += 2*pi

        # ── 상태 결정 및 동작 수행 ──
        cmd = Twist()
        if f_dist < 0.30 or l_dist < 0.22 or r_dist < 0.22:
            action = "stop"
        elif f_dist < 0.40:
            action = "go_back"
        elif fl_dist < 0.35 or fr_dist < 0.35:
            action = "turn_avoid"
            cmd.linear.x  = 0.05
            avoid_dir = -0.5 if fl_dist < fr_dist else 0.5
            cmd.angular.z = avoid_dir + (0.3 * sin(alpha))
        elif f_dist < 0.55:
            action = "turn_avoid"
            cmd.linear.x  = 0.05
            avoid_dir = 0.5 if l_dist >= r_dist else -0.5
            cmd.angular.z = avoid_dir + (0.3 * sin(alpha))
        else:
            action = "go_forward"

        if action == "go_forward":
            current_speed = self.linear_vel * (0.8 if l_dist < 0.35 or r_dist < 0.35 else 1.0)
            cmd.linear.x  = current_speed
            steering      = (2.2 * sin(alpha)) / self.lookahead_dist
            if l_dist < 0.35: steering -= 0.6 * (0.35 - l_dist)
            if r_dist < 0.35: steering += 0.6 * (0.35 - r_dist)
            cmd.angular.z = cmd.linear.x * steering
        elif action == "go_back":
            cmd.linear.x  = -0.15 if b_dist > 0.2 else 0.0
            cmd.angular.z = 0.0
        elif action == "stop":
            cmd.linear.x  = 0.0
            cmd.angular.z = 0.0

        self.pub_cmd.publish(cmd)
        self.get_logger().info(
            f"ACT:{action} | F:{f_dist:.2f} L:{l_dist:.2f} R:{r_dist:.2f} "
            f"FL:{fl_dist:.2f} FR:{fr_dist:.2f} Ang:{cmd.angular.z:.2f}"
        )

    def world_to_grid(self, world):
        return (int((world[1]-self.map_origin[1])/self.map_resolution),
                int((world[0]-self.map_origin[0])/self.map_resolution))

    def grid_to_world(self, grid):
        return [(grid[1]*self.map_resolution)+self.map_origin[0],
                (grid[0]*self.map_resolution)+self.map_origin[1]]

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