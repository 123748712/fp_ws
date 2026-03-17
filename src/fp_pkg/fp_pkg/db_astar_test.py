import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import String, Bool, Int32
from math import atan2, sqrt, sin, pi
import heapq
import numpy as np
from sensor_msgs.msg import LaserScan, BatteryState
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import threading
import sys
import tty
import termios

sys.path.append('/home/dev/fp_ws/src/fp_pkg')
from database.node_service import NodeService
from database.map_service import map_service

class NodeAStar:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0
    def __eq__(self, other): return self.position == other.position
    def __lt__(self, other): return self.f < other.f

class NavigationDBHelper:
    def __init__(self):
        self.node_service = NodeService()
    def get_active_map_id(self):
        result = map_service.get_active_map()
        if result:
            return result['map_seq']
        return None

    def load_common_nodes(self):
        map_id = self.get_active_map_id()
        if map_id is None:
            raise RuntimeError("활성화된 맵이 없습니다!")
        nodes = self.node_service.get_nodes({'map_id': map_id})
        common = {}
        waypoints_dict = {}
        for node in nodes:
            node_id = node['node_id']
            x = float(node['node_x_coord'])
            y = float(node['node_y_coord'])
            if node_id == 'WAIT001': common['standby'] = [x, y]
            if node_id == 'CHRG001': common['charge'] = [x, y]
            if node_id.startswith('NODE') and len(node_id) == 7:
                try:
                    idx = int(node_id[4:])
                    waypoints_dict[idx] = [x, y]
                except ValueError:
                    pass
        waypoints = [waypoints_dict[k] for k in sorted(waypoints_dict.keys())]
        return common, waypoints

    def load_rooms(self, zone: int):
        map_id = self.get_active_map_id()
        if map_id is None:
            raise RuntimeError("활성화된 맵이 없습니다!")

        nodes = self.node_service.get_nodes({'map_id': map_id})
        rooms = {}
        waypoints_dict = {}
        for node in nodes:
            node_id = node['node_id']
            x = float(node['node_x_coord'])
            y = float(node['node_y_coord'])
            if node_id == f'LOAD00{zone}': rooms[f'load_{zone}'] = [x, y]
            if node_id == f'UNLD00{zone}': rooms[f'unload_{zone}'] = [x, y]
            if node_id == 'WAIT001':       rooms['standby'] = [x, y]
            if node_id == 'CHRG001':       rooms['charge'] = [x, y]
            # NODE001~NODE004 경유 노드 수집
            if node_id.startswith('NODE') and len(node_id) == 7:
                try:
                    idx = int(node_id[4:])
                    waypoints_dict[idx] = [x, y]
                except ValueError:
                    pass

        if f'load_{zone}'   not in rooms: raise RuntimeError(f"LOAD00{zone} 노드가 DB에 없습니다!")
        if f'unload_{zone}' not in rooms: raise RuntimeError(f"UNLD00{zone} 노드가 DB에 없습니다!")
        if 'standby'        not in rooms: raise RuntimeError("WAIT001 노드가 DB에 없습니다!")
        if 'charge'         not in rooms: raise RuntimeError("CHRG001 노드가 DB에 없습니다!")

        # 경유 노드 번호 순 정렬
        waypoints = [waypoints_dict[k] for k in sorted(waypoints_dict.keys())]
        return rooms, map_id, waypoints

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
        # ★ carry_done QoS 수정: TRANSIENT_LOCAL로 변경 (웹에서 latched로 보낼 경우 대비)
        reliable_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        # ── 대기 관련 변수 ──
        self.is_waiting = False
        self.wait_duration = 3.0
        self.wait_start_time = None
        self.current_room_type = None  # 'load' | 'unload' | 'standby'
        self._waiting_stop_sent = False  # 대기 진입 시 정지 1회만 발행 플래그

        # 비상정지 플래그
        self.is_emergency_paused = False

        # ── 경유 순환 주행 변수 ──
        self.is_waypoint_loop = False   # /waypoint_drive 트리거 경유 주행 모드
        self.wp_loop_idx = 0             # 현재 경유 노드 인덱스
        self.wp_final_pose = None        # 최종 목적지 [x, y]
        self.wp_target_yaw = 0.0         # 최종 목적지 도착 후 정렬할 yaw
        self.wp_final_aligning = False   # 최종 방향 정렬 중 플래그

        # ── 배터리 관련 ──
        self.battery_percent = 100.0   # 현재 배터리 %
        self.battery_threshold = 40.0    # 이 % 이하면 충전 노드로
        self.is_charging = False   # 충전 이동 중 플래그
        self._battery_warn_sent = False   # 배터리 부족 경고 1회만 출력 플래그

        # ── 주행 설정 ──
        self.lookahead_dist = 0.5
        self.linear_vel = 0.15
        self.stop_tolerance = 0.15

        # ── rooms / 미션 ──
        self.rooms = {}
        self.waypoints = []   # 경유 노드 목록 (순서대로)
        self.current_load_key = 'load_1'
        self.current_unload_key = 'unload_1'
        self.mission_sequence = []
        self.mission_idx = 0
        self.is_mission_active = False

        # ── 상태 관리 변수 ──
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
        self._last_progress_time = None  # 마지막 경로 진행 시각
        self._last_progress_pose = None  # 마지막 진행 위치
        self._stuck_timeout = 5.0        # 이 시간(초) 동안 못 움직이면 경로 재설정
        self.current_zone = 1

        # zone → 하차 색상 매핑 (아르코마커)
        # zone 1 → BLUE(0번), zone 2 → RED(1번), zone 3 → GREEN(2번)
        self.zone_color_map = {1: 'BLUE', 2: 'RED', 3: 'GREEN'}

        # ── 퍼블리셔 ──
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_path = self.create_publisher(Path, '/planned_path', 10)
        self.pub_forklift_cmd = self.create_publisher(String, '/forklift_cmd', 10)
        self.pub_mission_start = self.create_publisher(Bool, '/mission_start', 10)
        self.pub_mission_done = self.create_publisher(Bool, '/mission_done', 10)
        self.pub_emergency_resolve = self.create_publisher(Bool, '/emergency_resolve', 10)
        self.pub_carry = self.create_publisher(Int32, '/carry', reliable_profile)
        self.pub_unld_sig = self.create_publisher(String, '/get_unload', reliable_profile)

        # ── 서브스크라이버 ──
        self.sub_map = self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos_profile)
        self.sub_pose = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile)
        self.sub_battery = self.create_subscription(BatteryState, '/battery_state', self.battery_callback, 10)
        self.sub_goal_pose = self.create_subscription(PoseStamped, '/waypoint_drive', self.goal_pose_callback, reliable_profile)
        self.sub_emergency = self.create_subscription(Bool, '/emergency_stop', self.emergency_callback, 10)
        self.sub_forklift_done = self.create_subscription(Bool, '/forklift_done', self.forklift_done_callback, 10)
        self.sub_trigger = self.create_subscription(Int32, '/carry_id', self.trigger_callback, 10)
        self.sub_carry_done = self.create_subscription(String, '/carry_done', self.carry_done_callback, reliable_profile)
        self.sub_unld_done = self.create_subscription(String, '/done', self.done_callback, 10)

        # 시작 시 공통 노드(WAIT001, CHRG001) 미리 로드
        try:
            nav_db = NavigationDBHelper()
            common, waypoints = nav_db.load_common_nodes()
            self.rooms.update(common)
            self.waypoints = waypoints
            self.get_logger().info(f"공통 노드 로드 완료: {list(common.keys())} | 경유 노드 {len(waypoints)}개")
        except Exception as e:
            self.get_logger().warn(f"공통 노드 로드 실패 (미션 실행 시 로드됨): {e}")

        self.timer = self.create_timer(0.1, self.control_loop)

        self.keyboard_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.keyboard_thread.start()

        self.get_logger().info("🎮 Ready! '1~3' = 구역 선택 | 'S' = 강제 중지 | 'O' = 비상정지 해제")

    # ── 배터리 콜백 ──
    def battery_callback(self, msg):
        self.battery_percent = msg.percentage  # 이미 % 단위 (0~100)
        if self.battery_percent <= self.battery_threshold and not self.is_charging:
            if not self._battery_warn_sent:
                self.get_logger().warn(
                    f"🔋 배터리 부족 ({self.battery_percent:.1f}%) → 충전 노드로 즉시 이동! (최우선)")
                self._battery_warn_sent = True
            # 진행 중인 미션/순환 주행 중단
            self.is_mission_active = False
            self.is_waypoint_loop = False
            self.is_waiting = False
            self.global_path = []
            self._go_to_charge()
        else:
            self._battery_warn_sent = False  # 충전 완료 후 플래그 초기화

    def _go_to_charge(self):
        if self.map_data is None or self.current_pose is None:
            self.get_logger().error("❌ 맵/위치 정보 없음, 충전 이동 불가")
            return
        if 'charge' not in self.rooms:
            self.get_logger().error("❌ 충전 노드(CHRG001) 미로드, 충전 이동 불가")
            return
        self.is_charging = True
        self.get_logger().info("🔌 충전 노드로 이동 시작")
        self.set_next_destination(self.rooms['charge'])

    # ── /waypoint_drive 수신 → 경유 노드 주행 후 최종 좌표 정렬 정지 ──
    def goal_pose_callback(self, msg):
        if self.is_mission_active:
            self.get_logger().warn("⚠️  미션 주행 중 — /waypoint_drive 무시")
            return
        if self.map_data is None or self.current_pose is None:
            self.get_logger().error("❌ 맵/위치 정보 미수신, 잠시 후 다시 시도하세요")
            return
        if not self.waypoints:
            self.get_logger().error("❌ 경유 노드 없음 (미션 미실행 상태)")
            return
        if self.is_waypoint_loop:
            self.get_logger().info("ℹ️  이미 경유 주행 중")
            return
        # x, y 저장
        self.get_logger().info(f"📩 /waypoint_drive 수신! x={msg.pose.position.x:.3f} y={msg.pose.position.y:.3f} qz={msg.pose.orientation.z:.4f} qw={msg.pose.orientation.w:.4f}")
        self.wp_final_pose = [msg.pose.position.x, msg.pose.position.y]
        # qz, qw → yaw 변환
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        self.wp_target_yaw = atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)
        self.get_logger().info(
            f"📍 /waypoint_drive 수신 → WP1→WP2→WP3→WP4→최종({self.wp_final_pose}) yaw={self.wp_target_yaw:.2f}rad")
        self.is_waypoint_loop = True
        self.wp_loop_idx = 0
        self.wp_final_aligning = False
        self.set_next_destination(self.waypoints[self.wp_loop_idx])

    # ── 비상정지 ──
    def emergency_callback(self, msg):
        if msg.data:
            self.is_emergency_paused = True
            self.stop_robot()
            self.get_logger().warn("⚠️  비상정지 신호 수신! 로봇 정지.")
        else:
            self.is_emergency_paused = False
            self.is_initial_turn = True
            self.get_logger().info("✅ 비상정지 해제. 주행 재개합니다.")

    # ── 하차 완료 (/done String) ──
    def done_callback(self, msg):
        self.get_logger().info(f"📩 /done 수신: '{msg.data}'")
        if self.is_waiting and self.current_room_type == 'unload':
            self.is_waiting = False
            self.get_logger().info("✅ 하차 완료 신호(/done)! 다음 목적지로 출발합니다.")
            self._advance_mission()

    # ── 기존 forklift_done (standby 전용으로 축소) ──
    def forklift_done_callback(self, msg):
        if msg.data and self.is_waiting and self.current_room_type == 'standby':
            self.is_waiting = False
            self.get_logger().info("✅ 대기 완료 신호(forklift_done)! 다음 목적지로 출발합니다.")
            self._advance_mission()

    # ── 상차 완료 (/carry_done String) ──
    def carry_done_callback(self, msg):
        self.get_logger().info(f"📩 /carry_done 수신: {msg.data} | is_waiting={self.is_waiting} | room={self.current_room_type}")
        if msg.data and self.is_waiting and self.current_room_type == 'load':
            self.is_waiting = False
            self.get_logger().info("✅ 상차 완료 신호(/carry_done)! 다음 목적지로 출발합니다.")
            self._advance_mission()

    def _advance_mission(self):
        # ── 경유 주행 모드 ──
        if self.is_waypoint_loop:
            if not self.waypoints:
                self.is_waypoint_loop = False
                return
            next_idx = self.wp_loop_idx + 1
            if next_idx < len(self.waypoints):
                # 다음 경유 노드로
                self.wp_loop_idx = next_idx
                self.get_logger().info(
                    f"🔀 경유: {self.wp_loop_idx+1}/{len(self.waypoints)}번 노드로 이동")
                self.set_next_destination(self.waypoints[self.wp_loop_idx])
            else:
                # 모든 경유 노드 완료 → 최종 목적지로
                self.get_logger().info(f"🏁 경유 완료 → 최종 목적지로 이동: {self.wp_final_pose}")
                self.set_next_destination(self.wp_final_pose)
            return

        # ── 충전 이동 완료 ──
        if self.is_charging:
            self.get_logger().info("🔌 충전 노드 도착! 충전 대기 중...")
            self.is_charging = False
            return

        if not self.is_mission_active:
            return
        self.mission_idx += 1
        if self.mission_idx < len(self.mission_sequence):
            self.set_next_destination(self.mission_sequence[self.mission_idx])
        else:
            self.get_logger().info("🏁 MISSION COMPLETE!")
            self.is_mission_active = False
            done_msg = Bool(); done_msg.data = True
            self.pub_mission_done.publish(done_msg)
            self.global_path = []

    # ── 키보드 ──
    def keyboard_listener(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while True:
                ch = sys.stdin.read(1)
                if ch in ('1', '2', '3'):
                    self.trigger_mission(zone=int(ch))
                elif ch in ('s', 'S'):
                    self.get_logger().info("🛑 강제 중지 명령 수신.")
                    self.is_mission_active = False
                    self.is_waiting = False
                    self.is_waypoint_loop = False
                    self.is_charging = False
                    self.global_path = []
                    self.stop_robot()
                    done_msg = Bool(); done_msg.data = True
                    self.pub_mission_done.publish(done_msg)
                elif ch in ('o', 'O'):
                    if self.is_emergency_paused:
                        self.is_emergency_paused = False
                        self.is_initial_turn = True
                        resolve_msg = Bool(); resolve_msg.data = True
                        self.pub_emergency_resolve.publish(resolve_msg)
                        self.get_logger().info("✅ [O키] 비상정지 해제. 주행 재개합니다.")
                    else:
                        self.get_logger().info("ℹ️  현재 비상정지 상태가 아닙니다.")
                elif ch == '\x03':
                    break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    # ── /carry 수신 트리거 ──
    def trigger_callback(self, msg):
        zone_map = {0: 1, 1: 2, 2: 3}
        if msg.data not in zone_map:
            self.get_logger().error(f"❌ /carry 유효하지 않은 값: {msg.data}")
            return
        zone = zone_map[msg.data]
        self.get_logger().info(f"🔔 /carry 수신: {msg.data} → {zone}번 구역 미션 시작")
        self.trigger_mission(zone=zone)

    # ── 미션 시작 ──
    def trigger_mission(self, zone: int):
        if self.is_mission_active:
            self.get_logger().warn("⚠️  미션이 이미 진행 중입니다!")
            return
        if self.map_data is None or self.current_pose is None:
            self.get_logger().error("❌ 맵 또는 위치 정보가 준비되지 않았습니다!")
            return

        self.current_zone = zone

        try:
            nav_db = NavigationDBHelper()
            self.rooms, active_map_id, self.waypoints = nav_db.load_rooms(zone)
            self.get_logger().info(
                f"✅ [{zone}번 구역] DB 로드 완료 (map_id={active_map_id})\n"
                f"   상차(LOAD)  : {self.rooms[f'load_{zone}']}\n"
                f"   하차(UNLOAD): {self.rooms[f'unload_{zone}']}\n"
                f"   대기(STANDBY): {self.rooms['standby']}"
            )
        except Exception as e:
            self.get_logger().error(f"❌ DB 로드 실패: {e}")
            return

        self.current_load_key = f'load_{zone}'
        self.current_unload_key = f'unload_{zone}'

        self.mission_sequence = []
        wp_fwd = self.waypoints          # 상차→하차: 1,2,3,4 순서
        wp_rev = self.waypoints[::-1]    # 하차→상차: 4,3,2,1 역순
        for _ in range(5):
            # 상차 → 경유(순방향) → 하차
            self.mission_sequence.append(self.rooms[self.current_load_key])
            self.mission_sequence.extend(wp_fwd)
            self.mission_sequence.append(self.rooms[self.current_unload_key])
            # 하차 → 경유(역방향) → 상차 (마지막 회차 제외)
            if _ < 4:
                self.mission_sequence.extend(wp_rev)
        self.mission_sequence.append(self.rooms['standby'])

        self.get_logger().info(f"🚀 미션 시작! [{zone}번 구역] 상차→하차 5회 왕복.")

        start_msg = Bool(); start_msg.data = True
        self.pub_mission_start.publish(start_msg)

        self.is_mission_active = True
        self.mission_idx = 0
        self.set_next_destination(self.mission_sequence[0])

    # ── 경로 탐색 ──
    def set_next_destination(self, goal_pose):
        self.get_logger().info(f"📍 다음 목표: {goal_pose}")

        drive_msg = Bool(); drive_msg.data = True
        self.pub_mission_start.publish(drive_msg)

        path_grid = self.run_astar(self.world_to_grid(self.current_pose),
                                   self.world_to_grid(goal_pose))
        if path_grid:
            self.global_path = [self.grid_to_world(p) for p in path_grid]
            self.path_index = 0
            self.is_initial_turn = True
            self._last_progress_time = self.get_clock().now().nanoseconds
            self._last_progress_pose = self.current_pose[:]
            self.publish_path_viz()
            self.get_logger().info(f"경로 탐색 완료! {self.mission_idx+1}/{len(self.mission_sequence)}")
        else:
            self.get_logger().error("❌ 경로를 찾을 수 없습니다!")
            self.is_mission_active = False

    def run_astar(self, start, end):
        if self.map_data is None: return None
        if not (0 <= start[0] < self.map_height and 0 <= start[1] < self.map_width): return None
        if not (0 <= end[0]   < self.map_height and 0 <= end[1]   < self.map_width): return None

        start_node = NodeAStar(None, start)
        end_node = NodeAStar(None, end)
        open_list = []; heapq.heappush(open_list, start_node)
        visited = set()
        moves = [(0,1,1.0),(0,-1,1.0),(1,0,1.0),(-1,0,1.0),
                      (1,1,1.41),(1,-1,1.41),(-1,1,1.41),(-1,-1,1.41)]
        safety_margin = 2; preferred_margin = 10; obstacle_threshold = 80

        while open_list:
            cur = heapq.heappop(open_list)
            if cur.position in visited: continue
            visited.add(cur.position)

            if cur.position == end_node.position:
                path = []
                c = cur
                while c: path.append(c.position); c = c.parent
                full = path[::-1]
                smoothed = full[::3]
                if full[-1] not in smoothed: smoothed.append(full[-1])
                return smoothed

            for dy, dx, cost in moves:
                ny, nx = cur.position[0]+dy, cur.position[1]+dx
                if not (0 <= ny < self.map_height and 0 <= nx < self.map_width): continue
                if self.map_data[ny][nx] > obstacle_threshold or self.map_data[ny][nx] == -1: continue

                penalty = 0; too_close = False
                for r in range(1, preferred_margin+1):
                    for cy, cx in [(ny+r,nx),(ny-r,nx),(ny,nx+r),(ny,nx-r)]:
                        if 0 <= cy < self.map_height and 0 <= cx < self.map_width:
                            if self.map_data[cy][cx] > obstacle_threshold:
                                if r <= safety_margin: too_close = True; break
                                penalty += (preferred_margin - r) * 15
                    if too_close: break
                if too_close: continue

                nn = NodeAStar(cur, (ny, nx))
                nn.g = cur.g + cost + penalty
                nn.h = sqrt((ny-end[0])**2 + (nx-end[1])**2)
                nn.f = nn.g + nn.h
                heapq.heappush(open_list, nn)
        return None

    # ── 메인 제어 루프 ──
    def control_loop(self):
        if self.is_emergency_paused:
            self.stop_robot(); return
        if self.scan_data is None or self.current_pose is None:
            return

        # ── 최종 목적지 yaw 정렬 ──
        if self.wp_final_aligning:
            yaw_err = self.wp_target_yaw - self.current_yaw
            while yaw_err >  pi: yaw_err -= 2*pi
            while yaw_err < -pi: yaw_err += 2*pi
            if abs(yaw_err) > 0.05:
                cmd = Twist()
                cmd.angular.z = 0.3 if yaw_err > 0 else -0.3
                self.pub_cmd.publish(cmd)
                return
            else:
                self.stop_robot()
                self.wp_final_aligning = False
                self.wp_final_pose = None
                self.get_logger().info("✅ 최종 방향 정렬 완료! 정지.")
                return

        # ── 대기 중 처리 ──
        if self.is_waiting:
            # ★ 정지 명령은 대기 진입 시 1회만 발행 → 다른 노드 cmd_vel 충돌 방지
            if not self._waiting_stop_sent:
                self.stop_robot()
                self._waiting_stop_sent = True

            # 상차: carry_done 신호 대기 (타이머 없음) → RealTurtlebotMissionNode가 cmd_vel 담당
            if self.current_room_type == 'load':
                elapsed = (self.get_clock().now().nanoseconds - self.wait_start_time) / 1e9
                if int(elapsed) != int(elapsed - 0.1):
                    self.get_logger().info(f"🔼 상차 대기 중... ({elapsed:.0f}초 경과, /carry_done 신호 대기)")
                return

            # 하차: /done 신호 대기 (타이머 없음) → FSMUnloadNode가 cmd_vel 담당
            if self.current_room_type == 'unload':
                elapsed = (self.get_clock().now().nanoseconds - self.wait_start_time) / 1e9
                if int(elapsed) != int(elapsed - 0.1):
                    self.get_logger().info(f"🔽 하차 대기 중... ({elapsed:.0f}초 경과, /done 신호 대기)")
                return

            # 대기실: 타이머 (standby는 다른 노드 없으므로 stop 유지 무관)
            if self.current_room_type == 'standby':
                elapsed = (self.get_clock().now().nanoseconds - self.wait_start_time) / 1e9
                remaining = self.wait_duration - elapsed
                if int(elapsed) != int(elapsed - 0.1):
                    self.get_logger().info(f"🅿️  대기 중... ({remaining:.1f}초 남음)")
                if elapsed >= self.wait_duration:
                    self.is_waiting = False
                    self.get_logger().info("✅ 대기 완료! 다음 목적지로 출발합니다.")
                    self._advance_mission()
            return

        if not self.global_path:
            return

        # ── 라이다 전처리 ──
        ranges = np.array(self.scan_data)
        ranges[np.isnan(ranges) | np.isinf(ranges)] = 3.5
        ranges[ranges < 0.15] = 3.5

        num_points = len(ranges)
        idx_per_deg = num_points / 360.0
        f_idx = int(40 * idx_per_deg)

        front_ranges = np.concatenate((ranges[:f_idx], ranges[-f_idx:]))
        left_ranges = ranges[int(45*idx_per_deg):int(120*idx_per_deg)]
        right_ranges = ranges[int(240*idx_per_deg):int(315*idx_per_deg)]
        back_ranges = ranges[int(150*idx_per_deg):int(210*idx_per_deg)]
        fl_ranges = ranges[int(20*idx_per_deg):int(60*idx_per_deg)]
        fr_ranges = ranges[int(300*idx_per_deg):int(340*idx_per_deg)]

        f_dist = np.min(front_ranges)
        l_dist = np.min(left_ranges)
        r_dist = np.min(right_ranges)
        b_dist = np.min(back_ranges)
        fl_dist = np.min(fl_ranges)
        fr_dist = np.min(fr_ranges)

        # ── 도착 판정 ──
        final_goal = self.global_path[-1]
        dist_to_goal = sqrt((final_goal[0]-self.current_pose[0])**2 +
                            (final_goal[1]-self.current_pose[1])**2)

        if dist_to_goal < self.stop_tolerance:
            self.stop_robot()
            cmd_msg = String()

            # ── 경유 주행 모드 도착 판정 ──
            if self.is_waypoint_loop:
                # 최종 목적지 도착 여부 확인
                is_final = (self.wp_final_pose is not None and
                            self.wp_loop_idx >= len(self.waypoints) - 1 and
                            self.global_path[-1] == self.grid_to_world(self.world_to_grid(self.wp_final_pose)))
                if is_final or (self.wp_final_pose is not None and
                                abs(self.current_pose[0] - self.wp_final_pose[0]) < self.stop_tolerance and
                                abs(self.current_pose[1] - self.wp_final_pose[1]) < self.stop_tolerance):
                    # 최종 목적지 도착 → yaw 정렬 후 정지
                    self.stop_robot()
                    self.global_path = []
                    self.wp_final_aligning = True
                    self.get_logger().info(f"🎯 최종 목적지 도착! yaw={self.wp_target_yaw:.2f}rad 정렬 시작")
                    self.is_waypoint_loop = False
                    return
                self.get_logger().info(
                    f"🔀 경유 통과! ({self.wp_loop_idx+1}/{len(self.waypoints)}) → 다음 노드로")
                self.global_path = []
                self._advance_mission()
                return

            # ── 충전 노드 도착 (mission_sequence 참조 전에 체크) ──
            if self.is_charging:
                self.get_logger().info("🔌 충전 노드 도착!")
                self.is_charging = False
                self.is_waiting = False
                self.global_path = []
                return

            current_goal = self.mission_sequence[self.mission_idx]

            # ── 경유 노드 도착: 대기 없이 바로 다음으로 ──
            if any(current_goal == wp for wp in self.waypoints):
                self.get_logger().info("🔀 경유 노드 통과! 다음으로 진행")
                self.global_path = []
                self._advance_mission()
                return

            if current_goal == self.rooms[self.current_load_key]:
                # ── 상차 도착 ──
                self.get_logger().info(f"📦 {self.current_load_key} 상차 도착! /carry_done 신호 대기 중...")
                self.current_room_type = 'load'
                cmd_msg.data = 'LOAD'
                self.pub_forklift_cmd.publish(cmd_msg)
                carry_msg = Int32()
                carry_msg.data = self.current_zone - 1
                self.pub_carry.publish(carry_msg)
                self.is_waiting = True
                self._waiting_stop_sent = False
                self.wait_start_time = self.get_clock().now().nanoseconds
                self.global_path = []
                return
            elif current_goal == self.rooms[self.current_unload_key]:
                # ── 하차 도착: 색상 신호 발행 후 /done 대기 ──
                self.current_room_type = 'unload'
                color = self.zone_color_map.get(self.current_zone, 'BLUE')
                self.get_logger().info(f"📦 {self.current_unload_key} 하차 도착! /get_unload → {color} 발행, /done 신호 대기")
                cmd_msg.data = 'UNLD'
                self.pub_forklift_cmd.publish(cmd_msg)
                # ★ 하차 색상(아르코마커) 신호 발행
                unld_msg = String()
                unld_msg.data = color   # 'BLUE' | 'RED' | 'GREEN'
                self.pub_unld_sig.publish(unld_msg)

            elif current_goal == self.rooms['standby']:
                # ── 대기실 도착 ──
                self.current_room_type = 'standby'
                self.get_logger().info("🅿️  대기실 도착!")

            self.is_waiting = True
            self._waiting_stop_sent = False  # 정지 1회 발행 플래그 초기화
            self.wait_start_time = self.get_clock().now().nanoseconds
            self.global_path = []
            return

        # ── 초기 제자리 회전 ──
        if self.is_initial_turn:
            target_idx = min(len(self.global_path)-1, 5)
            target_x, target_y = self.global_path[target_idx]
            alpha_init = atan2(target_y-self.current_pose[1],
                               target_x-self.current_pose[0]) - self.current_yaw
            while alpha_init >  pi: alpha_init -= 2*pi
            while alpha_init < -pi: alpha_init += 2*pi

            turn_margin = 0.25
            cmd = Twist()
            if (f_dist < turn_margin) or \
               (alpha_init > 0 and l_dist < turn_margin) or \
               (alpha_init < 0 and r_dist < turn_margin):
                cmd.linear.x = -0.1 if b_dist > 0.3 else 0.0
                cmd.angular.z = 0.0
                self.pub_cmd.publish(cmd); return
            elif b_dist < turn_margin:
                cmd.linear.x = 0.1 if f_dist > 0.3 else 0.0
                cmd.angular.z = 0.0
                self.pub_cmd.publish(cmd); return
            if abs(alpha_init) > 0.15:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.4 if alpha_init > 0 else -0.4
                self.pub_cmd.publish(cmd); return
            else:
                self.get_logger().info("★★★ 방향 정렬 완료! ★★★")
                self.is_initial_turn = False

        # ── 경로 진행 stuck 감지 → 경로 재설정 ──
        if self._last_progress_time is not None and self._last_progress_pose is not None:
            now = self.get_clock().now().nanoseconds
            elapsed_stuck = (now - self._last_progress_time) / 1e9
            moved = sqrt((self.current_pose[0]-self._last_progress_pose[0])**2 +
                         (self.current_pose[1]-self._last_progress_pose[1])**2)
            if moved > 0.05:  # 5cm 이상 움직이면 타이머 리셋
                self._last_progress_time = now
                self._last_progress_pose = self.current_pose[:]
            elif elapsed_stuck >= self._stuck_timeout:
                self.get_logger().warn(f"⚠️  {self._stuck_timeout:.0f}초 동안 경로 진행 없음 → 경로 재설정!")
                self._last_progress_time = None
                goal = self.global_path[-1]
                self.global_path = []
                self.set_next_destination(goal)
                return

        # ── Pure Pursuit ──
        target_x, target_y = self.global_path[-1]
        for i in range(self.path_index, len(self.global_path)):
            px, py = self.global_path[i]
            dist = sqrt((px-self.current_pose[0])**2 + (py-self.current_pose[1])**2)
            if dist >= self.lookahead_dist:
                target_x, target_y = px, py
                self.path_index = i; break

        alpha = atan2(target_y-self.current_pose[1],
                      target_x-self.current_pose[0]) - self.current_yaw
        while alpha >  pi: alpha -= 2*pi
        while alpha < -pi: alpha += 2*pi

        # ── 장애물 회피 + 주행 ──
        cmd = Twist()
        if f_dist < 0.22 or l_dist < 0.18 or r_dist < 0.18:
            action = "stop"
        elif f_dist < 0.27:
            action = "go_back"
        elif fl_dist < 0.25 or fr_dist < 0.25:
            action = "turn_avoid"
            cmd.linear.x = 0.05
            avoid_dir = -0.25 if fl_dist < fr_dist else 0.25
            cmd.angular.z = avoid_dir + (0.3 * sin(alpha))
        elif f_dist < 0.35:
            action = "turn_avoid"
            cmd.linear.x = 0.05
            avoid_dir = 0.22 if l_dist >= r_dist else -0.22
            cmd.angular.z = avoid_dir + (0.3 * sin(alpha))
        else:
            action = "go_forward"

        if action == "go_forward":
            spd = self.linear_vel * (0.8 if l_dist < 0.35 or r_dist < 0.35 else 1.0)
            cmd.linear.x = spd
            steering = (2.2 * sin(alpha)) / self.lookahead_dist
            if l_dist < 0.35: steering -= 0.6 * (0.35 - l_dist)
            if r_dist < 0.35: steering += 0.6 * (0.35 - r_dist)
            cmd.angular.z = cmd.linear.x * steering
        elif action == "go_back":
            cmd.linear.x = -0.15 if b_dist > 0.2 else 0.0
            cmd.angular.z = 0.0
        elif action == "stop":
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.pub_cmd.publish(cmd)
        self.get_logger().info(
            f"ACT:{action} | F:{f_dist:.2f} L:{l_dist:.2f} R:{r_dist:.2f} "
            f"FL:{fl_dist:.2f} FR:{fr_dist:.2f} Ang:{cmd.angular.z:.2f}"
        )

    # ── 유틸 ──
    def scan_callback(self, msg):
        r = np.array(msg.ranges)
        self.scan_data = np.where(np.isinf(r), 3.5, np.where(np.isnan(r), 3.5, r))

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

    def world_to_grid(self, world):
        return (int((world[1]-self.map_origin[1])/self.map_resolution),
                int((world[0]-self.map_origin[0])/self.map_resolution))

    def grid_to_world(self, grid):
        return [(grid[1]*self.map_resolution)+self.map_origin[0],
                (grid[0]*self.map_resolution)+self.map_origin[1]]

    def publish_path_viz(self):
        msg = Path(); msg.header.frame_id = 'map'
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