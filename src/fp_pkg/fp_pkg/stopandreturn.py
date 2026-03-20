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

import threading
import time
import copy
from dataclasses import dataclass, field
from typing import Optional, List, Tuple
from math import sqrt


# ══════════════════════════════════════════════════════════════
#  상태 정의
# ══════════════════════════════════════════════════════════════

class RobotState:
    IDLE            = 'idle'
    MISSION         = 'mission'
    WAYPOINT        = 'waypoint'
    LOADING         = 'loading'
    UNLOADING       = 'unloading'
    ABORTING        = 'aborting'
    RUNNING_TO_HOME = 'running_to_home'
    RETURNING       = 'returning'
    RESUMING        = 'resuming'
    CHARGING        = 'charging'
    EMERGENCY       = 'emergency'
    SAFE_EXIT       = 'safe_exit'       # 수동 리셋 전까지 STOP 무한 발행


VALID_TRANSITIONS = {
    RobotState.IDLE:            [RobotState.MISSION, RobotState.WAYPOINT,
                                  RobotState.CHARGING, RobotState.EMERGENCY],
    RobotState.MISSION:         [RobotState.LOADING, RobotState.UNLOADING,
                                  RobotState.ABORTING, RobotState.CHARGING,
                                  RobotState.EMERGENCY],
    RobotState.WAYPOINT:        [RobotState.IDLE, RobotState.ABORTING,
                                  RobotState.CHARGING, RobotState.EMERGENCY],
    RobotState.LOADING:         [RobotState.MISSION, RobotState.UNLOADING,
                                  RobotState.ABORTING, RobotState.EMERGENCY],
    RobotState.UNLOADING:       [RobotState.MISSION, RobotState.ABORTING,
                                  RobotState.EMERGENCY],
    RobotState.ABORTING:        [RobotState.RUNNING_TO_HOME,
                                  RobotState.SAFE_EXIT, RobotState.EMERGENCY],
    RobotState.RUNNING_TO_HOME: [RobotState.IDLE, RobotState.RESUMING,
                                  RobotState.EMERGENCY, RobotState.SAFE_EXIT],
    RobotState.RETURNING:       [RobotState.IDLE, RobotState.RESUMING,
                                  RobotState.EMERGENCY, RobotState.SAFE_EXIT],
    RobotState.RESUMING:        [RobotState.RUNNING_TO_HOME, RobotState.RETURNING,
                                  RobotState.EMERGENCY, RobotState.SAFE_EXIT],
    RobotState.CHARGING:        [RobotState.RETURNING, RobotState.IDLE,
                                  RobotState.EMERGENCY],
    RobotState.EMERGENCY:       [RobotState.RESUMING, RobotState.IDLE],
    RobotState.SAFE_EXIT:       [RobotState.IDLE],
}


# ══════════════════════════════════════════════════════════════
#  아두이노 명령 상수
# ══════════════════════════════════════════════════════════════

class ArduinoCmd:
    LOAD = 'LOAD'   # 상차 작업 시작
    UNLD = 'UNLD'   # 하차 작업 시작
    HOME = 'HOME'   # 귀환 완료 대기
    STOP = 'STOP'   # 비상 정지 (SAFE_EXIT)


# ══════════════════════════════════════════════════════════════
#  스냅샷
# ══════════════════════════════════════════════════════════════

@dataclass
class MissionSnapshot:
    """중단 시점 컨텍스트 전체 보존 (버퍼 교체 방식으로 오염 방지)"""
    robot_state: str = RobotState.IDLE
    current_node_key: str = 'standby'
    pending_abort: bool = False
    mission_idx: int = 0
    mission_sequence: List = field(default_factory=list)
    current_zone: int = 1
    current_load_key: str = 'load_1'
    current_unload_key: str = 'unload_1'
    current_room_type: Optional[str] = None
    is_waypoint_loop: bool = False
    wp_loop_idx: int = 0
    wp_final_pose: Optional[List] = None
    wp_target_yaw: float = 0.0
    pose_x: float = 0.0
    pose_y: float = 0.0
    pose_yaw: float = 0.0
    global_path: List = field(default_factory=list)
    path_index: int = 0
    timestamp: float = 0.0
    abort_reason: str = ''
    abort_pose: Optional[List] = None


# ══════════════════════════════════════════════════════════════
#  ReturnManager
# ══════════════════════════════════════════════════════════════

class ReturnManager:
    """
    내부 완결형 FSM 귀환 관리자
    웹 피드백 없음 - 모든 결과는 /forklift_cmd → 아두이노로만 전달
    """

    SAFE_FRONT_DIST  = 0.40
    SAFE_SIDE_DIST   = 0.25
    SCAN_MAX_AGE     = 0.5
    PRECHECK_TIMEOUT = 10.0
    MAX_RETRY        = 3
    SAFE_EXIT_INTERVAL = 1.0   # SAFE_EXIT 시 STOP 발행 주기 (초)

    def __init__(self, node):
        self._node = node
        self._state_lock = threading.Lock()

        self.robot_state = RobotState.IDLE
        self.current_node_key = 'standby'

        self._snapshot: Optional[MissionSnapshot] = None
        self._return_sequence: List[Tuple[str, list]] = []
        self._return_idx: int = 0
        self._pending_abort: bool = False
        self._retry_count: int = 0
        self._precheck_start: Optional[float] = None
        self._precheck_resume_state: str = RobotState.RUNNING_TO_HOME
        self.RETURN_NEXT: dict = {}

        self._path_calc_thread: Optional[threading.Thread] = None

        # SAFE_EXIT 무한 STOP 발행 스레드
        self._safe_exit_thread: Optional[threading.Thread] = None
        self._safe_exit_active: bool = False

    # ══════════════════════════════════════════════════════════
    #  상태 전이
    # ══════════════════════════════════════════════════════════

    def transition(self, new_state: str) -> bool:
        with self._state_lock:
            allowed = VALID_TRANSITIONS.get(self.robot_state, [])
            if new_state not in allowed:
                self._node.get_logger().error(
                    f"❌ 전이 차단: {self.robot_state} → {new_state}")
                return False
            prev = self.robot_state
            self.robot_state = new_state
        self._node.get_logger().info(f"[FSM] State Changed: {prev} → {new_state}")

        # SAFE_EXIT 진입 시 즉시 정지 + STOP 무한 발행 스레드 시작
        if new_state == RobotState.SAFE_EXIT:
            self._node.stop_robot()   # Active Braking
            self._start_safe_exit_loop()
        return True

    def get_state(self) -> str:
        with self._state_lock:
            return self.robot_state

    def update_node_key(self, key: str):
        with self._state_lock:
            self.current_node_key = key

    # ══════════════════════════════════════════════════════════
    #  RETURN_NEXT 동적 생성
    # ══════════════════════════════════════════════════════════

    def build_return_graph(self, waypoints: list):
        n = len(waypoints)
        self.RETURN_NEXT = {}
        for i in range(n):
            key = f'wp_{i + 1}'
            self.RETURN_NEXT[key] = f'wp_{i}' if i > 0 else 'standby'
        self.RETURN_NEXT['unload']  = f'wp_{n}' if n > 0 else 'standby'
        self.RETURN_NEXT['load']    = 'standby'
        self.RETURN_NEXT['charge']  = 'standby'
        self.RETURN_NEXT['standby'] = None
        self._node.get_logger().info(f"🗺️  RETURN_NEXT: {self.RETURN_NEXT}")

    # ══════════════════════════════════════════════════════════
    #  스냅샷 (버퍼 교체 방식)
    # ══════════════════════════════════════════════════════════

    def take_snapshot(self, abort_reason: str = ''):
        """
        Lock 밖에서 deepcopy(Heavy) → 완성 후 Lock 안에서 참조 교체(Atomic)
        생성 중 인터럽트 발생해도 이전 스냅샷 오염 없음
        """
        node = self._node
        try:
            pose_x = node.current_pose[0] if node.current_pose else 0.0
            pose_y = node.current_pose[1] if node.current_pose else 0.0
            buf = MissionSnapshot(
                robot_state=self.robot_state,
                current_node_key=self.current_node_key,
                pending_abort=self._pending_abort,
                mission_idx=node.mission_idx,
                mission_sequence=copy.deepcopy(node.mission_sequence),
                current_zone=node.current_zone,
                current_load_key=node.current_load_key,
                current_unload_key=node.current_unload_key,
                current_room_type=node.current_room_type,
                is_waypoint_loop=node.is_waypoint_loop,
                wp_loop_idx=node.wp_loop_idx,
                wp_final_pose=copy.deepcopy(node.wp_final_pose),
                wp_target_yaw=node.wp_target_yaw,
                pose_x=pose_x,
                pose_y=pose_y,
                pose_yaw=node.current_yaw,
                global_path=copy.deepcopy(node.global_path),
                path_index=node.path_index,
                timestamp=time.time(),
                abort_reason=abort_reason,
                abort_pose=[pose_x, pose_y],
            )
            with self._state_lock:
                self._snapshot = buf   # 원자적 교체
            node.get_logger().info(
                f"📸 스냅샷 | 노드={buf.current_node_key} "
                f"위치=({pose_x:.2f},{pose_y:.2f}) 원인={abort_reason}")
        except Exception as e:
            node.get_logger().error(f"❌ 스냅샷 실패: {e}")

    # ══════════════════════════════════════════════════════════
    #  abort_and_return() - 핵심 진입점
    # ══════════════════════════════════════════════════════════

    def abort_and_return(self, reason: str = 'web_button'):
        """
        웹 /return_home 수신 시 호출

        [단계 1] Lock 안: 공유변수 즉시 파기 (Atomic)
        [단계 2] Lock 밖: 스냅샷 저장 (Heavy deepcopy)
        [단계 3] 별도 스레드: A* 홈 경로 계산 (Non-blocking)
        """
        node = self._node

        with self._state_lock:
            state = self.robot_state
            if state in (RobotState.ABORTING, RobotState.RUNNING_TO_HOME,
                         RobotState.RETURNING, RobotState.RESUMING,
                         RobotState.EMERGENCY, RobotState.SAFE_EXIT):
                node.get_logger().info(f"ℹ️  이미 {state} — 무시")
                return
            # 충전 이동은 최우선 → 중단 불가, 로그만 출력
            if state == RobotState.CHARGING:
                node.get_logger().warn(
                    "🔋 충전 이동 중 — 중단 및 복귀 불가 (충전 완료 후 자동 대기)")
                return

            # [단계 1] 즉시 파기 (Lock 안, Atomic)
            node.global_path = []
            node.is_mission_active = False
            node.is_waypoint_loop = False
            node.is_waiting = False
            node.path_index = 0

            if state in (RobotState.LOADING, RobotState.UNLOADING):
                self._pending_abort = True
                node.get_logger().info(f"⏳ {state} 대기 중 — 완료 후 귀환")
                return  # 완료 신호 후 on_load/unload_done에서 처리

        # [단계 2] 스냅샷 저장 (Lock 밖)
        self.take_snapshot(abort_reason=reason)
        node.stop_robot()

        if not self.transition(RobotState.ABORTING):
            return

        node.get_logger().info(
            f"🚨 미션 파기 | 원인={reason} | 노드={self.current_node_key}")

        # [단계 3] 비차단 홈 경로 계산
        self._retry_count = 0
        self._start_home_path_thread()

    # ══════════════════════════════════════════════════════════
    #  비차단 홈 경로 계산 스레드
    # ══════════════════════════════════════════════════════════

    def _start_home_path_thread(self):
        if self._path_calc_thread and self._path_calc_thread.is_alive():
            return
        self._path_calc_thread = threading.Thread(
            target=self._calc_home_path_async, daemon=True)
        self._path_calc_thread.start()

    def _calc_home_path_async(self):
        """
        [Non-blocking 구조]
        Lock → 공유변수 복사 → Lock 해제
        → A* 계산 (Heavy, Lock 없음, control_loop 차단 없음)
        → Lock → 결과 저장 → Lock 해제
        """
        node = self._node

        # Step 1: 공유변수 복사만 (Lock 최소 보유)
        with self._state_lock:
            if node.current_pose is None or node.map_data is None:
                node.get_logger().error("❌ 맵/위치 없음 → SAFE_EXIT")
                self.transition(RobotState.SAFE_EXIT)
                return
            current_pose_copy = node.current_pose[:]
            start_key = self.current_node_key

        # Step 2: RETURN_NEXT 체인으로 귀환 시퀀스 생성 (Lock 밖)
        route_keys = []
        key = self.RETURN_NEXT.get(start_key)
        while key is not None:
            route_keys.append(key)
            key = self.RETURN_NEXT.get(key)

        # Step 3: 키 → 좌표 변환 (Lock 밖)
        return_sequence = []
        for k in route_keys:
            pose = self._key_to_pose(k)
            if pose is None:
                self._handle_failure(f'KEY_TO_POSE:{k}')
                return
            return_sequence.append((k, pose))

        if not return_sequence:
            # 이미 standby
            return_sequence = [('standby', node.rooms.get('standby'))]
            if return_sequence[0][1] is None:
                self._handle_failure('NO_STANDBY')
                return

        # Step 4: 첫 목적지 A* 계산 (Heavy, Lock 밖)
        first_key, first_pose = return_sequence[0]
        node.get_logger().info(f"[A*] Calculating home path: {start_key} → {[k for k in route_keys]}")
        path_grid = node.run_astar(
            node.world_to_grid(current_pose_copy),
            node.world_to_grid(first_pose)
        )
        if path_grid is None:
            node.get_logger().error(f"[A*] Home Path FAILED: {start_key} → {first_key}")
            self._handle_failure('ASTAR_FAILED')
            return
        node.get_logger().info(f"[A*] Home Path Success: {len(path_grid)} waypoints")

        new_path = [node.grid_to_world(p) for p in path_grid]
        now_ns = node.get_clock().now().nanoseconds

        # Step 5: 결과만 원자적 저장 (Lock 최소 보유)
        with self._state_lock:
            node.global_path = new_path
            node.path_index = 0
            node.is_initial_turn = True
            node._last_progress_time = now_ns
            node._last_progress_pose = current_pose_copy
            self._return_sequence = return_sequence
            self._return_idx = 0
            self._retry_count = 0

        if self.transition(RobotState.RUNNING_TO_HOME):
            node.publish_path_viz()
            node.get_logger().info(
                f"🏠 귀환 시작 | 경로: {[k for k, _ in return_sequence]}")

    # ══════════════════════════════════════════════════════════
    #  귀환 진행
    # ══════════════════════════════════════════════════════════

    def advance_home(self):
        """RUNNING_TO_HOME 중 현재 노드 도착 시 호출"""
        self._return_idx += 1
        if self._return_idx < len(self._return_sequence):
            key, pose = self._return_sequence[self._return_idx]
            self._set_destination_safe(pose, key)
        else:
            self._on_home_arrived()

    def advance_return(self):
        """일반 RETURNING 중 다음 노드로"""
        self._return_idx += 1
        if self._return_idx < len(self._return_sequence):
            key, pose = self._return_sequence[self._return_idx]
            self._set_destination_safe(pose, key)
        else:
            self._on_home_arrived()

    # ══════════════════════════════════════════════════════════
    #  홈 도착 → 아두이노 HOME 신호
    # ══════════════════════════════════════════════════════════

    def _on_home_arrived(self):
        node = self._node
        node.get_logger().info("🏠 홈(대기노드) 귀환 완료!")
        self.update_node_key('standby')
        self.transition(RobotState.IDLE)
        self._retry_count = 0
        self._return_sequence = []
        self._return_idx = 0
        self._pending_abort = False
        # 잔여 미션 데이터 초기화 (귀환 후 오작동 방지)
        node.is_mission_active = False
        node.is_waypoint_loop = False
        node.mission_sequence = []
        node.mission_idx = 0

        # 아두이노에 HOME 신호 (웹 알림 없음)
        cmd_home = String(); cmd_home.data = ArduinoCmd.HOME
        node.pub_forklift_cmd.publish(cmd_home)
        node.get_logger().info(f"[Serial] Sent HOME command to Arduino → 귀환 완료")

        # 미션 완료 토픽 (웹이 구독할 수도 있지만 선택적)
        done_msg_home = Bool(); done_msg_home.data = True
        node.pub_mission_done.publish(done_msg_home)

    # ══════════════════════════════════════════════════════════
    #  Pre-Check (LiDAR 신선도 + 안전 확인)
    # ══════════════════════════════════════════════════════════

    def start_precheck(self, resume_to: str = RobotState.RUNNING_TO_HOME):
        if self.transition(RobotState.RESUMING):
            self._precheck_start = time.time()
            self._precheck_resume_state = resume_to
            self._node.get_logger().info(
                f"🔍 Pre-Check 시작 → 통과 시 {resume_to}")

    def check_precheck(self) -> bool:
        """
        control_loop에서 0.1초마다 호출

        [LiDAR 신선도] scan_data_stamp 헤더 타임스탬프 확인
        → SCAN_MAX_AGE 초과 시 신뢰 불가, 신선 데이터 대기

        [Lock 최소화] scan_data 로컬 복사 후 Lock 없이 계산
        """
        node = self._node

        # 전체 타임아웃
        elapsed = time.time() - (self._precheck_start or time.time())
        if elapsed >= self.PRECHECK_TIMEOUT:
            node.get_logger().error(
                f"❌ Pre-Check 타임아웃 ({self.PRECHECK_TIMEOUT}s)")
            self._handle_failure('PRECHECK_TIMEOUT')
            return False

        # LiDAR 신선도 확인
        scan_stamp = getattr(node, 'scan_data_stamp', None)
        if scan_stamp is None or node.scan_data is None:
            node.get_logger().warn("⏳ Pre-Check: LiDAR 데이터 대기 중...")
            node.stop_robot()
            return False

        now_sec = node.get_clock().now().nanoseconds / 1e9
        scan_age = now_sec - scan_stamp
        if scan_age > self.SCAN_MAX_AGE:
            node.get_logger().warn(
                f"⏳ Pre-Check: LiDAR 오래됨 ({scan_age:.2f}s) — 대기")
            node.stop_robot()
            return False

        # 로컬 복사본으로 계산 (Lock 없음)
        scan_local = np.array(node.scan_data)
        scan_local[np.isnan(scan_local) | np.isinf(scan_local)] = 3.5
        scan_local[scan_local < 0.15] = 3.5
        n = len(scan_local)
        d = n / 360.0
        f_dist = float(np.min(
            np.concatenate((scan_local[:int(40*d)], scan_local[-int(40*d):]))))
        l_dist = float(np.min(scan_local[int(45*d):int(120*d)]))
        r_dist = float(np.min(scan_local[int(240*d):int(315*d)]))
        is_safe = (f_dist >= self.SAFE_FRONT_DIST and
                   l_dist >= self.SAFE_SIDE_DIST and
                   r_dist >= self.SAFE_SIDE_DIST)

        if is_safe:
            node.get_logger().info(
                f"✅ Pre-Check 통과 | 신선도:{scan_age:.3f}s "
                f"F:{f_dist:.2f} L:{l_dist:.2f} R:{r_dist:.2f}")
            resume_to = self._precheck_resume_state
            if self.transition(resume_to):
                if self._return_sequence and \
                        self._return_idx < len(self._return_sequence):
                    key, pose = self._return_sequence[self._return_idx]
                    self._set_destination_safe(pose, key)
                else:
                    self._start_home_path_thread()
            return True
        else:
            node.get_logger().warn(
                f"⚠️  Pre-Check 장애물 | "
                f"F:{f_dist:.2f} L:{l_dist:.2f} R:{r_dist:.2f}")
            node.stop_robot()
            return False

    # ══════════════════════════════════════════════════════════
    #  대기 완료 콜백 (pending_abort)
    # ══════════════════════════════════════════════════════════

    def on_load_done(self) -> bool:
        with self._state_lock:
            pending = self._pending_abort
            if pending:
                self._pending_abort = False
        if pending:
            self._node.get_logger().info("✅ 상차 완료 → pending abort 실행")
            self._node.stop_robot()
            if self.transition(RobotState.ABORTING):
                self._retry_count = 0
                self._start_home_path_thread()
            return True
        return False

    def on_unload_done(self) -> bool:
        with self._state_lock:
            pending = self._pending_abort
            if pending:
                self._pending_abort = False
        if pending:
            self._node.get_logger().info("✅ 하차 완료 → pending abort 실행")
            self._node.stop_robot()
            if self.transition(RobotState.ABORTING):
                self._retry_count = 0
                self._start_home_path_thread()
            return True
        return False

    # ══════════════════════════════════════════════════════════
    #  SAFE_EXIT → 아두이노 STOP 무한 발행
    # ══════════════════════════════════════════════════════════

    def _start_safe_exit_loop(self):
        """
        SAFE_EXIT 진입 시 별도 스레드에서 STOP 주기적 발행
        수동 리셋 전까지 하드웨어 안전 확보
        """
        self._safe_exit_active = True
        self._safe_exit_thread = threading.Thread(
            target=self._safe_exit_loop, daemon=True)
        self._safe_exit_thread.start()
        self._node.get_logger().error(
            "🚨 [FSM] SAFE_EXIT 진입 — 수동 리셋(R키)으로만 복구 가능")

    def _safe_exit_loop(self):
        """STOP 신호 주기적 발행 (수동 리셋 시 중단)"""
        node = self._node
        while self._safe_exit_active:
            try:
                cmd = String(); cmd.data = ArduinoCmd.STOP
                node.pub_forklift_cmd.publish(cmd)
                node.get_logger().warn("[Serial] Sent STOP command to Arduino due to SAFE_EXIT")
            except Exception:
                pass
            time.sleep(self.SAFE_EXIT_INTERVAL)

    # ══════════════════════════════════════════════════════════
    #  실패 처리
    # ══════════════════════════════════════════════════════════

    def _handle_failure(self, reason: str = ''):
        node = self._node
        self._retry_count += 1
        if self._retry_count <= self.MAX_RETRY:
            node.get_logger().warn(
                f"⚠️  경로 실패 ({self._retry_count}/{self.MAX_RETRY}) "
                f"원인={reason} | 2초 후 재시도")
            threading.Timer(2.0, self._retry_once).start()
        else:
            node.get_logger().error(
                f"❌ {self.MAX_RETRY}회 실패 → SAFE_EXIT | 원인={reason}\n"
                f"   노드={self.current_node_key}")
            self.transition(RobotState.SAFE_EXIT)
            node.stop_robot()

    def _retry_once(self):
        state = self.get_state()
        if state in (RobotState.ABORTING, RobotState.RUNNING_TO_HOME,
                     RobotState.RETURNING):
            # 스레드 중복 방지: 이미 계산 중이면 스킵
            if self._path_calc_thread and self._path_calc_thread.is_alive():
                return
            self._start_home_path_thread()

    # ══════════════════════════════════════════════════════════
    #  수동 리셋
    # ══════════════════════════════════════════════════════════

    def manual_reset(self):
        """
        SAFE_EXIT에서 수동 리셋 (astar_v2의 _full_reset()에서 호출)
        [체크2] ReturnManager 내부 상태만 초기화
        기존 노드 플래그(is_mission_active 등)는 _full_reset()이 담당
        """
        with self._state_lock:
            if self.robot_state != RobotState.SAFE_EXIT:
                self._node.get_logger().info("ℹ️  SAFE_EXIT 상태가 아닙니다.")
                return

        # STOP 무한 발행 스레드 중단
        self._safe_exit_active = False
        if self._safe_exit_thread:
            self._safe_exit_thread.join(timeout=2.0)

        # ReturnManager 내부 상태 전체 초기화
        self._retry_count = 0
        self._snapshot = None
        self._pending_abort = False
        self._return_sequence = []
        self._return_idx = 0
        self._precheck_start = None
        self._path_calc_thread = None

        node = self._node
        # [체크1] Active Braking - 리셋 즉시 정지 보장
        node.stop_robot()
        # [체크3] global_path 원자적 초기화
        with self._state_lock:
            node.global_path = []
            node.path_index = 0
        self.transition(RobotState.IDLE)
        node.get_logger().info("[System] Manual Reset Triggered. All flags cleared. → IDLE")

    # ══════════════════════════════════════════════════════════
    #  안전한 목적지 설정 (Non-blocking)
    # ══════════════════════════════════════════════════════════

    def _set_destination_safe(self, goal_pose: list, node_key: str = ''):
        """
        [Non-blocking 구조]
        Lock → 복사 → Lock 해제 → A*(Heavy) → Lock → 저장
        """
        node = self._node
        with self._state_lock:
            if node.current_pose is None or node.map_data is None:
                node.get_logger().error("❌ 맵/위치 없음")
                self._handle_failure('NO_MAP_OR_POSE')
                return
            current_pose_copy = node.current_pose[:]

        # A* 계산 (Lock 밖, Heavy)
        node.get_logger().info(f"[A*] Calculating path to: {node_key}")
        path_grid = node.run_astar(
            node.world_to_grid(current_pose_copy),
            node.world_to_grid(goal_pose)
        )
        if path_grid is None:
            node.get_logger().error(f"[A*] Path FAILED to: {node_key}")
            self._handle_failure(f'ASTAR_TO_{node_key}')
            return
        node.get_logger().info(f"[A*] Path Success to: {node_key} ({len(path_grid)} pts)")

        new_path = [node.grid_to_world(p) for p in path_grid]
        now_ns = node.get_clock().now().nanoseconds

        # 결과만 저장 (Lock 최소 보유)
        with self._state_lock:
            node.global_path = new_path
            node.path_index = 0
            node.is_initial_turn = True
            node._last_progress_time = now_ns
            node._last_progress_pose = current_pose_copy
            self._retry_count = 0

        node.publish_path_viz()
        node.get_logger().info(f"📍 귀환 목적지: {node_key}")

    # ══════════════════════════════════════════════════════════
    #  유틸
    # ══════════════════════════════════════════════════════════

    def _key_to_pose(self, key: str) -> Optional[list]:
        node = self._node
        if key == 'standby': return node.rooms.get('standby')
        if key == 'load':    return node.rooms.get(node.current_load_key)
        if key == 'unload':  return node.rooms.get(node.current_unload_key)
        if key == 'charge':  return node.rooms.get('charge')
        if key.startswith('wp_'):
            try:
                idx = int(key.split('_')[1]) - 1
                if 0 <= idx < len(node.waypoints):
                    return node.waypoints[idx]
            except (ValueError, IndexError):
                pass
        node.get_logger().error(f"❌ 알 수 없는 키: {key}")
        return None

    def is_going_home(self) -> bool:
        return self.get_state() in (
            RobotState.ABORTING,
            RobotState.RUNNING_TO_HOME,
            RobotState.RETURNING,
            RobotState.RESUMING)

    def is_suspended(self) -> bool:
        return self.get_state() in (
            RobotState.LOADING, RobotState.UNLOADING)



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

    def load_rooms(self, unload_zone: int):
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
            if node_id == 'LOAD001': rooms['load_1'] = [x, y]
            if node_id == f'UNLD00{unload_zone}': rooms[f'unload_{unload_zone}'] = [x, y]
            if node_id == 'WAIT001':              rooms['standby'] = [x, y]
            if node_id == 'CHRG001':              rooms['charge']  = [x, y]
            if node_id.startswith('NODE') and len(node_id) == 7:
                try:
                    idx = int(node_id[4:])
                    waypoints_dict[idx] = [x, y]
                except ValueError:
                    pass

        if 'load_1' not in rooms: raise RuntimeError("LOAD001 노드가 DB에 없습니다!")
        if f'unload_{unload_zone}' not in rooms: raise RuntimeError(f"UNLD00{unload_zone} 노드가 DB에 없습니다!")
        if 'standby'               not in rooms: raise RuntimeError("WAIT001 노드가 DB에 없습니다!")
        if 'charge'                not in rooms: raise RuntimeError("CHRG001 노드가 DB에 없습니다!")

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
        self.battery_threshold = 30.0    # 이 % 이하면 충전 노드로
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
        self.zone_color_map = {1: 'BLUE', 2: 'RED', 3: 'YELLOW'}

        # ── 퍼블리셔 ──
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_path = self.create_publisher(Path, '/planned_path', 10)
        self.pub_forklift_cmd = self.create_publisher(String, '/forklift_cmd', 10)
        self.pub_mission_start = self.create_publisher(Bool, '/mission_start', 10)
        self.pub_mission_done = self.create_publisher(Bool, '/mission_done', 10)
        self.pub_emergency_resolve = self.create_publisher(Bool, '/emergency_resolve', 10)
        # Flask drive_bp에서 접근하는 퍼블리셔 (변수명 통일)
        self.return_home_publisher = self.create_publisher(Bool, '/return_home', 10)
        self.emergency_stop_publisher = self.create_publisher(Bool, '/emergency_stop', 10)
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
        self.sub_return_home = self.create_subscription(Bool, '/return_home', self._return_home_callback, 10)

        # 시작 시 공통 노드(WAIT001, CHRG001) 미리 로드
        try:
            nav_db = NavigationDBHelper()
            common, waypoints = nav_db.load_common_nodes()
            self.rooms.update(common)
            self.waypoints = waypoints
            self.get_logger().info(f"공통 노드 로드 완료: {list(common.keys())} | 경유 노드 {len(waypoints)}개")
            # ReturnManager가 이미 초기화된 경우 체인 업데이트
            if hasattr(self, 'return_mgr'):
                self.return_mgr.build_return_graph(waypoints)
        except Exception as e:
            self.get_logger().warn(f"공통 노드 로드 실패 (미션 실행 시 로드됨): {e}")

        # ── ReturnManager (FSM 귀환 시스템) - 공통노드 로드 후 초기화 ──
        self.return_mgr = ReturnManager(self)
        self.scan_data_stamp = None
        if self.waypoints:
            self.return_mgr.build_return_graph(self.waypoints)

        # ── ReturnManager (FSM 귀환 시스템) ──
        self.return_mgr = ReturnManager(self)
        self.scan_data_stamp = None
        # 공통 노드 로드 후 ReturnManager 체인 그래프 생성
        if self.waypoints:
            self.return_mgr.build_return_graph(self.waypoints)

        self.timer = self.create_timer(0.1, self.control_loop)

        self.keyboard_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.keyboard_thread.start()

        self.get_logger().info("🎮 Ready! '1~3' = 구역 선택 | 'S' = 강제 중지 | 'O' = 비상정지 해제")

    # ── 배터리 콜백 ──
    def battery_callback(self, msg):
        self.battery_percent = msg.percentage  # 이미 % 단위 (0~100)
        if self.battery_percent <= self.battery_threshold and not self.is_charging:
            # 귀환 중이면 배터리 충전 이동 스킵 (귀환 완료 후 처리)
            if self.return_mgr.is_going_home():
                return
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
        if self.return_mgr.is_going_home():
            self.get_logger().warn("⚠️  귀환 중 — 충전 이동 스킵")
            return
        if self.map_data is None or self.current_pose is None:
            self.get_logger().error("❌ 맵/위치 정보 없음, 충전 이동 불가")
            return
        if 'charge' not in self.rooms:
            self.get_logger().error("❌ 충전 노드(CHRG001) 미로드, 충전 이동 불가")
            return
        self.is_charging = True
        self.return_mgr.transition(RobotState.CHARGING)
        self.get_logger().info("🔌 충전 노드로 이동 시작")
        self.set_next_destination(self.rooms['charge'])

    # ── /waypoint_drive 수신 → 경유 노드 주행 후 최종 좌표 정렬 정지 ──
    def goal_pose_callback(self, msg):
        if self.is_mission_active:
            self.get_logger().warn("⚠️  미션 주행 중 — /waypoint_drive 무시")
            return
        if self.return_mgr.is_going_home():
            self.get_logger().warn(f"⚠️  귀환 중 ({self.return_mgr.get_state()}) — /waypoint_drive 무시")
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
        self.return_mgr.transition(RobotState.WAYPOINT)
        self.set_next_destination(self.waypoints[self.wp_loop_idx])

    # ── 비상정지 ──
    def emergency_callback(self, msg):
        if msg.data:
            self.get_logger().warn("[Web Input] Emergency Stop Received → SAFE_EXIT")
            self.is_emergency_paused = True
            self.is_mission_active = False
            self.is_waypoint_loop = False
            self.is_waiting = False
            self.global_path = []
            self.stop_robot()
            # 웹 비상정지 → SAFE_EXIT (키보드 R키로만 복구)
            if self.return_mgr.get_state() != RobotState.SAFE_EXIT:
                # EMERGENCY 전이 후 SAFE_EXIT으로
                self.return_mgr.transition(RobotState.EMERGENCY)
                self.return_mgr.transition(RobotState.SAFE_EXIT)
        else:
            # False 신호는 하드웨어 비상버튼 해제 (웹에서는 미사용)
            self.is_emergency_paused = False
            self.is_initial_turn = True
            self.get_logger().info("✅ 비상정지 해제. 주행 재개합니다.")

    # ── 하차 완료 (/done String) ──
    def done_callback(self, msg):
        self.get_logger().info(f"📩 /done 수신: '{msg.data}'")
        # ReturnManager pending_abort 선처리
        if self.return_mgr.on_unload_done():
            self.is_waiting = False
            return
        if self.is_waiting and self.current_room_type == 'unload':
            self.is_waiting = False
            self.return_mgr.transition(RobotState.MISSION)
            self.get_logger().info("✅ 하차 완료 신호(/done)! 다음 목적지로 출발합니다.")
            self._advance_mission()

    # ── 기존 forklift_done (standby 전용으로 축소) ──
    def forklift_done_callback(self, msg):
        if msg.data and self.is_waiting and self.current_room_type == 'standby':
            self.is_waiting = False
            self.return_mgr.transition(RobotState.IDLE)
            self.get_logger().info("✅ 대기 완료 신호(forklift_done)! 다음 목적지로 출발합니다.")
            self._advance_mission()

    # ── 상차 완료 (/carry_done String) ──
    def carry_done_callback(self, msg):
        self.get_logger().info(f"📩 /carry_done 수신: {msg.data} | is_waiting={self.is_waiting} | room={self.current_room_type}")
        # ReturnManager pending_abort 선처리
        if self.return_mgr.on_load_done():
            self.is_waiting = False
            return
        if msg.data and self.is_waiting and self.current_room_type == 'load':
            self.is_waiting = False
            self.return_mgr.transition(RobotState.MISSION)
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
                self.wp_loop_idx = next_idx
                self.return_mgr.update_node_key(f'wp_{self.wp_loop_idx+1}')
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
            self.return_mgr.transition(RobotState.IDLE)
            return

        if not self.is_mission_active:
            return
        self.mission_idx += 1
        if self.mission_idx < len(self.mission_sequence):
            self.set_next_destination(self.mission_sequence[self.mission_idx])
        else:
            self.get_logger().info("🏁 MISSION COMPLETE!")
            self.is_mission_active = False
            self.return_mgr.transition(RobotState.IDLE)
            done_msg = Bool(); done_msg.data = True
            self.pub_mission_done.publish(done_msg)
            self.global_path = []

    # ── [체크2] Full Reset: 모든 플래그 일괄 초기화 ──
    def _full_reset(self):
        """
        R키 또는 SAFE_EXIT 수동 리셋 시 호출
        ReturnManager 상태 + 기존 주행 플래그 모두 초기화
        한쪽만 리셋되어 먹통이 되는 현상 원천 차단
        """
        # ReturnManager 리셋 (SAFE_EXIT → IDLE 전이)
        self.return_mgr.manual_reset()

        # 기존 주행 플래그 전체 초기화
        self.is_mission_active = False
        self.is_waiting = False
        self.is_waypoint_loop = False
        self.is_charging = False
        self.is_emergency_paused = False
        self.is_initial_turn = False
        self.wp_final_aligning = False
        self._waiting_stop_sent = False
        self._pending_return = False if hasattr(self, '_pending_return') else False
        self._battery_warn_sent = False
        self._last_progress_time = None
        self._last_progress_pose = None
        self.scan_data_stamp = None   # LiDAR 신선도 초기화

        # 경로 초기화 (체크3: 빈 배열 참조 전에 stop 보장)
        self.stop_robot()
        self.global_path = []
        self.path_index = 0
        self.mission_sequence = []
        self.mission_idx = 0
        self.current_room_type = None

        self.get_logger().info("[System] Full Reset Complete. All mission/nav flags cleared.")

    # ── /return_home 토픽 콜백 ──
    def _return_home_callback(self, msg):
        if msg.data:
            self.get_logger().info("[Web Input] Abort & Return Requested")
            self.return_mgr.abort_and_return(reason='web_button')

    # ── 키보드 ──
    def keyboard_listener(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while True:
                ch = sys.stdin.read(1)
                if ch in ('1', '2', '3'):
                    self.trigger_mission(unload_zone=int(ch))
                elif ch in ('s', 'S'):
                    self.get_logger().info("🛑 강제 중지 명령 수신.")
                    self.is_mission_active = False
                    self.is_waiting = False
                    self.is_waypoint_loop = False
                    self.is_charging = False
                    self.global_path = []
                    self.stop_robot()
                    self.return_mgr.transition(RobotState.IDLE)
                    done_msg = Bool(); done_msg.data = True
                    self.pub_mission_done.publish(done_msg)
                elif ch in ('o', 'O'):
                    if self.is_emergency_paused:
                        self.is_emergency_paused = False
                        self.is_initial_turn = True
                        resolve_msg = Bool(); resolve_msg.data = True
                        self.pub_emergency_resolve.publish(resolve_msg)
                        self.get_logger().info("✅ [O키] 비상정지 해제. 주행 재개합니다.")
                        self.return_mgr.start_precheck(RobotState.RUNNING_TO_HOME)
                    else:
                        self.get_logger().info("ℹ️  현재 비상정지 상태가 아닙니다.")
                elif ch == '\x03':
                    break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    # ── /carry_id 수신 트리거 ──
    def trigger_callback(self, msg):
        unload_map = {0: 1, 1: 2, 2: 3}
        if msg.data not in unload_map:
            self.get_logger().error(f"❌ /carry_id 유효하지 않은 값: {msg.data} (0,1,2만 가능)")
            return
        unload_zone = unload_map[msg.data]
        self.get_logger().info(f"🔔 /carry_id 수신: {msg.data} → 상차 LOAD001 / 하차 UNLD00{unload_zone} 미션 시작")
        self.trigger_mission(unload_zone=unload_zone)

    # ── 미션 시작 ──
    def trigger_mission(self, unload_zone: int = 1):
        if self.is_mission_active:
            self.get_logger().warn("⚠️  미션이 이미 진행 중입니다!")
            return
        # 귀환/충전 중이면 미션 시작 차단
        if self.return_mgr.is_going_home():
            self.get_logger().warn(
                f"⚠️  귀환 중 ({self.return_mgr.get_state()}) — 미션 시작 차단")
            return
        if self.is_charging:
            self.get_logger().warn("⚠️  충전 이동 중 — 미션 시작 차단")
            return
        if self.map_data is None or self.current_pose is None:
            self.get_logger().error("❌ 맵 또는 위치 정보가 준비되지 않았습니다!")
            return

        self.current_zone = unload_zone  # 색상 매핑용

        try:
            nav_db = NavigationDBHelper()
            self.rooms, active_map_id, self.waypoints = nav_db.load_rooms(unload_zone)
            self.get_logger().info(
                f"✅ DB 로드 완료 (map_id={active_map_id})\n"
                f"   상차(LOAD)  : {self.rooms['load_1']}\n"
                f"   하차(UNLOAD): {self.rooms[f'unload_{unload_zone}']}\n"
                f"   대기(STANDBY): {self.rooms['standby']}"
            )
        except Exception as e:
            self.get_logger().error(f"❌ DB 로드 실패: {e}")
            return

        self.current_load_key = 'load_1'
        self.current_unload_key = f'unload_{unload_zone}'
        self.return_mgr.build_return_graph(self.waypoints)
        self.return_mgr.update_node_key('load')

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

        self.get_logger().info(f"🚀 미션 시작! [하차 UNLD00{unload_zone}] 상차→하차 5회 왕복.")

        start_msg = Bool(); start_msg.data = True
        self.pub_mission_start.publish(start_msg)

        self.is_mission_active = True
        self.mission_idx = 0
        self.return_mgr.transition(RobotState.MISSION)
        self.set_next_destination(self.mission_sequence[0])

    # ── 경로 탐색 ──
    def set_next_destination(self, goal_pose):
        self.get_logger().info(f"📍 다음 목표: {goal_pose}")

        # [문제3 해결] 귀환 중 일반 미션 A* 재진입 차단
        if self.return_mgr.is_going_home():
            self.get_logger().warn("⚠️  귀환 중 일반 경로 탐색 요청 차단")
            return

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
            # self.get_logger().info(f"경로 탐색 완료! {self.mission_idx+1}/{len(self.mission_sequence)}")
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
        # ══ [문제1 해결] ReturnManager 상태 우선 분기 ══════════════
        # SAFE_EXIT: 모터 명령 완전 차단 (STOP은 ReturnManager 스레드가 담당)
        rm_state = self.return_mgr.get_state()

        # [체크1] SAFE_EXIT/IDLE: Active Braking - cmd_vel=0 즉시 발행
        if rm_state == RobotState.SAFE_EXIT:
            self.stop_robot()   # 관성 방지 즉시 정지
            return
        if rm_state == RobotState.IDLE and not self.is_mission_active                 and not self.is_waypoint_loop and not self.is_charging:
            # 귀환 완료 직후 IDLE 진입 시 한 번 더 stop 보장
            if not self.global_path:
                self.stop_robot()
                return
        # RESUMING: Pre-Check 전용 처리
        if rm_state == RobotState.RESUMING:
            self.return_mgr.check_precheck()
            return
        # ════════════════════════════════════════════════════════════

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
                self.return_mgr.update_node_key('standby')
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
                    self.return_mgr.transition(RobotState.IDLE)
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
        # [문제2 해결] global_path 빈 배열 Race Condition 방지
        if not self.global_path:
            return
        final_goal = self.global_path[-1]
        dist_to_goal = sqrt((final_goal[0]-self.current_pose[0])**2 +
                            (final_goal[1]-self.current_pose[1])**2)

        if dist_to_goal < self.stop_tolerance:
            self.stop_robot()
            cmd_msg = String()

            # ══ [문제4 해결] ReturnManager 귀환 중이면 LOAD/UNLD 발행 차단 ══
            if self.return_mgr.is_going_home():
                self.global_path = []
                if rm_state == RobotState.RUNNING_TO_HOME:
                    self.return_mgr.update_node_key(
                        self.return_mgr._return_sequence[
                            self.return_mgr._return_idx][0]
                        if self.return_mgr._return_sequence else 'standby')
                    self.return_mgr.advance_home()
                else:
                    self.return_mgr.advance_return()
                return
            # ════════════════════════════════════════════════════════════

            # ── 경유 주행 모드 도착 판정 ──
            if self.is_waypoint_loop:
                # 최종 목적지 도착 여부 확인
                # [체크3] global_path 빈 배열 접근 방지
                if not self.global_path:
                    return
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
                    self.return_mgr.transition(RobotState.IDLE)
                    return
                self.return_mgr.update_node_key(f'wp_{self.wp_loop_idx+1}')
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
                self.return_mgr.update_node_key('charge')
                self.return_mgr.transition(RobotState.IDLE)
                return

            current_goal = self.mission_sequence[self.mission_idx]

            # ── 경유 노드 도착: 대기 없이 바로 다음으로 ──
            if any(current_goal == wp for wp in self.waypoints):
                wp_idx = next((i for i, wp in enumerate(self.waypoints) if wp == current_goal), -1)
                if wp_idx >= 0:
                    self.return_mgr.update_node_key(f'wp_{wp_idx+1}')
                self.get_logger().info("🔀 경유 노드 통과! 다음으로 진행")
                self.global_path = []
                self._advance_mission()
                return

            if current_goal == self.rooms[self.current_load_key]:
                # ── 상차 도착 ──
                self.get_logger().info(f"📦 {self.current_load_key} 상차 도착! /carry_done 신호 대기 중...")
                self.current_room_type = 'load'
                self.return_mgr.update_node_key('load')
                self.return_mgr.transition(RobotState.LOADING)
                cmd_msg.data = 'LOAD'
                self.pub_forklift_cmd.publish(cmd_msg)
                self.get_logger().info("[Serial] Sent LOAD command to Arduino")
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
                self.return_mgr.update_node_key('unload')
                self.return_mgr.transition(RobotState.UNLOADING)
                color = self.zone_color_map.get(self.current_zone, 'BLUE')
                self.get_logger().info(f"📦 {self.current_unload_key} 하차 도착! /get_unload → {color} 발행, /done 신호 대기")
                cmd_msg.data = 'UNLD'
                self.pub_forklift_cmd.publish(cmd_msg)
                self.get_logger().info("[Serial] Sent UNLD command to Arduino")
                # ★ 하차 색상(아르코마커) 신호 발행
                unld_msg = String()
                unld_msg.data = color   # 'BLUE' | 'RED' | 'GREEN'
                self.pub_unld_sig.publish(unld_msg)

            elif current_goal == self.rooms['standby']:
                # ── 대기실 도착 ──
                self.current_room_type = 'standby'
                self.return_mgr.update_node_key('standby')
                self.return_mgr.transition(RobotState.IDLE)
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
                if not self.global_path:
                    return
                goal = self.global_path[-1][:]
                self.global_path = []
                # 귀환 중이면 ReturnManager에 재탐색 위임
                if self.return_mgr.is_going_home():
                    self.return_mgr._set_destination_safe(goal, 'stuck_retry')
                else:
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
        # self.get_logger().info(
        #     f"ACT:{action} | F:{f_dist:.2f} L:{l_dist:.2f} R:{r_dist:.2f} "
        #     f"FL:{fl_dist:.2f} FR:{fr_dist:.2f} Ang:{cmd.angular.z:.2f}"
        # )

    # ── 유틸 ──
    def scan_callback(self, msg):
        r = np.array(msg.ranges)
        self.scan_data = np.where(np.isinf(r), 3.5, np.where(np.isnan(r), 3.5, r))
        # LiDAR 신선도 타임스탬프 저장 (Pre-Check용)
        self.scan_data_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9

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