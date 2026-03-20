#!/usr/bin/env python3
"""
FSM Align Line-Tracing Unload Node  v6.6
TurtleBot3 Waffle Pi + Forklift Add-on
ROS2 Humble | Remote PC Control

■ v6.6 변경사항 (v6.5 → v6.6)
  ──────────────────────────────────────────────────────────────────────────
  [1] _s_return: 180° 회전 후 프레임 복구 대기(2프레임) 후 상태 전환
   
  [2] 'RED' HSV: 픽셀 면적 기준 우세 마스크 1개만 선택

■ v6.5 변경사항 (v6.4 → v6.5) — 180° 회전: 시간 기반 + odom yaw 교차 검증 구조로 개선
■ v6.4 변경사항 (v6.2 → v6.4) — 전진 슬립 계측 구조 전면 재설계
■ v6.1 변경사항 (v6 → v6.1) — LiDAR 전처리 강화 + UNLOAD E-STOP 재설계
■ v6 변경사항 (v5r → v6) — 회전 슬립 자동 계측 + UNLOAD_ALIGN 개선

■ 핵심 우선순위 원칙
  1. 엔드포인트 ArUco 마커 인식 → 즉시 완전 정지 → UNLOAD_ALIGN
  2. 마커 미인식 상태에서만 라인트레이싱 수행
  3. RETURN 진입 시 0·1·2번 마커를 엔드포인트 목록에서 완전 제거
  4. returning=True 구간에서는 라인트레이싱 우선, 마커 무시

■ 마커 ID 등록 규칙
  BLUE → 0 / RED → 1 / YELLOW → 2 / 100 은 항상 엔드포인트

■ States
  WAIT           → /get_unload 수신 대기
  IDLE           → 정지 대기
  SEARCH         → 제자리 CW→CCW 라인 탐색 + 회전 슬립 계측
  APPROACH       → 라인트레이싱 / 마커 우선 전진
  APPROACH_ALIGN → 라인 중앙 정렬 (극저속)
  EMERGENCY_STOP → LiDAR 전방 장애물 대기
  UNLOAD_ALIGN   → ArUco 기준 정밀 정렬 (전체 프레임 탐색)
  UNLOAD_STANDBY → 전진(2단계 분기) → fork DOWN → 후진
  RETURN         → 180° 회전(슬립 보정 + yaw 교차 검증) → 귀환
  DONE           → 완료 → WAIT
"""

import math
import threading
import time
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String

# ════════════════════════════════════════════════════════════════════════════
#  모듈 상수
# ════════════════════════════════════════════════════════════════════════════

_QOS_RELIABLE = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=10,
)
_QOS_CMD_VEL = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
)
_QOS_IMG = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=1
)
img_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,   # Gazebo: RELIABLE / 실물: BEST_EFFORT
    durability=DurabilityPolicy.VOLATILE,
    depth=1,
)

COLOR_HSV = {
    'BLUE':   [((100, 80,  80), (130, 255, 255))],
    'RED':    [((0,   80,  80), (10,  255, 255)),
               ((170, 80,  80), (180, 255, 255))],
    'YELLOW': [((14,  95, 130), (24,  255, 255))],
}
COLOR_TO_MARKER_ID: dict = {'BLUE': 0, 'RED': 1, 'YELLOW': 2}


class St:
    WAIT           = 'WAIT'
    IDLE           = 'IDLE'
    SEARCH         = 'SEARCH'
    APPROACH       = 'APPROACH'
    APPROACH_ALIGN = 'APPROACH_ALIGN'
    EMERGENCY_STOP = 'EMERGENCY_STOP'
    UNLOAD_ALIGN   = 'UNLOAD_ALIGN'
    UNLOAD_STANDBY = 'UNLOAD_STANDBY'
    RETURN         = 'RETURN'
    DONE           = 'DONE'


# ════════════════════════════════════════════════════════════════════════════
class FSMUnloadNode(Node):

    def __init__(self) -> None:
        super().__init__('fsm_unload_node')
        self._lock = threading.Lock()
        self._init_params()
        self._init_state()
        self._init_ros()
        threading.Thread(target=self._fsm_loop, daemon=True).start()
        self.get_logger().info(
            f"✅ FSMUnloadNode v6.5 준비 완료 — WAIT 대기 중\n"
            f"   ArUco 처리 주기: {2 * self.ARUCO_PROCESS_EVERY} 프레임마다 1회\n"
            f"   [회전] turn_correction={self.turn_correction:.4f}  "
            f"turn_slip_measured={self.turn_slip_measured}\n"
            f"   [전진] slip_measured={self.slip_measured}"
        )

    # ── 튜닝 상수 ────────────────────────────────────────────────────────────
    def _init_params(self) -> None:

        # 속도
        self.SPD_LINE   = 0.05
        self.SPD_ALIGN  = 0.01
        self.SPD_ANG    = 0.40
        self.SPD_SEARCH = 0.40
        self.SPD_BACK   = 0.10

        # 카메라 / 마커
        self.FRAME_W            = 640
        self.FRAME_H            = 480
        self.ROI_RATIO          = 0.40
        self.MARKER_STOP_BOT_Y  = 336
        self.MARKER_TOO_CLOSE_Y = 470
        self.ALIGN_CX_MIN       = 315
        self.ALIGN_CX_MAX       = 325
        self.ALIGN_TOP_TOL      = 5.0

        # ArUco 프레임 스킵
        self.ARUCO_PROCESS_EVERY = 3

        # LiDAR
        self.LIDAR_DIST            = 0.35
        self.LIDAR_ANGLE           = 30
        self.LIDAR_NARROW_ANGLE    = 5
        self.LIDAR_MEDIAN_N        = 5
        self.UNLOAD_TARGET_DIST    = 0.28
        self.LIDAR_UNLOAD_STOP     = 0.27
        self.LIDAR_UNLOAD_MIN_ODOM = 0.005

        # 회전 슬립 (fallback: 실물 2.0 / Gazebo 1.0)
        self.TURN_CORRECTION          = 2.0
        self.turn_correction          = self.TURN_CORRECTION
        self.turn_slip_measured       = False
        self.TURN_MIN_DEG             = 90.0
        self.TURN_SUPPLEMENT_DEG      = 45.0
        self.TURN_SUPPLEMENT_MIN_DEG  = 10.0  # 추가 계측 기각 기준 (파라미터화)
        self.TURN_SLIP_TOL_DEG        = 5.0   # 계측 오차 경고 기준 (기각 아님)

        # 전진 슬립
        # slip_measured  : True = 2단계 진입 가능
        #                  오직 _unload_phase1_lidar 완료 시에만 True 설정
        # slip_ratio     : 진단용 (1 - UNLOAD_D_odom / E)
        # UNLOAD_t_real  : 1단계 실측 경과 시간 → 2단계 time_target
        # UNLOAD_D_odom  : 1단계 실측 오도메트리 → 2단계 odom_target
        self.slip_measured  = False
        self.slip_ratio     = 0.0
        self.UNLOAD_t_real  = 0.0
        self.UNLOAD_D_odom  = 0.0

        # odom yaw 안정화 대기
        self.ODOM_STABLE_TOL_DEG = 1.0   # 안정화 판정 기준 (°/0.5s)
        self.ODOM_STABLE_TIMEOUT = 5.0   # 안정화 최대 대기 시간 (s)

        # RETURN 교차 검증
        self.RETURN_CORRECT_TOL_DEG = 5.0   # 교차 검증 허용 오차 (°)
        self.RETURN_TRIM_TIMEOUT    = 3.0   # 보정 단계 최대 시간 (s)

    # ── 런타임 상태 ──────────────────────────────────────────────────────────
    def _init_state(self) -> None:

        self.state        = St.WAIT
        self.returning    = False
        self.target_color: Optional[str] = None
        self.endpoint_ids = {100}

        self.line_detected = False
        self.line_cx: Optional[int] = None

        self.marker_found  = False
        self.marker_id: Optional[int]             = None
        self.marker_top_y  = 0.0
        self.marker_bot_y  = 0.0
        self.marker_cx     = 0
        self.marker_tl: Optional[Tuple[float, float]] = None
        self.marker_tr: Optional[Tuple[float, float]] = None

        self.obstacle_front     = False
        self.pre_estop_state    = St.APPROACH
        self._lidar_front_dist  = float('inf')
        self._lidar_narrow_dist = float('inf')

        self._odom_x  = 0.0
        self._odom_y  = 0.0
        self._odom_oz = 0.0
        self._odom_ow = 1.0

        self.fork_done = False

        self._cb_skip_counter    = 0
        self._aruco_skip_counter = 0

        self._last_corners_all: list = []
        self._last_ids_all           = None

        self._ignore_camera_event = threading.Event()
        self._frame_event         = threading.Event()

        self.aruco = _ARUCO_DETECTOR

    # ── ROS2 I/O ─────────────────────────────────────────────────────────────
    def _init_ros(self) -> None:
        self.sub_unload    = self.create_subscription(String,    '/get_unload',  self._cb_unload,    _QOS_RELIABLE)
        self.sub_img       = self.create_subscription(Image,     '/image_raw',   self._cb_image,     _QOS_IMG)
        self.sub_lidar     = self.create_subscription(LaserScan, '/scan',        self._cb_lidar,     qos_profile_sensor_data)
        self.sub_fork_done = self.create_subscription(String,    '/fork_done',   self._cb_fork_done, _QOS_RELIABLE)
        self.sub_odom      = self.create_subscription(Odometry,  '/odom',        self._cb_odom,      qos_profile_sensor_data)

        self.pub_vel  = self.create_publisher(Twist,  '/cmd_vel',  _QOS_CMD_VEL)
        self.pub_fork = self.create_publisher(String, '/fork_cmd', _QOS_RELIABLE)
        self.pub_done = self.create_publisher(String, '/done',     _QOS_RELIABLE)

    # ════════════════════════════════════════════════════════════════════════
    #  ROS 콜백
    # ════════════════════════════════════════════════════════════════════════

    def _cb_unload(self, msg: String) -> None:
        color = msg.data.strip().upper()
        if color not in COLOR_HSV:
            self.get_logger().warn(f"⚠️  알 수 없는 색상: '{color}'")
            return
        if self.state != St.WAIT:
            self.get_logger().warn("⚠️  미션 진행 중 — /get_unload 무시")
            return
        mid = COLOR_TO_MARKER_ID[color]
        with self._lock:
            self.target_color = color
            self.endpoint_ids = {100, mid}
            self.returning    = False
        self.get_logger().info(f"📩 /get_unload: {color} → 마커 ID {mid} → SEARCH")
        self._change_state(St.SEARCH)

    def _cb_image(self, msg: Image) -> None:
        if self._ignore_camera_event.is_set():
            return
        self._cb_skip_counter += 1
        if self._cb_skip_counter % 2 != 0:
            return
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            img = arr.reshape((msg.height, msg.width, -1))
            if msg.encoding == 'rgb8':
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            elif msg.encoding == 'mono8':
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            self._aruco_skip_counter += 1
            do_aruco = (self._aruco_skip_counter % self.ARUCO_PROCESS_EVERY == 0)
            self._process_frame(img.copy(), do_aruco=do_aruco)
        except Exception as exc:
            self.get_logger().error(f"_cb_image 오류: {exc}")

    def _cb_lidar(self, msg: LaserScan) -> None:
        try:
            valid_wide   = self._lidar_valid_ranges(msg, self.LIDAR_ANGLE)
            valid_narrow = self._lidar_valid_ranges(msg, self.LIDAR_NARROW_ANGLE)

            front_dist  = min(valid_wide)   if valid_wide   else float('inf')
            narrow_dist = min(valid_narrow) if valid_narrow else float('inf')
            obstacle    = bool(valid_wide and front_dist < self.LIDAR_DIST)

            with self._lock:
                self._lidar_front_dist  = front_dist
                self._lidar_narrow_dist = narrow_dist

            if obstacle and not self.obstacle_front:
                if self.state in (St.APPROACH, St.APPROACH_ALIGN):
                    with self._lock:
                        self.obstacle_front  = True
                        self.pre_estop_state = self.state
                    self._change_state(St.EMERGENCY_STOP)
            elif not obstacle and self.obstacle_front:
                with self._lock:
                    self.obstacle_front = False

        except Exception as exc:
            self.get_logger().error(f"_cb_lidar 오류: {exc}")

    def _cb_fork_done(self, msg: String) -> None:
        self.get_logger().info(f"🔔 /fork_done 수신: '{msg.data}'")
        with self._lock:
            self.fork_done = True

    def _cb_odom(self, msg: Odometry) -> None:
        with self._lock:
            self._odom_x  = msg.pose.pose.position.x
            self._odom_y  = msg.pose.pose.position.y
            self._odom_oz = msg.pose.pose.orientation.z
            self._odom_ow = msg.pose.pose.orientation.w

    # ════════════════════════════════════════════════════════════════════════
    #  이미지 처리
    # ════════════════════════════════════════════════════════════════════════

    def _process_frame(self, img: np.ndarray, do_aruco: bool = True) -> None:
        h, w = img.shape[:2]
        with self._lock:
            self.FRAME_W = w
            self.FRAME_H = h
        roi_top = int(h * (1.0 - self.ROI_RATIO))

        if do_aruco:
            if self.state == St.UNLOAD_ALIGN:
                self._detect_aruco(img, roi_top_offset=0)
            else:
                self._detect_aruco(img[roi_top:h, :], roi_top_offset=roi_top)
            self._frame_event.set()

        self._detect_line(img[roi_top:h, :], w)

    def _detect_aruco(self, roi: np.ndarray, roi_top_offset: int = 0) -> None:
        """엔드포인트 ID에 해당하는 첫 번째 마커만 처리."""
        corners, ids, _ = self.aruco.detectMarkers(roi)
        self._last_corners_all = corners
        self._last_ids_all     = ids

        found = False
        if ids is not None:
            for i, mid in enumerate(ids.flatten().tolist()):
                if int(mid) not in self.endpoint_ids:
                    continue
                pts            = corners[i][0]
                tl, tr, br, bl = pts
                h_roi          = roi.shape[0]

                def to_orig(pt: np.ndarray) -> Tuple[float, float]:
                    return float(pt[0]), float(pt[1]) + roi_top_offset

                tl_o, tr_o, br_o, bl_o = (to_orig(tl), to_orig(tr),
                                           to_orig(br), to_orig(bl))
                top_y = float(min(tl_o[1], tr_o[1]))
                bot_y = float(max(br_o[1], bl_o[1]))
                cx    = int(np.mean(pts[:, 0]))

                if br[1] >= h_roi - 2 or bl[1] >= h_roi - 2:
                    bot_y = float(roi_top_offset + h_roi)

                with self._lock:
                    self.marker_found = True
                    self.marker_id    = int(mid)
                    self.marker_top_y = top_y
                    self.marker_bot_y = bot_y
                    self.marker_cx    = cx
                    self.marker_tl    = tl_o
                    self.marker_tr    = tr_o
                found = True
                break

            if not found:
                self.get_logger().debug(
                    f"ArUco 감지됨 but 엔드포인트 불일치: "
                    f"{ids.flatten().tolist()} / endpoints={self.endpoint_ids}")

        if not found:
            with self._lock:
                self.marker_found = False

    def _detect_line(self, roi: np.ndarray, frame_w: int) -> None:
        """색상 라인 감지 — RED 2구간 편향 방지: 더 큰 blob 마스크만 사용."""
        if self.target_color is None:
            with self._lock:
                self.line_detected = False
                self.line_cx       = None
            return

        small = cv2.resize(roi, None, fx=0.5, fy=0.5,
                        interpolation=cv2.INTER_LINEAR)
        hsv  = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)

        ranges = COLOR_HSV[self.target_color]
        if len(ranges) == 1:
            lo, hi = ranges[0]
            mask = cv2.inRange(hsv, np.array(lo), np.array(hi))
        else:
            # 각 구간 마스크를 개별 생성, m00 비교 후 우세한 것만 사용
            masks = []
            for lo, hi in ranges:
                m = cv2.inRange(hsv, np.array(lo), np.array(hi))
                masks.append(m)
            areas = [cv2.countNonZero(m) for m in masks]
            # 가장 넓은 마스크 하나만 선택 (편향 방지)
            mask = masks[int(np.argmax(areas))]

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

        M = cv2.moments(mask)
        if M['m00'] < 125:
            with self._lock:
                self.line_detected = False
                self.line_cx       = None
        else:
            cx_small = int(M['m10'] / M['m00'])
            with self._lock:
                self.line_detected = True
                self.line_cx       = cx_small * 2

    # ════════════════════════════════════════════════════════════════════════
    #  헬퍼 — 센서
    # ════════════════════════════════════════════════════════════════════════

    @staticmethod
    def _lidar_valid_ranges(msg: LaserScan, angle_deg: float) -> List[float]:
        """전방 ±angle_deg 범위 유효 거리 목록. nan/inf 차단 후 range 필터."""
        n     = len(msg.ranges)
        step  = msg.angle_increment
        ang   = np.deg2rad(angle_deg)
        idx_r = int(ang / step) % n
        idx_l = int((2 * np.pi - ang) / step) % n
        raw   = list(msg.ranges[:idx_r + 1]) + list(msg.ranges[idx_l:])
        return [r for r in raw
                if math.isfinite(r) and msg.range_min < r < msg.range_max]

    def _get_lidar_narrow_median(self, n_samples: Optional[int] = None) -> float:
        """전방 ±5° 중앙값 반환. 샘플 간 0.13 s 대기 (LDS ≈ 8 Hz)."""
        n_samples = n_samples or self.LIDAR_MEDIAN_N
        samples: List[float] = []
        for _ in range(n_samples):
            with self._lock:
                d = self._lidar_narrow_dist
            if d != float('inf'):
                samples.append(d)
            time.sleep(0.13)

        if not samples:
            self.get_logger().warn("⚠️  _get_lidar_narrow_median: 유효 샘플 없음")
            return float('inf')

        median = float(np.median(samples))
        self.get_logger().info(
            f"📡 LiDAR ±5° 중앙값: {median:.4f} m  ({len(samples)} 샘플)")
        return median

    def _get_odom_pos(self) -> Tuple[float, float]:
        with self._lock:
            return self._odom_x, self._odom_y

    def _odom_dist_from(self, x0: float, y0: float) -> float:
        x, y = self._get_odom_pos()
        return math.sqrt((x - x0) ** 2 + (y - y0) ** 2)

    def _get_yaw(self) -> float:
        with self._lock:
            oz, ow = self._odom_oz, self._odom_ow
        return 2.0 * math.atan2(oz, ow)

    @staticmethod
    def _yaw_diff(yaw_start: float, yaw_end: float) -> float:
        diff = yaw_end - yaw_start
        return math.atan2(math.sin(diff), math.cos(diff))

    def _wait_odom_stable(self) -> bool:
        """
        /odom yaw 값이 안정될 때까지 대기.
        0.5 s 간격으로 yaw 변화량이 ODOM_STABLE_TOL_DEG 이내이면 안정으로 판정.
        ODOM_STABLE_TIMEOUT 초 초과 시 경고 후 False 반환.
        """
        self.get_logger().info(
            f"⏳ /odom yaw 안정화 대기 중 "
            f"(허용 변화량 ±{self.ODOM_STABLE_TOL_DEG:.1f}°/0.5s, "
            f"최대 {self.ODOM_STABLE_TIMEOUT:.1f} s)")

        end = time.time() + self.ODOM_STABLE_TIMEOUT
        while time.time() < end and rclpy.ok():
            y0 = self._get_yaw()
            time.sleep(0.5)
            y1 = self._get_yaw()
            delta_deg = abs(math.degrees(self._yaw_diff(y0, y1)))
            if delta_deg < self.ODOM_STABLE_TOL_DEG:
                self.get_logger().info(
                    f"✅ /odom yaw 안정화 완료  변화량={delta_deg:.2f}°")
                return True
            self.get_logger().debug(
                f"   yaw 변화량={delta_deg:.2f}° — 대기 중")

        self.get_logger().warn(
            f"⚠️  /odom yaw 안정화 타임아웃 ({self.ODOM_STABLE_TIMEOUT:.1f} s) "
            f"— 미안정 상태로 진행")
        return False

    # ── 회전 슬립 계측 ───────────────────────────────────────────────────────

    def _try_commit_turn_slip(self, yaw_start: float,
                              yaw_now: float, cmd_deg: float) -> bool:
        actual_rad = abs(self._yaw_diff(yaw_start, yaw_now))
        actual_deg = math.degrees(actual_rad)

        if actual_deg < self.TURN_MIN_DEG:
            self.get_logger().warn(
                f"⚠️  회전 슬립 계측 기각: 실제 {actual_deg:.1f}° < {self.TURN_MIN_DEG}°")
            return False

        self.turn_correction   = math.radians(cmd_deg) / actual_rad
        self.turn_slip_measured = True

        error_deg = cmd_deg - actual_deg
        if abs(error_deg) > self.TURN_SLIP_TOL_DEG:
            self.get_logger().warn(
                f"⚠️  슬립 계측 오차 큼: 명령 {cmd_deg:.1f}°  실제 {actual_deg:.1f}°  "
                f"오차={error_deg:+.1f}° (허용 ±{self.TURN_SLIP_TOL_DEG:.1f}°)  "
                f"보정 계수={self.turn_correction:.4f}")
        else:
            self.get_logger().info(
                f"📐 회전 슬립 계측 완료  명령 {cmd_deg:.1f}°  실제 {actual_deg:.1f}°  "
                f"오차={error_deg:+.1f}°  보정 계수={self.turn_correction:.4f}")
        return True

    def _measure_turn_slip_supplement(self, yaw_before: float) -> None:
        """조기 라인 발견(90° 미만) 시 CW/CCW 각 45° 추가 계측 후 평균."""
        supp_deg  = self.TURN_SUPPLEMENT_DEG
        supp_rad  = math.radians(supp_deg)
        supp_time = supp_rad / self.SPD_SEARCH

        self.get_logger().info(
            f"📐 조기 발견 — CW +{supp_deg:.0f}° / CCW +{supp_deg:.0f}° 각각 계측 후 평균")

        self._ignore_camera_event.set()

        yaw_cw_start = self._get_yaw()
        self._timed_move(0.0, -self.SPD_SEARCH, supp_time)
        actual_cw_rad  = abs(self._yaw_diff(yaw_cw_start, self._get_yaw()))

        yaw_ccw_start = self._get_yaw()
        self._timed_move(0.0,  self.SPD_SEARCH, supp_time)
        actual_ccw_rad = abs(self._yaw_diff(yaw_ccw_start, self._get_yaw()))

        self._ignore_camera_event.clear()

        actual_cw_deg  = math.degrees(actual_cw_rad)
        actual_ccw_deg = math.degrees(actual_ccw_rad)

        self.get_logger().info(
            f"   CW  계측: 명령 {supp_deg:.1f}°  실제 {actual_cw_deg:.1f}°\n"
            f"   CCW 계측: 명령 {supp_deg:.1f}°  실제 {actual_ccw_deg:.1f}°")

        if (actual_cw_deg  < self.TURN_SUPPLEMENT_MIN_DEG or
                actual_ccw_deg < self.TURN_SUPPLEMENT_MIN_DEG):
            self.get_logger().warn(
                f"⚠️  추가 회전 계측 기각: CW={actual_cw_deg:.1f}° / "
                f"CCW={actual_ccw_deg:.1f}° — "
                f"{self.TURN_SUPPLEMENT_MIN_DEG:.1f}° 미만 존재")
            return

        correction_cw  = supp_rad / actual_cw_rad
        correction_ccw = supp_rad / actual_ccw_rad
        self.turn_correction    = (correction_cw + correction_ccw) / 2.0
        self.turn_slip_measured = True
        self.get_logger().info(
            f"📐 추가 회전 슬립 계측 완료  "
            f"CW={correction_cw:.4f}  CCW={correction_ccw:.4f}  "
            f"평균={self.turn_correction:.4f}")

    # ════════════════════════════════════════════════════════════════════════
    #  헬퍼 — 구동
    # ════════════════════════════════════════════════════════════════════════

    @staticmethod
    def _line_angular(cx: int, frame_w: int, gain: float) -> float:
        return -float(cx - frame_w // 2) / (frame_w / 2) * gain

    @staticmethod
    def _marker_angular(marker_cx: int, frame_w: int, gain: float) -> float:
        return -float(marker_cx - frame_w // 2) / (frame_w / 2) * gain

    def _stop(self) -> None:
        self.pub_vel.publish(Twist())

    def _move(self, lin: float, ang: float) -> None:
        t = Twist()
        t.linear.x  = float(lin)
        t.angular.z = float(ang)
        self.pub_vel.publish(t)

    def _timed_move(self, lin: float, ang: float, duration: float) -> None:
        end = time.time() + duration
        while time.time() < end and rclpy.ok():
            self._move(lin, ang)
            time.sleep(0.05)
        self._stop()

    def _sleep_ok(self, sec: float) -> None:
        end = time.time() + sec
        while time.time() < end and rclpy.ok():
            time.sleep(0.05)

    def _change_state(self, new: str) -> None:
        if self.state != new:
            self.get_logger().info(f"🔄  {self.state} → {new}")
        self.state = new

    # ════════════════════════════════════════════════════════════════════════
    #  FSM 루프
    # ════════════════════════════════════════════════════════════════════════

    def _fsm_loop(self) -> None:
        _DISPATCH = {
            St.WAIT:           self._s_wait,
            St.IDLE:           self._s_idle,
            St.SEARCH:         self._s_search,
            St.APPROACH:       self._s_approach,
            St.APPROACH_ALIGN: self._s_approach_align,
            St.EMERGENCY_STOP: self._s_estop,
            St.UNLOAD_ALIGN:   self._s_unload_align,
            St.UNLOAD_STANDBY: self._s_unload_standby,
            St.RETURN:         self._s_return,
            St.DONE:           self._s_done,
        }
        while rclpy.ok():
            _DISPATCH.get(self.state, self._s_wait)()
            time.sleep(0.05)

    # ════════════════════════════════════════════════════════════════════════
    #  FSM 상태 핸들러
    # ════════════════════════════════════════════════════════════════════════

    def _s_wait(self) -> None:
        time.sleep(0.1)

    def _s_idle(self) -> None:
        self._stop()
        time.sleep(0.1)

    def _s_search(self) -> None:
        """제자리 CW → CCW 각 1바퀴 라인 탐색. CW 구간 회전 슬립 1회 계측."""
        self.get_logger().info(f"🔍 SEARCH — 목표 색상: {self.target_color}")

        # ── odom 안정화 대기 (미계측 시에만) ─────────────────────────────────
        if not self.turn_slip_measured:
            self._wait_odom_stable()

        SEARCH_TURN_FACTOR = 4.0
        one_turn = (SEARCH_TURN_FACTOR * np.pi) / self.SPD_SEARCH

        for direction, label in ((1.0, 'CW'), (-1.0, 'CCW')):
            yaw_start  = self._get_yaw()
            end        = time.time() + one_turn
            do_measure = (label == 'CW' and not self.turn_slip_measured)

            while time.time() < end and rclpy.ok():
                if self.state != St.SEARCH:
                    return
                if self.line_detected:
                    self._stop()
                    if do_measure:
                        yaw_now    = self._get_yaw()
                        actual_deg = abs(math.degrees(
                            self._yaw_diff(yaw_start, yaw_now)))
                        if actual_deg >= self.TURN_MIN_DEG:
                            self._try_commit_turn_slip(yaw_start, yaw_now, actual_deg)
                        else:
                            self._measure_turn_slip_supplement(yaw_now)
                    self.get_logger().info(f"✅ SEARCH({label}): 라인 발견 → APPROACH")
                    self._change_state(St.APPROACH)
                    return
                self._move(0.0, -self.SPD_SEARCH * direction)
                time.sleep(0.05)

            self._stop()
            if do_measure and not self.turn_slip_measured:
                yaw_now    = self._get_yaw()
                actual_deg = abs(math.degrees(self._yaw_diff(yaw_start, yaw_now)))
                self._try_commit_turn_slip(yaw_start, yaw_now, actual_deg)
            self._sleep_ok(0.3)

        self.get_logger().warn("⚠️  SEARCH: 라인 미발견 — 1 s 후 재탐색")
        self._sleep_ok(1.0)

    def _s_approach(self) -> None:
        """
        returning=True  → 라인트레이싱 우선 (귀환 모드)
        returning=False → 마커 인식 최우선, 미인식 시 라인트레이싱
        전진 슬립 계측은 UNLOAD_STANDBY 에서만 수행.
        """
        with self._lock:
            returning     = self.returning
            found         = self.marker_found
            bot_y         = self.marker_bot_y
            marker_cx     = self.marker_cx
            line_detected = self.line_detected
            line_cx       = self.line_cx
            frame_w       = self.FRAME_W

        # ── 귀환 모드 ────────────────────────────────────────────────────────
        if returning:
            if not line_detected:
                self._stop()
                self.get_logger().info("🏁 귀환 중 라인 소실 → DONE")
                self._change_state(St.DONE)
                return
            if abs(line_cx - frame_w // 2) > frame_w * 0.10:
                self._stop()
                self._change_state(St.APPROACH_ALIGN)
                return
            self._move(self.SPD_LINE,
                       self._line_angular(line_cx, frame_w, self.SPD_ANG))
            return

        # ── 마커 우선 모드 ───────────────────────────────────────────────────
        if found:
            self._stop()
            if bot_y >= self.MARKER_TOO_CLOSE_Y:
                self.get_logger().warn(
                    f"↩️  너무 가까움 bot_y={bot_y:.0f} — 후진 1.0 s")
                self._timed_move(-self.SPD_BACK, 0.0, 1.0)
                return
            if bot_y >= self.MARKER_STOP_BOT_Y:
                self.get_logger().info(
                    f"🎯 정렬 범위 도달 bot_y={bot_y:.0f} → UNLOAD_ALIGN")
                self._change_state(St.UNLOAD_ALIGN)
                return
            self._move(self.SPD_LINE,
                       self._marker_angular(marker_cx, frame_w, self.SPD_ANG))
            return

        # ── 라인트레이싱 폴백 ────────────────────────────────────────────────
        if not line_detected:
            self._stop()
            self.get_logger().warn("⚠️  APPROACH: 라인 소실 → SEARCH")
            self._change_state(St.SEARCH)
            return
        if abs(line_cx - frame_w // 2) > frame_w * 0.10:
            self._stop()
            self._change_state(St.APPROACH_ALIGN)
            return
        self._move(self.SPD_LINE,
                   self._line_angular(line_cx, frame_w, self.SPD_ANG))

    def _s_approach_align(self) -> None:
        """라인 중앙 정렬. returning=True 시 마커 체크 생략."""
        with self._lock:
            returning     = self.returning
            found         = self.marker_found
            line_detected = self.line_detected
            line_cx       = self.line_cx
            frame_w       = self.FRAME_W

        if not returning and found:
            self._stop()
            self._change_state(St.APPROACH)
            return
        if not line_detected:
            self._stop()
            self._change_state(St.DONE if returning else St.SEARCH)
            return

        err = line_cx - frame_w // 2
        if abs(err) <= frame_w * 0.05:
            self._stop()
            self.get_logger().info("✅ APPROACH_ALIGN: 정렬 완료 → APPROACH")
            self._change_state(St.APPROACH)
            return
        self._move(self.SPD_ALIGN,
                   self._line_angular(line_cx, frame_w, self.SPD_ANG))

    def _s_estop(self) -> None:
        self._stop()
        with self._lock:
            obstacle = self.obstacle_front
            prev     = self.pre_estop_state
        if not obstacle:
            self.get_logger().info(f"✅ EMERGENCY_STOP 해제 → {prev}")
            self._change_state(prev)

    def _s_unload_align(self) -> None:
        """
        ArUco 기준 정밀 정렬.
        조건 1: TL·TR y 차 ≤ ALIGN_TOP_TOL
        조건 2: 마커 cx ∈ [ALIGN_CX_MIN, ALIGN_CX_MAX]
        전체 프레임 탐색으로 후진 시에도 마커 인식 유지.
        """
        self.get_logger().info("🔧 UNLOAD_ALIGN: 정밀 정렬 시작")
        MAX_ITER      = 40
        FRAME_TIMEOUT = 1.0

        for it in range(MAX_ITER):
            if not rclpy.ok():
                return

            self._ignore_camera_event.set()
            self._frame_event.clear()
            self._sleep_ok(0.05)
            self._ignore_camera_event.clear()

            if not self._frame_event.wait(timeout=FRAME_TIMEOUT):
                self.get_logger().warn(f"⚠️  UNLOAD_ALIGN iter={it}: 프레임 타임아웃")
                continue

            with self._lock:
                found = self.marker_found
                tl    = self.marker_tl
                tr    = self.marker_tr
                cx    = self.marker_cx

            if not found:
                self.get_logger().warn(
                    f"⚠️  UNLOAD_ALIGN iter={it}: 마커 미인식 → 후진 0.2 s")
                self._timed_move(-self.SPD_LINE, 0.0, 0.2)
                continue

            _, tl_y   = tl
            _, tr_y   = tr
            top_delta = abs(tl_y - tr_y)
            top_ok    = top_delta <= self.ALIGN_TOP_TOL
            cx_ok     = self.ALIGN_CX_MIN <= cx <= self.ALIGN_CX_MAX

            self.get_logger().info(
                f"  [iter={it}] top_delta={top_delta:.1f}px(ok={top_ok})  "
                f"cx={cx}(ok={cx_ok})")

            if top_ok and cx_ok:
                self._stop()
                self.get_logger().info("✅ UNLOAD_ALIGN: 조건 충족 → UNLOAD_STANDBY")
                self._change_state(St.UNLOAD_STANDBY)
                return

            if not cx_ok:
                err = cx - self.FRAME_W // 2
                ang = -float(np.sign(err)) * 0.15
                self.get_logger().info(
                    f"  → cx 보정: err={err}px  ang={ang:.2f}  0.25 s")
                self._timed_move(0.0, ang, 0.25)

            if not top_ok:
                tilt = tl_y - tr_y
                ang  = float(np.sign(tilt)) * 0.10
                self.get_logger().info(
                    f"  → 수평 보정: tilt={tilt:.1f}px  ang={ang:.2f}  0.20 s")
                self._timed_move(0.0, ang, 0.20)

        self.get_logger().warn("⚠️  UNLOAD_ALIGN: 최대 반복 → 강제 UNLOAD_STANDBY")
        self._change_state(St.UNLOAD_STANDBY)

    # ────────────────────────────────────────────────────────────────────────
    def _s_unload_standby(self) -> None:
        """
        slip_measured=False → _unload_phase1_lidar()
        slip_measured=True  → _unload_phase2_cross()

        slip_measured 는 오직 _unload_phase1_lidar 완료 시에만 True 설정됨.
        APPROACH / UNLOAD_ALIGN 경로에서는 slip_measured 를 변경하지 않음.
        """
        if not self.slip_measured:
            self._unload_phase1_lidar()
        else:
            self._unload_phase2_cross()

    def _unload_phase1_lidar(self) -> None:
        """
        UNLOAD_ALIGN 완료 직후 B 측정 → LiDAR 28 cm 도달까지 전진.

        B : UNLOAD_STANDBY 진입 직후 LiDAR ±5° 중앙값
            = UNLOAD_ALIGN 완료 시점의 실제 벽까지 거리 (정확한 기준점)
        E : B - UNLOAD_TARGET_DIST (실제로 이동해야 할 거리)

        저장값:
          UNLOAD_t_real = 실측 경과 시간 (슬립 포함) → 2단계 time_target
          UNLOAD_D_odom = 실측 오도메트리             → 2단계 odom_target
          slip_ratio    = 1 - UNLOAD_D_odom / E       → 진단용

        B 유효 범위 밖 → fallback 7 s 전진.
        """
        B = self._get_lidar_narrow_median()

        if B == float('inf') or B <= self.UNLOAD_TARGET_DIST + 0.05:
            self.get_logger().warn(
                f"⚠️  1단계: B={B:.3f} m — 유효 범위 밖 → fallback 7.0 s 전진")
            self._timed_move(self.SPD_LINE, 0.0, 7.0)
            self._do_fork_and_return()
            return

        E = B - self.UNLOAD_TARGET_DIST
        self.get_logger().info(
            f"📦 UNLOAD_STANDBY [1단계]\n"
            f"   B={B:.4f} m  E={E:.4f} m  "
            f"목표 → {self.UNLOAD_TARGET_DIST*100:.0f} cm")

        x0, y0  = self._get_odom_pos()
        t_start = time.time()
        timeout = (E / self.SPD_LINE) * 2.5
        reached = False

        while rclpy.ok():
            elapsed = time.time() - t_start
            with self._lock:
                d_now = self._lidar_narrow_dist
            if d_now <= self.UNLOAD_TARGET_DIST:
                reached = True
                break
            if elapsed >= timeout:
                self.get_logger().warn(
                    f"⚠️  1단계 타임아웃 ({timeout:.1f} s)  LiDAR={d_now:.3f} m")
                break
            self._move(self.SPD_LINE, 0.0)
            time.sleep(0.05)

        self._stop()

        self.UNLOAD_t_real = time.time() - t_start
        self.UNLOAD_D_odom = self._odom_dist_from(x0, y0)
        self.slip_ratio    = (1.0 - self.UNLOAD_D_odom / E) if E > 0.001 else 0.0
        self.slip_measured = True

        self.get_logger().info(
            f"✅ 1단계 완료\n"
            f"   UNLOAD_t_real={self.UNLOAD_t_real:.3f} s\n"
            f"   UNLOAD_D_odom={self.UNLOAD_D_odom:.4f} m\n"
            f"   E={E:.4f} m  slip={self.slip_ratio:.4f}  "
            f"도달: {'성공' if reached else '타임아웃'}")

        self._do_fork_and_return()

    def _unload_phase2_cross(self) -> None:
        """
        UNLOAD_t_real + UNLOAD_D_odom 교차 감시 전진.

        time_target = UNLOAD_t_real  (1단계 실측 경과 시간)
        odom_target = UNLOAD_D_odom  (1단계 실측 오도메트리)
          근거: 동일 슬립 조건 재현 시 오도메트리도 동일하게 재현
               → 포크 끝단 → 벽 6 cm 보장

        정지 우선순위:
          P1. LiDAR < 27 cm + odom > 5 mm → 비상 정지 (과주행 최후 차단)
          P2. elapsed ≥ time_target AND odom ≥ odom_target → 정상 완료
          P3. elapsed ≥ time_target, odom 부족 ≤ 3 cm → 추가 전진
          P4. elapsed ≥ time_target, odom 부족 > 3 cm → 강제 정지
          P5. odom ≥ odom_target, elapsed < time_target → 조기 정지
        """
        time_target = self.UNLOAD_t_real
        odom_target = self.UNLOAD_D_odom

        self.get_logger().info(
            f"📦 UNLOAD_STANDBY [2단계]\n"
            f"   time_target={time_target:.3f} s  "
            f"odom_target={odom_target:.4f} m  "
            f"slip={self.slip_ratio:.4f} (진단용)")

        x0, y0      = self._get_odom_pos()
        t_start     = time.time()
        stop_reason = 'none'

        while rclpy.ok():
            elapsed    = time.time() - t_start
            odom_moved = self._odom_dist_from(x0, y0)
            with self._lock:
                front_dist = self._lidar_narrow_dist

            # P1: 비상 정지 (과주행 최후 차단)
            if (front_dist < self.LIDAR_UNLOAD_STOP
                    and odom_moved > self.LIDAR_UNLOAD_MIN_ODOM):
                stop_reason = 'lidar_estop'
                self.get_logger().warn(
                    f"🚨 UNLOAD E-STOP: LiDAR={front_dist:.3f} m  "
                    f"포크 끝단 → 벽 ≈ {(front_dist - 0.22)*100:.1f} cm  "
                    f"odom={odom_moved:.4f} m")
                break

            # P2: 정상 완료
            if elapsed >= time_target and odom_moved >= odom_target:
                stop_reason = 'normal'
                self.get_logger().info(
                    f"✅ 정상 완료: elapsed={elapsed:.3f} s  "
                    f"odom={odom_moved:.4f} m")
                break

            # P3/P4: 시간 초과
            if elapsed >= time_target:
                gap = odom_target - odom_moved
                if gap > 0.03:
                    stop_reason = 'odom_gap_exceed'
                    self.get_logger().warn(
                        f"⚠️  odom 부족 gap={gap*100:.1f} cm > 3 cm → 강제 정지")
                    break
                self._move(self.SPD_LINE, 0.0)
                time.sleep(0.05)
                continue

            # P5: 조기 정지
            if odom_moved >= odom_target:
                stop_reason = 'odom_early'
                self.get_logger().info(
                    f"✅ 조기 odom 달성: {odom_moved:.4f} m / {elapsed:.3f} s")
                break

            self._move(self.SPD_LINE, 0.0)
            time.sleep(0.05)

        self._stop()
        self.get_logger().info(f"🏁 2단계 완료 — 정지 사유: {stop_reason}")
        self._do_fork_and_return()

    # ────────────────────────────────────────────────────────────────────────
    def _do_fork_and_return(self) -> None:
        """포크 DOWN → /fork_done 대기 → 후진 5 s → RETURN."""
        FORK_TIMEOUT = 10.0
        with self._lock:
            self.fork_done = False

        msg      = String()
        msg.data = 'DOWN'
        self.pub_fork.publish(msg)
        self.get_logger().info("🔽 /fork_cmd: DOWN 발행")

        wait_start = time.time()
        while rclpy.ok():
            with self._lock:
                done = self.fork_done
            if done:
                self.get_logger().info("✅ /fork_done 수신")
                break
            if time.time() - wait_start > FORK_TIMEOUT:
                self.get_logger().warn(
                    f"⚠️  /fork_done 타임아웃 ({FORK_TIMEOUT:.0f} s) — 강제 진행")
                break
            time.sleep(0.05)

        self.get_logger().info("↩️  후진 5.0 s")
        self._timed_move(-self.SPD_LINE, 0.0, 5.0)
        self._change_state(St.RETURN)

    def _s_return(self) -> None:
        """
        180° 회전 후 귀환.
        1차: 슬립 보정 시간 기반 회전 (주 제어)
        2차: odom yaw 교차 검증 → 잔여 오차 보정 (보완)
             - 오차 ≤ ±RETURN_CORRECT_TOL_DEG → 보정 불필요
             - 오차 > 0 (부족)   → 1차와 같은 방향, yaw 도달 시 즉시 정지
             - 오차 < 0 (과회전) → 1차와 반대 방향, yaw 도달 시 즉시 정지
        """
        with self._lock:
            self.endpoint_ids -= {0, 1, 2}
            self.returning = True
        self.get_logger().info(
            f"🗑️  엔드포인트 0·1·2 제거 — 잔여: {self.endpoint_ids}  [RETURNING ON]")

        TARGET_RAD   = np.pi
        CORRECT_TOL  = math.radians(self.RETURN_CORRECT_TOL_DEG)
        TRIM_TIMEOUT = 15.0   # 최대 180° @ 0.40 rad/s ≈ 7.85 s 를 충분히 커버
        TRIM_TARGET  = TARGET_RAD * 0.97   # 174.6° 이상이면 완료 판정

        correction = (self.turn_correction
                      if self.turn_slip_measured
                      else self.TURN_CORRECTION)
        half_turn  = (TARGET_RAD / self.SPD_ANG) * correction

        self.get_logger().info(
            f"↩️  RETURN: 180° 회전  보정 계수={correction:.4f}  "
            f"명령 시간={half_turn:.2f} s  "
            f"({'실측' if self.turn_slip_measured else 'fallback'})")

        # ── 1차: 슬립 보정 시간 기반 회전 ────────────────────────────────────
        yaw_start = self._get_yaw()

        self._ignore_camera_event.set()
        self._timed_move(0.0, self.SPD_ANG, half_turn)

        # ── 2차: odom yaw 교차 검증 ──────────────────────────────────────────
        yaw_after      = self._get_yaw()
        signed_rotated = self._yaw_diff(yaw_start, yaw_after)   # 부호 보존
        rotated        = abs(signed_rotated)
        turn_dir       = 1.0 if signed_rotated >= 0 else -1.0   # 1차 회전 방향
        error_rad      = TARGET_RAD - rotated                    # 양수=부족 / 음수=과회전
        error_deg      = math.degrees(error_rad)

        self.get_logger().info(
            f"📐 교차 검증: 목표={math.degrees(TARGET_RAD):.1f}°  "
            f"실측={math.degrees(rotated):.1f}°  "
            f"오차={error_deg:+.1f}°")

        if abs(error_rad) <= CORRECT_TOL:
            self.get_logger().info(
                f"✅ 교차 검증 통과 — 보정 불필요 "
                f"(허용 ±{self.RETURN_CORRECT_TOL_DEG:.1f}°)")

        else:
            # 부족 → 1차와 같은 방향 / 과회전 → 1차와 반대 방향
            trim_dir    = turn_dir if error_rad > 0 else -turn_dir
            t_trim_end  = time.time() + TRIM_TIMEOUT

            self.get_logger().info(
                f"🔧 보정 회전: {'부족 → 추가' if error_rad > 0 else '과회전 → 역방향'}  "
                f"오차={error_deg:+.1f}°  "
                f"최대 대기={TRIM_TIMEOUT:.1f} s")

            # yaw 도달 시 즉시 정지 / TRIM_TIMEOUT 초과 시 안전 정지
            while time.time() < t_trim_end and rclpy.ok():
                current_rotated = abs(self._yaw_diff(yaw_start, self._get_yaw()))
                if current_rotated >= TRIM_TARGET:
                    self.get_logger().info(
                        f"✅ 보정 중 목표 yaw 도달  실측={math.degrees(current_rotated):.1f}°")
                    break
                self._move(0.0, self.SPD_ANG * trim_dir)
                time.sleep(0.05)
            else:
                self.get_logger().warn(
                    f"⚠️  보정 TRIM_TIMEOUT ({TRIM_TIMEOUT:.1f} s) 초과 — 강제 정지")

            self._stop()

            yaw_final   = self._get_yaw()
            rotated_fin = abs(self._yaw_diff(yaw_start, yaw_final))
            self.get_logger().info(
                f"✅ 보정 완료: 최종 실측={math.degrees(rotated_fin):.1f}°")

        self._ignore_camera_event.clear()

        # ── 카메라 프레임 안정화 대기 (race condition 방지) ──────────────────
        self.get_logger().info("⏳ 카메라 프레임 안정화 대기 (2프레임)...")
        for _ in range(2):
            self._frame_event.clear()
            self._frame_event.wait(timeout=1.0)

        self.get_logger().info("✅ RETURN: 회전 완료 → 귀환")
        self._change_state(St.APPROACH)

    def _s_done(self) -> None:
        self.get_logger().info("🏁 DONE — 미션 완료")
        with self._lock:
            self.target_color = None
            self.endpoint_ids = {100}
            self.returning    = False
            self.marker_found = False
        self._stop()
        msg      = String()
        msg.data = 'Done'
        self.pub_done.publish(msg)
        self._change_state(St.IDLE)
        time.sleep(0.1)
        self._stop()
        self._change_state(St.WAIT)


# ════════════════════════════════════════════════════════════════════════════
#  ArUco 검출기 싱글턴
# ════════════════════════════════════════════════════════════════════════════
def _build_aruco_detector() -> cv2.aruco.ArucoDetector:
    _dict   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    _params = cv2.aruco.DetectorParameters()
    _params.adaptiveThreshWinSizeMax    = 7
    _params.adaptiveThreshWinSizeStep   = 4
    _params.minMarkerPerimeterRate      = 0.02
    _params.polygonalApproxAccuracyRate = 0.05
    return cv2.aruco.ArucoDetector(_dict, _params)


_ARUCO_DETECTOR = _build_aruco_detector()


# ════════════════════════════════════════════════════════════════════════════
def main(args=None) -> None:
    rclpy.init(args=args)
    node = FSMUnloadNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()