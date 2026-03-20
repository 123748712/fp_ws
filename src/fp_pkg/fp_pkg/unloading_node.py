#!/usr/bin/env python3
"""
FSM Align Line-Tracing Unload Node  v4
TurtleBot3 Waffle Pi + Forklift Add-on
ROS2 Humble | Remote PC Control

■ v4 변경사항 (v3 → v4) — 4.5 mm × 4.5 mm ArUco 마커 기준 영상처리 최적화
  ──────────────────────────────────────────────────────────────────────────
  [최적화 근거]
    TurtleBot3 Waffle Pi Camera V2 (640×480, 수평 FOV ≈ 62.2°)
    추정 초점거리 fx ≈ 530.5 px

    거리별 마커 픽셀 크기 (4.5 mm 마커):
      10 cm → ~24 px  |  15 cm → ~16 px  |  20 cm → ~12 px
      30 cm →  ~8 px  |  50 cm →  ~5 px

    DICT_4X4 최소 인식 픽셀: ~18 px (6×6 비트 구조, 셀당 최소 3 px)
    → 유효 인식 거리: 13 cm 이내 (scale=1.0 기준)

  [결론: 다운스케일(scale<1.0) 사용 불가]
    scale=0.5 적용 시 15 cm 거리에서 마커가 8 px → 인식 실패
    → _detect_aruco 는 반드시 원본 해상도(640×480) 유지

  [적용된 최적화 수단 4가지]
  최적화 A  _cb_image         : 전체 프레임 스킵 (2프레임마다 1회 처리)
  최적화 B  _detect_aruco     : ArUco 전용 프레임 스킵 (ARUCO_PROCESS_EVERY=3)
                                → 실질적으로 6프레임마다 1회 ArUco 연산
  최적화 C  _detect_aruco     : 입력 이미지를 하단 ROI 크롭(640×192)으로 제한
                                → 원본 해상도 유지 + 처리 픽셀 수 60% 감소
                                → ROI 내 y 좌표를 원본 좌표로 보정(+roi_top)
  최적화 D  _detect_aruco     : DetectorParameters 튜닝
                                  adaptiveThreshWinSizeMax  23 → 7
                                  adaptiveThreshWinSizeStep 10 → 4
                                  minMarkerPerimeterRate  0.03 → 0.02 (작은 마커 허용)
                                  polygonalApproxAccuracyRate 0.03 → 0.05
  최적화 E  _detect_line      : 라인 감지용 ROI를 scale=0.5 로 다운스케일
                                (색상 blob 감지는 해상도에 비교적 둔감)
                                → cx 계산 후 원본 좌표로 역보정(×2)

  ──────────────────────────────────────────────────────────────────────────
  [주석] _process_frame 의 '''...''' 블록은 의도적으로 남겨둔 비활성 코드가 아닌
         실수로 포함된 다중행 문자열입니다. v4에서도 동일하게 # 로 변환합니다.

■ 핵심 우선순위 원칙
  1. 엔드포인트 ArUco 마커 인식 → 즉시 완전 정지 → UNLOAD_ALIGN
  2. 마커 미인식 상태에서만 라인트레이싱 수행
  3. RETURN 진입 시 0·1·2번 마커를 엔드포인트 목록에서 완전 제거
  4. returning=True 구간(귀환 중)에서는 라인트레이싱 우선, 마커 무시

■ 마커 ID 등록 규칙
  - /get_unload 메세지로만 엔드포인트 활성화
  - BLUE  수신 → ID 0 추가
  - RED   수신 → ID 1 추가
  - YELLOW 수신 → ID 2 추가
  - RETURN 진입 → ID 0, 1, 2 모두 제거

■ States
  IDLE           → /get_unload 수신 대기
  SEARCH         → 제자리 CW 1바퀴 → CCW 1바퀴, 색상 라인 탐색
  APPROACH       → 라인트레이싱 저속(0.05 m/s), 마커 감지 시 즉시 정지
                   (returning=True 시 라인 우선)
  APPROACH_ALIGN → 라인 중앙 ±5% 벗어날 때 극저속(0.01 m/s) 정렬
                   (returning=True 시 마커 체크 생략)
  EMERGENCY_STOP → LiDAR 전방 장애물, 해제될 때까지 정지
  UNLOAD_ALIGN   → ArUco 기준 3단계 정밀 정렬
  UNLOAD_STANDBY → 전진 7 s → fork DOWN → 후진 5 s
  RETURN         → 180° 회전 → 라인트레이싱 귀환
  DONE           → 귀환 완료 로그 → IDLE
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from rclpy.qos import (qos_profile_sensor_data, QoSProfile,
                       ReliabilityPolicy, HistoryPolicy, DurabilityPolicy)

import cv2
import numpy as np
import threading
import time

reliable_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=10
)

cmd_vel_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST
)

img_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,  # 발행 측도 BEST_EFFORT 이어야 연결됨
    durability=DurabilityPolicy.VOLATILE,
    depth=1   # 최신 프레임 1개만 유지 → 큐 누적 방지
)

# ── 색상 HSV 범위 ─────────────────────────────────────────────────────────────
COLOR_HSV = {
    'BLUE':  [((100, 80,  80), (130, 255, 255))],
    'RED':   [((0,   80,  80), (10,  255, 255)),
              ((170, 80,  80), (180, 255, 255))],
    #'YELLOW': [((20, 150, 150), (35, 255, 255))],

    # 실측: H=170~178, S=124~251, V=62~109 (어두운 버건디)
    # H:0~10도 함께 유지 (wrap-around 안전망)
    # V 하한을 40으로 낮춰 어두운 테이프 포함
    #'RED':    [((0,   80,  40), (10,  255, 255)),
    #           ((165, 80,  40), (180, 255, 255))],

    # 실측: H=15~22, S=106~192, V=175~204
    # 현재 설정(S:150, V:150)이 픽셀 수 적었던 이유: S 하한이 약간 높았음
    # S:95, V:130으로 낮춰 안정적으로 감지
    'YELLOW': [((14, 95, 130), (24, 255, 255))],
}
COLOR_TO_MARKER_ID = {'BLUE': 0, 'RED': 1, 'YELLOW': 2}


# ── FSM 상태 ──────────────────────────────────────────────────────────────────
class St:
    WAIT            = 'WAIT'
    IDLE            = 'IDLE'
    SEARCH          = 'SEARCH'
    APPROACH        = 'APPROACH'
    APPROACH_ALIGN  = 'APPROACH_ALIGN'
    EMERGENCY_STOP  = 'EMERGENCY_STOP'
    UNLOAD_ALIGN    = 'UNLOAD_ALIGN'
    UNLOAD_STANDBY  = 'UNLOAD_STANDBY'
    RETURN          = 'RETURN'
    DONE            = 'DONE'


class FSMUnloadNode(Node):

    # ════════════════════════════════════════════════════════════════════════
    #  초기화
    # ════════════════════════════════════════════════════════════════════════
    def __init__(self):
        super().__init__('fsm_unload_node')

        # ── 속도 파라미터 ────────────────────────────────────────────────────
        self.SPD_LINE     = 0.05   # APPROACH 라인트레이싱 전진 속도
        self.SPD_ALIGN    = 0.01   # APPROACH_ALIGN 극저속
        self.SPD_ANG      = 0.40   # 일반 회전 속도
        self.SPD_SEARCH   = 0.40   # SEARCH 회전 속도
        self.SPD_BACK     = 0.10   # 후진 속도

        # ── 이미지 / 마커 파라미터 ───────────────────────────────────────────
        self.FRAME_W             = 640
        self.FRAME_H             = 480
        self.ROI_RATIO           = 0.40  # 하단 40% ROI (라인 감지 + ArUco 공용)

        # 마커 거리 판단은 하단(BR/BL) y 좌표 기준으로 통일
        # → 마커 상단이 프레임 밖으로 잘려도 안정적으로 동작
        self.MARKER_STOP_BOT_Y   = 336   # 마커 하단 y >= 이 값이면 정지 → UNLOAD_ALIGN
        self.MARKER_TOO_CLOSE_Y  = 470   # 마커 하단 y >= 이 값이면 너무 가까움 → 후진
        self.ALIGN_CX_MIN        = 315   # 정렬 완료 중심 x 범위
        self.ALIGN_CX_MAX        = 325
        self.ALIGN_TOP_TOL       = 5.0   # 상단 꼭짓점 y 차 허용 (px)

        # ── LiDAR 파라미터 ───────────────────────────────────────────────────
        self.LIDAR_DIST   = 0.35   # 장애물 거리 임계 (m)
        self.LIDAR_ANGLE  = 30     # 전방 ± 각도 (deg)

        # ── [최적화 A/B] 프레임 스킵 카운터 설정 ────────────────────────────
        # _cb_image  : 2프레임마다 1회 처리 (전체 스킵)
        # _detect_aruco : ARUCO_PROCESS_EVERY 프레임마다 1회 ArUco 연산
        #
        # 결과: ArUco 연산 주기 = 2 × ARUCO_PROCESS_EVERY 프레임마다 1회
        #       30 fps 기준 → 2×3=6 프레임마다 1회 ≈ 5 Hz
        #
        # 주의: ARUCO_PROCESS_EVERY 를 높이면 UNLOAD_ALIGN 의
        #       _frame_event.wait(timeout=1.0) 대기 시간이 길어질 수 있음.
        #       timeout 은 (2 × ARUCO_PROCESS_EVERY / fps) × 3 배 이상으로 유지.
        self._cb_skip_counter    = 0
        self._aruco_skip_counter = 0
        self.ARUCO_PROCESS_EVERY = 3   # ArUco 전용 스킵 주기 (조정 가능: 2~5)

        # ── FSM 상태 변수 ────────────────────────────────────────────────────
        self.state          = St.WAIT
        self.returning      = False   # RETURN 맥락 플래그
        self.target_color   = None    # 현재 미션 색상
        # 100번은 항상 엔드포인트. 미션 색상에 따라 0·1·2가 추가됨.
        self.endpoint_ids   = {100}   # 현재 활성 엔드포인트 마커 ID 집합

        # ── 이미지 처리 결과 (콜백 → FSM 공유) ──────────────────────────────
        self.line_detected  = False
        self.line_cx        = None

        self.marker_found   = False   # 엔드포인트 마커 인식 여부
        self.marker_id      = None
        self.marker_top_y   = 0.0
        self.marker_bot_y   = 0.0
        self.marker_cx      = 0
        self.marker_tl      = None    # (x, y)
        self.marker_tr      = None    # (x, y)

        # ── E-STOP ───────────────────────────────────────────────────────────
        self.obstacle_front  = False
        self.pre_estop_state = St.APPROACH

        # ── 포크 완료 플래그 (/fork_done 수신 시 True) ───────────────────────
        self.fork_done = False

        # ── ignore_camera → threading.Event ─────────────────────────────────
        # set()   : 카메라 콜백 억제 (ignore 상태)
        # clear() : 카메라 콜백 허용 (정상 상태)
        self._ignore_camera_event = threading.Event()

        # ── 새 프레임(ArUco 포함) 수신 확인용 Event ──────────────────────────
        # _detect_aruco 실행 완료 시 set() → UNLOAD_ALIGN 에서 wait() 후 clear()
        self._frame_event = threading.Event()

        self._last_corners_all = []
        self._last_ids_all     = None

        # ── ArUco ────────────────────────────────────────────────────────────
        # DICT_4X4_250: ID 0~249 지원 (ID 100번 사용 가능)
        _dict   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        _params = cv2.aruco.DetectorParameters()

        # [최적화 D] DetectorParameters 튜닝 — 4.5 mm 소형 마커 대응 ─────────
        #
        # adaptiveThreshWinSizeMax: 적응형 이진화 탐색 창 최대 크기 감소
        #   기본값 23 → 7
        #   이유: 소형 마커는 넓은 탐색 창이 불필요, 창 수 감소로 속도 향상
        #   공식: 탐색 창 = range(Min, Max+1, Step) → (3→7, step 4) = [3, 7] 2회
        #         원래 (3→23, step 10) = [3, 13, 23] 3회 → 33% 감소
        _params.adaptiveThreshWinSizeMax  = 7
        _params.adaptiveThreshWinSizeStep = 4

        # minMarkerPerimeterRate: 마커 최소 둘레 비율 (프레임 대각선 기준)
        #   기본값 0.03 → 0.02
        #   이유: 4.5 mm 마커는 멀면 매우 작음 → 더 작은 마커도 후보로 허용
        #   주의: 너무 낮추면 노이즈 오인식 증가 → 0.02가 합리적 하한
        _params.minMarkerPerimeterRate    = 0.02

        # polygonalApproxAccuracyRate: 꼭짓점 근사 허용 오차
        #   기본값 0.03 → 0.05
        #   이유: 소형 마커의 픽셀 경계가 부정확할 수 있음 → 느슨하게 허용
        _params.polygonalApproxAccuracyRate = 0.05

        self.aruco = cv2.aruco.ArucoDetector(_dict, _params)

        # ── 공유 변수 보호 락 ────────────────────────────────────────────────
        self._lock = threading.Lock()

        # ── ROS2 I/O ─────────────────────────────────────────────────────────
        self.sub_unload    = self.create_subscription(
            String,    '/get_unload',  self._cb_unload,    reliable_profile)
        self.sub_img       = self.create_subscription(
            Image,     '/image_raw',   self._cb_image,     img_qos)
        self.sub_lidar     = self.create_subscription(
            LaserScan, '/scan',        self._cb_lidar,     qos_profile_sensor_data)
        self.sub_fork_done = self.create_subscription(
            String,    '/fork_done',   self._cb_fork_done, reliable_profile)

        self.pub_vel  = self.create_publisher(Twist,  '/cmd_vel',  cmd_vel_qos)
        self.pub_fork = self.create_publisher(String, '/fork_cmd', reliable_profile)
        self.pub_done = self.create_publisher(String, '/done',     reliable_profile)

        # ── FSM 메인 루프 스레드 ─────────────────────────────────────────────
        threading.Thread(target=self._fsm_loop, daemon=True).start()

        self.get_logger().info(
            "✅ FSMUnloadNode v4 준비 완료 — WAIT 대기 중\n"
            "   /get_unload 에 BLUE / RED / YELLOW 을 발행하면 미션 시작\n"
            f"   ArUco 처리 주기: 콜백 스킵×2 × ArUco 스킵×{self.ARUCO_PROCESS_EVERY}"
            f" = {2 * self.ARUCO_PROCESS_EVERY} 프레임마다 1회")

    # ════════════════════════════════════════════════════════════════════════
    #  ROS 콜백
    # ════════════════════════════════════════════════════════════════════════

    def _cb_unload(self, msg: String):
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

        self.get_logger().info(
            f"📩 /get_unload: {color} → 마커 ID {mid} 등록 → SEARCH")
        self._change_state(St.SEARCH)

    def _cb_image(self, msg: Image):
        # ── ignore_camera_event 억제 체크 ───────────────────────────────────
        if self._ignore_camera_event.is_set():
            return

        # ── [최적화 A] 전체 프레임 스킵: 2프레임마다 1회 처리 ───────────────
        # 이유: 카메라 30 fps → 콜백 처리 주기를 15 fps 로 낮춰 CPU 부하 50% 감소
        # 라인 감지(blob)와 ArUco 모두 15 fps 이하에서도 충분히 동작
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

            # [최적화 B] ArUco 전용 프레임 스킵 여부 결정
            # _cb_skip_counter 는 짝수만 여기 도달하므로 //2 로 ArUco 카운터 관리
            self._aruco_skip_counter += 1
            do_aruco = (self._aruco_skip_counter % self.ARUCO_PROCESS_EVERY == 0)

            self._process_frame(img.copy(), do_aruco=do_aruco)
        except Exception as e:
            self.get_logger().error(f"_cb_image 오류: {e}")

    def _cb_lidar(self, msg: LaserScan):
        try:
            n     = len(msg.ranges)
            step  = msg.angle_increment
            ang   = np.deg2rad(self.LIDAR_ANGLE)
            idx_r = int(ang / step) % n
            idx_l = int((2 * np.pi - ang) / step) % n
            front = list(msg.ranges[:idx_r + 1]) + list(msg.ranges[idx_l:])
            valid = [r for r in front if msg.range_min < r < msg.range_max]
            obstacle = bool(valid and min(valid) < self.LIDAR_DIST)

            if obstacle and not self.obstacle_front:
                if self.state in (St.APPROACH, St.APPROACH_ALIGN):
                    with self._lock:
                        self.obstacle_front  = True
                        self.pre_estop_state = self.state
                    self._change_state(St.EMERGENCY_STOP)
            elif not obstacle and self.obstacle_front:
                with self._lock:
                    self.obstacle_front = False
        except Exception as e:
            self.get_logger().error(f"_cb_lidar 오류: {e}")

    def _cb_fork_done(self, msg: String):
        """Pi 측 아두이노 노드가 포크 동작 완료 후 발행하는 /fork_done 수신"""
        self.get_logger().info(f"🔔 /fork_done 수신: '{msg.data}'")
        with self._lock:
            self.fork_done = True

    # ════════════════════════════════════════════════════════════════════════
    #  이미지 처리
    # ════════════════════════════════════════════════════════════════════════

    def _process_frame(self, img: np.ndarray, do_aruco: bool = True):
        h, w = img.shape[:2]
        with self._lock:
            self.FRAME_W = w
            self.FRAME_H = h

        roi_top = int(h * (1.0 - self.ROI_RATIO))  # 하단 40% ROI 시작 y

        # 1. ArUco 감지 (do_aruco=True 일 때만)
        # [최적화 C] 하단 ROI 크롭 이미지를 전달 (원본 해상도 유지)
        # → roi_top 을 함께 전달해 y 좌표 보정에 사용
        if do_aruco:
            self._detect_aruco(img[roi_top:h, :], roi_top_offset=roi_top)
            # [수정 6·10] ArUco 처리 완료 신호 — UNLOAD_ALIGN 에서 wait()로 수신 확인
            # ArUco 스킵 프레임에서는 set() 하지 않음
            # → UNLOAD_ALIGN 은 반드시 ArUco 가 실행된 프레임을 기다림
            self._frame_event.set()

        # 2. 색상 라인 감지 (하단 40% ROI)
        # [최적화 E] 라인 감지용 ROI는 0.5 다운스케일 적용
        # 이유: 색상 blob 감지는 픽셀 정밀도보다 무게중심(cx) 위치가 중요하므로
        #       해상도를 절반으로 줄여도 cx 정확도 유지 가능 (역보정: cx × 2)
        # 효과: ROI(640×192) → 다운스케일(320×96), 처리 픽셀 75% 감소
        self._detect_line(img[roi_top:h, :], w)

    def _detect_aruco(self, roi: np.ndarray, roi_top_offset: int = 0):
        """
        [최적화 C] 하단 ROI 크롭 이미지로 ArUco 감지
          - roi        : 원본에서 크롭된 하단 40% 이미지 (640 × ~192 px)
          - roi_top_offset: 원본 프레임 기준 roi 시작 y 좌표
                            → 감지 결과 y 좌표에 이 값을 더해 원본 좌표로 보정

        [최적화 D] DetectorParameters 튜닝은 __init__ 에서 이미 적용됨
        반드시 선/오버레이가 없는 원본 이미지로 호출할 것.
        감지된 corners/ids 를 self._last_corners_all 에 저장해 시각화에 재사용.
        """
        corners, ids, _ = self.aruco.detectMarkers(roi)

        # 시각화용으로 저장 (debug 이미지에 재감지하지 않기 위해)
        # 주의: corners 의 y 좌표는 ROI 기준이므로 시각화에서 별도 보정 필요
        self._last_corners_all = corners
        self._last_ids_all     = ids

        found = False
        if ids is not None:
            all_ids = ids.flatten().tolist()
            for i, mid in enumerate(all_ids):
                if int(mid) not in self.endpoint_ids:
                    continue
                pts            = corners[i][0]   # TL, TR, BR, BL (ROI 좌표)
                tl, tr, br, bl = pts
                h_roi          = roi.shape[0]

                # [최적화 C] ROI 내 y 좌표 → 원본 프레임 y 좌표로 보정
                # ROI 크롭 좌표 + roi_top_offset = 원본 절대 좌표
                tl_orig = (float(tl[0]), float(tl[1]) + roi_top_offset)
                tr_orig = (float(tr[0]), float(tr[1]) + roi_top_offset)
                br_orig = (float(br[0]), float(br[1]) + roi_top_offset)
                bl_orig = (float(bl[0]), float(bl[1]) + roi_top_offset)

                top_y = float(min(tl_orig[1], tr_orig[1]))
                bot_y = float(max(br_orig[1], bl_orig[1]))
                cx    = int(np.mean(pts[:, 0]))   # x 좌표는 보정 불필요

                # 마커가 ROI 하단 밖으로 잘린 경우 bot_y를 원본 프레임 높이로 보정
                # (잘리면 실제보다 작게 나옴 → 프레임 하단을 하한으로 사용)
                h_orig = roi_top_offset + h_roi
                if br[1] >= h_roi - 2 or bl[1] >= h_roi - 2:
                    bot_y = float(h_orig)

                with self._lock:
                    self.marker_found = True
                    self.marker_id    = int(mid)
                    self.marker_top_y = top_y
                    self.marker_bot_y = bot_y
                    self.marker_cx    = cx
                    self.marker_tl    = tl_orig
                    self.marker_tr    = tr_orig
                found = True
                break
            if not found:
                self.get_logger().debug(
                    f"ArUco 감지됨 but 엔드포인트 불일치: {all_ids} "
                    f"/ endpoints={self.endpoint_ids}")
        if not found:
            with self._lock:
                self.marker_found = False

    def _detect_line(self, roi: np.ndarray, frame_w: int):
        """
        [최적화 E] 라인 감지용 ROI를 scale=0.5 다운스케일로 처리
          - 색상 blob 감지는 픽셀 정밀도보다 무게중심 위치가 중요
          - 다운스케일 후 cx 계산 → 원본 좌표로 역보정 (cx × 2)
          - 처리 픽셀 수: 640×192 → 320×96 (75% 감소)
          - 임계값 m00 도 스케일²에 맞게 조정: 500 → 125 (= 500 / 4)
        """
        if self.target_color is None:
            with self._lock:
                self.line_detected = False
                self.line_cx       = None
            return

        # 0.5 다운스케일 (라인 감지 전용)
        small = cv2.resize(roi, None, fx=0.5, fy=0.5,
                           interpolation=cv2.INTER_LINEAR)

        hsv  = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
        mask = np.zeros(small.shape[:2], dtype=np.uint8)
        for lo, hi in COLOR_HSV[self.target_color]:
            mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        # 다운스케일로 픽셀 수가 1/4 감소 → 임계값도 1/4 로 조정
        M = cv2.moments(mask)
        if M['m00'] < 125:   # 원래 500 → 0.5² = 0.25 배
            with self._lock:
                self.line_detected = False
                self.line_cx       = None
        else:
            # 다운스케일 cx → 원본 cx 역보정: cx_small × 2
            cx_small = int(M['m10'] / M['m00'])
            with self._lock:
                self.line_detected = True
                self.line_cx       = cx_small * 2   # 원본 해상도 cx 복원

    # ════════════════════════════════════════════════════════════════════════
    #  헬퍼
    # ════════════════════════════════════════════════════════════════════════

    def _change_state(self, new: str):
        if self.state != new:
            self.get_logger().info(f"🔄  {self.state}  →  {new}")
        self.state = new

    def _stop(self):
        self.pub_vel.publish(Twist())

    def _move(self, lin: float, ang: float):
        t = Twist()
        t.linear.x  = float(lin)
        t.angular.z = float(ang)
        self.pub_vel.publish(t)

    def _timed_move(self, lin: float, ang: float, duration: float):
        end = time.time() + duration
        while time.time() < end and rclpy.ok():
            self._move(lin, ang)
            time.sleep(0.05)
        self._stop()

    def _sleep_ok(self, sec: float):
        end = time.time() + sec
        while time.time() < end and rclpy.ok():
            time.sleep(0.05)

    # ════════════════════════════════════════════════════════════════════════
    #  FSM 루프
    # ════════════════════════════════════════════════════════════════════════

    def _fsm_loop(self):
        while rclpy.ok():
            s = self.state
            if   s == St.WAIT:            self._s_wait()
            elif s == St.IDLE:            self._s_idle()
            elif s == St.SEARCH:          self._s_search()
            elif s == St.APPROACH:        self._s_approach()
            elif s == St.APPROACH_ALIGN:  self._s_approach_align()
            elif s == St.EMERGENCY_STOP:  self._s_estop()
            elif s == St.UNLOAD_ALIGN:    self._s_unload_align()
            elif s == St.UNLOAD_STANDBY:  self._s_unload_standby()
            elif s == St.RETURN:          self._s_return()
            elif s == St.DONE:            self._s_done()
            time.sleep(0.05)

    # ────────────────────────────────────────────────────────────────────────
    def _s_wait(self):
        time.sleep(0.1)

    # ────────────────────────────────────────────────────────────────────────
    def _s_idle(self):
        self._stop()
        time.sleep(0.1)

    # ────────────────────────────────────────────────────────────────────────
    def _s_search(self):
        self.get_logger().info(f"🔍 SEARCH — 목표 색상: {self.target_color}")

        # 1바퀴(360°) 기준: 인데 현실에선 이유는 모르겠지만 1/2배가 됨
        one_turn = (4.0 * np.pi) / self.SPD_SEARCH

        for direction, label in [(1.0, "CW"), (-1.0, "CCW")]:
            ang_vel = -self.SPD_SEARCH * direction
            end = time.time() + one_turn
            while time.time() < end and rclpy.ok():
                if self.state != St.SEARCH:
                    return
                if self.line_detected:
                    self._stop()
                    self.get_logger().info(
                        f"✅ SEARCH({label}): 라인 발견 → APPROACH")
                    self._change_state(St.APPROACH)
                    return
                self._move(0.0, ang_vel)
                time.sleep(0.05)
            self._stop()
            self._sleep_ok(0.3)

        self.get_logger().warn("⚠️  SEARCH: 라인 미발견 — 1 s 후 재탐색")
        self._sleep_ok(1.0)

    # ────────────────────────────────────────────────────────────────────────
    def _s_approach(self):
        """
        returning 플래그에 따라 주행 모드 분기
          - returning=True  : 라인트레이싱 우선, 마커 무시
          - returning=False : 마커 인식 최우선, 마커 없을 때만 라인트레이싱
        공유 변수 Race Condition 해소 — 스냅샷 읽기
        """

        # ════ [귀환 중] 라인트레이싱 우선 모드 ═════════════════════════════
        if self.returning:
            with self._lock:
                line_detected = self.line_detected
                line_cx       = self.line_cx
                frame_w       = self.FRAME_W

            if not line_detected:
                self._stop()
                self.get_logger().info("🏁 귀환 중 라인 소실 → DONE")
                self._change_state(St.DONE)
                return

            err = line_cx - frame_w // 2
            if abs(err) > frame_w * 0.10:
                self._stop()
                self._change_state(St.APPROACH_ALIGN)
                return

            angular = -float(err) / (frame_w / 2) * self.SPD_ANG
            self._move(self.SPD_LINE, angular)
            return

        # ════ [일반 전진] 마커 인식 최우선 모드 ════════════════════════════

        with self._lock:
            found     = self.marker_found
            bot_y     = self.marker_bot_y
            marker_cx = self.marker_cx
            frame_w   = self.FRAME_W

        # ── ① 마커 우선 처리 ────────────────────────────────────────────────
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

            err     = marker_cx - frame_w // 2
            angular = -float(err) / (frame_w / 2) * self.SPD_ANG
            self._move(self.SPD_LINE, angular)
            return

        # ── ② 라인트레이싱 (마커 미인식 시에만) ────────────────────────────
        with self._lock:
            line_detected = self.line_detected
            line_cx       = self.line_cx

        if not line_detected:
            self._stop()
            self.get_logger().warn("⚠️  APPROACH: 라인 소실 → SEARCH")
            self._change_state(St.SEARCH)
            return

        err = line_cx - frame_w // 2
        if abs(err) > frame_w * 0.10:
            self._stop()
            self._change_state(St.APPROACH_ALIGN)
            return

        angular = -float(err) / (frame_w / 2) * self.SPD_ANG
        self._move(self.SPD_LINE, angular)

    # ────────────────────────────────────────────────────────────────────────
    def _s_approach_align(self):
        """
        returning=True 시 마커 체크 생략 — 라인 정렬만 수행
        공유 변수 스냅샷 읽기
        """
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
            if returning:
                self._change_state(St.DONE)
            else:
                self._change_state(St.SEARCH)
            return

        err = line_cx - frame_w // 2
        if abs(err) <= frame_w * 0.05:
            self._stop()
            self.get_logger().info("✅ APPROACH_ALIGN: 정렬 완료 → APPROACH")
            self._change_state(St.APPROACH)
            return

        angular = -float(err) / (frame_w / 2) * self.SPD_ANG
        self._move(self.SPD_ALIGN, angular)

    # ────────────────────────────────────────────────────────────────────────
    def _s_estop(self):
        self._stop()
        with self._lock:
            obstacle = self.obstacle_front
            prev     = self.pre_estop_state
        if not obstacle:
            self.get_logger().info(f"✅ EMERGENCY_STOP 해제 → {prev}")
            self._change_state(prev)

    # ────────────────────────────────────────────────────────────────────────
    def _s_unload_align(self):
        """
        [조건 1] TL·TR y 차 <= ALIGN_TOP_TOL (수평)
        [조건 2] 마커 중심 x in [ALIGN_CX_MIN, ALIGN_CX_MAX]
        두 조건 충족 → UNLOAD_STANDBY

        _frame_event 로 ArUco 가 실제로 실행된 새 프레임 수신을 확인한 뒤 스냅샷 읽기.

        [최적화 B 연동] ArUco 스킵으로 인해 frame_event 신호 간격이 늘어남.
        FRAME_TIMEOUT 은 아래와 같이 여유있게 설정:
          timeout = (2 × ARUCO_PROCESS_EVERY / fps) × 4 = 약 0.8 s @ 30 fps
          → 1.0 s 로 설정 (충분한 여유)
        """
        self.get_logger().info("🔧 UNLOAD_ALIGN: 정밀 정렬 시작")
        MAX_ITER      = 40
        FRAME_TIMEOUT = 1.0   # ArUco 스킵 주기를 고려한 여유 타임아웃 (s)

        for it in range(MAX_ITER):
            if not rclpy.ok():
                return

            # 카메라 억제 → frame_event 초기화 → 억제 해제 → ArUco 처리 프레임 대기
            self._ignore_camera_event.set()
            self._frame_event.clear()
            self._sleep_ok(0.05)
            self._ignore_camera_event.clear()

            # ArUco 가 실제로 실행된 프레임까지 대기 (스킵 프레임은 set() 안 함)
            arrived = self._frame_event.wait(timeout=FRAME_TIMEOUT)
            if not arrived:
                self.get_logger().warn(
                    f"⚠️  UNLOAD_ALIGN iter={it}: 프레임 수신 타임아웃 "
                    f"(ARUCO_PROCESS_EVERY={self.ARUCO_PROCESS_EVERY})")
                continue

            with self._lock:
                found = self.marker_found
                tl    = self.marker_tl
                tr    = self.marker_tr
                cx    = self.marker_cx

            if not found:
                self.get_logger().warn(
                    f"⚠️  UNLOAD_ALIGN iter={it}: 마커 미인식 → 후진 0.5 s")
                self._timed_move(-self.SPD_LINE, 0.0, 0.5)
                continue

            tl_x, tl_y = tl
            tr_x, tr_y = tr
            top_delta   = abs(tl_y - tr_y)
            top_ok      = top_delta <= self.ALIGN_TOP_TOL
            cx_ok       = self.ALIGN_CX_MIN <= cx <= self.ALIGN_CX_MAX

            self.get_logger().info(
                f"  [iter={it}] top_delta={top_delta:.1f}px(ok={top_ok})  "
                f"cx={cx}(ok={cx_ok})")

            if top_ok and cx_ok:
                self._stop()
                self.get_logger().info(
                    "✅ UNLOAD_ALIGN: 조건 충족 → UNLOAD_STANDBY")
                self._change_state(St.UNLOAD_STANDBY)
                return

            if not cx_ok:
                err = cx - self.FRAME_W // 2
                ang = -float(np.sign(err)) * 0.15
                self.get_logger().info(
                    f"  → cx 보정: err={err}px  ang={ang:.2f} rad/s  0.25 s")
                self._timed_move(0.0, ang, 0.25)

            if not top_ok:
                tilt = tl_y - tr_y
                ang  = float(np.sign(tilt)) * 0.10
                self.get_logger().info(
                    f"  → 수평 보정: tilt={tilt:.1f}px  "
                    f"ang={ang:.2f} rad/s  0.20 s")
                self._timed_move(0.0, ang, 0.20)

        self.get_logger().warn(
            "⚠️  UNLOAD_ALIGN: 최대 반복 도달 — 강제 UNLOAD_STANDBY")
        self._change_state(St.UNLOAD_STANDBY)

    # ────────────────────────────────────────────────────────────────────────
    def _s_unload_standby(self):
        """
        전진 → 포크 DOWN → 후진 순서로 동작.
        이 구간에서는 E-STOP 이 적용되지 않음.
        (LiDAR 콜백의 E-STOP 분기는 APPROACH / APPROACH_ALIGN 만 대상이며,
        전방에 선반이 있는 상황이므로 UNLOAD_STANDBY 는 의도적으로 제외)
        """
        self.get_logger().info("📦 UNLOAD_STANDBY: 전진 7.0 s")
        self._timed_move(self.SPD_LINE, 0.0, 7.0)

        with self._lock:
            self.fork_done = False

        msg = String()
        msg.data = 'DOWN'
        self.pub_fork.publish(msg)
        self.get_logger().info(
            "🔽 /fork_cmd: DOWN 발행 — /fork_done 대기 중...")

        FORK_TIMEOUT = 10.0
        wait_start   = time.time()
        while rclpy.ok():
            with self._lock:
                done = self.fork_done
            if done:
                self.get_logger().info("✅ /fork_done 수신 — 포크 완료 확인")
                break
            if time.time() - wait_start > FORK_TIMEOUT:
                self.get_logger().warn(
                    f"⚠️  /fork_done 미수신 "
                    f"({FORK_TIMEOUT:.0f} s 타임아웃) — 강제 진행")
                break
            time.sleep(0.05)

        self.get_logger().info("↩️  후진 5.0 s")
        self._timed_move(-self.SPD_LINE, 0.0, 5.0)
        self._change_state(St.RETURN)

    # ────────────────────────────────────────────────────────────────────────
    def _s_return(self):
        with self._lock:
            self.endpoint_ids -= {0, 1, 2}
            self.returning = True
        self.get_logger().info(
            f"🗑️  엔드포인트 0·1·2 제거 — 잔여: {self.endpoint_ids}  "
            "[RETURNING ON]")

        # 180° 회전: pi / SPD_ANG
        # 인데 현실에선 이유는 모르겠지만 1/2배가 됨
        self.get_logger().info("↩️  RETURN: 180° 회전 시작")
        half_turn = (2 * np.pi) / self.SPD_ANG
        self._ignore_camera_event.set()
        self._timed_move(0.0, self.SPD_ANG, half_turn)
        self._ignore_camera_event.clear()

        self.get_logger().info("✅ RETURN: 회전 완료 → 귀환")
        self._change_state(St.APPROACH)

    # ────────────────────────────────────────────────────────────────────────
    def _s_done(self):
        self.get_logger().info("🏁 DONE — 미션 완료, IDLE 복귀")
        with self._lock:
            self.target_color = None
            self.endpoint_ids = {100}
            self.returning    = False
            self.marker_found = False
        self._stop()
        msg = String()
        msg.data = 'Done'
        self.pub_done.publish(msg)
        self._change_state(St.IDLE)
        time.sleep(0.1)
        self._stop()   # WAIT 전환 직전 잔류 속도 확실히 제거
        self._change_state(St.WAIT)


# ════════════════════════════════════════════════════════════════════════════
def main(args=None):
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