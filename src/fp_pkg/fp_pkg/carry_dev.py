import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32
import cv2
import numpy as np
import time
import threading
from cv_bridge import CvBridge

class RealTurtlebotMissionNode(Node):
    def __init__(self):
        super().__init__('remote_mission_node')
        self.bridge = CvBridge()

        # --- [1. 파라미터 및 설정] ---
        self.LIN_SPD = 0.06
        self.MIS_LIN_SPD = 0.05
        self.ANG_SPD = 0.4
        self.P2M = 0.00075          # 픽셀 당 미터 변환 계수 (환경에 따라 조정)
        self.TURN_CONST = 2.95      # 90도 회전을 위한 시간 상수
        self.DEADZONE_M = 0.001     # 1mm 이내 정밀 정렬
        self.TARGET_Y = 336         # 정지 목표 Y 좌표 (거리)
        self.allowed_ids = [0, 1, 2]

        # --- [2. 상태 변수] ---
        self.is_running = False
        self.marker_found = False
        self.error_px = 0
        self.marker_top_y = 0
        self.marker_corners = None
        self.current_target_id = None
        self.arduino_done_received = False

        # ArUco 설정
        self.dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dict, self.params)

        # --- [3. Pub/Sub] ---
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_done = self.create_publisher(String, '/carry_done', 10)
        self.sub_trigger = self.create_subscription(Int32, '/carry', self.trigger_callback, 10)
        self.pub_forklift_cmd = self.create_publisher(String, '/forklift_cmd', 10)
        
        # [에러 해결] bridge_done_callback 연결
        self.sub_forklift_done = self.create_subscription(String, '/forklift_done', self.bridge_done_callback, 10)

        self.sub_img = None
        self.get_logger().info("🚀 Raw 데이터 기반 정밀 노드 준비 완료")

    # --- [콜백 함수들] ---

    def bridge_done_callback(self, msg):
        """아두이노(지게차)로부터 완료 신호를 수신"""
        self.get_logger().info(f"📩 아두이노 수신: {msg.data}")
        # 메시지에 'done'이 포함되어 있으면 완료로 간주
        if "done" in msg.data.lower() or "8" in msg.data:
            self.arduino_done_received = True

    def img_callback(self, msg):
        try:
            if self.current_target_id is None: return
            
            # Raw 이미지 변환 (bgr8)
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            center_x = img.shape[1] // 2
            corners, ids, _ = self.detector.detectMarkers(img)

            if ids is not None:
                for i, m_id in enumerate(ids.flatten()):
                    if m_id == self.current_target_id:
                        self.marker_corners = corners[i][0]
                        # 마커의 상단 Y 좌표 (거리 측정용)
                        self.marker_top_y = np.min(self.marker_corners[:, 1])
                        # 마커의 중심 X 좌표 (좌우 편차 계산용)
                        m_x = int(np.mean(self.marker_corners[:, 0]))
                        self.error_px = m_x - center_x
                        self.marker_found = True

                        # 디버깅 시각화
                        cv2.line(img, (0, self.TARGET_Y), (img.shape[1], self.TARGET_Y), (0, 255, 0), 2)
                        cv2.aruco.drawDetectedMarkers(img, corners, ids)
                        cv2.imshow("Raw Debug Window", img)
                        cv2.waitKey(1)
                        return
            
            self.marker_found = False
            cv2.imshow("Raw Debug Window", img)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"이미지 콜백 에러: {e}")

    def trigger_callback(self, msg):
        if msg.data in self.allowed_ids and not self.is_running:
            self.current_target_id = msg.data
            self.get_logger().info(f"🔔 미션 수신: Target ID {msg.data}")
            
            # 미션 시작 시에만 이미지 구독 시작
            self.sub_img = self.create_subscription(Image, '/image_raw', self.img_callback, 10)
            threading.Thread(target=self.run_full_sequence, daemon=True).start()

    # --- [동작 유틸리티 함수] ---

    def stop_robot(self, wait=0.8):
        self.pub_vel.publish(Twist())
        time.sleep(wait)

    def timed_move(self, linear, angular, duration):
        """특정 시간 동안 로봇을 이동"""
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        start_t = time.time()
        while (time.time() - start_t) < duration:
            self.pub_vel.publish(msg)
            time.sleep(0.05)
        self.stop_robot()

    def align_yaw_precision(self):
        """마커가 화면 중앙에 오도록 제자리 회전 정렬"""
        self.get_logger().info("🎯 정밀 각도 정렬 시작")
        while rclpy.ok():
            if not self.marker_found:
                time.sleep(0.1)
                continue
            
            # 에러가 작으면 정지
            if abs(self.error_px) < 3:
                break
            
            msg = Twist()
            # P-제어 방식으로 속도 조절 (선택 사항)
            msg.angular.z = -0.002 * float(self.error_px)
            self.pub_vel.publish(msg)
            time.sleep(0.05)
        self.stop_robot()

    def find_zero_point(self, target_y):
        """TARGET_Y 좌표에 도달할 때까지 전후진"""
        self.get_logger().info(f"📏 거리 조절 시작 (Target Y: {target_y})")
        while rclpy.ok():
            if not self.marker_found:
                time.sleep(0.1)
                continue
            
            diff = target_y - self.marker_top_y
            if abs(diff) < 3:
                break
            
            msg = Twist()
            msg.linear.x = 0.02 if diff > 0 else -0.02
            self.pub_vel.publish(msg)
            time.sleep(0.05)
        self.stop_robot()

    def wait_for_arduino(self, timeout):
        self.arduino_done_received = False
        start_t = time.time()
        while (time.time() - start_t) < timeout:
            if self.arduino_done_received: return True
            time.sleep(0.1)
        return False

    # --- [메인 시퀀스] ---

    def run_full_sequence(self):
        self.is_running = True
        try:
            # 1. 초기 정면 정렬
            self.align_yaw_precision()

            # 2. 거리 조절
            self.find_zero_point(self.TARGET_Y)

            # 3. 크랩 워킹 (Side-shift) - 오차만큼 옆으로 이동 후 복귀
            self.get_logger().info("↔️ 좌우 편차 보정 시작")
            for _ in range(2):
                if not self.marker_found: break
                
                err_m = self.error_px * self.P2M
                if abs(err_m) < self.DEADZONE_M:
                    self.get_logger().info(f"✅ 정렬 완료 (오차: {err_m:.4f}m)")
                    break
                
                rot_dir = -1.0 if err_m > 0 else 1.0
                # 90도 회전 -> 직진 -> -90도 회전
                self.timed_move(0.0, self.ANG_SPD * rot_dir, self.TURN_CONST / self.ANG_SPD)
                self.timed_move(self.LIN_SPD, 0.0, abs(err_m) / self.LIN_SPD)
                self.timed_move(0.0, -self.ANG_SPD * rot_dir, self.TURN_CONST / self.ANG_SPD)
                self.align_yaw_precision()

            # 4. 최종 미션 수행
            self.get_logger().info("➡️ 최종 진입 및 미션 수행")
            self.timed_move(self.MIS_LIN_SPD, 0.0, 8.5) # 진입 거리
            
            # 아두이노에 지게차 동작 명령 (예: '8')
            self.pub_forklift_cmd.publish(String(data='8'))
            if not self.wait_for_arduino(20.0):
                raise Exception("아두이노 응답 시간 초과")
            
            # 후진 및 종료
            self.timed_move(-self.MIS_LIN_SPD, 0.0, 8.5)
            print("======================== carry_done publish start ========================")
            self.pub_done.publish(String(data=f"success: ID {self.current_target_id}"))
            print("======================== carry_done publish end ========================")
            self.get_logger().info("🎉 미션 완료!")

        except Exception as e:
            self.get_logger().error(f"❌ 미션 에러: {e}")
        finally:
            self.stop_robot()
            if self.sub_img:
                self.destroy_subscription(self.sub_img)
                self.sub_img = None
                cv2.destroyAllWindows()
            self.is_running = False
            self.current_target_id = None

def main():
    rclpy.init()
    node = RealTurtlebotMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()