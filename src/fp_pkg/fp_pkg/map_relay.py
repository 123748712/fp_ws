import rclpy
from rclpy.node import Node
# 여기에 QoSHistoryPolicy를 추가해야 함!
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import OccupancyGrid

class MapRelayNode(Node):
    def __init__(self):
        super().__init__('map_relay')

        # map_server(Transient Local) 대응용
        sub_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST # 아까 여기서 에러 났을 거임
        )

        # 웹(Rosbridge) 대응용
        pub_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST
        )

        self.last_map = None
        
        self.sub = self.create_subscription(OccupancyGrid, '/map', self.callback, sub_qos)
        self.pub = self.create_publisher(OccupancyGrid, '/map_web', pub_qos)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('=== [Map Relay] 노드 시작됨. /map 기다리는 중... ===')

    def callback(self, msg):
        # 맵이 한 번이라도 들어오면 여기 로그가 찍혀야 함
        self.get_logger().info(f'=== [Map Relay] /map 수신 성공! (크기: {len(msg.data)}) ===')
        self.last_map = msg
        self.pub.publish(msg)

    def timer_callback(self):
        if self.last_map is not None:
            # 1초마다 반복 발행
            self.pub.publish(self.last_map)
        else:
            # 맵을 아직 못 받았으면 로그 찍기
            self.get_logger().warn('=== [Map Relay] 아직 /map 데이터를 받지 못해 대기 중... ===', once=True)

def main():
    rclpy.init()
    node = MapRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()