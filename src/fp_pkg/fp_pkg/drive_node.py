import rclpy
from rclpy.node import Node

class DriveNode(Node):
    def __init__(self):
        super().__init__('drive_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, # 무조건 보장
            durability=DurabilityPolicy.TRANSIENT_LOCAL, # 나중에 들어온 구독자도 마지막 메시지 받기 가능
            depth=10
        )

        # self.publisher = self.create_publisher(msg, '/mission_topic', qos_profile)