from flask import Blueprint, request, jsonify
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

driving_node = None

class DrivingNode(Node):
    def __init__(self):
        super().__init__('driving_node')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        self.carry_publisher = self.create_publisher(Int32, '/carry_id', qos_profile)
        self.waypoints_publisher = self.create_publisher(PoseStamped, '/waypoint_drive', qos_profile)
        #self.carry_publisher = self.create_publisher(Int32, '/carry', qos_profile)

def get_driving_node():
    global driving_node
    if driving_node is None:
        # 이때 딱 한 번만 노드를 생성
        driving_node = DrivingNode()
    return driving_node

drive_bp = Blueprint('drive_bp', __name__)

@drive_bp.route('/startDrive.do', methods=['POST'])
def start_drive():
    print(f"======================= start_drive start =======================")
    node = get_driving_node()
    data = request.get_json()
    try:
        msg = Int32()
        msg.data = data['palletId']
        driving_node.carry_publisher.publish(msg)
        print(f"======================= start_drive {msg} =======================")
        return jsonify({"result": "success", "message": f"{data.palletId}번 상/하차 시작"})
    except Exception as e:
        return jsonify({"result": "error", "message": str(e)})

@drive_bp.route('/waypointDrive.do', methods=['POST'])
def waypoint_drive():
    print(f"======================= waypoint_drive start =======================")
    node = get_driving_node()
    data = request.get_json()

    # data["x"], data["y"], data["yaw"]
    print(data)
    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.pose.position.x = float(data['x'])
    msg.pose.position.y = float(data['y'])

    # yaw → quaternion 변환
    msg.pose.orientation.z = float(data['qz'])
    msg.pose.orientation.w = float(data['qw'])
    
    driving_node.waypoints_publisher.publish(msg)
    try:
       
        print(f"======================= waypoint_drive {msg} =======================")
        return jsonify({"result": "success", "message": f" 주행 시작"})
    except Exception as e:
        return jsonify({"result": "error", "message": str(e)})

