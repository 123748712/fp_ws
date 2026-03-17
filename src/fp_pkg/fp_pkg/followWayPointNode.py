import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from flask import Flask, request, jsonify
from flask_cors import CORS
from threading import Thread
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
import time

class FollowWayPointNode(Node):
    def __init__(self):
        super().__init__('drive_waypoint_node')
        self._action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

    def send_waypoints(self, waypoints_data):
        self.get_logger().info('Waiting for action server...')
        
        ready = self._action_client.wait_for_server(timeout_sec=10.0)
        if not ready:
            self.get_logger().error('follow_waypoints action server 응답 없음 (waypoint_follower가 active 상태인지 확인)')
            return

        goal_msg = FollowWaypoints.Goal()
        poses = []

        for p in waypoints_data:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(p['x'])
            pose.pose.position.y = float(p['y'])
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            poses.append(pose)

        goal_msg.poses = poses
        self.get_logger().info(f'{len(poses)}개 노드 로봇에게 전송중')
        
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted')

def main(args=None):
    rclpy.init(args=args)
    node = FollowWayPointNode()
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