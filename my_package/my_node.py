from typing import List
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Twist

import socket
import sys
sys.path.append('/home/gus/ufg/SSL/ros2_ws/src/my_package/my_package')

from messages_robocup_ssl_wrapper_pb2 import SSL_WrapperPacket




class SSLVisionProtobufToROS(Node):

    def __init__(self):
        super().__init__('protobuf_to_ros')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('blue_robot_count', 3),
                ('yellow_robot_count', 3),
                ('frequency', 60),
            ]
        )
        self.blue_robot_count = self.get_parameter('blue_robot_count').get_parameter_value().integer_value
        self.yellow_robot_count = self.get_parameter('yellow_robot_count').get_parameter_value().integer_value
        frequency = self.get_parameter('frequency').get_parameter_value().integer_value
        time_step_ms = 1000 // frequency

        self.ssl_vision_ip = '224.5.23.2'
        self.ssl_vision_port = 10006

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 128)
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
        self.sock.bind((self.ssl_vision_ip, self.ssl_vision_port))



        field_type = 1  # 0 for Division A, 1 for Division B, 2 Hardware Challenges
        # ball initial position [x, y, v_x, v_y] in meters and meter/s
        ball_pos = [0.0, 0.0, 0.0, 0.0]

        # robots initial positions [[x, y, angle], [x, y, angle]...], where [[id_0], [id_1]...]
        # Units are meters and degrees
        blue_robots_pos = [[-1 * i - 1, 0.0, 0.0] for i in range(self.blue_robot_count)]
        yellow_robots_pos = [[1 * i + 1, 0.0, 0.0] for i in range(self.yellow_robot_count)]

        self.ball_publisher = self.create_publisher(Pose2D, '/simulator/poses/ball', 10)

        # /simulator/robots_poses/blue/0, /simulator/robots_poses/blue/1, ...
        self.pose_publishers = []
        for i in range(self.blue_robot_count):
            self.pose_publishers.append(
                self.create_publisher(Pose2D, f'/simulator/poses/blue/robot{i}', 10)
            )
        for i in range(self.yellow_robot_count):
            self.pose_publishers.append(
                self.create_publisher(Pose2D, f'/simulator/poses/yellow/robot{i}', 10)
            )
        
        self.latest_robot_actions = [[0.0 for _ in range(6)] for _ in range(self.blue_robot_count + self.yellow_robot_count)]
        
        timer_period = time_step_ms / 1000.0  # seconds
        self.timer = self.create_timer(timer_period, self.get_protobuf_and_publish)

        self.get_logger().info('Protobuf Connector Node Started')

    def get_protobuf_and_publish(self):
        # Units are meters, meters/s, degrees
        # state is [ball_x, ball_y, ball_z, ball_v_x, ball_v_y,
        #           blue_0_x, blue_0_y, blue_0_angle, blue_0_v_x, blue_0_v_y, blue_0_v_angle,
        #           blue_0_infrared, blue_0_desired_wheel0_speed, blue_0_desired_wheel1_speed,
        #           blue_0_desired_wheel2_speed, blue_0_desired_wheel3_speed, ...]
        data = self.sock.recvfrom(1024)
        decode_data = SSL_WrapperPacket.FromString(data[0])
        print("HELLOOOO")
        for robot in decode_data.detection.robots_yellow:
            print("x = ",robot.x)



        ball_msg = Pose2D()
        ball_msg.x = 10.0
        ball_msg.y = 20.0
        ball_msg.theta = 1.23
        self.ball_publisher.publish(ball_msg)
        
        for i in range(self.blue_robot_count):
            msg = Pose2D()
            msg.x = 1.0
            msg.y = 2.0
            # msg.theta = state[7 + i * 11]
            theta_deg = 1.23
            self.pose_publishers[i].publish(msg)
        
        for i in range(self.yellow_robot_count):
            msg = Pose2D()
            msg.x = -1.0
            msg.y = -2.0
            # msg.theta = state[7 + self.blue_robot_count * 11 + i * 11]
            theta_deg = 1.34
            self.pose_publishers[self.blue_robot_count + i].publish(msg)


def main(args=None):
    rclpy.init(args=args)

    protobuf_to_ros = SSLVisionProtobufToROS()

    rclpy.spin(protobuf_to_ros)

    protobuf_to_ros.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
