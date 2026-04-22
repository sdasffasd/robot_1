#!/usr/bin/env python3
# encoding: utf-8

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import json
import threading
import numpy as np

class RobotSpeedPublisher(Node):
    def __init__(self):
        super().__init__('robot_001_speed')
        self.latest_odom = None
        self.odom_lock = threading.Lock()
        self.publish_count = 0
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_footprint')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('topic_name', '/robot/velocity')
        self.declare_parameter('print_frequency', 10.0)
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.child_frame_id = self.get_parameter('child_frame_id').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.print_frequency = self.get_parameter('print_frequency').get_parameter_value().double_value
        self.print_interval = int(self.publish_rate / self.print_frequency) if self.print_frequency > 0 else 1
        self.sub_odom_raw = self.create_subscription(
            Odometry,
            'odom_raw',
            self.odom_callback,
            10
        )
        self.publisher = self.create_publisher(
            String,
            self.topic_name,
            10
        )
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_odometry)
        self.get_logger().info(f'机器人速度发布节点已启动')
        self.get_logger().info(f'发布频率: {self.publish_rate} Hz')
        self.get_logger().info(f'订阅话题: /odom_raw')
        self.get_logger().info(f'发布话题: {self.topic_name}')
        self.get_logger().info(f'打印频率: {self.print_frequency} Hz')
        self.get_logger().info('等待接收odom_raw数据...')

    def odom_callback(self, msg):
        try:
            with self.odom_lock:
                self.latest_odom = msg
            if self.print_frequency > 0:
                current_sec = msg.header.stamp.sec
                current_nsec = msg.header.stamp.nanosec
                timestamp = current_sec + current_nsec / 1e9
                linear_x = msg.twist.twist.linear.x
                linear_y = msg.twist.twist.linear.y
                linear_z = msg.twist.twist.linear.z
                linear_speed = np.sqrt(linear_x**2 + linear_y**2 + linear_z**2)
                angular_z = msg.twist.twist.angular.z
                pos_x = msg.pose.pose.position.x
                pos_y = msg.pose.pose.position.y
                pos_z = msg.pose.pose.position.z
                self.get_logger().debug(
                    f'接收数据[{timestamp:.3f}s]: '
                    f'linear: {linear_speed:.3f} m/s '
                    f'angular: {angular_z:.3f} rad/s '
                    f'location: {pos_x:.3f}, {pos_y:.3f}, {pos_z:.3f} m'
                )
        except Exception as e:
            self.get_logger().error(f'处理odom回调失败: {e}')

    def publish_odometry(self):
        if self.latest_odom is None:
            if self.publish_count == 0:
                self.get_logger().warn('尚未收到odom_raw数据,等待中...')
            return
        try:
            with self.odom_lock:
                msg = self.latest_odom
                pos_x = msg.pose.pose.position.x
                pos_y = msg.pose.pose.position.y
                pos_z = msg.pose.pose.position.z
                orientation_x = msg.pose.pose.orientation.x
                orientation_y = msg.pose.pose.orientation.y
                orientation_z = msg.pose.pose.orientation.z
                orientation_w = msg.pose.pose.orientation.w
                linear_x = msg.twist.twist.linear.x
                linear_y = msg.twist.twist.linear.y
                linear_z = msg.twist.twist.linear.z
                angular_x = msg.twist.twist.angular.x
                angular_y = msg.twist.twist.angular.y
                angular_z = msg.twist.twist.angular.z
            data = {
                "robot_id": "robot_001",
                "velocity_x": linear_x,
                "velocity_y": linear_y,
                "velocity_z": angular_z,
                "position_x": pos_x,
                "position_y": pos_y,
                "position_z": pos_z,
                "orientation": {
                    "x": orientation_x,
                    "y": orientation_y,
                    "z": orientation_z,
                    "w": orientation_w
                },
                "linear_x": linear_x,
                "linear_y": linear_y,
                "angular_z": angular_z
            }
            json_str = json.dumps(data)
            pub_msg = String()
            pub_msg.data = json_str
            self.publisher.publish(pub_msg)
            self.publish_count += 1
            current_time = self.get_clock().now()
            current_sec = current_time.nanoseconds / 1e9
            linear_speed = np.sqrt(linear_x**2 + linear_y**2 + linear_z**2)
            if self.print_frequency > 0 and self.publish_count % self.print_interval == 0:
                self.get_logger().info(
                    f'[{current_sec:.3f}s] '
                    f'linear: {linear_speed:.3f} m/s  '
                    f'angular: {angular_z:.3f} rad/s  '
                    f'location: {pos_x:.3f}, {pos_y:.3f}, {pos_z:.3f} m'
                )
        except Exception as e:
            self.get_logger().error(f'发布消息失败: {e}')

    def destroy_node(self):
        self.get_logger().info('机器人速度发布节点正在关闭...')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    try:
        node = RobotSpeedPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'节点运行异常: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
