#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import json
import time
import socket
import os
import signal
import subprocess
import re
import threading    # xxx

from std_msgs.msg import String, Empty, Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class RobotStatePublisher(Node):
    def __init__(self):
        super().__init__('robot_001_state_pub')
        self.declare_parameter('robot_id', 'robot_001')
        self.declare_parameter('robot_name', '巡检机器人1号')
        self.declare_parameter('wifi_interface', 'wlan0')
        self.declare_parameter('wifi_update_rate', 1.0)
        self.declare_parameter('full_voltage', 12.6)
        self.declare_parameter('empty_voltage', 5.0)
        self.declare_parameter('warning_voltage', 6.5)
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.wifi_interface = self.get_parameter('wifi_interface').get_parameter_value().string_value
        self.wifi_update_rate = self.get_parameter('wifi_update_rate').get_parameter_value().double_value
        self.full_voltage = self.get_parameter('full_voltage').get_parameter_value().double_value
        self.empty_voltage = self.get_parameter('empty_voltage').get_parameter_value().double_value
        self.warning_voltage = self.get_parameter('warning_voltage').get_parameter_value().double_value
        self.wifi_connected = False
        self.wifi_signal_dbm = 0
        self.wifi_ip_address = ""
        self.battery_voltage = 0.0
        self.battery_percentage = 0.0
        self.current_status = 'online'
        self.current_event_type = 'status_update'
        self.moving_state = False
        self.has_error = False
        self.error_timer = None
        self.last_moving_time = self.get_clock().now()
        self.power_on_sent = False
        self.shutting_down = False
        self.velocity_threshold = 0.01
        self.moving_timeout = 2.0
        self.status_pub = self.create_publisher(String, '/robot/status', 10)
        self.battery_pub = self.create_publisher(String, '/robot/battery', 10)  # xxx
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.error_sub = self.create_subscription(String, '/robot/error', self.error_callback, 10)
        self.shutdown_sub = self.create_subscription(Empty, '/robot/shutdown', self.shutdown_callback, 10)
        self.battery_voltage_sub = self.create_subscription(Float32, '/voltage', self.voltage_callback, 10)
        self.timer = self.create_timer(1.0, self.publish_status)
        wifi_timer_period = 1.0 / self.wifi_update_rate
        self.wifi_timer = self.create_timer(wifi_timer_period, self.update_wifi_info)
        self.startup_time = time.time()
        self.create_timer(3.0, self.send_power_on_once)
        self.setup_signal_handlers()
        self.update_wifi_info()
        self.get_logger().info(f'机器人状态发布节点已启动, 机器人ID: {self.robot_id}, 名称: {self.robot_name}')
        self.get_logger().info(f'WiFi接口: {self.wifi_interface}, 更新频率: {self.wifi_update_rate}Hz')
        self.get_logger().info(f'电池参数: 满电电压: {self.full_voltage}V, 空电电压: {self.empty_voltage}V')

    def setup_signal_handlers(self):
        signal.signal(signal.SIGTERM, self.signal_handler)
        signal.signal(signal.SIGINT, self.signal_handler)

    def signal_handler(self, signum, frame):
        signal_name = "SIGINT" if signum == signal.SIGINT else "SIGTERM"
        self.get_logger().warn(f'收到信号 {signal_name}, 发送关机事件...')
        if not self.shutting_down:
            self.shutting_down = True
            self.send_event('power_off')
            # xxx
            self.safe_shutdown()

    def get_reliable_ip_address(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip_address = s.getsockname()[0]
            s.close()
            return ip_address
        except Exception as e:
            self.get_logger().warning(f"获取IP地址失败: {e}")
            try:
                hostname = socket.gethostname()
                ip_address = socket.gethostbyname(hostname)
                if ip_address and ip_address not in ('127.0.0.1', '127.0.1.1'):
                    return ip_address
            except:
                pass
            return "127.0.0.1"

    def get_wifi_info(self):
        wifi_info = {'signal_dbm': 0, 'ip_address': ''}
        try:
            try:
                with open('/proc/net/wireless', 'r') as f:
                    for line in f.readlines()[2:]:
                        if self.wifi_interface in line:
                            parts = line.strip().split()
                            if len(parts) >= 4:
                                signal_str = parts[3].rstrip('.')
                                try:
                                    wifi_info['signal_dbm'] = int(signal_str)
                                    self.wifi_connected = True
                                except ValueError:
                                    pass
                                break
            except Exception as e:
                self.get_logger().debug(f"读取/proc/net/wireless失败: {e}")
            if wifi_info['signal_dbm'] == 0:
                try:
                    result = subprocess.run(['iw', 'dev', self.wifi_interface, 'link'],
                                          capture_output=True, text=True, timeout=2)
                    if result.returncode == 0:
                        signal_match = re.search(r'signal: (-?\d+) dBm', result.stdout)
                        if signal_match:
                            wifi_info['signal_dbm'] = int(signal_match.group(1))
                            self.wifi_connected = True
                except Exception as e:
                    self.get_logger().debug(f"iw命令失败: {e}")
            if wifi_info['signal_dbm'] == 0:
                try:
                    result = subprocess.run(['iwconfig', self.wifi_interface],
                                          capture_output=True, text=True, timeout=2)
                    if result.returncode == 0:
                        signal_match = re.search(r'Signal level=(-?\d+) dBm', result.stdout)
                        if signal_match:
                            wifi_info['signal_dbm'] = int(signal_match.group(1))
                            self.wifi_connected = True
                except Exception as e:
                    self.get_logger().debug(f"iwconfig命令失败: {e}")
            try:
                result = subprocess.run(['ip', '-4', 'addr', 'show', self.wifi_interface],
                                      capture_output=True, text=True, timeout=2)
                if result.returncode == 0:
                    ip_match = re.search(r'inet (\d+\.\d+\.\d+\.\d+)', result.stdout)
                    wifi_info['ip_address'] = ip_match.group(1) if ip_match else self.get_reliable_ip_address()
                else:
                    wifi_info['ip_address'] = self.get_reliable_ip_address()
            except Exception:
                wifi_info['ip_address'] = self.get_reliable_ip_address()
        except Exception as e:
            self.get_logger().warn(f'获取WiFi信息失败: {e}')
            self.wifi_connected = False
        return wifi_info

    def update_wifi_info(self):
        wifi_info = self.get_wifi_info()
        old_signal = self.wifi_signal_dbm
        old_ip = self.wifi_ip_address
        self.wifi_signal_dbm = wifi_info['signal_dbm']
        self.wifi_ip_address = wifi_info['ip_address']
        if abs(old_signal - self.wifi_signal_dbm) > 5 or old_ip != self.wifi_ip_address:
            if self.wifi_connected:
                self.get_logger().info(f'WiFi信息更新: 信号强度={self.wifi_signal_dbm}dBm, IP={self.wifi_ip_address}')
        current_time = time.time()
        if current_time - self.startup_time < 10 or int(current_time) % 30 == 0:
            if self.wifi_connected:
                self.get_logger().info(f'WiFi状态: 信号强度={self.wifi_signal_dbm}dBm, IP={self.wifi_ip_address}')
            else:
                self.get_logger().info('WiFi未连接')

    def voltage_callback(self, msg):
        self.battery_voltage = msg.data
        if self.battery_voltage >= self.full_voltage:
            self.battery_percentage = 1.0
        elif self.battery_voltage <= self.empty_voltage:
            self.battery_percentage = 0.0
        else:
            self.battery_percentage = (self.battery_voltage - self.empty_voltage) / (self.full_voltage - self.empty_voltage)
            self.battery_percentage = max(0.0, min(1.0, self.battery_percentage))
        if self.battery_voltage < self.empty_voltage:
            self.get_logger().warn(f'电池电压过低: {self.battery_voltage}V, 电量: {self.battery_percentage:.1%}')
        elif self.battery_voltage < self.warning_voltage:
            self.get_logger().info(f'电池电压警告: {self.battery_voltage}V, 电量: {self.battery_percentage:.1%}')
        # xxx
        battery_msg = {
            "robot_id": self.robot_id,
            "level": float(self.battery_percentage),
            "voltage": float(self.battery_voltage),
            "temperature": None
        }
        pub_msg = String()
        pub_msg.data = json.dumps(battery_msg, ensure_ascii=False)
        self.battery_pub.publish(pub_msg)

    def send_power_on_once(self):
        if not self.power_on_sent:
            self.send_event('power_on')
            self.power_on_sent = True

    def odom_callback(self, msg):
        current_time = self.get_clock().now()
        linear_vel = msg.twist.twist.linear.x
        angular_vel = msg.twist.twist.angular.z
        total_velocity = abs(linear_vel) + abs(angular_vel)
        if total_velocity > self.velocity_threshold:
            if not self.moving_state:
                self.moving_state = True
                self.send_event('moving')
            self.last_moving_time = current_time
        else:
            time_since_last_move = (current_time - self.last_moving_time).nanoseconds / 1e9
            if time_since_last_move > self.moving_timeout and self.moving_state:
                self.moving_state = False
                self.send_event('stopped')

    def cmd_vel_callback(self, msg):
        if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
            self.last_moving_time = self.get_clock().now()
            if not self.moving_state:
                self.moving_state = True
                self.send_event('moving')

    def error_callback(self, msg):
        self.get_logger().error(f'收到错误: {msg.data}')
        self.has_error = True
        self.send_event('error', {'error_message': msg.data})
        # xxx
        if self.error_timer:
            self.error_timer.cancel()
            self.destroy_timer(self.error_timer)
            self.error_timer = None
        self.error_timer = self.create_timer(5.0, self._clear_error)
    # xxx
    def _clear_error(self):
        if self.error_timer:
            self.error_timer.cancel()
            self.destroy_timer(self.error_timer)
            self.error_timer = None
        self.has_error = False

    def shutdown_callback(self, msg):
        self.get_logger().info('收到关机信号')
        self.send_event('power_off')
        self.create_timer(1.0, self.safe_shutdown)

    def send_event(self, event_type, extra_data=None):
        self.current_event_type = event_type
        if event_type == 'error':
            self.current_status = 'error'
        elif event_type == 'power_on':
            self.current_status = 'online'
        elif event_type == 'power_off':
            self.current_status = 'offline'
        elif event_type in ('moving', 'stopped', 'status_update'):
            self.current_status = 'error' if self.has_error else 'online'
        message = {
            "robot_id": self.robot_id,
            "name": self.robot_name,
            "status": self.current_status,
            "event_type": self.current_event_type,
            "ip_address": self.wifi_ip_address,
            "battery": float(self.battery_percentage),
            "signal_dbm": self.wifi_signal_dbm
        }
        if extra_data:
            extended_message = message.copy()
            extended_message["extra_data"] = extra_data
            json_str = json.dumps(extended_message, ensure_ascii=False)
        else:
            json_str = json.dumps(message, ensure_ascii=False)
        msg = String()
        msg.data = json_str
        try:
            self.status_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'发布事件失败: {e}')
        current_time = time.time()
        log_level = 'debug' if current_time - self.startup_time > (30 if event_type == 'status_update' else 10) else 'info'
        log_msg = f'发送事件: {event_type}, 状态: {self.current_status}, 电池: {self.battery_percentage:.1%}'
        if log_level == 'debug':
            self.get_logger().debug(log_msg)
        else:
            self.get_logger().info(log_msg)

    def publish_status(self):
        event_type = 'moving' if self.moving_state else 'stopped'
        self.send_event(event_type)

    def safe_shutdown(self):
        if not self.shutting_down:
            self.shutting_down = True
        self.get_logger().info('正在关闭状态发布节点...')
        try:
            self.timer.cancel()
            self.wifi_timer.cancel()
        except:
            pass
        # xxx
        try:
            self.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass
        # xxx

    def cleanup(self):
        self.get_logger().info('清理节点资源...')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RobotStatePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node and not node.shutting_down:
            node.get_logger().info('接收到Ctrl+C,发送关机事件...')
            node.shutting_down = True
            node.send_event('power_off')
            # xxx
    except Exception as e:
        if node:
            node.get_logger().error(f'节点异常: {e}')
            node.send_event('error', {'error_message': str(e)})
    finally:
        if node:
            node.cleanup()
            try:
                node.destroy_node()
            except:
                pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
