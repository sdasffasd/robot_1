#!/usr/bin/env python3
"""
简单舵机控制脚本 - 纯交互式控制
用法: python3 steering_engine.py
进入交互后,输入两个数字(S3角度 S4角度)即可设置舵机,输入 quit 退出。
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time

class SimpleServoController(Node):
    def __init__(self):
        super().__init__('simple_servo_controller')
        
        # 创建舵机控制发布者
        self.servo_pub = self.create_publisher(Float32MultiArray, 'servo_control', 10)
        
        # 舵机角度限制
        self.s3_min = 0.0
        self.s3_max = 180.0
        self.s4_min = 0.0
        self.s4_max = 180.0
        
        self.get_logger().info("舵机控制节点已启动")
        self.get_logger().info("请输入 S3角度 S4角度(用空格分隔),输入 quit 退出")
        
    def set_servo_position(self, angles):
        """设置舵机角度"""
        try:
            s3_angle = float(angles[0])
            s4_angle = float(angles[1])
            
            # 限制角度范围
            s3_angle = max(self.s3_min, min(self.s3_max, s3_angle))
            s4_angle = max(self.s4_min, min(self.s4_max, s4_angle))
            
            # 发布控制消息
            msg = Float32MultiArray()
            msg.data = [s3_angle, s4_angle]
            self.servo_pub.publish(msg)
            
            self.get_logger().info(f"设置舵机角度: S3={s3_angle:.1f}°, S4={s4_angle:.1f}°")
            time.sleep(0.1)  # 确保消息发送
            
        except Exception as e:
            self.get_logger().error(f"设置舵机角度失败: {e}")
    
    def run_interactive(self):
        """交互式控制循环"""
        while rclpy.ok():
            try:
                user_input = input("\n请输入 S3角度 S4角度: ").strip().lower()
                
                if user_input == 'quit' or user_input == 'exit':
                    self.get_logger().info("退出程序")
                    break
                
                # 解析两个数字
                parts = user_input.split()
                if len(parts) == 2:
                    try:
                        s3 = float(parts[0])
                        s4 = float(parts[1])
                        self.set_servo_position([s3, s4])
                    except ValueError:
                        self.get_logger().error("输入无效,请输入两个数字,例如:90 90")
                else:
                    self.get_logger().error("请输入两个角度值，用空格分隔")
                    
            except KeyboardInterrupt:
                self.get_logger().info("\n收到中断信号")
                break
            except Exception as e:
                self.get_logger().error(f"处理输入时出错: {e}")
        
        # 退出前可选复位（根据需求可保留或移除）
        # self.set_servo_position([30.0, 90.0])
        # time.sleep(0.5)

def main():
    rclpy.init()
    controller = SimpleServoController()
    
    try:
        time.sleep(1.0)  # 等待发布者连接
        controller.run_interactive()
    except Exception as e:
        controller.get_logger().error(f"程序运行出错: {e}")
    finally:
        controller.destroy_node()
        rclpy.shutdown()
        print("程序已退出")

if __name__ == '__main__':
    main()