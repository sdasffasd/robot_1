#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import cv2
import threading
import time
import os
import datetime
import numpy as np

class RobotCameraNode(Node):
    def __init__(self):
        super().__init__('robot_001_camera')
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('frame_width', 1280)
        self.declare_parameter('frame_height', 720)
        self.declare_parameter('fps', 5)
        self.declare_parameter('save_path', '/root/pictures')
        self.camera_index = self.get_parameter('camera_index').value
        self.frame_width = self.get_parameter('frame_width').value
        self.frame_height = self.get_parameter('frame_height').value
        self.fps = self.get_parameter('fps').value
        self.save_path = self.get_parameter('save_path').value
        os.makedirs(self.save_path, exist_ok=True)
        self.video_streaming = False
        self.camera_initialized = False
        self.camera = None
        self.create_publishers_and_services()
        self.frame_count = 0
        self.start_time = time.time()
        self.last_frame = None
        self.last_frame_time = 0
        self.lock = threading.Lock()
        self.timer = None
        self.capture_requested = False
        self.capture_frame = None
        self.capture_event = threading.Event()
        self.get_logger().info(
            f"机器人摄像头节点启动 - 分辨率: {self.frame_width}x{self.frame_height}, "
            f"帧率: {self.fps}, 保存路径: {self.save_path}"
        )
        self.get_logger().info("等待命令启动视频流...")

    def create_publishers_and_services(self):
        self.video_publisher = self.create_publisher(
            CompressedImage,
            '/robot/camera/video_stream',
            10
        )
        self.capture_publisher = self.create_publisher(
            CompressedImage,
            '/robot/camera/image',
            10
        )
        self.command_subscriber = self.create_subscription(
            String,
            '/robot/command',
            self.command_callback,
            10
        )
        self.capture_service = self.create_service(
            Trigger,
            'capture_image',
            self.capture_callback
        )
        self.status_service = self.create_service(
            Trigger,
            '/robot/camera/get_status',
            self.status_callback
        )
        self.get_logger().info("发布者、订阅者和服务创建完成")

    def command_callback(self, msg):
        command = msg.data.strip().lower()
        if command == "start_video":
            self.start_video_stream()
        elif command == "stop_video":
            self.stop_video_stream()
        else:
            self.get_logger().warn(f"未知命令: {command}")

    def start_video_stream(self):
        if self.video_streaming:
            self.get_logger().info("视频流已经在运行")
            return
        try:
            if not self.camera_initialized:
                self.initialize_camera()
            timer_period = 1.0 / self.fps if self.fps > 0 else 0.033
            self.timer = self.create_timer(timer_period, self.publish_video_frame)
            self.video_streaming = True
            self.get_logger().info(f"开始发布视频流，帧率: {self.fps}Hz")
        except Exception as e:
            self.get_logger().error(f"启动视频流失败: {e}")
            self.video_streaming = False

    def stop_video_stream(self):
        if not self.video_streaming:
            self.get_logger().info("视频流已经停止")
            return
        try:
            if self.timer is not None:
                self.destroy_timer(self.timer)
                self.timer = None
            self.video_streaming = False
            self.get_logger().info("视频流已停止")
            # xxx
        except Exception as e:
            self.get_logger().error(f"停止视频流失败: {e}")

    def initialize_camera(self):
        try:
            self.camera = cv2.VideoCapture(self.camera_index)
            if not self.camera.isOpened():
                raise Exception(f"无法打开摄像头 (index={self.camera_index})")
            self.set_camera_properties()
            self.camera_initialized = True
            self.get_logger().info("摄像头初始化成功")
        except Exception as e:
            self.get_logger().error(f"摄像头初始化失败: {e}")
            self.camera_initialized = False
            raise

    def set_camera_properties(self):
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        self.camera.set(cv2.CAP_PROP_FPS, self.fps)
        self.camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
        self.camera.set(cv2.CAP_PROP_BRIGHTNESS, 60)
        self.camera.set(cv2.CAP_PROP_CONTRAST, 50)
        self.camera.set(cv2.CAP_PROP_SATURATION, 50)
        self.camera.set(cv2.CAP_PROP_HUE, 50)
        self.camera.set(cv2.CAP_PROP_EXPOSURE, 150)
        self.camera.set(cv2.CAP_PROP_GAIN, 20)
        self.camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        actual_width = int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.camera.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(
            f"摄像头设置 - 分辨率: {actual_width}x{actual_height}, "
            f"帧率: {actual_fps:.1f}"
        )

    def release_camera(self):
        if self.camera is not None:
            self.camera.release()
            self.camera = None
            self.camera_initialized = False
            self.get_logger().info("摄像头已释放")

    def publish_video_frame(self):
        if not self.video_streaming:
            return
        try:
            with self.lock:
                if not self.camera_initialized or self.camera is None:
                    self.get_logger().warn("摄像头未初始化，跳过帧发布")
                    return
                ret, frame = self.camera.read()
                if not ret:
                    self.get_logger().warn("读取视频帧失败")
                    return
                self.last_frame = frame.copy()
                self.last_frame_time = time.time()
                self.frame_count += 1
                if self.frame_count % 30 == 0:
                    elapsed_time = time.time() - self.start_time
                    actual_fps = self.frame_count / elapsed_time
                    self.get_logger().debug(
                        f"已发布 {self.frame_count} 帧, "
                        f"实际FPS: {actual_fps:.1f}"
                    )
                self.publish_compressed_frame(frame, self.video_publisher)
        except Exception as e:
            self.get_logger().error(f"发布视频帧时出错: {e}")

    def capture_callback(self, request, response):
        try:
            self.get_logger().info("收到抓拍请求")
            if self.video_streaming:
                #  xxx
                with self.lock:
                    if self.last_frame is not None and time.time() - self.last_frame_time < 0.1:
                        frame = self.last_frame.copy()
                        self.get_logger().info("从视频流中抓拍")
                    else:
                        # xxx
                        ret, frame = self.camera.read()
                        if not ret:
                            raise Exception("无法读取摄像头帧")
            else:
                self.get_logger().info("视频流未运行，临时打开摄像头抓拍")
                temp_camera = cv2.VideoCapture(self.camera_index)
                if not temp_camera.isOpened():
                    raise Exception("无法临时打开摄像头")
                temp_camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
                temp_camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
                ret, frame = temp_camera.read()
                temp_camera.release()
                if not ret:
                    raise Exception("临时摄像头读取失败")
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            filename = os.path.join(self.save_path, f"capture_{timestamp}.jpg")
            cv2.imwrite(filename, frame)
            self.publish_compressed_frame(frame, self.capture_publisher)
            height, width = frame.shape[:2]
            self.get_logger().info(
                f"抓拍成功 - 文件: {filename}, "
                f"尺寸: {width}x{height}"
            )
            response.success = True
            response.message = f"图像已保存到: {filename}"
        except Exception as e:
            self.get_logger().error(f"抓拍失败: {e}")
            response.success = False
            response.message = f"抓拍失败: {str(e)}"
        return response

    def status_callback(self, request, response):
        try:
            elapsed_time = time.time() - self.start_time
            actual_fps = self.frame_count / elapsed_time if elapsed_time > 0 else 0
            status_info = {
                "node_name": "robot_001_camera",
                "video_streaming": self.video_streaming,
                "camera_initialized": self.camera_initialized,
                "frame_count": self.frame_count,
                "actual_fps": round(actual_fps, 1),
                "resolution": f"{self.frame_width}x{self.frame_height}",
                "save_path": self.save_path,
                "uptime": round(elapsed_time, 1)
            }
            response.success = True
            response.message = str(status_info)
        except Exception as e:
            response.success = False
            response.message = f"获取状态失败: {str(e)}"
        return response

    def publish_compressed_frame(self, frame, publisher):
        try:
            compressed_msg = CompressedImage()
            compressed_msg.header.stamp = self.get_clock().now().to_msg()
            compressed_msg.header.frame_id = "camera_frame"
            compressed_msg.format = "jpeg"
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
            ret, compressed_data = cv2.imencode('.jpg', frame, encode_param)
            if ret:
                compressed_msg.data = compressed_data.tobytes()
                publisher.publish(compressed_msg)
            else:
                self.get_logger().warn("图像压缩失败")
        except Exception as e:
            self.get_logger().error(f"发布压缩图像失败: {e}")

    def destroy_node(self):
        self.get_logger().info("正在关闭摄像头节点...")
        elapsed_time = time.time() - self.start_time
        final_fps = self.frame_count / elapsed_time if elapsed_time > 0 else 0
        self.stop_video_stream()
        self.release_camera()
        self.get_logger().info(
            f"节点运行统计 - 总帧数: {self.frame_count}, "
            f"平均FPS: {final_fps:.1f}, "
            f"运行时间: {elapsed_time:.1f}秒"
        )
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = RobotCameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到Ctrl+C,正在关闭节点...")
    except Exception as e:
        node.get_logger().error(f"节点运行出错: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
