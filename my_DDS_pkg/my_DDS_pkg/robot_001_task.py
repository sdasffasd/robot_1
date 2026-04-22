#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import Trigger
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32MultiArray
import json
import argparse
import math    # xxx

class CommandWaypointServo(Node):
    def __init__(self, name):
        super().__init__(name)
        self.declare_parameter('robot_id', 'robot_001')
        self.declare_parameter('task_id', 1)
        self.robot_id = self.get_parameter('robot_id').value
        self.task_id = self.get_parameter('task_id').value
        self.nav_cb_group = ReentrantCallbackGroup()
        self.servo_cb_group = MutuallyExclusiveCallbackGroup()
        self.capture_cb_group = MutuallyExclusiveCallbackGroup()
        self.cmd_cb_group = ReentrantCallbackGroup()
        self.photo_cb_group = ReentrantCallbackGroup()
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self.nav_cb_group
        )
        self.servo_pub = self.create_publisher(
            Float32MultiArray, 'servo_control', 10,
            callback_group=self.servo_cb_group
        )
        self.progress_pub = self.create_publisher(
            String, '/robot/inspection/progress', 10,
            callback_group=self.servo_cb_group
        )
        self.photo_pub = self.create_publisher(
            CompressedImage, '/robot/inspection/photo', 10,
            callback_group=self.photo_cb_group
        )
        self.camera_image_sub = self.create_subscription(
            CompressedImage, '/robot/camera/image',
            self.camera_image_callback, 10,
            callback_group=self.photo_cb_group
        )
        self.capture_client = self.create_client(
            Trigger, 'capture_image',
            callback_group=self.capture_cb_group
        )
        self.command_sub = self.create_subscription(
            String, '/robot/command', self.command_callback, 10,
            callback_group=self.cmd_cb_group
        )
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )
        self.initial_position = [30.0, 90.0]
        self.left_position = [80.0, 0.0]
        self.capture_angles = [80.0, 60.0, 30.0]
        z90 = math.sin(math.pi / 4)
        w90 = math.cos(math.pi / 4)
        self.waypoints = {
            'origin': (0.0, 0.0, 0.0, 1.0, 0),
            'one': (3.50, -0.002128615975379944, z90, w90, 1),
            'two': (4.20, -0.002128615975379944, z90, w90, 2),
            'three': (4.60, -0.0097811259329319, z90, w90, 3),
            'four': (5.30, 0.015180479735136032, z90, w90, 4),
            'five': (5.70, 0.05292219579219818, z90, w90, 5),
            'six': (6.40, 0.092436215579509735, z90, w90, 6),
            'seven': (6.90, 0.1093964883685112, z90, w90, 7),
            'eight': (7.50, 0.15740046501159668, z90, w90, 8),
            'nine': (8.00, 0.15745352268218994, z90, w90, 9),
            'ten': (8.50, 0.15590383172035217, z90, w90, 10),
        }
        self.waypoint_sequence = [
            'one', 'two', 'three', 'four', 'five', 'six', 'seven',
            'eight', 'nine', 'ten', 'origin'
        ]
        self.is_task_running = False
        self.current_wp_index = 0
        self.current_servo_step = 0
        self.capture_success_count = 0
        self.nav_retry_count = 0
        self.retry_timer = None
        self.current_goal_handle = None
        self.task_paused = False
        self.paused_state = {}
        self.waiting_for_photo = False
        self.current_photo_metadata = {
            'robot_id': self.robot_id,
            'task_id': self.task_id,
            'point_id': '',
            'photo_index': 0
        }
        # xxx
        self.photo_start_time = None
        self.photo_timeout_timer = None
        self.delay_timers = []
        self.single_point_mode = False
        self.get_logger().info(f'Command waypoint servo node started (robot_id={self.robot_id}, task_id={self.task_id})')
        self.get_logger().info("等待命令: 发送 'start' 到 /robot/command 开始多点巡检")

    def publish_initial_pose(self, x=0.0, y=0.0, yaw=0.0):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance = [0.0] * 36
        self.initial_pose_pub.publish(msg)
        self.get_logger().info(f'已发布初始位姿: x={x}, y={y}, yaw={yaw}')

    def command_callback(self, msg):
        command = msg.data.strip().lower()
        self.get_logger().info(f'收到命令: {command}')
        if command == 'start':
            if self.is_task_running:
                self.get_logger().warn('任务已在运行中')
                return
            if not self.nav_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().error('导航服务不可用，无法启动任务')
                return
            if not self.capture_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn('拍照服务不可用，任务将继续但不拍照')
            self.start_task_async()
        elif command == 'pause_inspection':
            if not self.is_task_running:
                self.get_logger().warn('没有正在运行的任务，无法暂停')
                return
            if self.task_paused:
                self.get_logger().warn('任务已处于暂停状态')
                return
            self.pause_task()
        elif command == 'resume_inspection':
            if not self.task_paused:
                self.get_logger().warn('没有暂停的任务，无法恢复')
                return
            self.resume_task()
        elif command == 'cancel_inspection':
            self.get_logger().info('取消巡检任务，返回原点')
            self.cancel_task_and_return_home()
        elif command == 'emergency_stop':
            self.get_logger().warn('紧急停止！返回原点')
            self.cancel_task_and_return_home()
        elif command == 'return_home':
            self.get_logger().info('指令: 返回原点')
            self.return_to_origin()
        else:
            self.get_logger().warn(f'未知命令: {command}')

    def start_task_async(self):
        if self.is_task_running:
            self.get_logger().warn('任务已在运行中')
            return
        self.is_task_running = True
        self.task_paused = False
        self.paused_state.clear()
        self.current_wp_index = 0
        # xxx
        self.single_point_mode = False
        self.get_logger().info("=== 开始多点巡检（异步模式） ===")
        self.navigate_next()

    def pause_task(self):
        self.get_logger().info('正在暂停巡检任务...')
        if self.current_goal_handle:
            self.get_logger().info('取消当前导航目标')
            self.current_goal_handle.cancel_goal_async()
            self.current_goal_handle = None
        if self.retry_timer:
            self.destroy_timer(self.retry_timer)
            self.retry_timer = None
        self._clear_delay_timers()
        if self.photo_timeout_timer:
            self.photo_timeout_timer.cancel()
            self.destroy_timer(self.photo_timeout_timer)
            self.photo_timeout_timer = None
        self.paused_state = {
            'current_wp_index': self.current_wp_index,
            'current_servo_step': self.current_servo_step,
            'capture_success_count': self.capture_success_count,
            'nav_retry_count': self.nav_retry_count,
            'waiting_for_photo': self.waiting_for_photo,
            'current_photo_metadata': self.current_photo_metadata.copy(),
            'photo_start_time': self.photo_start_time,
        }
        self.task_paused = True
        self.is_task_running = False
        self.get_logger().info(f'任务已暂停，保存航点索引 {self.paused_state["current_wp_index"]}')
        wp_name = self.waypoint_sequence[self.current_wp_index]
        _, _, _, _, point_id = self.waypoints[wp_name]
        self.publish_point_status('paused', f'任务暂停，航点 {self._point_id_str(point_id)}', self.capture_success_count)

    def resume_task(self):
        if not self.task_paused or not self.paused_state:
            self.get_logger().error('无有效的暂停状态，无法恢复')
            return
        self.get_logger().info('正在恢复巡检任务...')
        self.current_wp_index = self.paused_state['current_wp_index']
        self.current_servo_step = self.paused_state['current_servo_step']
        self.capture_success_count = self.paused_state['capture_success_count']
        self.nav_retry_count = self.paused_state['nav_retry_count']
        self.waiting_for_photo = self.paused_state['waiting_for_photo']
        self.current_photo_metadata = self.paused_state['current_photo_metadata'].copy()
        self.photo_start_time = self.paused_state['photo_start_time']
        self.is_task_running = True
        self.task_paused = False
        self.paused_state.clear()
        wp_name = self.waypoint_sequence[self.current_wp_index]
        _, _, _, _, point_id = self.waypoints[wp_name]
        self.get_logger().info(f'恢复任务，当前航点 {point_id} ({wp_name})，舵机步骤 {self.current_servo_step}')
        if self.waiting_for_photo:
            self.get_logger().info('恢复等待拍照...')
            self._reset_photo_timeout()
            return
        if self.current_servo_step == 0:
            self._execute_servo_step()
        elif self.current_servo_step > 0 and self.current_servo_step < 5:
            self._execute_servo_step()
        else:
            self.get_logger().info('重新发送导航目标')
            self.navigate_next()

    def cancel_task_and_return_home(self):
        if self.is_task_running:
            self.is_task_running = False
        self.task_paused = False
        self.paused_state.clear()
        if self.current_goal_handle:
            self.current_goal_handle.cancel_goal_async()
            self.current_goal_handle = None
        if self.retry_timer:
            self.destroy_timer(self.retry_timer)
            self.retry_timer = None
        self._clear_delay_timers()
        if self.photo_timeout_timer:
            self.photo_timeout_timer.cancel()
            self.destroy_timer(self.photo_timeout_timer)
            self.photo_timeout_timer = None
        self.return_to_origin()

    def return_to_origin(self):
        if self.is_task_running:
            self.get_logger().info('任务正在运行，先取消任务再返回原点')
            self.is_task_running = False
            self.task_paused = False
            self.paused_state.clear()
            if self.current_goal_handle:
                self.current_goal_handle.cancel_goal_async()
                self.current_goal_handle = None
            if self.retry_timer:
                self.destroy_timer(self.retry_timer)
                self.retry_timer = None
            self._clear_delay_timers()
            if self.photo_timeout_timer:
                self.photo_timeout_timer.cancel()
                self.destroy_timer(self.photo_timeout_timer)
                self.photo_timeout_timer = None
        x, y, z, w, point_id = self.waypoints['origin']
        self.get_logger().info('导航到原点...')
        self.start_single_point_navigation(x, y, z, w, point_id, with_photo=False)

    def navigate_next(self):
        if not self.is_task_running:
            return
        if self.current_wp_index >= len(self.waypoint_sequence):
            self.get_logger().info("=== 多点巡检完成 ===")
            self.is_task_running = False
            return
        self.nav_retry_count = 0
        if self.retry_timer:
            self.destroy_timer(self.retry_timer)
            self.retry_timer = None
        wp_name = self.waypoint_sequence[self.current_wp_index]
        x, y, z, w, point_id = self.waypoints[wp_name]
        self.get_logger().info(f'导航到航点 {point_id} ({wp_name})')
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = z
        goal_msg.pose.pose.orientation.w = w
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('导航目标被拒绝')
            self.handle_navigation_failure()
            return
        self.current_goal_handle = goal_handle
        self.get_logger().info('导航目标已接受，等待到达...')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.on_navigation_result)

    def on_navigation_result(self, future):
        if not self.is_task_running:
            return
        self.current_goal_handle = None
        result = future.result()
        wp_name = self.waypoint_sequence[self.current_wp_index]
        _, _, _, _, point_id = self.waypoints[wp_name]
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'到达航点 {point_id} ({wp_name})')
            self.nav_retry_count = 0
            if self.retry_timer:
                self.destroy_timer(self.retry_timer)
                self.retry_timer = None
            if point_id > 0:
                self.publish_point_status('arrived', f'到达点位 {self._point_id_str(point_id)}', 0)
                self._start_delay(1.0, lambda: self._delayed_publish_started(point_id))
                self.start_servo_sequence()
            else:
                self.advance_to_next_waypoint()
        else:
            self.handle_navigation_failure()

    def _delayed_publish_started(self, point_id):
        if not self.is_task_running:
            return
        if self.current_wp_index >= len(self.waypoint_sequence):
            return
        wp_name = self.waypoint_sequence[self.current_wp_index]
        _, _, _, _, current_point_id = self.waypoints[wp_name]
        if current_point_id == point_id:
            self.publish_point_status('started', f'开始巡检 {self._point_id_str(point_id)}', 0)

    def _delayed_publish_data_transferred(self, point_id, photos_taken):
        if not self.is_task_running:
            return
        if self.current_wp_index >= len(self.waypoint_sequence):
            return
        wp_name = self.waypoint_sequence[self.current_wp_index]
        _, _, _, _, current_point_id = self.waypoints[wp_name]
        if current_point_id == point_id:
            self.publish_point_status('data_transferred', f'数据传输完成', photos_taken)

    def handle_navigation_failure(self):
        if not self.is_task_running:
            return
        wp_name = self.waypoint_sequence[self.current_wp_index]
        _, _, _, _, point_id = self.waypoints[wp_name]
        self.nav_retry_count += 1
        self.get_logger().warn(f'导航失败，重试次数 {self.nav_retry_count}/10 (当前航点 {point_id})')
        if self.nav_retry_count < 10:
            self.get_logger().info('10秒后重试当前航点...')
            self.retry_timer = self.create_timer(10.0, self._retry_navigation)
        else:
            self.get_logger().error(f'导航失败已达10次,任务终止 (航点 {point_id})')
            self.publish_point_status('nav_failed', f'导航失败已达10次,任务终止', 0)
            self.is_task_running = False
            if self.retry_timer:
                self.destroy_timer(self.retry_timer)
                self.retry_timer = None

    def _retry_navigation(self):
        if self.retry_timer:
            self.destroy_timer(self.retry_timer)
            self.retry_timer = None
        if self.is_task_running:
            self.navigate_next()

    def start_servo_sequence(self):
        if not self.is_task_running:
            return
        wp_name = self.waypoint_sequence[self.current_wp_index]
        _, _, _, _, point_id = self.waypoints[wp_name]
        self.publish_point_status('executing', f'正在执行拍照任务', 0)
        self.get_logger().info("=== 开始舵机序列 ===")
        self.current_servo_step = 0
        self.capture_success_count = 0
        self._execute_servo_step()

    def _execute_servo_step(self):
        if not self.is_task_running:
            return
        if self.current_servo_step == 0:
            self.get_logger().info('舵机左转...')
            self.set_servo_position(self.left_position)
            self._start_delay(2.0, self._execute_servo_step)
            self.current_servo_step = 1
            return
        angle_index = self.current_servo_step - 1
        if angle_index < len(self.capture_angles):
            angle = self.capture_angles[angle_index]
            self.get_logger().info(f'设置 S3={angle}°, S4=0.0°...')
            self.set_servo_position([angle, 0.0])
            self._start_delay(1.5, lambda: self._capture_at_current_angle())
            return
        if self.current_servo_step == 4:
            self.get_logger().info('舵机返回初始位置...')
            self.set_servo_position(self.initial_position)
            self._start_delay(1.0, self._finish_servo_sequence)
            self.current_servo_step = 5
            return

    def _capture_at_current_angle(self):
        if not self.is_task_running:
            return
        angle_index = self.current_servo_step - 1
        angle = self.capture_angles[angle_index]
        self.get_logger().info(f'拍照: S3={angle}°...')
        wp_name = self.waypoint_sequence[self.current_wp_index]
        _, _, _, _, point_id = self.waypoints[wp_name]
        self.current_photo_metadata = {
            'robot_id': self.robot_id,
            'task_id': self.task_id,
            'point_id': self._point_id_str(point_id),
            'photo_index': angle_index + 1
        }
        self.waiting_for_photo = True
        self.photo_start_time = self.get_clock().now()
        self._reset_photo_timeout()
        if not self.capture_client.service_is_ready():
            self.get_logger().warn('拍照服务不可用，跳过此张照片')
            self._on_photo_timeout()
            return
        request = Trigger.Request()
        call_future = self.capture_client.call_async(request)
        call_future.add_done_callback(self._on_capture_done)

    def _reset_photo_timeout(self):
        if self.photo_timeout_timer:
            self.photo_timeout_timer.cancel()
            self.destroy_timer(self.photo_timeout_timer)
            self.photo_timeout_timer = None
        self.photo_timeout_timer = self.create_timer(5.0, self._on_photo_timeout)

    def _on_photo_timeout(self):
        if self.photo_timeout_timer:
            self.photo_timeout_timer.cancel()
            self.destroy_timer(self.photo_timeout_timer)
            self.photo_timeout_timer = None
        if self.waiting_for_photo:
            self.get_logger().warn('拍照超时，跳过此张照片')
            self.waiting_for_photo = False
            self._after_capture(False)

    def _on_capture_done(self, future):
        if not self.is_task_running:
            if self.photo_timeout_timer:
                self.photo_timeout_timer.cancel()
                self.destroy_timer(self.photo_timeout_timer)
                self.photo_timeout_timer = None
            self.waiting_for_photo = False
            return
        try:
            result = future.result()
            if result.success:
                self.get_logger().info(f'拍照成功: {result.message}')
                self.capture_success_count += 1
            else:
                self.get_logger().warn(f'拍照失败: {result.message}')
                if self.photo_timeout_timer:
                    self.photo_timeout_timer.cancel()
                    self.destroy_timer(self.photo_timeout_timer)
                    self.photo_timeout_timer = None
                self.waiting_for_photo = False
                self._after_capture(False)
                return
        except Exception as e:
            self.get_logger().error(f'拍照服务异常: {e}')
            if self.photo_timeout_timer:
                self.photo_timeout_timer.cancel()
                self.destroy_timer(self.photo_timeout_timer)
                self.photo_timeout_timer = None
            self.waiting_for_photo = False
            self._after_capture(False)
            return

    def _after_capture(self, success=True):
        if not self.is_task_running:
            return
        self.current_servo_step += 1
        angle_index = self.current_servo_step - 1
        if angle_index < len(self.capture_angles):
            self._execute_servo_step()
        else:
            self.current_servo_step = 4
            self._execute_servo_step()

    def _finish_servo_sequence(self):
        wp_name = self.waypoint_sequence[self.current_wp_index]
        _, _, _, _, point_id = self.waypoints[wp_name]
        self.get_logger().info(f"当前航点拍照完成: {self.capture_success_count}/{len(self.capture_angles)} 张成功")
        self.publish_point_status('completed', f'拍照任务已执行完毕', self.capture_success_count)
        self._start_delay(1.0, lambda: self._delayed_publish_data_transferred(point_id, self.capture_success_count))
        self.advance_to_next_waypoint()

    def publish_point_status(self, point_status, message, photos_taken=0, point_id_override=None):
        if point_id_override is not None:
            point_id_str = self._point_id_str(point_id_override)
            point_index = self.current_wp_index if not self.single_point_mode else 0
        else:
            if self.current_wp_index >= len(self.waypoint_sequence):
                return
            wp_name = self.waypoint_sequence[self.current_wp_index]
            _, _, _, _, point_id_num = self.waypoints[wp_name]
            point_id_str = self._point_id_str(point_id_num)
            point_index = self.current_wp_index
        progress_msg = {
            "robot_id": self.robot_id,
            "task_id": self.task_id,
            "point_id": point_id_str,
            "point_index": point_index,
            "point_status": point_status,
            "photos_taken": photos_taken,
            "message": message
        }
        json_str = json.dumps(progress_msg, ensure_ascii=False)
        msg = String()
        msg.data = json_str
        self.progress_pub.publish(msg)
        self.get_logger().info(f'[进度] {point_id_str} {point_status} 照片:{photos_taken}')

    @staticmethod
    def _point_id_str(num):
        return f'P{num:03d}'

    def camera_image_callback(self, msg):
        if not self.waiting_for_photo:
            return
        now = self.get_clock().now()
        if self.photo_start_time is not None and (now - self.photo_start_time).nanoseconds > 6e9:
            self.waiting_for_photo = False
            if self.photo_timeout_timer:
                self.photo_timeout_timer.cancel()
                self.destroy_timer(self.photo_timeout_timer)
                self.photo_timeout_timer = None
            self._after_capture(False)
            return
        md = self.current_photo_metadata
        frame_id = f"{md['robot_id']}|{md['task_id']}|{md['point_id']}|{md['photo_index']}"
        msg.header.frame_id = frame_id
        self.photo_pub.publish(msg)
        self.get_logger().info(f'[照片转发] {frame_id}')
        self.waiting_for_photo = False
        if self.photo_timeout_timer:
            self.photo_timeout_timer.cancel()
            self.destroy_timer(self.photo_timeout_timer)
            self.photo_timeout_timer = None
        self._after_capture()

    def advance_to_next_waypoint(self):
        self.current_wp_index += 1
        self.navigate_next()

    def set_servo_position(self, angles):
        msg = Float32MultiArray()
        msg.data = [float(a) for a in angles]
        self.servo_pub.publish(msg)
        self.get_logger().info(f'设置舵机角度: S3={angles[0]}°, S4={angles[1]}°')

    def _start_delay(self, seconds, callback):
        timer = self.create_timer(seconds, lambda: self._delay_callback(timer, callback))
        self.delay_timers.append(timer)

    def _delay_callback(self, timer, callback):
        if timer in self.delay_timers:
            self.delay_timers.remove(timer)
        timer.cancel()
        self.destroy_timer(timer)
        callback()

    def _clear_delay_timers(self):
        for timer in self.delay_timers:
            timer.cancel()
            self.destroy_timer(timer)
        self.delay_timers.clear()

    def start_single_point_navigation(self, x, y, z, w, point_id, with_photo=True):
        self.nav_retry_count = 0
        if self.retry_timer:
            self.destroy_timer(self.retry_timer)
            self.retry_timer = None
        self.single_point_mode = True
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = z
        goal_msg.pose.pose.orientation.w = w
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(
            lambda future: self._single_point_goal_response(future, point_id, with_photo)
        )

    def _single_point_goal_response(self, future, point_id, with_photo=True):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('单点导航目标被拒绝')
            self._handle_single_point_failure(point_id, with_photo)
            return
        self.current_goal_handle = goal_handle
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(
            lambda fut: self._single_point_result(fut, point_id, with_photo)
        )

    def _single_point_result(self, future, point_id, with_photo):
        result = future.result()
        self.current_goal_handle = None
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'单点导航到 {point_id} 成功')
            self.nav_retry_count = 0
            if self.retry_timer:
                self.destroy_timer(self.retry_timer)
                self.retry_timer = None
            if point_id > 0 and with_photo:
                self.publish_point_status('arrived', f'到达点位 {self._point_id_str(point_id)}', 0, point_id_override=point_id)
                self._start_delay(1.0, lambda: self.publish_point_status('started', f'开始巡检 {self._point_id_str(point_id)}', 0, point_id_override=point_id))
                self.current_wp_index = 0
                self.start_servo_sequence()
            else:
                self.get_logger().info('到达原点或无拍照任务')
        else:
            self._handle_single_point_failure(point_id, with_photo)
        self.single_point_mode = False

    def _handle_single_point_failure(self, point_id, with_photo=True):
        self.nav_retry_count += 1
        self.get_logger().warn(f'单点导航失败，重试次数 {self.nav_retry_count}/10 (航点 {point_id})')
        if self.nav_retry_count < 10:
            self.get_logger().info('10秒后重试单点导航...')
            self.retry_timer = self.create_timer(10.0, lambda: self._retry_single_point(point_id, with_photo))
        else:
            self.get_logger().error(f'单点导航失败已达10次,任务终止 (航点 {point_id})')
            self.publish_point_status('nav_failed', f'单点导航失败已达10次,任务终止', 0, point_id_override=point_id)
            self.current_wp_index = 0
            if self.retry_timer:
                self.destroy_timer(self.retry_timer)
                self.retry_timer = None
        self.single_point_mode = False

    def _retry_single_point(self, point_id, with_photo=True):
        if self.retry_timer:
            self.destroy_timer(self.retry_timer)
            self.retry_timer = None
        for name, (x, y, z, w, pid) in self.waypoints.items():
            if pid == point_id:
                self.start_single_point_navigation(x, y, z, w, point_id, with_photo)
                return
        self.get_logger().error(f'无法找到航点 {point_id} 的坐标，重试失败')
        self.single_point_mode = False

    def wait_for_services(self, timeout=10.0):
        self.get_logger().info('等待导航服务...')
        if not self.nav_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error('导航服务不可用')
            return False
        self.get_logger().info('导航服务已连接')
        if not self.capture_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().warn('拍照服务不可用')
        return True


def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description='Navigate to waypoints with servo control (async)')
    parser.add_argument('--loop', action='store_true', help='Loop through all waypoints')
    parser.add_argument('--point', type=str, choices=['origin', 'one', 'two', 'three', 'four', 'five', 'six', 'seven',
                                                      'eight', 'nine', 'ten', 'eleven', 'twelve', 'thirteen',
                                                      'fourteen', 'fifteen', 'sixteen'],
                       help='Navigate to specific point')
    args, unknown = parser.parse_known_args()
    node = CommandWaypointServo('robot_001_task')
    if not node.wait_for_services():
        node.get_logger().error('服务不可用，节点退出')
        node.destroy_node()
        rclpy.shutdown()
        return
    node.publish_initial_pose(0.0, 0.0, 0.0)
    if args.loop:
        node.get_logger().info("命令行参数 --loop 触发，直接启动多点巡检")
        node.start_task_async()
    elif args.point:
        node.get_logger().info(f"命令行参数 --point 触发，单点导航至 {args.point}")
        x, y, z, w, point_id = node.waypoints[args.point]
        with_photo = (point_id > 0)
        node.start_single_point_navigation(x, y, z, w, point_id, with_photo)
    else:
        node.get_logger().info("无参数模式，等待 '/robot/command' 命令启动...")
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("用户中断")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
