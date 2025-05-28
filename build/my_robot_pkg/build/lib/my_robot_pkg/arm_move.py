#!/usr/bin/env python3
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from threading import Thread
import time
from pymoveit2 import MoveIt2
import numpy as np
from scipy.spatial.transform import Rotation as R
import tf2_ros
from geometry_msgs.msg import TransformStamped
from my_robot_pkg_msg.msg import ChessMove
from threading import Thread, Lock
from std_msgs.msg import Int8,Bool,Int32MultiArray,Float64
from queue import Queue


class ArmMoveWithInput(Node):
    def __init__(self):
        super().__init__("arm_move_with_input")

        self.arm_joint_names = ["j1", "j2", "j3", "j4", "j5", "j6"]
        self.group_name = "dummy"
        self.base_link = "base_link"
        self.l5 = "l5"
        self.end_effector = "l6"
        self.black = 1
        self.white = 1
        self.action_queue = Queue()  # 动作队列
        self.queue_lock = Lock()  # 队列锁
        self.ready_to_process = False  # 等待开始信号后才执行队列


        callback_group = ReentrantCallbackGroup()
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=self.arm_joint_names,
            base_link_name=self.base_link,
            end_effector_name=self.end_effector,
            group_name=self.group_name,
            callback_group=callback_group,
        )

        self.moveit2.planner_id = "RRTConnect"
        self.moveit2.max_velocity = 1.0
        self.moveit2.max_acceleration = 1.0

        # TF2监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.subscription = self.create_subscription(ChessMove,
                                                "/move",
                                                self.move_chess,
                                                1)
        self.create_subscription(
                                Int32MultiArray,
                                '/correction_move',
                                self.move_correction,
                                1
                            )
        self.create_subscription(
                                Bool,
                                '/start_exec',
                                self.start_exec_callback,
                                1
                            )

        self.grip_publisher = self.create_publisher(Int8, '/grip', 10)      
        self.chessboard_angle = 0.0  # 默认旋转角度
        self.chessboard_center = [0.3350, 0.0550, -0.008]  # 格子5的坐标作为旋转中心
        self.angle_subscription = self.create_subscription(
            Float64,
            '/chessboard_angle',
            self.angle_callback,
            10
        )
        self.action_thread = Thread(target=self.process_action_queue, daemon=True)
        self.action_thread.start()                                  

        self.commands = {
            "init": ([0.3272, 0.042798, 0.35154], [0.498732, 0.510627, 0.500207, 0.49223]),
            "0": ([0.228716, 0.040018, 0.241234], [0.0, 0.0, 0.71,0.7]),
            "4": ([0.3350, 0.0250, -0.008], [0.0, 0.0, 0.71, 0.7]),
            "5": ([0.3350, 0.0550, -0.008], [0.0, 0.0, 0.71, 0.7]),
            "6": ([0.3350, 0.0850, -0.008], [0.0, 0.0, 0.71, 0.7]),
            "7": ([0.3650, 0.0250, -0.008], [0.0, 0.0, 0.71, 0.7]),
            "8": ([0.3650, 0.0550, -0.008], [0.0, 0.0, 0.71, 0.7]),
            "9": ([0.3657, 0.0880, -0.008], [0.0, 0.0, 0.71, 0.7]),
            "1": ([0.305, 0.0250, -0.008], [0.0, 0.0, 0.71, 0.7]),
            "2": ([0.305, 0.0550, -0.008], [0.0, 0.0, 0.71, 0.7]),
            "3": ([0.305, 0.0880, -0.008], [0.0, 0.0, 0.71, 0.7]),

            "4_up": ([0.3350, 0.0250, 0.05], [0.0, 0.0, 0.71, 0.7]),
            "5_up": ([0.3350, 0.0550, 0.05], [0.0, 0.0, 0.71, 0.7]),
            "6_up": ([0.3350, 0.0850, 0.05], [0.0, 0.0, 0.71, 0.7]),
            "7_up": ([0.3650, 0.0250, 0.05], [0.0, 0.0, 0.71, 0.7]),
            "8_up": ([0.3650, 0.0550, 0.05], [0.0, 0.0, 0.71, 0.7]),
            "9_up": ([0.3657, 0.0880, 0.05], [0.0, 0.0, 0.71, 0.7]),
            "1_up": ([0.305, 0.0250,  0.05], [0.0, 0.0, 0.71, 0.7]),
            "2_up": ([0.305, 0.0550,  0.05], [0.0, 0.0, 0.71, 0.7]),
            "3_up": ([0.305, 0.0880,  0.05], [0.0, 0.0, 0.71, 0.7]),

            "white_1": ([0.3300, -0.1550, -0.008], [0.0, 0.0, 0.071, 0.7]),
            "white_2": ([0.3000, -0.1550, -0.008], [0.0, 0.0, 0.071, 0.7]),
            "white_3": ([0.2700, -0.1550, -0.008], [0.0, 0.0, 0.071, 0.7]),
            "white_4": ([0.2300, -0.1550, -0.008], [0.0, 0.0, 0.071, 0.7]),
            "white_5": ([0.2000, -0.1550, -0.008], [0.0, 0.0, 0.071, 0.7]),

            "white_1_up": ([0.3300, -0.1550, 0.05], [0.0, 0.0, 0.071, 0.7]),
            "white_2_up": ([0.3000, -0.1550, 0.05], [0.0, 0.0, 0.071, 0.7]),
            "white_3_up": ([0.2700, -0.1550, 0.05], [0.0, 0.0, 0.071, 0.7]),
            "white_4_up": ([0.2300, -0.1550, 0.05], [0.0, 0.0, 0.071, 0.7]),
            "white_5_up": ([0.2000, -0.1550, 0.05], [0.0, 0.0, 0.071, 0.7]),


            "black_1_up": ([0.3150, 0.2450, 0.05], [0.0, 0.0, 0.71, 0.7]),
            "black_2_up": ([0.2850, 0.2400, 0.05], [0.0, 0.0, 0.71, 0.7]),
            "black_3_up": ([0.2550, 0.2350, 0.05], [0.0, 0.0, 0.71, 0.7]),
            "black_4_up": ([0.2250, 0.2350, 0.05], [0.0, 0.0, 0.71, 0.7]),
            "black_5_up": ([0.1950, 0.2350, 0.05], [0.0, 0.0, 0.71, 0.7]),

            "black_1": ([0.3150, 0.2450, -0.008], [0.0, 0.0, 0.71, 0.7]),
            "black_2": ([0.2850, 0.2400, -0.008], [0.0, 0.0, 0.71, 0.7]),
            "black_3": ([0.2550, 0.2350, -0.008], [0.0, 0.0, 0.71, 0.7]),
            "black_4": ([0.2250, 0.2350, -0.008], [0.0, 0.0, 0.71, 0.7]),
            "black_5": ([0.1950, 0.2350, -0.010], [0.0, 0.0, 0.71, 0.7])
        }
        self.original_commands = self.commands.copy()  # 保存原始坐标

    def angle_callback(self, msg):
            """接收棋盘旋转角度并更新棋格坐标"""
            with self.queue_lock:
                if abs(self.chessboard_angle - msg.data) > 1.0:  # 仅在角度变化超过1度时更新
                    self.chessboard_angle = msg.data
                    self.update_grid_coordinates()
                    self.get_logger().info(f"收到棋盘旋转角度：{self.chessboard_angle:.2f} 度，坐标已更新")

    def update_grid_coordinates(self):
        """根据旋转角度更新棋格1-9及其 _up 版本的坐标"""
        angle_rad = np.deg2rad(self.chessboard_angle)
        cos_theta = np.cos(angle_rad)
        sin_theta = np.sin(angle_rad)
        
        # 旋转矩阵
        rotation_matrix = np.array([
            [cos_theta, -sin_theta],
            [sin_theta, cos_theta]
        ])

        # 更新棋格1-9及其 _up 版本的坐标
        for grid in ["1", "2", "3", "4", "5", "6", "7", "8", "9",
                    "1_up", "2_up", "3_up", "4_up", "5_up", "6_up", "7_up", "8_up", "9_up"]:
            if grid in self.original_commands:
                pos, quat = self.original_commands[grid]
                # 将坐标相对于棋盘中心平移
                relative_pos = np.array([
                    pos[0] - self.chessboard_center[0],
                    pos[1] - self.chessboard_center[1]
                ])
                # 应用旋转
                rotated_pos = rotation_matrix @ relative_pos
                # 转换回绝对坐标
                new_pos = [
                    rotated_pos[0] + self.chessboard_center[0],
                    rotated_pos[1] + self.chessboard_center[1],
                    pos[2]  # 保持 z 不变
                ]
                # 更新 commands 字典
                self.commands[grid] = (new_pos, quat)
                self.get_logger().debug(f"更新格子 {grid} 坐标：{new_pos}")

    def start_exec_callback(self, msg: Bool):
        """接收启动指令，允许执行动作队列"""
        if msg.data:
            with self.queue_lock:
                self.ready_to_process = True
            self.get_logger().info("收到开始执行信号，开始处理动作队列")

    def wait_until_success(self, command: str, retry_delay=1):
        """发送命令并等待返回 True"""
        while True:
            success = self.send_command(command)
            time.sleep(retry_delay)
            if success:
                return
    
    def move_correction(self,msg):
        if len(msg.data) == 2:
            original_pos = msg.data[0]+1
            moved_pos = msg.data[1]+1
            self.move_correction_action(original_pos, moved_pos)
    
    def move_correction_action(self, original_pos, moved_pos):
        """处理 ChessMove 动作"""
        original_pos = str(original_pos)
        moved_pos = str(moved_pos)
        self.wait_until_success(moved_pos+f"_up")
        self.wait_until_success(moved_pos)
        move_msg = Int8()
        move_msg.data = 1
        self.grip_publisher.publish(move_msg)
        self.wait_until_success(moved_pos+f"_up")
        self.wait_until_success(original_pos+f"_up")
        self.wait_until_success(original_pos)
        move_msg = Int8()
        move_msg.data = 0
        self.grip_publisher.publish(move_msg)
        self.wait_until_success(original_pos+f"_up")
        self.wait_until_success("0")
        

    def move_chess(self, msg):
        """处理 ChessMove 消息（task2 和 task3）"""
        with self.queue_lock:
            self.action_queue.put(('chess', (msg.grid_index+1), msg.color))
            self.get_logger().info(f"添加 ChessMove 动作到队列: grid_index={msg.grid_index}, color={msg.color}")

    def process_action_queue(self):
        """处理动作队列，在收到启动信号前不执行"""
        while True:
            time.sleep(0.05)
            with self.queue_lock:
                if not self.ready_to_process or self.action_queue.empty():
                    continue

                action = self.action_queue.get()
                if action[0] == 'chess':
                    self.move_chess_action(action[1], action[2])
                if not action[1] :
                    self.ready_to_process = False
                self.action_queue.task_done()

   

    
    def move_chess_action(self, grid_index, color):
        """处理 ChessMove 动作"""
        grid_index = str(grid_index)
        if color == "black" :
            prefix = "black" 
            chess_key = f"{prefix}_{self.black}"
            self.black = (self.black % 5) + 1
        elif color =="white" :
            prefix = "white"
            chess_key = f"{prefix}_{self.white}"
            self.white = (self.white % 5) + 1
        self.get_logger().info(f"发布抓取命令 ({prefix}, ChessMove: data=1)")
        self.wait_until_success(chess_key+f"_up")
        self.wait_until_success(chess_key)
        move_msg = Int8()
        move_msg.data = 1
        self.grip_publisher.publish(move_msg)
        self.wait_until_success(chess_key+f"_up")
        self.wait_until_success(grid_index+f"_up")
        self.wait_until_success(grid_index)
        move_msg = Int8()
        move_msg.data = 0
        self.grip_publisher.publish(move_msg)
        self.wait_until_success(grid_index+f"_up")
        self.get_logger().info(f"发布释放命令 ({prefix}, Int8: data=0)")
        self.wait_until_success("0")

    def get_end_effector_pose(self):
        # 查询 l6 相对于 base_link 的变换，等待最长0.5秒
        trans: TransformStamped = self.tf_buffer.lookup_transform(
            self.base_link,
            self.end_effector,
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=0.1)
        )
        pos = [
            trans.transform.translation.x,
            trans.transform.translation.y,
            trans.transform.translation.z
        ]
        return pos


    def pose_reached(self, target_pos, target_quat, current_pos, pos_tol=0.01):
        pos_err = np.linalg.norm(np.array(target_pos) - np.array(current_pos))
        #self.get_logger().info(f"规划成功，开始执行{pos_err}")
        return pos_err < pos_tol

    def send_command(self, command_key):
        if command_key in self.commands:
            target_pos, target_quat = self.commands[command_key]

            self.moveit2.move_to_pose(
                position=target_pos,
                quat_xyzw=target_quat,
                cartesian=False,
            )

            current_pos  = self.get_end_effector_pose()

            if self.pose_reached(target_pos, target_quat, current_pos):
                self.get_logger().info("机械臂末端已到达目标位置")
                return True

            else:
                self.get_logger().warn("机械臂末端未在限定时间内到达目标位置")

        else:
            self.get_logger().warn(f"未知指令: {command_key}")

def main():
    rclpy.init()
    node = ArmMoveWithInput()

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)

    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    node.get_logger().info("请输入数字命令（输入'1'执行移动），输入'q'退出")

    try:
        while rclpy.ok():
            user_input = input("请输入命令: ").strip()
            if user_input == 'q':
                node.get_logger().info("退出程序")
                break
            elif user_input in node.commands:
                node.send_command(user_input)
                #node.move_demo_1()
            else:
                node.get_logger().warn(f"未定义的命令：{user_input}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
    executor_thread.join()

if __name__ == "__main__":
    main()
