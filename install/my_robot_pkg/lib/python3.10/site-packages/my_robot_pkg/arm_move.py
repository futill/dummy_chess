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
from std_msgs.msg import Int8,Bool
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
                                Bool,
                                '/start_exec',
                                self.start_exec_callback,
                                1
                            )

        self.grip_publisher = self.create_publisher(Int8, '/grip', 10)      
        self.action_thread = Thread(target=self.process_action_queue, daemon=True)
        self.action_thread.start()                                  

        self.commands = {
            "init": ([0.3272, 0.042798, 0.35154], [0.498732, 0.510627, 0.500207, 0.49223]),
            "0": ([0.35244, 0.043087, 0.23896], [0.94828, 0.001971, 0.31729, -0.00894]),
            "4": ([0.3500, 0.0085, 0.02691], [1.0, 0.0, 0.0, 0.0]),
            "5": ([0.3500, 0.0417, 0.02721], [1.0, 0.0, 0.0, 0.0]),
            "6": ([0.3500, 0.07840, 0.02743], [1.0, 0.0, 0.0, 0.0]),
            "7": ([0.3807, 0.011423, 0.033833], [1.0, 0.0, 0.0, 0.0]),
            "8": ([0.3807, 0.04584, 0.033833], [1.0, 0.0, 0.0, 0.0]),
            "9": ([0.3807, 0.078521, 0.033793], [1.0, 0.0, 0.0, 0.0]),
            "1": ([0.32752, 0.0017, 0.02017], [1.0, 0.0, 0.0, 0.0]),
            "2": ([0.327723, 0.0368313, 0.0206634], [1.0, 0.0, 0.0, 0.0]),
            "3": ([0.32748, 0.07406, 0.02017], [1.0, 0.0, 0.0, 0.0]),

            "white_1": ([0.32754, -0.16553, 0.04679], [1.0, 0.0, 0.0, 0.0]),
            "white_2": ([0.29988, -0.17098, 0.0409434], [1.0, 0.0, 0.0, 0.0]),
            "white_3": ([0.26843, -0.17164, 0.0331875], [1.0, 0.0, 0.0, 0.0]),
            "white_4": ([0.23621, -0.17233, 0.029472], [1.0, 0.0, 0.0, 0.0]),
            "white_5": ([0.20261, -0.17817, 0.024545], [1.0, 0.0, 0.0, 0.0]),

            "black_1": ([0.3277, 0.2342, 0.0320], [1.0, 0.0, 0.0, 0.0]),
            "black_2": ([0.3026, 0.2368, 0.0261], [1.0, 0.0, 0.0, 0.0]),
            "black_3": ([0.2731, 0.2363, 0.0245], [1.0, 0.0, 0.0, 0.0]),
            "black_4": ([0.2393, 0.2358, 0.0176], [1.0, 0.0, 0.0, 0.0]),
            "black_5": ([0.2084, 0.2352, 0.0156], [1.0, 0.0, 0.0, 0.0])
        }

    def start_exec_callback(self, msg: Bool):
        """接收启动指令，允许执行动作队列"""
        if msg.data:
            with self.queue_lock:
                self.ready_to_process = True
            self.get_logger().info("收到开始执行信号，开始处理动作队列")

    def wait_until_success(self, command: str, retry_delay=0.2):
        """发送命令并等待返回 True"""
        while True:
            success = self.send_command(command)
            if success:
                return
            time.sleep(retry_delay)

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
                elif action[0] == 'start_match':
                    self.start_match_action()

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
        self.wait_until_success(chess_key)
        move_msg = Int8()
        move_msg.data = 1
        self.grip_publisher.publish(move_msg)
        self.wait_until_success("0")
        self.wait_until_success(grid_index)
        move_msg = Int8()
        move_msg.data = 0
        self.grip_publisher.publish(move_msg)
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


            reached = False
            timeout = 3.0  # 最长等待时间（秒）
            start_time = time.time()

            while time.time() - start_time < timeout:
                current_pos  = self.get_end_effector_pose()
                if current_pos is None:
                    continue

                if self.pose_reached(target_pos, target_quat, current_pos):
                    reached = True
                    break
                #self.get_logger().info(f"规划成功，开始执行{current_pos}")


            if reached:
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
