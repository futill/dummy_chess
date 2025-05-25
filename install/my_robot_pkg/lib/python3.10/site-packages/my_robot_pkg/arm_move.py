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
from std_msgs.msg import Int8

class ArmMoveWithInput(Node):
    def __init__(self):
        super().__init__("arm_move_with_input")

        self.arm_joint_names = ["j1", "j2", "j3", "j4", "j5", "j6"]
        self.group_name = "dummy"
        self.base_link = "base_link"
        self.l5 = "l5"
        self.end_effector = "l6"

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
        self.black=0

        # TF2监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.subscription = self.create_subscription(Int8,
                                                "/move",
                                                self.move,
                                                1)
        self.grip_publisher = self.create_publisher(Int8, '/grip', 10)                                        

        self.commands = {
            "init": ([0.3272, 0.042798, 0.35154], [0.498732, 0.510627, 0.500207, 0.49223]),
            "0": ([0.26244, 0.043087, 0.23896], [1.0, 0.0, 0.0, 0.0]),
            "4": ([0.36085, 0.0085, 0.02691], [1.0, 0.0, 0.0, 0.0]),
            "5": ([0.36106, 0.0417, 0.02721], [1.0, 0.0, 0.0, 0.0]),
            "6": ([0.36100, 0.07840, 0.02743], [1.0, 0.0, 0.0, 0.0]),
            "7": ([0.3907, 0.011423, 0.033833], [1.0, 0.0, 0.0, 0.0]),
            "8": ([0.3907, 0.04584, 0.033833], [1.0, 0.0, 0.0, 0.0]),
            "9": ([0.3907, 0.078521, 0.033793], [1.0, 0.0, 0.0, 0.0]),
            "1": ([0.32752, 0.0017, 0.02017], [1.0, 0.0, 0.0, 0.0]),
            "2": ([0.327723, 0.0368313, 0.0206634], [1.0, 0.0, 0.0, 0.0]),
            "3": ([0.32748, 0.07406, 0.02017], [1.0, 0.0, 0.0, 0.0]),

            "black_1": ([0.3277, 0.2342, 0.0320], [1.0, 0.0, 0.0, 0.0]),
            "black_2": ([0.3026, 0.2368, 0.0261], [1.0, 0.0, 0.0, 0.0]),
            "black_3": ([0.2731, 0.2363, 0.0245], [1.0, 0.0, 0.0, 0.0]),
            "black_4": ([0.2393, 0.2358, 0.0176], [1.0, 0.0, 0.0, 0.0]),
            "black_5": ([0.2084, 0.2352, 0.0156], [1.0, 0.0, 0.0, 0.0])
        }
        self.grip={            
            "black_1": ([0.3277, 0.2342, 0.0320], [1.0, 0.0, 0.0, 0.0]),
            "black_2": ([0.3026, 0.2368, 0.0261], [1.0, 0.0, 0.0, 0.0]),
            "black_3": ([0.2731, 0.2363, 0.0245], [1.0, 0.0, 0.0, 0.0]),
            "black_4": ([0.2393, 0.2358, 0.0176], [1.0, 0.0, 0.0, 0.0]),
            "black_5": ([0.2084, 0.2352, 0.0156], [1.0, 0.0, 0.0, 0.0])
            }

    def wait_until_success(self, command: str, retry_delay=1):
        """发送命令并等待返回 True"""
        while True:
            success = self.send_command(command)
            if success:
                return
            time.sleep(retry_delay)

    def move_demo_1(self):
        move_msg= Int8()
        move_msg.data=1
        self.grip_publisher.publish(move_msg)
        self.wait_until_success("black_1")
        self.black += 1
        self.wait_until_success("0")
        self.wait_until_success("1")
        move_msg= Int8()
        move_msg.data=0
        self.grip_publisher.publish(move_msg)
        self.wait_until_success("0")
        self.get_logger().info("动作结束")

    
    def move(self,msg):
        mode = msg.data
        self.send_command(mode)

    def get_end_effector_pose(self):
            # 查询 l6 相对于 base_link 的变换，等待最长0.5秒
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                self.base_link,
                self.end_effector,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
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
            self.get_logger().info(f"接收到指令 {command_key}，移动到位置: {target_pos}，姿态: {target_quat}")

            self.moveit2.move_to_pose(
                position=target_pos,
                quat_xyzw=target_quat,
                cartesian=False,
            )


            reached = False
            timeout = 5.0  # 最长等待时间（秒）
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
                #node.send_command(user_input)
                node.move_demo_1()
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
