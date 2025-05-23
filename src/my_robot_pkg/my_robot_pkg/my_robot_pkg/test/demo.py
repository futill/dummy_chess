import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import threading

class JointCommandPublisher(Node):
    def __init__(self):
        super().__init__('joint_command_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/dummy_controller/commands', 10)

        self.commands = {
            '0': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],             # init
            '1': [-0.10472, 0.7854, 0.6109, 0.0, -0.15708, 0.0],
            '2': [0.03491, 0.78540, 0.59341, 0.0, -0.15708, 0.0],
            '3': [0.17453, 0.82030, 0.54105, 0.0, -0.19199, 0.0],
            '4': [-0.08727, 0.89012, 0.27925, -0.03491, -0.38397, 0.0],
            '5': [0.01745, 0.89012, 0.26180, -0.03491, -0.40143, 0.0],
            '6': [0.15708, 0.90757, 0.24435, -0.03491, -0.41888, 0.0],
            '7': [-0.06981, 0.97738, 0.01745, -0.03491, -0.54105, 0.0],
            '8': [0.03491, 0.99484, 0.0, -0.03491, -0.5585, 0.0],
            '9': [0.13963, 1.02974, -0.06981, -0.03491, -0.59341, 0.0]
        }

        threading.Thread(target=self.keyboard_listener, daemon=True).start()

    def keyboard_listener(self):
        while rclpy.ok():
            user_input = input("请输入指令数字(0-9), 输入q退出: ").strip()
            if user_input == 'q':
                self.get_logger().info('退出程序')
                rclpy.shutdown()
                break
            if user_input in self.commands:
                self.send_command(self.commands[user_input])
            else:
                self.get_logger().warn('无效输入，请输入0-9之间的数字或q退出')

    def send_command(self, joint_angles):
        msg = Float64MultiArray()
        msg.data = joint_angles
        self.publisher_.publish(msg)
        self.get_logger().info(f'发送关节目标: {msg.data}')

def main():
    rclpy.init()
    node = JointCommandPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
