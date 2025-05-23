from PySide6.QtWidgets import (
    QApplication, QWidget, QPushButton, QGridLayout, QLabel, QHBoxLayout, QVBoxLayout
)
from PySide6.QtGui import QFont
from PySide6.QtCore import QTimer
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from my_robot_pkg_msg.msg import ChessMove
from threading import Thread


class ChessGUI(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle("井字棋 ROS2 控制界面")
        self.grid = [QLabel("") for _ in range(9)]           # ROS接收棋盘
        self.input_grid = [QLabel("") for _ in range(9)]     # 可点击交互棋盘
        self.input_states = [None for _ in range(9)]         # 状态：black/white/None
        self.current_color = "black"
        self.init_ui()

    def init_ui(self):
        main_layout = QHBoxLayout()

        # ===== 左侧 ROS 显示棋盘 =====
        ros_layout = QGridLayout()
        ros_label = QLabel("ROS 棋盘")
        ros_label.setFont(QFont("Arial", 14))
        ros_layout.addWidget(ros_label, 0, 0, 1, 3)

        for i in range(9):
            label = self.grid[i]
            label.setFont(QFont("Arial", 20))
            label.setFixedSize(100, 100)
            label.setStyleSheet("border: 1px solid black;")
            # 默认数字，字体颜色灰，居中，棋子覆盖后数字被替换
            label.setText(f"<div align='center' style='color:gray;'>{i + 1}</div>")
            ros_layout.addWidget(label, 1 + i // 3, i % 3)

        main_layout.addLayout(ros_layout)

        # ===== 右侧 可点击棋盘与控制 =====
        right_layout = QVBoxLayout()

        input_label = QLabel("输入棋盘")
        input_label.setFont(QFont("Arial", 14))
        right_layout.addWidget(input_label)

        input_grid_layout = QGridLayout()
        for i in range(9):
            label = self.input_grid[i]
            label.setFont(QFont("Arial", 20))
            label.setFixedSize(100, 100)
            label.setStyleSheet("border: 1px solid gray;")
            label.setText(f"<div align='center' style='color:gray;'>{i + 1}</div>")
            label.mousePressEvent = lambda event, idx=i: self.handle_click(idx)
            input_grid_layout.addWidget(label, i // 3, i % 3)

        right_layout.addLayout(input_grid_layout)

        # 控制按钮布局
        control_layout = QGridLayout()

        # 切换颜色
        self.color_btn = QPushButton("当前颜色：⚫（点我切换）")
        self.color_btn.clicked.connect(self.toggle_color)
        control_layout.addWidget(self.color_btn, 0, 0)

        # 确认按钮
        confirm_btn = QPushButton("确认发送输入")
        confirm_btn.clicked.connect(self.send_input_moves)
        control_layout.addWidget(confirm_btn, 0, 1)

        # 清空按钮
        clear_btn = QPushButton("清空输入棋盘")
        clear_btn.clicked.connect(self.clear_input_grid)
        control_layout.addWidget(clear_btn, 0, 2)

        # 任务按钮
        task_btns = [
            ("任务 1", self.task1),
            ("任务 2", self.task2),
            ("任务 3", self.task3),
            ("任务 4", self.task4),
        ]
        for i, (text, func) in enumerate(task_btns):
            btn = QPushButton(text)
            btn.clicked.connect(func)
            control_layout.addWidget(btn, 1 + i // 2, i % 2)

        right_layout.addLayout(control_layout)

        main_layout.addLayout(right_layout)
        self.setLayout(main_layout)

    def toggle_color(self):
        self.current_color = "white" if self.current_color == "black" else "black"
        icon = "⚪" if self.current_color == "white" else "⚫"
        self.color_btn.setText(f"当前颜色：{icon}（点我切换）")

    def handle_click(self, idx):
        self.input_states[idx] = self.current_color
        icon = "⚪" if self.current_color == "white" else "⚫"
        self.input_grid[idx].setText(icon)

    def send_input_moves(self):
        for idx, color in enumerate(self.input_states):
            if color:
                msg = ChessMove()
                msg.grid_index = idx
                msg.color = color
                self.ros_node.publisher.publish(msg)

    def clear_input_grid(self):
        for i in range(9):
            self.input_states[i] = None
            self.input_grid[i].setText(f"<div align='center' style='color:gray;'>{i + 1}</div>")

    def task1(self):
        msg = ChessMove()
        msg.grid_index = 4
        msg.color = "black"
        self.ros_node.publisher.publish(msg)

    def task2(self):
        for i, color in zip([0, 1, 2, 3], ["black", "black", "white", "white"]):
            msg = ChessMove()
            msg.grid_index = i
            msg.color = color
            self.ros_node.publisher.publish(msg)

    def task3(self):
        for i, color in zip([0, 2, 6, 8], ["black", "black", "white", "white"]):
            msg = ChessMove()
            msg.grid_index = i
            msg.color = color
            self.ros_node.publisher.publish(msg)

    def task4(self):
        msg = String()
        msg.data = "start_match"
        self.ros_node.match_start_pub.publish(msg)

    def update_grid(self, index, color):
        if color == "black":
            self.grid[index].setText("⚫")
        elif color == "white":
            self.grid[index].setText("⚪")
        else:
            # 恢复数字显示
            self.grid[index].setText(f"<div align='center' style='color:gray;'>{index + 1}</div>")


class GuiNode(Node):
    def __init__(self, gui):
        super().__init__('qt_gui_node')
        self.gui = gui
        self.publisher = self.create_publisher(ChessMove, '/chess_move', 10)
        self.match_start_pub = self.create_publisher(String, '/start_match', 10)
        self.subscription = self.create_subscription(
            ChessMove,
            'chess_state',
            self.chess_state_callback,
            10
        )

    def chess_state_callback(self, msg):
        self.gui.update_grid(msg.grid_index, msg.color)


def main():
    rclpy.init()
    app = QApplication([])
    gui = ChessGUI(None)
    ros_node = GuiNode(gui)
    gui.ros_node = ros_node

    timer = QTimer()
    timer.timeout.connect(lambda: None)
    timer.start(100)

    rclpy_thread = Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    rclpy_thread.start()

    gui.show()
    app.exec()

    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    from threading import Thread
    main()
