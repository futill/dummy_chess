from PySide6.QtWidgets import (
    QApplication, QWidget, QPushButton, QGridLayout, QLabel, QHBoxLayout, QVBoxLayout
)
from PySide6.QtGui import QFont
from PySide6.QtCore import QTimer, QMetaObject, Qt, Q_ARG, Slot
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool,Int32MultiArray
from my_robot_pkg_msg.msg import ChessMove
from threading import Thread
import time

class ChessGUI(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle("井字棋 ROS2 控制界面")
        self.grid = [QLabel("") for _ in range(9)]
        self.input_grid = [QLabel("") for _ in range(9)]
        self.input_states = [None for _ in range(9)]
        self.board_state = [None for _ in range(9)]
        self.current_color = "black"
        self.previous_ros_state = [None] * 9
        self.state_buffer = [None] * 9  # 状态缓冲区
        self.last_update_time = [0.0] * 9  # 每个格子的最后更新时间
        self.init_ui()
        self.is_first_player = False

    def init_ui(self):
        main_layout = QHBoxLayout()

        ros_layout = QGridLayout()
        ros_label = QLabel("ROS 棋盘")
        ros_label.setFont(QFont("Arial", 14))
        ros_layout.addWidget(ros_label, 0, 0, 1, 3)

        for i in range(9):
            label = self.grid[i]
            label.setFont(QFont("Arial", 20))
            label.setFixedSize(100, 100)
            label.setStyleSheet("border: 1px solid black;")
            label.setText(f"<div align='center' style='color:gray;'>{i + 1}</div>")
            ros_layout.addWidget(label, 1 + i // 3, i % 3)

        main_layout.addLayout(ros_layout)

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

        control_layout = QGridLayout()

        self.color_btn = QPushButton("当前颜色：⚫（点我切换）")
        self.color_btn.clicked.connect(self.toggle_color)
        control_layout.addWidget(self.color_btn, 0, 0)

        clear_btn = QPushButton("清空输入棋盘")
        clear_btn.clicked.connect(self.clear_input_grid)
        control_layout.addWidget(clear_btn, 0, 2)

        task_btns = [
            ("任务 1", self.task1),
            ("任务 2", self.task2),
            ("任务 3", self.task3),
            ("任务 4", self.task4),
            ("任务 5", self.task5),
            ("AI 下子", self.ai_turn),
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
        self.board_state[idx] = self.current_color
        icon = "⚪" if self.current_color == "white" else "⚫"
        self.input_grid[idx].setText(icon)

    def clear_input_grid(self):
        for i in range(9):
            self.input_states[i] = None
            self.board_state[i] = None
            self.input_grid[i].setText(f"<div align='center' style='color:gray;'>{i + 1}</div>")

    def task1(self):
        msg = ChessMove()
        msg.grid_index = 4
        msg.color = "black"
        self.ros_node.publisher.publish(msg)

    def task2(self):
        moves = [(i, color) for i, color in enumerate(self.input_states) if color is not None]
        if not moves:
            print("输入棋盘为空，请先在输入棋盘上放置棋子！")
            return
        for index, color in moves:
            msg = ChessMove()
            msg.grid_index = index
            msg.color = color
            self.ros_node.publisher.publish(msg)
            self.ros_node.get_logger().info(f"任务 2: 发布 ChessMove (grid_index={index}, color={color})")
            time.sleep(0.5)
        msg = Bool()
        msg.data = True
        self.ros_node.start_exec.publish(msg)

    def task3(self):
        self.task2()

    def task4(self):
        self.is_first_player = True
        moves = [(i, color) for i, color in enumerate(self.input_states) if color is not None]
        if not moves:
            print("输入棋盘为空，请先在输入棋盘上放置棋子！")
            return
        for index, color in moves:
            msg = ChessMove()
            msg.grid_index = index
            msg.color = color
            self.ros_node.publisher.publish(msg)
            self.ros_node.get_logger().info(f"任务 4: 发布 ChessMove (grid_index={index}, color={color})")
            time.sleep(0.5)
        msg = Bool()
        msg.data = True
        self.ros_node.start_exec.publish(msg)
        print("AI 执黑棋已落子中心，请人类通过真实棋盘落白棋。")

    def task5(self):
        self.is_first_player = False
        print("请人工落子（执黑），然后点击“AI 下子”")

    def ai_turn(self):
        msg = ChessMove()
        current_state = self.state_buffer.copy()
        print(f"AI 检查 - 当前状态: {current_state}")
        print(f"AI 检查 - 之前状态: {self.previous_ros_state}")
                # 检测是否有棋子被人为移动位置

        for color in ("white", "black"):
            removed = [
                i for i in range(9) if self.previous_ros_state[i] == color and current_state[i] is None
            ]
            added = [
                i for i in range(9) if self.previous_ros_state[i] is None and current_state[i] == color
            ]

            if len(removed) == 1 and len(added) == 1:
                print(f"⚠️ 检测到 {color} 棋子从位置 {removed[0]+1} 移动到位置 {added[0]+1}，将自动纠正。")

                # 发布纠正位置信息到 correction_move topic
                correction_msg = Int32MultiArray()
                correction_msg.data = [removed[0], added[0]]
                self.ros_node.correction_pub.publish(correction_msg)
                time.sleep(0.3)

                # 更新 GUI 显示
                self.grid[removed[0]].setText("⚫" if color == "black" else "⚪")
                self.state_buffer[removed[0]] = color
                self.previous_ros_state[removed[0]] = color

                self.grid[added[0]].setText(f"<div align='center' style='color:gray;'>{added[0] + 1}</div>")
                self.state_buffer[added[0]] = None
                self.previous_ros_state[added[0]] = None

                return 


        # 检测新白棋（人类落子）
        if self.is_first_player:
            new_white_positions = [
                i for i in range(9) if self.previous_ros_state[i] != "white" and current_state[i] == "white"
            ]

            if len(new_white_positions) != 1:
                print(f"⚠️ 检测到 {len(new_white_positions)} 个新白棋，请确保仅落一子！")
                if len(new_white_positions) == 0:
                    print("建议：检查 /chess_state 话题是否发布白棋消息，或确保 ROS 棋盘已更新。")
                return
            new_white = new_white_positions[0]
            print(f"检测到人类落白棋：位置 {new_white + 1}")
        else:
            new_black_positions = [
                i for i in range(9) if self.previous_ros_state[i] != "black" and current_state[i] == "black"
            ]

            if len(new_black_positions) != 1:
                print(f"⚠️ 检测到 {len(new_black_positions)} 个新黑棋，请确保仅落一子！")
                if len(new_black_positions) == 0:
                    print("建议：检查 /chess_state 话题是否发布白棋消息，或确保 ROS 棋盘已更新。")
                return
            new_black = new_black_positions[0]
            print(f"检测到人类落黑棋：位置 {new_black + 1}")


        # 更新状态记录
        self.previous_ros_state = current_state.copy()

        # AI落黑棋策略
        if self.is_first_player:
            # 先手策略：先查找必胜落子
            move = self.find_winning_move(current_state, "black")
            if move is None:
                # 没必胜就防守阻止对方获胜
                move = self.find_blocking_move(current_state, "white")
            if move is None:
                # 仍没找到，选择最优落子
                move = self.find_best_ai_move(current_state)
            msg.color = "black"
        else:
            # 后手策略：主要防守，阻止对方获胜
            move = self.find_blocking_move(current_state, "white")
            if move is None:
                move = self.find_best_ai_move(current_state)
            msg.color = "white"
        if move is None:
            print("没有可落子的空位了")
            return

        # 更新 GUI 和状态
        if self.is_first_player:
            self.grid[move].setText("⚫")
            self.state_buffer[move] = "black"
            self.previous_ros_state[move] = "black"
        else:
            self.grid[move].setText("⚪")
            self.state_buffer[move] = "white"
            self.previous_ros_state[move] = "white"

        msg.grid_index = move
        self.ros_node.publisher.publish(msg)
        time.sleep(0.5)
        msg_bool = Bool()
        msg_bool.data = True
        self.ros_node.start_exec.publish(msg_bool)
        self.ros_node.get_logger().info(f"AI 响应落黑棋，位置 {move + 1}")


    def find_winning_move(self, state, color):
        """查找是否有一步能直接获胜"""
        for i in range(9):
            if state[i] is None:
                state[i] = color
                if self.check_win(state, color):
                    state[i] = None
                    return i
                state[i] = None
        return None

    def find_blocking_move(self, state, color):
        """查找阻止对方获胜的落子"""
        opponent_color = "black" if color == "white" else "white"
        for i in range(9):
            if state[i] is None:
                state[i] = opponent_color
                if self.check_win(state, opponent_color):
                    state[i] = None
                    return i  # 阻止对方获胜
                state[i] = None
        return None

    def find_best_ai_move(self, state):
        best_score = -float('inf')
        best_move = None
        for i in range(9):
            if state[i] is None:
                state[i] = "black"  # AI执黑
                score = self.minimax(state, False)
                state[i] = None
                if score > best_score:
                    best_score = score
                    best_move = i
        return best_move

    def minimax(self, state, is_maximizing):
        if self.check_win(state, "black"):
            return 1  # AI赢了
        elif self.check_win(state, "white"):
            return -1  # 人类赢了
        elif all(pos is not None for pos in state):
            return 0  # 平局

        if is_maximizing:
            best_score = -float('inf')
            for i in range(9):
                if state[i] is None:
                    state[i] = "black"
                    score = self.minimax(state, False)
                    state[i] = None
                    best_score = max(score, best_score)
            return best_score
        else:
            best_score = float('inf')
            for i in range(9):
                if state[i] is None:
                    state[i] = "white"
                    score = self.minimax(state, True)
                    state[i] = None
                    best_score = min(score, best_score)
            return best_score
    def check_win(self, state, color):
        """判断指定颜色是否获胜"""
        win_positions = [
            [0,1,2], [3,4,5], [6,7,8],  # 横向三行
            [0,3,6], [1,4,7], [2,5,8],  # 纵向三列
            [0,4,8], [2,4,6]            # 两条斜线
        ]
        for line in win_positions:
            if all(state[pos] == color for pos in line):
                return True
        return False

    @Slot(int, str)
    def update_grid_safe(self, index, color):
        self.update_grid(index - 1, color)

    def update_grid(self, index, color):
        if 0 <= index < 9:
            current_time = time.time()
            # 防闪烁：忽略 100ms 内的重复更新
            if current_time - self.last_update_time[index] < 0.1:
                return
            self.last_update_time[index] = current_time

            if color == "black":
                self.grid[index].setText("⚫")
                self.state_buffer[index] = "black"
            elif color == "white":
                self.grid[index].setText("⚪")
                self.state_buffer[index] = "white"
            else:  # 处理 color: ' '，重置格子
                self.grid[index].setText(f"<div align='center' style='color:gray;'>{index + 1}</div>")
                self.state_buffer[index] = None

class GuiNode(Node):
    def __init__(self, gui):
        super().__init__('qt_gui_node')
        self.gui = gui
        self.publisher = self.create_publisher(ChessMove, '/move', 10)
        self.start_exec = self.create_publisher(Bool, '/start_exec', 10)
        self.correction_pub = self.create_publisher(Int32MultiArray, '/correction_move', 10)
        self.subscription = self.create_subscription(
            ChessMove,
            '/chess_state',
            self.chess_state_callback,
            10
        )
        

    def chess_state_callback(self, msg):
        # 接受所有颜色，包括 ' '
        print(f"收到 ChessMove: grid_index={msg.grid_index}, color={msg.color}")
        QMetaObject.invokeMethod(
            self.gui,
            "update_grid_safe",
            Qt.QueuedConnection,
            Q_ARG(int, msg.grid_index),
            Q_ARG(str, msg.color)
        )

def main():
    rclpy.init()
    app = QApplication([])
    gui = ChessGUI(None)
    ros_node = GuiNode(gui)
    gui.ros_node = ros_node

    # 优化 QTimer，定期处理 ROS 消息
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0.01))
    timer.start(10)  # 每 10ms 检查一次

    rclpy_thread = Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    rclpy_thread.start()

    gui.show()
    app.exec()

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()