import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import queue
import time
import math
from my_robot_pkg_msg.msg import ChessMove
from std_msgs.msg import Int8

class ChessboardDetectorNode(Node):
    def __init__(self):
        super().__init__('chessboard_detector')
        image_topic = '/camera/color/image_raw'
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            qos_profile_sensor_data
        )
        self.move_publisher = self.create_publisher(ChessMove, '/chess_state', 10)
        self.get_logger().info(f"已订阅图像话题：{image_topic}")

        self.image_queue = queue.Queue(maxsize=10)
        self.lock = threading.Lock()
        self.shutdown_event = threading.Event()

        self.rect_filters = {}
        self.circle_filters = {} 
        self.tracked_circles = []  # 每项是 dict: {'id': int, 'kf': KalmanFilter, 'last_seen': frame_count, 'pos': (x, y)}
        self.next_circle_id = 0
        self.frame_count = 0
        self.processing_thread = threading.Thread(target=self.process_images)
        self.processing_thread.daemon = True
        self.processing_thread.start()
        self.get_logger().info("图像处理线程已启动")
        

    def init_kalman_filter(self):
        kf = cv2.KalmanFilter(4, 2)
        kf.transitionMatrix = np.array([
            [1, 0, 1, 0],
            [0, 1, 0, 1],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ], dtype=np.float32)
        kf.measurementMatrix = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ], dtype=np.float32)
        kf.processNoiseCov = np.eye(4, dtype=np.float32) * 0.01
        kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 1.0
        kf.errorCovPost = np.eye(4, dtype=np.float32)
        kf.statePost = np.zeros((4, 1), dtype=np.float32)
        return kf

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if frame is None or frame.size == 0:
            self.get_logger().error("接收到无效图像")
            return
        self.image_queue.put_nowait((frame, time.time()))


    def process_images(self):
        while not self.shutdown_event.is_set():
            try:
                frame, timestamp = self.image_queue.get(timeout=0.1)
                self.process_single_image(frame)
                self.image_queue.task_done()
            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f"处理图像时发生错误：{e}")

    def calculate_rotation_angle(self, rect):
        p1, p2 = rect[0], rect[1]
        delta_y = p2[1] - p1[1]
        delta_x = p2[0] - p1[0]
        angle_rad = math.atan2(delta_y, delta_x)
        angle_deg = math.degrees(angle_rad) % 360
        angle_deg = (angle_deg + 90) % 360
        return angle_deg

    def process_single_image(self, frame):
        h, w = frame.shape[:2]
        crop_w = int(w * 0.6)
        crop_h = int(h * 0.6)
        x0 = max(0, w // 2 - crop_w // 2)
        y0 = max(0, h // 2 - crop_h // 2)
        if crop_w < 100 or crop_h < 100 or x0 + crop_w > w or y0 + crop_h > h:
            self.get_logger().warn("无效裁剪区域")
            return

        frame_crop = frame[y0:y0 + crop_h, x0:x0 + crop_w]
        # 假设 frame_crop 已经是裁剪过的图像
        hsv = cv2.cvtColor(frame_crop, cv2.COLOR_BGR2HSV)

        # 设定黑色的 HSV 范围（可以根据实际图像微调）
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 100])  # V 值较低是黑色

        # 创建掩码，提取黑色区域
        mask_black = cv2.inRange(hsv, lower_black, upper_black)

        # 可选：做开操作去除小噪声
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        mask_clean = cv2.morphologyEx(mask_black, cv2.MORPH_OPEN, kernel)

        # 用掩码提取黑色前景
        foreground = cv2.bitwise_and(frame_crop, frame_crop, mask=mask_clean)

        # 转为灰度进行后续处理
        gray = cv2.cvtColor(foreground, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)

        # 膨胀与腐蚀
        dilated = cv2.dilate(edges, kernel, iterations=1)
        eroded = cv2.erode(dilated, kernel, iterations=1)

        contours, _ = cv2.findContours(eroded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        board_rect = self.find_largest_rectangle(contours)

        if board_rect is not None:
            color = (0, 255, 0)
        else:
            # 如果没有检测到棋盘，但已有卡尔曼滤波器，使用预测框
            if len(self.rect_filters) == 4:
                predicted_rect = np.zeros((4, 2), dtype=np.int32)
                for i in range(4):
                    kf = self.rect_filters[i]
                    predicted = kf.predict()
                    predicted_rect[i] = [int(predicted[0]), int(predicted[1])]
                board_rect = predicted_rect
                color = (0, 255, 255)  # 黄色表示使用的是预测框
                #self.get_logger().warn("未检测到棋盘，使用卡尔曼预测框")
            else:
                #self.get_logger().warn("未检测到棋盘，也无预测框可用")
                return


        cv2.polylines(frame_crop, [board_rect], isClosed=True, color=color, thickness=3)
        angle = self.calculate_rotation_angle(board_rect)
        text = f"Angle: {angle:.2f} deg"
        cv2.putText(frame_crop, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

        # 透视变换得到正方形棋盘图像
        width = max(np.linalg.norm(board_rect[0] - board_rect[1]),
                    np.linalg.norm(board_rect[2] - board_rect[3]))
        height = max(np.linalg.norm(board_rect[0] - board_rect[3]),
                     np.linalg.norm(board_rect[1] - board_rect[2]))
        size = int(max(width, height))
        dst = np.array([
            [0, 0], [size - 1, 0], [size - 1, size - 1], [0, size - 1]
        ], dtype="float32")
        M = cv2.getPerspectiveTransform(board_rect.astype(np.float32), dst)
        warp = cv2.warpPerspective(frame_crop, M, (size, size))

        self.detect_and_publish_chess_pieces(warp, board_rect)

        with self.lock:
            #cv2.imshow("Chessboard Detection", frame_crop)
            cv2.imshow("Warped Board", warp)
            cv2.waitKey(1)

    def find_largest_rectangle(self, contours):
        max_area = 0
        best_rect = None
        min_area_threshold = 0.1 * 640 * 480
        aspect_ratio_range = (0.8, 1.2)
        angle_tolerance = 10.0

        for cnt in contours:
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.01 * peri, True)
            if len(approx) != 4 or not cv2.isContourConvex(approx):
                continue

            area = cv2.contourArea(approx)
            if area < min_area_threshold:
                continue

            rect = approx.reshape(4, 2)
            rect = self.order_points(rect)
            width = max(np.linalg.norm(rect[0] - rect[1]), np.linalg.norm(rect[2] - rect[3]))
            height = max(np.linalg.norm(rect[0] - rect[3]), np.linalg.norm(rect[1] - rect[2]))
            aspect_ratio = width / height if height != 0 else float('inf')
            if not (aspect_ratio_range[0] <= aspect_ratio <= aspect_ratio_range[1]):
                continue

            angles = []
            for i in range(4):
                p1, p2, p3 = rect[i], rect[(i + 1) % 4], rect[(i + 2) % 4]
                v1 = p2 - p1
                v2 = p3 - p2
                cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-6)
                angle = np.degrees(np.arccos(np.clip(cos_angle, -1.0, 1.0)))
                angles.append(angle)
            if not all(abs(a - 90) < angle_tolerance for a in angles):
                continue

            if area > max_area:
                max_area = area
                best_rect = rect

        if best_rect is not None:
            best_rect = self.apply_kalman_to_rectangle(best_rect)
        return best_rect

    def apply_kalman_to_rectangle(self, rect):
        new_filters = {}
        smoothed_rect = np.zeros((4, 2), dtype=np.int32)

        for i in range(4):
            x, y = rect[i]
            if i not in self.rect_filters:
                self.rect_filters[i] = self.init_kalman_filter()
            kf = self.rect_filters[i]

            predicted = kf.predict()
            measurement = np.array([[float(x)], [float(y)]], dtype=np.float32)
            corrected = kf.correct(measurement)
            smoothed_rect[i] = [int(corrected[0]), int(corrected[1])]
            new_filters[i] = kf

        self.rect_filters = new_filters
        return smoothed_rect

    def order_points(self, pts):
        rect = np.zeros((4, 2), dtype="float32")
        s = pts.sum(axis=1)
        diff = np.diff(pts, axis=1)
        rect[0] = pts[np.argmin(s)]  # top-left
        rect[2] = pts[np.argmax(s)]  # bottom-right
        rect[1] = pts[np.argmin(diff)]  # top-right
        rect[3] = pts[np.argmax(diff)]  # bottom-left
        return rect

    def detect_circles(self, warp, cell_size):
        self.frame_count += 1
        gray = cv2.cvtColor(warp, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        circles = cv2.HoughCircles(edges, cv2.HOUGH_GRADIENT, 1, cell_size//4,
                                    param1=100, param2=30,
                                    minRadius=cell_size//6, maxRadius=cell_size//2)
        detected = np.uint16(np.around(circles[0])) if circles is not None else []

        matched_ids = set()
        for (x, y, r) in detected:
            best_match = None
            best_dist = float('inf')
            for obj in self.tracked_circles:
                px, py = obj['pos']
                dist = np.hypot(x - px, y - py)
                if dist < cell_size // 2 and obj['id'] not in matched_ids:
                    if dist < best_dist:
                        best_match = obj
                        best_dist = dist

            if best_match:
                kf = best_match['kf']
                kf.predict()
                corrected = kf.correct(np.array([[np.float32(x)], [np.float32(y)]]))
                x_f, y_f = int(corrected[0]), int(corrected[1])
                best_match['pos'] = (x_f, y_f)
                best_match['last_seen'] = self.frame_count
                matched_ids.add(best_match['id'])
            else:
                # 新圆，分配一个滤波器
                kf = self.init_kalman_filter()
                kf.statePost = np.array([[x], [y], [0], [0]], dtype=np.float32)
                self.tracked_circles.append({
                    'id': self.next_circle_id,
                    'kf': kf,
                    'pos': (x, y),
                    'last_seen': self.frame_count
                })
                self.next_circle_id += 1

        # 清除长期未更新的圆
        self.tracked_circles = [obj for obj in self.tracked_circles if self.frame_count - obj['last_seen'] < 5]

        return [(obj['pos'][0], obj['pos'][1], cell_size // 4) for obj in self.tracked_circles]




    def get_grid_index(self, cx, cy, cell_size):
        col = cx // cell_size
        row = cy // cell_size
        if 0 <= col < 3 and 0 <= row < 3:
            return int(row * 3 + col + 1)  # 返回格子编号1~9
        return None

    def detect_and_publish_chess_pieces(self, warp, board_rect):
        cell_size = warp.shape[0] // 3
        circles = self.detect_circles(warp, cell_size)


        # 用于标记哪些格子被占用
        occupied_grids = set()

        for (cx, cy, r) in circles:
            grid_index = self.get_grid_index(cx, cy, cell_size)
            if grid_index is None:
                continue
            occupied_grids.add(grid_index)

            mask = np.zeros((warp.shape[0], warp.shape[1]), dtype=np.uint8)
            cv2.circle(mask, (cx, cy), r, 255, -1)
            mean_val = cv2.mean(warp, mask=mask)[0:3]
            brightness = sum(mean_val) / 3
            if brightness < 120:
                color = "black" 
            elif brightness > 150:
                color = "white"
            else:
                color = " "

            cv2.circle(warp, (cx, cy), r, (0, 255, 0), 2)
            cv2.putText(warp, color, (cx - 20, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

            move_msg = ChessMove()
            move_msg.grid_index = grid_index
            move_msg.color = color
            self.move_publisher.publish(move_msg)

        # 对未被占用的格子发布“空”
        for idx in range(1, 10):
            if idx not in occupied_grids:
                move_msg = ChessMove()
                move_msg.grid_index = idx
                move_msg.color = " "
                self.move_publisher.publish(move_msg)


    def destroy_node(self):
        self.shutdown_event.set()
        self.processing_thread.join()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ChessboardDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('节点关闭')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
