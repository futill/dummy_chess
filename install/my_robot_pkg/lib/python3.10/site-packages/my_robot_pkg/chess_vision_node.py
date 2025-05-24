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

MAX_MISSED_FRAMES = 5  # 最多容忍的未检测帧数

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
        self.get_logger().info(f"已订阅图像话题：{image_topic}")

        self.image_queue = queue.Queue(maxsize=10)
        self.lock = threading.Lock()
        self.shutdown_event = threading.Event()

        self.rect_filters = {}
        self.missed_frames = 0

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
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"图像转换错误：{e}")
            return
        if frame is None or frame.size == 0:
            self.get_logger().error("接收到无效图像")
            return
        try:
            self.image_queue.put_nowait((frame, time.time()))
        except queue.Full:
            self.get_logger().warn("图像队列已满，丢弃新图像")

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
        crop_w = int(w * 0.4)
        crop_h = int(h * 0.4)
        x0 = max(0, w // 2 - crop_w // 2)
        y0 = max(0, h // 2 - crop_h // 2)
        if crop_w < 100 or crop_h < 100 or x0 + crop_w > w or y0 + crop_h > h:
            self.get_logger().warn("无效裁剪区域")
            return

        frame = frame[y0:y0 + crop_h, x0:x0 + crop_w]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)

        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        board_rect = self.find_largest_rectangle(contours)

        if board_rect is not None:
            self.missed_frames = 0
            color = (0, 255, 0)
        else:
            board_rect = self.predict_next_rectangle()
            if board_rect is not None:
                color = (0, 255, 255)
            else:
                self.missed_frames = MAX_MISSED_FRAMES
                self.get_logger().warn("无法预测矩形")
                return
            self.missed_frames += 1
            if self.missed_frames >= MAX_MISSED_FRAMES:
                self.rect_filters.clear()
                self.get_logger().info("连续丢失太多帧，重置滤波器")
                return

        cv2.polylines(frame, [board_rect], isClosed=True, color=color, thickness=3)
        self.draw_grid_numbers(frame, board_rect)
        angle = self.calculate_rotation_angle(board_rect)
        text = f"Angle: {angle:.2f} deg"
        cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

        with self.lock:
            cv2.imshow("Chessboard Detection", frame)
            cv2.imshow("Edges", edges)
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
                p1, p2, p3 = rect[i], rect[(i+1)%4], rect[(i+2)%4]
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

    def predict_next_rectangle(self):
        if len(self.rect_filters) != 4:
            return None
        predicted_rect = np.zeros((4, 2), dtype=np.int32)
        for i in range(4):
            kf = self.rect_filters.get(i)
            if kf is None:
                return None
            pred = kf.predict()
            predicted_rect[i] = [int(pred[0]), int(pred[1])]
        return predicted_rect

    def order_points(self, pts):
        rect = np.zeros((4, 2), dtype="float32")
        s = pts.sum(axis=1)
        rect[0] = pts[np.argmin(s)]
        rect[2] = pts[np.argmax(s)]
        diff = np.diff(pts, axis=1)
        rect[1] = pts[np.argmin(diff)]
        rect[3] = pts[np.argmax(diff)]
        return rect.astype(np.int32)

    def draw_grid_numbers(self, image, board_rect):
        width = max(np.linalg.norm(board_rect[0] - board_rect[1]),
                    np.linalg.norm(board_rect[2] - board_rect[3]))
        height = max(np.linalg.norm(board_rect[0] - board_rect[3]),
                     np.linalg.norm(board_rect[1] - board_rect[2]))
        size = int(max(width, height))
        dst = np.array([
            [0, 0], [size-1, 0], [size-1, size-1], [0, size-1]
        ], dtype="float32")
        M = cv2.getPerspectiveTransform(board_rect.astype(np.float32), dst)
        warp = cv2.warpPerspective(image, M, (size, size))
        cell_size = size // 3

        for i in range(3):
            for j in range(3):
                x, y = j * cell_size, i * cell_size
                cell = warp[y:y+cell_size, x:x+cell_size]
                gray_cell = cv2.cvtColor(cell, cv2.COLOR_BGR2GRAY)
                blur = cv2.GaussianBlur(gray_cell, (5, 5), 0)
                _, thresh = cv2.threshold(blur, 50, 255, cv2.THRESH_BINARY_INV)
                white_pixels = cv2.countNonZero(thresh)
                area = cell_size * cell_size
                if white_pixels / area > 0.05:
                    number = i * 3 + j + 1
                    cx = x + cell_size // 2
                    cy = y + cell_size // 2
                    cv2.putText(warp, str(number), (cx - 10, cy + 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3)

        Minv = cv2.getPerspectiveTransform(dst, board_rect.astype(np.float32))
        warp_back = cv2.warpPerspective(warp, Minv, (image.shape[1], image.shape[0]))
        gray_warp = cv2.cvtColor(warp_back, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray_warp, 10, 255, cv2.THRESH_BINARY)
        mask_inv = cv2.bitwise_not(mask)
        img_bg = cv2.bitwise_and(image, image, mask=mask_inv)
        img_fg = cv2.bitwise_and(warp_back, warp_back, mask=mask)
        image[:] = cv2.add(img_bg, img_fg)

    def destroy_node(self):
        self.get_logger().info("正在关闭节点...")
        self.shutdown_event.set()
        self.processing_thread.join()
        super().destroy_node()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = ChessboardDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到终止信号，关闭节点")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
