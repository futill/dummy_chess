import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import cv2
import numpy as np

class ChessboardDetectorNode(Node):
    def __init__(self):
        super().__init__('chessboard_detector')

        # 修改为你摄像头图像话题
        image_topic = '/camera/colcor/image_raw'

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )
        self.get_logger().info(f"Subscribed to image topic: {image_topic}")

    def image_callback(self, msg):
        self.get_logger().info("Received image frame")
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge conversion error: {e}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)

        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        board_rect = self.find_largest_rectangle(contours)

        if board_rect is not None:
            cv2.polylines(frame, [board_rect], isClosed=True, color=(0,255,0), thickness=3)

            # 将棋盘切割成9个格子，标记数字1-9
            self.draw_grid_numbers(frame, board_rect)
        else:
            self.get_logger().info("No chessboard rectangle found")

        cv2.imshow("Chessboard Detection", frame)
        cv2.waitKey(1)

    def find_largest_rectangle(self, contours):
        max_area = 0
        best_rect = None

        for cnt in contours:
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)

            if len(approx) == 4 and cv2.isContourConvex(approx):
                area = cv2.contourArea(approx)
                if area > max_area:
                    max_area = area
                    best_rect = approx.reshape(4, 2)

        if best_rect is not None:
            # 按顺序排序四个点（左上，右上，右下，左下）
            best_rect = self.order_points(best_rect)
            self.get_logger().info(f"Found largest rectangle with area: {max_area:.2f}")
        return best_rect

    def order_points(self, pts):
        # 坐标排序，左上、右上、右下、左下
        rect = np.zeros((4, 2), dtype="float32")
        s = pts.sum(axis=1)
        rect[0] = pts[np.argmin(s)]
        rect[2] = pts[np.argmax(s)]

        diff = np.diff(pts, axis=1)
        rect[1] = pts[np.argmin(diff)]
        rect[3] = pts[np.argmax(diff)]

        return rect.astype(int)

    def draw_grid_numbers(self, image, board_rect):
        # 将四边形变换成正方形，方便均分
        width = max(np.linalg.norm(board_rect[0] - board_rect[1]),
                    np.linalg.norm(board_rect[2] - board_rect[3]))
        height = max(np.linalg.norm(board_rect[0] - board_rect[3]),
                     np.linalg.norm(board_rect[1] - board_rect[2]))

        size = int(max(width, height))
        dst = np.array([
            [0, 0],
            [size - 1, 0],
            [size - 1, size - 1],
            [0, size - 1]
        ], dtype="float32")

        M = cv2.getPerspectiveTransform(board_rect.astype(np.float32), dst)
        warp = cv2.warpPerspective(image, M, (size, size))

        # 每个格子大小
        cell_size = size // 3

        # 在warp图像上标数字
        for i in range(3):
            for j in range(3):
                number = i * 3 + j + 1
                x = j * cell_size + cell_size // 2
                y = i * cell_size + cell_size // 2
                cv2.putText(
                    warp,
                    str(number),
                    (x - 10, y + 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.5,
                    (0, 0, 255),
                    3,
                    cv2.LINE_AA
                )

        # 反变换标注回原图
        Minv = cv2.getPerspectiveTransform(dst, board_rect.astype(np.float32))
        warp_back = cv2.warpPerspective(warp, Minv, (image.shape[1], image.shape[0]))

        # 创建掩膜，融合标注
        gray_warp = cv2.cvtColor(warp_back, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray_warp, 10, 255, cv2.THRESH_BINARY)
        mask_inv = cv2.bitwise_not(mask)

        img_bg = cv2.bitwise_and(image, image, mask=mask_inv)
        img_fg = cv2.bitwise_and(warp_back, warp_back, mask=mask)

        image[:] = cv2.add(img_bg, img_fg)

def main(args=None):
    rclpy.init(args=args)
    node = ChessboardDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
