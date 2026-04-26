# perception/perception/grid_detector_node.py
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
import numpy as np
import cv2
from cv_bridge import CvBridge
import os
from std_msgs.msg import Float32MultiArray
import json


class GridDetectorNode(Node):
    class QuoridorBoard:
        def __init__(self):
            self.board = np.zeros((5, 5), dtype=int)

        def clear(self):
            self.board[:] = 0

        def set_cell(self, row, col, value):
            if 0 <= row < 5 and 0 <= col < 5:
                self.board[row, col] = value

        def print_board(self):
            print("\nDigital Board:\n")
            for r in range(5):
                for c in range(5):
                    print(self.board[r, c], end=" ")
                print()
            print()

    def __init__(self):
        super().__init__('grid_detector_node')

        self.bridge = CvBridge()

        self.pub_image = self.create_publisher(Image, '/perception/topdown_grid', 10)
        self.pub_board = self.create_publisher(Int32MultiArray, '/perception/board_state', 10)
        self.pub_pawns = self.create_publisher(Float32MultiArray, '/perception/pawns_3d', 10)

        self.sub_color = self.create_subscription(
            Image, '/camera/color', self.color_callback, 10)
        self.sub_depth = self.create_subscription(
            Image, '/camera/depth', self.depth_callback, 10)

        self.latest_color = None
        self.latest_depth = None

        cv2.namedWindow("Camera View", cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow("Top-Down Board", cv2.WINDOW_AUTOSIZE)

        self.board = self.QuoridorBoard()

        # self.grid_file = os.path.expanduser("/ros2_ws/src/perception/grid_coords.txt")
        self.grid_file = os.path.expanduser("~/rs2_ws/src/perception/pawn_coords.txt")
        self.grid_lookup = {}
        self.load_grid_coordinates()

        self.intrinsics_file = os.path.expanduser("~/rs2_ws/src/perception/camera_intrinsics.json")
        with open(self.intrinsics_file, "r") as f:
            intr = json.load(f)
        self.fx = intr["fx"]
        self.fy = intr["fy"]
        self.ppx = intr["ppx"]
        self.ppy = intr["ppy"]

        # In __init__
        self.pub_corners = self.create_publisher(Float32MultiArray, '/perception/board_corners', 10)

        self.get_logger().info("Grid Detector Node started")
        self.timer = self.create_timer(0.1, self.timer_callback)

    def color_callback(self, msg):
        self.latest_color = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def depth_callback(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def load_grid_coordinates(self):
        try:
            with open(self.grid_file, "r") as f:
                for line in f:
                    parts = line.strip().split(",")
                    if len(parts) != 5:
                        continue
                    r, c = int(parts[0]), int(parts[1])
                    x, y, z = float(parts[2]), float(parts[3]), float(parts[4])
                    self.grid_lookup[(r, c)] = [x, y, z]
            self.get_logger().info(f"Loaded {len(self.grid_lookup)} grid coordinates from file")
        except Exception as e:
            self.get_logger().warn(f"Failed to load grid file: {e}")

    def detect_board_corners(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) == 0:
            return None

        min_area = 20000
        max_area = image.shape[0] * image.shape[1]
        board_contours = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if min_area < area < max_area:
                epsilon = 0.02 * cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, epsilon, True)
                if len(approx) == 4:
                    board_contours.append(approx)
        if not board_contours:
            return None

        largest = max(board_contours, key=cv2.contourArea)
        corners = largest.reshape(4, 2)
        s = corners.sum(axis=1)
        diff = np.diff(corners, axis=1)
        ordered = np.zeros((4, 2), dtype=np.float32)
        ordered[0] = corners[np.argmin(s)]
        ordered[2] = corners[np.argmax(s)]
        ordered[1] = corners[np.argmin(diff)]
        ordered[3] = corners[np.argmax(diff)]
        return ordered

    def compute_homography(self, corners):
        board_size = 500
        dst = np.array([[0, 0], [board_size, 0], [board_size, board_size], [0, board_size]], dtype=np.float32)
        H, _ = cv2.findHomography(np.array(corners, dtype=np.float32), dst)
        return H

    def warp_board(self, image, H):
        return cv2.warpPerspective(image, H, (500, 500))

    def draw_grid(self, img):
        cell = 100
        for i in range(6):
            x = i * cell
            cv2.line(img, (x, 0), (x, 500), (0, 255, 0), 1)
            cv2.line(img, (0, x), (500, x), (0, 255, 0), 1)
        return img

    # def detect_black_objects(self, img, pawns_3d_grid, H_inv=None, depth_frame=None):
    def detect_black_objects(self, img, pawns_list, H_inv=None, depth_frame=None):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, 70, 255, cv2.THRESH_BINARY_INV)
        cell = 100
        margin = 30

        for r in range(5):
            for c in range(5):
                x1, y1 = c * cell + margin, r * cell + margin
                x2, y2 = (c + 1) * cell - margin, (r + 1) * cell - margin

                cell_region = mask[y1:y2, x1:x2]
                black_pixels = cv2.countNonZero(cell_region)
                area = (cell - 2 * margin) ** 2

                if (black_pixels / area) > 0.05:
                    self.board.set_cell(r, c, 1)

                    cnts, _ = cv2.findContours(cell_region, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    if cnts:
                        largest_cnt = max(cnts, key=cv2.contourArea)
                        bx, by, bw, bh = cv2.boundingRect(largest_cnt)
                        cv2.rectangle(img, (x1 + bx, y1 + by), (x1 + bx + bw, y1 + by + bh), (0, 0, 255), 2)

                        if (r, c) in self.grid_lookup:
                            point_3d = self.grid_lookup[(r, c)]
                            # pawns_3d_grid[r][c] = point_3d
                            pawns_list.append([
                                float(r),
                                float(c),
                                point_3d[0],
                                point_3d[1],
                                point_3d[2]
                            ])
                            self.get_logger().info(
                                f"Pawn at [{r},{c}] 3D: x={point_3d[0]:.3f}, y={point_3d[1]:.3f}, z={point_3d[2]:.3f}"
                            )
                        else:
                            self.get_logger().warn(f"Pawn at [{r},{c}] detected but grid coordinate not found")
                else:
                    self.board.set_cell(r, c, 0)

    def timer_callback(self):
        if self.latest_color is None or self.latest_depth is None:
            return

        img = self.latest_color.copy()
        depth_frame = self.latest_depth.copy()

        # pawns_3d_grid = [[[0.0, 0.0, 0.0] for _ in range(5)] for _ in range(5)]
        pawns_list = []  # each entry: [row, col, x, y, z]

        # Bug fix: was `color` (undefined), now correctly `img`
        corners = self.detect_board_corners(img)
        if corners is None:
            cv2.imshow("Camera View", img)
            cv2.waitKey(1)
            return

        for p in corners:
            cv2.circle(img, (int(p[0]), int(p[1])), 6, (0, 255, 0), -1)


        # In timer_callback, after detecting corners:
        if corners is not None:
            msg_corners = Float32MultiArray()
            msg_corners.data = corners.flatten().tolist()
            self.pub_corners.publish(msg_corners)

        H = self.compute_homography(corners)
        H_inv = np.linalg.inv(H)
        topdown = self.warp_board(img, H)

        self.board.clear()
        # self.detect_black_objects(topdown, pawns_3d_grid, H_inv, depth_frame)
        self.detect_black_objects(topdown, pawns_list, H_inv, depth_frame)
        topdown = self.draw_grid(topdown)

        board_str = "\n".join(" ".join(str(c) for c in row) for row in self.board.board)
        self.get_logger().info(f"\nDigital Board:\n{board_str}")

        cv2.imshow("Camera View", img)
        cv2.imshow("Top-Down Board", topdown)
        cv2.waitKey(1)

        msg_img = self.bridge.cv2_to_imgmsg(topdown, encoding="bgr8")
        self.pub_image.publish(msg_img)

        msg_board = Int32MultiArray()
        msg_board.data = self.board.board.flatten().tolist()
        self.pub_board.publish(msg_board)

        # flat_data = []
        # for row in pawns_3d_grid:
        #     for cell in row:
        #         flat_data.extend(cell)

        flat_data = []
        for pawn in pawns_list:
            flat_data.extend(pawn)

        msg_pawns = Float32MultiArray()
        msg_pawns.data = flat_data
        self.pub_pawns.publish(msg_pawns)


def main(args=None):
    rclpy.init(args=args)
    node = GridDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'pipeline'):
            node.pipeline.stop()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()