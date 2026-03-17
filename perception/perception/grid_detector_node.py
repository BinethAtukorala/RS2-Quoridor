# perception/perception/grid_detector_node.py
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
import pyrealsense2 as rs
import numpy as np
import cv2
from cv_bridge import CvBridge

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

    class RealsenseCamera:
        def __init__(self):
            self.width = 1280
            self.height = 720
            self.fps = 30
            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
            config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
            profile = self.pipeline.start(config)
            depth_stream = profile.get_stream(rs.stream.depth)
            self.intrinsics = depth_stream.as_video_stream_profile().get_intrinsics()
            print("RealSense camera started")

        def get_frames(self):
            frames = self.pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                return None, None
            depth = np.asanyarray(depth_frame.get_data())
            color = np.asanyarray(color_frame.get_data())
            return color, depth

    def __init__(self):
        super().__init__('grid_detector_node')
        
        self.bridge = CvBridge()
        self.pub_image = self.create_publisher(Image, '/quoridor/topdown_grid', 10)
        self.pub_board = self.create_publisher(Int32MultiArray, '/quoridor/board_state', 10)

        # Create windows once here
        cv2.namedWindow("Camera View", cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow("Top-Down Board", cv2.WINDOW_AUTOSIZE)

        self.camera = self.RealsenseCamera()
        self.board = self.QuoridorBoard()

        self.get_logger().info("Grid Detector Node started")
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

    # -----------------------------
    # Helper functions as methods
    # -----------------------------
    def detect_board_corners(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)
        edges = cv2.Canny(blur, 50, 150)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) == 0:
            return None

        min_area = 20000
        max_area = image.shape[0]*image.shape[1]
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
        corners = largest.reshape(4,2)
        s = corners.sum(axis=1)
        diff = np.diff(corners, axis=1)
        ordered_corners = np.zeros((4,2), dtype=np.float32)
        ordered_corners[0] = corners[np.argmin(s)]
        ordered_corners[2] = corners[np.argmax(s)]
        ordered_corners[1] = corners[np.argmin(diff)]
        ordered_corners[3] = corners[np.argmax(diff)]
        return ordered_corners

    def compute_homography(self, corners):
        board_size = 500
        dst = np.array([[0,0],[board_size,0],[board_size,board_size],[0,board_size]], dtype=np.float32)
        src = np.array(corners, dtype=np.float32)
        H, _ = cv2.findHomography(src, dst)
        return H

    def warp_board(self, image, H):
        return cv2.warpPerspective(image, H, (500,500))

    def draw_grid(self, img):
        cell = 100
        for i in range(6):
            x = i*cell
            cv2.line(img, (x,0), (x,500), (0,255,0), 1)
            cv2.line(img, (0,x), (500,x), (0,255,0), 1)
        return img

    def detect_black_objects(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, 70, 255, cv2.THRESH_BINARY_INV)
        cell = 100
        margin = 30
        for r in range(5):
            for c in range(5):
                x1 = c*cell + margin
                y1 = r*cell + margin
                x2 = (c+1)*cell - margin
                y2 = (r+1)*cell - margin
                cell_region = mask[y1:y2, x1:x2]
                black_pixels = cv2.countNonZero(cell_region)
                area = (cell - 2*margin)**2
                ratio = black_pixels / area
                if ratio > 0.05:
                    self.board.set_cell(r, c, 1)
                    cv2.rectangle(img, (x1,y1),(x2,y2),(0,0,255),2)
                else:
                    self.board.set_cell(r, c, 0)

    # -----------------------------
    # Timer callback
    # -----------------------------
    # def timer_callback(self):
    #     color, _ = self.camera.get_frames()
    #     if color is None:
    #         return

    #     corners = self.detect_board_corners(color)
    #     if corners is not None:
    #         for p in corners:
    #             cv2.circle(color, (int(p[0]), int(p[1])), 6, (0,255,0), -1)
    #         H = self.compute_homography(corners)
    #         topdown = self.warp_board(color, H)
    #         self.board.clear()
    #         self.detect_black_objects(topdown)
    #         topdown = self.draw_grid(topdown)

    #         # Publish
    #         msg_img = self.bridge.cv2_to_imgmsg(topdown, encoding="bgr8")
    #         self.pub_image.publish(msg_img)
    #         msg_board = Int32MultiArray()
    #         msg_board.data = self.board.board.flatten().tolist()
    #         self.pub_board.publish(msg_board)

    # def timer_callback(self):
    #     color, _ = self.camera.get_frames()
    #     if color is None:
    #         return

    #     corners = self.detect_board_corners(color)
    #     if corners is not None:
    #         # Draw corners on color image
    #         for p in corners:
    #             cv2.circle(color, (int(p[0]), int(p[1])), 6, (0,255,0), -1)

    #         # Compute homography and top-down view
    #         H = self.compute_homography(corners)
    #         topdown = self.warp_board(color, H)

    #         # Clear board and detect black objects
    #         self.board.clear()
    #         self.detect_black_objects(topdown)
    #         topdown = self.draw_grid(topdown)

    #         # -------------------
    #         # Print board state
    #         # -------------------
    #         self.board.print_board()

    #         # -------------------
    #         # Show camera and top-down views
    #         # -------------------
    #         cv2.imshow("Camera View", color)
    #         cv2.imshow("Top-Down Board", topdown)
    #         cv2.waitKey(1)  # Needed to refresh the OpenCV windows

    #         # -------------------
    #         # Publish to ROS2
    #         # -------------------
    #         msg_img = self.bridge.cv2_to_imgmsg(topdown, encoding="bgr8")
    #         self.pub_image.publish(msg_img)

    #         msg_board = Int32MultiArray()
    #         msg_board.data = self.board.board.flatten().tolist()
    #         self.pub_board.publish(msg_board)

    def timer_callback(self):
        color, _ = self.camera.get_frames()
        if color is None:
            return

        corners = self.detect_board_corners(color)

        if corners is None:
            # Show at least the raw camera view even if no board is found
            cv2.imshow("Camera View", color)
            cv2.waitKey(1)
            return

        if corners is not None:
            for p in corners:
                cv2.circle(color, (int(p[0]), int(p[1])), 6, (0,255,0), -1)

            H = self.compute_homography(corners)
            topdown = self.warp_board(color, H)

            self.board.clear()
            self.detect_black_objects(topdown)
            topdown = self.draw_grid(topdown)

            # Print board to ROS2 console
            board_str = "\n".join(" ".join(str(c) for c in row) for row in self.board.board)
            self.get_logger().info(f"\nDigital Board:\n{board_str}")

            # Show camera & top-down board windows
            # cv2.namedWindow("Camera View", cv2.WINDOW_NORMAL)
            # cv2.namedWindow("Top-Down Board", cv2.WINDOW_NORMAL)
            cv2.imshow("Camera View", color)
            cv2.imshow("Top-Down Board", topdown)
            cv2.waitKey(1)

            # Publish ROS2 messages
            msg_img = self.bridge.cv2_to_imgmsg(topdown, encoding="bgr8")
            self.pub_image.publish(msg_img)

            msg_board = Int32MultiArray()
            msg_board.data = self.board.board.flatten().tolist()
            self.pub_board.publish(msg_board)

def main(args=None):
    rclpy.init(args=args)
    node = GridDetectorNode()
    rclpy.spin(node)
    node.camera.pipeline.stop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()