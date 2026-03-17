#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
import pyrealsense2 as rs
import numpy as np
import cv2
from cv_bridge import CvBridge

class CircleDetectorNode(Node):
    def __init__(self):
        super().__init__('circle_detector_node')
        
        # 1. ROS Parameters and Pubs
        self.bridge = CvBridge()
        self.pub_image = self.create_publisher(Image, '/quoridor/topdown_circles', 10)
        self.pub_walls = self.create_publisher(Int32MultiArray, '/quoridor/wall_state', 10)

        # 2. Setup Parameters for Detection
        self.board_size = 500
        self.rows, self.cols = 4, 4
        self.cell_size = 100
        self.wall_circles = np.zeros((self.rows, self.cols), dtype=int)
        self.prev_corners = None

        # 3. Setup GUI Windows
        cv2.namedWindow("Camera View (Circles)", cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow("Top-Down Walls", cv2.WINDOW_AUTOSIZE)

        # 4. Setup RealSense
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        self.pipeline.start(self.config)
        
        self.get_logger().info("Circle Detector Node started with RealSense")
        
        # 5. Timer (10 Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)

    # --- Detection Logic ---
    def detect_board_corners(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)
        edges = cv2.Canny(blur, 50, 150)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None

        min_area, max_area = 20000, image.shape[0] * image.shape[1]
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
        return largest.reshape(4, 2)

    def order_corners(self, corners):
        rect = np.zeros((4, 2), dtype=np.float32)
        s = corners.sum(axis=1)
        diff = np.diff(corners, axis=1)
        rect[0] = corners[np.argmin(s)]      # top-left
        rect[2] = corners[np.argmax(s)]      # bottom-right
        rect[1] = corners[np.argmin(diff)]   # top-right
        rect[3] = corners[np.argmax(diff)]   # bottom-left
        return rect

    # def detect_circles_and_walls(self, warped_img):
    #     hsv = cv2.cvtColor(warped_img, cv2.COLOR_BGR2HSV)

    #     # White circle mask
    #     lower_white = np.array([0, 0, 200])
    #     upper_white = np.array([180, 50, 255])
    #     white_mask = cv2.inRange(hsv, lower_white, upper_white)

    #     # Red wall mask
    #     lower_red1, upper_red1 = np.array([0, 120, 70]), np.array([10, 255, 255])
    #     lower_red2, upper_red2 = np.array([170, 120, 70]), np.array([180, 255, 255])
    #     red_mask = cv2.bitwise_or(cv2.inRange(hsv, lower_red1, upper_red1), 
    #                               cv2.inRange(hsv, lower_red2, upper_red2))

    #     self.wall_circles[:] = 0

    #     for r in range(self.rows):
    #         for c in range(self.cols):
    #             cx = (c + 1) * self.cell_size
    #             cy = (r + 1) * self.cell_size
    #             x1, y1 = max(cx - 8, 0), max(cy - 8, 0)
    #             x2, y2 = min(cx + 8, self.board_size), min(cy + 8, self.board_size)

    #             circle_crop = white_mask[y1:y2, x1:x2]
    #             if np.mean(circle_crop) > 50:
    #                 self.wall_circles[r, c] = 0
    #                 cv2.circle(warped_img, (cx, cy), 8, (255, 0, 0), 2)  # blue circle
    #             else:
    #                 red_crop = red_mask[y1:y2, x1:x2]
    #                 if np.mean(red_crop) > 50:
    #                     self.wall_circles[r, c] = 1
    #                     cv2.rectangle(warped_img, (x1, y1), (x2, y2), (0, 0, 255), 2)
    #     return warped_img

    # ------------ Detected horizontal and vertical walls
    # def detect_circles_and_walls(self, warped_img):
    #     hsv = cv2.cvtColor(warped_img, cv2.COLOR_BGR2HSV)

    #     # White circle mask
    #     lower_white = np.array([0, 0, 200])
    #     upper_white = np.array([180, 50, 255])
    #     white_mask = cv2.inRange(hsv, lower_white, upper_white)

    #     # Red wall mask
    #     lower_red1, upper_red1 = np.array([0, 120, 70]), np.array([10, 255, 255])
    #     lower_red2, upper_red2 = np.array([170, 120, 70]), np.array([180, 255, 255])
    #     red_mask = cv2.bitwise_or(cv2.inRange(hsv, lower_red1, upper_red1), 
    #                               cv2.inRange(hsv, lower_red2, upper_red2))

    #     self.wall_circles[:] = 0

    #     # Parameters for orientation check
    #     ext = 25  # How many pixels to look out from the center
    #     thickness = 5 # Width of the "search" beam

    #     for r in range(self.rows):
    #         for c in range(self.cols):
    #             cx = (c + 1) * self.cell_size
    #             cy = (r + 1) * self.cell_size

    #             # 1. Check if the white circle is visible
    #             x1, y1 = max(cx - 8, 0), max(cy - 8, 0)
    #             x2, y2 = min(cx + 8, self.board_size), min(cy + 8, self.board_size)
    #             circle_crop = white_mask[y1:y2, x1:x2]

    #             if np.mean(circle_crop) > 50:
    #                 self.wall_circles[r, c] = 0
    #                 cv2.circle(warped_img, (cx, cy), 8, (255, 0, 0), 2) # Blue = Empty
    #             else:
    #                 # 2. Circle is covered. Check orientation by looking at red mask extensions.
                    
    #                 # Horizontal "beam" (left to right)
    #                 h_crop = red_mask[cy-thickness : cy+thickness, cx-ext : cx+ext]
    #                 # Vertical "beam" (up to down)
    #                 v_crop = red_mask[cy-ext : cy+ext, cx-thickness : cx+thickness]

    #                 h_score = np.mean(h_crop) if h_crop.size > 0 else 0
    #                 v_score = np.mean(v_crop) if v_crop.size > 0 else 0

    #                 if h_score > v_score and h_score > 40:
    #                     self.wall_circles[r, c] = 2 # Horizontal
    #                     cv2.line(warped_img, (cx-ext, cy), (cx+ext, cy), (0, 255, 0), 3)
    #                 elif v_score > h_score and v_score > 40:
    #                     self.wall_circles[r, c] = 1 # Vertical
    #                     cv2.line(warped_img, (cx, cy-ext), (cx, cy+ext), (0, 0, 255), 3)
    #                 else:
    #                     self.wall_circles[r, c] = 0 # Ambiguous
        
    #     return warped_img


    #  ---------- Detects red and blue walls or robot VS human
    def detect_circles_and_walls(self, warped_img):
        hsv = cv2.cvtColor(warped_img, cv2.COLOR_BGR2HSV)

        # 1. White circle mask
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 50, 255])
        white_mask = cv2.inRange(hsv, lower_white, upper_white)

        # 2. Red wall mask (two ranges for red wrap-around)
        lower_red1, upper_red1 = np.array([0, 120, 70]), np.array([10, 255, 255])
        lower_red2, upper_red2 = np.array([170, 120, 70]), np.array([180, 255, 255])
        red_mask = cv2.bitwise_or(cv2.inRange(hsv, lower_red1, upper_red1), 
                                  cv2.inRange(hsv, lower_red2, upper_red2))

        # 3. Blue wall mask
        # Adjust these values if your blue walls are darker/lighter
        # lower_blue = np.array([100, 150, 50])
        # upper_blue = np.array([140, 255, 255])
        # blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # Expanded Blue mask
        # H: 90-130 covers most blues
        # S: 50-255 allows for "pale" or "washed out" blue
        # V: 30-255 allows for dark blue/shadows
        lower_blue = np.array([90, 50, 30]) 
        upper_blue = np.array([130, 255, 255])
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

        self.wall_circles[:] = 0
        ext = 25  
        thickness = 5 

        for r in range(self.rows):
            for c in range(self.cols):
                cx = (c + 1) * self.cell_size
                cy = (r + 1) * self.cell_size

                # Check if circle is visible
                x1, y1 = max(cx - 8, 0), max(cy - 8, 0)
                x2, y2 = min(cx + 8, self.board_size), min(cy + 8, self.board_size)
                circle_crop = white_mask[y1:y2, x1:x2]

                if np.mean(circle_crop) > 50:
                    self.wall_circles[r, c] = 0
                    cv2.circle(warped_img, (cx, cy), 8, (200, 200, 200), 2) # Light Grey = Circle
                else:
                    # Circle is covered. Check which color is dominant.
                    red_center = red_mask[y1:y2, x1:x2]
                    blue_center = blue_mask[y1:y2, x1:x2]

                    if np.mean(red_center) > 40:
                        # Logic for RED orientation
                        h_crop = red_mask[cy-thickness : cy+thickness, cx-ext : cx+ext]
                        v_crop = red_mask[cy-ext : cy+ext, cx-thickness : cx+thickness]
                        h_score, v_score = np.mean(h_crop), np.mean(v_crop)

                        if h_score > v_score:
                            self.wall_circles[r, c] = 2 # Horizontal Red
                            cv2.line(warped_img, (cx-ext, cy), (cx+ext, cy), (0, 0, 255), 3)
                        else:
                            self.wall_circles[r, c] = 1 # Vertical Red
                            cv2.line(warped_img, (cx, cy-ext), (cx, cy+ext), (0, 0, 255), 3)

                    elif np.mean(blue_center) > 40:
                        # Logic for BLUE orientation
                        h_crop = blue_mask[cy-thickness : cy+thickness, cx-ext : cx+ext]
                        v_crop = blue_mask[cy-ext : cy+ext, cx-thickness : cx+thickness]
                        h_score, v_score = np.mean(h_crop), np.mean(v_crop)

                        if h_score > v_score:
                            self.wall_circles[r, c] = 4 # Horizontal Blue
                            cv2.line(warped_img, (cx-ext, cy), (cx+ext, cy), (255, 0, 0), 3)
                        else:
                            self.wall_circles[r, c] = 3 # Vertical Blue
                            cv2.line(warped_img, (cx, cy-ext), (cx, cy+ext), (255, 0, 0), 3)
        # Check blue and red masks
        # cv2.imshow("Red Debug Mask", red_mask)
        # cv2.imshow("Blue Debug Mask", blue_mask)

        return warped_img

    # --- Main Loop ---
    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return
        
        img = np.asanyarray(color_frame.get_data())
        raw_corners = self.detect_board_corners(img)

        if raw_corners is None:
            cv2.imshow("Camera View (Circles)", img)
            cv2.waitKey(1)
            return

        corners = self.order_corners(raw_corners)
        
        # Draw corners for visual feedback
        for p in corners:
            cv2.circle(img, (int(p[0]), int(p[1])), 6, (0, 255, 0), -1)

        # Compute Homography and Warp
        dst = np.array([[0,0], [self.board_size,0], [self.board_size,self.board_size], [0,self.board_size]], dtype=np.float32)
        H, _ = cv2.findHomography(corners, dst)
        warped = cv2.warpPerspective(img, H, (self.board_size, self.board_size))

        # Detect Walls
        warped = self.detect_circles_and_walls(warped)

        # Terminal Output
        # wall_str = "\n".join(" ".join(str(c) for c in row) for row in self.wall_circles)
        # self.get_logger().info(f"\nWall State (0=Circle, 1=Wall):\n{wall_str}")
        # Terminal Output
        # wall_labels = {0: "Empty", 1: "Vertical", 2: "Horizontal"}
        # self.get_logger().info("\n--- Wall Orientation State ---")
        # for row in self.wall_circles:
        #     self.get_logger().info(str([wall_labels[val] for val in row]))
        # Terminal Output
        wall_labels = {
            0: "Empty", 
            1: "V-Red", 
            2: "H-Red", 
            3: "V-Blue", 
            4: "H-Blue"
        }
        self.get_logger().info("\n--- Wall Orientation & Color State ---")
        for row in self.wall_circles:
            self.get_logger().info(str([wall_labels[val] for val in row]))

        # GUI Update
        cv2.imshow("Camera View (Circles)", img)
        cv2.imshow("Top-Down Walls", warped)

        cv2.waitKey(1)

        # Publish Images and Data
        msg_img = self.bridge.cv2_to_imgmsg(warped, encoding="bgr8")
        self.pub_image.publish(msg_img)
        
        msg_wall = Int32MultiArray()
        msg_wall.data = self.wall_circles.flatten().tolist()
        self.pub_walls.publish(msg_wall)

def main(args=None):
    rclpy.init(args=args)
    node = CircleDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pipeline.stop()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()