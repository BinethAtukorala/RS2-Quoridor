# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from std_msgs.msg import Int32MultiArray
# import pyrealsense2 as rs
# import numpy as np
# import cv2
# from cv_bridge import CvBridge

# class CircleDetectorNode(Node):
#     def __init__(self):
#         super().__init__('circle_detector_node')
        
#         # 1. ROS Parameters and Pubs
#         self.bridge = CvBridge()
#         self.pub_image = self.create_publisher(Image, '/quoridor/topdown_circles', 10)
#         self.pub_walls = self.create_publisher(Int32MultiArray, '/quoridor/wall_state', 10)

#         # 2. Setup Parameters for Detection
#         self.board_size = 500
#         self.rows, self.cols = 4, 4
#         self.cell_size = 100
#         self.wall_circles = np.zeros((self.rows, self.cols), dtype=int)
#         self.prev_corners = None

#         # 3. Setup GUI Windows
#         cv2.namedWindow("Camera View (Circles)", cv2.WINDOW_AUTOSIZE)
#         cv2.namedWindow("Top-Down Walls", cv2.WINDOW_AUTOSIZE)

#         # # 4. Setup RealSense
#         # self.pipeline = rs.pipeline()
#         # self.config = rs.config()
#         # self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
#         # self.pipeline.start(self.config)

#         # 4. Setup RealSense (MODIFIED FOR BAG SUPPORT)
#         self.pipeline = rs.pipeline()
#         self.config = rs.config()

#         # Add bag file support
#         self.declare_parameter('bag_file', '')
#         bag_file = self.get_parameter('bag_file').get_parameter_value().string_value

#         self.is_bag = bag_file != ""

#         if self.is_bag:
#             self.get_logger().info(f"Using BAG file: {bag_file}")
#             rs.config.enable_device_from_file(self.config, bag_file, repeat_playback=False)
#         else:
#             self.get_logger().info("Using LIVE camera")
#             self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

#         # Start pipeline
#         self.profile = self.pipeline.start(self.config)

#         # Fix playback timing
#         if self.is_bag:
#             device = self.profile.get_device()
#             playback = device.as_playback()
#             playback.set_real_time(False)
        
#         self.get_logger().info("Circle Detector Node started with RealSense")
        
#         # 5. Timer (10 Hz)
#         self.timer = self.create_timer(0.1, self.timer_callback)

#     # --- Detection Logic ---
#     def detect_board_corners(self, image):
#         gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
#         blur = cv2.GaussianBlur(gray, (5,5), 0)
#         edges = cv2.Canny(blur, 50, 150)
#         contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
#         if not contours:
#             return None

#         min_area, max_area = 20000, image.shape[0] * image.shape[1]
#         board_contours = []
#         for cnt in contours:
#             area = cv2.contourArea(cnt)
#             if min_area < area < max_area:
#                 epsilon = 0.02 * cv2.arcLength(cnt, True)
#                 approx = cv2.approxPolyDP(cnt, epsilon, True)
#                 if len(approx) == 4:
#                     board_contours.append(approx)

#         if not board_contours:
#             return None

#         largest = max(board_contours, key=cv2.contourArea)
#         return largest.reshape(4, 2)

#     def order_corners(self, corners):
#         rect = np.zeros((4, 2), dtype=np.float32)
#         s = corners.sum(axis=1)
#         diff = np.diff(corners, axis=1)
#         rect[0] = corners[np.argmin(s)]      # top-left
#         rect[2] = corners[np.argmax(s)]      # bottom-right
#         rect[1] = corners[np.argmin(diff)]   # top-right
#         rect[3] = corners[np.argmax(diff)]   # bottom-left
#         return rect

#     #  ---------- Detects circles and red walls
#     def detect_circles_and_walls(self, warped_img):
#         hsv = cv2.cvtColor(warped_img, cv2.COLOR_BGR2HSV)

#         # 1. Masks
#         lower_white = np.array([0, 0, 200]); upper_white = np.array([180, 50, 255])
#         white_mask = cv2.inRange(hsv, lower_white, upper_white)

#         lower_red1, upper_red1 = np.array([0, 120, 70]), np.array([10, 255, 255])
#         lower_red2, upper_red2 = np.array([170, 120, 70]), np.array([180, 255, 255])
#         red_mask = cv2.bitwise_or(cv2.inRange(hsv, lower_red1, upper_red1), 
#                                 cv2.inRange(hsv, lower_red2, upper_red2))

#         self.wall_circles[:] = 0
#         ext = 25  
        
#         # --- 1. Global Red Detection ---
#         red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#         for cnt in red_contours:
#             if cv2.contourArea(cnt) < 100: 
#                 continue
                
#             # Get the GLOBAL bounding box of the red mask
#             rx, ry, rw, rh = cv2.boundingRect(cnt)
            
#             # CENTER A: The exact center of the red bounding box (The "Leaning" Top)
#             cx_blob = rx + (rw // 2)
#             cy_blob = ry + (rh // 2)

#             # Draw the actual Red Mask Bounding Box
#             cv2.rectangle(warped_img, (rx, ry), (rx + rw, ry + rh), (0, 0, 255), 2)

#             # --- VISUALIZATION 1: Red Blob Centroid (Yellow Star + Red Line) ---
#             cv2.drawMarker(warped_img, (cx_blob, cy_blob), (0, 255, 255), 
#                         markerType=cv2.MARKER_STAR, markerSize=10, thickness=1)
            
#             is_horizontal = rw > rh
#             if is_horizontal:
#                 cv2.line(warped_img, (cx_blob - ext, cy_blob), (cx_blob + ext, cy_blob), (0, 0, 255), 1)
#             else:
#                 cv2.line(warped_img, (cx_blob, cy_blob - ext), (cx_blob, cy_blob + ext), (0, 0, 255), 1)

#             # Map this blob to the nearest Grid Cell
#             c = int(round(cx_blob / self.cell_size)) - 1
#             r = int(round(cy_blob / self.cell_size)) - 1

#             # --- VISUALIZATION 2: Circle Center (Cyan Star + Green Line) ---
#             if 0 <= r < self.rows and 0 <= c < self.cols:
#                 # CENTER B: The "Ground Truth" coordinate (The Target)
#                 cx_circle = (c + 1) * self.cell_size
#                 cy_circle = (r + 1) * self.cell_size
                
#                 self.wall_circles[r, c] = 2 if is_horizontal else 1

#                 # Draw Ground Truth Marker
#                 cv2.drawMarker(warped_img, (cx_circle, cy_circle), (255, 255, 0), 
#                             markerType=cv2.MARKER_STAR, markerSize=12, thickness=2)
                
#                 if is_horizontal:
#                     cv2.line(warped_img, (cx_circle - ext, cy_circle), (cx_circle + ext, cy_circle), (0, 255, 0), 2)
#                 else:
#                     cv2.line(warped_img, (cx_circle, cy_circle - ext), (cx_circle, cy_circle + ext), (0, 255, 0), 2)

#                 # Draw a white connector to show the "leaning" error
#                 cv2.line(warped_img, (cx_blob, cy_blob), (cx_circle, cy_circle), (255, 255, 255), 1)

#         # --- 2. Empty Circle Detection (Optional Visual) ---
#         for r in range(self.rows):
#             for c in range(self.cols):
#                 if self.wall_circles[r, c] == 0:
#                     cx = (c + 1) * self.cell_size
#                     cy = (r + 1) * self.cell_size
#                     x1, y1 = max(cx - 8, 0), max(cy - 8, 0)
#                     x2, y2 = min(cx + 8, warped_img.shape[1]), min(cy + 8, warped_img.shape[0])
#                     if np.mean(white_mask[y1:y2, x1:x2]) > 50:
#                         cv2.circle(warped_img, (cx, cy), 8, (200, 200, 200), 1)

#         return warped_img
    
#     def draw_bounding_box(self, mask, warped_img, color):
#         contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#         for cnt in contours:
#             area = cv2.contourArea(cnt)

#             # Ignore tiny noise
#             if area < 50:
#                 continue

#             x, y, w, h = cv2.boundingRect(cnt)

#             cv2.rectangle(
#                 warped_img,
#                 (x, y),
#                 (x + w, y + h),
#                 color,
#                 2
#             )

#     # --- Main Loop ---
#     def timer_callback(self):
#         frames = self.pipeline.wait_for_frames()
#         color_frame = frames.get_color_frame()
#         if not color_frame:
#             return
        
#         img = np.asanyarray(color_frame.get_data())
#         raw_corners = self.detect_board_corners(img)

#         if raw_corners is None:
#             cv2.imshow("Camera View (Circles)", img)
#             cv2.waitKey(1)
#             return

#         corners = self.order_corners(raw_corners)
        
#         # Draw corners for visual feedback
#         for p in corners:
#             cv2.circle(img, (int(p[0]), int(p[1])), 6, (0, 255, 0), -1)

#         # Compute Homography and Warp
#         dst = np.array([[0,0], [self.board_size,0], [self.board_size,self.board_size], [0,self.board_size]], dtype=np.float32)
#         H, _ = cv2.findHomography(corners, dst)
#         warped = cv2.warpPerspective(img, H, (self.board_size, self.board_size))

#         # Detect Walls
#         warped = self.detect_circles_and_walls(warped)

#         # Terminal Output
#         # wall_str = "\n".join(" ".join(str(c) for c in row) for row in self.wall_circles)
#         # self.get_logger().info(f"\nWall State (0=Circle, 1=Wall):\n{wall_str}")
#         # Terminal Output
#         # wall_labels = {0: "Empty", 1: "Vertical", 2: "Horizontal"}
#         # self.get_logger().info("\n--- Wall Orientation State ---")
#         # for row in self.wall_circles:
#         #     self.get_logger().info(str([wall_labels[val] for val in row]))
#         # Terminal Output
#         wall_labels = {
#             0: "Empty", 
#             1: "V-Red", 
#             2: "H-Red", 
#             3: "V-Blue", 
#             4: "H-Blue"
#         }
#         self.get_logger().info("\n--- Wall Orientation & Color State ---")
#         for row in self.wall_circles:
#             self.get_logger().info(str([wall_labels[val] for val in row]))

#         # GUI Update
#         cv2.imshow("Camera View (Circles)", img)
#         cv2.imshow("Top-Down Walls", warped)

#         cv2.waitKey(1)

#         # Publish Images and Data
#         msg_img = self.bridge.cv2_to_imgmsg(warped, encoding="bgr8")
#         self.pub_image.publish(msg_img)
        
#         msg_wall = Int32MultiArray()
#         msg_wall.data = self.wall_circles.flatten().tolist()
#         self.pub_walls.publish(msg_wall)

# def main(args=None):
#     rclpy.init(args=args)
#     node = CircleDetectorNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.pipeline.stop()
#         cv2.destroyAllWindows()
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()


#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
import pyrealsense2 as rs
import numpy as np
import cv2
import os
from cv_bridge import CvBridge

class CircleDetectorNode(Node):
    def __init__(self):
        super().__init__('circle_detector_node')
        
        # 1. ROS Parameters and Pubs
        self.bridge = CvBridge()
        self.pub_image = self.create_publisher(Image, '/quoridor/topdown_circles', 10)
        self.pub_walls = self.create_publisher(Int32MultiArray, '/quoridor/wall_state', 10)

        # 2. File Reading Setup
        self.circle_file = os.path.expanduser("~/ros2_ws/src/perception/circle_coords.txt")
        self.saved_coords = {} # Stores (gx, gy) -> (x, y, z)
        self.load_circle_coords()

        # 3. Setup Parameters for Detection
        self.board_size = 500
        self.rows, self.cols = 4, 4
        self.cell_size = 100
        self.wall_circles = np.zeros((self.rows, self.cols), dtype=int)
        self.prev_corners = None

        # 4. Setup GUI Windows
        cv2.namedWindow("Camera View (Circles)", cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow("Top-Down Walls", cv2.WINDOW_AUTOSIZE)

        # 5. Setup RealSense (MODIFIED FOR BAG SUPPORT)
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        self.declare_parameter('bag_file', '')
        bag_file = self.get_parameter('bag_file').get_parameter_value().string_value
        self.is_bag = bag_file != ""

        if self.is_bag:
            self.get_logger().info(f"Using BAG file: {bag_file}")
            rs.config.enable_device_from_file(self.config, bag_file, repeat_playback=False)
        else:
            self.get_logger().info("Using LIVE camera")
            self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

        self.profile = self.pipeline.start(self.config)

        if self.is_bag:
            device = self.profile.get_device()
            playback = device.as_playback()
            playback.set_real_time(False)
        
        self.get_logger().info("Circle Detector Node started with RealSense and Coordinate File Support")
        self.timer = self.create_timer(0.1, self.timer_callback)

    def load_circle_coords(self):
        """Read the 3D coordinates from the text file saved by the calibration node."""
        if not os.path.exists(self.circle_file):
            self.get_logger().error(f"Circle coordinate file NOT FOUND at {self.circle_file}. Run calibration first!")
            return
        
        try:
            with open(self.circle_file, 'r') as f:
                for line in f:
                    # Format: gx, gy, x, y, z
                    parts = line.strip().split(',')
                    if len(parts) == 5:
                        gx, gy = int(parts[0]), int(parts[1])
                        x, y, z = float(parts[2]), float(parts[3]), float(parts[4])
                        self.saved_coords[(gx, gy)] = (x, y, z)
            self.get_logger().info(f"Successfully loaded {len(self.saved_coords)} circle coordinates.")
        except Exception as e:
            self.get_logger().error(f"Error reading circle file: {e}")

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

    def detect_circles_and_walls(self, warped_img):
        hsv = cv2.cvtColor(warped_img, cv2.COLOR_BGR2HSV)

        # Masks
        lower_white = np.array([0, 0, 200]); upper_white = np.array([180, 50, 255])
        white_mask = cv2.inRange(hsv, lower_white, upper_white)

        lower_red1, upper_red1 = np.array([0, 120, 70]), np.array([10, 255, 255])
        lower_red2, upper_red2 = np.array([170, 120, 70]), np.array([180, 255, 255])
        red_mask = cv2.bitwise_or(cv2.inRange(hsv, lower_red1, upper_red1), 
                                cv2.inRange(hsv, lower_red2, upper_red2))

        self.wall_circles[:] = 0
        ext = 25  
        
        red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in red_contours:
            if cv2.contourArea(cnt) < 100: 
                continue
                
            rx, ry, rw, rh = cv2.boundingRect(cnt)
            cx_blob = rx + (rw // 2)
            cy_blob = ry + (rh // 2)

            cv2.rectangle(warped_img, (rx, ry), (rx + rw, ry + rh), (0, 0, 255), 2)
            cv2.drawMarker(warped_img, (cx_blob, cy_blob), (0, 255, 255), 
                        markerType=cv2.MARKER_STAR, markerSize=10, thickness=1)
            
            is_horizontal = rw > rh
            
            # Map this blob to the nearest Grid Cell
            c = int(round(cx_blob / self.cell_size)) - 1
            r = int(round(cy_blob / self.cell_size)) - 1

            if 0 <= r < self.rows and 0 <= c < self.cols:
                cx_circle = (c + 1) * self.cell_size
                cy_circle = (r + 1) * self.cell_size
                
                self.wall_circles[r, c] = 2 if is_horizontal else 1

                # Visuals
                cv2.drawMarker(warped_img, (cx_circle, cy_circle), (255, 255, 0), 
                            markerType=cv2.MARKER_STAR, markerSize=12, thickness=2)
                
                color = (0, 255, 0) if is_horizontal else (0, 255, 0)
                if is_horizontal:
                    cv2.line(warped_img, (cx_circle - ext, cy_circle), (cx_circle + ext, cy_circle), color, 2)
                else:
                    cv2.line(warped_img, (cx_circle, cy_circle - ext), (cx_circle, cy_circle + ext), color, 2)

                # --- OUTPUT COORDINATES FROM FILE IN CM ---
                if (c, r) in self.saved_coords:
                    x, y, z = self.saved_coords[(c, r)]
                    self.get_logger().info(f"Wall at Circle [{c},{r}] 3D (cm): x={x*100:.2f}, y={y*100:.2f}, z={z*100:.2f}")

        # Empty Circle Detection Visual
        for r in range(self.rows):
            for c in range(self.cols):
                if self.wall_circles[r, c] == 0:
                    cx = (c + 1) * self.cell_size
                    cy = (r + 1) * self.cell_size
                    x1, y1 = max(cx - 8, 0), max(cy - 8, 0)
                    x2, y2 = min(cx + 8, warped_img.shape[1]), min(cy + 8, warped_img.shape[0])
                    if np.mean(white_mask[y1:y2, x1:x2]) > 50:
                        cv2.circle(warped_img, (cx, cy), 8, (200, 200, 200), 1)

        return warped_img

    # def timer_callback(self):
    #     frames = self.pipeline.wait_for_frames()
    #     color_frame = frames.get_color_frame()
    #     if not color_frame:
    #         return
        
    #     img = np.asanyarray(color_frame.get_data())
    #     raw_corners = self.detect_board_corners(img)

    #     if raw_corners is None:
    #         cv2.imshow("Camera View (Circles)", img)
    #         cv2.waitKey(1)
    #         return

    #     corners = self.order_corners(raw_corners)
    #     for p in corners:
    #         cv2.circle(img, (int(p[0]), int(p[1])), 6, (0, 255, 0), -1)

    #     dst = np.array([[0,0], [self.board_size,0], [self.board_size,self.board_size], [0,self.board_size]], dtype=np.float32)
    #     H, _ = cv2.findHomography(corners, dst)
    #     warped = cv2.warpPerspective(img, H, (self.board_size, self.board_size))

    #     warped = self.detect_circles_and_walls(warped)

    #     # Log wall state
    #     wall_labels = {0: "Empty", 1: "V-Red", 2: "H-Red", 3: "V-Blue", 4: "H-Blue"}
    #     self.get_logger().info("\n--- Wall Orientation & Color State ---")
    #     for row in self.wall_circles:
    #         self.get_logger().info(str([wall_labels[val] for val in row]))

    #     cv2.imshow("Camera View (Circles)", img)
    #     cv2.imshow("Top-Down Walls", warped)
    #     cv2.waitKey(1)

    #     # Publish
    #     msg_img = self.bridge.cv2_to_imgmsg(warped, encoding="bgr8")
    #     self.pub_image.publish(msg_img)
    #     msg_wall = Int32MultiArray(data=self.wall_circles.flatten().tolist())
    #     self.pub_walls.publish(msg_wall)


    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()   ### NEW: Get depth for outside walls
        if not color_frame:
            return

        img = np.asanyarray(color_frame.get_data())
        raw_corners = self.detect_board_corners(img)

        if raw_corners is None:
            cv2.imshow("Camera View (Circles)", img)
            cv2.waitKey(1)
            return

        corners = self.order_corners(raw_corners)
        for p in corners:
            cv2.circle(img, (int(p[0]), int(p[1])), 6, (0, 255, 0), -1)

        dst = np.array([[0,0], [self.board_size,0], [self.board_size,self.board_size], [0,self.board_size]], dtype=np.float32)
        H, _ = cv2.findHomography(corners, dst)
        warped = cv2.warpPerspective(img, H, (self.board_size, self.board_size))

        warped = self.detect_circles_and_walls(warped)

        # --- NEW: Detect red walls outside the board in the original image
        # outside_red_walls = self.detect_outside_walls(img, depth_frame)
        outside_red_walls = self.detect_outside_walls(img, depth_frame, raw_corners)  # pass the board corners
        for wall in outside_red_walls:
            cx, cy, x3d, y3d, z3d = wall
            cv2.circle(img, (cx, cy), 10, (0, 0, 255), 2)
            self.get_logger().info(f"Outside Wall 3D (cm): x={x3d*100:.2f}, y={y3d*100:.2f}, z={z3d*100:.2f}")

        # Log wall state
        wall_labels = {0: "Empty", 1: "V-Red", 2: "H-Red", 3: "V-Blue", 4: "H-Blue"}
        self.get_logger().info("\n--- Wall Orientation & Color State ---")
        for row in self.wall_circles:
            self.get_logger().info(str([wall_labels[val] for val in row]))

        cv2.imshow("Camera View (Circles)", img)
        cv2.imshow("Top-Down Walls", warped)
        cv2.waitKey(1)

        # Publish
        msg_img = self.bridge.cv2_to_imgmsg(warped, encoding="bgr8")
        self.pub_image.publish(msg_img)
        msg_wall = Int32MultiArray(data=self.wall_circles.flatten().tolist())
        self.pub_walls.publish(msg_wall)


    # # --- NEW FUNCTION: Detect red walls outside the board
    # def detect_outside_walls(self, img, depth_frame):
    #     """Detect red walls outside the board using the original camera image and return their 3D coordinates."""
    #     hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #     lower_red1, upper_red1 = np.array([0, 120, 70]), np.array([10, 255, 255])
    #     lower_red2, upper_red2 = np.array([170, 120, 70]), np.array([180, 255, 255])
    #     red_mask = cv2.bitwise_or(cv2.inRange(hsv, lower_red1, upper_red1),
    #                             cv2.inRange(hsv, lower_red2, upper_red2))

    #     contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #     outside_walls = []

    #     for cnt in contours:
    #         if cv2.contourArea(cnt) < 150:  # Filter small noise
    #             continue
    #         x, y, w, h = cv2.boundingRect(cnt)
    #         cx, cy = x + w//2, y + h//2

    #         # --- Compute 3D coordinate from depth
    #         if depth_frame:
    #             depth = depth_frame.get_distance(cx, cy)
    #             if depth > 0:
    #                 intrinsics = self.profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    #                 X = (cx - intrinsics.ppx) / intrinsics.fx * depth
    #                 Y = (cy - intrinsics.ppy) / intrinsics.fy * depth
    #                 Z = depth
    #                 outside_walls.append((cx, cy, X, Y, Z))
        
    #     return outside_walls

    # def detect_outside_walls(self, img, depth_frame, board_corners):
    #     """
    #     Detect red walls outside the board using the original camera image.
    #     Only considers red blobs **outside the detected board quadrilateral**.
    #     Returns a list of 3D coordinates.
    #     """
    #     hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #     lower_red1, upper_red1 = np.array([0, 120, 70]), np.array([10, 255, 255])
    #     lower_red2, upper_red2 = np.array([170, 120, 70]), np.array([180, 255, 255])
    #     red_mask = cv2.bitwise_or(cv2.inRange(hsv, lower_red1, upper_red1),
    #                             cv2.inRange(hsv, lower_red2, upper_red2))

    #     # --- Create mask of the board to exclude inside-board walls
    #     board_mask = np.zeros(img.shape[:2], dtype=np.uint8)
    #     pts = board_corners.astype(np.int32)
    #     cv2.fillPoly(board_mask, [pts], 255)
    #     red_mask = cv2.bitwise_and(red_mask, cv2.bitwise_not(board_mask))  # Only outside

    #     contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #     outside_walls = []

    #     for cnt in contours:
    #         if cv2.contourArea(cnt) < 150:  # Filter small noise
    #             continue
    #         x, y, w, h = cv2.boundingRect(cnt)
    #         cx, cy = x + w // 2, y + h // 2

    #         # --- Compute 3D coordinate from depth
    #         if depth_frame:
    #             depth = depth_frame.get_distance(cx, cy)
    #             if depth > 0:
    #                 intrinsics = self.profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    #                 X = (cx - intrinsics.ppx) / intrinsics.fx * depth
    #                 Y = (cy - intrinsics.ppy) / intrinsics.fy * depth
    #                 Z = depth
    #                 outside_walls.append((cx, cy, X, Y, Z))
        
    #     return outside_walls

    def detect_outside_walls(self, img, depth_frame, board_corners):
        """
        Detect red walls outside the board and visualize them like inside walls.
        Returns a list of tuples: (cx, cy, X, Y, Z)
        """
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_red1, upper_red1 = np.array([0, 120, 70]), np.array([10, 255, 255])
        lower_red2, upper_red2 = np.array([170, 120, 70]), np.array([180, 255, 255])
        red_mask = cv2.bitwise_or(cv2.inRange(hsv, lower_red1, upper_red1),
                                cv2.inRange(hsv, lower_red2, upper_red2))

        # Mask out inside of the board
        board_mask = np.zeros(img.shape[:2], dtype=np.uint8)
        pts = board_corners.astype(np.int32)
        cv2.fillPoly(board_mask, [pts], 255)
        red_mask = cv2.bitwise_and(red_mask, cv2.bitwise_not(board_mask))  # Only outside

        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        outside_walls = []

        ext = 25  # line half-length for orientation visualization

        for cnt in contours:
            if cv2.contourArea(cnt) < 150:  # filter noise
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            cx, cy = x + w // 2, y + h // 2

            # Visuals: bounding box and asterisk
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.drawMarker(img, (cx, cy), (0, 255, 255), markerType=cv2.MARKER_STAR, markerSize=10, thickness=1)

            # Orientation line (horizontal or vertical)
            is_horizontal = w > h
            color = (0, 255, 0)
            if is_horizontal:
                cv2.line(img, (cx - ext, cy), (cx + ext, cy), color, 2)
            else:
                cv2.line(img, (cx, cy - ext), (cx, cy + ext), color, 2)

            # 3D coordinates
            X = Y = Z = 0
            if depth_frame:
                depth = depth_frame.get_distance(cx, cy)
                if depth > 0:
                    intrinsics = self.profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
                    X = (cx - intrinsics.ppx) / intrinsics.fx * depth
                    Y = (cy - intrinsics.ppy) / intrinsics.fy * depth
                    Z = depth

            outside_walls.append((cx, cy, X, Y, Z))
            self.get_logger().info(f"Outside Wall 3D (cm): x={X*100:.2f}, y={Y*100:.2f}, z={Z*100:.2f}")

        return outside_walls
        
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