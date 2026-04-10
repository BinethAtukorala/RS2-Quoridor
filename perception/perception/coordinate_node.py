#!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32MultiArray
# import pyrealsense2 as rs
# import numpy as np
# import cv2
# import sys
# import os

# class CoordinateNode(Node):

#     class RealsenseCamera:
#         def __init__(self, bag_file=None):
#             self.width, self.height, self.fps = 1280, 720, 30
#             self.pipeline = rs.pipeline()
#             config = rs.config()
#             if bag_file:
#                 config.enable_device_from_file(bag_file, repeat_playback=True)
#             else:
#                 config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
#                 config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
#             profile = self.pipeline.start(config)
#             self.intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()

#         def get_frames(self):
#             frames = self.pipeline.wait_for_frames()
#             depth_frame = frames.get_depth_frame()
#             color_frame = frames.get_color_frame()
#             if not depth_frame or not color_frame: return None, None, None
#             return np.asanyarray(color_frame.get_data()), np.asanyarray(depth_frame.get_data()), depth_frame

#     def __init__(self):
#         super().__init__('board_3d_detector_node')
#         self.declare_parameter("bag_file", "")
#         bag_file = self.get_parameter("bag_file").get_parameter_value().string_value
#         self.camera = self.RealsenseCamera(bag_file if bag_file != "" else None)

#         self.circle_file = os.path.expanduser("~/rs2_ws/src/perception/circle_coords.txt")
#         self.grid_file = os.path.expanduser("~/rs2_ws/src/perception/grid_coords.txt")

#         self.pub_grid_array = self.create_publisher(Float32MultiArray, '/quoridor/grid_3d_array', 10)
#         self.pub_wall_array = self.create_publisher(Float32MultiArray, '/quoridor/wall_3d_array', 10)

#         self.timer = self.create_timer(0.1, self.timer_callback)
#         self.get_logger().info("Waiting for full board detection (25 Grids, 16 Walls)...")

#     def draw_asterisk_star(self, img, center, size, color, thickness=1):
#         cx, cy = center
#         for i in range(3):
#             angle = i * (np.pi / 3)
#             dx, dy = int(size * np.cos(angle)), int(size * np.sin(angle))
#             cv2.line(img, (cx - dx, cy - dy), (cx + dx, cy + dy), color, thickness)

#     def detect_board_corners(self, image):
#         gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
#         blur = cv2.GaussianBlur(gray, (5,5), 0)
#         edges = cv2.Canny(blur, 50, 150)
#         contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#         if not contours: return None
#         candidates = [cv2.approxPolyDP(c, 0.02*cv2.arcLength(c, True), True) for c in contours if cv2.contourArea(c) > 20000]
#         if not candidates: return None
#         largest = max(candidates, key=cv2.contourArea).reshape(4,2)
#         s, diff = largest.sum(axis=1), np.diff(largest, axis=1)
#         ordered = np.zeros((4,2), dtype=np.float32)
#         ordered[0], ordered[2] = largest[np.argmin(s)], largest[np.argmax(s)]
#         ordered[1], ordered[3] = largest[np.argmin(diff)], largest[np.argmax(diff)]
#         return ordered

#     def detect_white_elements(self, img):
#         hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
#         mask = cv2.inRange(hsv, np.array([0, 0, 140]), np.array([180, 100, 255]))
#         mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3,3), np.uint8))
#         contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#         detected = []
#         for cnt in contours:
#             area = cv2.contourArea(cnt)
#             if 200 < area < 12000:
#                 x, y, w, h = cv2.boundingRect(cnt)
#                 detected.append((x + w//2, y + h//2, w, h))
#         return detected

#     def detect_wall_circles(self, img):
#         gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#         circles = cv2.HoughCircles(cv2.GaussianBlur(gray, (9,9), 2), cv2.HOUGH_GRADIENT, 1.2, 40, param1=100, param2=20, minRadius=5, maxRadius=30)
#         return [] if circles is None else np.uint16(np.around(circles[0]))

#     def pixel_to_3d(self, u, v, depth_frame):
#         try:
#             d = depth_frame.get_distance(u, v)
#             return rs.rs2_deproject_pixel_to_point(self.camera.intrinsics, [u, v], d) if d > 0 else [0.0, 0.0, 0.0]
#         except: return [0.0, 0.0, 0.0]

#     def print_layout(self, data, rows, cols, label):
#         self.get_logger().info(f"--- {label} (X, Y in cm) ---")
#         output = ""
#         for r in range(rows):
#             row_str = []
#             for c in range(cols):
#                 idx = (r * cols + c) * 3
#                 x, y = data[idx], data[idx+1]
#                 if x == 0.0 and y == 0.0:
#                     row_str.append("[  -----  ]")
#                 else:
#                     row_str.append(f"[{int(x*100):+3d}, {int(y*100):+3d}]")
#             output += "  ".join(row_str) + "\n"
#         sys.stdout.write(output + "\n")
#         sys.stdout.flush()

#     def timer_callback(self):
#         import sys
#         color, depth, depth_frame = self.camera.get_frames()
#         if color is None: return
        
#         corners = self.detect_board_corners(color)
#         if corners is None:
#             cv2.imshow("Calibration View", color)
#             cv2.waitKey(1)
#             return

#         H = cv2.findHomography(corners, np.array([[0,0],[500,0],[500,500],[0,500]], dtype=np.float32))[0]
#         H_inv = np.linalg.inv(H)
#         topdown = cv2.warpPerspective(color, H, (500,500))

#         # --- 1. PROCESS WALLS ---
#         circles = self.detect_wall_circles(topdown)
#         wall_data = [0.0] * (4 * 4 * 3)
#         det_walls = 0
        
#         for (cx_td, cy_td, r_td) in circles:
#             gx, gy = int(round((cx_td - 62.5) / 125)), int(round((cy_td - 62.5) / 125))
#             if 0 <= gx < 4 and 0 <= gy < 4:
#                 pt = H_inv @ np.array([cx_td, cy_td, 1.0])
#                 u, v = int(pt[0]/pt[2]), int(pt[1]/pt[2])
#                 p3d = self.pixel_to_3d(u, v, depth_frame)
#                 if p3d != [0.0, 0.0, 0.0]:
#                     idx = (gy * 4 + gx) * 3
#                     wall_data[idx:idx+3] = p3d
#                     with open(self.circle_file, "a") as f:
#                         f.write(f"{gx},{gy},{p3d[0]:.3f},{p3d[1]:.3f},{p3d[2]:.3f}\n")  
#                     det_walls += 1
#                     cv2.rectangle(topdown, (cx_td-r_td, cy_td-r_td), (cx_td+r_td, cy_td+r_td), (0, 255, 0), 1)
#                     self.draw_asterisk_star(topdown, (cx_td, cy_td), 10, (0, 0, 255), 2)

#         # --- 2. PROCESS GRIDS ---
#         whites = self.detect_white_elements(topdown)
#         grid_data = [0.0] * (5 * 5 * 3)
#         det_grids = 0

#         for (cx_td, cy_td, w, h) in whites:
#             gx, gy = int(round((cx_td - 50) / 100)), int(round((cy_td - 50) / 100))
#             if 0 <= gx < 5 and 0 <= gy < 5:
#                 if any(np.sqrt((cx_td-c[0])**2 + (cy_td-c[1])**2) < 20 for c in circles): continue
#                 pt = H_inv @ np.array([cx_td, cy_td, 1.0])
#                 u, v = int(pt[0]/pt[2]), int(pt[1]/pt[2])
#                 p3d = self.pixel_to_3d(u, v, depth_frame)
#                 if p3d != [0.0, 0.0, 0.0]:
#                     idx = (gy * 5 + gx) * 3
#                     grid_data[idx:idx+3] = p3d
#                     with open(self.grid_file, "a") as f:
#                         f.write(f"{gx},{gy},{p3d[0]:.3f},{p3d[1]:.3f},{p3d[2]:.3f}\n")
#                     det_grids += 1
#                     cv2.rectangle(topdown, (cx_td-w//2, cy_td-h//2), (cx_td+w//2, cy_td+h//2), (255, 0, 255), 2)
#                     self.draw_asterisk_star(topdown, (cx_td, cy_td), 12, (255, 255, 255), 2)

#         # --- SIMPLE PROGRESS LOG ---
#         self.get_logger().info(f"Progress: Grids [{det_grids}/25], Walls [{det_walls}/16]")
#         cv2.imshow("Calibration View", topdown)
#         cv2.waitKey(1)

#         # --- COMPLETION CHECK ---
#         if det_grids == 25 and det_walls == 16:
#             self.get_logger().info("SUCCESS: Full board detected. Publishing...")
            
#             # Print layouts normally (will appear right under the success message)
#             self.print_layout(grid_data, 5, 5, "FINAL GRID COORDINATES")
#             self.print_layout(wall_data, 4, 4, "FINAL WALL COORDINATES")

#             self.pub_grid_array.publish(Float32MultiArray(data=grid_data))
#             self.pub_wall_array.publish(Float32MultiArray(data=wall_data))
            
#             import time
#             time.sleep(1.0)
            
#             self.get_logger().info("Setup Complete. Node shutting down.")
#             rclpy.shutdown()

# def main(args=None):
#     rclpy.init(args=args)
#     node = CoordinateNode()
#     try:
#         rclpy.spin(node)
#     except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
#         pass
#     finally:
#         if rclpy.ok():
#             node.camera.pipeline.stop()
#             node.destroy_node()
#             rclpy.shutdown()

# if __name__ == "__main__":
#     main()

# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32MultiArray
# import pyrealsense2 as rs
# import numpy as np
# import cv2
# import sys
# import os

# class CoordinateNode(Node):

#     class RealsenseCamera:
#         def __init__(self, bag_file=None):
#             self.width, self.height, self.fps = 1280, 720, 30
#             self.pipeline = rs.pipeline()
#             config = rs.config()
#             if bag_file:
#                 config.enable_device_from_file(bag_file, repeat_playback=True)
#             else:
#                 config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
#                 config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
#             profile = self.pipeline.start(config)
#             self.intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()

#         def get_frames(self):
#             frames = self.pipeline.wait_for_frames()
#             depth_frame = frames.get_depth_frame()
#             color_frame = frames.get_color_frame()
#             if not depth_frame or not color_frame: return None, None, None
#             return np.asanyarray(color_frame.get_data()), np.asanyarray(depth_frame.get_data()), depth_frame

#     def __init__(self):
#         super().__init__('board_3d_detector_node')
#         self.declare_parameter("bag_file", "")
#         bag_file = self.get_parameter("bag_file").get_parameter_value().string_value
#         self.camera = self.RealsenseCamera(bag_file if bag_file != "" else None)

#         self.circle_file = os.path.expanduser("~/ros2_ws/src/perception/circle_coords.txt")
#         self.grid_file = os.path.expanduser("~/ros2_ws/src/perception/grid_coords.txt")

#         self.pub_grid_array = self.create_publisher(Float32MultiArray, '/quoridor/grid_3d_array', 10)
#         self.pub_wall_array = self.create_publisher(Float32MultiArray, '/quoridor/wall_3d_array', 10)

#         self.timer = self.create_timer(0.1, self.timer_callback)
#         self.get_logger().info("Waiting for full board detection (25 Grids, 16 Walls)...")

#     # def draw_asterisk_star(self, img, center, size, color, thickness=1):
#     #     cx, cy = center
#     #     for i in range(3):
#     #         angle = i * (np.pi / 3)
#     #         dx, dy = int(size * np.cos(angle)), int(size * np.sin(angle))
#     #         cv2.line(img, (cx - dx, cy - dy), (cx + dx, cy + dy), color, thickness)

#     def draw_asterisk_star(self, img, center, size, color, thickness):
#         cx, cy = center

#         # Force to normal int
#         cx = int(cx)
#         cy = int(cy)
#         size = int(size)

#         for angle in range(0, 360, 45):
#             dx = int(size * np.cos(np.deg2rad(angle)))
#             dy = int(size * np.sin(np.deg2rad(angle)))

#             cv2.line(img,
#                     (cx - dx, cy - dy),
#                     (cx + dx, cy + dy),
#                     color,
#                     thickness)
                    
#     def detect_board_corners(self, image):
#         gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
#         blur = cv2.GaussianBlur(gray, (5,5), 0)
#         edges = cv2.Canny(blur, 50, 150)
#         contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#         if not contours: return None
#         candidates = [cv2.approxPolyDP(c, 0.02*cv2.arcLength(c, True), True) for c in contours if cv2.contourArea(c) > 20000]
#         if not candidates: return None
#         largest = max(candidates, key=cv2.contourArea).reshape(4,2)
#         s, diff = largest.sum(axis=1), np.diff(largest, axis=1)
#         ordered = np.zeros((4,2), dtype=np.float32)
#         ordered[0], ordered[2] = largest[np.argmin(s)], largest[np.argmax(s)]
#         ordered[1], ordered[3] = largest[np.argmin(diff)], largest[np.argmax(diff)]
#         return ordered

#     def detect_white_elements(self, img):
#         hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
#         mask = cv2.inRange(hsv, np.array([0, 0, 140]), np.array([180, 100, 255]))
#         mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3,3), np.uint8))
#         contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#         detected = []
#         for cnt in contours:
#             area = cv2.contourArea(cnt)
#             if 200 < area < 12000:
#                 x, y, w, h = cv2.boundingRect(cnt)
#                 detected.append((x + w//2, y + h//2, w, h))
#         return detected

#     def detect_wall_circles(self, img):
#         gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#         circles = cv2.HoughCircles(cv2.GaussianBlur(gray, (9,9), 2), cv2.HOUGH_GRADIENT, 1.2, 40, param1=100, param2=20, minRadius=5, maxRadius=30)
#         return [] if circles is None else np.uint16(np.around(circles[0]))

#     def pixel_to_3d(self, u, v, depth_frame):
#         try:
#             d = depth_frame.get_distance(u, v)
#             return rs.rs2_deproject_pixel_to_point(self.camera.intrinsics, [u, v], d) if d > 0 else [0.0, 0.0, 0.0]
#         except: return [0.0, 0.0, 0.0]

#     def print_layout(self, data, rows, cols, label):
#         self.get_logger().info(f"--- {label} (X, Y in cm) ---")
#         output = ""
#         for r in range(rows):
#             row_str = []
#             for c in range(cols):
#                 idx = (r * cols + c) * 3
#                 x, y = data[idx], data[idx+1]
#                 if x == 0.0 and y == 0.0:
#                     row_str.append("[  -----  ]")
#                 else:
#                     row_str.append(f"[{int(x*100):+3d}, {int(y*100):+3d}]")
#             output += "  ".join(row_str) + "\n"
#         sys.stdout.write(output + "\n")
#         sys.stdout.flush()

#     def timer_callback(self):
#         import sys
#         color, depth, depth_frame = self.camera.get_frames()
#         if color is None: return
        
#         corners = self.detect_board_corners(color)
#         if corners is None:
#             cv2.imshow("Calibration View", color)
#             cv2.waitKey(1)
#             return

#         H = cv2.findHomography(corners, np.array([[0,0],[500,0],[500,500],[0,500]], dtype=np.float32))[0]
#         H_inv = np.linalg.inv(H)
#         topdown = cv2.warpPerspective(color, H, (500,500))

#         # --- 1. PROCESS WALLS ---
#         circles = self.detect_wall_circles(topdown)
#         wall_data = [0.0] * (4 * 4 * 3)
#         det_walls = 0
        
#         for (cx_td, cy_td, r_td) in circles:
#             gx, gy = int(round((cx_td - 62.5) / 125)), int(round((cy_td - 62.5) / 125))
#             if 0 <= gx < 4 and 0 <= gy < 4:
#                 pt = H_inv @ np.array([cx_td, cy_td, 1.0])
#                 u, v = int(pt[0]/pt[2]), int(pt[1]/pt[2])
#                 p3d = self.pixel_to_3d(u, v, depth_frame)
#                 if p3d != [0.0, 0.0, 0.0]:
#                     idx = (gy * 4 + gx) * 3
#                     wall_data[idx:idx+3] = p3d
#                     det_walls += 1
#                     cv2.rectangle(topdown, (cx_td-r_td, cy_td-r_td), (cx_td+r_td, cy_td+r_td), (0, 255, 0), 1)
#                     self.draw_asterisk_star(topdown, (cx_td, cy_td), 10, (0, 0, 255), 2)

#         # --- 2. PROCESS GRIDS ---
#         whites = self.detect_white_elements(topdown)
#         grid_data = [0.0] * (5 * 5 * 3)
#         det_grids = 0

#         for (cx_td, cy_td, w, h) in whites:
#             gx, gy = int(round((cx_td - 50) / 100)), int(round((cy_td - 50) / 100))
#             if 0 <= gx < 5 and 0 <= gy < 5:
#                 if any(np.sqrt((cx_td-c[0])**2 + (cy_td-c[1])**2) < 20 for c in circles): continue
#                 pt = H_inv @ np.array([cx_td, cy_td, 1.0])
#                 u, v = int(pt[0]/pt[2]), int(pt[1]/pt[2])
#                 p3d = self.pixel_to_3d(u, v, depth_frame)
#                 if p3d != [0.0, 0.0, 0.0]:
#                     idx = (gy * 5 + gx) * 3
#                     grid_data[idx:idx+3] = p3d
#                     det_grids += 1
#                     cv2.rectangle(topdown, (cx_td-w//2, cy_td-h//2), (cx_td+w//2, cy_td+h//2), (255, 0, 255), 2)
#                     self.draw_asterisk_star(topdown, (cx_td, cy_td), 12, (255, 255, 255), 2)

#         # --- SIMPLE PROGRESS LOG ---
#         self.get_logger().info(f"Progress: Grids [{det_grids}/25], Walls [{det_walls}/16]")
#         cv2.imshow("Calibration View", topdown)
#         cv2.waitKey(1)

#         # --- COMPLETION CHECK ---
#         if det_grids == 25 and det_walls == 16:
#             self.get_logger().info("SUCCESS: Full board detected. Saving and Publishing...")
            
#             # 1. Clear and write Walls file
#             with open(self.circle_file, "w") as f:
#                 for r in range(4):
#                     for c in range(4):
#                         idx = (r * 4 + c) * 3
#                         f.write(f"{r},{c},{wall_data[idx]:.3f},{wall_data[idx+1]:.3f},{wall_data[idx+2]:.3f}\n")

#             # 2. Clear and write Grids file
#             with open(self.grid_file, "w") as f:
#                 for r in range(5):
#                     for c in range(5):
#                         idx = (r * 5 + c) * 3
#                         f.write(f"{r},{c},{grid_data[idx]:.3f},{grid_data[idx+1]:.3f},{grid_data[idx+2]:.3f}\n")

#             # Print layouts normally
#             self.print_layout(grid_data, 5, 5, "FINAL GRID COORDINATES")
#             self.print_layout(wall_data, 4, 4, "FINAL WALL COORDINATES")

#             self.pub_grid_array.publish(Float32MultiArray(data=grid_data))
#             self.pub_wall_array.publish(Float32MultiArray(data=wall_data))
            
#             import time
#             time.sleep(1.0)
            
#             self.get_logger().info("Setup Complete. Node shutting down.")
#             rclpy.shutdown()

# def main(args=None):
#     rclpy.init(args=args)
#     node = CoordinateNode()
#     try:
#         rclpy.spin(node)
#     except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
#         pass
#     finally:
#         if rclpy.ok():
#             node.camera.pipeline.stop()
#             node.destroy_node()
#             rclpy.shutdown()

# if __name__ == "__main__":
#     main()


#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import pyrealsense2 as rs
import numpy as np
import cv2
import sys
import os

class CoordinateNode(Node):

    class RealsenseCamera:
        def __init__(self, bag_file=None):
            self.width, self.height, self.fps = 640, 480, 15
            self.pipeline = rs.pipeline()
            config = rs.config()
            self.align = rs.align(rs.stream.color)
            if bag_file:
                config.enable_device_from_file(bag_file, repeat_playback=True)
            else:
                config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
                config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
            profile = self.pipeline.start(config)
            # self.intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
            self.intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

        def get_frames(self):
            # frames = self.pipeline.wait_for_frames()
            # depth_frame = frames.get_depth_frame()
            # color_frame = frames.get_color_frame()

            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)

            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not depth_frame or not color_frame:
                return None, None, None
            return np.asanyarray(color_frame.get_data()), np.asanyarray(depth_frame.get_data()), depth_frame

    def __init__(self):
        super().__init__('board_3d_detector_node')
        self.declare_parameter("bag_file", "")
        bag_file = self.get_parameter("bag_file").get_parameter_value().string_value
        self.camera = self.RealsenseCamera(bag_file if bag_file != "" else None)

        self.circle_file = os.path.expanduser("/rs2_ws/src/perception/circle_coords.txt")
        self.grid_file = os.path.expanduser("/rs2_ws/src/perception/grid_coords.txt")

        self.pub_grid_array = self.create_publisher(Float32MultiArray, '/perception/grids_3d', 10)
        self.pub_wall_array = self.create_publisher(Float32MultiArray, '/perception/circles_3d', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Waiting for full board detection (25 Grids, 16 Walls)...")

    def draw_asterisk_star(self, img, center, size, color, thickness):
        h, w = img.shape[:2]
        cx, cy = int(center[0]), int(center[1])
        size = int(size)

        for angle in range(0, 360, 45):
            dx = int(size * np.cos(np.deg2rad(angle)))
            dy = int(size * np.sin(np.deg2rad(angle)))

            x1 = max(0, min(w-1, cx - dx))
            y1 = max(0, min(h-1, cy - dy))
            x2 = max(0, min(w-1, cx + dx))
            y2 = max(0, min(h-1, cy + dy))

            cv2.line(img, (x1, y1), (x2, y2), color, thickness)

    def detect_board_corners(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)
        edges = cv2.Canny(blur, 50, 150)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours: return None
        candidates = [cv2.approxPolyDP(c, 0.02*cv2.arcLength(c, True), True)
                      for c in contours if cv2.contourArea(c) > 20000]
        if not candidates: return None
        largest = max(candidates, key=cv2.contourArea).reshape(4,2)
        s, diff = largest.sum(axis=1), np.diff(largest, axis=1)
        ordered = np.zeros((4,2), dtype=np.float32)
        ordered[0], ordered[2] = largest[np.argmin(s)], largest[np.argmax(s)]
        ordered[1], ordered[3] = largest[np.argmin(diff)], largest[np.argmax(diff)]
        return ordered

    def detect_white_elements(self, img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([0,0,140]), np.array([180,100,255]))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3,3), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detected = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if 200 < area < 12000:
                x, y, w, h = cv2.boundingRect(cnt)
                detected.append((float(x + w//2), float(y + h//2), w, h))
        return detected

    def detect_wall_circles(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(
            cv2.GaussianBlur(gray, (9,9), 2),
            cv2.HOUGH_GRADIENT, 1.2, 40, param1=100, param2=20, minRadius=5, maxRadius=30
        )
        if circles is None:
            return []
        return [(float(x), float(y), float(r)) for x, y, r in circles[0]]

    def pixel_to_3d(self, u, v, depth_frame):
        try:
            d = depth_frame.get_distance(u, v)
            return rs.rs2_deproject_pixel_to_point(self.camera.intrinsics, [u, v], d) if d > 0 else [0.0,0.0,0.0]
        except:
            return [0.0,0.0,0.0]

    def print_layout(self, data, rows, cols, label):
        self.get_logger().info(f"--- {label} (X, Y in cm) ---")
        output = ""
        for r in range(rows):
            row_str = []
            for c in range(cols):
                idx = (r * cols + c) * 3
                x, y = data[idx], data[idx+1]
                if x == 0.0 and y == 0.0:
                    row_str.append("[  -----  ]")
                else:
                    row_str.append(f"[{int(x*100):+3d}, {int(y*100):+3d}]")
            output += "  ".join(row_str) + "\n"
        sys.stdout.write(output + "\n")
        sys.stdout.flush()

    def timer_callback(self):
        color, depth, depth_frame = self.camera.get_frames()
        if color is None:
            return

        corners = self.detect_board_corners(color)
        if corners is None:
            cv2.imshow("Calibration View", color)
            cv2.waitKey(1)
            return

        H = cv2.findHomography(corners, np.array([[0,0],[500,0],[500,500],[0,500]], dtype=np.float32))[0]
        H_inv = np.linalg.inv(H)
        topdown = cv2.warpPerspective(color, H, (500,500)).astype(np.uint8)

        # --- PROCESS WALLS ---
        circles = self.detect_wall_circles(topdown)
        wall_data = [0.0]*(4*4*3)
        det_walls = 0
        h_img, w_img = color.shape[:2]

        for (cx_td, cy_td, r_td) in circles:
            cx_td = float(cx_td)
            cy_td = float(cy_td)
            r_td = float(r_td)
            gx, gy = int(round((cx_td - 62.5)/125)), int(round((cy_td - 62.5)/125))
            if 0 <= gx < 4 and 0 <= gy < 4:
                pt = H_inv @ np.array([cx_td, cy_td, 1.0])
                if abs(pt[2]) < 1e-6: continue
                u = max(0, min(w_img-1, int(pt[0]/pt[2])))
                v = max(0, min(h_img-1, int(pt[1]/pt[2])))
                p3d = self.pixel_to_3d(u, v, depth_frame)
                if p3d != [0.0,0.0,0.0]:
                    idx = (gy*4+gx)*3
                    wall_data[idx:idx+3] = p3d
                    det_walls += 1
                    cv2.rectangle(topdown, (int(cx_td-r_td), int(cy_td-r_td)), (int(cx_td+r_td), int(cy_td+r_td)), (0,255,0), 1)
                    self.draw_asterisk_star(topdown, (cx_td, cy_td), 10, (0,0,255), 2)

        # --- PROCESS GRIDS ---
        whites = self.detect_white_elements(topdown)
        grid_data = [0.0]*(5*5*3)
        det_grids = 0

        for (cx_td, cy_td, w, h) in whites:
            cx_td = float(cx_td)
            cy_td = float(cy_td)
            gx, gy = int(round((cx_td - 50)/100)), int(round((cy_td - 50)/100))
            if 0 <= gx < 5 and 0 <= gy < 5:
                if any(np.hypot(cx_td-c[0], cy_td-c[1])<20 for c in circles): continue
                pt = H_inv @ np.array([cx_td, cy_td, 1.0])
                if abs(pt[2]) < 1e-6: continue
                u = max(0, min(w_img-1, int(pt[0]/pt[2])))
                v = max(0, min(h_img-1, int(pt[1]/pt[2])))
                p3d = self.pixel_to_3d(u, v, depth_frame)
                if p3d != [0.0,0.0,0.0]:
                    idx = (gy*5+gx)*3
                    grid_data[idx:idx+3] = p3d
                    det_grids += 1
                    cv2.rectangle(topdown, (int(cx_td-w//2), int(cy_td-h//2)), (int(cx_td+w//2), int(cy_td+h//2)), (255,0,255), 2)
                    self.draw_asterisk_star(topdown, (cx_td, cy_td), 12, (255,255,255), 2)

        self.get_logger().info(f"Progress: Grids [{det_grids}/25], Walls [{det_walls}/16]")
        cv2.imshow("Calibration View", topdown)
        cv2.waitKey(1)

        if det_grids==25 and det_walls==16:
            self.get_logger().info("SUCCESS: Full board detected. Saving and Publishing...")

            # Write walls
            with open(self.circle_file,"w") as f:
                for r in range(4):
                    for c in range(4):
                        idx = (r*4+c)*3
                        f.write(f"{r},{c},{wall_data[idx]:.3f},{wall_data[idx+1]:.3f},{wall_data[idx+2]:.3f}\n")

            # Write grids
            with open(self.grid_file,"w") as f:
                for r in range(5):
                    for c in range(5):
                        idx = (r*5+c)*3
                        f.write(f"{r},{c},{grid_data[idx]:.3f},{grid_data[idx+1]:.3f},{grid_data[idx+2]:.3f}\n")

            self.print_layout(grid_data,5,5,"FINAL GRID COORDINATES")
            self.print_layout(wall_data,4,4,"FINAL WALL COORDINATES")

            self.pub_grid_array.publish(Float32MultiArray(data=grid_data))
            self.pub_wall_array.publish(Float32MultiArray(data=wall_data))

            import time
            time.sleep(1.0)
            self.get_logger().info("Setup Complete. Node shutting down.")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = CoordinateNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.camera.pipeline.stop()
            node.destroy_node()
            rclpy.shutdown()

if __name__=="__main__":
    main()