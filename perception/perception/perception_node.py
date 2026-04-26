# # perception/perception/perception_node.py

# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from std_msgs.msg import Int32MultiArray, Float32MultiArray
# import numpy as np
# import cv2
# from cv_bridge import CvBridge
# import os
# import json

# class PerceptionNode(Node):
#     def __init__(self):
#         super().__init__('perception_node')
#         self.bridge = CvBridge()

#         # --- Publishers ---
#         self.pub_topdown = self.create_publisher(Image, '/perception/topdown_view', 10)
#         self.pub_board_state = self.create_publisher(Int32MultiArray, '/perception/board_state', 10)
#         self.pub_wall_state = self.create_publisher(Int32MultiArray, '/perception/wall_state', 10)
#         self.pub_pawns_3d = self.create_publisher(Float32MultiArray, '/perception/pawns_3d', 10)
#         self.pub_walls_inside_3d = self.create_publisher(Float32MultiArray, '/perception/walls_inside_3d', 10)
#         self.pub_walls_outside_3d = self.create_publisher(Float32MultiArray, '/perception/walls_outside_3d', 10)
#         self.pub_corners = self.create_publisher(Float32MultiArray, '/perception/board_corners', 10)

#         # --- Subscriptions ---
#         self.sub_color = self.create_subscription(Image, '/camera/color', self.color_callback, 10)
#         self.sub_depth = self.create_subscription(Image, '/camera/depth', self.depth_callback, 10)

#         self.latest_color = None
#         self.latest_depth = None

#         # --- Configuration & Files ---
#         self.board_size = 500
#         self.cell_size = 100
#         self.pawn_coords_file = os.path.expanduser("~/rs2_ws/src/perception/pawn_coords.txt")
#         self.wall_coords_file = os.path.expanduser("~/rs2_ws/src/perception/wall_coords.txt")
#         self.intrinsics_file = os.path.expanduser("~/rs2_ws/src/perception/camera_intrinsics.json")

#         self.pawn_lookup = {}
#         self.wall_lookup = {}
#         self.load_coordinates()
#         self.load_intrinsics()

#         # State Storage
#         self.grid_state = np.zeros((5, 5), dtype=int)
#         self.wall_state = np.zeros((4, 4), dtype=int)

#         # GUI
#         cv2.namedWindow("Perception View", cv2.WINDOW_AUTOSIZE)
        
#         self.timer = self.create_timer(0.1, self.timer_callback)
#         self.get_logger().info("Unified Perception Node Started")

#     def load_coordinates(self):
#         # Load Pawn Grid Coords
#         try:
#             with open(self.pawn_coords_file, "r") as f:
#                 for line in f:
#                     p = line.strip().split(",")
#                     if len(p) == 5: self.pawn_lookup[(int(p[0]), int(p[1]))] = [float(p[2]), float(p[3]), float(p[4])]
#             self.get_logger().info(f"Loaded {len(self.pawn_lookup)} pawn coordinates.")
#         except: self.get_logger().warn("Pawn coords file not found.")

#         # Load Wall Circle Coords
#         try:
#             with open(self.wall_coords_file, "r") as f:
#                 for line in f:
#                     p = line.strip().split(",")
#                     if len(p) == 5: self.wall_lookup[(int(p[0]), int(p[1]))] = [float(p[2]), float(p[3]), float(p[4])]
#             self.get_logger().info(f"Loaded {len(self.wall_lookup)} wall coordinates.")
#         except: self.get_logger().warn("Wall coords file not found.")

#     def load_intrinsics(self):
#         with open(self.intrinsics_file, "r") as f:
#             intr = json.load(f)
#             self.fx, self.fy = intr["fx"], intr["fy"]
#             self.ppx, self.ppy = intr["ppx"], intr["ppy"]

#     def color_callback(self, msg): self.latest_color = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
#     def depth_callback(self, msg): self.latest_depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')

#     def get_ordered_corners(self, img):
#         gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#         blur = cv2.GaussianBlur(gray, (5, 5), 0)
#         edges = cv2.Canny(blur, 50, 150)
#         contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#         if not contours: return None
        
#         board_contours = []
#         for cnt in contours:
#             if 20000 < cv2.contourArea(cnt) < (img.shape[0]*img.shape[1]):
#                 approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
#                 if len(approx) == 4: board_contours.append(approx)
        
#         if not board_contours: return None
#         pts = max(board_contours, key=cv2.contourArea).reshape(4, 2)
        
#         rect = np.zeros((4, 2), dtype=np.float32)
#         s = pts.sum(axis=1)
#         diff = np.diff(pts, axis=1)
#         rect[0], rect[2] = pts[np.argmin(s)], pts[np.argmax(s)]
#         rect[1], rect[3] = pts[np.argmin(diff)], pts[np.argmax(diff)]
#         return rect

#     def process_topdown(self, topdown, depth_frame):
#         # 1. Pawn Detection (Black Objects)
#         gray = cv2.cvtColor(topdown, cv2.COLOR_BGR2GRAY)
#         _, black_mask = cv2.threshold(gray, 70, 255, cv2.THRESH_BINARY_INV)
#         pawns_list_3d = []
#         self.grid_state[:] = 0

#         margin = 30
#         for r in range(5):
#             for c in range(5):
#                 x1, y1 = c*100 + margin, r*100 + margin
#                 x2, y2 = (c+1)*100 - margin, (r+1)*100 - margin
#                 cell_roi = black_mask[y1:y2, x1:x2]
#                 if cv2.countNonZero(cell_roi) / ((100-2*margin)**2) > 0.05:
#                     self.grid_state[r, c] = 1
#                     cv2.rectangle(topdown, (x1, y1), (x2, y2), (255, 0, 0), 2) # Blue for Pawns
#                     if (r, c) in self.pawn_lookup:
#                         p3d = self.pawn_lookup[(r, c)]
#                         pawns_list_3d.extend([float(r), float(c), p3d[0], p3d[1], p3d[2]])

#         # 2. Wall Detection (Red Objects)
#         hsv = cv2.cvtColor(topdown, cv2.COLOR_BGR2HSV)
#         lower_red1, upper_red1 = np.array([0, 120, 70]), np.array([10, 255, 255])
#         lower_red2, upper_red2 = np.array([170, 120, 70]), np.array([180, 255, 255])
#         red_mask = cv2.bitwise_or(cv2.inRange(hsv, lower_red1, upper_red1), cv2.inRange(hsv, lower_red2, upper_red2))
        
#         self.wall_state[:] = 0
#         walls_in_3d = []
#         red_cnts, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
#         for cnt in red_cnts:
#             if cv2.contourArea(cnt) < 100: continue
#             rx, ry, rw, rh = cv2.boundingRect(cnt)
#             cx, cy = rx + rw//2, ry + rh//2
#             is_horz = rw > rh
            
#             c_idx = int(round(cx / 100)) - 1
#             r_idx = int(round(cy / 100)) - 1
            
#             if 0 <= r_idx < 4 and 0 <= c_idx < 4:
#                 self.wall_state[r_idx, c_idx] = 2 if is_horz else 1
#                 target_x, target_y = (c_idx+1)*100, (r_idx+1)*100
#                 cv2.rectangle(topdown, (rx, ry), (rx+rw, ry+rh), (0, 0, 255), 2) # Red for Walls
#                 # Orientation Line
#                 ext = 25
#                 if is_horz: cv2.line(topdown, (target_x-ext, target_y), (target_x+ext, target_y), (0, 255, 0), 2)
#                 else: cv2.line(topdown, (target_x, target_y-ext), (target_x, target_y+ext), (0, 255, 0), 2)
                
#                 if (c_idx, r_idx) in self.wall_lookup:
#                     w3d = self.wall_lookup[(c_idx, r_idx)]
#                     walls_in_3d.extend([float(r_idx), float(c_idx), w3d[0], w3d[1], w3d[2]])

#         # 3. Draw Grid Lines
#         for i in range(6):
#             cv2.line(topdown, (i*100, 0), (i*100, 500), (0, 255, 0), 1)
#             cv2.line(topdown, (0, i*100), (500, i*100), (0, 255, 0), 1)

#         return topdown, pawns_list_3d, walls_in_3d

#     def detect_outside_walls(self, img, depth_frame, corners):
#         hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
#         lower_red = cv2.bitwise_or(cv2.inRange(hsv, np.array([0,120,70]), np.array([10,255,255])),
#                                    cv2.inRange(hsv, np.array([170,120,70]), np.array([180,255,255])))
        
#         board_mask = np.zeros(img.shape[:2], dtype=np.uint8)
#         cv2.fillPoly(board_mask, [corners.astype(np.int32)], 255)
#         outside_mask = cv2.bitwise_and(lower_red, cv2.bitwise_not(board_mask))
        
#         cnts, _ = cv2.findContours(outside_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#         outside_3d = []
#         for cnt in cnts:
#             if cv2.contourArea(cnt) < 150: continue
#             x, y, w, h = cv2.boundingRect(cnt)
#             cx, cy = x + w//2, y + h//2
#             cv2.rectangle(img, (x, y), (x+w, y+h), (0, 0, 255), 2)
            
#             if depth_frame is not None:
#                 d = depth_frame[cy, cx] * 0.001
#                 if d > 0:
#                     X = (cx - self.ppx) / self.fx * d
#                     Y = (cy - self.ppy) / self.fy * d
#                     outside_3d.extend([float(X), float(Y), float(d)])
#         return outside_3d

#     def timer_callback(self):
#         if self.latest_color is None or self.latest_depth is None: return

#         raw_img = self.latest_color.copy()
#         depth = self.latest_depth.copy()
#         corners = self.get_ordered_corners(raw_img)

#         if corners is None:
#             cv2.imshow("Perception View", raw_img)
#             cv2.waitKey(1)
#             return

#         # Homography
#         dst = np.array([[0,0], [500,0], [500,500], [0,500]], dtype=np.float32)
#         H, _ = cv2.findHomography(corners, dst)
#         topdown_raw = cv2.warpPerspective(raw_img, H, (500, 500))

#         # Main Processing
#         topdown_viz, pawns_3d, walls_in_3d = self.process_topdown(topdown_raw, depth)
#         walls_out_3d = self.detect_outside_walls(raw_img, depth, corners)

#         # Combine for GUI
#         for p in corners: cv2.circle(raw_img, (int(p[0]), int(p[1])), 6, (0, 255, 0), -1)
        
#         # Display
#         cv2.imshow("Perception View", raw_img)
#         cv2.imshow("Top-Down Unified", topdown_viz)
#         cv2.waitKey(1)

#         # Publish Messages
#         self.pub_topdown.publish(self.bridge.cv2_to_imgmsg(topdown_viz, "bgr8"))
#         self.pub_board_state.publish(Int32MultiArray(data=self.grid_state.flatten().tolist()))
#         self.pub_wall_state.publish(Int32MultiArray(data=self.wall_state.flatten().tolist()))
#         self.pub_pawns_3d.publish(Float32MultiArray(data=pawns_3d))
#         self.pub_walls_inside_3d.publish(Float32MultiArray(data=walls_in_3d))
#         self.pub_walls_outside_3d.publish(Float32MultiArray(data=walls_out_3d))
#         self.pub_corners.publish(Float32MultiArray(data=corners.flatten().tolist()))

# def main(args=None):
#     rclpy.init(args=args)
#     node = PerceptionNode()
#     try: rclpy.spin(node)
#     except KeyboardInterrupt: pass
#     finally:
#         cv2.destroyAllWindows()
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from std_msgs.msg import Int32MultiArray, Float32MultiArray
# import numpy as np
# import cv2
# from cv_bridge import CvBridge
# import os
# import json

# class PerceptionNode(Node):
#     def __init__(self):
#         super().__init__('perception_node')
#         self.bridge = CvBridge()

#         # --- Publishers ---
#         self.pub_topdown = self.create_publisher(Image, '/perception/topdown_view', 10)
#         self.pub_board_state = self.create_publisher(Int32MultiArray, '/perception/board_state', 10)
#         self.pub_wall_state = self.create_publisher(Int32MultiArray, '/perception/wall_state', 10)
#         self.pub_pawns_3d = self.create_publisher(Float32MultiArray, '/perception/pawns_3d', 10)
#         self.pub_walls_inside_3d = self.create_publisher(Float32MultiArray, '/perception/walls_inside_3d', 10)
#         self.pub_walls_outside_3d = self.create_publisher(Float32MultiArray, '/perception/walls_outside_3d', 10)
#         self.pub_corners = self.create_publisher(Float32MultiArray, '/perception/board_corners', 10)

#         # --- Subscriptions ---
#         self.sub_color = self.create_subscription(Image, '/camera/color', self.color_callback, 10)
#         self.sub_depth = self.create_subscription(Image, '/camera/depth', self.depth_callback, 10)

#         self.latest_color = None
#         self.latest_depth = None

#         # --- Configuration & Files ---
#         self.board_size = 500
#         self.cell_size = 100
#         self.pawn_coords_file = os.path.expanduser("~/rs2_ws/src/perception/pawn_coords.txt")
#         self.wall_coords_file = os.path.expanduser("~/rs2_ws/src/perception/wall_coords.txt")
#         self.intrinsics_file = os.path.expanduser("~/rs2_ws/src/perception/camera_intrinsics.json")

#         self.pawn_lookup = {}
#         self.wall_lookup = {}
#         self.load_coordinates()
#         self.load_intrinsics()

#         # State Storage
#         self.grid_state = np.zeros((5, 5), dtype=int)
#         self.wall_state = np.zeros((4, 4), dtype=int)

#         # GUI
#         cv2.namedWindow("Camera View", cv2.WINDOW_AUTOSIZE)
#         cv2.namedWindow("Top-Down Unified", cv2.WINDOW_AUTOSIZE)
        
#         self.timer = self.create_timer(0.1, self.timer_callback)
#         self.get_logger().info("Unified Perception Node Started")

#     def load_coordinates(self):
#         try:
#             with open(self.pawn_coords_file, "r") as f:
#                 for line in f:
#                     p = line.strip().split(",")
#                     if len(p) == 5: self.pawn_lookup[(int(p[0]), int(p[1]))] = [float(p[2]), float(p[3]), float(p[4])]
#             self.get_logger().info(f"Loaded {len(self.pawn_lookup)} pawn coordinates.")
#         except: self.get_logger().warn("Pawn coords file not found.")

#         try:
#             with open(self.wall_coords_file, "r") as f:
#                 for line in f:
#                     p = line.strip().split(",")
#                     if len(p) == 5: self.wall_lookup[(int(p[0]), int(p[1]))] = [float(p[2]), float(p[3]), float(p[4])]
#             self.get_logger().info(f"Loaded {len(self.wall_lookup)} wall coordinates.")
#         except: self.get_logger().warn("Wall coords file not found.")

#     def load_intrinsics(self):
#         with open(self.intrinsics_file, "r") as f:
#             intr = json.load(f)
#             self.fx, self.fy = intr["fx"], intr["fy"]
#             self.ppx, self.ppy = intr["ppx"], intr["ppy"]

#     def color_callback(self, msg): self.latest_color = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
#     def depth_callback(self, msg): self.latest_depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')

#     def get_ordered_corners(self, img):
#         gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#         blur = cv2.GaussianBlur(gray, (5, 5), 0)
#         edges = cv2.Canny(blur, 50, 150)
#         contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#         if not contours: return None
#         board_contours = [cnt for cnt in contours if 20000 < cv2.contourArea(cnt) < (img.shape[0]*img.shape[1])]
#         if not board_contours: return None
#         largest = max(board_contours, key=cv2.contourArea)
#         approx = cv2.approxPolyDP(largest, 0.02 * cv2.arcLength(largest, True), True)
#         if len(approx) != 4: return None
#         pts = approx.reshape(4, 2)
#         rect = np.zeros((4, 2), dtype=np.float32)
#         s = pts.sum(axis=1); diff = np.diff(pts, axis=1)
#         rect[0], rect[2] = pts[np.argmin(s)], pts[np.argmax(s)]
#         rect[1], rect[3] = pts[np.argmin(diff)], pts[np.argmax(diff)]
#         return rect

#     def process_topdown(self, topdown):
#         # --- PAWN DETECTION (BLACK) ---
#         gray = cv2.cvtColor(topdown, cv2.COLOR_BGR2GRAY)
#         _, black_mask = cv2.threshold(gray, 70, 255, cv2.THRESH_BINARY_INV)
#         pawns_list_3d = []
#         self.grid_state[:] = 0
#         margin = 30

#         for r in range(5):
#             for c in range(5):
#                 x1, y1, x2, y2 = c*100+margin, r*100+margin, (c+1)*100-margin, (r+1)*100-margin
#                 cell_roi = black_mask[y1:y2, x1:x2]
#                 if cv2.countNonZero(cell_roi) / ((100-2*margin)**2) > 0.05:
#                     self.grid_state[r, c] = 1
#                     cnts, _ = cv2.findContours(cell_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#                     if cnts:
#                         bx, by, bw, bh = cv2.boundingRect(max(cnts, key=cv2.contourArea))
#                         cv2.rectangle(topdown, (x1+bx, y1+by), (x1+bx+bw, y1+by+bh), (0, 0, 255), 2)
#                     if (r, c) in self.pawn_lookup:
#                         p3d = self.pawn_lookup[(r, c)]
#                         pawns_list_3d.extend([float(r), float(c), p3d[0], p3d[1], p3d[2]])
#                         self.get_logger().info(f"Pawn at [{r},{c}] 3D: x={p3d[0]:.3f}, y={p3d[1]:.3f}, z={p3d[2]:.3f}")

#         # --- WALL DETECTION (RED) ---
#         hsv = cv2.cvtColor(topdown, cv2.COLOR_BGR2HSV)
#         lower_red = cv2.bitwise_or(cv2.inRange(hsv, np.array([0,120,70]), np.array([10,255,255])),
#                                    cv2.inRange(hsv, np.array([170,120,70]), np.array([180,255,255])))
#         self.wall_state[:] = 0
#         walls_in_3d = []
#         red_cnts, _ = cv2.findContours(lower_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
#         for cnt in red_cnts:
#             if cv2.contourArea(cnt) < 100: continue
#             rx, ry, rw, rh = cv2.boundingRect(cnt)
#             cx_blob, cy_blob = rx + rw//2, ry + rh//2
#             is_horz = rw > rh
            
#             # Draw Marker at blob centroid (STAR)
#             cv2.drawMarker(topdown, (cx_blob, cy_blob), (0, 255, 255), markerType=cv2.MARKER_STAR, markerSize=10, thickness=1)
#             cv2.rectangle(topdown, (rx, ry), (rx+rw, ry+rh), (0, 0, 255), 2)

#             c_idx, r_idx = int(round(cx_blob / 100)) - 1, int(round(cy_blob / 100)) - 1
#             if 0 <= r_idx < 4 and 0 <= c_idx < 4:
#                 self.wall_state[r_idx, c_idx] = 2 if is_horz else 1
#                 cx_circle, cy_circle = (c_idx+1)*100, (r_idx+1)*100
                
#                 # Draw Marker at grid circle centroid (STAR)
#                 cv2.drawMarker(topdown, (cx_circle, cy_circle), (255, 255, 0), markerType=cv2.MARKER_STAR, markerSize=12, thickness=2)
                
#                 # Orientation line
#                 ext = 25
#                 if is_horz: cv2.line(topdown, (cx_circle-ext, cy_circle), (cx_circle+ext, cy_circle), (0, 255, 0), 2)
#                 else: cv2.line(topdown, (cx_circle, cy_circle-ext), (cx_circle, cy_circle+ext), (0, 255, 0), 2)
                
#                 if (c_idx, r_idx) in self.wall_lookup:
#                     w3d = self.wall_lookup[(c_idx, r_idx)]
#                     walls_in_3d.extend([float(r_idx), float(c_idx), w3d[0], w3d[1], w3d[2]])
#                     self.get_logger().info(f"Wall at Circle [{c_idx},{r_idx}] 3D (cm): x={w3d[0]*100:.2f}, y={w3d[1]*100:.2f}, z={w3d[2]*100:.2f}")

#         # Logging states to terminal
#         board_str = "\n".join(" ".join(str(c) for c in row) for row in self.grid_state)
#         self.get_logger().info(f"\nDigital Board:\n{board_str}")
#         wall_labels = {0: "Empty", 1: "Vertical", 2: "Horizontal"}
#         self.get_logger().info("\n--- Wall Orientation State ---")
#         for row in self.wall_state:
#             self.get_logger().info(str([wall_labels[val] for val in row]))

#         # Grid Lines
#         for i in range(6):
#             cv2.line(topdown, (i*100, 0), (i*100, 500), (0, 255, 0), 1)
#             cv2.line(topdown, (0, i*100), (500, i*100), (0, 255, 0), 1)

#         return topdown, pawns_list_3d, walls_in_3d

#     def detect_outside_walls(self, img, depth_frame, corners):
#         hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
#         lower_red = cv2.bitwise_or(cv2.inRange(hsv, np.array([0,120,70]), np.array([10,255,255])),
#                                    cv2.inRange(hsv, np.array([170,120,70]), np.array([180,255,255])))
#         board_mask = np.zeros(img.shape[:2], dtype=np.uint8)
#         cv2.fillPoly(board_mask, [corners.astype(np.int32)], 255)
#         outside_mask = cv2.bitwise_and(lower_red, cv2.bitwise_not(board_mask))
        
#         cnts, _ = cv2.findContours(outside_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#         outside_3d = []
#         for cnt in cnts:
#             if cv2.contourArea(cnt) < 150: continue
#             x, y, w, h = cv2.boundingRect(cnt)
#             cx, cy = x + w//2, y + h//2
#             cv2.rectangle(img, (x, y), (x+w, y+h), (0, 0, 255), 2)
#             cv2.drawMarker(img, (cx, cy), (0, 255, 255), markerType=cv2.MARKER_STAR, markerSize=10, thickness=1)
            
#             if depth_frame is not None:
#                 d = depth_frame[cy, cx] * 0.001
#                 if d > 0:
#                     X, Y = (cx - self.ppx) / self.fx * d, (cy - self.ppy) / self.fy * d
#                     outside_3d.extend([float(X), float(Y), float(d)])
#                     self.get_logger().info(f"Outside Wall 3D (cm): x={X*100:.2f}, y={Y*100:.2f}, z={d*100:.2f}")
#         return outside_3d

#     def timer_callback(self):
#         if self.latest_color is None or self.latest_depth is None: return
#         img, depth = self.latest_color.copy(), self.latest_depth.copy()
#         corners = self.get_ordered_corners(img)

#         if corners is None:
#             cv2.imshow("Camera View", img); cv2.waitKey(1)
#             return

#         H, _ = cv2.findHomography(corners, np.array([[0,0], [500,0], [500,500], [0,500]], dtype=np.float32))
#         topdown_viz, pawns_3d, walls_in_3d = self.process_topdown(cv2.warpPerspective(img, H, (500, 500)))
#         walls_out_3d = self.detect_outside_walls(img, depth, corners)

#         for p in corners: cv2.circle(img, (int(p[0]), int(p[1])), 6, (0, 255, 0), -1)
        
#         cv2.imshow("Camera View", img)
#         cv2.imshow("Top-Down Unified", topdown_viz)
#         cv2.waitKey(1)

#         self.pub_topdown.publish(self.bridge.cv2_to_imgmsg(topdown_viz, "bgr8"))
#         self.pub_board_state.publish(Int32MultiArray(data=self.grid_state.flatten().tolist()))
#         self.pub_wall_state.publish(Int32MultiArray(data=self.wall_state.flatten().tolist()))
#         self.pub_pawns_3d.publish(Float32MultiArray(data=pawns_3d))
#         self.pub_walls_inside_3d.publish(Float32MultiArray(data=walls_in_3d))
#         self.pub_walls_outside_3d.publish(Float32MultiArray(data=walls_out_3d))
#         self.pub_corners.publish(Float32MultiArray(data=corners.flatten().tolist()))

# def main(args=None):
#     rclpy.init(args=args)
#     node = PerceptionNode()
#     try: rclpy.spin(node)
#     except KeyboardInterrupt: pass
#     finally:
#         cv2.destroyAllWindows()
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()


# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from std_msgs.msg import Int32MultiArray, Float32MultiArray
# import numpy as np
# import cv2
# from cv_bridge import CvBridge
# import os
# import json

# class PerceptionNode(Node):
#     def __init__(self):
#         super().__init__('perception_node')
#         self.bridge = CvBridge()

#         # Publishers
#         self.pub_topdown = self.create_publisher(Image, '/perception/topdown_view', 10)
#         self.pub_board_state = self.create_publisher(Int32MultiArray, '/perception/board_state', 10)
#         self.pub_wall_state = self.create_publisher(Int32MultiArray, '/perception/wall_state', 10)
#         self.pub_pawns_3d = self.create_publisher(Float32MultiArray, '/perception/pawns_3d', 10)
#         self.pub_walls_inside_3d = self.create_publisher(Float32MultiArray, '/perception/walls_inside_3d', 10)
#         self.pub_walls_outside_3d = self.create_publisher(Float32MultiArray, '/perception/walls_outside_3d', 10)
#         self.pub_corners = self.create_publisher(Float32MultiArray, '/perception/board_corners', 10)

#         # Subscriptions
#         self.sub_color = self.create_subscription(Image, '/camera/color', self.color_callback, 10)
#         self.sub_depth = self.create_subscription(Image, '/camera/depth', self.depth_callback, 10)

#         self.latest_color = None
#         self.latest_depth = None

#         # Config & Files
#         self.board_size = 500
#         self.cell_size = 100
#         self.pawn_coords_file = os.path.expanduser("~/rs2_ws/src/perception/pawn_coords.txt")
#         self.wall_coords_file = os.path.expanduser("~/rs2_ws/src/perception/wall_coords.txt")
#         self.intrinsics_file = os.path.expanduser("~/rs2_ws/src/perception/camera_intrinsics.json")

#         self.pawn_lookup = {}
#         self.wall_lookup = {}
#         self.load_coordinates()
#         self.load_intrinsics()

#         self.grid_state = np.zeros((5, 5), dtype=int)
#         self.wall_circles = np.zeros((4, 4), dtype=int)

#         cv2.namedWindow("Camera View", cv2.WINDOW_AUTOSIZE)
#         cv2.namedWindow("Top-Down Unified", cv2.WINDOW_AUTOSIZE)
        
#         self.timer = self.create_timer(0.1, self.timer_callback)
#         self.get_logger().info("Unified Perception Node Started (Dot-Grid Mode)")

#     def load_coordinates(self):
#         try:
#             with open(self.pawn_coords_file, "r") as f:
#                 for line in f:
#                     p = line.strip().split(",")
#                     if len(p) == 5: self.pawn_lookup[(int(p[0]), int(p[1]))] = [float(p[2]), float(p[3]), float(p[4])]
#         except: self.get_logger().warn("Pawn file missing.")
#         try:
#             with open(self.wall_coords_file, "r") as f:
#                 for line in f:
#                     p = line.strip().split(",")
#                     if len(p) == 5: self.wall_lookup[(int(p[0]), int(p[1]))] = [float(p[2]), float(p[3]), float(p[4])]
#         except: self.get_logger().warn("Wall file missing.")

#     def load_intrinsics(self):
#         with open(self.intrinsics_file, "r") as f:
#             intr = json.load(f)
#             self.fx, self.fy, self.ppx, self.ppy = intr["fx"], intr["fy"], intr["ppx"], intr["ppy"]

#     def color_callback(self, msg): self.latest_color = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
#     def depth_callback(self, msg): self.latest_depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')

#     def get_ordered_corners(self, img):
#         gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#         edges = cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0), 50, 150)
#         contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#         if not contours: return None
#         board_cnts = [cnt for cnt in contours if 20000 < cv2.contourArea(cnt) < (img.shape[0]*img.shape[1])]
#         if not board_cnts: return None
#         pts = cv2.approxPolyDP(max(board_cnts, key=cv2.contourArea), 0.02 * cv2.arcLength(max(board_cnts, key=cv2.contourArea), True), True).reshape(4, 2)
#         rect = np.zeros((4, 2), dtype=np.float32)
#         s, d = pts.sum(axis=1), np.diff(pts, axis=1)
#         rect[0], rect[2], rect[1], rect[3] = pts[np.argmin(s)], pts[np.argmax(s)], pts[np.argmin(d)], pts[np.argmax(d)]
#         return rect

#     def process_topdown(self, topdown):
#         # --- PAWN DETECTION ---
#         gray = cv2.cvtColor(topdown, cv2.COLOR_BGR2GRAY)
#         _, black_mask = cv2.threshold(gray, 70, 255, cv2.THRESH_BINARY_INV)
#         pawns_list_3d = []
#         self.grid_state[:] = 0
        
#         for r in range(5):
#             for c in range(5):
#                 x1, y1, x2, y2 = c*100+30, r*100+30, (c+1)*100-30, (r+1)*100-30
#                 roi = black_mask[y1:y2, x1:x2]
#                 if cv2.countNonZero(roi) / (40**2) > 0.05:
#                     self.grid_state[r, c] = 1
#                     cnts, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#                     if cnts:
#                         bx, by, bw, bh = cv2.boundingRect(max(cnts, key=cv2.contourArea))
#                         cv2.rectangle(topdown, (x1+bx, y1+by), (x1+bx+bw, y1+by+bh), (0, 0, 255), 2)
#                     if (r, c) in self.pawn_lookup:
#                         p = self.pawn_lookup[(r, c)]
#                         pawns_list_3d.extend([float(r), float(c), p[0], p[1], p[2]])
#                         self.get_logger().info(f"Pawn at [{r},{c}] 3D: x={p[0]:.3f}, y={p[1]:.3f}, z={p[2]:.3f}")

#         # --- WALL DETECTION ---
#         hsv = cv2.cvtColor(topdown, cv2.COLOR_BGR2HSV)
#         red_mask = cv2.bitwise_or(cv2.inRange(hsv, np.array([0,120,70]), np.array([10,255,255])),
#                                   cv2.inRange(hsv, np.array([170,120,70]), np.array([180,255,255])))
#         self.wall_circles[:] = 0
#         walls_in_3d = []
#         red_cnts, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
#         for cnt in red_cnts:
#             if cv2.contourArea(cnt) < 100: continue
#             rx, ry, rw, rh = cv2.boundingRect(cnt)
#             c_idx, r_idx = int(round((rx+rw//2)/100))-1, int(round((ry+rh//2)/100))-1
            
#             if 0 <= r_idx < 4 and 0 <= c_idx < 4:
#                 self.wall_circles[r_idx, c_idx] = 2 if rw > rh else 1
#                 cx_c, cy_c = (c_idx+1)*100, (r_idx+1)*100
#                 # Visuals ONLY if inside grid
#                 cv2.rectangle(topdown, (rx, ry), (rx+rw, ry+rh), (0, 0, 255), 2)
#                 cv2.drawMarker(topdown, (rx+rw//2, ry+rh//2), (0, 255, 255), cv2.MARKER_STAR, 10, 1)
#                 cv2.drawMarker(topdown, (cx_c, cy_c), (255, 255, 0), cv2.MARKER_STAR, 12, 2)
#                 ext = 25
#                 if rw > rh: cv2.line(topdown, (cx_c-ext, cy_c), (cx_c+ext, cy_c), (0, 255, 0), 2)
#                 else: cv2.line(topdown, (cx_c, cy_c-ext), (cx_c, cy_c+ext), (0, 255, 0), 2)
                
#                 if (c_idx, r_idx) in self.wall_lookup:
#                     w = self.wall_lookup[(c_idx, r_idx)]
#                     walls_in_3d.extend([float(r_idx), float(c_idx), w[0], w[1], w[2]])
#                     self.get_logger().info(f"Wall at [{c_idx},{r_idx}] 3D (cm): x={w[0]*100:.2f}, y={w[1]*100:.2f}")

#         # --- DRAW DOT GRID (NO LINES) ---
#         for i in range(6):
#             for j in range(6):
#                 cv2.circle(topdown, (i*100, j*100), 3, (0, 255, 0), -1)

#         # Terminal Output
#         print(f"\nDigital Board:\n{self.grid_state}")
#         print(f"Wall Orientation:\n{self.wall_circles}")

#         return topdown, pawns_list_3d, walls_in_3d

#     def detect_outside_walls(self, img, depth, corners):
#         hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
#         red = cv2.bitwise_or(cv2.inRange(hsv, np.array([0,120,70]), np.array([10,255,255])),
#                              cv2.inRange(hsv, np.array([170,120,70]), np.array([180,255,255])))
#         mask = np.zeros(img.shape[:2], dtype=np.uint8)
#         cv2.fillPoly(mask, [corners.astype(np.int32)], 255)
#         cnts, _ = cv2.findContours(cv2.bitwise_and(red, cv2.bitwise_not(mask)), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#         out_3d = []
#         for cnt in cnts:
#             if cv2.contourArea(cnt) < 150: continue
#             x, y, w, h = cv2.boundingRect(cnt)
#             cx, cy = x+w//2, y+h//2
#             cv2.rectangle(img, (x, y), (x+w, y+h), (0, 0, 255), 2)
#             cv2.drawMarker(img, (cx, cy), (0, 255, 255), cv2.MARKER_STAR, 10, 1)
#             if depth is not None:
#                 d_m = depth[cy, cx] * 0.001
#                 if d_m > 0:
#                     X, Y = (cx-self.ppx)/self.fx*d_m, (cy-self.ppy)/self.fy*d_m
#                     out_3d.extend([float(X), float(Y), float(d_m)])
#                     self.get_logger().info(f"Outside Wall 3D (cm): x={X*100:.2f}, y={Y*100:.2f}")
#         return out_3d

#     def timer_callback(self):
#         if self.latest_color is None or self.latest_depth is None: return
#         img, depth = self.latest_color.copy(), self.latest_depth.copy()
#         corners = self.get_ordered_corners(img)
#         if corners is None:
#             cv2.imshow("Camera View", img); cv2.waitKey(1)
#             return

#         H, _ = cv2.findHomography(corners, np.array([[0,0], [500,0], [500,500], [0,500]], dtype=np.float32))
#         topdown_viz, p_3d, w_in_3d = self.process_topdown(cv2.warpPerspective(img, H, (500, 500)))
#         w_out_3d = self.detect_outside_walls(img, depth, corners)

#         for p in corners: cv2.circle(img, (int(p[0]), int(p[1])), 6, (0, 255, 0), -1)
#         cv2.imshow("Camera View", img); cv2.imshow("Top-Down Unified", topdown_viz); cv2.waitKey(1)

#         self.pub_topdown.publish(self.bridge.cv2_to_imgmsg(topdown_viz, "bgr8"))
#         self.pub_board_state.publish(Int32MultiArray(data=self.grid_state.flatten().tolist()))
#         self.pub_wall_state.publish(Int32MultiArray(data=self.wall_circles.flatten().tolist()))
#         self.pub_pawns_3d.publish(Float32MultiArray(data=p_3d))
#         self.pub_walls_inside_3d.publish(Float32MultiArray(data=w_in_3d))
#         self.pub_walls_outside_3d.publish(Float32MultiArray(data=w_out_3d))
#         self.pub_corners.publish(Float32MultiArray(data=corners.flatten().tolist()))

# def main(args=None):
#     rclpy.init(args=args)
#     node = PerceptionNode()
#     try: rclpy.spin(node)
#     except KeyboardInterrupt: pass
#     finally:
#         cv2.destroyAllWindows()
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__': main()


# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from std_msgs.msg import Int32MultiArray, Float32MultiArray
# import numpy as np
# import cv2
# from cv_bridge import CvBridge
# import os
# import json

# class PerceptionNode(Node):
#     def __init__(self):
#         super().__init__('perception_node')
#         self.bridge = CvBridge()

#         # Publishers
#         self.pub_topdown = self.create_publisher(Image, '/perception/topdown_view', 10)
#         self.pub_board_state = self.create_publisher(Int32MultiArray, '/perception/board_state', 10)
#         self.pub_wall_state = self.create_publisher(Int32MultiArray, '/perception/wall_state', 10)
#         self.pub_pawns_3d = self.create_publisher(Float32MultiArray, '/perception/pawns_3d', 10)
#         self.pub_walls_inside_3d = self.create_publisher(Float32MultiArray, '/perception/walls_inside_3d', 10)
#         self.pub_walls_outside_3d = self.create_publisher(Float32MultiArray, '/perception/walls_outside_3d', 10)
#         self.pub_corners = self.create_publisher(Float32MultiArray, '/perception/board_corners', 10)

#         # Subscriptions
#         self.sub_color = self.create_subscription(Image, '/camera/color', self.color_callback, 10)
#         self.sub_depth = self.create_subscription(Image, '/camera/depth', self.depth_callback, 10)

#         self.latest_color = None
#         self.latest_depth = None

#         # Config & Files
#         self.board_size = 500
#         self.pawn_coords_file = os.path.expanduser("~/rs2_ws/src/perception/pawn_coords.txt")
#         self.wall_coords_file = os.path.expanduser("~/rs2_ws/src/perception/wall_coords.txt")
#         self.intrinsics_file = os.path.expanduser("~/rs2_ws/src/perception/camera_intrinsics.json")

#         self.pawn_lookup = {}
#         self.wall_lookup = {}
#         self.load_coordinates()
#         self.load_intrinsics()

#         self.grid_state = np.zeros((5, 5), dtype=int)
#         self.wall_circles = np.zeros((4, 4), dtype=int)

#         cv2.namedWindow("Camera View", cv2.WINDOW_AUTOSIZE)
#         cv2.namedWindow("Top-Down Unified", cv2.WINDOW_AUTOSIZE)
        
#         self.timer = self.create_timer(0.1, self.timer_callback)
#         self.get_logger().info("Perception Node: Visuals Mirrored & Corner Dots Only.")

#     def load_coordinates(self):
#         try:
#             with open(self.pawn_coords_file, "r") as f:
#                 for line in f:
#                     p = line.strip().split(",")
#                     if len(p) == 5: self.pawn_lookup[(int(p[0]), int(p[1]))] = [float(p[2]), float(p[3]), float(p[4])]
#         except: self.get_logger().warn("Pawn file missing.")
#         try:
#             with open(self.wall_coords_file, "r") as f:
#                 for line in f:
#                     p = line.strip().split(",")
#                     if len(p) == 5: self.wall_lookup[(int(p[0]), int(p[1]))] = [float(p[2]), float(p[3]), float(p[4])]
#         except: self.get_logger().warn("Wall file missing.")

#     def load_intrinsics(self):
#         with open(self.intrinsics_file, "r") as f:
#             intr = json.load(f)
#             self.fx, self.fy, self.ppx, self.ppy = intr["fx"], intr["fy"], intr["ppx"], intr["ppy"]

#     def color_callback(self, msg): self.latest_color = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
#     def depth_callback(self, msg): self.latest_depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')

#     def get_ordered_corners(self, img):
#         gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#         edges = cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0), 50, 150)
#         contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#         if not contours: return None
#         board_cnts = [cnt for cnt in contours if 20000 < cv2.contourArea(cnt) < (img.shape[0]*img.shape[1])]
#         if not board_cnts: return None
#         pts = cv2.approxPolyDP(max(board_cnts, key=cv2.contourArea), 0.02 * cv2.arcLength(max(board_cnts, key=cv2.contourArea), True), True).reshape(4, 2)
#         rect = np.zeros((4, 2), dtype=np.float32)
#         s, d = pts.sum(axis=1), np.diff(pts, axis=1)
#         rect[0], rect[2], rect[1], rect[3] = pts[np.argmin(s)], pts[np.argmax(s)], pts[np.argmin(d)], pts[np.argmax(d)]
#         return rect

#     def process_frame(self, topdown, camera_img, H_inv):
#         # --- PAWN DETECTION ---
#         gray = cv2.cvtColor(topdown, cv2.COLOR_BGR2GRAY)
#         _, black_mask = cv2.threshold(gray, 70, 255, cv2.THRESH_BINARY_INV)
#         pawns_list_3d = []
#         self.grid_state[:] = 0
        
#         for r in range(5):
#             for c in range(5):
#                 x1, y1, x2, y2 = c*100+30, r*100+30, (c+1)*100-30, (r+1)*100-30
#                 roi = black_mask[y1:y2, x1:x2]
#                 if cv2.countNonZero(roi) / (40**2) > 0.05:
#                     self.grid_state[r, c] = 1
#                     cnts, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#                     if cnts:
#                         bx, by, bw, bh = cv2.boundingRect(max(cnts, key=cv2.contourArea))
#                         # Top-down rect
#                         cv2.rectangle(topdown, (x1+bx, y1+by), (x1+bx+bw, y1+by+bh), (0, 0, 255), 2)
#                         # Mirror to camera feed
#                         pts = np.array([[x1+bx, y1+by], [x1+bx+bw, y1+by+bh]], dtype='float32').reshape(-1,1,2)
#                         cam_pts = cv2.perspectiveTransform(pts, H_inv)
#                         cv2.rectangle(camera_img, tuple(cam_pts[0][0].astype(int)), tuple(cam_pts[1][0].astype(int)), (0, 0, 255), 2)

#                     if (r, c) in self.pawn_lookup:
#                         p = self.pawn_lookup[(r, c)]
#                         pawns_list_3d.extend([float(r), float(c), p[0], p[1], p[2]])

#         # --- WALL DETECTION ---
#         hsv = cv2.cvtColor(topdown, cv2.COLOR_BGR2HSV)
#         red_mask = cv2.bitwise_or(cv2.inRange(hsv, np.array([0,120,70]), np.array([10,255,255])),
#                                   cv2.inRange(hsv, np.array([170,120,70]), np.array([180,255,255])))
#         self.wall_circles[:] = 0
#         walls_in_3d = []
#         red_cnts, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
#         for cnt in red_cnts:
#             if cv2.contourArea(cnt) < 100: continue
#             rx, ry, rw, rh = cv2.boundingRect(cnt)
#             cx_blob, cy_blob = rx + rw//2, ry + rh//2
#             c_idx, r_idx = int(round(cx_blob/100))-1, int(round(cy_blob/100))-1
            
#             # STRICT CHECK: Only draw if it maps to a valid circle/grid intersection
#             if 0 <= r_idx < 4 and 0 <= c_idx < 4:
#                 self.wall_circles[r_idx, c_idx] = 2 if rw > rh else 1
#                 cx_grid, cy_grid = (c_idx+1)*100, (r_idx+1)*100
#                 ext = 25
                
#                 # Draw on Topdown
#                 cv2.rectangle(topdown, (rx, ry), (rx+rw, ry+rh), (0, 0, 255), 2)
#                 cv2.drawMarker(topdown, (cx_blob, cy_blob), (0, 255, 255), cv2.MARKER_STAR, 10, 1)
#                 cv2.drawMarker(topdown, (cx_grid, cy_grid), (255, 255, 0), cv2.MARKER_STAR, 12, 2)
                
#                 # Draw on Camera
#                 pts_wall = np.array([[rx, ry], [rx+rw, ry+rh], [cx_blob, cy_blob], [cx_grid, cy_grid]], dtype='float32').reshape(-1,1,2)
#                 cam_wall = cv2.perspectiveTransform(pts_wall, H_inv).astype(int)
#                 cv2.rectangle(camera_img, tuple(cam_wall[0][0]), tuple(cam_wall[1][0]), (0, 0, 255), 2)
#                 cv2.drawMarker(camera_img, tuple(cam_wall[2][0]), (0, 255, 255), cv2.MARKER_STAR, 10, 1)
#                 cv2.drawMarker(camera_img, tuple(cam_wall[3][0]), (255, 255, 0), cv2.MARKER_STAR, 12, 2)

#                 if rw > rh: # Horizontal
#                     cv2.line(topdown, (cx_grid-ext, cy_grid), (cx_grid+ext, cy_grid), (0, 255, 0), 2)
#                 else: # Vertical
#                     cv2.line(topdown, (cx_grid, cy_grid-ext), (cx_grid, cy_grid+ext), (0, 255, 0), 2)
                
#                 if (c_idx, r_idx) in self.wall_lookup:
#                     w = self.wall_lookup[(c_idx, r_idx)]
#                     walls_in_3d.extend([float(r_idx), float(c_idx), w[0], w[1], w[2]])

#         # Print States
#         print(f"\nDigital Board:\n{self.grid_state}")
#         print(f"Wall Orientation:\n{self.wall_circles}")

#         return topdown, camera_img, pawns_list_3d, walls_in_3d

#     def detect_outside_walls(self, camera_img, depth, corners):
#         hsv = cv2.cvtColor(camera_img, cv2.COLOR_BGR2HSV)
#         red = cv2.bitwise_or(cv2.inRange(hsv, np.array([0,120,70]), np.array([10,255,255])),
#                              cv2.inRange(hsv, np.array([170,120,70]), np.array([180,255,255])))
#         mask = np.zeros(camera_img.shape[:2], dtype=np.uint8)
#         cv2.fillPoly(mask, [corners.astype(np.int32)], 255)
#         cnts, _ = cv2.findContours(cv2.bitwise_and(red, cv2.bitwise_not(mask)), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#         out_3d = []
#         for cnt in cnts:
#             if cv2.contourArea(cnt) < 150: continue
#             x, y, w, h = cv2.boundingRect(cnt)
#             cx, cy = x+w//2, y+h//2
#             cv2.rectangle(camera_img, (x, y), (x+w, y+h), (0, 0, 255), 2)
#             cv2.drawMarker(camera_img, (cx, cy), (0, 255, 255), cv2.MARKER_STAR, 10, 1)
#             if depth is not None:
#                 d_m = depth[cy, cx] * 0.001
#                 if d_m > 0:
#                     X, Y = (cx-self.ppx)/self.fx*d_m, (cy-self.ppy)/self.fy*d_m
#                     out_3d.extend([float(X), float(Y), float(d_m)])
#         return out_3d

#     def timer_callback(self):
#         if self.latest_color is None or self.latest_depth is None: return
#         img, depth = self.latest_color.copy(), self.latest_depth.copy()
#         corners = self.get_ordered_corners(img)
#         if corners is None:
#             cv2.imshow("Camera View", img); cv2.waitKey(1)
#             return

#         H = cv2.getPerspectiveTransform(corners, np.array([[0,0], [500,0], [500,500], [0,500]], dtype=np.float32))
#         H_inv = np.linalg.inv(H)
#         warped = cv2.warpPerspective(img, H, (500, 500))

#         topdown_viz, camera_viz, p_3d, w_in_3d = self.process_frame(warped, img, H_inv)
#         w_out_3d = self.detect_outside_walls(camera_viz, depth, corners)

#         # Draw ONLY the 4 corners on both views
#         for p in corners:
#             cv2.circle(camera_viz, (int(p[0]), int(p[1])), 8, (0, 255, 0), -1)
        
#         # 4 corners on topdown
#         td_corners = [(0,0), (500,0), (500,500), (0,500)]
#         for cp in td_corners:
#             cv2.circle(topdown_viz, cp, 8, (0, 255, 0), -1)

#         cv2.imshow("Camera View", camera_viz)
#         cv2.imshow("Top-Down Unified", topdown_viz)
#         cv2.waitKey(1)

#         self.pub_topdown.publish(self.bridge.cv2_to_imgmsg(topdown_viz, "bgr8"))
#         self.pub_board_state.publish(Int32MultiArray(data=self.grid_state.flatten().tolist()))
#         self.pub_wall_state.publish(Int32MultiArray(data=self.wall_circles.flatten().tolist()))
#         self.pub_pawns_3d.publish(Float32MultiArray(data=p_3d))
#         self.pub_walls_inside_3d.publish(Float32MultiArray(data=w_in_3d))
#         self.pub_walls_outside_3d.publish(Float32MultiArray(data=w_out_3d))
#         self.pub_corners.publish(Float32MultiArray(data=corners.flatten().tolist()))

# def main(args=None):
#     rclpy.init(args=args)
#     node = PerceptionNode()
#     try: rclpy.spin(node)
#     except KeyboardInterrupt: pass
#     finally:
#         cv2.destroyAllWindows()
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__': main()


# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from std_msgs.msg import Int32MultiArray, Float32MultiArray
# import numpy as np
# import cv2
# from cv_bridge import CvBridge
# import os
# import json

# class PerceptionNode(Node):
#     def __init__(self):
#         super().__init__('perception_node')
#         self.bridge = CvBridge()

#         # Publishers
#         self.pub_topdown = self.create_publisher(Image, '/perception/topdown_view', 10)
#         self.pub_board_state = self.create_publisher(Int32MultiArray, '/perception/board_state', 10)
#         self.pub_wall_state = self.create_publisher(Int32MultiArray, '/perception/wall_state', 10)
#         self.pub_pawns_3d = self.create_publisher(Float32MultiArray, '/perception/pawns_3d', 10)
#         self.pub_walls_inside_3d = self.create_publisher(Float32MultiArray, '/perception/walls_inside_3d', 10)
#         self.pub_walls_outside_3d = self.create_publisher(Float32MultiArray, '/perception/walls_outside_3d', 10)
#         self.pub_corners = self.create_publisher(Float32MultiArray, '/perception/board_corners', 10)

#         # Subscriptions
#         self.sub_color = self.create_subscription(Image, '/camera/color', self.color_callback, 10)
#         self.sub_depth = self.create_subscription(Image, '/camera/depth', self.depth_callback, 10)

#         self.latest_color = None
#         self.latest_depth = None

#         # Config & Files
#         self.board_size = 500
#         self.pawn_coords_file = os.path.expanduser("~/rs2_ws/src/perception/pawn_coords.txt")
#         self.wall_coords_file = os.path.expanduser("~/rs2_ws/src/perception/wall_coords.txt")
#         self.intrinsics_file = os.path.expanduser("~/rs2_ws/src/perception/camera_intrinsics.json")

#         self.pawn_lookup = {}
#         self.wall_lookup = {}
#         self.load_coordinates()
#         self.load_intrinsics()

#         self.grid_state = np.zeros((5, 5), dtype=int)
#         self.wall_circles = np.zeros((4, 4), dtype=int)

#         cv2.namedWindow("Camera View", cv2.WINDOW_AUTOSIZE)
#         cv2.namedWindow("Top-Down Unified", cv2.WINDOW_AUTOSIZE)
        
#         self.timer = self.create_timer(0.1, self.timer_callback)
#         self.get_logger().info("Perception Node: Board Outlined | Blue Pawn Boxes.")

#     def load_coordinates(self):
#         try:
#             with open(self.pawn_coords_file, "r") as f:
#                 for line in f:
#                     p = line.strip().split(",")
#                     if len(p) == 5: self.pawn_lookup[(int(p[0]), int(p[1]))] = [float(p[2]), float(p[3]), float(p[4])]
#         except: self.get_logger().warn("Pawn file missing.")
#         try:
#             with open(self.wall_coords_file, "r") as f:
#                 for line in f:
#                     p = line.strip().split(",")
#                     if len(p) == 5: self.wall_lookup[(int(p[0]), int(p[1]))] = [float(p[2]), float(p[3]), float(p[4])]
#         except: self.get_logger().warn("Wall file missing.")

#     def load_intrinsics(self):
#         with open(self.intrinsics_file, "r") as f:
#             intr = json.load(f)
#             self.fx, self.fy, self.ppx, self.ppy = intr["fx"], intr["fy"], intr["ppx"], intr["ppy"]

#     def color_callback(self, msg): self.latest_color = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
#     def depth_callback(self, msg): self.latest_depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')

#     def get_ordered_corners(self, img):
#         gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#         edges = cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0), 50, 150)
#         contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#         if not contours: return None
#         board_cnts = [cnt for cnt in contours if 20000 < cv2.contourArea(cnt) < (img.shape[0]*img.shape[1])]
#         if not board_cnts: return None
#         pts = cv2.approxPolyDP(max(board_cnts, key=cv2.contourArea), 0.02 * cv2.arcLength(max(board_cnts, key=cv2.contourArea), True), True).reshape(4, 2)
#         rect = np.zeros((4, 2), dtype=np.float32)
#         s, d = pts.sum(axis=1), np.diff(pts, axis=1)
#         rect[0], rect[2], rect[1], rect[3] = pts[np.argmin(s)], pts[np.argmax(s)], pts[np.argmin(d)], pts[np.argmax(d)]
#         return rect

#     def process_frame(self, topdown, camera_img, H_inv):
#         # --- PAWN DETECTION (BLUE BOXES) ---
#         gray = cv2.cvtColor(topdown, cv2.COLOR_BGR2GRAY)
#         _, black_mask = cv2.threshold(gray, 70, 255, cv2.THRESH_BINARY_INV)
#         pawns_list_3d = []
#         self.grid_state[:] = 0
        
#         for r in range(5):
#             for c in range(5):
#                 x1, y1, x2, y2 = c*100+30, r*100+30, (c+1)*100-30, (r+1)*100-30
#                 roi = black_mask[y1:y2, x1:x2]
#                 if cv2.countNonZero(roi) / (40**2) > 0.05:
#                     self.grid_state[r, c] = 1
#                     cnts, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#                     if cnts:
#                         bx, by, bw, bh = cv2.boundingRect(max(cnts, key=cv2.contourArea))
#                         # Top-down Blue Rect
#                         cv2.rectangle(topdown, (x1+bx, y1+by), (x1+bx+bw, y1+by+bh), (255, 0, 0), 2)
#                         # Mirror to camera feed
#                         pts = np.array([[x1+bx, y1+by], [x1+bx+bw, y1+by+bh]], dtype='float32').reshape(-1,1,2)
#                         cam_pts = cv2.perspectiveTransform(pts, H_inv)
#                         cv2.rectangle(camera_img, tuple(cam_pts[0][0].astype(int)), tuple(cam_pts[1][0].astype(int)), (255, 0, 0), 2)

#                     if (r, c) in self.pawn_lookup:
#                         p = self.pawn_lookup[(r, c)]
#                         pawns_list_3d.extend([float(r), float(c), p[0], p[1], p[2]])

#         # --- WALL DETECTION ---
#         hsv = cv2.cvtColor(topdown, cv2.COLOR_BGR2HSV)
#         red_mask = cv2.bitwise_or(cv2.inRange(hsv, np.array([0,120,70]), np.array([10,255,255])),
#                                   cv2.inRange(hsv, np.array([170,120,70]), np.array([180,255,255])))
#         self.wall_circles[:] = 0
#         walls_in_3d = []
#         red_cnts, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
#         for cnt in red_cnts:
#             if cv2.contourArea(cnt) < 100: continue
#             rx, ry, rw, rh = cv2.boundingRect(cnt)
#             cx_blob, cy_blob = rx + rw//2, ry + rh//2
#             c_idx, r_idx = int(round(cx_blob/100))-1, int(round(cy_blob/100))-1
            
#             if 0 <= r_idx < 4 and 0 <= c_idx < 4:
#                 self.wall_circles[r_idx, c_idx] = 2 if rw > rh else 1
#                 cx_grid, cy_grid = (c_idx+1)*100, (r_idx+1)*100
#                 ext = 25
                
#                 # Topdown Wall Visuals
#                 cv2.rectangle(topdown, (rx, ry), (rx+rw, ry+rh), (0, 0, 255), 2)
#                 cv2.drawMarker(topdown, (cx_blob, cy_blob), (0, 255, 255), cv2.MARKER_STAR, 10, 1)
#                 cv2.drawMarker(topdown, (cx_grid, cy_grid), (255, 255, 0), cv2.MARKER_STAR, 12, 2)
                
#                 # Mirror to Camera
#                 pts_wall = np.array([[rx, ry], [rx+rw, ry+rh], [cx_blob, cy_blob], [cx_grid, cy_grid]], dtype='float32').reshape(-1,1,2)
#                 cam_wall = cv2.perspectiveTransform(pts_wall, H_inv).astype(int)
#                 cv2.rectangle(camera_img, tuple(cam_wall[0][0]), tuple(cam_wall[1][0]), (0, 0, 255), 2)
#                 cv2.drawMarker(camera_img, tuple(cam_wall[2][0]), (0, 255, 255), cv2.MARKER_STAR, 10, 1)
#                 cv2.drawMarker(camera_img, tuple(cam_wall[3][0]), (255, 255, 0), cv2.MARKER_STAR, 12, 2)

#                 if rw > rh:
#                     cv2.line(topdown, (cx_grid-ext, cy_grid), (cx_grid+ext, cy_grid), (0, 255, 0), 2)
#                 else:
#                     cv2.line(topdown, (cx_grid, cy_grid-ext), (cx_grid, cy_grid+ext), (0, 255, 0), 2)
                
#                 if (c_idx, r_idx) in self.wall_lookup:
#                     w = self.wall_lookup[(c_idx, r_idx)]
#                     walls_in_3d.extend([float(r_idx), float(c_idx), w[0], w[1], w[2]])

#         print(f"\nBoard State:\n{self.grid_state}")
#         return topdown, camera_img, pawns_list_3d, walls_in_3d

#     def timer_callback(self):
#         if self.latest_color is None or self.latest_depth is None: return
#         img, depth = self.latest_color.copy(), self.latest_depth.copy()
#         corners = self.get_ordered_corners(img)
#         if corners is None:
#             cv2.imshow("Camera View", img); cv2.waitKey(1)
#             return

#         H = cv2.getPerspectiveTransform(corners, np.array([[0,0], [500,0], [500,500], [0,500]], dtype=np.float32))
#         H_inv = np.linalg.inv(H)
#         warped = cv2.warpPerspective(img, H, (500, 500))

#         topdown_viz, camera_viz, p_3d, w_in_3d = self.process_frame(warped, img, H_inv)
        
#         # --- DRAW BOARD OUTLINE AND 4 CORNERS ---
#         # 1. Camera Feed Outline
#         pts_outline = corners.astype(np.int32).reshape((-1, 1, 2))
#         cv2.polylines(camera_viz, [pts_outline], True, (0, 255, 0), 2)
#         for p in corners:
#             cv2.circle(camera_viz, (int(p[0]), int(p[1])), 8, (0, 255, 0), -1)
        
#         # 2. Top-down Outline
#         cv2.rectangle(topdown_viz, (0, 0), (500, 500), (0, 255, 0), 3)
#         td_corners = [(0,0), (500,0), (500,500), (0,500)]
#         for cp in td_corners:
#             cv2.circle(topdown_viz, cp, 8, (0, 255, 0), -1)

#         cv2.imshow("Camera View", camera_viz)
#         cv2.imshow("Top-Down Unified", topdown_viz)
#         cv2.waitKey(1)

#         self.pub_topdown.publish(self.bridge.cv2_to_imgmsg(topdown_viz, "bgr8"))
#         self.pub_board_state.publish(Int32MultiArray(data=self.grid_state.flatten().tolist()))
#         self.pub_wall_state.publish(Int32MultiArray(data=self.wall_circles.flatten().tolist()))
#         self.pub_pawns_3d.publish(Float32MultiArray(data=p_3d))
#         self.pub_walls_inside_3d.publish(Float32MultiArray(data=w_in_3d))
#         self.pub_corners.publish(Float32MultiArray(data=corners.flatten().tolist()))

# def main(args=None):
#     rclpy.init(args=args)
#     node = PerceptionNode()
#     try: rclpy.spin(node)
#     except KeyboardInterrupt: pass
#     finally:
#         cv2.destroyAllWindows()
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__': main()


# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from std_msgs.msg import Int32MultiArray, Float32MultiArray
# import numpy as np
# import cv2
# from cv_bridge import CvBridge
# import os
# import json

# class PerceptionNode(Node):
#     def __init__(self):
#         super().__init__('perception_node')
#         self.bridge = CvBridge()

#         # Publishers
#         self.pub_topdown = self.create_publisher(Image, '/perception/topdown_view', 10)
#         self.pub_board_state = self.create_publisher(Int32MultiArray, '/perception/board_state', 10)
#         self.pub_wall_state = self.create_publisher(Int32MultiArray, '/perception/wall_state', 10)
#         self.pub_pawns_3d = self.create_publisher(Float32MultiArray, '/perception/pawns_3d', 10)
#         self.pub_walls_inside_3d = self.create_publisher(Float32MultiArray, '/perception/walls_inside_3d', 10)
#         self.pub_corners = self.create_publisher(Float32MultiArray, '/perception/board_corners', 10)

#         # Subscriptions
#         self.sub_color = self.create_subscription(Image, '/camera/color', self.color_callback, 10)
#         self.sub_depth = self.create_subscription(Image, '/camera/depth', self.depth_callback, 10)

#         self.latest_color = None
#         self.latest_depth = None

#         # Files
#         self.pawn_coords_file = os.path.expanduser("~/rs2_ws/src/perception/pawn_coords.txt")
#         self.wall_coords_file = os.path.expanduser("~/rs2_ws/src/perception/wall_coords.txt")
#         self.intrinsics_file = os.path.expanduser("~/rs2_ws/src/perception/camera_intrinsics.json")

#         self.pawn_lookup = {}
#         self.wall_lookup = {}
#         self.load_coordinates()
#         self.load_intrinsics()

#         self.grid_state = np.zeros((5, 5), dtype=int)
#         self.wall_circles = np.zeros((4, 4), dtype=int)

#         cv2.namedWindow("Camera View", cv2.WINDOW_AUTOSIZE)
#         cv2.namedWindow("Top-Down Unified", cv2.WINDOW_AUTOSIZE)
        
#         self.timer = self.create_timer(0.1, self.timer_callback)
#         self.get_logger().info("Perception Node Started: Tracking Pawns and Walls.")

#     def load_coordinates(self):
#         try:
#             with open(self.pawn_coords_file, "r") as f:
#                 for line in f:
#                     p = line.strip().split(",")
#                     if len(p) == 5: self.pawn_lookup[(int(p[0]), int(p[1]))] = [float(p[2]), float(p[3]), float(p[4])]
#         except: self.get_logger().warn("Pawn coordinate file not found.")
#         try:
#             with open(self.wall_coords_file, "r") as f:
#                 for line in f:
#                     p = line.strip().split(",")
#                     if len(p) == 5: self.wall_lookup[(int(p[0]), int(p[1]))] = [float(p[2]), float(p[3]), float(p[4])]
#         except: self.get_logger().warn("Wall coordinate file not found.")

#     def load_intrinsics(self):
#         with open(self.intrinsics_file, "r") as f:
#             intr = json.load(f)
#             self.fx, self.fy, self.ppx, self.ppy = intr["fx"], intr["fy"], intr["ppx"], intr["ppy"]

#     def color_callback(self, msg): self.latest_color = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
#     def depth_callback(self, msg): self.latest_depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')

#     def get_ordered_corners(self, img):
#         gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#         edges = cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0), 50, 150)
#         contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#         if not contours: return None
#         board_cnts = [cnt for cnt in contours if 20000 < cv2.contourArea(cnt) < (img.shape[0]*img.shape[1])]
#         if not board_cnts: return None
#         pts = cv2.approxPolyDP(max(board_cnts, key=cv2.contourArea), 0.02 * cv2.arcLength(max(board_cnts, key=cv2.contourArea), True), True).reshape(4, 2)
#         rect = np.zeros((4, 2), dtype=np.float32)
#         s, d = pts.sum(axis=1), np.diff(pts, axis=1)
#         rect[0], rect[2], rect[1], rect[3] = pts[np.argmin(s)], pts[np.argmax(s)], pts[np.argmin(d)], pts[np.argmax(d)]
#         return rect

#     def process_visuals(self, topdown, camera_img, H_inv):
#         # --- PAWN DETECTION (BLUE BOXES) ---
#         gray = cv2.cvtColor(topdown, cv2.COLOR_BGR2GRAY)
#         _, black_mask = cv2.threshold(gray, 70, 255, cv2.THRESH_BINARY_INV)
#         pawns_list_3d = []
#         self.grid_state[:] = 0
        
#         self.get_logger().info("--- Pawn Detections ---")
#         for r in range(5):
#             for c in range(5):
#                 x1, y1, x2, y2 = c*100+30, r*100+30, (c+1)*100-30, (r+1)*100-30
#                 roi = black_mask[y1:y2, x1:x2]
#                 if cv2.countNonZero(roi) / (40**2) > 0.05:
#                     self.grid_state[r, c] = 1
#                     cnts, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#                     if cnts:
#                         bx, by, bw, bh = cv2.boundingRect(max(cnts, key=cv2.contourArea))
#                         cv2.rectangle(topdown, (x1+bx, y1+by), (x1+bx+bw, y1+by+bh), (255, 0, 0), 2)
#                         pts = np.array([[x1+bx, y1+by], [x1+bx+bw, y1+by+bh]], dtype='float32').reshape(-1,1,2)
#                         cam_pts = cv2.perspectiveTransform(pts, H_inv).astype(int)
#                         cv2.rectangle(camera_img, tuple(cam_pts[0][0]), tuple(cam_pts[1][0]), (255, 0, 0), 2)
#                     if (r, c) in self.pawn_lookup:
#                         p = self.pawn_lookup[(r, c)]
#                         pawns_list_3d.extend([float(r), float(c), p[0], p[1], p[2]])
#                         self.get_logger().info(f"Pawn [{r},{c}] 3D: x={p[0]:.2f}, y={p[1]:.2f}, z={p[2]:.2f}")

#         # --- WALL DETECTION (ORIENTATION LINES) ---
#         hsv = cv2.cvtColor(topdown, cv2.COLOR_BGR2HSV)
#         red_mask = cv2.bitwise_or(cv2.inRange(hsv, np.array([0,120,70]), np.array([10,255,255])),
#                                   cv2.inRange(hsv, np.array([170,120,70]), np.array([180,255,255])))
#         self.wall_circles[:] = 0
#         walls_in_3d = []
#         red_cnts, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
#         self.get_logger().info("--- Wall Detections ---")
#         for cnt in red_cnts:
#             if cv2.contourArea(cnt) < 100: continue
#             rx, ry, rw, rh = cv2.boundingRect(cnt)
#             cx_blob, cy_blob = rx + rw//2, ry + rh//2
#             c_idx, r_idx = int(round(cx_blob/100))-1, int(round(cy_blob/100))-1
            
#             if 0 <= r_idx < 4 and 0 <= c_idx < 4:
#                 is_horiz = rw > rh
#                 self.wall_circles[r_idx, c_idx] = 2 if is_horiz else 1
#                 cx_grid, cy_grid = (c_idx+1)*100, (r_idx+1)*100
#                 ext = 25
                
#                 # Visuals on Topdown
#                 cv2.rectangle(topdown, (rx, ry), (rx+rw, ry+rh), (0, 0, 255), 2)
#                 if is_horiz: cv2.line(topdown, (cx_grid-ext, cy_grid), (cx_grid+ext, cy_grid), (0, 255, 0), 2)
#                 else: cv2.line(topdown, (cx_grid, cy_grid-ext), (cx_grid, cy_grid+ext), (0, 255, 0), 2)
                
#                 # Mirror to Camera
#                 pts_wall = np.array([[rx, ry], [rx+rw, ry+rh], [cx_grid-ext, cy_grid], [cx_grid+ext, cy_grid], [cx_grid, cy_grid-ext], [cx_grid, cy_grid+ext]], dtype='float32').reshape(-1,1,2)
#                 cw = cv2.perspectiveTransform(pts_wall, H_inv).astype(int)
#                 cv2.rectangle(camera_img, tuple(cw[0][0]), tuple(cw[1][0]), (0, 0, 255), 2)
#                 if is_horiz: cv2.line(camera_img, tuple(cw[2][0]), tuple(cw[3][0]), (0, 255, 0), 2)
#                 else: cv2.line(camera_img, tuple(cw[4][0]), tuple(cw[5][0]), (0, 255, 0), 2)

#                 if (c_idx, r_idx) in self.wall_lookup:
#                     w = self.wall_lookup[(c_idx, r_idx)]
#                     walls_in_3d.extend([float(r_idx), float(c_idx), w[0], w[1], w[2]])
#                     self.get_logger().info(f"Wall [{r_idx},{c_idx}] ({'H' if is_horiz else 'V'}) 3D: x={w[0]:.2f}, y={w[1]:.2f}")

#         # Wall State Terminal Log
#         wall_labels = {0: ".", 1: "V", 2: "H"}
#         self.get_logger().info("Wall Grid State:")
#         for row in self.wall_circles:
#             print(" ".join([wall_labels[val] for val in row]))

#         return topdown, camera_img, pawns_list_3d, walls_in_3d

#     def timer_callback(self):
#         if self.latest_color is None or self.latest_depth is None: return
#         img, depth = self.latest_color.copy(), self.latest_depth.copy()
#         corners = self.get_ordered_corners(img)
#         if corners is None:
#             cv2.imshow("Camera View", img); cv2.waitKey(1)
#             return

#         H = cv2.getPerspectiveTransform(corners, np.array([[0,0], [500,0], [500,500], [0,500]], dtype=np.float32))
#         H_inv = np.linalg.inv(H)
#         warped = cv2.warpPerspective(img, H, (500, 500))

#         topdown_viz, camera_viz, p_3d, w_in_3d = self.process_visuals(warped, img, H_inv)
        
#         # Board Outline & Corners
#         cv2.polylines(camera_viz, [corners.astype(np.int32).reshape((-1, 1, 2))], True, (0, 255, 0), 2)
#         for p in corners: cv2.circle(camera_viz, (int(p[0]), int(p[1])), 8, (0, 255, 0), -1)
#         cv2.rectangle(topdown_viz, (0, 0), (500, 500), (0, 255, 0), 3)
#         for cp in [(0,0), (500,0), (500,500), (0,500)]: cv2.circle(topdown_viz, cp, 8, (0, 255, 0), -1)

#         cv2.imshow("Camera View", camera_viz)
#         cv2.imshow("Top-Down Unified", topdown_viz)
#         cv2.waitKey(1)

#         # Publish Messages
#         self.pub_topdown.publish(self.bridge.cv2_to_imgmsg(topdown_viz, "bgr8"))
#         self.pub_board_state.publish(Int32MultiArray(data=self.grid_state.flatten().tolist()))
#         self.pub_wall_state.publish(Int32MultiArray(data=self.wall_circles.flatten().tolist()))
#         self.pub_pawns_3d.publish(Float32MultiArray(data=p_3d))
#         self.pub_walls_inside_3d.publish(Float32MultiArray(data=w_in_3d))
#         self.pub_corners.publish(Float32MultiArray(data=corners.flatten().tolist()))

# def main(args=None):
#     rclpy.init(args=args)
#     node = PerceptionNode()
#     try: rclpy.spin(node)
#     except KeyboardInterrupt: pass
#     finally:
#         cv2.destroyAllWindows()
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__': main()



# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from std_msgs.msg import Int32MultiArray, Float32MultiArray
# import numpy as np
# import cv2
# from cv_bridge import CvBridge
# import os
# import json

# class PerceptionNode(Node):
#     def __init__(self):
#         super().__init__('perception_node')
#         self.bridge = CvBridge()

#         # Publishers
#         self.pub_topdown = self.create_publisher(Image, '/perception/topdown_view', 10)
#         self.pub_board_state = self.create_publisher(Int32MultiArray, '/perception/board_state', 10)
#         self.pub_wall_state = self.create_publisher(Int32MultiArray, '/perception/wall_state', 10)
#         self.pub_pawns_3d = self.create_publisher(Float32MultiArray, '/perception/pawns_3d', 10)
#         self.pub_walls_inside_3d = self.create_publisher(Float32MultiArray, '/perception/walls_inside_3d', 10)
#         self.pub_walls_outside_3d = self.create_publisher(Float32MultiArray, '/perception/walls_outside_3d', 10)
#         self.pub_corners = self.create_publisher(Float32MultiArray, '/perception/board_corners', 10)

#         # Subscriptions
#         self.sub_color = self.create_subscription(Image, '/camera/color', self.color_callback, 10)
#         self.sub_depth = self.create_subscription(Image, '/camera/depth', self.depth_callback, 10)

#         self.latest_color = None
#         self.latest_depth = None

#         # Config & Files
#         self.pawn_coords_file = os.path.expanduser("~/rs2_ws/src/perception/pawn_coords.txt")
#         self.wall_coords_file = os.path.expanduser("~/rs2_ws/src/perception/wall_coords.txt")
#         self.intrinsics_file = os.path.expanduser("~/rs2_ws/src/perception/camera_intrinsics.json")

#         self.pawn_lookup = {}
#         self.wall_lookup = {}
#         self.load_coordinates()
#         self.load_intrinsics()

#         self.grid_state = np.zeros((5, 5), dtype=int)
#         self.wall_circles = np.zeros((4, 4), dtype=int)

#         cv2.namedWindow("Camera View", cv2.WINDOW_AUTOSIZE)
#         cv2.namedWindow("Top-Down Unified", cv2.WINDOW_AUTOSIZE)
        
#         self.timer = self.create_timer(0.1, self.timer_callback)
#         self.get_logger().info("Perception Node: Visuals + Specific Logging Active.")

#     def load_coordinates(self):
#         try:
#             with open(self.pawn_coords_file, "r") as f:
#                 for line in f:
#                     p = line.strip().split(",")
#                     if len(p) == 5: self.pawn_lookup[(int(p[0]), int(p[1]))] = [float(p[2]), float(p[3]), float(p[4])]
#         except: pass
#         try:
#             with open(self.wall_coords_file, "r") as f:
#                 for line in f:
#                     p = line.strip().split(",")
#                     if len(p) == 5: self.wall_lookup[(int(p[0]), int(p[1]))] = [float(p[2]), float(p[3]), float(p[4])]
#         except: pass

#     def load_intrinsics(self):
#         with open(self.intrinsics_file, "r") as f:
#             intr = json.load(f)
#             self.fx, self.fy, self.ppx, self.ppy = intr["fx"], intr["fy"], intr["ppx"], intr["ppy"]

#     def color_callback(self, msg): self.latest_color = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
#     def depth_callback(self, msg): self.latest_depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')

#     def get_ordered_corners(self, img):
#         gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#         edges = cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0), 50, 150)
#         contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#         if not contours: return None
#         board_cnts = [cnt for cnt in contours if 20000 < cv2.contourArea(cnt) < (img.shape[0]*img.shape[1])]
#         if not board_cnts: return None
#         pts = cv2.approxPolyDP(max(board_cnts, key=cv2.contourArea), 0.02 * cv2.arcLength(max(board_cnts, key=cv2.contourArea), True), True).reshape(4, 2)
#         rect = np.zeros((4, 2), dtype=np.float32)
#         s, d = pts.sum(axis=1), np.diff(pts, axis=1)
#         rect[0], rect[2], rect[1], rect[3] = pts[np.argmin(s)], pts[np.argmax(s)], pts[np.argmin(d)], pts[np.argmax(d)]
#         return rect

#     def process_frame(self, topdown, camera_img, H_inv):
#         # --- PAWNS (BLUE BOXES) ---
#         gray = cv2.cvtColor(topdown, cv2.COLOR_BGR2GRAY)
#         _, black_mask = cv2.threshold(gray, 70, 255, cv2.THRESH_BINARY_INV)
#         p_3d = []
#         self.grid_state[:] = 0
#         for r in range(5):
#             for c in range(5):
#                 x1, y1, x2, y2 = c*100+30, r*100+30, (c+1)*100-30, (r+1)*100-30
#                 roi = black_mask[y1:y2, x1:x2]
#                 if cv2.countNonZero(roi) / (40**2) > 0.05:
#                     self.grid_state[r, c] = 1
#                     cnts, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#                     if cnts:
#                         bx, by, bw, bh = cv2.boundingRect(max(cnts, key=cv2.contourArea))
#                         cv2.rectangle(topdown, (x1+bx, y1+by), (x1+bx+bw, y1+by+bh), (255, 0, 0), 2)
#                         p_cam = cv2.perspectiveTransform(np.array([[x1+bx, y1+by], [x1+bx+bw, y1+by+bh]], dtype='float32').reshape(-1,1,2), H_inv).astype(int)
#                         cv2.rectangle(camera_img, tuple(p_cam[0][0]), tuple(p_cam[1][0]), (255, 0, 0), 2)
#                     if (r, c) in self.pawn_lookup: p_3d.extend([float(r), float(c)] + self.pawn_lookup[(r, c)])

#         # --- WALLS (RED) ---
#         hsv = cv2.cvtColor(topdown, cv2.COLOR_BGR2HSV)
#         red_mask = cv2.bitwise_or(cv2.inRange(hsv, np.array([0,120,70]), np.array([10,255,255])),
#                                   cv2.inRange(hsv, np.array([170,120,70]), np.array([180,255,255])))
#         self.wall_circles[:] = 0
#         w_in_3d = []
#         cnts, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#         for cnt in cnts:
#             if cv2.contourArea(cnt) < 100: continue
#             rx, ry, rw, rh = cv2.boundingRect(cnt)
#             cx_blob, cy_blob = rx + rw//2, ry + rh//2
#             c_idx, r_idx = int(round(cx_blob/100))-1, int(round(cy_blob/100))-1
            
#             if 0 <= r_idx < 4 and 0 <= c_idx < 4:
#                 is_horiz = rw > rh
#                 self.wall_circles[r_idx, c_idx] = 2 if is_horiz else 1
#                 cx_grid, cy_grid, ext = (c_idx+1)*100, (r_idx+1)*100, 25
                
#                 # Visuals (Asterisks + Orientation)
#                 for img_ptr, is_top in [(topdown, True), (camera_img, False)]:
#                     if is_top:
#                         cv2.rectangle(img_ptr, (rx, ry), (rx+rw, ry+rh), (0, 0, 255), 2)
#                         cv2.drawMarker(img_ptr, (cx_blob, cy_blob), (0, 255, 255), cv2.MARKER_STAR, 10, 1)
#                         cv2.drawMarker(img_ptr, (cx_grid, cy_grid), (255, 255, 0), cv2.MARKER_STAR, 12, 2)
#                         if is_horiz: cv2.line(img_ptr, (cx_grid-ext, cy_grid), (cx_grid+ext, cy_grid), (0, 255, 0), 2)
#                         else: cv2.line(img_ptr, (cx_grid, cy_grid-ext), (cx_grid, cy_grid+ext), (0, 255, 0), 2)
#                     else:
#                         pts = np.array([[rx, ry], [rx+rw, ry+rh], [cx_blob, cy_blob], [cx_grid, cy_grid], 
#                                         [cx_grid-ext, cy_grid], [cx_grid+ext, cy_grid], 
#                                         [cx_grid, cy_grid-ext], [cx_grid, cy_grid+ext]], dtype='float32').reshape(-1,1,2)
#                         c_pts = cv2.perspectiveTransform(pts, H_inv).astype(int)
#                         cv2.rectangle(img_ptr, tuple(c_pts[0][0]), tuple(c_pts[1][0]), (0, 0, 255), 2)
#                         cv2.drawMarker(img_ptr, tuple(c_pts[2][0]), (0, 255, 255), cv2.MARKER_STAR, 10, 1)
#                         cv2.drawMarker(img_ptr, tuple(c_pts[3][0]), (255, 255, 0), cv2.MARKER_STAR, 12, 2)
#                         if is_horiz: cv2.line(img_ptr, tuple(c_pts[4][0]), tuple(c_pts[5][0]), (0, 255, 0), 2)
#                         else: cv2.line(img_ptr, tuple(c_pts[6][0]), tuple(c_pts[7][0]), (0, 255, 0), 2)

#                 if (c_idx, r_idx) in self.wall_lookup:
#                     w_in_3d.extend([float(r_idx), float(c_idx)] + self.wall_lookup[(c_idx, r_idx)])

#         return topdown, camera_img, p_3d, w_in_3d

#     def detect_outside(self, img, depth, corners):
#         hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
#         red = cv2.bitwise_or(cv2.inRange(hsv, np.array([0,120,70]), np.array([10,255,255])),
#                              cv2.inRange(hsv, np.array([170,120,70]), np.array([180,255,255])))
#         mask = np.zeros(img.shape[:2], dtype=np.uint8)
#         cv2.fillPoly(mask, [corners.astype(np.int32)], 255)
#         cnts, _ = cv2.findContours(cv2.bitwise_and(red, cv2.bitwise_not(mask)), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#         out_3d = []
#         for cnt in cnts:
#             if cv2.contourArea(cnt) < 150: continue
#             x, y, w, h = cv2.boundingRect(cnt)
#             cx, cy = x+w//2, y+h//2
#             cv2.circle(img, (cx, cy), 10, (0, 0, 255), 2)
#             if depth is not None:
#                 d_m = depth[cy, cx] * 0.001
#                 if d_m > 0:
#                     X, Y = (cx-self.ppx)/self.fx*d_m, (cy-self.ppy)/self.fy*d_m
#                     out_3d.extend([float(X), float(Y), float(d_m)])
#                     self.get_logger().info(f"Outside Wall 3D (cm): x={X*100:.2f}, y={Y*100:.2f}, z={d_m*100:.2f}")
#         return out_3d

#     def timer_callback(self):
#         if self.latest_color is None or self.latest_depth is None: return
#         img, depth = self.latest_color.copy(), self.latest_depth.copy()
#         raw_corners = self.get_ordered_corners(img)
#         if raw_corners is None:
#             cv2.imshow("Camera View", img); cv2.waitKey(1)
#             return

#         H = cv2.getPerspectiveTransform(raw_corners, np.array([[0,0], [500,0], [500,500], [0,500]], dtype=np.float32))
#         H_inv = np.linalg.inv(H)
#         warped = cv2.warpPerspective(img, H, (500, 500))

#         topdown_viz, camera_viz, p_3d, w_in_3d = self.process_frame(warped, img, H_inv)
#         w_out_3d = self.detect_outside(camera_viz, depth, raw_corners)

#         # Logging as requested
#         wall_labels = {0: "Empty", 1: "Vertical", 2: "Horizontal"}
#         self.get_logger().info("\n--- Wall Orientation & Color State ---")
#         for row in self.wall_circles:
#             self.get_logger().info(str([wall_labels[val] for val in row]))

#         board_str = "\n".join(" ".join(str(c) for c in row) for row in self.grid_state)
#         self.get_logger().info(f"\nDigital Board:\n{board_str}")

#         # Visualization finalize
#         cv2.polylines(camera_viz, [raw_corners.astype(np.int32).reshape((-1, 1, 2))], True, (0, 255, 0), 2)
#         for p in raw_corners: cv2.circle(camera_viz, (int(p[0]), int(p[1])), 8, (0, 255, 0), -1)
#         cv2.rectangle(topdown_viz, (0, 0), (500, 500), (0, 255, 0), 3)
#         for cp in [(0,0), (500,0), (500,500), (0,500)]: cv2.circle(topdown_viz, cp, 8, (0, 255, 0), -1)

#         cv2.imshow("Camera View", camera_viz)
#         cv2.imshow("Top-Down Unified", topdown_viz)
#         cv2.waitKey(1)

#         # Publishers
#         self.pub_topdown.publish(self.bridge.cv2_to_imgmsg(topdown_viz, "bgr8"))
#         self.pub_board_state.publish(Int32MultiArray(data=self.grid_state.flatten().tolist()))
#         self.pub_wall_state.publish(Int32MultiArray(data=self.wall_circles.flatten().tolist()))
#         self.pub_pawns_3d.publish(Float32MultiArray(data=p_3d))
#         self.pub_walls_inside_3d.publish(Float32MultiArray(data=w_in_3d))
#         self.pub_walls_outside_3d.publish(Float32MultiArray(data=w_out_3d))
#         self.pub_corners.publish(Float32MultiArray(data=raw_corners.flatten().tolist()))

# def main(args=None):
#     rclpy.init(args=args)
#     node = PerceptionNode()
#     try: rclpy.spin(node)
#     except KeyboardInterrupt: pass
#     finally:
#         cv2.destroyAllWindows()
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__': main()


# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from std_msgs.msg import Int32MultiArray, Float32MultiArray
# import numpy as np
# import cv2
# from cv_bridge import CvBridge
# import os
# import json

# class PerceptionNode(Node):
#     def __init__(self):
#         super().__init__('perception_node')
#         self.bridge = CvBridge()

#         # Publishers
#         self.pub_topdown = self.create_publisher(Image, '/perception/topdown_view', 10)
#         self.pub_board_state = self.create_publisher(Int32MultiArray, '/perception/board_state', 10)
#         self.pub_wall_state = self.create_publisher(Int32MultiArray, '/perception/wall_state', 10)
#         self.pub_pawns_3d = self.create_publisher(Float32MultiArray, '/perception/pawns_3d', 10)
#         self.pub_walls_inside_3d = self.create_publisher(Float32MultiArray, '/perception/walls_inside_3d', 10)
#         self.pub_walls_outside_3d = self.create_publisher(Float32MultiArray, '/perception/walls_outside_3d', 10)
#         self.pub_corners = self.create_publisher(Float32MultiArray, '/perception/board_corners', 10)

#         # Subscriptions
#         self.sub_color = self.create_subscription(Image, '/camera/color', self.color_callback, 10)
#         self.sub_depth = self.create_subscription(Image, '/camera/depth', self.depth_callback, 10)

#         self.latest_color = None
#         self.latest_depth = None

#         # Config & Files
#         self.pawn_coords_file = os.path.expanduser("~/rs2_ws/src/perception/pawn_coords.txt")
#         self.wall_coords_file = os.path.expanduser("~/rs2_ws/src/perception/wall_coords.txt")
#         self.intrinsics_file = os.path.expanduser("~/rs2_ws/src/perception/camera_intrinsics.json")

#         self.pawn_lookup = {}
#         self.wall_lookup = {}
#         self.load_coordinates()
#         self.load_intrinsics()

#         self.grid_state = np.zeros((5, 5), dtype=int)
#         self.wall_circles = np.zeros((4, 4), dtype=int)

#         cv2.namedWindow("Camera View", cv2.WINDOW_AUTOSIZE)
#         cv2.namedWindow("Top-Down Unified", cv2.WINDOW_AUTOSIZE)
        
#         self.timer = self.create_timer(0.1, self.timer_callback)
#         self.get_logger().info("Perception Node: Unified Tracking & Full Visuals Active.")

#     def load_coordinates(self):
#         try:
#             with open(self.pawn_coords_file, "r") as f:
#                 for line in f:
#                     p = line.strip().split(",")
#                     if len(p) == 5: self.pawn_lookup[(int(p[0]), int(p[1]))] = [float(p[2]), float(p[3]), float(p[4])]
#         except: pass
#         try:
#             with open(self.wall_coords_file, "r") as f:
#                 for line in f:
#                     p = line.strip().split(",")
#                     if len(p) == 5: self.wall_lookup[(int(p[0]), int(p[1]))] = [float(p[2]), float(p[3]), float(p[4])]
#         except: pass

#     def load_intrinsics(self):
#         with open(self.intrinsics_file, "r") as f:
#             intr = json.load(f)
#             self.fx, self.fy, self.ppx, self.ppy = intr["fx"], intr["fy"], intr["ppx"], intr["ppy"]

#     def color_callback(self, msg): self.latest_color = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
#     def depth_callback(self, msg): self.latest_depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')

#     def get_ordered_corners(self, img):
#         gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#         edges = cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0), 50, 150)
#         contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#         if not contours: return None
#         board_cnts = [cnt for cnt in contours if 20000 < cv2.contourArea(cnt) < (img.shape[0]*img.shape[1])]
#         if not board_cnts: return None
#         pts = cv2.approxPolyDP(max(board_cnts, key=cv2.contourArea), 0.02 * cv2.arcLength(max(board_cnts, key=cv2.contourArea), True), True).reshape(4, 2)
#         rect = np.zeros((4, 2), dtype=np.float32)
#         s, d = pts.sum(axis=1), np.diff(pts, axis=1)
#         rect[0], rect[2], rect[1], rect[3] = pts[np.argmin(s)], pts[np.argmax(s)], pts[np.argmin(d)], pts[np.argmax(d)]
#         return rect

#     def process_frame(self, topdown, camera_img, H_inv):
#         # --- PAWNS (BLUE BOXES) ---
#         gray = cv2.cvtColor(topdown, cv2.COLOR_BGR2GRAY)
#         _, black_mask = cv2.threshold(gray, 70, 255, cv2.THRESH_BINARY_INV)
#         p_3d = []
#         self.grid_state[:] = 0
#         for r in range(5):
#             for c in range(5):
#                 x1, y1, x2, y2 = c*100+30, r*100+30, (c+1)*100-30, (r+1)*100-30
#                 roi = black_mask[y1:y2, x1:x2]
#                 if cv2.countNonZero(roi) / (40**2) > 0.05:
#                     self.grid_state[r, c] = 1
#                     cnts, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#                     if cnts:
#                         bx, by, bw, bh = cv2.boundingRect(max(cnts, key=cv2.contourArea))
#                         cv2.rectangle(topdown, (x1+bx, y1+by), (x1+bx+bw, y1+by+bh), (255, 0, 0), 2)
#                         p_cam = cv2.perspectiveTransform(np.array([[x1+bx, y1+by], [x1+bx+bw, y1+by+bh]], dtype='float32').reshape(-1,1,2), H_inv).astype(int)
#                         cv2.rectangle(camera_img, tuple(p_cam[0][0]), tuple(p_cam[1][0]), (255, 0, 0), 2)
#                     if (r, c) in self.pawn_lookup: p_3d.extend([float(r), float(c)] + self.pawn_lookup[(r, c)])

#         # --- INSIDE WALLS (RED) ---
#         hsv = cv2.cvtColor(topdown, cv2.COLOR_BGR2HSV)
#         red_mask = cv2.bitwise_or(cv2.inRange(hsv, np.array([0,120,70]), np.array([10,255,255])),
#                                   cv2.inRange(hsv, np.array([170,120,70]), np.array([180,255,255])))
#         self.wall_circles[:] = 0
#         w_in_3d = []
#         cnts, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#         for cnt in cnts:
#             if cv2.contourArea(cnt) < 100: continue
#             rx, ry, rw, rh = cv2.boundingRect(cnt)
#             cx_blob, cy_blob = rx + rw//2, ry + rh//2
#             c_idx, r_idx = int(round(cx_blob/100))-1, int(round(cy_blob/100))-1
            
#             if 0 <= r_idx < 4 and 0 <= c_idx < 4:
#                 is_horiz = rw > rh
#                 self.wall_circles[r_idx, c_idx] = 2 if is_horiz else 1
#                 cx_grid, cy_grid, ext = (c_idx+1)*100, (r_idx+1)*100, 25
                
#                 # Topdown Visuals
#                 cv2.rectangle(topdown, (rx, ry), (rx+rw, ry+rh), (0, 0, 255), 2)
#                 cv2.drawMarker(topdown, (cx_blob, cy_blob), (0, 255, 255), cv2.MARKER_STAR, 10, 1)
#                 cv2.drawMarker(topdown, (cx_grid, cy_grid), (255, 255, 0), cv2.MARKER_STAR, 12, 2)
#                 if is_horiz: cv2.line(topdown, (cx_grid-ext, cy_grid), (cx_grid+ext, cy_grid), (0, 255, 0), 2)
#                 else: cv2.line(topdown, (cx_grid, cy_grid-ext), (cx_grid, cy_grid+ext), (0, 255, 0), 2)

#                 # Camera Mirroring
#                 pts = np.array([[rx, ry], [rx+rw, ry+rh], [cx_blob, cy_blob], [cx_grid, cy_grid], 
#                                 [cx_grid-ext, cy_grid], [cx_grid+ext, cy_grid], 
#                                 [cx_grid, cy_grid-ext], [cx_grid, cy_grid+ext]], dtype='float32').reshape(-1,1,2)
#                 c_pts = cv2.perspectiveTransform(pts, H_inv).astype(int)
#                 cv2.rectangle(camera_img, tuple(c_pts[0][0]), tuple(c_pts[1][0]), (0, 0, 255), 2)
#                 cv2.drawMarker(camera_img, tuple(c_pts[2][0]), (0, 255, 255), cv2.MARKER_STAR, 10, 1)
#                 cv2.drawMarker(camera_img, tuple(c_pts[3][0]), (255, 255, 0), cv2.MARKER_STAR, 12, 2)
#                 if is_horiz: cv2.line(camera_img, tuple(c_pts[4][0]), tuple(c_pts[5][0]), (0, 255, 0), 2)
#                 else: cv2.line(camera_img, tuple(c_pts[6][0]), tuple(c_pts[7][0]), (0, 255, 0), 2)

#                 if (c_idx, r_idx) in self.wall_lookup:
#                     w_in_3d.extend([float(r_idx), float(c_idx)] + self.wall_lookup[(c_idx, r_idx)])

#         return topdown, camera_img, p_3d, w_in_3d

#     def detect_outside_walls(self, img, depth_frame, board_corners):
#         hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
#         red = cv2.bitwise_or(cv2.inRange(hsv, np.array([0, 120, 70]), np.array([10, 255, 255])),
#                              cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255])))

#         # Mask out inside board
#         board_mask = np.zeros(img.shape[:2], dtype=np.uint8)
#         cv2.fillPoly(board_mask, [board_corners.astype(np.int32)], 255)
#         outside_red_mask = cv2.bitwise_and(red, cv2.bitwise_not(board_mask))

#         cnts, _ = cv2.findContours(outside_red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#         outside_data = []
#         ext = 25 

#         for cnt in cnts:
#             if cv2.contourArea(cnt) < 150: continue
#             x, y, w, h = cv2.boundingRect(cnt)
#             cx, cy = x + w // 2, y + h // 2

#             # Visuals: Box + Asterisk + Orientation
#             cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
#             cv2.drawMarker(img, (cx, cy), (0, 255, 255), cv2.MARKER_STAR, 10, 1)
            
#             is_horizontal = w > h
#             if is_horizontal: cv2.line(img, (cx - ext, cy), (cx + ext, cy), (0, 255, 0), 2)
#             else: cv2.line(img, (cx, cy - ext), (cx, cy + ext), (0, 255, 0), 2)

#             # 3D Depth Logic
#             X = Y = Z = 0.0
#             if depth_frame is not None:
#                 depth_val = depth_frame[cy, cx]
#                 if depth_val > 0:
#                     Z = depth_val * 0.001
#                     X = (cx - self.ppx) / self.fx * Z
#                     Y = (cy - self.ppy) / self.fy * Z
#                     outside_data.extend([float(X), float(Y), float(Z)])
#                     self.get_logger().info(f"Outside Wall 3D (cm): x={X*100:.2f}, y={Y*100:.2f}, z={Z*100:.2f}")

#         return outside_data

#     def timer_callback(self):
#         if self.latest_color is None or self.latest_depth is None: return
#         img, depth = self.latest_color.copy(), self.latest_depth.copy()
#         raw_corners = self.get_ordered_corners(img)
#         if raw_corners is None:
#             cv2.imshow("Camera View", img); cv2.waitKey(1)
#             return

#         H = cv2.getPerspectiveTransform(raw_corners, np.array([[0,0], [500,0], [500,500], [0,500]], dtype=np.float32))
#         H_inv = np.linalg.inv(H)
#         warped = cv2.warpPerspective(img, H, (500, 500))

#         # Process Inner Frame
#         topdown_viz, camera_viz, p_3d, w_in_3d = self.process_frame(warped, img, H_inv)
        
#         # Process Outside Walls using your logic
#         w_out_data = self.detect_outside_walls(camera_viz, depth, raw_corners)

#         # Logging
#         wall_labels = {0: "Empty", 1: "Vertical", 2: "Horizontal"}
#         self.get_logger().info("\n--- Wall Orientation & Color State ---")
#         for row in self.wall_circles:
#             self.get_logger().info(str([wall_labels[val] for val in row]))

#         board_str = "\n".join(" ".join(str(c) for c in row) for row in self.grid_state)
#         self.get_logger().info(f"\nDigital Board:\n{board_str}")

#         # Final Board Outlining
#         cv2.polylines(camera_viz, [raw_corners.astype(np.int32).reshape((-1, 1, 2))], True, (0, 255, 0), 2)
#         for p in raw_corners: cv2.circle(camera_viz, (int(p[0]), int(p[1])), 8, (0, 255, 0), -1)
#         cv2.rectangle(topdown_viz, (0, 0), (500, 500), (0, 255, 0), 3)
#         for cp in [(0,0), (500,0), (500,500), (0,500)]: cv2.circle(topdown_viz, cp, 8, (0, 255, 0), -1)

#         cv2.imshow("Camera View", camera_viz)
#         cv2.imshow("Top-Down Unified", topdown_viz)
#         cv2.waitKey(1)

#         # ROS Publishing
#         self.pub_topdown.publish(self.bridge.cv2_to_imgmsg(topdown_viz, "bgr8"))
#         self.pub_board_state.publish(Int32MultiArray(data=self.grid_state.flatten().tolist()))
#         self.pub_wall_state.publish(Int32MultiArray(data=self.wall_circles.flatten().tolist()))
#         self.pub_pawns_3d.publish(Float32MultiArray(data=p_3d))
#         self.pub_walls_inside_3d.publish(Float32MultiArray(data=w_in_3d))
#         self.pub_walls_outside_3d.publish(Float32MultiArray(data=w_out_data))
#         self.pub_corners.publish(Float32MultiArray(data=raw_corners.flatten().tolist()))

# def main(args=None):
#     rclpy.init(args=args)
#     node = PerceptionNode()
#     try: rclpy.spin(node)
#     except KeyboardInterrupt: pass
#     finally:
#         cv2.destroyAllWindows()
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__': main()


#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray, Float32MultiArray
import numpy as np
import cv2
from cv_bridge import CvBridge
import os
import json

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        self.bridge = CvBridge()

        # --- Publishers ---
        # Image Streams
        self.pub_topdown = self.create_publisher(Image, '/perception/topdown', 10)
        self.pub_ar_view = self.create_publisher(Image, '/perception/augmented_reality', 10)
        
        # State & Coordinates
        self.pub_board_state = self.create_publisher(Int32MultiArray, '/perception/board_state', 10)
        self.pub_wall_state = self.create_publisher(Int32MultiArray, '/perception/wall_state', 10)
        self.pub_pawns_3d = self.create_publisher(Float32MultiArray, '/perception/pawns_3d', 10)
        self.pub_walls_inside_3d = self.create_publisher(Float32MultiArray, '/perception/walls_inside_3d', 10)
        self.pub_walls_outside_3d = self.create_publisher(Float32MultiArray, '/perception/walls_outside_3d', 10)
        self.pub_corners = self.create_publisher(Float32MultiArray, '/perception/board_corners', 10)

        # --- Subscriptions ---
        self.sub_color = self.create_subscription(Image, '/camera/color', self.color_callback, 10)
        self.sub_depth = self.create_subscription(Image, '/camera/depth', self.depth_callback, 10)

        self.latest_color = None
        self.latest_depth = None

        # --- Config & Files ---
        self.pawn_coords_file = os.path.expanduser("~/rs2_ws/src/perception/pawn_coords.txt")
        self.wall_coords_file = os.path.expanduser("~/rs2_ws/src/perception/wall_coords.txt")
        self.intrinsics_file = os.path.expanduser("~/rs2_ws/src/perception/camera_intrinsics.json")

        self.pawn_lookup = {}
        self.wall_lookup = {}
        self.load_coordinates()
        self.load_intrinsics()

        self.grid_state = np.zeros((5, 5), dtype=int)
        self.wall_circles = np.zeros((4, 4), dtype=int)

        # GUI Setup
        cv2.namedWindow("AR View", cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow("Top-Down View", cv2.WINDOW_AUTOSIZE)
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Perception Node: Publishing AR and Top-Down streams.")

    def load_coordinates(self):
        try:
            if os.path.exists(self.pawn_coords_file):
                with open(self.pawn_coords_file, "r") as f:
                    for line in f:
                        p = line.strip().split(",")
                        if len(p) == 5: self.pawn_lookup[(int(p[0]), int(p[1]))] = [float(p[2]), float(p[3]), float(p[4])]
            if os.path.exists(self.wall_coords_file):
                with open(self.wall_coords_file, "r") as f:
                    for line in f:
                        p = line.strip().split(",")
                        if len(p) == 5: self.wall_lookup[(int(p[0]), int(p[1]))] = [float(p[2]), float(p[3]), float(p[4])]
        except Exception as e:
            self.get_logger().error(f"Error loading coordinates: {e}")

    def load_intrinsics(self):
        try:
            with open(self.intrinsics_file, "r") as f:
                intr = json.load(f)
                self.fx, self.fy, self.ppx, self.ppy = intr["fx"], intr["fy"], intr["ppx"], intr["ppy"]
        except Exception as e:
            self.get_logger().error(f"Error loading intrinsics: {e}")

    def color_callback(self, msg): self.latest_color = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
    def depth_callback(self, msg): self.latest_depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')

    def get_ordered_corners(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0), 50, 150)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours: return None
        board_cnts = [cnt for cnt in contours if 20000 < cv2.contourArea(cnt) < (img.shape[0]*img.shape[1])]
        if not board_cnts: return None
        pts = cv2.approxPolyDP(max(board_cnts, key=cv2.contourArea), 0.02 * cv2.arcLength(max(board_cnts, key=cv2.contourArea), True), True).reshape(4, 2)
        rect = np.zeros((4, 2), dtype=np.float32)
        s, d = pts.sum(axis=1), np.diff(pts, axis=1)
        rect[0], rect[2], rect[1], rect[3] = pts[np.argmin(s)], pts[np.argmax(s)], pts[np.argmin(d)], pts[np.argmax(d)]
        return rect

    def process_frame(self, topdown, camera_img, H_inv):
        # --- PAWNS (BLUE BOXES) ---
        gray = cv2.cvtColor(topdown, cv2.COLOR_BGR2GRAY)
        _, black_mask = cv2.threshold(gray, 70, 255, cv2.THRESH_BINARY_INV)
        p_3d = []
        self.grid_state[:] = 0
        for r in range(5):
            for c in range(5):
                x1, y1, x2, y2 = c*100+30, r*100+30, (c+1)*100-30, (r+1)*100-30
                roi = black_mask[y1:y2, x1:x2]
                if cv2.countNonZero(roi) / (40**2) > 0.05:
                    self.grid_state[r, c] = 1
                    cnts, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    if cnts:
                        bx, by, bw, bh = cv2.boundingRect(max(cnts, key=cv2.contourArea))
                        cv2.rectangle(topdown, (x1+bx, y1+by), (x1+bx+bw, y1+by+bh), (255, 0, 0), 2)
                        p_cam = cv2.perspectiveTransform(np.array([[x1+bx, y1+by], [x1+bx+bw, y1+by+bh]], dtype='float32').reshape(-1,1,2), H_inv).astype(int)
                        cv2.rectangle(camera_img, tuple(p_cam[0][0]), tuple(p_cam[1][0]), (255, 0, 0), 2)
                    if (r, c) in self.pawn_lookup: p_3d.extend([float(r), float(c)] + self.pawn_lookup[(r, c)])

        # --- INSIDE WALLS (RED) ---
        hsv = cv2.cvtColor(topdown, cv2.COLOR_BGR2HSV)
        red_mask = cv2.bitwise_or(cv2.inRange(hsv, np.array([0,120,70]), np.array([10,255,255])),
                                  cv2.inRange(hsv, np.array([170,120,70]), np.array([180,255,255])))
        self.wall_circles[:] = 0
        w_in_3d = []
        cnts, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in cnts:
            if cv2.contourArea(cnt) < 100: continue
            rx, ry, rw, rh = cv2.boundingRect(cnt)
            cx_blob, cy_blob = rx + rw//2, ry + rh//2
            c_idx, r_idx = int(round(cx_blob/100))-1, int(round(cy_blob/100))-1
            
            if 0 <= r_idx < 4 and 0 <= c_idx < 4:
                is_horiz = rw > rh
                self.wall_circles[r_idx, c_idx] = 2 if is_horiz else 1
                cx_grid, cy_grid, ext = (c_idx+1)*100, (r_idx+1)*100, 25
                
                # Topdown Visuals
                cv2.rectangle(topdown, (rx, ry), (rx+rw, ry+rh), (0, 0, 255), 2)
                cv2.drawMarker(topdown, (cx_blob, cy_blob), (0, 255, 255), cv2.MARKER_STAR, 10, 1)
                cv2.drawMarker(topdown, (cx_grid, cy_grid), (255, 255, 0), cv2.MARKER_STAR, 12, 2)
                if is_horiz: cv2.line(topdown, (cx_grid-ext, cy_grid), (cx_grid+ext, cy_grid), (0, 255, 0), 2)
                else: cv2.line(topdown, (cx_grid, cy_grid-ext), (cx_grid, cy_grid+ext), (0, 255, 0), 2)

                # Camera Mirroring
                pts = np.array([[rx, ry], [rx+rw, ry+rh], [cx_blob, cy_blob], [cx_grid, cy_grid], 
                                [cx_grid-ext, cy_grid], [cx_grid+ext, cy_grid], 
                                [cx_grid, cy_grid-ext], [cx_grid, cy_grid+ext]], dtype='float32').reshape(-1,1,2)
                c_pts = cv2.perspectiveTransform(pts, H_inv).astype(int)
                cv2.rectangle(camera_img, tuple(c_pts[0][0]), tuple(c_pts[1][0]), (0, 0, 255), 2)
                cv2.drawMarker(camera_img, tuple(c_pts[2][0]), (0, 255, 255), cv2.MARKER_STAR, 10, 1)
                cv2.drawMarker(camera_img, tuple(c_pts[3][0]), (255, 255, 0), cv2.MARKER_STAR, 12, 2)
                if is_horiz: cv2.line(camera_img, tuple(c_pts[4][0]), tuple(c_pts[5][0]), (0, 255, 0), 2)
                else: cv2.line(camera_img, tuple(c_pts[6][0]), tuple(c_pts[7][0]), (0, 255, 0), 2)

                if (c_idx, r_idx) in self.wall_lookup:
                    w_in_3d.extend([float(r_idx), float(c_idx)] + self.wall_lookup[(c_idx, r_idx)])

        return topdown, camera_img, p_3d, w_in_3d

    def detect_outside_walls(self, img, depth_frame, board_corners):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        red = cv2.bitwise_or(cv2.inRange(hsv, np.array([0, 120, 70]), np.array([10, 255, 255])),
                             cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255])))

        board_mask = np.zeros(img.shape[:2], dtype=np.uint8)
        cv2.fillPoly(board_mask, [board_corners.astype(np.int32)], 255)
        outside_red_mask = cv2.bitwise_and(red, cv2.bitwise_not(board_mask))

        cnts, _ = cv2.findContours(outside_red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        outside_data = []
        ext = 25 

        for cnt in cnts:
            if cv2.contourArea(cnt) < 150: continue
            x, y, w, h = cv2.boundingRect(cnt)
            cx, cy = x + w // 2, y + h // 2

            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.drawMarker(img, (cx, cy), (0, 255, 255), cv2.MARKER_STAR, 10, 1)
            
            is_horizontal = w > h
            if is_horizontal: cv2.line(img, (cx - ext, cy), (cx + ext, cy), (0, 255, 0), 2)
            else: cv2.line(img, (cx, cy - ext), (cx, cy + ext), (0, 255, 0), 2)

            X = Y = Z = 0.0
            if depth_frame is not None:
                depth_val = depth_frame[cy, cx]
                if depth_val > 0:
                    Z = depth_val * 0.001
                    X = (cx - self.ppx) / self.fx * Z
                    Y = (cy - self.ppy) / self.fy * Z
                    outside_data.extend([float(X), float(Y), float(Z)])
                    self.get_logger().info(f"Outside Wall 3D (cm): x={X*100:.2f}, y={Y*100:.2f}, z={Z*100:.2f}")

        return outside_data

    def timer_callback(self):
        if self.latest_color is None or self.latest_depth is None: return
        img, depth = self.latest_color.copy(), self.latest_depth.copy()
        raw_corners = self.get_ordered_corners(img)
        if raw_corners is None:
            # Publish raw camera view to AR topic if board not found
            self.pub_ar_view.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
            cv2.imshow("AR View", img); cv2.waitKey(1)
            return

        H = cv2.getPerspectiveTransform(raw_corners, np.array([[0,0], [500,0], [500,500], [0,500]], dtype=np.float32))
        H_inv = np.linalg.inv(H)
        warped = cv2.warpPerspective(img, H, (500, 500))

        # Core Processing
        topdown_viz, camera_viz, p_3d, w_in_3d = self.process_frame(warped, img, H_inv)
        w_out_data = self.detect_outside_walls(camera_viz, depth, raw_corners)

        # Logging
        wall_labels = {0: "Empty", 1: "Vertical", 2: "Horizontal"}
        self.get_logger().info("\n--- Wall Orientation & Color State ---")
        for row in self.wall_circles:
            self.get_logger().info(str([wall_labels[val] for val in row]))
        
        board_str = "\n".join(" ".join(str(c) for c in row) for row in self.grid_state)
        self.get_logger().info(f"\nDigital Board:\n{board_str}")

        # Final Overlays
        cv2.polylines(camera_viz, [raw_corners.astype(np.int32).reshape((-1, 1, 2))], True, (0, 255, 0), 2)
        for p in raw_corners: cv2.circle(camera_viz, (int(p[0]), int(p[1])), 8, (0, 255, 0), -1)
        cv2.rectangle(topdown_viz, (0, 0), (500, 500), (0, 255, 0), 3)

        cv2.imshow("AR View", camera_viz)
        cv2.imshow("Top-Down View", topdown_viz)
        cv2.waitKey(1)

        # --- Publishing ---
        # Image Streams
        self.pub_topdown.publish(self.bridge.cv2_to_imgmsg(topdown_viz, "bgr8"))
        self.pub_ar_view.publish(self.bridge.cv2_to_imgmsg(camera_viz, "bgr8"))
        
        # Data Streams
        self.pub_board_state.publish(Int32MultiArray(data=self.grid_state.flatten().tolist()))
        self.pub_wall_state.publish(Int32MultiArray(data=self.wall_circles.flatten().tolist()))
        self.pub_pawns_3d.publish(Float32MultiArray(data=p_3d))
        self.pub_walls_inside_3d.publish(Float32MultiArray(data=w_in_3d))
        self.pub_walls_outside_3d.publish(Float32MultiArray(data=w_out_data))
        self.pub_corners.publish(Float32MultiArray(data=raw_corners.flatten().tolist()))

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': main()