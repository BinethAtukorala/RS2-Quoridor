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
        self.pawn_coords_file = os.path.expanduser("/rs2_ws/src/perception/pawn_coords.txt")
        self.wall_coords_file = os.path.expanduser("/rs2_ws/src/perception/wall_coords.txt")
        self.intrinsics_file = os.path.expanduser("/rs2_ws/src/perception/camera_intrinsics.json")

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
        best = max(board_cnts, key=cv2.contourArea)
        approx = cv2.approxPolyDP(best, 0.02 * cv2.arcLength(best, True), True)
        if len(approx) != 4:
            return None
        pts = approx.reshape(4, 2)
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
        self.get_logger().info("\n--- Wall State ---")
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