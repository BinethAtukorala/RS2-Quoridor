# # perception/perception/ar_node.py
# #!/usr/bin/env python3

# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from std_msgs.msg import Float32MultiArray
# import numpy as np
# import cv2
# from cv_bridge import CvBridge

# class ARNode(Node):
#     def __init__(self):
#         super().__init__('ar_node')
#         self.bridge = CvBridge()

#         # Subscriptions
#         self.sub_color = self.create_subscription(Image, '/camera/color', self.color_callback, 10)
#         self.sub_pawns = self.create_subscription(Float32MultiArray, '/perception/pawns_3d', self.pawns_callback, 10)
#         self.sub_walls_in = self.create_subscription(Float32MultiArray, '/perception/walls_inside_3d', self.walls_in_callback, 10)
#         self.sub_walls_out = self.create_subscription(Float32MultiArray, '/perception/walls_outside_3d', self.walls_out_callback, 10)
#         self.sub_corners = self.create_subscription(Float32MultiArray, '/perception/board_corners', self.corners_callback, 10)

#         # Data storage
#         self.latest_img = None
#         self.corners = None
#         self.pawns = []     # List of [r, c, x, y, z]
#         self.walls_in = []  # List of [r, c, x, y, z]
#         self.walls_out = [] # List of [x, y, z]

#         self.timer = self.create_timer(0.05, self.draw_callback)
#         self.get_logger().info("AR Visualization Node Started")

#     def color_callback(self, msg):
#         self.latest_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

#     def corners_callback(self, msg):
#         self.corners = np.array(msg.data).reshape(-1, 2).astype(np.float32)

#     def pawns_callback(self, msg):
#         # Data is flat: [r, c, x, y, z, r, c, x, y, z...]
#         self.pawns = [msg.data[i:i+5] for i in range(0, len(msg.data), 5)]

#     def walls_in_callback(self, msg):
#         self.walls_in = [msg.data[i:i+5] for i in range(0, len(msg.data), 5)]

#     def walls_out_callback(self, msg):
#         self.walls_out = [msg.data[i:i+3] for i in range(0, len(msg.data), 3)]

#     def draw_callback(self):
#         if self.latest_img is None or self.corners is None:
#             return

#         canvas = self.latest_img.copy()
#         board_size = 500
#         dst = np.array([[0, 0], [board_size, 0], [board_size, board_size], [0, board_size]], dtype=np.float32)
        
#         # H maps Camera -> TopDown. We need H_inv to map TopDown -> Camera
#         H, _ = cv2.findHomography(self.corners, dst)
#         if H is None: return
#         H_inv = np.linalg.inv(H)

#         def get_img_coords(tx, ty):
#             # tx, ty are coordinates in the 500x500 top-down space
#             pts = np.array([[[tx, ty]]], dtype=np.float32)
#             transformed = cv2.perspectiveTransform(pts, H_inv)
#             return tuple(transformed[0][0].astype(int))

#         # 1. Draw Grid Lines
#         for i in range(6):
#             # Vertical lines
#             p1 = get_img_coords(i * 100, 0)
#             p2 = get_img_coords(i * 100, 500)
#             cv2.line(canvas, p1, p2, (0, 255, 0), 1)
#             # Horizontal lines
#             p3 = get_img_coords(0, i * 100)
#             p4 = get_img_coords(500, i * 100)
#             cv2.line(canvas, p3, p4, (0, 255, 0), 1)

#         # 2. Draw Pawns (Bounding Boxes)
#         for pawn in self.pawns:
#             r, c = pawn[0], pawn[1]
#             # Calculate box in top-down then project
#             margin = 35
#             top_left = get_img_coords(c*100 + margin, r*100 + margin)
#             bot_right = get_img_coords((c+1)*100 - margin, (r+1)*100 - margin)
#             cv2.rectangle(canvas, top_left, bot_right, (255, 0, 0), 2) # Blue for pawns
#             cv2.putText(canvas, "Pawn", top_left, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 1)

#         # 3. Draw Inside Walls
#         ext = 25
#         for wall in self.walls_in:
#             r, c = int(wall[1]), int(wall[0]) # From circle node logic
#             cx, cy = (r + 1) * 100, (c + 1) * 100
#             # Note: You'd need orientation (H/V) passed in the msg to draw lines correctly.
#             # For now, we draw the circle centroid.
#             center = get_img_coords(cx, cy)
#             cv2.drawMarker(canvas, center, (0, 255, 255), cv2.MARKER_STAR, 10, 2)

#         # 4. Draw Outside Walls (These are already in image coords if using the detect_outside_walls logic)
#         # However, since you publish 3D coords, we project them back if we have intrinsics, 
#         # or simply rely on the circle node's visual logic. 
#         # If the Circle node doesn't publish pixel coords, you'll need to add them to the message.

#         cv2.imshow("Master AR View", canvas)
#         cv2.waitKey(1)

# def main(args=None):
#     rclpy.init(args=args)
#     node = ARNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
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
# from std_msgs.msg import Float32MultiArray
# import numpy as np
# import cv2
# from cv_bridge import CvBridge

# class ARNode(Node):
#     def __init__(self):
#         super().__init__('ar_node')
#         self.bridge = CvBridge()

#         # Subscribers
#         self.create_subscription(Image, '/camera/color', self.color_callback, 10)
#         self.create_subscription(Float32MultiArray, '/perception/pawns_3d', self.pawns_callback, 10)
#         self.create_subscription(Float32MultiArray, '/perception/walls_inside_3d', self.walls_in_callback, 10)
#         self.create_subscription(Float32MultiArray, '/perception/board_corners', self.corners_callback, 10)

#         self.latest_img = None
#         self.corners = None
#         self.pawns = []    # [r, c, x, y, z]
#         self.walls_in = [] # [r, c, x, y, z, orient, w, h]

#         self.timer = self.create_timer(0.05, self.draw_callback)

#     def color_callback(self, msg):
#         self.latest_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

#     def corners_callback(self, msg):
#         self.corners = np.array(msg.data).reshape(-1, 2).astype(np.float32)

#     def pawns_callback(self, msg):
#         self.pawns = [msg.data[i:i+5] for i in range(0, len(msg.data), 5)]

#     def walls_in_callback(self, msg):
#         # Updated to step by 8 based on new data structure
#         self.walls_in = [msg.data[i:i+8] for i in range(0, len(msg.data), 8)]

#     def draw_callback(self):
#         if self.latest_img is None or self.corners is None:
#             return

#         canvas = self.latest_img.copy()
#         board_size = 500
#         dst = np.array([[0, 0], [board_size, 0], [board_size, board_size], [0, board_size]], dtype=np.float32)
        
#         H, _ = cv2.findHomography(self.corners, dst)
#         if H is None: return
#         H_inv = np.linalg.inv(H)

#         def project(tx, ty):
#             pts = np.array([[[tx, ty]]], dtype=np.float32)
#             transformed = cv2.perspectiveTransform(pts, H_inv)
#             return tuple(transformed[0][0].astype(int))

#         # 1. Draw Grid Separation Lines
#         for i in range(6):
#             cv2.line(canvas, project(i*100, 0), project(i*100, 500), (0, 255, 0), 1)
#             cv2.line(canvas, project(0, i*100), project(500, i*100), (0, 255, 0), 1)

#         # 2. Draw Pawns (Blue Bbox)
#         for p in self.pawns:
#             r, c = p[0], p[1]
#             m = 30 # margin
#             pts = np.array([
#                 [c*100+m, r*100+m], [(c+1)*100-m, r*100+m], 
#                 [(c+1)*100-m, (r+1)*100-m], [c*100+m, (r+1)*100-m]
#             ], dtype=np.float32).reshape(-1, 1, 2)
#             proj_pts = cv2.perspectiveTransform(pts, H_inv).astype(np.int32)
#             cv2.polylines(canvas, [proj_pts], True, (255, 0, 0), 2)

#         # 3. Draw Walls (Inside Board)
#         for w in self.walls_in:
#             r, c, orient = int(w[0]), int(w[1]), int(w[5])
#             cx_td, cy_td = (c + 1) * 100, (r + 1) * 100 # Centroid in top-down
            
#             # Draw Orientation Line (mapped from top-down)
#             ext = 25
#             if orient == 2: # Horizontal
#                 p1, p2 = project(cx_td - ext, cy_td), project(cx_td + ext, cy_td)
#             else: # Vertical
#                 p1, p2 = project(cx_td, cy_td - ext), project(cx_td, cy_td + ext)
            
#             cv2.line(canvas, p1, p2, (0, 255, 255), 3) # Thick yellow line

#             # Draw Wall Bounding Box
#             # Defining box points in Top-Down then projecting
#             ww, wh = (60, 15) if orient == 2 else (15, 60)
#             box = np.array([
#                 [cx_td - ww, cy_td - wh], [cx_td + ww, cy_td - wh],
#                 [cx_td + ww, cy_td + wh], [cx_td - ww, cy_td + wh]
#             ], dtype=np.float32).reshape(-1, 1, 2)
#             proj_box = cv2.perspectiveTransform(box, H_inv).astype(np.int32)
#             cv2.polylines(canvas, [proj_box], True, (0, 0, 255), 2)

#         cv2.imshow("AR Augmented View", canvas)
#         cv2.waitKey(1)

# def main(args=None):
#     rclpy.init(args=args)
#     node = ARNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         cv2.destroyAllWindows()
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
import numpy as np
import cv2
from cv_bridge import CvBridge

class ARNode(Node):
    def __init__(self):
        super().__init__('ar_node')
        self.bridge = CvBridge()

        # Subscribers for the visualization feeds
        self.sub_grid_viz = self.create_subscription(Image, '/perception/topdown_grid', self.grid_viz_callback, 10)
        self.sub_circle_viz = self.create_subscription(Image, '/perception/topdown_circles', self.circle_viz_callback, 10)
        self.sub_color = self.create_subscription(Image, '/camera/color', self.color_callback, 10)
        self.sub_corners = self.create_subscription(Float32MultiArray, '/perception/board_corners', self.corners_callback, 10)

        self.latest_grid_viz = None
        self.latest_circle_viz = None
        self.latest_color = None
        self.corners = None

        self.timer = self.create_timer(0.05, self.process_ar)
        self.get_logger().info("AR Node: Pixel-Merge Mode Active")

    def grid_viz_callback(self, msg):
        self.latest_grid_viz = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def circle_viz_callback(self, msg):
        self.latest_circle_viz = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def color_callback(self, msg):
        self.latest_color = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def corners_callback(self, msg):
        # Flattened [tl_x, tl_y, tr_x, tr_y, br_x, br_y, bl_x, bl_y]
        self.corners = np.array(msg.data).reshape(-1, 2).astype(np.float32)

    def process_ar(self):
        if any(x is None for x in [self.latest_grid_viz, self.latest_circle_viz, self.latest_color, self.corners]):
            return

        # 1. Combine the two Top-Down visualizations
        # We use cv2.addWeighted or a bitwise approach. 
        # Since the background of both is the board, we can just take the maximum pixel values
        # to ensure the red/green/yellow lines from both nodes show up.
        combined_topdown = cv2.max(self.latest_grid_viz, self.latest_circle_viz)

        # 2. Prepare Homography
        board_size = 500
        dst_pts = np.array([[0, 0], [board_size, 0], [board_size, board_size], [0, board_size]], dtype=np.float32)
        
        # We want to warp from Top-Down (500x500) back to Camera View
        H_rev, _ = cv2.findHomography(dst_pts, self.corners)

        # 3. Warp the combined visualization back to the original perspective
        h, w, _ = self.latest_color.shape
        warped_viz = cv2.warpPerspective(combined_topdown, H_rev, (w, h))

        # 4. Overlay the warped visualization onto the original camera feed
        # We create a mask of the warped viz (where it's not black) so we don't overwrite the whole image
        viz_gray = cv2.cvtColor(warped_viz, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(viz_gray, 1, 255, cv2.THRESH_BINARY)
        mask_inv = cv2.bitwise_not(mask)

        # Black out the area of the board in the original image
        img_bg = cv2.bitwise_and(self.latest_color, self.latest_color, mask=mask_inv)
        # Take only the viz elements
        img_fg = cv2.bitwise_and(warped_viz, warped_viz, mask=mask)

        # Combine
        final_ar = cv2.add(img_bg, img_fg)

        cv2.imshow("Final Combined AR Feed", final_ar)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ARNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()