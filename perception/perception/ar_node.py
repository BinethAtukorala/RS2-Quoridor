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