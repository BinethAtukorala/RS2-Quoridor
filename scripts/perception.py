#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
import cv2
import pyrealsense2 as rs


# class QuoridorPerception(Node):
class QuoridorPerception:


    def __init__(self):

        super().__init__("quoridor_perception_node")
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.width = width
        self.height = height
        self.fps = fps
 
        self.is_bag = False
 
        if bag_file:
            rs.config.enable_device_from_file(self.config, bag_file)
            self.is_bag = True
        else:
        # Enable streams
            self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
            self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
 
        profile = self.pipeline.start(self.config)
 
 
        color_sensor = profile.get_device().first_color_sensor()
 
        target_white_balance = 3400  # e.g., 4500 K for neutral indoor lighting
        color_sensor.set_option(rs.option.white_balance, target_white_balance)
        color_sensor.set_option(rs.option.enable_auto_exposure, True)
 
        self.decimation = rs.decimation_filter()
        self.spatial = rs.spatial_filter()
        self.temporal = rs.temporal_filter()
        self.hole_filling = rs.hole_filling_filter()
       
        # Default HSV ranges for colors
        self.color_ranges = {
            "red": ([170, 70, 50], [180, 255, 255]),
            "green": ([40, 70, 70], [80, 255, 255]),
            "blue": ([100, 120, 80], [130, 255, 255])
        }

    # ---------------------------------------------------------

    def capture_frames(self):

    # ---------------------------------------------------------

    def filter_image(self, image):

    # ---------------------------------------------------------

    def detect_board(self, image):

    # ---------------------------------------------------------

    def detect_pawns(self, image):

    # ---------------------------------------------------------

    def detect_walls(self, image):

    # ---------------------------------------------------------

    def compute_3D_point(self, pixel, depth, intrinsics):

    # ---------------------------------------------------------

    def camera_to_robot(self, point_cam):

    # ---------------------------------------------------------

    def run_pipeline(self):


    def detect_white_squares(self, color_image, depth_image, intrinsics, min_area_px=1500):
        """
        Detect white squares on the Quoridor board and compute their 3D coordinates.

        Returns:
            color_image : annotated image
            mask : binary mask of detected white regions
            detections : list containing
                - pixel center
                - 3D coordinate in camera frame
                - contour
                - area
        """

        font = cv2.FONT_HERSHEY_SIMPLEX

        # Convert to HSV
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        # White colour range (low saturation, high value)
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 50, 255])

        mask = cv2.inRange(hsv, lower_white, upper_white)

        # Morphological filtering to remove noise
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detections = []

        for contour in contours:

            area = cv2.contourArea(contour)

            if area < min_area_px:
                continue

            # Approximate contour shape
            epsilon = 0.04 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            # Keep only quadrilateral shapes (squares)
            if len(approx) != 4:
                continue

            x, y, w, h = cv2.boundingRect(contour)

            aspect_ratio = float(w) / h

            # Ensure shape is roughly square
            if aspect_ratio < 0.7 or aspect_ratio > 1.3:
                continue

            center = (int(x + w/2), int(y + h/2))

            depth = depth_image[center[1], center[0]]

            if depth == 0:
                continue

            # Convert pixel to 3D coordinate
            point_3d = rs.rs2_deproject_pixel_to_point(
                intrinsics,
                [center[0], center[1]],
                depth
            )

            # Draw detection
            cv2.drawContours(color_image, [approx], -1, (0,255,0), 2)
            cv2.circle(color_image, center, 3, (0,255,0), -1)

            cv2.putText(
                color_image,
                str(np.round(point_3d,3)),
                (center[0], center[1]-10),
                font,
                0.4,
                (0,255,0),
                1
            )

            detections.append({
                "center_px": center,
                "point_3d": point_3d,
                "area_px": area
            })

        return color_image, mask, detections

def main(args=None):

    rclpy.init(args=args)

    node = QuoridorPerception()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()
    

if __name__ == "__main__":
    main()