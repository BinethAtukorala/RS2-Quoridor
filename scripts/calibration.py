#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np


class HandEyeCalibration(Node):

    def __init__(self):
        super().__init__("hand_eye_calibration_node")

    # ---------------------------------------------------------

    def capture_image(self):

    # ---------------------------------------------------------

    def detect_checkerboard(self, image):

    # ---------------------------------------------------------

    def compute_camera_pose(self, corners):

    # ---------------------------------------------------------

    def add_robot_pose(self, pose):

    # ---------------------------------------------------------

    def perform_hand_eye_calibration(self):

    # ---------------------------------------------------------

    def save_calibration(self, R, t):



def main(args=None):

    rclpy.init(args=args)

    node = HandEyeCalibration()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()