#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
import cv2
import pyrealsense2 as rs


class QuoridorPerception(Node):

    def __init__(self):

        super().__init__("quoridor_perception_node")

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


def main(args=None):

    rclpy.init(args=args)

    node = QuoridorPerception()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()
    

if __name__ == "__main__":
    main()