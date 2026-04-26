#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import pyrealsense2 as rs
import numpy as np
from cv_bridge import CvBridge
import json
import os
from quoridor_interfaces.srv import GetCoords

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.bridge = CvBridge()

        # Publishers
        self.pub_color = self.create_publisher(Image, '/camera/color', 10)
        self.pub_depth = self.create_publisher(Image, '/camera/depth', 10)

        # RealSense setup
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # Bag file parameter
        self.declare_parameter('bag_file', '')
        bag_file = self.get_parameter('bag_file').get_parameter_value().string_value
        self.is_bag = bag_file != ""

        if self.is_bag:
            self.get_logger().info(f"Using BAG file: {bag_file}")
            rs.config.enable_device_from_file(self.config, bag_file, repeat_playback=True)
        else:
            self.get_logger().info("Using LIVE camera")
            self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
            self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)

        self.profile = self.pipeline.start(self.config)

        if self.is_bag:
            device = self.profile.get_device()
            playback = device.as_playback()
            playback.set_real_time(False)

        # Path to save intrinsics
        # self.intrinsics_file = os.path.expanduser("/ros2_ws/src/perception/camera_intrinsics.json")
        self.intrinsics_file = os.path.expanduser("~/rs2_ws/src/perception/camera_intrinsics.json")

        # Save intrinsics only if file does not exist
        if not os.path.exists(self.intrinsics_file):
            self.get_logger().info("Saving camera intrinsics to file...")
            color_stream = self.profile.get_stream(rs.stream.color).as_video_stream_profile()
            intr = color_stream.get_intrinsics()
            intrinsics_data = {
                "fx": intr.fx,
                "fy": intr.fy,
                "ppx": intr.ppx,
                "ppy": intr.ppy,
                "width": intr.width,
                "height": intr.height
            }
            with open(self.intrinsics_file, "w") as f:
                json.dump(intrinsics_data, f, indent=4)
            self.get_logger().info(f"Intrinsics saved to {self.intrinsics_file}")
        else:
            self.get_logger().info("Intrinsics file already exists, skipping save.")

        self.pawn_file = os.path.expanduser("/rs2_ws/src/perception/pawn_coords.txt")
        self.wall_file = os.path.expanduser("/rs2_ws/src/perception/wall_coords.txt")

        self.wall_service = self.create_service(
            GetCoords,
            '/get_walls',
            self.handle_get_wall_coords
        )

        self.pawn_service = self.create_service(
            GetCoords,
            '/get_pawns',
            self.handle_get_pawn_coords
        )

        self.get_logger().info("Camera Node Started")
        self.timer = self.create_timer(0.033, self.timer_callback)

    def handle_get_wall_coords(self, request, response):
        data_array = []

        if os.path.exists(self.wall_file):
            with open(self.wall_file, 'r') as f:
                for line in f:
                    parts = line.strip().split(',')
                    if len(parts) == 5:
                        data_array.extend([float(p) for p in parts])

        response.data = data_array
        return response

    def handle_get_pawn_coords(self, request, response):
        data_array = []

        if os.path.exists(self.pawn_file):
            with open(self.pawn_file, 'r') as f:
                for line in f:
                    parts = line.strip().split(',')
                    if len(parts) == 5:
                        data_array.extend([float(p) for p in parts])

        response.data = data_array
        return response

    def timer_callback(self):
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
        except RuntimeError as e:
            if self.is_bag:
                self.get_logger().info("Bag file finished.")
                rclpy.shutdown()
            else:
                self.get_logger().warn(f"Frame timeout: {e}")
            return

        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            return

        color = np.asanyarray(color_frame.get_data())
        depth = np.asanyarray(depth_frame.get_data())

        msg_color = self.bridge.cv2_to_imgmsg(color, encoding='bgr8')
        msg_depth = self.bridge.cv2_to_imgmsg(depth, encoding='passthrough')

        self.pub_color.publish(msg_color)
        self.pub_depth.publish(msg_depth)


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pipeline.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()