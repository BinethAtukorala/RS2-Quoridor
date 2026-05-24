#!/usr/bin/env python3

import pyrealsense2 as rs
import numpy as np
import cv2


class QuoridorPerception:

    def __init__(self):

        self.width = 1280
        self.height = 720
        self.fps = 30

        # Start RealSense pipeline
        self.pipeline = rs.pipeline()
        config = rs.config()

        config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
        config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)

        profile = self.pipeline.start(config)

        # Get camera intrinsics
        depth_stream = profile.get_stream(rs.stream.depth)
        self.intrinsics = depth_stream.as_video_stream_profile().get_intrinsics()

        print("Camera started")

    # ---------------------------------------------------------

    def get_frames(self):

        frames = self.pipeline.wait_for_frames()

        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            return None, None

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        return color_image, depth_image

    # ---------------------------------------------------------

    def detect_white_squares(self, color_image, depth_image):

        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 50, 255])

        mask = cv2.inRange(hsv, lower_white, upper_white)

        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detections = []

        for contour in contours:

            area = cv2.contourArea(contour)

            if area < 1500:
                continue

            epsilon = 0.04 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            if len(approx) != 4:
                continue

            x, y, w, h = cv2.boundingRect(contour)

            aspect_ratio = float(w)/h
            if aspect_ratio < 0.7 or aspect_ratio > 1.3:
                continue

            center = (int(x+w/2), int(y+h/2))

            depth = depth_image[center[1], center[0]]

            if depth == 0:
                continue

            # Convert to 3D coordinate
            point_3d = rs.rs2_deproject_pixel_to_point(
                self.intrinsics,
                [center[0], center[1]],
                depth
            )

            print("Square 3D position:", np.round(point_3d,3))

            cv2.drawContours(color_image, [approx], -1, (0,255,0), 2)
            cv2.circle(color_image, center, 5, (0,255,0), -1)

            cv2.putText(
                color_image,
                str(np.round(point_3d,3)),
                center,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (0,255,0),
                1
            )

        return color_image, mask


# ---------------------------------------------------------

def main():

    perception = QuoridorPerception()

    while True:

        color, depth = perception.get_frames()

        if color is None:
            continue

        annotated, mask = perception.detect_white_squares(color, depth)

        cv2.imshow("RGB", annotated)
        cv2.imshow("Mask", mask)

        key = cv2.waitKey(1)

        if key == 27:  # ESC
            break

    perception.pipeline.stop()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()