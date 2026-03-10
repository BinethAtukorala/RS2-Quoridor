#!/usr/bin/env python3

import pyrealsense2 as rs
import numpy as np
import cv2

# ============================================================
# DIGITAL BOARD REPRESENTATION
# ============================================================

class QuoridorBoard:

    def __init__(self):

        # 5x5 board squares
        self.board = np.zeros((5, 5), dtype=int)

        # horizontal walls (4x5)
        self.h_walls = np.zeros((4, 5), dtype=int)

        # vertical walls (5x4)
        self.v_walls = np.zeros((5, 4), dtype=int)

    def clear(self):
        self.board[:] = 0

    def place_pawn(self, player, row, col):
        if 0 <= row < 5 and 0 <= col < 5:
            self.board[row, col] = player

    def print_board(self):
        print("\nDigital Board:\n")
        for r in range(5):
            for c in range(5):
                print(self.board[r, c], end=" ")
            print()
        print()


# ============================================================
# REALSENSE CAMERA
# ============================================================

class RealsenseCamera:

    def __init__(self):

        self.width = 1280
        self.height = 720
        self.fps = 30

        self.pipeline = rs.pipeline()
        config = rs.config()

        config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
        config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)

        profile = self.pipeline.start(config)

        depth_stream = profile.get_stream(rs.stream.depth)
        self.intrinsics = depth_stream.as_video_stream_profile().get_intrinsics()

        print("RealSense camera started")

    def get_frames(self):

        frames = self.pipeline.wait_for_frames()

        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            return None, None

        depth = np.asanyarray(depth_frame.get_data())
        color = np.asanyarray(color_frame.get_data())

        return color, depth


# ============================================================
# BOARD DETECTION
# ============================================================

def detect_board_corners(image):

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5,5), 0)
    edges = cv2.Canny(blur, 50, 150)

    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) == 0:
        return None

    largest = max(contours, key=cv2.contourArea)
    epsilon = 0.02 * cv2.arcLength(largest, True)
    approx = cv2.approxPolyDP(largest, epsilon, True)

    if len(approx) != 4:
        return None

    corners = approx.reshape(4,2)
    return corners


# ============================================================
# HOMOGRAPHY
# ============================================================

def compute_homography(corners):
    board_size = 500  # smaller board for 5x5

    dst = np.array([
        [0,0],
        [board_size,0],
        [board_size,board_size],
        [0,board_size]
    ], dtype=np.float32)

    src = np.array(corners, dtype=np.float32)

    H, _ = cv2.findHomography(src, dst)
    return H


def warp_board(image, H):
    board_size = 500
    warped = cv2.warpPerspective(image, H, (board_size, board_size))
    return warped


# ============================================================
# GRID OVERLAY
# ============================================================

def draw_grid(board_img):
    cell = 100  # 500 / 5 = 100 pixels per cell
    for i in range(6):  # 5x5 grid requires 6 lines
        x = i * cell
        cv2.line(board_img, (x,0), (x,500), (0,255,0), 1)
        cv2.line(board_img, (0,x), (500,x), (0,255,0), 1)
    return board_img


# ============================================================
# GRID CONVERSION
# ============================================================

def pixel_to_grid(x, y):
    cell = 100
    col = int(x / cell)
    row = int(y / cell)
    return row, col


# ============================================================
# SIMPLE PAWN DETECTION (placeholder)
# ============================================================

def detect_pawns(board_img):
    hsv = cv2.cvtColor(board_img, cv2.COLOR_BGR2HSV)
    lower_red = np.array([0, 120, 70])
    upper_red = np.array([10, 255, 255])
    mask = cv2.inRange(hsv, lower_red, upper_red)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    pawns = []

    for c in contours:
        area = cv2.contourArea(c)
        if area < 100:
            continue
        x, y, w, h = cv2.boundingRect(c)
        center = (int(x+w/2), int(y+h/2))
        pawns.append(center)
        cv2.circle(board_img, center, 6, (0,0,255), -1)

    return pawns


# ============================================================
# MAIN LOOP
# ============================================================

def main():

    camera = RealsenseCamera()
    board = QuoridorBoard()

    while True:

        color, depth = camera.get_frames()

        if color is None:
            continue

        corners = detect_board_corners(color)

        if corners is not None:

            for p in corners:
                cv2.circle(color, tuple(p), 6, (0,255,0), -1)

            H = compute_homography(corners)
            topdown = warp_board(color, H)
            topdown = draw_grid(topdown)

            pawns = detect_pawns(topdown)

            board.clear()
            for pawn in pawns:
                row, col = pixel_to_grid(pawn[0], pawn[1])
                board.place_pawn(1, row, col)

            board.print_board()
            cv2.imshow("Top Down Board", topdown)

        cv2.imshow("Camera", color)

        key = cv2.waitKey(1)
        if key == 27:
            break

    camera.pipeline.stop()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()