#!/usr/bin/env python3

import pyrealsense2 as rs
import numpy as np
import cv2

# ============================================================
# DIGITAL BOARD REPRESENTATION
# ============================================================

class QuoridorBoard:

    def __init__(self):
        # 5x5 squares: 0 = empty, 1 = pawn
        self.board = np.zeros((5, 5), dtype=int)

        # 4x4 wall spots (circles between squares): 0 = empty, 1 = wall placed
        self.wall_circles = np.zeros((4, 4), dtype=int)

        # horizontal walls (4x5)
        self.h_walls = np.zeros((4, 5), dtype=int)

        # vertical walls (5x4)
        self.v_walls = np.zeros((5, 4), dtype=int)

    def clear(self):
        self.board[:] = 0
        self.wall_circles[:] = 0
        self.h_walls[:] = 0
        self.v_walls[:] = 0

    def place_pawn(self, player, row, col):
        if 0 <= row < 5 and 0 <= col < 5:
            self.board[row, col] = player

    def place_wall(self, row, col):
        """Mark a wall circle as occupied"""
        if 0 <= row < 4 and 0 <= col < 4:
            self.wall_circles[row, col] = 1

    def print_board(self):
        print("\n5x5 Squares (0=empty,1=pawn):")
        for r in range(5):
            print(" ".join(str(self.board[r, c]) for c in range(5)))

        print("\n4x4 Wall Circles (0=empty,1=wall):")
        for r in range(4):
            print(" ".join(str(self.wall_circles[r, c]) for c in range(4)))
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
    board_size = 500  # pixels
    dst = np.array([[0,0],[board_size,0],[board_size,board_size],[0,board_size]], dtype=np.float32)
    src = np.array(corners, dtype=np.float32)
    H, _ = cv2.findHomography(src, dst)
    return H

def warp_board(image, H):
    board_size = 500
    warped = cv2.warpPerspective(image, H, (board_size, board_size))
    return warped


# ============================================================
# GRID AND WALL CIRCLES OVERLAY
# ============================================================

def draw_grid_and_circles(board_img):
    board_size = 500
    cell = board_size // 5  # 100 pixels per square
    radius = 8  # circle radius for wall spots

    # Draw squares
    for i in range(6):
        x = i*cell
        cv2.line(board_img, (x,0), (x,board_size), (0,255,0), 1)
        cv2.line(board_img, (0,x), (board_size,x), (0,255,0), 1)

    # Draw wall circles (4x4)
    for r in range(4):
        for c in range(4):
            cx = (c+1)*cell
            cy = (r+1)*cell
            cv2.circle(board_img, (cx, cy), radius, (255,0,0), 2)

    return board_img


# ============================================================
# WALL DETECTION (placeholder)
# ============================================================

def detect_walls(board_img):
    """
    Placeholder: detect if a circle disappears (wall placed)
    Returns list of coordinates where walls exist
    """
    # In practice: threshold around each circle's position to see if white circle disappeared
    walls = []
    # Example: just return empty for now
    return walls


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
            topdown = draw_grid_and_circles(topdown)

            # Update wall state (placeholder)
            wall_positions = detect_walls(topdown)
            for row, col in wall_positions:
                board.place_wall(row, col)

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