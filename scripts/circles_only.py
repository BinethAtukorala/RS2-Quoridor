# #!/usr/bin/env python3

# import pyrealsense2 as rs
# import numpy as np
# import cv2

# # ---------------------------
# # PARAMETERS
# # ---------------------------
# CELL_SIZE = 100
# CIRCLE_RADIUS = 8
# BOARD_SIZE = 500  # pixels
# ROWS, COLS = 4, 4  # 4x4 circles

# # ---------------------------
# # SETUP REALSENSE
# # ---------------------------
# pipeline = rs.pipeline()
# config = rs.config()
# config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
# pipeline.start(config)
# print("RealSense started")

# # ---------------------------
# # WALL CIRCLE ARRAY
# # ---------------------------
# wall_circles = np.zeros((ROWS, COLS), dtype=int)  # 0 = circle visible, 1 = wall present

# # ---------------------------
# # MAIN LOOP
# # ---------------------------
# while True:
#     frames = pipeline.wait_for_frames()
#     color_frame = frames.get_color_frame()
#     if not color_frame:
#         continue
#     img = np.asanyarray(color_frame.get_data())
#     topdown = cv2.resize(img, (BOARD_SIZE, BOARD_SIZE))

#     # Reset circles
#     wall_circles[:] = 0

#     hsv = cv2.cvtColor(topdown, cv2.COLOR_BGR2HSV)

#     # White circle mask
#     lower_white = np.array([0, 0, 200])
#     upper_white = np.array([180, 50, 255])
#     white_mask = cv2.inRange(hsv, lower_white, upper_white)

#     # Red wall mask
#     lower_red1 = np.array([0, 120, 70])
#     upper_red1 = np.array([10, 255, 255])
#     lower_red2 = np.array([170, 120, 70])
#     upper_red2 = np.array([180, 255, 255])
#     red_mask = cv2.inRange(hsv, lower_red1, upper_red1)
#     red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
#     red_mask = cv2.bitwise_or(red_mask, red_mask2)

#     # Loop through all 4x4 circle positions
#     for r in range(ROWS):
#         for c in range(COLS):
#             cx = (c + 1) * CELL_SIZE
#             cy = (r + 1) * CELL_SIZE

#             # Small crop around the circle
#             x1, y1 = max(cx - CIRCLE_RADIUS, 0), max(cy - CIRCLE_RADIUS, 0)
#             x2, y2 = min(cx + CIRCLE_RADIUS, BOARD_SIZE), min(cy + CIRCLE_RADIUS, BOARD_SIZE)

#             # Check if circle is present (white mask)
#             circle_area = white_mask[y1:y2, x1:x2]
#             if np.mean(circle_area) > 50:  # still visible
#                 wall_circles[r, c] = 0
#                 cv2.circle(topdown, (cx, cy), CIRCLE_RADIUS, (255, 0, 0), 2)  # blue circle
#             else:
#                 # Check if red wall is present
#                 red_area = red_mask[y1:y2, x1:x2]
#                 if np.mean(red_area) > 50:
#                     wall_circles[r, c] = 1
#                     cv2.rectangle(topdown, (x1, y1), (x2, y2), (0, 0, 255), 2)  # red rectangle
#                 else:
#                     # If circle gone but no red detected, mark as unknown (still 0)
#                     wall_circles[r, c] = 0

#     print("Wall Circles (0=circle visible,1=wall):")
#     print(wall_circles)

#     cv2.imshow("Circles & Walls", topdown)
#     key = cv2.waitKey(1)
#     if key == 27:
#         break

# pipeline.stop()
# cv2.destroyAllWindows()

#!/usr/bin/env python3
import pyrealsense2 as rs
import numpy as np
import cv2

# ============================================================
# QUORIDOR WALL CIRCLE DETECTOR
# ============================================================

class QuoridorWalls:

    def __init__(self, board_size=500, rows=4, cols=4, cell_size=100):
        # Digital representation of circles
        self.rows = rows
        self.cols = cols
        self.board_size = board_size
        self.cell_size = cell_size
        self.wall_circles = np.zeros((rows, cols), dtype=int)  # 0=circle visible,1=wall

        # Realsense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        self.pipeline.start(self.config)
        print("RealSense camera started")

    # ----------------------------
    # BOARD CORNERS DETECTION
    # ----------------------------
    def detect_board_corners(self, image):
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

    # ----------------------------
    # HOMOGRAPHY
    # ----------------------------
    def compute_homography(self, corners):
        dst = np.array([
            [0,0],
            [self.board_size,0],
            [self.board_size,self.board_size],
            [0,self.board_size]
        ], dtype=np.float32)
        src = np.array(corners, dtype=np.float32)
        H, _ = cv2.findHomography(src, dst)
        return H

    def warp_board(self, image, H):
        return cv2.warpPerspective(image, H, (self.board_size, self.board_size))

    # ----------------------------
    # DETECT CIRCLES AND WALLS
    # ----------------------------
    def detect_circles_and_walls(self, warped_img):
        hsv = cv2.cvtColor(warped_img, cv2.COLOR_BGR2HSV)

        # White circle mask
        lower_white = np.array([0,0,200])
        upper_white = np.array([180,50,255])
        white_mask = cv2.inRange(hsv, lower_white, upper_white)

        # Red wall mask
        lower_red1 = np.array([0,120,70])
        upper_red1 = np.array([10,255,255])
        lower_red2 = np.array([170,120,70])
        upper_red2 = np.array([180,255,255])
        red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)

        # Reset digital board
        self.wall_circles[:] = 0

        for r in range(self.rows):
            for c in range(self.cols):
                cx = (c+1) * self.cell_size
                cy = (r+1) * self.cell_size
                x1, y1 = max(cx-8,0), max(cy-8,0)
                x2, y2 = min(cx+8,self.board_size), min(cy+8,self.board_size)

                # Crop around circle
                circle_crop = white_mask[y1:y2, x1:x2]
                if np.mean(circle_crop) > 50:
                    self.wall_circles[r,c] = 0
                    cv2.circle(warped_img, (cx, cy), 8, (255,0,0), 2)  # blue circle
                else:
                    # Check for red wall
                    red_crop = red_mask[y1:y2, x1:x2]
                    if np.mean(red_crop) > 50:
                        self.wall_circles[r,c] = 1
                        cv2.rectangle(warped_img, (x1, y1), (x2, y2), (0,0,255), 2)
                    else:
                        self.wall_circles[r,c] = 0  # circle missing but no wall

        return warped_img

    # ----------------------------
    # MAIN LOOP
    # ----------------------------
    def run(self):
        while True:
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            img = np.asanyarray(color_frame.get_data())
            corners = self.detect_board_corners(img)
            if corners is None:
                cv2.imshow("Camera", img)
                key = cv2.waitKey(1)
                if key == 27:
                    break
                continue

            for p in corners:
                cv2.circle(img, tuple(p), 6, (0,255,0), -1)

            H = self.compute_homography(corners)
            warped = self.warp_board(img, H)
            warped = self.detect_circles_and_walls(warped)

            print("Wall Circles (0=circle,1=wall):")
            print(self.wall_circles)

            cv2.imshow("TopDown Circles & Walls", warped)
            cv2.imshow("Camera", img)
            key = cv2.waitKey(1)
            if key == 27:
                break

        self.pipeline.stop()
        cv2.destroyAllWindows()


# ----------------------------
# RUN
# ----------------------------
if __name__ == "__main__":
    detector = QuoridorWalls()
    detector.run()