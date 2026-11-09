
import cv2
import numpy as np

WIDTH = 640
HEIGHT = 480

data = np.fromfile("CAM640.BIN", dtype=np.uint8)
frame = data.reshape(HEIGHT, WIDTH, 2)

# Standard limited-range UYVY
img_limited = cv2.cvtColor(
    frame, cv2.COLOR_YUV2BGR_UYVY
)
cv2.imwrite("CAM640_LIMITED.png", img_limited)

# Try full-range YUV conversion
img_full = cv2.cvtColor(
    frame, cv2.COLOR_YUV2BGR_UYVY
)
img_full = np.clip(
    (img_full.astype(np.float32) - 16) * (255.0 / 219.0),
    0, 255
).astype(np.uint8)

cv2.imwrite("CAM640_FULL.png", img_full)

print("Saved both conversion versions")
