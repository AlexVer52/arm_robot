## It should do:
##     threshold color
##     find contours
##     estimate shape

import cv2
import numpy as np

class OpenCVDetect:
    def __init__(self, detection_threshold=0.5):
        self.detection_threshold = detection_threshold

        self.color_dict = {
            "red": {"hsv_low": (0, 50, 40), "hsv_high": (10, 255, 255)},
            "green": {"hsv_low": (35, 50, 40), "hsv_high": (85, 255, 255)},
            "blue": {"hsv_low": (100, 50, 40), "hsv_high": (140, 255, 255)},
            "yellow": {"hsv_low": (20, 50, 40), "hsv_high": (35, 255, 255)},
        }
        
    def detect(self, image, depth_image, camera_info):
        # Convert the image to HSV color space
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        detections: list[dict] = []
        detections_pose: list[dict] = []

        ## Camera Info
        k = camera_info.k
        fx = k[0]
        fy = k[4]
        cx0 = k[2]
        cy0 = k[5]

        for _, hsv_range in self.color_dict.items():
            mask = cv2.inRange(hsv_image, hsv_range["hsv_low"], hsv_range["hsv_high"])
            color = list(self.color_dict.keys())[list(self.color_dict.values()).index(hsv_range)]
            # Find contours in the masked image
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                if cv2.contourArea(contour) > self.detection_threshold:
                    # Approximate the contour to a polygon
                    epsilon = 0.02 * cv2.arcLength(contour, True)
                    approx = cv2.approxPolyDP(contour, epsilon, True)

                    # Determine the shape based on the number of vertices
                    if len(approx) == 3:
                        shape = "Triangle"
                    elif len(approx) == 4:
                        shape = "Rectangle"
                    elif len(approx) > 4:
                        shape = "Circle"
                    else:
                        shape = "Unknown"

                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = M["m10"] / M["m00"]
                        cy = M["m01"] / M["m00"]
                    else:
                        cx, cy = 0, 0

                    rect = cv2.minAreaRect(contour)
                    angle = rect[2]
                    if angle < -45:
                        angle += 90

                    if color == "red":
                        detections.append(
                            {
                                "color": str(color),
                                "shape": shape,
                                "cx": float(cx),
                                "cy": float(cy),
                            }
                        )

        for detection in detections:
            x = int(detection["cx"])
            y = int(detection["cy"])
            x_norm = (x - cx0) / fx
            y_norm = (y - cy0) / fy
            depth_value = depth_image[y, x]
            if depth_value == 0:
                continue

            X = x_norm * depth_value
            Y = y_norm * depth_value
            Z = depth_value

            detections_pose.append(
                {
                    "color": detection["color"],
                    "shape": detection["shape"],
                    "x": float(X),
                    "y": float(Y),
                    "z": float(Z),
                }
            )
        
        return detections_pose