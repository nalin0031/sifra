#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import os

class LaneDetector:
    def _init_(self):
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber('/sifra/camera_node/image/compressed', CompressedImage, self.callback)
        self.video_white = cv2.VideoWriter('white_filter.avi', cv2.VideoWriter_fourcc(*'XVID'), 10, (640, 480))
        self.video_yellow = cv2.VideoWriter('yellow_filter.avi', cv2.VideoWriter_fourcc(*'XVID'), 10, (640, 480))
        self.video_hough = cv2.VideoWriter('hough_lines.avi', cv2.VideoWriter_fourcc(*'XVID'), 10, (640, 480))

    def callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # White mask
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 30, 255])
        mask_white = cv2.inRange(hsv, lower_white, upper_white)
        white_result = cv2.bitwise_and(image, image, mask=mask_white)
        self.video_white.write(white_result)
        cv2.imshow("White Filter", white_result)

        # Yellow mask
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        yellow_result = cv2.bitwise_and(image, image, mask=mask_yellow)
        self.video_yellow.write(yellow_result)
        cv2.imshow("Yellow Filter", yellow_result)

        # Hough Lines on combined mask
        combined_mask = cv2.bitwise_or(mask_white, mask_yellow)
        edges = cv2.Canny(combined_mask, 50, 150)
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=50, minLineLength=50, maxLineGap=50)
        hough_image = self.output_lines(image, lines)
        self.video_hough.write(hough_image)
        cv2.imshow("Hough Transform", hough_image)

        cv2.waitKey(1)

    def output_lines(self, original_image, lines):
        output = np.copy(original_image)
        if lines is not None:
            for i in range(len(lines)):
                l = lines[i][0]
                cv2.line(output, (l[0], l[1]), (l[2], l[3]), (255, 0, 0), 2, cv2.LINE_AA)
                cv2.circle(output, (l[0], l[1]), 2, (0, 255, 0))
                cv2.circle(output, (l[2], l[3]), 2, (0, 0, 255))
        return output

    def _del_(self):
        self.video_white.release()
        self.video_yellow.release()
        self.video_hough.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('lane_detector', anonymous=True)
    LaneDetector()
    rospy.spin()
