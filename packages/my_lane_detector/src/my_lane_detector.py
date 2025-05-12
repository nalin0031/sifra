#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

class LaneDetector:
    def __init__(self):
        rospy.init_node('lane_detector_node', anonymous=True)
        self.bridge = CvBridge()
        rospy.Subscriber('/sifra/camera_node/image/compressed', CompressedImage, self.image_callback)
        rospy.on_shutdown(self.cleanup)
        rospy.spin()

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        bgr_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        height, width = bgr_image.shape[:2]
        cropped = bgr_image[int(height/2):, :]
        hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)

        lower_white = np.array([0, 0, 160])
        upper_white = np.array([180, 60, 255])
        white_mask = cv2.inRange(hsv, lower_white, upper_white)
        white_result = cv2.bitwise_and(cropped, cropped, mask=white_mask)

        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        yellow_result = cv2.bitwise_and(cropped, cropped, mask=yellow_mask)

        white_edges = cv2.Canny(white_result, 50, 150)
        yellow_edges = cv2.Canny(yellow_result, 50, 150)

        white_lines = cv2.HoughLinesP(white_edges, 1, np.pi/180, threshold=40, minLineLength=50, maxLineGap=10)
        yellow_lines = cv2.HoughLinesP(yellow_edges, 1, np.pi/180, threshold=40, minLineLength=50, maxLineGap=10)

        final_output = self.draw_lines(cropped.copy(), white_lines, (255, 255, 255))
        final_output = self.draw_lines(final_output, yellow_lines, (255, 255, 0))

        cv2.imshow("White Filtered", white_result)
        cv2.imshow("Yellow Filtered", yellow_result)
        cv2.imshow("Final with Hough Lines", final_output)
        cv2.waitKey(1)

    def draw_lines(self, img, lines, color):
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(img, (x1, y1), (x2, y2), color, 2)
        return img

    def cleanup(self):
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        LaneDetector()
    except rospy.ROSInterruptException:
        pass
