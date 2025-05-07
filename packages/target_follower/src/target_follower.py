#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import AprilTagDetectionArray, Twist2DStamped

class TargetFollower:
    def __init__(self):
        rospy.init_node('target_follower_node')

        self.cmd_pub = rospy.Publisher(
            '/sifra/car_cmd_switch_node/cmd',
            Twist2DStamped,
            queue_size=1
        )

        rospy.Subscriber(
            '/sifra/apriltag_detector_node/detections',
            AprilTagDetectionArray,
            self.callback
        )

        self.twist = Twist2DStamped()
        self.seeking = True

        rospy.loginfo("[INFO] TargetFollower node initialized")
        self.run()

    def callback(self, data):
        if not data.detections:
            rospy.loginfo("[INFO] No detections. Seeking object...")
            self.seek_object()
            return

        rospy.loginfo("[INFO] AprilTag detected")
        self.seeking = False

        tag = data.detections[0]
        x_offset = tag.transform.translation.x
        rospy.loginfo(f"[INFO] x_offset: {x_offset:.2f}")

        self.twist.header.stamp = rospy.Time.now()
        self.twist.v = 0.0
        self.twist.omega = max(min(2.0 * (-x_offset), 1.0), -1.0)

        rospy.loginfo(f"[INFO] Sending cmd with omega: {self.twist.omega:.2f}")
        self.cmd_pub.publish(self.twist)

    def seek_object(self):
        self.seeking = True
        self.twist.header.stamp = rospy.Time.now()
        self.twist.v = 0.0
        self.twist.omega = 2.0  # rotation speed for searching
        rospy.loginfo(f"[INFO] Seeking... sending cmd with omega: {self.twist.omega:.2f}")
        self.cmd_pub.publish(self.twist)

    def stop_robot(self):
        self.twist.header.stamp = rospy.Time.now()
        self.twist.v = 0.0
        self.twist.omega = 0.0
        rospy.loginfo("[INFO] Stopping robot...")
        self.cmd_pub.publish(self.twist)

    def run(self):
        rospy.on_shutdown(self.stop_robot)
        rospy.spin()

if __name__ == '__main__':
    TargetFollower()
