#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import AprilTagDetectionArray

class Target_Follower:
    def _init_(self):
        # Initialize ROS node
        rospy.init_node('target_follower_node', anonymous=True)
        rospy.on_shutdown(self.clean_shutdown)

        # Replace 'akandb' with your Duckiebot's name
        self.robot_name = 'sifra'
        self.cmd_vel_pub = rospy.Publisher(
            f'/{self.robot_name}/car_cmd_switch_node/cmd',
            Twist2DStamped,
            queue_size=1
        )
        rospy.Subscriber(
            f'/{self.robot_name}/apriltag_detector_node/detections',
            AprilTagDetectionArray,
            self.tag_callback,
            queue_size=1
        )

        # Robot state
        self.target_visible = False
        self.last_detection_time = rospy.Time.now()

        # Timer to check if detection was lost
        rospy.Timer(rospy.Duration(0.1), self.search_and_follow)

        rospy.spin()

    def tag_callback(self, msg):
        if len(msg.detections) == 0:
            return
        
        self.target_visible = True
        self.last_detection_time = rospy.Time.now()
        self.follow_target(msg.detections[0])

    def search_and_follow(self, event):
        time_since_last = rospy.Time.now() - self.last_detection_time
        if time_since_last.to_sec() > 0.5:
            self.target_visible = False
            self.rotate_to_search()

    def follow_target(self, detection):
        x = detection.transform.translation.x
        z = detection.transform.translation.z

        rospy.loginfo("Following tag at x: %.2f, z: %.2f", x, z)

        # Control parameters
        forward_gain = 0.5
        angular_gain = 3.0
        stop_distance = 0.25

        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()

        if z > stop_distance:
            cmd_msg.v = forward_gain * z
            cmd_msg.omega = -angular_gain * x
        else:
            cmd_msg.v = 0.0
            cmd_msg.omega = 0.0

        self.cmd_vel_pub.publish(cmd_msg)

    def rotate_to_search(self):
        rospy.loginfo("Searching for target... Rotating.")
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = 2.0  # adjust rotation speed as needed
        self.cmd_vel_pub.publish(cmd_msg)

    def stop_robot(self):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

    def clean_shutdown(self):
        rospy.loginfo("Shutting down. Stopping robot.")
        self.stop_robot()

if _name_ == '_main_':
    try:
        Target_Follower()
    except rospy.ROSInterruptException:
        pass
