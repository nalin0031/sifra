#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import time

def move_square():
    rospy.init_node('square_turtle', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    move_cmd = Twist()
    rate = rospy.Rate(1)

    side_length = 2.0  # seconds moving forward
    turn_time = 1.0    # seconds turning 90 degrees

    for _ in range(40):
        # Move forward
        move_cmd.linear.x = 2.0
        move_cmd.angular.z = 0.0
        pub.publish(move_cmd)
        time.sleep(side_length)

        # Stop
        move_cmd.linear.x = 0.0
        pub.publish(move_cmd)
        time.sleep(0.5)

        # Turn 90 degrees
        move_cmd.angular.z = 1.57
        pub.publish(move_cmd)
        time.sleep(turn_time)

        # Stop
        move_cmd.angular.z = 0.0
        pub.publish(move_cmd)
        time.sleep(0.5)

    # Stop turtle finally
    move_cmd.linear.x = 0.0
    move_cmd.angular.z = 0.0
    pub.publish(move_cmd)

if __name__ == '__main__':
    try:
        move_square()
    except rospy.ROSInterruptException:
        pass
