#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def move_square():
    rospy.init_node('square_turtle', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    move_cmd = Twist()

    rate = rospy.Rate(10)  # 10 Hz
    linear_speed = 1.0
    angular_speed = 1.57   # approx 90 degrees/sec
    side_time = 2.0        # forward duration in seconds
    turn_time = 1.0        # turning 90 degrees

    while not rospy.is_shutdown():
        for _ in range(4):
            # Move forward
            move_cmd.linear.x = linear_speed
            move_cmd.angular.z = 0.0
            pub.publish(move_cmd)
            rospy.sleep(side_time)

            # Pause
            move_cmd.linear.x = 0.0
            pub.publish(move_cmd)
            rospy.sleep(0.3)

            # Turn 90 degrees
            move_cmd.angular.z = angular_speed
            pub.publish(move_cmd)
            rospy.sleep(turn_time)

            # Pause
            move_cmd.angular.z = 0.0
            pub.publish(move_cmd)
            rospy.sleep(0.3)

if __name__ == '__main__':
    try:
        move_square()
    except rospy.ROSInterruptException:
        pass
