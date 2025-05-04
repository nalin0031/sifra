#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, FSMState

class Drive_Square:
    def __init__(self):
        self.cmd_msg = Twist2DStamped()
        rospy.init_node('drive_square_node', anonymous=True)
        self.pub = rospy.Publisher('/sifra/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/sifra/fsm_node/mode', FSMState, self.fsm_callback, queue_size=1)
        self.running = False

    def fsm_callback(self, msg):
        rospy.loginfo("FSM state: %s", msg.state)
        if msg.state == "NORMAL_JOYSTICK_CONTROL":
            self.stop_robot()
        elif msg.state == "LANE_FOLLOWING" and not self.running:
            rospy.sleep(1)
            self.running = True
            self.move_robot()
            self.running = False

    def stop_robot(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
        rospy.loginfo("Robot stopped.")

    def run(self):
        rospy.spin()

    def move_robot(self):
        # Tuned values for floor rug
        linear_speed = 0.28
        angular_speed = 2.3
        straight_time = 2.0
        turn_time = 1.15

        for i in range(4):
            rospy.loginfo(f"Side {i+1}: Moving forward")
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.cmd_msg.v = linear_speed
            self.cmd_msg.omega = 0.0
            self.pub.publish(self.cmd_msg)
            rospy.sleep(straight_time)

            rospy.loginfo(f"Corner {i+1}: Turning")
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.cmd_msg.v = 0.0
            self.cmd_msg.omega = angular_speed
            self.pub.publish(self.cmd_msg)
            rospy.sleep(turn_time)

        self.stop_robot()
        rospy.loginfo("Finished drawing square.")

if __name__ == '__main__':
    try:
        duckiebot_movement = Drive_Square()
        duckiebot_movement.run()
    except rospy.ROSInterruptException:
        pass

