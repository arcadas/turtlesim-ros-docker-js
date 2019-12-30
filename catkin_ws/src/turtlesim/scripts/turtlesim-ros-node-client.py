#!/usr/bin/python

import termios, sys, tty
import rospy
from std_msgs.msg import String

if __name__ == '__main__':

    rospy.init_node('turtlesim_client')
    rospy.loginfo("Turtlesim Client node has been started")
    rospy.loginfo("Commands: ")
    rospy.loginfo("- W: forward")
    rospy.loginfo("- S: backward")
    rospy.loginfo("- A: left")
    rospy.loginfo("- D: right")
    rospy.loginfo("- R: reset")
    rospy.loginfo("- Q: quit")

    pub = rospy.Publisher("/turtlesim_commands", String, queue_size=10)
    rate = rospy.Rate(2)

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    cmds = ['w','s','a','d','r','q']

    while not rospy.is_shutdown():
        try:
            tty.setraw(fd)
            cmd = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

        if cmd in cmds:
            if cmd == 'q':
                rospy.signal_shutdown("Quit")
            else:
                rospy.loginfo(cmd)
                msg = String()
                msg.data = cmd
                pub.publish(msg)
                rate.sleep()
        else:
            rospy.loginfo("Wrong command!")

    rospy.loginfo("Node was stopped")
