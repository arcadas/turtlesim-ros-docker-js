#!/user/bin/python

import rospy
from std_msgs.msg import String

def callback_receive_commands(msg):
    rospy.loginfo(msg)

if __name__ == '__main__':
    rospy.init_node('turtlesim_listener')
    rospy.loginfo("Turtlesim Listener node has been started")
    sub = rospy.Subscriber("/turtlesim_commands", String, callback_receive_commands)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
