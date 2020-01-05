#!/usr/bin/python

import rospy
from std_msgs.srv import Empty
from turtlesim.srv import Velocity, BgColor

class TurtlesimService:

    def __init__(self):
        rospy.loginfo("Turtlesim Service server node has been started")
        rospy.Service("/bg_color", BgColor, self.handle_bg_color)
        rospy.Service("/velocity", Velocity, self.handle_velocity)
        rospy.Service("/reset", Empty, self.handle_reset)

    def handle_bg_color(req):
        rospy.loginfo("Color: " +
            str(req.bg_color_g) + ", " +
            str(req.bg_color_b) + ", " +
            str(req.bg_color_r))
        return true

    def handle_velocity(req):
        rospy.loginfo("Velocity: " + str(req.velocity))
        return true

    def handle_reset(req):
        rospy.loginfo("Reset")

if __name__ = '__main__':
  rospy.init_node("turtlesim_service")
  TurtlesimService()
  rospy.spin()
