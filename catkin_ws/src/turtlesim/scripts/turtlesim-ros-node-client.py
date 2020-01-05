#!/usr/bin/python

import termios, sys, tty
import rospy
from std_msgs.msg import String
from std_msgs.srv import Empty
from turtlesim.srv import Velocity, BgColor

class TurtlesimClient:

    def __init__(self):
        self.rate = rospy.Rate(rospy.get_param("/rate"))
        self.pub = rospy.Publisher("/turtlesim_commands", String, queue_size=10)
        self.show_info()
        self.init_stdin()
        self.wait_services()

        while not rospy.is_shutdown():
            cmd = self.read_cmd()
            self.process_cmd(cmd)
            self.rate.sleep()

        rospy.loginfo("Node was stopped")

    def show_info(self):
        rospy.loginfo("""Turtlesim Client node has been started
Commands:
- W: forward
- S: backward
- A: left
- D: right
- R: reset
- B: background
- V: velocity
- Q: quit""")

    def init_stdin(self):
        self.fd = sys.stdin.fileno()
        self.old_stdin_settings = termios.tcgetattr(self.fd)

    def wait_services(self):
        rospy.wait_for_service("/reset")
        rospy.wait_for_service("/velocity")
        rospy.wait_for_service("/bg_color")

    def read_cmd(self):
        try:
            tty.setraw(self.fd)
            cmd = sys.stdin.read(1)
        finally:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_stdin_settings)
        rospy.loginfo(cmd)
        return cmd

    def process_cmd(self):
        if cmd == 'q':
            rospy.signal_shutdown("Quit")
        elif cmd == 'r':
            self.call_reset()
        elif cmd == 'v':
            velocity = int(raw_input('Velocity (0-100): '))
            self.call_velocity(velocity)
        elif cmd == 'b':
            colors = raw_input('Colors (0-255,0-255,0-255): ')
            self.call_bg_color(colors.split(','))
        elif cmd in ['w','s','a','d']:
            self.publish(cmd)
        else:
            rospy.loginfo("Wrong command")

    def call_reset(self):
        self.call_service("/reset", Empty, [])

    def call_velocity(self, velocity):
        self.call_service("/velocity", Velocity, [velocity])

    def call_bg_color(self, colors):
        self.call_service("/bg_color", BgColor, colors)

    def call_service(self, name, type, params):
        try:
            fn = rospy.ServiceProxy(name, type)
            fn(*params)
        except rospy.ServiceException as e:
            rospy.logwarn("Service failed: " + str(e))

    def publish(self, cmd):
        msg = String()
        msg.data = cmd
        self.pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('turtlesim_client')
    TurtlesimClient()
