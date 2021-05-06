#!/usr/bin/env python

import rospy
# import the Twist message for publishing velocity commands:
from geometry_msgs.msg import Twist
# import the Odometry message for subscribing to the odom topic:
from nav_msgs.msg import Odometry
# import the function to convert orientation from quaternions to angles:
from tf.transformations import euler_from_quaternion
# import some useful mathematical operations (and pi), which you may find useful:
import math
import colourMasks

class move_square:

    def callback_function(self, odom_data):
        # obtain the orientation co-ords:
        orientation = odom_data.pose.pose.orientation

        # obtain the position co-ords:
        pos = odom_data.pose.pose.position

        # convert orientation co-ords to roll, pitch & yaw (theta_x, theta_y, theta_z):
        (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w], 'sxyz')

        # We are only interested in the x, y and theta_z odometry data for this
        # robot, so we only assign these to class variables (so that we can
        # access them elsewhere within our move_square class):
        self.x = pos.x
        self.y = pos.y
        self.theta_z = yaw

        # If this is the first time that this callback_function has run, then
        # obtain the "initial robot pose"
        if self.startup:
            # don't initialise again:
            self.startup = False
            # set the robot starting position:
            self.x0 = self.x
            self.y0 = self.y
            self.theta_z0 = self.theta_z

    def __init__(self):
        # a flag if this node has just been launched
        self.startup = True

        self.turn_complete = False

        # setup a cmd_vel publisher and an odom subscriber:
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/odom', Odometry, self.callback_function)

        rospy.init_node('moveturn_node', anonymous=True)
        self.rate = rospy.Rate(10) # hz

        # define the robot pose variables and set them all to zero to start with:
        self.x = 0.0
        self.y = 0.0
        self.theta_z = 0.0
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta_z0 = 0.0

        # define a Twist instance, which can be used to set robot velocities
        self.vel = Twist()

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        rospy.loginfo("the moveturn node has been initialised...")

    def shutdownhook(self):
        self.shutdown_function()
        self.ctrl_c = True

    def shutdown_function(self):
        # publish an empty twist message to stop the robot (by default all
        # velocities within this will be zero):
        self.pub.publish(Twist())

    def print_stuff(self):
        # a function to print information to the terminal (use as you wish):
        # print the message that has been passed in to the method via the "a_message" input:
        # print(a_message)
        # you could use this to print the current velocity command:
        #print("current velocity: lin.x = {:.1f}, ang.z = {:.1f}".format(self.vel.linear.x, self.vel.angular.z))
        # you could also print the current odometry to the terminal here, if you wanted to:
        #print("current theta_z = {:.3f}".format(self.theta_z))

    def main_loop(self):
        while not self.ctrl_c:
            if not self.turn_complete:
                self.vel.angular.z = 0.3
                print(math.degrees(self.theta_z))
                if self.theta_z < 0:
                    self.vel.angular.z = 0
                    print("turn has been completed")
                    self.turn_complete = True

            # publish whatever velocity command has been set in your code above:
            self.pub.publish(self.vel)
            # call a function which prints some information to the terminal:
            self.print_stuff()
            # maintain the loop rate @ 10 hz
            self.rate.sleep()

if __name__ == '__main__':
    movesquare_instance = move_square()
    try:
        movesquare_instance.main_loop()
    except rospy.ROSInterruptException:
        pass
