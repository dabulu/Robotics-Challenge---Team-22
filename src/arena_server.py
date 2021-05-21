#! /usr/bin/python

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import actionlib

# Import all the necessary ROS message types:
from com2009_srv_examples.srv import TimedMovement, TimedMovementResponse
from com2009_actions.msg import SearchFeedback, SearchResult, SearchAction
from sensor_msgs.msg import LaserScan

# Import some other modules from within this package (copied from other package)
from move_tb3 import MoveTB3
from tb3_odometry import TB3Odometry

# Import some other useful Python Modules
from math import radians, pi
import datetime as dt
import os
import numpy as np
import copy

class SearchAS(object):
    feedback = SearchFeedback()
    result = SearchResult()

    def __init__(self):
        # Initialise action server
        self.actionserver = actionlib.SimpleActionServer("/arena_action_server",
            SearchAction, self.action_server_launcher, auto_start=False)
        self.actionserver.start()

        # Lidar subscriber
        self.lidar_subscriber = rospy.Subscriber('/scan', LaserScan, self.callback_lidar)
        self.lidar = {'range': 0.0,
                      'wider range': 0.0,
                      'closest': 0.0,
                      'closest angle': 0}

        # Robot movement and odometry
        self.robot_controller = MoveTB3()
        self.robot_odom = TB3Odometry()

        self.current_yaw = 0.0

    def callback_lidar(self, lidar_data):
        """Returns arrays of lidar data"""

        raw_data = np.array(lidar_data.ranges)

        # Arc size
        angle_tolerance = 12
        # The closest obstacle in an arc directly in front of the robot
        self.lidar['range'] = min(min(raw_data[:angle_tolerance]),
                               min(raw_data[-angle_tolerance:]))

        # Wider arc size
        wider_tolerance = 30
        # The closest obstacle in a wider arc directly in front of the robot
        self.lidar['wider range'] = min(min(raw_data[:wider_tolerance]),
                               min(raw_data[-wider_tolerance:]))

        # Closest object to any side of the robot
        self.lidar['closest'] = min(raw_data)

        # Angle of the closest object to any side of the robot
        self.lidar['closest angle']=raw_data.argmin()

    def move_back(self):
        # Back up from a wall if it's too close
        self.robot_controller.set_move_cmd(linear=-0.2)
        while self.lidar['range'] <= 0.2:
            self.robot_controller.publish()
        self.robot_controller.stop()

    def avoid_wall(self):
        self.robot_controller.stop()

        # Turn away from a close wall until it's not longer close
        if self.lidar['closest'] <= 0.15 and self.lidar['closest angle'] < 90:
            while self.lidar['closest'] <= 0.15:
                self.robot_controller.set_move_cmd(linear=-0.2, angular=0.4)
                self.robot_controller.publish()
        elif self.lidar['closest'] <= 0.15 and self.lidar['closest angle'] < 180:
            while self.lidar['closest'] <= 0.15:
                self.robot_controller.set_move_cmd(linear=0.2, angular=-0.4)
                self.robot_controller.publish()
        elif self.lidar['closest'] <= 0.15 and self.lidar['closest angle'] < 270:
            while self.lidar['closest'] <= 0.15:
                self.robot_controller.set_move_cmd(linear=0.2, angular=0.4)
                self.robot_controller.publish()
        elif self.lidar['closest'] <= 0.15 and self.lidar['closest angle'] < 360:
            while self.lidar['closest'] <= 0.15:
                self.robot_controller.set_move_cmd(linear=-0.2, angular=-0.4)
                self.robot_controller.publish()
        self.robot_controller.stop()

    def action_server_launcher(self, goal):
        r = rospy.Rate(10)

        success = True

        # Checking if set robot speed and obstacle stopping distance is appropriate
        if goal.fwd_velocity <= 0 or goal.fwd_velocity > 0.26:
            print("Invalid fwd_velocity {:.2f}: 0 < fwd_velocity < 0.26".format(goal.fwd_velocity))
            success = False
        if goal.approach_distance <=0:
            print("approach_distance must be > 0 (requested {:.2f})".format(goal.approach_distance))
            success = False

        if not success:
            self.actionserver.set_aborted()
            return

        print("Request to move at {:.2f} m/s until {:.2f} m away from object".format(
            goal.fwd_velocity, goal.approach_distance))

        # Get the current robot odometry
        ref_x = self.robot_odom.posx
        start_x = self.robot_odom.posx

        ref_y = self.robot_odom.posy
        start_y = self.robot_odom.posy
        distance_travelled = 0.0

        # Starting Time and Cancel Boolean
        StartTime = rospy.get_rostime()
        self.cancel = False

        # Main Navigation Loop
        while rospy.get_rostime().secs - StartTime.secs < 210 and self.cancel == False:
            # set the robot velocity:
            self.robot_controller.set_move_cmd(linear=goal.fwd_velocity)

            # Move forward while there is not an obstacle in front of the robot
            while self.lidar['range'] >= goal.approach_distance:
                self.robot_controller.publish()
                # check if there has been a request to cancel the action mid-way through:
                if self.actionserver.is_preempt_requested():
                    rospy.loginfo('Cancelling the movement request.')
                    self.actionserver.set_preempted()
                    # stop the robot:
                    self.robot_controller.stop()
                    success = False
                    self.cancel = True
                    # exit the loop:
                    break

                # Calculate distance travelled
                distance_travelled = np.sqrt(pow(start_x-ref_x, 2) + pow(start_y-ref_y, 2))

                # populate the feedback message and publish it:
                rospy.loginfo('Current distance to object: {:.2f} m'.format(self.lidar['range']))
                rospy.loginfo('{} seconds have passed'.format(rospy.get_rostime().secs - StartTime.secs))
                self.feedback.current_distance_travelled = distance_travelled
                self.actionserver.publish_feedback(self.feedback)

                # update the reference odometry:
                ref_x = self.robot_odom.posx
                ref_y = self.robot_odom.posy

            self.robot_controller.stop()
            # Periodic checking to see whether it's near a wall
            self.avoid_wall()

            previous_dir = 0.0
            self.turn_again = True

            # Setting speed to turn away from the closest obstacle
            if self.lidar['closest angle'] > 180:
                self.robot_controller.set_move_cmd(angular=0.6)
                previous_dir = 0.6
            elif self.lidar['closest angle'] <= 180:
                self.robot_controller.set_move_cmd(angular=-0.6)
                previous_dir = -0.6

            # Turn away from the closest obstacle at most just above 90 degrees
            self.current_yaw = copy.deepcopy(self.robot_odom.yaw2)
            while abs(self.current_yaw - self.robot_odom.yaw2) < pi/1.8:
                if self.lidar['wider range'] >= 0.5:
                    self.robot_controller.stop()
                    self.turn_again = False
                    break
                self.robot_controller.publish()
                if self.actionserver.is_preempt_requested():
                    rospy.loginfo('Cancelling the movement request.')
                    self.actionserver.set_preempted()
                    # stop the robot:
                    self.robot_controller.stop()
                    success = False
                    self.cancel = True
                    self.turn_again = False
                    # exit the loop:
                    break
            self.robot_controller.stop()
            # Periodic checking to see whether it's near a wall
            self.avoid_wall()


            # Turn in the other direction until the robot sees a path
            if previous_dir > 0.0:
                self.robot_controller.set_move_cmd(angular=-0.6)
            else:
                self.robot_controller.set_move_cmd(angular=0.6)
            self.current_yaw = copy.deepcopy(self.robot_odom.yaw2)
            while self.turn_again == True:
                if self.lidar['wider range'] >= 0.5:
                    self.robot_controller.stop()
                    self.turn_again = False
                    break
                self.robot_controller.publish()
                if self.actionserver.is_preempt_requested():
                    rospy.loginfo('Cancelling the movement request.')
                    self.actionserver.set_preempted()
                    # stop the robot:
                    self.robot_controller.stop()
                    success = False
                    self.cancel = True
                    self.turn_again = False
                    # exit the loop:
                    break

            self.robot_controller.stop()
            # Periodic checking to see whether it's near a wall
            self.avoid_wall()

            self.turn_again = True

        if success:
            rospy.loginfo('Motion finished successfully.')
            self.result.total_distance_travelled = distance_travelled
            self.result.closest_object_distance = self.lidar['closest']
            self.result.closest_object_angle = self.lidar['closest angle']
            self.actionserver.set_succeeded(self.result)
            self.robot_controller.stop()

if __name__ == '__main__':
    rospy.init_node('search_server')
    SearchAS()
    rospy.spin()
