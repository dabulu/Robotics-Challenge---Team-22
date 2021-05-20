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
# Import all the necessary ROS message types:
from sensor_msgs.msg import Image
import cv2

# Import some other modules from within this package
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge

import colourMasks
import math

# Import some other useful Python Modules
from math import radians, pi
import datetime as dt
import os
import numpy as np
import copy

class beacon_search(object):
    feedback = SearchFeedback()
    result = SearchResult()

    def __init__(self):
        # arena_server.py
        self.actionserver = actionlib.SimpleActionServer("/arena_action_server", SearchAction, self.action_server_launcher, auto_start=False)
        self.actionserver.start()

        # Lidar subscriber
        self.lidar_subscriber = rospy.Subscriber('/scan', LaserScan, self.callback_lidar)
        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_callback)
        self.lidar = {'range': 0.0,
        'wider range': 0.0,
        'closest': 0.0,
        'closest angle': 0}

        # Robot movement and odometry
        self.robot_controller = MoveTB3()
        self.robot_odom = TB3Odometry()

        self.current_yaw = 0.0

        # beacon_search

        self.cvbridge_interface = CvBridge()
        self.turn_vel_fast = -0.5
        self.turn_vel_slow = -0.1
        self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)

        self.move_rate = '' # fast, slow or stop
        self.stop_counter = 0
        self.start_yaw = 0.0
        self.start_posy = 0.0
        self.start_posx = 0.0

        #var to check task is complete
        self.complete = False
        #var to check if move forward is needed
        self.move_forward = True
        #var to turn to check the color
        # self.turn = True
        #var to turn back facing the front
        self.turn_back = True
        #var to check if statr yaw has been initiated
        self.face_turn = False

        self.pillar_lined_with_home = False

        self.finding_pillar = False

        self.get_colour = False

        self.colour = -1

        self.check = True

        self.counter = 0

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

        self.rate = rospy.Rate(5)

        self.area = 0

        self.m00 = 0
        self.m00_min = 10000

        self.finished_initialising = False

    def shutdown_ops(self):
        self.robot_controller.stop()
        cv2.destroyAllWindows()
        self.ctrl_c = True

    def callback_lidar(self, lidar_data):
        """Returns arrays of lidar data"""

        raw_data = np.array(lidar_data.ranges)

        # Directly in front of object
        angle_tolerance = 12
        self.lidar['range'] = min(min(raw_data[:angle_tolerance]), min(raw_data[-angle_tolerance:]))

        # Closest object
        self.lidar['closest'] = min(raw_data)

        # Angle of closest object
        self.lidar['closest angle']=raw_data.argmin()

        # Wider arc size
        wider_tolerance = 30
        # The closest obstacle in a wider arc directly in front of the robot
        self.lidar['wider range'] = min(min(raw_data[:wider_tolerance]),
        min(raw_data[-wider_tolerance:]))

        # beacon_search
    def camera_callback(self, img_data):
        global waiting_for_image
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        height, width, channels = cv_img.shape

        crop_width = width - 800
        crop_height = 160
        crop_x = int((width/2) - (crop_width/2))
        crop_y = int((height/2) - (crop_height/2))

        crop_img = cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        if self.get_colour:
            self.colour = colourMasks.determineColour(hsv_img)
            print("SEARCH INITIATED: The target colour is {}.".format(colourMasks.getColour(self.colour)))
            # print(self.colour)
            self.get_colour = False

        if self.finding_pillar:
            #[turquoise, red, green, yellow, magenta, blue]
            #[     0  ,   1 ,   2  ,   3   ,  4  ,    5   ]
            mask = colourMasks.getMask(hsv_img, self.colour)
            #mask = cv2.inRange(hsv_img, lower, upper)
            res = cv2.bitwise_and(crop_img, crop_img, mask = mask)

            m = cv2.moments(mask)
            self.m00 = m['m00']
            self.cy = m['m10'] / (m['m00'] + 1e-5)

            if self.m00 > self.m00_min:
                cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 4)

        # cv2.imshow('cropped image', crop_img)
        cv2.waitKey(1)

    # arena_server
    def move_back(self):
        # Back up from a wall if it's too close
        self.robot_controller.set_move_cmd(linear=-0.2)
        while self.lidar['range'] <= 0.2:
            self.robot_controller.publish()
            self.robot_controller.stop

    # arena_server
    def avoid_wall(self):
        self.robot_controller.stop()
        # Turn away from a close wall until it's not longer close
        if self.lidar['closest'] <= 0.1 and self.lidar['closest angle'] < 90:
            while self.lidar['closest'] <= 0.1:
                self.robot_controller.set_move_cmd(linear=-0.2, angular=0.4)
                self.robot_controller.publish()
        elif self.lidar['closest'] <= 0.1 and self.lidar['closest angle'] < 180:
            while self.lidar['closest'] <= 0.1:
                self.robot_controller.set_move_cmd(linear=0.2, angular=-0.4)
                self.robot_controller.publish()
        elif self.lidar['closest'] <= 0.1 and self.lidar['closest angle'] < 270:
            while self.lidar['closest'] <= 0.1:
                self.robot_controller.set_move_cmd(linear=0.2, angular=0.4)
                self.robot_controller.publish()
        elif self.lidar['closest'] <= 0.1 and self.lidar['closest angle'] < 360:
            while self.lidar['closest'] <= 0.1:
                self.robot_controller.set_move_cmd(linear=-0.2, angular=-0.4)
                self.robot_controller.publish()
                self.robot_controller.stop()

    # beacon_search
    def set_robot_turning(self, turning_right):
        if turning_right:
            self.turn_vel_slow = -0.15
            self.turn_vel_fast =  -0.5
        else:
            self.turn_vel_slow = 0.15
            self.turn_vel_fast = 0.5

    def find_target_pillar(self):
        if self.m00 > self.m00_min:
            # blob detected
            if self.cy >= 560-100 and self.cy <= 560+100:
                #blob is centred, move forward
                self.move_towards_pillar()
                self.complete = True
                self.robot_controller.stop()
                return True
            elif self.cy < 460:
                #turn left, blob is to the left
                self.robot_controller.set_move_cmd(0.0, 0.2)
                self.robot_controller.publish()
                return True
            elif self.cy > 660:
                #turn right, blob is to the right
                self.robot_controller.set_move_cmd(0.0, -0.2)
                self.robot_controller.publish()
                return True
        else:
            return False

    def get_bearing(self, a1, a2, b1 , b2):
        if (a1 == b1 and a2 == b2):
            return False
            theta = math.atan2(b1 - a1, b2 - a2)
        if (theta < 0.0):
            return math.degrees(theta + 2*math.pi)
            return math.degrees(theta)

    def get_percentage_difference(self, a, b):
        if a > b:
            if a == 0:
                return 0
                return ((a-b)/a)*100
            elif b > a:
                return ((b-a)/b)*100
            else:
                return 0

    def check_facing_home(self, posy, posx, yaw):
        original_posy, original_posx = self. start_posy, self.start_posx
        bearing = self.get_bearing(posy, posx, original_posy, original_posx)
        yaw = self.get_yaw_as_bearing(yaw)
        # print("Bearing: {}".format(points_bearing))
        pd = self.get_percentage_difference(bearing, yaw)
        # print("Percentage Difference: {}".format(pd))
        if pd <= 12.5:
            return True
        else:
            return False

    def get_yaw_as_bearing(self, yaw):
        if yaw < 0:
            return yaw + 360
        else:
            return yaw

    def move_towards_pillar(self):
        print("BEACON DETECTED: Beaconing initiated.")
        while not self.complete:
            self.robot_controller.set_move_cmd(0.3, 0)
            # print(self.lidar['range'])
            if self.lidar['range'] <= 0.25:
                self.robot_controller.stop()
                print("BEACONING COMPLETE: The robot has now stopped.")
                self.complete = True
                break
                self.robot_controller.publish()
                self.rate.sleep()

                                                                    # function to turn the robot from the hard coding time
    def turn(self, target, right = True):
        ref_yaw = self.get_yaw_as_bearing(self.robot_odom.yaw)
        turning = True
        if right:
            turn_speed = 0.3
        else:
            turn_speed = -0.3
            while turning:
                self.robot_controller.set_move_cmd(0, turn_speed)
                self.robot_controller.publish()
                self.rate.sleep()
                if abs(ref_yaw - self.get_yaw_as_bearing(self.robot_odom.yaw)) >= target:
                    self.robot_controller.stop()
                    turning = False

                                                                                    # function to move the robot forwards from the hard coding time
    def forwards(self, target, pos):
        straight = True
        while straight:
            if pos == "y":
                self.robot_controller.set_move_cmd(0.2, 0.0)
                self.robot_controller.publish()
                self.rate.sleep()
                # print(abs(self.robot_odom.posy))
                if abs(self.robot_odom.posy) <= target:
                    self.robot_controller.stop()
                    straight = False
                elif pos == "x":
                    self.robot_controller.set_move_cmd(0.2, 0.0)
                    self.robot_controller.publish()
                    self.rate.sleep()
                    # print(abs(self.robot_odom.posx))
                    if abs(self.robot_odom.posx) <= target:
                        self.robot_controller.stop()
                        straight = False

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
        while rospy.get_rostime().secs - StartTime.secs < 150 and self.cancel == False:
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
                # rospy.loginfo('Current distance to object: {:.2f} m'.format(self.lidar['range']))
                # rospy.loginfo('{} seconds have passed'.format(rospy.get_rostime().secs - StartTime.secs))
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

            # Set initial robot direction based on where the closest obstacle is
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
    rospy.init_node('task5_obstacle_server')
    beacon_search()
    rospy.spin()
