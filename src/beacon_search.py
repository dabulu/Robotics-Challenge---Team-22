#! /usr/bin/python

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy

# Import some image processing modules:
import cv2
from cv_bridge import CvBridge

# Import all the necessary ROS message types:
from sensor_msgs.msg import Image

# Import some other modules from within this package
from move_tb3 import MoveTB3
from tb3_odometry import TB3Odometry
from sensor_msgs.msg import LaserScan

import numpy as np
#import colourMasks.py for image thresholds
import colourMasks
import math



class colour_search(object):

    def __init__(self):
        rospy.init_node('turn_and_face')
        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_callback)
        self.lidar_subscriber = rospy.Subscriber('/scan', LaserScan, self.callback_lidar)
        self.cvbridge_interface = CvBridge()

        self.lidar = {'range': 0.0,
                      'closest': 0.0,
                      'closest angle': 0}

        self.robot_controller = MoveTB3()
        self.turn_vel_fast = -0.5
        self.turn_vel_slow = -0.1
        self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)

        self.move_rate = '' # fast, slow or stop
        self.stop_counter = 0
        self.start_yaw = 0.0
        self.start_posy = 0.0
        self.start_posx = 0.0

        self.robot_controller = MoveTB3()
        self.robot_odom = TB3Odometry()
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
        angle_tolerance = 10
        self.lidar['range'] = min(min(raw_data[:angle_tolerance]),
                               min(raw_data[-angle_tolerance:]))

        # Closest object
        self.lidar['closest'] = min(raw_data)

        # Angle of closest object
        self.lidar['closest angle']=raw_data.argmin()

    def camera_callback(self, img_data):
        global waiting_for_image
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        height, width, channels = cv_img.shape

        crop_width = width - 800
        crop_height = 180
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
            #CHANGE BACK TO SELF.COLOUR AFTER DEBUGGING
            mask = colourMasks.getMask(hsv_img, 1)
            #mask = cv2.inRange(hsv_img, lower, upper)
            res = cv2.bitwise_and(crop_img, crop_img, mask = mask)

            m = cv2.moments(mask)
            self.m00 = m['m00']
            self.cy = m['m10'] / (m['m00'] + 1e-5)

            if self.m00 > self.m00_min:
                cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 4)

        cv2.imshow('cropped image', crop_img)
        cv2.waitKey(1)

    def set_robot_turning(self, turning_right):
        if turning_right:
            self.turn_vel_slow = -0.15
            self.turn_vel_fast =  -0.5
        else:
            self.turn_vel_slow = 0.15
            self.turn_vel_fast = 0.5

    def find_target_pillar(self, target):

        # get reference point for start of turn
        self.start_yaw = self.robot_odom.yaw
        # print("Start yaw: {}".format(self.start_yaw))
        # set var to stop turn when complete
        turn_complete = False
        # stop robot before turn for accurate data
        self.robot_controller.stop()

        while not turn_complete:
            # if (self.start_yaw - self.robot_odom.yaw) < 0:
            #     print("Yaw was less than {}".format((self.start_yaw - self.robot_odom.yaw)))
            #     break

            if self.m00 > self.m00_min:
                print("detected")
                # blob detected
                if self.cy >= 560-100 and self.cy <= 560+100:
                    print("centre")
                    if not self.pillar_lined_with_home:
                        self.move_towards_pillar()
                        self.complete = True
                        self.robot_controller.stop()
                        break
                    if not self.check_facing_home(self.robot_odom.posy, self.robot_odom.posx, self.robot_odom.yaw):
                        #     # if self.move_rate == 'slow':
                        self.move_towards_pillar()
                        self.complete = True
                        self.robot_controller.stop()
                        break
                else:
                    if not self.check_facing_home(self.robot_odom.posy, self.robot_odom.posx, self.robot_odom.yaw):
                        # self.move_rate = 'slow'
                        self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)
            else:
                # self.move_rate = 'fast'
                self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)

            if abs(self.robot_odom.yaw - self.start_yaw) >= target:
                self.robot_controller.stop()
                turn_complete = True

            # print("Yaw: {}".format(self.robot_odom.yaw))

            self.robot_controller.publish()
            self.rate.sleep()
        # print("Finished loop")

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

    def main(self):
        while not self.ctrl_c:

            # pass time for data to catch up from odom
            # if not self.finished_initialising:
            for i in range (0,5):
                self.rate.sleep()
            #
            # self.start_posy = -1.974
            # self.start_posx = -2.067
            #     self.finished_initialising = True


            """
             [A , B , C]
             [1 , 2 , 3]
             check which area its in
            """
            while self.check:
                if self.robot_odom.posy > 1.900 and self.robot_odom.posx > 2:
                    print('Position C')
                    self.area = "C"
                elif self.robot_odom.posy > 2.0600 and self.robot_odom.posy < 2.080:
                    # print('Position B')
                    self.area = "B"
                else:
                    print('Position A')
                    self.area = "A"

                self.check = False

            self.start_posy = self.robot_odom.posy
            self.start_posx = self.robot_odom.posx

            # self.turn(90)
            # self.get_colour = True
            # # print("turn finished")
            # self.turn(90, False)

            if self.area == "B":
                while self.robot_odom.posy >= 1.09:
                    self.robot_controller.set_move_cmd(0.15, 0.3)
                    self.robot_controller.publish()
                    self.rate.sleep()
                while self.robot_odom.posx <= -0.1:
                    self.robot_controller.set_move_cmd(0.2, 0.07)
                    self.robot_controller.publish()
                    self.rate.sleep()
                self.robot_controller.stop()
                self.finding_pillar = True
                self.set_robot_turning(True)
                self.find_target_pillar(45)
                if self.complete:
                    break
                self.set_robot_turning(False)
                self.pillar_lined_with_home = True
                self.find_target_pillar(160)
                self.pillar_lined_with_home = False
                if self.complete:
                    break
                self.finding_pillar = False
                self.turn(45)
                while self.robot_odom.posx >= -1.2:
                    self.robot_controller.set_move_cmd(0.3, -0.02)
                    self.robot_controller.publish()
                    self.rate.sleep()
                while self.robot_odom.posy >= 0.2:
                    self.robot_controller.set_move_cmd(0.15, 0.25)
                    self.robot_controller.publish()
                    self.rate.sleep()
                while self.robot_odom.posx <= -0.1:
                    self.robot_controller.set_move_cmd(0.3, 0.01)
                    self.robot_controller.publish()
                    self.rate.sleep()
                self.robot_controller.stop()
                self.turn(15, False)
                self.finding_pillar = True
                self.set_robot_turning(False)
                self.find_target_pillar(120)
                if self.complete:
                    break
                self.find_target_pillar(140)
            elif self.area == "C":
                while self.robot_odom.posy >= 1.3:
                    self.robot_controller.set_move_cmd(0.3, -0.05)
                    self.robot_controller.publish()
                    self.rate.sleep()
                self.robot_controller.stop()
                self.finding_pillar = True
                self.set_robot_turning(True)
                self.find_target_pillar(90)
                if self.complete:
                    break
                self.find_target_pillar(25)
                if self.complete:
                    break
                self.turn(115)
                while self.robot_odom.posy >= -1:
                    self.robot_controller.set_move_cmd(0.3, 0)
                    self.robot_controller.publish()
                self.turn(80, False)
                while self.robot_odom.posx >= 0:
                    self.robot_controller.set_move_cmd(0.3, 0)
                    self.robot_controller.publish()
                self.robot_controller.stop()
                self.find_target_pillar(90)



            break

            # self.finding_pillar = True
            # self.set_robot_turning(True)
            # self.find_target_pillar(90)



            #beacon

            #check beacon is not home walls

            if self.complete:
                print("SEARCH COMPLETE: The robot is now facing the target pillar.")
                break

if __name__ == '__main__':
    search_ob = colour_search()
    try:
        search_ob.main()
    except rospy.ROSInterruptException:
        pass
