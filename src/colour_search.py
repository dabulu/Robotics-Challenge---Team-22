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

#import colourMasks.py for image thresholds
import colourMasks



class colour_search(object):

    def __init__(self):
        rospy.init_node('pillar_detection')
        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_callback)
        self.cvbridge_interface = CvBridge()

        self.robot_controller = MoveTB3()
        self.turn_vel_fast = -0.5
        self.turn_vel_slow = -0.1
        self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)

        self.move_rate = '' # fast, slow or stop
        self.stop_counter = 0
        self.start_yaw = 0.0
        self.start_posy = 0.0

        self.robot_controller = MoveTB3()
        self.robot_odom = TB3Odometry()
        #var to check task is complete
        self.complete = False
        #var to check if move forward is needed
        self.move_forward = True
        #var to turn to check the color
        self.turn = True
        #var to turn back facing the front
        self.turn_back = True
        #var to check if statr yaw has been initiated
        self.face_turn = False

        self.finding_pillar = False

        self.get_colour = False

        self.colour = -1

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

        self.rate = rospy.Rate(5)


        self.m00 = 0
        self.m00_min = 10000

    def shutdown_ops(self):
        self.robot_controller.stop()
        cv2.destroyAllWindows()
        self.ctrl_c = True


    def camera_callback(self, img_data):
        global waiting_for_image
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        height, width, channels = cv_img.shape

        crop_width = width - 800
        crop_height = 400
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
            #[turquoise, red, green, yellow, blue, magenta]
            #[     0  ,   1 ,   2  ,   3   ,  4  ,    5   ]
            mask = colourMasks.getMask(hsv_img, self.colour)
            #mask = cv2.inRange(hsv_img, lower, upper)
            res = cv2.bitwise_and(crop_img, crop_img, mask = mask)

            m = cv2.moments(mask)
            self.m00 = m['m00']
            self.cy = m['m10'] / (m['m00'] + 1e-5)

            if self.m00 > self.m00_min:
                cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)

        cv2.imshow('cropped image', crop_img)
        cv2.waitKey(1)

    def set_robot_turning(self, turning_right):
        if turning_right:
            self.turn_vel_slow = -0.1
            self.turn_vel_fast =  -0.5
        else:
            self.turn_vel_slow = 0.1
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
                # blob detected
                if self.cy >= 560-100 and self.cy <= 560+100:
                    # if self.move_rate == 'slow':
                    # print("STOP")
                    self.complete = True
                    self.robot_controller.stop()
                    break
                else:
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

    def main(self):
        while not self.ctrl_c:

            # pass time for data to catch up from odom
            for i in range (0,5):
                self.rate.sleep()

            self.start_posy = self.robot_odom.posy

            #turn to check color
            while self.turn:
                self.robot_controller.set_move_cmd(0.0, 0.3)
                self.robot_controller.publish()
                self.rate.sleep()
                if abs(self.robot_odom.posy) > 1.0493:
                    self.turn = False
                    self.get_colour = True

            #turn back to initial position
            while self.turn_back:
                self.robot_controller.set_move_cmd(0.0, -0.3)
                self.robot_controller.publish()
                self.rate.sleep()
                if abs(self.robot_odom.posy) < 1.0425:
                    self.turn_back = False

            #move to X
            while self.move_forward:
                self.robot_controller.set_move_cmd(0.2, 0.0)
                self.robot_controller.publish()
                self.rate.sleep()
                # print(self.start_posy - self.robot_odom.posy)
                if abs(self.start_posy - self.robot_odom.posy) > 1:
                    self.move_forward = False
                    self.finding_pillar = True



            self.robot_controller.stop()

            #set robot to turn right
            # print("Turning RIGHT!")
            self.set_robot_turning(True)
            #try to find pillar turning right
            self.find_target_pillar(90)
            #otherwise turn left if not finished
            while not self.complete:
                # print("Turning LEFT!")
                self.set_robot_turning(False)
                self.find_target_pillar(200)
                # print("Turning RIGHT!")
                self.set_robot_turning(True)
                self.find_target_pillar(180)

            if self.complete:
                print("SEARCH COMPLETE: The robot is now facing the target pillar.")
                break

if __name__ == '__main__':
    search_ob = colour_search()
    try:
        search_ob.main()
    except rospy.ROSInterruptException:
        pass
