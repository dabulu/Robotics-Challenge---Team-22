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
        rospy.init_node('turn_and_face')
        self.base_image_path = '/home/student/myrosdata/week6_images'
        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
            Image, self.camera_callback)
        self.cvbridge_interface = CvBridge()

        self.robot_controller = MoveTB3()
        self.turn_vel_fast = -0.5
        self.turn_vel_slow = -0.1
        self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)

        self.move_rate = '' # fast, slow or stop
        self.stop_counter = 0
        self.start_yaw = 0.0
        self.start_posx = 0.0

        self.robot_controller = MoveTB3()
        self.robot_odom = TB3Odometry()
        #var to check task is complete
        self.complete = False
        #var to check if move forward is needed
        self.move_forward = True
        #var to check if statr yaw has been initiated
        self.face_turn = False

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

        lower = (115, 224, 100)
        upper = (130, 255, 255)
        #[turquoise, red, green, yellow, blue, magenta]
        #[     0  ,   1 ,   2  ,   3   ,  4  ,    5   ]
        mask = colourMasks.getMask(hsv_img, 4)
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

        self.start_yaw = self.robot_odom.yaw
        print("Start yaw: {}".format(self.start_yaw))

        complete = False
        self.robot_controller.stop()
        while not complete:
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
                complete = True

            print("Yaw: {}".format(self.robot_odom.yaw))

            self.robot_controller.publish()
            self.rate.sleep()
        print("Finished loop")

    def main(self):
        while not self.ctrl_c:

            self.start_posx = self.robot_odom.posx
            #move to X
            while self.move_forward:
                self.robot_controller.set_move_cmd(0.2, 0.0)
                self.robot_controller.publish()
                self.rate.sleep()
                print(self.start_posx - self.robot_odom.posx)
                if abs(self.start_posx - self.robot_odom.posx) > 0.003:
                    self.move_forward = False

            self.robot_controller.stop()

            #set current yaw
            # if not self.face_turn:
            #     self.start_yaw = self.robot_odom.yaw
            #     print("Start yaw: {}".format(self.start_yaw))
            #     self.face_turn = True

            #set robot to turn right
            print("Turning RIGHT!")
            self.set_robot_turning(True)
            #try to find pillar turning right
            self.find_target_pillar(90)
            #otherwise turn left if not finished
            while not self.complete:
                print("Turning LEFT!")
                self.set_robot_turning(False)
                self.find_target_pillar(180)
                print("Turning RIGHT!")
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
