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

        self.robot_controller = MoveTB3()
        self.robot_odom = TB3Odometry()

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
        #[cyan, red, green, yellow, blue, magenta]
        #[ 0  ,  1 ,   2  ,   3   ,  4  ,    5   ]
        mask = colourMasks.getMask(hsv_img, 0)
        #mask = cv2.inRange(hsv_img, lower, upper)
        res = cv2.bitwise_and(crop_img, crop_img, mask = mask)

        m = cv2.moments(mask)
        self.m00 = m['m00']
        self.cy = m['m10'] / (m['m00'] + 1e-5)

        if self.m00 > self.m00_min:
            cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)

        cv2.imshow('cropped image', crop_img)
        cv2.waitKey(1)

    def main(self):
        while not self.ctrl_c:
            if self.stop_counter > 0:
                self.stop_counter -= 1

            if self.m00 > self.m00_min:
                # blob detected
                if self.cy >= 560-100 and self.cy <= 560+100:
                    if self.move_rate == 'slow':
                        self.move_rate = 'stop'
                        self.stop_counter = 30
                else:
                    self.move_rate = 'slow'
            else:
                self.move_rate = 'fast'

            if self.move_rate == 'fast':
                print("MOVING FAST: I can't see anything at the moment, scanning the area...")
                self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)
            elif self.move_rate == 'slow':
                print("MOVING SLOW: A blob of colour of size {:.0f} pixels is in view at y-position: {:.0f} pixels.".format(self.m00, self.cy))
                self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)
            elif self.move_rate == 'stop' and self.stop_counter > 0:
                print("STOPPED: The blob of colour is now dead-ahead at y-position {:.0f} pixels... Counting down: {}".format(self.cy, self.stop_counter))
                self.robot_controller.set_move_cmd(0.0, 0.0)
            else:
                print("MOVING SLOW: A blob of colour of size {:.0f} pixels is in view at y-position: {:.0f} pixels.".format(self.m00, self.cy))
                self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)

            self.robot_controller.publish()
            self.rate.sleep()

if __name__ == '__main__':
    search_ob = colour_search()
    try:
        search_ob.main()
    except rospy.ROSInterruptException:
        pass
