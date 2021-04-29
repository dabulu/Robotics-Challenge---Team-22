import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class MoveTurn(object):
    def __init__(self):
        self.sub = rospy.Subscriber('/odom', Odometry, callback_function)
        self.roll = self.pitch = self.yaw = 0.0

    def callback_function(self, odom_data):
        orientation = odom_data.pose.pose.orientation
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion([orientation.x, orientation.y,
        orientation.z], 'sxyz')
        print self.yaw
        return self.yaw
