#!/usr/bin/env python

import rospy
# import roslib
# roslib.load_manifest('differential_drive')
from math import sin, cos, pi

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from std_msgs.msg import Int16, Int64, Float32, Float32MultiArray


#############################################################################
class DiffTf:
    #############################################################################

    #############################################################################
    def __init__(self):
        #############################################################################

        rospy.init_node("diff_tf")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)

        # internal data
        self.enc_left = None  # wheel encoder readings
        self.enc_right = None
        self.left = 0  # actual values coming back from robot
        self.right = 0
        self.lmult = 0
        self.rmult = 0
        self.prev_lencoder = 0
        self.prev_rencoder = 0
        self.d = 0
        self.x = 0  # position in xy plane
        self.y = 0
        self.th = 0
        self.dx = 0  # speeds in x/rotation
        self.dr = 0
        self.then = rospy.Time.now()
        self.longth = 0

       #### parameters #######
        self.rate = rospy.get_param('~rate',10.0)  # the rate at which to publish the transform
        self.ticks_meter = float(rospy.get_param('ticks_meter', 2415))  # The number of wheel encoder ticks per meter of travel
        self.base_width = float(rospy.get_param('~base_width', 0.3)) # The wheel base width in meters
        
        self.base_frame_id = 'base_footprint'
        self.odom_frame_id = 'odom'
        #self.base_frame_id = rospy.get_param('~base_frame_id','base_footprint') # the name of the base frame of the robot
        #self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom') # the name of the odometry reference frame
        
        self.encoder_min = rospy.get_param('encoder_min', -2147483648)
        self.encoder_max = rospy.get_param('encoder_max', 2147483648)
        self.encoder_low_wrap = rospy.get_param('wheel_low_wrap', (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min )
        self.encoder_high_wrap = rospy.get_param('wheel_high_wrap', (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min )
 
        self.t_delta = rospy.Duration(1.0/self.rate)
        self.t_next = rospy.Time.now() + self.t_delta
        

        # subscriptions
        rospy.Subscriber("dx_dtheta", Float32MultiArray, self.lwheelCallback)
        #rospy.Subscriber("dtheta", Float32, self.rwheelCallback)
        self.odomPub = rospy.Publisher("wheel_odom", Odometry, queue_size=10)
        self.odomBroadcaster = TransformBroadcaster()

    #############################################################################
    def spin(self):
        #############################################################################
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()

    #############################################################################
    def update(self):
        #############################################################################
        now = rospy.Time.now()
        elapsed = now - self.then
        self.then = now
        elapsed = elapsed.to_sec()

        # calculate velocities
        self.dx = self.d / elapsed
        self.dr = self.th / elapsed

        if (self.th != 0):
            self.longth = self.longth + self.th
        if (self.d != 0):
            # calculate distance traveled in x and y
            x = cos(self.th) * self.d
            y = (-sin(self.th) * self.d)
            # calculate the final position of the robot
            self.x = self.x + (cos(self.longth) * x - sin(self.longth) * y)
            self.y = self.y + (sin(self.longth) * x + cos(self.longth) * y)
        self.th = 0
        # publish the odom information
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = sin(self.longth / 2)
        quaternion.w = cos(self.longth / 2)
	'''
        self.odomBroadcaster.sendTransform(
            (self.x, self.y, 0),
            (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
            rospy.Time.now(),
            self.base_frame_id,
            self.odom_frame_id
        )
	'''
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0
        odom.pose.pose.orientation = quaternion
        odom.child_frame_id = self.base_frame_id
        odom.twist.twist.linear.x = self.dx
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = self.dr
        self.odomPub.publish(odom)

    #############################################################################
    def lwheelCallback(self, msg):
        #############################################################################

        self.d = msg.data[1]
        self.th =msg.data[0]

    #############################################################################
    def rwheelCallback(self, msg):
        #############################################################################
        enc = msg.data

        self.th = enc


#############################################################################
#############################################################################
if __name__ == '__main__':
    """ main """
    diffTf = DiffTf()
    diffTf.spin()



