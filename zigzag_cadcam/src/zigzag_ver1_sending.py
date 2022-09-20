#!/usr/bin/env python
# license removed for brevity
import rospy
#include <tf2/LinearMath/Quaternion.h>
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal,MoveBaseActionResult
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import OccupancyGrid
#####################################################################################
class ZigZag_Ver1():
    def __init__(self):

        rospy.init_node("zigzag_sender_ver1")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)
	self.rate = rospy.get_param('~rate',5.0)

        self.pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1000)
	rospy.Subscriber("move_base/result", MoveBaseActionResult, self.goal_statu_cb)
	rospy.Subscriber("bitmap", OccupancyGrid, self.map_cb1)
	rospy.Subscriber("map", OccupancyGrid, self.map_cb1)
	
	#self.goal = PoseStamped()
	self.starting_position =  [-1.5,1.5,0,0,0,0,1]
	self.flag = True
	self.zig_num_flag = 0
	self.a =0
	self.goal_pose = self.starting_position
	self.sequence = 1
	self.connection_test=0
	rospy.sleep(1)#Waiting For the Roscore To Connect
	self.zig()# Go To Starting Point

	self.incre = 3
	self.below_b = -1.0
	self.below_u = -0.1
	self.up_b = 0.2
	self.up_u = 1.2
	self.moving_flag=0

#####################################################################################
    def map_cb1(self,data):
	a=0
    def spin(self):
	r = rospy.Rate(self.rate)
	
	while not rospy.is_shutdown():
	    self.update()
	    r.sleep()

    def euler_to_quaternion(self,roll,pitch,yaw):
	quaternion = quaternion_from_euler(roll, pitch, yaw)
	x = quaternion[0]
	y = quaternion[1]
	z = quaternion[2]
	w = quaternion[3]
	qua = [x,y,z,w]
	return qua


    def quaternion_to_euler(self,x,y,z,w):
	quaternion = (x,y,z,w)
	euler = euler_from_quaternion(quaternion)
	roll = euler[0]
	pitch = euler[1]
	yaw = euler[2]
	return roll,pitch,yaw
	





    def move_up(self):
	increment = [self.incre,0,0,0,0,0,0]
	print(self.moving_flag)
	
	self.goal_pose = [self.goal_pose[i]+increment[i]for i in range(len(self.goal_pose))]
	"""
	if(self.moving_flag ==0):
		self.goal_pose[0] = self.below_b
	elif(self.moving_flag ==1):
		self.goal_pose[0] = self.below_u
	elif(self.moving_flag ==2):
		self.goal_pose[0] = self.up_b
	elif(self.moving_flag ==3):
		self.goal_pose[0] = self.up_u
	elif(self.moving_flag ==4):
		self.goal_pose[0] = 1.5
	self.moving_flag += 1
	"""



    def move_down(self):
	increment = [-self.incre,0,0,0,0,0,0]
	self.goal_pose = [self.goal_pose[i]+increment[i]for i in range(len(self.goal_pose))]
	"""
	if(self.moving_flag ==1):
		self.goal_pose[0] = -1.5
	elif(self.moving_flag ==2):
		self.goal_pose[0] = self.below_b
	elif(self.moving_flag ==3):
		self.goal_pose[0] = self.below_u
	elif(self.moving_flag ==4):
		self.goal_pose[0] = self.up_b
	elif(self.moving_flag ==5):
		self.goal_pose[0] = self.up_u
	print(self.moving_flag)
	self.moving_flag -= 1
	"""
    def move_right(self):
	increment = [0,-0.75,0,0,0,0,0]
	self.goal_pose = [self.goal_pose[i]+increment[i]for i in range(len(self.goal_pose))]

    def turn_right(self):

	turning = [ 0, 0, -0.7071068, 0.7071068 ]

	for i in range(4):
		self.goal_pose[i+3] = turning[i]

    def turn_up(self):
	turning = [ 0, 0, 0, 1 ]
	for i in range(4):
		self.goal_pose[i+3] = turning[i]
    def turn_down(self):
	turning = [ 0, 0, 1, 0 ]
	for i in range(4):
		self.goal_pose[i+3] = turning[i]


    def update_next_plan(self):
	
	num_increment = 3/self.incre
	print("zig_num_flag is")
	print(self.zig_num_flag)
	if(self.zig_num_flag<num_increment):
		self.move_up()
	elif(self.zig_num_flag == num_increment ):
		self.turn_right()
	elif(self.zig_num_flag == num_increment+1):
		self.move_right()
	elif(self.zig_num_flag == num_increment+2 ):
		self.turn_down()
	elif(self.zig_num_flag <2*num_increment+3):
		self.move_down()
	elif(self.zig_num_flag ==2*num_increment+3):
		self.turn_right()
	elif(self.zig_num_flag==2*num_increment+4 ):
		self.move_right()
	elif(self.zig_num_flag==2*num_increment+5):
		self.turn_up()

	self.zig_num_flag +=1
	
	if(self.zig_num_flag==2*num_increment+6):
		self.zig_num_flag=0

		
	
		
		
	
	
		

	

    def zig(self):
	goal = PoseStamped()
	goal.header.seq = self.sequence
        goal.header.frame_id = 'map'
	goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = self.goal_pose[0]
        goal.pose.position.y = self.goal_pose[1]
        goal.pose.position.z = self.goal_pose[2]
        goal.pose.orientation.x = self.goal_pose[3]
        goal.pose.orientation.y = self.goal_pose[4]
        goal.pose.orientation.z = self.goal_pose[5]
        goal.pose.orientation.w = self.goal_pose[6]
	self.sequence = self.sequence+1
        self.pub.publish(goal)
        print(goal)	

    def goal_statu_cb(self, data):
	print("got it")
	self.a = data.status.status
	if(self.a == 3):
		self.flag = False


    def update(self):
	if(self.flag == False):
		
		self.update_next_plan()
		self.zig()
		self.flag = True



	
	    
#self.flag = True

	    #self.update_next_plan()
	    
	




if __name__ == '__main__':
    """ main """
    zig = ZigZag_Ver1()
    
    zig.spin()


