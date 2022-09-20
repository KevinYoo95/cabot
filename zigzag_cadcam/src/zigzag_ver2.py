#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal,MoveBaseActionResult
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import  Odometry
import zig3



#####################################################################################
class ZigZag_Ver1():
    def __init__(self):

        rospy.init_node("zigzag_sender_ver3")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)
	self.rate = rospy.get_param('~rate',5.0)

        self.pub1 = rospy.Publisher('line_dot', OccupancyGrid, queue_size=1000)
        self.pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1000)
	rospy.Subscriber("move_base/result", MoveBaseActionResult, self.goal_statu_cb)
	
	self.radius = 0.15
	
	rospy.Subscriber('cspace', OccupancyGrid, self.map_cb1)
	rospy.Subscriber("odom", Odometry, self.Odom_cb)
	
	rospy.sleep(1)
	
	######################### REAL_WORLD_PARAMETER #############################

	self.dr_pixel = 0
	self.start_xm =0
	self.start_ym =0
	self.base_x =0
	self.base_y =0

	########################### MAP_CALLBACK #######################################

	self.map_array =[]
	self.width =0
	self.height =0
	self.resolution =0
	self.x_in_meter =0
	self.y_in_meter =0
	self.resolution	=0 #meter/pixel
	self.start_point =0
	self.phase = 0
	########################################################################################
	self.flag = True
	self.sequence = 1
	self.goal_array = 0
	
	self.line_points =[]
	self.min_distance =0
	self.r_c_point =0
	self.line_only = []
	self.line_num = 0
	self.seq = 1
	self.goal_pose = [0,0,0,0,0,0,1]

################################## PLANN ########################################
	
	self.line_x = []
	self.trajectorymap = OccupancyGrid()
	self.min_dis_num = 0
	self.point =0
	self.in_line_flag=0
	self.up_down_point =0
	self.next_target = []
	self.on_off_line = 0
	#woww = [1,2,3,4,5,6,7,8,9]
	#del woww[2:5]
	#print(woww)

###############################3 INIT ##########################################
	while self.map_array ==[]:
		rospy.sleep(0.01)

	zig3.global_variable_declaration(self.height,self.width,self.resolution,self.map_array)
	self.draw_()

	self.making_line_map()
	self.mapp_line_point()



#####################################################################################

    def map_cb1(self, data):
	self.map_array = data.data
	self.width = data.info.width
	self.height = data.info.height
	self.resolution = data.info.resolution
	self.x_in_meter = data.info.origin.position.x 
	self.y_in_meter = data.info.origin.position.y 
	self.dr_pixel = int(self.radius/self.resolution)


    def Odom_cb(self, data):
	self.base_x = data.pose.pose.position.x
	self.base_y = data.pose.pose.position.y


######################################################################################################
    def draw_(self):
	self.r_c_point= zig3.finding_right_down_point(self.map_array)
	for i in range(50):	
		self.draw_lines(self.r_c_point + self.height * int(self.radius/self.resolution)*i)
		self.line_points.append(zig3.draw_line(self.r_c_point + self.height *  int(self.radius/self.resolution)*i))


	self.bit_line = self.line_only


    def draw_lines(self,starting_array): 
	map_state = 0
	x,y = zig3.array_to_pixel(starting_array)
	py = y
	line =[]
	line_only1 = []




	for i in range(self.height):
		if((self.map_array[y*self.height+i]==0) and (map_state == 0)):
			map_state = 1
			piy,pix = divmod(y*self.height+i,self.height)
			self.line_only.append([pix,piy])
			

		elif((self.map_array[y*self.height+i]!=0 ) and ( map_state == 1)):
			piy,pix = divmod(y*self.height+i,self.height)
			self.line_only.append([pix,piy])
			self.line_only.append([self.line_num])
			line_only1.append([self.line_only])
			line_only = []
			self.line_num  = self.line_num  + 1
			line = []
			map_state = 0


#######################################################################################################

    def planning(self):


	print(self.next_target,self.on_off_line)
	if(self.on_off_line == 0): ##off line ->entering new line
		if(self.up_down_point == 0):
			x,y = zig3.pixel_to_meter(self.next_target[0][0],self.next_target[0][1])
			self.goal_pose[0] = x
			self.goal_pose[1] = y
			turning = [ 0, 0, 0, 1 ]
			for i in range(4):
				self.goal_pose[i+3] = turning[i]

		elif(self.up_down_point == 1):
			x,y = zig3.pixel_to_meter(self.next_target[1][0],self.next_target[1][1])
			self.goal_pose[0] = x
			self.goal_pose[1] = y
			
			turning = [ 0, 0, 1, 0 ]
			for i in range(4):
				self.goal_pose[i+3] = turning[i]
			
	
	else:  ##on line -> to go is already selected
		if(self.up_down_point == 0):
			x,y = zig3.pixel_to_meter(self.next_target[1][0],self.next_target[1][1])
			self.goal_pose[0] = x
			self.goal_pose[1] = y
		elif(self.up_down_point == 1):
			x,y = zig3.pixel_to_meter(self.next_target[0][0],self.next_target[0][1])
			self.goal_pose[0] = x
			self.goal_pose[1] = y


    def cost_function(self):
	#print("self.on_off_line",self.on_off_line)
	if(self.on_off_line == 0):
		#print(self.bit_line)
		min_dis_num = 9999999999
		min_dis = 9999999999
		pix,piy = zig3.meter_to_pixel(self.base_x, self.base_y)
		#print(pix,piy)
		delta_piy = piy - self.r_c_point/self.height
		ithy = delta_piy/int(self.radius/self.resolution)
		print(self.bit_line)
		#print(self.line_points[40])
		#print(ithy)
		#print(len(self.line_points))
		#print(len(self.line_only))
		#print(self.line_only)
		for i in range(len(self.line_points(ithy)):
			if((self.line_points[ithy][i][0][0]<=pix)and(self.line_points[ithy][i][1][0]>=pix)):
				self.line_num = self.line_points[ithy][i][2][0]

		############detemining where the robot is..############################################

		for i in range(len(self.bit_line)/3):
			#print(self.bit_line[3*i])
			distance = (pix - self.bit_line[3*i][0])*(pix - self.bit_line[3*i][0]) + (piy - self.bit_line[3*i][1])*(piy - self.bit_line[3*i][1])
			#print(distance)
			if(distance < min_dis):
				min_dis = distance
				aa = []
				aa.append(self.bit_line[3*i])
				aa.append(self.bit_line[3*i+1])
				aa.append(self.bit_line[3*i+2])
				self.min_line_num = self.bit_line[3*i+2][0]
				self.next_target = aa
				self.up_down_point = 0 #down
	
			distance = (pix - self.bit_line[3*i+1][0])*(pix - self.bit_line[3*i+1][0]) + (piy - self.bit_line[3*i+1][1])*(piy - self.bit_line[3*i+1][1])
	
			if(distance < min_dis):
				min_dis = distance
				aa = [] 
				aa.append(self.bit_line[3*i])
				aa.append(self.bit_line[3*i+1])
				aa.append(self.bit_line[3*i+2])
				self.min_line_num = self.bit_line[3*i+2][0]
				self.next_target = aa
				self.up_down_point = 1 #up
	
	
######################################################################################################

    def zig(self):
	goal = PoseStamped()
	goal.header.seq = self.seq
        goal.header.frame_id = 'map'
	goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = self.goal_pose[0]
        goal.pose.position.y = self.goal_pose[1]
        goal.pose.position.z = self.goal_pose[2]
        goal.pose.orientation.x = self.goal_pose[3]
        goal.pose.orientation.y = self.goal_pose[4]
        goal.pose.orientation.z = self.goal_pose[5]
        goal.pose.orientation.w = self.goal_pose[6]
        self.pub.publish(goal)
	deletion = -1
	print(goal)
	length = len(self.bit_line)/3
	for i in range(length):
		#print("bit_line",self.bit_line[3*i+2][0])
		#print("min_line_num",self.min_line_num)
		if(self.bit_line[3*i+2][0] == self.min_line_num):
			deletion = i
			#print(deletion)
	abc = len(self.bit_line)
	if (deletion>=0):
		#print(self.bit_line[3*deletion:3*deletion+3])
		del self.bit_line[3*deletion:3*deletion+3]
	d = len(self.bit_line)
	self.seq = self.seq + 1

###############################################################################################




 
				

    def mapp_line_point(self):

	self.trajectorymap.header.frame_id = 'line_dot'
	self.trajectorymap.header.seq = self.sequence
	self.trajectorymap.header.stamp = rospy.Time.now()
	self.trajectorymap.info.height = self.height
	self.trajectorymap.info.width = self.width
	self.trajectorymap.info.origin.position.x = -10
	self.trajectorymap.info.origin.position.y = -10
	self.trajectorymap.info.origin.position.z = 0
	self.trajectorymap.info.origin.orientation.w = 1
	self.trajectorymap.info.origin.orientation.x = 0
	self.trajectorymap.info.origin.orientation.y = 0
	self.trajectorymap.info.origin.orientation.z = 0
	self.trajectorymap.info.resolution = self.resolution

        self.pub1.publish(self.trajectorymap)



	self.sequence += self.sequence +1
	
    def making_line_map(self):
	
	for i in range(self.height*self.width):
		self.trajectorymap.data.append(0)

	for i in range(len(self.line_only)/3):	
		num = self.line_only[3*i][0] + self.height * self.line_only[3*i][1]
		self.trajectorymap.data[num] = 100
		num = self.line_only[3*i+1][0] + self.height * self.line_only[3*i+1][1]
		self.trajectorymap.data[num] = 100
	
    def spin(self):
	r = rospy.Rate(self.rate)
	

	self.cost_function()
	self.planning()
	self.zig()
																																																																																	
	while not rospy.is_shutdown():
		self.update()
		r.sleep()

    def goal_statu_cb(self, data):
	print("goal reached")
	self.a = data.status.status
	if(self.a == 3):
		self.flag = False
		print("entered to chang on_off")
		print(self.on_off_line)
		if(self.on_off_line == 1):
			self.on_off_line = 0
		else:
			self.on_off_line = 1
		print(self.on_off_line)



    def update(self):
	self.mapp_line_point()
	############Writing_Plan################
	if(self.flag == False):		
		
		self.cost_function()
		self.planning()
		self.zig()
		self.flag = True
	########################################




	




if __name__ == '__main__':
    """ main """
    zig = ZigZag_Ver1()
    
    zig.spin()


