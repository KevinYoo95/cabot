#!/usr/bin/env python
# license removed for brevity
import rospy
import math
import matplotlib.pyplot as plt
import numpy as np
import zig3
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from actionlib_msgs.msg import GoalStatusArray
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal,MoveBaseActionResult
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import  Odometry
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
import matplotlib.pyplot as plt



#####################################################################################
class ZigZag_Ver5():
	def __init__(self):
		self.map_array = None
		self.width = None
		self.height = None
		self.resolution = None
		self.x_in_meter = None
		self.y_in_meter = None
		self.dr_pixel = None
		self.map_flag = 0
		self.flag = True
		self.flag1 = False
		self.line_num = 0
		self.drawing_line_flag =0
		self.r_c_point = None
		self.bit_line = None
		self.line_only = []
		self.line_points = []
		self.line_coor = []
		self.radius = 0.15
        	self.msg = None
		self.publish_flag = 0
		self.trajectorymap = OccupancyGrid()
		self.theta = 0
		self.line_per_y = []
		self.goal_pose = [0,0,0,0,0,0,1]
		self.goal = PoseStamped()
		self.seq = 0
		self.line_num1 = 0
		self.up_down_flag = 0
		self.pix = []
		self.pix2 = []
		self.pix3 = []
		self.infl_factor = 3+2 # pixel -> 15cm + 10cm = 25cm of inflation
		self.base_x =0
		self.base_y = 0
		self.interpolator_flag = 0
		self.robot_pose = Pose()
		self.jumhyul = []
		self.a_set = []
		self.b_set = []
		self.c_set = []
		self.line_only_prev = []
		self.l_c_point =0

		self.criterion_points = []

		self.dis_criterion = 0.3
		self.dis_ = 0.2


		self.ith = 0
		self.jumhyul_dis = 1 #pixel

		self.line_flag = None
		self.flag2 = True
		self.counting_flag = False
		self.direction = 0
		self.inf_space = []
		self.lin_gap = 6 # pixel
		self.ith_publish = 0

		
		rospy.init_node("Please_work")
		self.nodename = rospy.get_name()
		
		rospy.Subscriber("move_base/result", MoveBaseActionResult, self.goal_statu_cb)
		rospy.Subscriber('map' , OccupancyGrid , self.map_cb)
		rospy.Subscriber("odom", Odometry, self.Odom_cb)
		rospy.loginfo("-I- %s started" % self.nodename)
		self.rate = rospy.get_param('~rate',10)
		self.pub1 = rospy.Publisher('line_dot', OccupancyGrid, queue_size=1000)
		self.pub = rospy.Publisher('path',Path, queue_size=100)
		self.pub2 = rospy.Publisher('move_base_simple/goal',PoseStamped, queue_size=100)
		rospy.loginfo("waiting for map")
		rospy.sleep(3)
	
		for i in range(self.height*self.width):
			self.trajectorymap.data.append(0)


	def map_cb(self, data):
		if(self.map_flag <2):
			self.map_array = list(data.data)
			self.width = data.info.width
			self.height = data.info.height
			self.resolution = data.info.resolution
			self.map_origin_x = data.info.origin.position.x 
			self.map_origin_y = data.info.origin.position.y 
			self.cell1 = data.data
			self.c_cell = list(self.cell1)
			self.dr_pixel = self.radius/self.resolution
			for i in range(self.width*self.height):
				if(self.map_array[i] == 100 ):
					x,y = divmod(i,384)
					self.pix.append([x,y])
			self.flag += 1
			self.map_flag +=1
			
			zig3.global_variable_declaration(self.height,self.width,self.resolution,self.map_array)
			rospy.loginfo("got %d message" % self.map_flag)




	def Odom_cb(self, data):
		self.robot_pose = data.pose
		self.interpolator_flag = 1


	def c_cell_cal(self):
		for i in range(len(self.pix)):
			for j in range(int(self.infl_factor * self.infl_factor)):
				for k in range(int(self.infl_factor * self.infl_factor)):
					if(k*k+j*j<=self.infl_factor*self.infl_factor):
						a = self.pix[i][0]+j
						b = self.pix[i][0]-j
						c = self.pix[i][1]+k
						d = self.pix[i][1]-k
						self.pix2.append([a,c])
						self.pix2.append([b,d])
						self.pix2.append([a,d])
						self.pix2.append([b,c])
		self.pix3 = self.pix + self.pix2
		aa = [self.pix3 for self.pix3 in self.pix3]
		#print(len(aa))
		for i in range(len(aa)):
			self.map_array[aa[i][0]*384+aa[i][1]] = 100

		self.inf_space = self.map_array


	def find_right_down_point(self):
		for i in range( self.height*self.width):
			if(self.map_array[i]==0):
				break
		return i

	def find_left_up_point(self):
		for i in range( self.height*self.width):
			if(self.map_array[self.height*self.width-i-1]==0):
				break
		return self.height*self.width-i

	def draw_(self):
		self.r_c_point= self.find_right_down_point()
		self.l_c_point= self.find_left_up_point()

		pixxx = self.base_x*self.resolution + 200
		piyyy = self.base_y*self.resolution + 200
		now = []
		next = []
		sy,sx = divmod(self.r_c_point,self.height)
		fy,fx = divmod(self.l_c_point,self.height)
		now = [sx,sy ]
		incre = int((fy-sy)/self.lin_gap)+2
		#print("fy,sy,incre",fy,sy,incre)
		#print(3*( int(self.radius/self.resolution)))
		for i in range(incre):	
			self.draw_lines(self.r_c_point + self.height * self.lin_gap*i)

			now = self.line_only[len(self.line_only)-1]
			if(i<incre-1):
				if(self.up_down_flag%2==0):

					 
					y,x = divmod(now[0] + now[1]*self.height +  self.height * self.lin_gap, self.height)
	
					for j in range(y-now[1]):
						self.line_only.append([now[0],now[1]+j,3])

				else:

					y,x = divmod(self.r_c_point  +  self.height *self.lin_gap*(i+1), self.height)
	
					for j in range(y-now[1]):
						self.line_only.append([now[0],now[1]+j,3])
			
			self.direction = 0
			self.up_down_flag = self.up_down_flag + 1
		self.bit_line = self.line_only


	def draw_lines(self,starting_array): 
		map_state =0
		line_= 0
		line_per_y = []
		yy,xx = divmod(starting_array,self.height)
		flag1 = 0

		if(self.up_down_flag%2==0):
			for i in range(self.height-xx):
				x= xx+i
				y = yy
				j = x + y*self.height
				flag = 0
				if(self.map_array[j] == 0):
					self.line_only.append([x,y,0])  #0 = straight / 2 = curve / 3 = turning
				elif(self.map_array[j] == 100):
					for h in range(10):
						a = j+h*self.height
						b = j-h*self.height
						if(self.map_array[a] == 0):
							if(flag1 == 0):
								flag1 = 1
								self.direction = 1

						elif(self.map_array[b] ==0):
							if(flag1 == 0):
								flag1 = 1
								self.direction = 2


						if((self.direction == 1) and (self.map_array[a] == 0)):
							if(flag == 0):
								flag = 1
								y_y,x_x = divmod(a,self.height)
								self.line_only.append([x_x,y_y,2]) #0 = straight / 2 = curve / 3 = turning
							
						elif((self.direction == 2) and (self.map_array[b] == 0)):
							if(flag == 0):
								flag = 1
								y_y,x_x = divmod(b,self.height)
								self.line_only.append([x_x,y_y,2]) #0 = straight / 2 = curve / 3 = turning

		else:
			for i in range(self.height):
				x= self.height - i
				y = yy
				j = x + y * self.height
				flag = 0
				if(self.map_array[j] == 0):
					self.line_only.append([x,y,0])
				elif(self.map_array[j] == 100):
					for h in range(10):
						a = j + h*self.height
						b = j - h*self.height
						if(self.map_array[a] == 0):
							if(flag1 == 0):
								flag1 = 1
								self.direction = 1

						elif(self.map_array[b] ==0):
							if(flag1 == 0):
								flag1 = 1
								self.direction = 2


						if((self.direction == 1) and (self.map_array[a] == 0)):
							if(flag == 0):
								flag = 1
								y_y,x_x = divmod(a,self.height)
								self.line_only.append([x_x,y_y,2]) #0 = straight / 2 = curve / 3 = turning
							
						elif((self.direction == 2) and (self.map_array[b] == 0)):
							if(flag == 0):
								flag = 1
								y_y,x_x = divmod(b,self.height)
								self.line_only.append([x_x,y_y,2]) #0 = straight / 2 = curve / 3 = turning
	

					

   	def mapp_line_point(self):
	
		self.trajectorymap.header.frame_id = 'line_dot'
		self.trajectorymap.header.seq = 0
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


	def pixel_to_meter(self):

		for j in range(int(len(self.line_only))):
			s_x = (self.line_only[j][0]-200)*self.resolution
			s_y = (self.line_only[j][1]-200)*self.resolution
			num = self.line_only[j][2]
			self.line_coor.append([s_x,s_y,num])

		


   	def making_line_map(self):
		
		for i in range(3):
			self.trajectorymap.data[500 + self.height*i]=100
			self.trajectorymap.data[500 - self.height*i]=100

		#self.trajectorymap.data[self.l_c_point] = 100
		#self.trajectorymap.data[self.r_c_point] = 100

		#self.ith

		#num = self.line_only[self.ith][0] + self.height * self.line_only[self.ith][1]
		#self.trajectorymap.data[num] = 100

		for i in range(int(len(self.line_only)/self.jumhyul_dis)):	
			num = self.line_only[self.jumhyul_dis*i][0] + self.height * self.line_only[self.jumhyul_dis*i][1]
			self.trajectorymap.data[num] = 100




        def zig(self):
		
		self.goal.header.seq = self.seq
		self.goal.header.frame_id = 'map'
		self.goal.header.stamp = rospy.Time.now()
		self.goal.pose.position.x = self.line_coor[self.ith_publish][0]
		self.goal.pose.position.y = self.line_coor[self.ith_publish][1]
		self.goal.pose.position.z = 0


		quaternion = quaternion_from_euler(0,0,self.theta)
	        self.goal.pose.orientation.x = quaternion[0]
	        self.goal.pose.orientation.y = quaternion[1]
	        self.goal.pose.orientation.z = quaternion[2]
	        self.goal.pose.orientation.w = quaternion[3]

		self.pub2.publish(self.goal)
		self.seq = self.seq + 1
		self.line_num1 = self.line_num1 +1
		self.interpolator_flag = 0



        def goal_statu_cb(self, data):
		self.a = data.status.status
		if(self.a == 3):
			self.flag2 = False
			self.flag1 = False    #if owner reached guide_dog, then all the flag is initialized
			self.counting_flag = True #evaluating_start

		#0 = straight / 2 = curve / 3 = turning



	def orientation_straight(self,a):
		dx = self.line_coor[a + self.jumhyul_dis][0]-self.line_coor[a][0]
		dy = self.line_coor[a + self.jumhyul_dis][1]-self.line_coor[a][1]
		self.theta = math.atan2(dy,dx)


	def orientation_curved(self,a):
		dx = []
		dy = []
		theta = []
		theta_sum =0
		for i in range(5):
			dx.append(self.line_coor[a + self.jumhyul_dis*i][0] - self.line_coor[a][0])
			dy.append(self.line_coor[a + self.jumhyul_dis*i][1] - self.line_coor[a][1])
			theta.append( math.atan2(dy[i],dx[i]))
		for i in range(len(theta)):
			theta_sum = theta_sum + theta[i]
		self.theta = theta_sum/len(theta)


	def orientation_turning(self,a):
		dx = self.line_coor[a + self.jumhyul_dis][0]-self.line_coor[a][0]
		dy = self.line_coor[a + self.jumhyul_dis][1]-self.line_coor[a][1]
		#print(dy,dx)
		self.theta = math.atan2(dy,dx)



	def straight_line(self):
			
		if(self.line_coor[self.ith + self.jumhyul_dis][2] != 0):
			self.flag2 = True
			self.orientation_straight(self.ith)
		else:
			self.orientation_straight(self.ith)
				
			
	def curved_line(self):
		if(self.line_coor[self.ith + self.jumhyul_dis][2] != 2):
			self.flag2 = True
			self.orientation_curved(self.ith)
		else:
			self.orientation_curved(self.ith)


	def turning_line(self):
		if(self.line_coor[self.ith + self.jumhyul_dis][2] != 3):
			self.flag2 = True
			self.orientation_turning(self.ith)
		else:
			self.orientation_turning(self.ith)
			


	def distance_straight(self):
		robot_x = self.robot_pose.pose.position.x
		robot_y = self.robot_pose.pose.position.y
		for i in range(len(self.line_coor)-self.ith):
			if(((self.line_coor[self.ith][0] - robot_x)*(self.line_coor[self.ith][0] - robot_x) + (self.line_coor[self.ith][1]-robot_y)*(self.line_coor[self.ith][1]-robot_y) >self.dis_criterion*self.dis_criterion) or (self.line_coor[self.ith + self.jumhyul_dis][2]!=0)):
				self.ith_publish = self.ith
				for j in range(len(self.line_coor)-self.ith_publish):
					if((self.line_coor[self.ith][0] - self.line_coor[self.ith_publish][0])*(self.line_coor[self.ith][0] - self.line_coor[self.ith_publish][0]) + (self.line_coor[self.ith][1] - self.line_coor[self.ith_publish][1]) *(self.line_coor[self.ith][1] - self.line_coor[self.ith_publish][1]) > (self.dis_)*(self.dis_) or (self.line_coor[self.ith_publish + self.jumhyul_dis][2]!=0)):
						
						break
					self.ith_publish= self.ith_publish + self.jumhyul_dis
				break
				

			self.ith= self.ith+self.jumhyul_dis*i

	def distance_curved(self):
		robot_x = self.robot_pose.pose.position.x
		robot_y = self.robot_pose.pose.position.y
		for i in range(len(self.line_coor)-self.ith):
			if(((self.line_coor[self.ith][0] - robot_x)*(self.line_coor[self.ith][0] - robot_x) + (self.line_coor[self.ith][1]-robot_y)*(self.line_coor[self.ith][1]-robot_y) >self.dis_criterion*self.dis_criterion) or (self.line_coor[self.ith + self.jumhyul_dis][2]!=2)):
				self.ith_publish = self.ith
				break
			self.ith= self.ith+self.jumhyul_dis


	def distance_turning(self):
		robot_x = self.robot_pose.pose.position.x
		robot_y = self.robot_pose.pose.position.y
		for i in range(len(self.line_coor)-self.ith):
			if(((self.line_coor[self.ith][0] - robot_x)*(self.line_coor[self.ith][0] - robot_x) + (self.line_coor[self.ith][1]-robot_y)*(self.line_coor[self.ith][1]-robot_y) > self.dis_criterion * self.dis_criterion) or (self.line_coor[self.ith + self.jumhyul_dis][2]!=3)):
				self.ith_publish = self.ith
				break
			self.ith= self.ith+self.jumhyul_dis


	def planning(self):	
		if((self.line_coor[self.ith+ self.jumhyul_dis][2] == 0) ):
			#print("entered straight")
			self.distance_straight()
			self.straight_line()

		elif(self.line_coor[self.ith+ self.jumhyul_dis][2] == 2):
			#print("entered curved")
			self.distance_curved()
			self.curved_line()

		elif(self.line_coor[self.ith+ self.jumhyul_dis][2] == 3):
			#print("entered turning")
			self.distance_turning()
			self.turning_line()



	def spin(self):
		r = rospy.Rate(self.rate)
																																																																													
		while not rospy.is_shutdown():
			self.update()
			r.sleep()



	def update(self):
		if(self.map_flag ==1):
			if(self.drawing_line_flag ==0):
				self.c_cell_cal()
				self.draw_()
				self.making_line_map()	
				self.pixel_to_meter()

				self.drawing_line_flag =1

			if(self.counting_flag==True):
				#print("plan started")
				self.planning()
				self.making_line_map()	

			#self.making_line_map()	
			self.mapp_line_point()

			if((self.interpolator_flag ==1) ):	## Starting:True False -> on line: False False ->reaching goal point True True
				#print(self.flag2,self.flag1,self.counting_flag,self.ith,self.line_coor[self.ith+ self.jumhyul_dis][2],self.ith_publish,self.line_coor[self.ith_publish+ self.jumhyul_dis][2])
				#rospy.loginfo(self.flag2,self.flag1,self.counting_flag,self.theta)
				if(self.flag2 == False):
					self.zig()
				elif(self.flag2 == True):		## if guide_dog needs to wait for the owner
					if(self.flag1 == False):
						self.counting_flag = False    ##publish once
						self.zig()
						self.flag1 = True
						




if __name__ == '__main__':
   	zig = ZigZag_Ver5()
	zig.spin()
		




































