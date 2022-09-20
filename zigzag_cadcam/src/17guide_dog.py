#!/usr/bin/env python
# license removed for brevity

import roslib
import rospy
import math
import tf
import geometry_msgs.msg
import matplotlib.pyplot as plt
import numpy as np
import zig3
import std_msgs.msg 
import nav_msgs.msg
import visualization_msgs.msg
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal,MoveBaseActionResult
from tf.transformations import euler_from_quaternion, quaternion_from_euler



#############################################################################
class Guide_Dog():
	def __init__(self):
		self.map_array = None
		self.width = None
		self.height = None
		self.resolution = None
		self.map_flag = None
		self.pix = []
		self.ith = 0
		self.map_flag = 0
		self.up_down_flag = 0
		self.line_only=[]
		self.counting_flag = False
		self.flag2 = True
		self.flag1 = False
		self.seq = 0
		self.theta = 0
		self.pix = []
		self.pix2 = []
		self.pix3 = []

		self.lin_gap = 5
		self.dis = 0.5 

		self.jumhyul_dis = 1
		self.infl_factor = 3+2 # pixel -> 15cm + 10cm = 25cm of inflation
		self.goal = PoseStamped()
		self.trajectorymap = OccupancyGrid()
		self.robot_pose = Pose()
		self.line_coor = []
		





 		self.nodename = rospy.get_name()
		rospy.init_node("Please_work")
		self.listener =tf.TransformListener()
		rospy.Subscriber("move_base/result", MoveBaseActionResult, self.goal_statu_cb)
		rospy.Subscriber('map' , OccupancyGrid , self.map_cb)
		rospy.loginfo("-I- %s started" % self.nodename)
		self.rate = rospy.get_param('~rate',10)
		self.pub1 = rospy.Publisher('line_dot', OccupancyGrid, queue_size=1000)
		self.pub2 = rospy.Publisher('move_base_simple/goal',PoseStamped, queue_size=100)
		rospy.loginfo("waiting for map")
		rospy.sleep(3)


	def map_cb(self, data):
		if(self.map_flag <2):
			self.map_array = list(data.data)
			self.width = data.info.width
			self.height = data.info.height
			self.resolution = data.info.resolution
			for i in range(self.width*self.height):
				if(self.map_array[i] == 100 ):
					x,y = divmod(i,384)
					self.pix.append([x,y])
			self.map_flag +=1
			
			zig3.global_variable_declaration(self.height,self.width,self.resolution,self.map_array)
			rospy.loginfo("got %d message" % self.map_flag)


	def goal_statu_cb(self,data):
		if(data.status.status == 3):	# if goal is reached
			self.flag2 = False
			self.flag1 = False    #if owner reached guide_dog, then all the flag is initialized
			self.counting_flag = True #evaluating_start
									#0 = straight / 2 = curve / 3 = turning

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
   	def making_line_map(self):
		
		for i in range(self.height*self.width):
			self.trajectorymap.data.append(0)
		for i in range(3):
			self.trajectorymap.data[500 + self.height*i]=100
			self.trajectorymap.data[500 - self.height*i]=100
		
		for i in range(int(len(self.line_only)/self.jumhyul_dis)):	
			num = self.line_only[self.jumhyul_dis*i][0] + self.height * self.line_only[self.jumhyul_dis*i][1]
			self.trajectorymap.data[num] = 100


	def estimate_current_position(self):
        	(self.robot_pose,self.rot) = self.listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
	def drawing_path(self):

		r_c_point= self.find_right_down_point()
		l_c_point= self.find_left_up_point()

		sy,sx = divmod(r_c_point,self.height)
		fy,fx = divmod(l_c_point,self.height)

		now = [sx,sy]

		incre = int((fy-sy)/self.lin_gap)+2
		flag = 0 
		for i in range(incre):
			self.draw_lines(r_c_point + self.height * self.lin_gap*(i+1))
			self.up_down_flag = self.up_down_flag + 1
			self.line_only[len(self.line_only)-1][2] = 3
		##########################################################################################
		
			n = len(self.line_only)
			now = self.line_only[len(self.line_only)-1]

			if(i<incre-1):
				if(self.up_down_flag%2==0):
					y,x = divmod(now[0] + now[1]*self.height +  self.height * self.lin_gap, self.height)
					for j in range(y-now[1]):
						if(self.map_array[now[0]+(now[1]+j)*self.height] != 100):
							self.line_only.append([now[0],now[1]+j,3])
				else:
					y,x = divmod(r_c_point  +  self.height *self.lin_gap*(i+1), self.height)
	
					for j in range(y-now[1]):
						if(self.map_array[now[0]+(now[1]+j)*self.height] != 100):
							self.line_only.append([now[0],now[1]+j,3])
		


	def draw_lines(self, starting_array): 

		yy,xx = divmod(starting_array,self.height)
		flag1 = 0
		direction = 0

		if(self.up_down_flag%2==0):
			for i in range(self.height):
				x = i
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
								direction = 1
						elif(self.map_array[b] ==0):
							if(flag1 == 0):
								flag1 = 1
								direction = 2
						if((direction == 1) and (self.map_array[a] == 0)):
							if(flag == 0):
								flag = 1
								y_y,x_x = divmod(a,self.height)
								self.line_only.append([x_x,y_y,2]) #0 = straight/2 = curve/3 = turning
							
						elif((direction == 2) and (self.map_array[b] == 0)):
							if(flag == 0):
								flag = 1
								y_y,x_x = divmod(b,self.height)
								self.line_only.append([x_x,y_y,2])#0 = straight/2 = curve/3 = turning

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
								direction = 1

						elif(self.map_array[b] ==0):
							if(flag1 == 0):
								flag1 = 1
								direction = 2


						if((direction == 1) and (self.map_array[a] == 0)):
							if(flag == 0):
								flag = 1
								y_y,x_x = divmod(a,self.height)
								self.line_only.append([x_x,y_y,2]) #0 = straight / 2 = curve / 3 = turning
							
						elif((direction == 2) and (self.map_array[b] == 0)):
							if(flag == 0):
								flag = 1
								y_y,x_x = divmod(b,self.height)
								self.line_only.append([x_x,y_y,2]) #0 = straight / 2 = curve / 3 = turning


	def pixel_to_meter(self):
		for j in range(int(len(self.line_only))):
			s_x = (self.line_only[j][0]-200)*self.resolution
			s_y = (self.line_only[j][1]-200)*self.resolution
			num = self.line_only[j][2]
			self.line_coor.append([s_x,s_y,num])


   	def zig(self):
		goal = PoseStamped()
		goal.header.seq = self.seq
		goal.header.frame_id = 'map'
		goal.header.stamp = rospy.Time.now()
		goal.pose.position.x = self.line_coor[self.ith][0]
		goal.pose.position.y = self.line_coor[self.ith][1]
		goal.pose.position.z = 0
		quaternion = quaternion_from_euler(0,0,self.theta)
		goal.pose.orientation.x = quaternion[0]
		goal.pose.orientation.y = quaternion[1]
		goal.pose.orientation.z = quaternion[2]
		goal.pose.orientation.w = quaternion[3]

		self.pub2.publish(goal)
		self.seq = self.seq + 1

	def orientation_straight(self, a):
		dx = self.line_coor[a + self.jumhyul_dis][0] - self.line_coor[a][0]
		dy = self.line_coor[a + self.jumhyul_dis][1] - self.line_coor[a][1]
		self.theta = math.atan2(dy, dx)

	def orientation_curved(self, a):
		dx = []
		dy = []
		theta = []
		theta_sum = 0
		for i in range(5):
			dx.append(self.line_coor[a + self.jumhyul_dis * i][0] - self.line_coor[a][0])
			dy.append(self.line_coor[a + self.jumhyul_dis * i][1] - self.line_coor[a][1])
			theta.append(math.atan2(dy[i], dx[i]))
		for i in range(len(theta)):
			theta_sum = theta_sum + theta[i]
		self.theta = theta_sum / len(theta)

	def distance_evaluation(self):
		self.estimate_current_position()
		for i in range(len(self.line_coor)-self.ith):
			self.estimate_current_position()
			if(  ((self.line_coor[self.ith][0]-self.robot_pose[0])**2 + (self.line_coor[self.ith][1]-self.robot_pose[1])**2 >self.dis**2)  ):
				break
			self.ith= self.ith+self.jumhyul_dis
			if(  (self.line_coor[self.ith][2]!=self.line_coor[self.ith+self.jumhyul_dis][2])):
				break

		if ((self.line_coor[self.ith][2] != self.line_coor[self.ith + self.jumhyul_dis][2])):
			self.flag2 = True  # publish and wait for the owner

	def planning1234(self):
		self.distance_evaluation()

		if(self.line_coor[self.ith][2]==2):				# Orientation
			self.orientation_curved(self.ith)
		else:
			self.orientation_straight(self.ith)




	def spin(self):
		r = rospy.Rate(self.rate)
		self.c_cell_cal()
		self.drawing_path()
		self.making_line_map()	
		self.pixel_to_meter()
		self.mapp_line_point()

		while not rospy.is_shutdown():
			self.update()
			r.sleep()



	def update(self):
		if(self.map_flag ==1):

			if(self.counting_flag==True):
				self.planning1234()
			


			#print(self.flag2,self.flag1,self.line_coor[self.ith][2],self.theta)
			#print(self.robot_pose,self.line_coor[self.ith])
			if(self.flag2 == False):
				self.zig()
			elif(self.flag2 == True):		## if guide_dog needs to wait for the owner
				if(self.flag1 == False):
					self.counting_flag = False    ##publish once
					self.zig()
					self.flag1 = True


if __name__ == '__main__':
   	gd = Guide_Dog()
	gd.spin()
		
