#!/usr/bin/env python
# license removed for brevity

import cv2
from cv_bridge import CvBridge
import numpy as np
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
from rospy.numpy_msg import numpy_msg
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal,MoveBaseActionResult
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Path



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
		self.flag1 = 0
		self.seq = 0
		self.theta = 0
		self.pix = []
		self.pix2 = []
		self.pix3 = []
		self.lin_gap = 5
		self.dis = 0.2
		self.originx = 0
		self.originy = 0
		self.turning_flag = False
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
		#rospy.Subscriber('map_metadata' , MapMetaData , self.meta_map_cb)
		self.pub = rospy.Publisher('path',Path, queue_size=100)
		rospy.loginfo("-I- %s started" % self.nodename)
		self.rate = rospy.get_param('~rate',2)
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
			self.originx = data.info.origin.position.x
			self.originy = data.info.origin.position.y

			for i in range(self.width*self.height):
				if(self.map_array[i] == 100 ):
					x,y = divmod(i,self.height)
					self.pix.append([x,y])
			self.map_flag +=1
			
			zig3.global_variable_declaration(self.height,self.width,self.resolution,self.map_array)
			rospy.loginfo("got %d message" % self.map_flag)


	
	def goal_statu_cb(self,data):
		if(data.status.status == 3):	# if goal is reached
			#self.flag2 = False    #if owner reached guide_dog, then all the flag is initialized
			#self.counting_flag = True #evaluating_start
			self.turning_flag = True
			self.flag1 = self.flag1 + 1				#0 = straight / 2 = curve / 3 = turning



	def c_cell_cal(self):

		kernel = np.ones((3, 3), np.uint8)
		#print(kernel)
		kernel[0][0] = 0
		kernel[2][0] = 0
		kernel[0][2] = 0
		kernel[2][2] = 0
		#print(kernel)
		arr_test = np.zeros((self.height,self.width,3), dtype="uint8")	# making np as opencv mat type
    		ddata = cv2.cvtColor(arr_test, cv2.COLOR_BGR2GRAY)		# bgr2gray

		num = 0
		for i in range(len(ddata)):
			for j in range(len(ddata[i])):
				ddata[i][j] = self.map_array[num]
				num = num + 1

		ddata1 = ddata #* (2.55 )

		dilation = cv2.dilate(ddata1, kernel, iterations=self.infl_factor)


		#cv2.imshow('dilation', dilation)
	   	#cv2.waitKey(1000)


		dst, contours, hierarchy = cv2.findContours(dilation , cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
		#print(len(contours), hierarchy)

		arr_test1 = 100*np.ones_like(arr_test)
		ddata2 = cv2.cvtColor(arr_test1, cv2.COLOR_BGR2GRAY)

	
		for idx, cont in enumerate(contours):
			if(idx %2 != 0):
				cv2.drawContours(ddata2, contours, idx, (0,0,0), -1)

		num = 0


		for i in range(len(dilation)):
			for j in range(len(dilation[i])):
				self.map_array[num] = dilation[i][j].view(np.int8)####### very important turning uint8 to int8
				num = num + 1





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
		self.trajectorymap.info.origin.position.x = self.originx
		self.trajectorymap.info.origin.position.y = self.originy
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
		

		self.trajectorymap.data = self.map_array

		for i in range(3):
			self.trajectorymap.data[500 + self.height*i]=100
			self.trajectorymap.data[500 - self.height*i]=100
		for i in range(int(len(self.line_only)/self.jumhyul_dis)):	
			num = self.line_only[self.jumhyul_dis*i][0] + self.width * self.line_only[self.jumhyul_dis*i][1]
			self.trajectorymap.data[num] = 120
		#self.trajectorymap.data[self.find_right_down_point()]=110
		#print(len(self.line_only))
		#print(self.line_only)
		#self.trajectorymap.data = np.zeros(self.height*self.width)
		#self.trajectorymap.data = self.map_array
		#self.trajectorymap.data[0] = 100
		#self.trajectorymap.data[self.width] = 100
		#self.trajectorymap.data[1] = 100
		#print(self.height,self.width)

		#for i,letter in enumerate(self.map_array):
		#	self.trajectorymap.data[i] = int(letter)
			#print(int(letter),letter)


		'''
		for i in range(int(len(self.line_only)/self.jumhyul_dis)):	
			num = self.line_only[self.jumhyul_dis*i][0] +self.width * self.line_only[self.jumhyul_dis*i][1]	
			#print(self.height, num)
			if( num < self.height * self.width - 1 ):
				self.trajectorymap.data[num] = 100
				#self.trajectorymap.data[4*self.height + 16] = 100
		'''

	def estimate_current_position(self):
        	(self.robot_pose,self.rot) = self.listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))



	def determine_next_orientation(self,a):
		dx = self.line_coor[a+self.jumhyul_dis][0]-self.line_coor[a][0]
		dy = self.line_coor[a+self.jumhyul_dis][1]-self.line_coor[a][1]
		if(dx != 0):
			self.theta = math.atan2(dy,dx)

	def publish_waypoints(self):
      	  	"""
        	Publish the ROS message containing the waypoints
        	"""
		
        	self.msg = Path()
        	self.msg.header.frame_id = "path_frame"
        	self.msg.header.stamp = rospy.Time.now()
		self.msg.header.seq = 1
		line_num = 0
		i = 0
	        pose1 = PoseStamped()
		pose1.header.seq = i
		pose1.header.frame_id = "path_frame"
        	pose1.pose.position.x = 0
        	pose1.pose.position.y = 0
		pose1.pose.position.z = 0
        	        		#quaternion = tf.transformations.quaternion_from_euler(0, 0, 1)
		self.determine_next_orientation(i)
		quaternion = quaternion_from_euler(0,0,self.theta)
		pose1.pose.orientation.x = quaternion[0]
	        pose1.pose.orientation.y = quaternion[1]
		pose1.pose.orientation.z = quaternion[2]
		pose1.pose.orientation.w = quaternion[3]
		self.msg.poses.append(pose1)

		i =1
		if(self.line_only is not []):
			for w in self.line_coor:
				
	        	       	pose = PoseStamped()
				pose.header.seq = i
				pose.header.frame_id = "path_frame"
        		       	pose.pose.position.x = w[0]
        		       	pose.pose.position.y = w[1]
				pose.pose.position.z = 0
        	        	#quaternion = tf.transformations.quaternion_from_euler(0, 0, 1)



				quaternion = quaternion_from_euler(0,0,self.theta)
			        pose.pose.orientation.x = quaternion[0]
			        pose.pose.orientation.y = quaternion[1]
			        pose.pose.orientation.z = quaternion[2]
			        pose.pose.orientation.w = quaternion[3]
				self.msg.poses.append(pose)
				#print(w[0],w[1],self.theta,i)


	        	       	pose3 = PoseStamped()
				pose3.header.seq = i
				pose3.header.frame_id = "path_frame"
        		       	pose3.pose.position.x = w[0]
        		       	pose3.pose.position.y = w[1]
				pose3.pose.position.z = 0



				if(line_num<(len(self.line_coor)-self.jumhyul_dis-1)):
					#print("len(self.lin_coor)",len(self.line_coor),line_num,i)
					self.determine_next_orientation(line_num)
				quaternion = quaternion_from_euler(0,0,self.theta)
			        pose3.pose.orientation.x = quaternion[0]
			        pose3.pose.orientation.y = quaternion[1]
			        pose3.pose.orientation.z = quaternion[2]
			        pose3.pose.orientation.w = quaternion[3]
				self.msg.poses.append(pose3)
				#print(w[0],w[1],self.theta)



				i = i+1
					


				line_num = line_num + 1

		self.pub.publish(self.msg)
		
	


	def drawing_path(self):

		r_c_point= self.find_right_down_point()
		l_c_point= self.find_left_up_point()

		sy,sx = divmod(r_c_point,self.height)
		fy,fx = divmod(l_c_point,self.height)

		now = [sx,sy]

		incre = int((fy-sy)/self.lin_gap)+2
		flag = 0 
		#self.line_only.append([sx,sy,0])
		print(incre)
		for i in range(incre-3):
			if(self.line_only != []):
				num = self.line_only[len(self.line_only)-1][0] + self.width *self.line_only[len(self.line_only)-1][1]
			else:
				num = sx + sy*(self.width)*(i+1)	
			self.draw_lines(num,i+1,sy)
			self.up_down_flag = self.up_down_flag + 1
			self.line_only[len(self.line_only)-1][2] = 3
		##########################################################################################
		
			n = len(self.line_only)
			now = self.line_only[len(self.line_only)-1]
			y,x = divmod(now[0] + (now[1]+ self.lin_gap)*self.width , self.width)

			#print(now,x,y)

			if(i<incre-1):
				if(self.up_down_flag%2==0):
					y,x = divmod(now[0] + (now[1]+ self.lin_gap)*self.width , self.width)
					for j in range(self.lin_gap):
						if(now[0]+(now[1]+j)*self.width<self.height*self.width):
							if(self.map_array[now[0]+(now[1]+j)*self.width] == 0):
								self.line_only.append([now[0],now[1]+j,3])
				else:
					y,x = divmod(now[0] + (now[1]+ self.lin_gap)*self.width , self.width)
	
					for j in range(self.lin_gap):
						if(now[0]+(now[1]+j)*self.width<self.height*self.width):
							if(self.map_array[now[0]+(now[1]+j)*self.width] == 0):
								self.line_only.append([now[0],now[1]+j,3])
		
		


	def draw_lines(self, starting_array,ith,sy): 

		yy,xx = divmod(starting_array,self.width)
		flag1 =0
		direction = 0
		print(ith)
		#print("xx",xx,yy,starting_array)
		if(self.up_down_flag%2==0):
			for i in range(self.width-xx-1):
				flag = 0
				x = i+xx+1
				y = self.lin_gap*(ith)+sy
				j = x + self.width * y
				#print(self.map_array[j])
				if(j < self.height*self.width):
					if(self.map_array[j] !=0):
						for h in range(5):
							a = j + h*self.width
							b = j - h*self.width

							if((a < self.height*self.width) and (b< self.height*self.width)):
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
										y_y,x_x = divmod(a,self.width)
										self.line_only.append([x_x,y_y,2]) #0 = straight / 2 = curve / 3 = turning
							
								elif((direction == 2) and (self.map_array[b] == 0)):
									if(flag == 0):
										flag = 1
										y_y,x_x = divmod(b,self.width)
										self.line_only.append([x_x,y_y,2]) #0 = straight / 2 = curve / 3 = turning
					else:
						self.line_only.append([x,y,0])  #0 = straight / 2 = curve / 3 = turning


		else:
			for i in range(xx):
				x = xx - i 
				y = self.lin_gap*(ith)+sy
				j = x + self.width * y
				flag = 0

				if(j < self.height*self.width):
					if(self.map_array[j] !=0):
						for h in range(5):
							a = j + h*self.width
							b = j - h*self.width
							if((a < self.height*self.width) and (b < self.height*self.width)):
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
										y_y,x_x = divmod(a,self.width)
										self.line_only.append([x_x,y_y,2]) #0 = straight / 2 = curve / 3 = turning
							
								elif((direction == 2) and (self.map_array[b] == 0)):
									if(flag == 0):
										flag = 1
										y_y,x_x = divmod(b,self.width)
										self.line_only.append([x_x,y_y,2]) #0 = straight / 2 = curve / 3 = turning
					else:
						self.line_only.append([x,y,0])

	def pixel_to_meter(self):
		for j in range(int(len(self.line_only))):
			s_x = (self.line_only[j][0]+(self.originx/self.resolution))*self.resolution
			s_y = (self.line_only[j][1]+(self.originy/self.resolution))*self.resolution
			num = self.line_only[j][2]
			self.line_coor.append([s_x,s_y,num])

	def turning(self):
		self.orientation_straight(self.ith+ 2*self.jumhyul_dis)


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



	def determine_next_orientation(self,a):
		dx = self.line_coor[a+self.jumhyul_dis][0]-self.line_coor[a][0]
		dy = self.line_coor[a+self.jumhyul_dis][1]-self.line_coor[a][1]
		if(dx != 0):
			self.theta = math.atan2(dy,dx)
		elif(dy>0):
			self.theta =0
		elif(dy<0):
			self.theta = math.pi

	def spin(self):
		r = rospy.Rate(self.rate)
		self.c_cell_cal()
		self.drawing_path()
		self.making_line_map()	
		self.pixel_to_meter()
		self.mapp_line_point()
		self.publish_waypoints()
		print(self.line_only)
		while not rospy.is_shutdown():
			self.update()
			r.sleep()



	def update(self):
		if(self.map_flag ==1):

			if(self.counting_flag==True):
				self.planning1234()

			self.mapp_line_point()
			
			#print("self.counting_flag,self.flag2,self.flag1")
			#print(self.counting_flag,self.flag2,self.flag1)
			

			'''
			if(self.flag2 == False):
				self.zig()
			elif(self.flag2 == True):		## if guide_dog needs to wait for the owner
				
				self.counting_flag = False    ##publish once
				self.zig()	
				if(self.flag1 >= 1):		##arived and turn
					
					self.turning()
					#print("theta",self.theta)
					self.zig()
					

					if((self.flag1 >= 2) and (self.turning_flag == True)):
						self.flag2 = False
						self.flag1 = 0
						self.counting_flag = True
						#print(self.counting_flag)
						'''


if __name__ == '__main__':
   	gd = Guide_Dog()
	gd.spin()
		
