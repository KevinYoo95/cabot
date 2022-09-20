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
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler
import zig3



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
		self.flag = False
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
		self.theta =0
		self.line_per_y = []
		self.goal_pose = [0,0,0,0,0,0,1]
		self.seq = 0
		self.line_num1 = 0
		self.chomchom = 2
		self.up_down_flag = 0


		rospy.init_node("Would_it_work")
		self.nodename = rospy.get_name()
		rospy.loginfo("-I- %s started" % self.nodename)
		self.rate = rospy.get_param('~rate',1)
		self.pub1 = rospy.Publisher('line_dot', OccupancyGrid, queue_size=1000)
		self.pub = rospy.Publisher('move_base/DWAPlannerROS/global_plan',Path, queue_size=100)
		rospy.Subscriber("move_base/result", MoveBaseActionResult, self.goal_statu_cb)
		rospy.Subscriber('cspace',OccupancyGrid,self.map_cb)
		rospy.Subscriber("odom", Odometry, self.Odom_cb)
		rospy.sleep(3)
	

        def map_cb(self, data):
		if(self.map_flag == 0):
			self.map_array = data.data
			self.width = data.info.width
			self.height = data.info.height
			self.resolution = data.info.resolution
			self.map_x = data.info.origin.position.x + (self.width/2)*self.resolution
			self.map_y = data.info.origin.position.y + (self.height/2)*self.resolution
			self.map_origin_x = data.info.origin.position.x 
			self.map_origin_y = data.info.origin.position.y 
			self.dr_pixel = int((self.radius)/self.resolution)
	
			self.c_cell = list(self.map_array)
			self.dr_pixel = self.radius/self.resolution

			print("map_recieved")

			for i in range(self.width*self.height):
				if(self.cell1[i] == 100 and self.flag == 0):
					x,y = divmod(i,384)
					self.pix.append([x,y])
					self.obs_xpix.append(x)
					self.obs_ypix.append(y)

			self.flag += 1


	def map_cb(self, data):
		if(self.map_flag == 0):
			self.map_array = data.data
			self.width = data.info.width
			self.height = data.info.height
			self.resolution = data.info.resolution
			self.x_in_meter = data.info.origin.position.x 
			self.y_in_meter = data.info.origin.position.y 
			self.dr_pixel = int((self.radius)/self.resolution)
			self.map_array = list(self.map_array)
			self.map_flag = 1 # map_recieved
			print("map_recieved")
			zig3.global_variable_declaration(self.height,self.width,self.resolution,self.map_array)



	def Odom_cb(self, data):
		self.base_x = data.pose.pose.position.x
		self.base_y = data.pose.pose.position.y




	def find_right_down_point(self):
		i = 0
		for i in range( self.height*self.width):
			if(self.map_array[i]==0):
				break
		return i



	def draw_(self):

		self.r_c_point= self.find_right_down_point()
		pixxx = self.base_x*self.resolution+200
		piyyy = self.base_y*self.resolution + 200
		#self.r_c_point= int(pixxx + piyyy*self.height)
		y,x = divmod(self.r_c_point,self.height)
		for i in range(20):	
			
			self.draw_lines(y*self.height + x +3*(self.height * int(self.radius/self.resolution))*i)
			self.up_down_flag = self.up_down_flag + 1


		self.bit_line = self.line_only

	

	def draw_lines(self,starting_array): 
		map_state =0
		line_= 0
		line_per_y = []
		yy,xx = divmod(starting_array,self.height)

		if(self.up_down_flag%2==0):
			for i in range(self.height-xx):
				x= xx+i
				y = yy
				j = x + y*self.height
				flag = 0
				if(self.map_array[j] == 0):
					self.line_only.append([x,y])
				elif(self.map_array[j] == 100):
					for h in range(10):
						a = j+h*self.height
						b = j-h*self.height
						if(self.map_array[a] == 0):
							if(flag == 0):
								y_y,x_x = divmod(a,self.height)
								flag = 1
								self.line_only.append([x_x,y_y])

						elif(self.map_array[b]==0):
							if(flag == 0):
								y_y,x_x = divmod(b,self.height)
								flag = 1
								self.line_only.append([x_x,y_y])

		else:

			for i in range(self.height):
				x= self.height - i
				y = yy
				j = x + y * self.height
				flag = 0
				if(self.map_array[j] == 0):
					self.line_only.append([x,y])
				elif(self.map_array[j] == 100):
					for h in range(10):
						a = j+h*self.height
						b = j-h*self.height
						if(self.map_array[a] == 0):
							if(flag == 0):
								y_y,x_x = divmod(a,self.height)
								flag = 1
								self.line_only.append([x_x,y_y])

						elif(self.map_array[b]==0):
							if(flag == 0):
								y_y,x_x = divmod(b,self.height)
								flag = 1
								self.line_only.append([x_x,y_y])
			a =0
			
		
		if(line_per_y != []):
			self.line_per_y.append(line_per_y)
		
	


        def c_cell_cal(self):
	    	a = 0
	    	b = 0

		for i in range(len(self.pix)):
			for j in range(int(self.infl_factor * self.dr_pixel)):
				for k in range(int(self.infl_factor * self.dr_pixel)):
				
					if(k*k+j*j<self.infl_factor*self.dr_pixel):
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
		self.ccell_ = list(self.cell1)
		for i in range(len(aa)):
			self.ccell_[aa[i][0]*384+aa[i][1]] = 100

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

		num = 0
		num =1 
		for j in range(len(self.line_only)):
			s_x = (self.line_only[j][0]-200)*self.resolution
			s_y = (self.line_only[j][1]-200)*self.resolution
			self.line_coor.append([s_x,s_y])
		num = num + 1
		



	def determine_next_orientation(self,a):
		dx = self.line_coor[a+self.chomchom][0]-self.line_coor[a][0]
		dy = self.line_coor[a+self.chomchom][1]-self.line_coor[a][1]
		if(dx != 0):
			self.theta = math.atan2(dy,dx)
		elif(dy>0):
			self.theta =0
		elif(dy<0):
			self.theta = math.pi

        def goal_statu_cb(self, data):
		print("goal reached")
		self.a = data.status.status
		if(self.a == 3):
			self.flag = False

	def planning(self):	
		#print(self.line_coor)
		#print(self.line_num1)
        	self.goal_pose[0] = self.line_coor[self.chomchom*self.line_num1][0]
        	self.goal_pose[1] = self.line_coor[self.chomchom*self.line_num1][1]
        	self.goal_pose[2] = 0
		if(self.line_num1<(len(self.line_coor)-1)):
			self.determine_next_orientation(self.chomchom*self.line_num1)
		quaternion = quaternion_from_euler(0,0,self.theta)
        	self.goal_pose[3] = quaternion[0]
        	self.goal_pose[4] = quaternion[1]
        	self.goal_pose[5] = quaternion[2]
        	self.goal_pose[6] = quaternion[3]
		self.line_num1 = self.line_num1 + 1

		
		

	def publish_waypoints(self):
      	  	"""
        	Publish the ROS message containing the waypoints
        	"""
		
        	self.msg = Path()
        	self.msg.header.frame_id = 'map'
        	self.msg.header.stamp = rospy.Time.now()
		self.msg.header.seq = 1
		line_num = 0
		i = 0
	        pose1 = PoseStamped()
		pose1.header.seq = i
		pose1.header.frame_id = 'map'
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
				pose.header.frame_id = 'map'
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
				pose3.header.frame_id = 'map'
        		       	pose3.pose.position.x = w[0]
        		       	pose3.pose.position.y = w[1]
				pose3.pose.position.z = 0



				if(line_num<(len(self.line_coor)-self.chomchom-1)):
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
		self.seq = self.seq + 1
		self.line_num1 = self.line_num1 +1
		self.pub.publish(goal)
		print(goal)
		

    	def making_line_map(self):
		
		for i in range(self.height*self.width):
			self.trajectorymap.data.append(0)
		self.trajectorymap.data[self.r_c_point] = 100
		for i in range(3):
		
			self.trajectorymap.data[500 + i]=100
			self.trajectorymap.data[500 - i]=100
			self.trajectorymap.data[500 + self.height*i]=100
			self.trajectorymap.data[500 - self.height*i]=100
		
		for i in range(len(self.line_only)):	
			num = self.line_only[i][0] + self.height * self.line_only[i][1]
			self.trajectorymap.data[num] = 100
		#print(len(self.trajectorymap.data))

	def spin(self):
		r = rospy.Rate(self.rate)
																																																																													
		while not rospy.is_shutdown():
			self.update()
			r.sleep()



	def update(self):
		if(self.map_flag ==1):
			if(self.drawing_line_flag ==0):
				
				self.draw_()
				self.pixel_to_meter()
				#print(self.line_coor)
				print(len(self.line_coor))
				self.drawing_line_flag =1
				self.making_line_map()	
			self.publish_waypoints()
			self.mapp_line_point()







	




if __name__ == '__main__':
   	zig = ZigZag_Ver5()
	zig.spin()
		
