#!/usr/bin/env python
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


		rospy.init_node("Would_it_work")
		self.nodename = rospy.get_name()
		rospy.loginfo("-I- %s started" % self.nodename)
		self.rate = rospy.get_param('~rate',0.1)
		self.pub1 = rospy.Publisher('line_dot', OccupancyGrid, queue_size=1000)
		self.pub = rospy.Publisher('path',Path, queue_size=100)
		rospy.Subscriber('cspace',OccupancyGrid,self.map_cb)
		rospy.sleep(3)
	
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
			zig3.global_variable_declaration(self.height,self.width,self.resolution,self.map_array)

	def find_right_down_point(self):
		i = 0
		for i in range( self.height*self.width):
			if(self.map_array[i]==0):
				break
		return i



	def draw_(self):

		self.r_c_point= self.find_right_down_point()
		
		y,x = divmod(self.r_c_point,self.height)
		print(x,y)
		for i in range(10):	
			
			self.draw_lines(y*self.height + 10*(self.height * int(self.radius/self.resolution))*i)
			#self.line_points.append(zig3.draw_line(self.r_c_point + self.height *  2*int(self.radius/self.resolution)*i))


		self.bit_line = self.line_only
		#print("line drawn")

	

	def draw_lines(self,starting_array): 
		map_state =0
		line_= 0
		line_per_y = []
		yy,xx = divmod(starting_array,self.height)
		for i in range(self.height):
			if((self.map_array[yy*self.height + i] == 0) and (map_state == 0)):
				map_state =1
				piy,pix = divmod(yy*self.height + i,self.height)
				self.line_only.append([pix,piy])
				line_per_y.append([pix,piy])

			elif((self.map_array[yy*self.height + i] != 0) and (map_state == 1)):
				line_ = line_+1
				piy,pix = divmod(yy*self.height+i-1,self.height)
				self.line_only.append([pix,piy])
				self.line_only.append([self.line_num])
				line_per_y.append([pix,piy])
				line_per_y.append([self.line_num])
				self.line_num  = self.line_num  + 1
				map_state = 0
		if(line_per_y != []):
			self.line_per_y.append(line_per_y)
		

		'''
    
		map_state = 0
		y,x = zig3.array_to_pixel(starting_array)
		py = y
		line =[]
		line_only1 = []

		for i in range(self.height):
			if((self.map_array[y*self.height+i]==0) and (map_state == 0)):
				map_state = 1
				#print(y*self.height+i,self.height)
				
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
		'''
		
		#print(self.line_only)



	



   	def mapp_line_point(self):
		#print("WOW!")
	
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
		
			self.trajectorymap.data[500 + i]=100
			self.trajectorymap.data[500 - i]=100
			self.trajectorymap.data[500 + self.height*i]=100
			self.trajectorymap.data[500 - self.height*i]=100
		
		for i in range(len(self.line_only)/3):	
			num = self.line_only[3*i][0] + self.height * self.line_only[3*i][1]
			self.trajectorymap.data[num] = 100
			num = self.line_only[3*i+1][0] + self.height * self.line_only[3*i+1][1]
			self.trajectorymap.data[num] = 100
		#print(len(self.trajectorymap.data))
		

	def pixel_to_meter(self):

		num = 0
		self.line_coor.append([0,0])
		self.line_coor.append([(self.line_per_y[0][0][0]-200)*self.resolution,(self.line_per_y[0][0][1]-200)*self.resolution])
		num =1 
		for i in range(len(self.line_per_y)):
			if(num %2 == 1):
				for j in range(len(self.line_per_y[i])/3):
					s_x = (self.line_per_y[i][3*j][0]-200)*self.resolution
					s_y = (self.line_per_y[i][3*j][1]-200)*self.resolution
					f_x = (self.line_per_y[i][3*j+1][0]-200)*self.resolution
					f_y = (self.line_per_y[i][3*j+1][1]-200)*self.resolution
					self.line_coor.append([s_x,s_y])
					self.line_coor.append([f_x,f_y])
			else:
				for j in range(len(self.line_per_y[i])/3):
					a = len(self.line_per_y[i])
					s_x = (self.line_per_y[i][a-(3*j+2)][0]-200)*self.resolution
					s_y = (self.line_per_y[i][a-(3*j+2)][1]-200)*self.resolution
					f_x = (self.line_per_y[i][a-(3*j+3)][0]-200)*self.resolution
					f_y = (self.line_per_y[i][a-(3*j+3)][1]-200)*self.resolution
					self.line_coor.append([s_x,s_y])
					self.line_coor.append([f_x,f_y])
			
			num = num + 1


	def determine_next_orientation(self,a):
		
		dx = self.line_coor[a+1][0]-self.line_coor[a][0]
		dy = self.line_coor[a+1][1]-self.line_coor[a][1]
		if(dx != 0):
			self.theta = math.atan2(dy,dx)
		elif(dy>0):
			self.theta =0
		elif(dy<0):
			self.theta = math.pi

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
				print(w[0],w[1],self.theta,i)


	        	       	pose3 = PoseStamped()
				pose3.header.seq = i
				pose3.header.frame_id = "path_frame"
        		       	pose3.pose.position.x = w[0]
        		       	pose3.pose.position.y = w[1]
				pose3.pose.position.z = 0
        	        	#quaternion = tf.transformations.quaternion_from_euler(0, 0, 1)



				if(line_num<(len(self.line_coor)-1)):
					#print("len(self.lin_coor)",len(self.line_coor),line_num,i)
					self.determine_next_orientation(line_num)
				quaternion = quaternion_from_euler(0,0,self.theta)
			        pose3.pose.orientation.x = quaternion[0]
			        pose3.pose.orientation.y = quaternion[1]
			        pose3.pose.orientation.z = quaternion[2]
			        pose3.pose.orientation.w = quaternion[3]
				self.msg.poses.append(pose3)
				print(w[0],w[1],self.theta)



				i = i+1
					


				line_num = line_num + 1

		self.pub.publish(self.msg)
		#print(self.msg)
		
	




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
				self.drawing_line_flag =1
				#for w in self.line_per_y:
				#	print(w)
				#print(self.line_only)
		
				self.making_line_map()
			
				self.publish_waypoints()
			self.mapp_line_point()



	




if __name__ == '__main__':
   	zig = ZigZag_Ver5()
	zig.spin()
		
