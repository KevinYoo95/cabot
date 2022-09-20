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


		rospy.init_node("Would_it_work")
		self.nodename = rospy.get_name()
		rospy.loginfo("-I- %s started" % self.nodename)
		self.rate = rospy.get_param('~rate',1)
		self.pub1 = rospy.Publisher('line_dot', OccupancyGrid, queue_size=1000)
		self.pub = rospy.Publisher('move_base_simple/goal',PoseStamped, queue_size=100)
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

		#self.r_c_point= self.find_right_down_point()
		pixxx = self.base_x*self.resolution+200
		piyyy = self.base_y*self.resolution + 200
		self.r_c_point= int(pixxx + piyyy*self.height)
		y,x = divmod(self.r_c_point,self.height)
		for i in range(1):	
			
			self.draw_lines(y*self.height +x +10*(self.height * int(self.radius/self.resolution))*i)
			#self.line_points.append(zig3.draw_line(self.r_c_point + self.height *  2*int(self.radius/self.resolution)*i))


		self.bit_line = self.line_only
		#print("line drawn")

	

	def draw_lines(self,starting_array): 
		map_state =0
		line_= 0
		line_per_y = []
		yy,xx = divmod(starting_array,self.height)


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

						
				


			
			a =0
			
		
		if(line_per_y != []):
			self.line_per_y.append(line_per_y)
		
	



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
				self.making_line_map()
				#print(self.line_coor)
			self.mapp_line_point()
		
			if(self.flag == False):	
				self.planning()
				self.zig()
		
				self.flag = True



	




if __name__ == '__main__':
   	zig = ZigZag_Ver5()
	zig.spin()
		
