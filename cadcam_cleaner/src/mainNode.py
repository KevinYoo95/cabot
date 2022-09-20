#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
import roslaunch
import std_msgs.msg
import os
import rosnode
import subprocess
#rosnode.get_node_names()


process_generate_running = True

class ProcessListener(roslaunch.pmon.ProcessListener):
    global process_generate_running

    def process_died(self, name, exit_code):
        global process_generate_running
        process_generate_running = False
        rospy.logwarn("%s died with code %s", name, exit_code)


class RocketSwitch():
	def __init__(self):
		
		self.state = 0
		self.statePrev = 0
 		self.nodename = rospy.get_name()
		rospy.init_node('MainNodeWithRocketSwitch', anonymous=True)
		rospy.loginfo("-I- %s started" % self.nodename)
		rospy.Subscriber("dx_dtheta", Float32MultiArray, self.callback)
		self.rate = rospy.get_param('~rate',10)
		self.explore = None
		self.gmapping = None
		self.amcl = None
		self.saveMap = None
		self.publishMap = None
		self.wallFollowing = None
		self.zigzag = None
		self.onemonitor = None
		self.killzigzag = None
		self.killwallFollowing = None
		self.killgmapping = None

		self.killwallFollowingFlag = 0
		self.killzigzagFlag = 0

		self.gmappingFlag = False
		self.amclFlag = False
		self.zigzagFlag = False
		self.wallFollowingFlag = False

		"""
		self.exploreCommand = "roslaunch explore_lite explore.launch"
		self.gmappingCommand = "roslaunch turtlebot3_slam turtlebot3_slam.launch"
		self.amclCommand = "roslaunch turtlebot3_navigation amcl.launch"
		self.saveMapCommand = "rosrun map_server map_saver -f ~/map_generated.yaml"
		self.publishMapCommand = "rosrun map_server map_server ~/map_generated.yaml"
		self.wallFollowingCommand = "rosrun ydlidar_ros_driver wallFollowing0812"
		self.zigzagCommand = "rosrun ydlidar_ros_driver wallFollowing0812"
		"""

		self.exploreCommand = ["roslaunch","explore_lite","explore.launch"]
		self.gmappingCommand = ["roslaunch","turtlebot3_slam","turtlebot3_slam.launch"]
		self.amclCommand = ["roslaunch","turtlebot3_navigation","amcl.launch"]
		self.saveMapCommand = ["rosrun","map_server","map_saver","-f","/home/cadcam/map_generated"]
		self.publishMapCommand = ["rosrun","map_server","map_server","/home/cadcam/map_generated.yaml"]
		self.wallFollowingCommand = ["rosrun","cadcam_cleaner","wallFollowing0824"]
		self.zigzagCommand = ["rosrun","cadcam_cleaner","zigzag_0824"]

		self.killwallFollowingCommand = ["rosnode","kill","wallFollowing0824"]
		self.killzigzagCommand = ["rosnode","kill","zigzag_0824"]
		self.killgmappingCommand = ["rosnode","kill","turtlebot3_slam_gmapping"]

		self.onemonitorCommand = ["xrandr","--output", "DSI-1", "--off"]



	def callback(self,msg):
		
		self.state = msg.data[3]
		#rospy.loginfo("-I- state is  %d " % self.state)
	


	def launch_roslaunch(self):
		rospy.loginfo("entered launch ")
		if(self.state==1):	#mapping_start
			self.gmapping = subprocess.Popen(self.gmappingCommand)
			rospy.loginfo("launched gmapping ")
			rospy.sleep(3)
			self.gmappingFlag = True
			self.explore = subprocess.Popen(self.exploreCommand)
			rospy.loginfo("launched explore ")
			rospy.sleep(3)

		elif(self.state==2):	#map_saver
			self.saveMap = subprocess.Popen(self.saveMapCommand)
			rospy.loginfo("launched saveMap ")
			rospy.sleep(3)

		elif(self.state==3):	#wall_follow
			if(self.amclFlag == False):
				self.amcl = subprocess.Popen(self.amclCommand)
				rospy.loginfo("launched amcl ")
				rospy.sleep(3)
				self.amclFlag = True
				self.publishMap = subprocess.Popen(self.publishMapCommand)
				rospy.loginfo("launched publishMap ")
				rospy.sleep(7)
			self.wallFollowing = subprocess.Popen(self.wallFollowingCommand)
			self.killwallFollowingFlag = 0
			rospy.loginfo("launched wall_follow ")
			rospy.sleep(3)

		elif(self.state==4):#zigzag
			if(self.amclFlag == False):
				self.amcl = subprocess.Popen(self.amclCommand)
				rospy.loginfo("launched amcl ")
				rospy.sleep(3)
				self.amclFlag = True
				self.publishMap = subprocess.Popen(self.publishMapCommand)
				rospy.loginfo("launched publishMap ")
				rospy.sleep(3)
			self.zigzag = subprocess.Popen(self.zigzagCommand)
			self.killzigzagFlag = 0
			rospy.loginfo("launched zigzag ")
			rospy.sleep(3)
	
		if(self.state != 0):
			self.statePrev=self.state


	def delaunch_roslaunch(self):
		rospy.loginfo("entered delaunch!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

		rospy.loginfo("statePrev is    :%d",self.statePrev)
		rospy.loginfo("state is        :%d",self.state)

		if(self.state == 0):
			if(self.explore != None):
				self.explore.terminate()
				rospy.loginfo("terminated explore")
				rospy.sleep(3)	
			if(self.zigzag != None):
				if(self.killzigzagFlag == 0):
					self.killzigzag = subprocess.Popen(self.killzigzagCommand)
					rospy.loginfo("terminated zigzag")
					rospy.sleep(3)	
					self.killzigzagFlag = 1
			if(self.wallFollowing != None):
				if(self.killwallFollowingFlag == 0):
					self.killwallFollowing = subprocess.Popen(self.killwallFollowingCommand)
					rospy.loginfo("terminated wallFollowing")
					rospy.sleep(3)	
					self.killwallFollowingFlag = 1

		#rospy.loginfo("entered delaunch ")
		if(self.statePrev==1):	#mapping_start
			if (self.state !=2):
				self.explore.terminate()
				rospy.loginfo("terminated explore ")
				rospy.sleep(3)	
				self.gmapping.terminate()
				rospy.loginfo("terminated gmapping ")
				rospy.sleep(3)

		elif(self.statePrev==2):  #map save
			

			if(self.gmappingFlag == True):
				#self.killgmapping = subprocess.Popen(self.killgmappingCommand)
				self.gmapping.terminate()	
				rospy.loginfo("terminated gmapping")
				rospy.sleep(3)	
				self.killgmappingFlag = False



			#self.gmapping.terminate()
			#rospy.loginfo("terminated gmapping ")
			#rospy.sleep(3)

		elif(self.statePrev==3):#wall_follow
			if(self.state==1):
				self.amcl.terminate()	
				rospy.loginfo("terminated amcl ")
				rospy.sleep(3)
				self.amclFlag = False
			self.wallFollowing.terminate()	
			rospy.loginfo("terminated wallFollowing ")
			rospy.sleep(3)

		elif(self.statePrev==4):#zigzag


			
			if(self.state==1):
				self.amcl.terminate()	
				rospy.loginfo("terminated amcl ")
				rospy.sleep(3)
				self.amclFlag = False
			self.zigzag.terminate()	
			rospy.loginfo("terminated zigzag ")
			rospy.sleep(3)


	def init_launch(self,launchfile, process_listener):
		uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
		roslaunch.configure_logging(uuid)
		launch = roslaunch.parent.ROSLaunchParent(
			uuid,
			[launchfile],
			process_listeners=[process_listener],
	        )
		return launch


	def spin(self):
		r = rospy.Rate(self.rate)
		self.onemonitor = subprocess.Popen(self.onemonitorCommand)
		while not rospy.is_shutdown():
			self.update()
			r.sleep()

	def update(self):
		if(self.statePrev!=self.state):
			self.delaunch_roslaunch()
			rospy.loginfo("delaunch complete!!!!!!!!!!!!!!!!!!!!!!!!!!!")
			if(self.state != 0):
				self.launch_roslaunch()
		


if __name__ == '__main__':
   	rs = RocketSwitch()
	rs.spin()
