#!/usr/bin/env python
# license removed for brevityimport math
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal,MoveBaseActionResult
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import  Odometry

global height
global width
global resolution
global map_data
global map_state # undefined = -1 #free = 0 #occupied = 100 #
global line_data 
global line_num 

def global_variable_declaration(hei,wid,reso,Map_data):
	global height
	global width
	global resolution
	global map_data
	global line_num 

	line_num = 0
	height = hei
	resolution = reso
	map_data = Map_data
	width = wid
###########################################################################
def a(a,b):
	print(c(1))
	f = c(3)
	return f
def c(a):
	e = a
	return e

def finding_up_point(x,y):
	flag = True
	pix,piy = meter_to_pixel(x,y)
	p_position = piy*height + pix
	for i in range(height):
		if(map_data[piy*384 + pix + i] != 0):
			a = piy*384 + pix + i
			flag = False
		if (flag == False):
			break
	mx,my = array_to_pixel(a)
	xx,yy = pixel_to_meter(mx,my)
	return xx,yy

def finding_down_point(x,y):
	flag = True
	pix,piy = meter_to_pixel(x,y)
	p_position = piy*height + pix
	for i in range(height):
		if(map_data[piy*384 + pix - i] != 0):
				flag = False
		if (flag == False):
			break
	mx,my = array_to_pixel(piy*384 + pix - i)
	xx,yy = pixel_to_meter(mx,my)
	return xx,yy

def finding_right_down_point(Map_dataa):
	j=0
	for m in range(width*height):
		if (Map_dataa[m] == 0):
			j =m
			break
	return j


def finding_line_ending_point( x,y ):
	flag = True
	for i in range (height ):
		if(map_data[y*height  + i] != 0 ):
			flag = False
		if (flag == False):
			break
	if( i == (height-1)):
		flag9 == 1
	map_state = 100
	piy,pix = divmod((y*height  + i),height)
	return pix,piy

def finding_line_starting_point( x,y ):
	flag = True
	for i in range (height ):
		if(map_data[y*height + i] == 0 ):
			flag = False
		if (flag == False):
			break
	
	map_state = 0
	if( i == (height-1)):
		flag9 == 1
	pit,pix = divmod(y*height  + i,height)
	return pix,piy

'''
    def finding_up_point(self,x,y):
	flag = True
	pix,piy = self.meter_to_pixel(x,y)
	p_position = piy*self.height + pix

	for i in range(self.height):
		if(self.map_array[piy*384 + pix + i] != 0):
			a = piy*384 + pix + i
			break

	mx,my = self.array_to_pixel(a)
	xx,yy = self.pixel_to_meter(mx,my)
	print(xx)
	print(yy)
	return xx,yy
'''


def draw_line(starting_array): 
	
	global line_data 
	global flag9
	global line_num 
	flag9 = 0
	line_data = []
	starting_array 
	
	map_state = 0
	
	x,y = array_to_pixel(starting_array)
	line_data = []
	py = y
	line =[]
	line_only = []
	for i in range(height):
		if((map_data[y*height+i]==0) and (map_state == 0)):
			map_state = 1
			piy,pix = divmod(y*height+i,height)
			line.append([pix,piy])
			line_only.append([pix,piy])

		elif((map_data[y*height+i]!=0 ) and ( map_state == 1)):
			map_state = 2
			piy,pix = divmod(y*height+i,height)
			line.append([pix,piy])
			line_only.append([pix,piy])
			line.append([line_num])
			line_only.append([line_num])
			line_num = line_num + 1
			#print("ending")
			#print(map_state)
			#ending point

		elif(map_state == 2):
			#print(line)
			line_data.append(line)
			line = []
			map_state = 0
	#print(line_data)
	return line_data


def draw_lines(starting_array): 
	
	global line_data 
	global flag9
	global line_num 
	flag9 = 0
	line_data = []
	starting_array 
	
	map_state = 0
	
	x,y = array_to_pixel(starting_array)
	line_data = []
	py = y
	line =[]
	line_only = []
	line_only1 = []
	flag = 0
	for i in range(height):
		if((map_data[y*height+i]==0) and (map_state == 0)):
			map_state = 1
			piy,pix = divmod(y*height+i,height)
			line_only.append([pix,piy])
			
			#print("starting")
			#print(map_state)
			#starting point

		elif((map_data[y*height+i]!=0 ) and ( map_state == 1)):
			map_state = 2
			piy,pix = divmod(y*height+i,height)
			line_only.append([pix,piy])
			line_only.append([line_num])
			line_only = []
			#print(line_only)
			line_only1.append([line_only])
			#print(line_only1)
			line_num = line_num + 1
			#print("ending")
			#print(map_state)
			#ending point

			line = []
			#line_only = []
			map_state = 0

	#print(line_only1)
	
	return line_only1

			
'''
def draw_line(starting_array): 
	
	print("will it work?")
	global line_data 
	global flag9
	flag9 = 0
	line_data = []
	starting_array 
	
	map_state = 0
	
	x,y = array_to_pixel(starting_array)
	
	py = y
	line_data.append([x,y])
	while(flag9 == 0):	
		
		if(map_state == 0):
			py = y
			pix,piy = finding_line_ending_point(x,y)
			x = pix
			y = piy
			line_data.append([x,y])
		if(flag9 != 0):
			break
		if(map_state == 100):
			py = y
			pix,piy = finding_line_starting_point(x,y)
			x = pix
			y = piy

			line_data.append([x,y])
		print(line_data)
	print(line_data)
	return line_data
			
		
		'''

###########################################################################33

###############################################################






############################################################################################

def meter_to_pixel(x,y):
	pix = int(x/resolution+200)
	piy = int(y/resolution+200)
	return pix, piy



def pixel_to_meter(pix,piy):
	x = (pix-200)*resolution
	y = (piy-200)*resolution
	return x, y

def array_to_pixel(data):
	piy,pix = divmod(data,height)
	return pix,piy

def meter_to_array(x,y):
	pix = int(x/resolution+200)
	piy = int(y/resolution+200)
	array_data = pix + piy*height
	return array_data

def array_to_meter(array_data):
	piy,pix = divmod(array_data,height)
	x,y = pixel_to_meter(pix,piy)
	return pix, piy

#############################################################################################

