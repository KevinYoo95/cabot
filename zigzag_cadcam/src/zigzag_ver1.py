#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped



def goal_statu_cb(data):
    
    a = data.status_list
    print(a)
    #rospy.loginfo(data.status_list[0].status)
def goal_cb(data):
    b = data
    print(data)
    #rospy.loginfo(a)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('zigzag_ver1', anonymous=True)

    rospy.Subscriber("move_base/status", GoalStatusArray, goal_statu_cb)
    rospy.Subscriber("move_base_simple/goal", PoseStamped, goal_cb)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
