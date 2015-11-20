#!/usr/bin/env python
import rospy
import sys
#import math
#from ar_coord.msg import ZumyCoord
from geometry_msgs.msg import Twist, Pose2D
from move_zumy.srv import Mov2LocSrv, Mov2LocSrvResponse
#import get_vel
#import config

def send_loc_req_stat(zumy_name, goal_position):
	service_name = '/'+zumy_name+'/zumy_tracking'
	rospy.wait_for_service(service_name)
	try:
		send_goal_pos = rospy.ServiceProxy(service_name, Mov2LocSrv)
		goal_reach_flag = send_goal_pos(zumy_name, goal_position)
		return goal_reach_flag.isPosReached
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e


if __name__== '__main__':
	is_goal_reached = True
	if len(sys.argv) < 2:
		print('Wrong Number of Arguments!  Use: move_to_location.py [ zumy name ]')
		sys.exit()
	zumy_ID = sys.argv[1]
	while is_goal_reached:
		goal_pos = Pose2D()
		goal_pos.x = input("Please input "+zumy_ID+"\'s x coord:")
		goal_pos.y = input("Please input "+zumy_ID+"\'s y coord:")
		goal_pos.theta = input("Please input "+zumy_ID+"\'s theta coord in deg:")
		is_goal_reached = send_loc_req_stat(zumy_ID, goal_pos)

