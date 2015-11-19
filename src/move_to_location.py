#!/usr/bin/env python
import rospy
import sys
import math
from ar_coord.msg import ZumyCoord
from geometry_msgs.msg import Twist
import get_vel

def move(msg, arg):
	
	#Set rospy rate
	rate = rospy.Rate(10)	

	#Expose variables from subscriber line
	goal = arg[0]
	zumy = arg[1]

	#Creating a new current state based on the information from Haoyu's code
	current_state = {'x': msg.position.x, 'y': msg.position.y, 'theta': msg.position.theta}
	
	#Plugging the information from Haoyu's code into Vijay's getCmdVel function to calculate v_x and omega_z
	vel = get_vel.getCmdVel(current_state, goal)

	#Creating the ability to publish to the zumy
	zumy_vel = rospy.Publisher('/%s/cmd_vel' % zumy, Twist, queue_size=2)

	#Creating the message type to publish to the zumy (information from vel)
	cmd = Twist()
	cmd.linear.x = vel['lin_x']
	cmd.linear.y = 0
	cmd.linear.z = 0
	cmd.angular.x = 0
	cmd.angular.y = 0
	cmd.angular.z = vel['ang_z']

	#Publish new velocity information to the zumy
	zumy_vel.publish(cmd)
	print cmd.linear.x
	print cmd.angular.z
	rate.sleep()

if __name__=='__main__':

	#Checks the number of arguments
	if len(sys.argv) < 2:
		print('Wrong Number of Arguments!  Use: move_to_location.py [ zumy name ]')
		sys.exit()

	#Initiates our node
	rospy.init_node('MoveZumy')
	
	#Defines parameters that ultimately get fed to our move function
	zumy = sys.argv[1]
	goal_state = {'x': 1, 'y': 1, 'theta': 0}

	#Subscribes to the zumy_position topic created by Haoyu's code
	rospy.Subscriber('zumy_position', ZumyCoord, move, [goal_state, zumy])
	rospy.spin()

	# #Set runtime
	# time = 50

	# #Publish useful twists to the Zumy
	# while time > 0:
	# 	#Subscribes to the zumy_position topic created by Haoyu's code
	# 	rospy.Subscriber('zumy_position', ZumyCoord, move, [goal_state, zumy])

	# 	#Calls all callbacks waiting to be called
	# 	# rospy.spin()

	# 	time = time - 1
	
	# #Creating the ability to publish to the zumy
	# zumy_vel = rospy.Publisher('/%s/cmd_vel' % zumy, Twist, queue_size=2)

	# #Creates and publishes a stopping twist
	# cmd = Twist()
	# cmd.linear.x = 0
	# cmd.linear.y = 0
	# cmd.linear.z = 0
	# cmd.angular.x = 0
	# cmd.angular.y = 0
	# cmd.angular.z = 0
	# zumy_vel.publish(cmd)
	# print 'Finished!'