#!/usr/bin/env python
import rospy
import sys
import math
from ar_coord.msg import ZumyCoord
from geometry_msgs.msg import Twist,Transform,TransformStamped
from tf2_msgs.msg import TFMessage
from move_zumy.srv import Mov2LocSrv, Mov2LocSrvResponse
import get_vel
import config


class MoveZumy:
	def __init__(self, zumy_name):
		rospy.init_node('move_zumy'+zumy_name)
		self.name = zumy_name
		self.position = ZumyCoord().position
		self.ARTag = config.zumy_ar_pair[self.name]
		rospy.Subscriber('/'+self.ARTag+'/AR_position', ZumyCoord, self.getPos)
		self.rate = rospy.Rate(10)
		self.goal = self.position
		self.goal_flag = True
		self.vel_pub = rospy.Publisher('/%s/cmd_vel' % self.name, Twist, queue_size=2)
		#self.pos_tf_pub = rospy.Publisher('/tf', TFMessage,queue_size=2)
		rospy.Service('/'+self.name+'/zumy_tracking', Mov2LocSrv, self.move)
		print self.name+' is alive'

	def run(self):
		print self.name+ " is running"
		print self.goal
		rospy.spin()
	def getPos(self, msg):
		self.position = msg.position
		print self.position

	# def pubTFPos(self):
	# 	counter = 0
	# 	state = Transform()
	# 	state.translation.x = self.position.x
	# 	state.translation.y = self.position.y
	# 	state.rotation.z = np.sin(self.position.theta/2.)
	# 	state.rotation.w = np.cos(self.position.theta/2.) # Quaternion form
	# 	self.state_pub.publish(state)

	# 	state_tf = TFMessage()
	# 	state_tf.transforms = [TransformStamped()]
	# 	state_tf.transforms[0].header.seq = counter
	# 	state_tf.transforms[0].header.frame_id = "ar_marker_0"
	# 	#print self.origin_tag
	# 	state_tf.transforms[0].child_frame_id = self.name
	# 	state_tf.transforms[0].transform = state
	# 	self.pos_tf_pub.publish(state_tf)



	def move(self, request):
			#Set rospy rate
		#Expose variables from subscriber line
		self.goal = request.goal
		self.goal_flag = False

		#Creating a new current state based on the information from Haoyu's code		
		#Plugging the information from Haoyu's code into Vijay's getCmdVel function to calculate v_x and omega_z
		while not self.goal_flag:
			#self.pubTFPos()

			(vel, self.goal_flag) = get_vel.getCmdVel(self.position, self.goal)

		#Creating the ability to publish to the zumy

		#Creating the message type to publish to the zumy (information from vel)
			cmd = Twist()
			
			cmd.linear.y = 0
			cmd.linear.z = 0
			cmd.angular.x = 0
			cmd.angular.y = 0
			if not self.goal_flag:
				cmd.angular.z = vel['ang_z']
				cmd.linear.x = vel['lin_x']
			else:
				cmd.angular.z = 0
				cmd.linear.x = 0

			#Publish new velocity information to the zumy
			self.vel_pub.publish(cmd)
			print cmd.linear.x
			print cmd.angular.z
			self.rate.sleep()

		return True
	
if __name__=='__main__':

	#Checks the number of arguments
	if len(sys.argv) < 2:
		print('Wrong Number of Arguments!  Use: move_to_location.py [ zumy name ]')
		sys.exit()
	little_zumy = MoveZumy(sys.argv[1])
	little_zumy.run()


	
	#rospy.Service('zumy_location', Mov2LocSrv, move)
	# print "Ready to receive cmds to move Zumys"
	# goal_state = {'x': 0.5, 'y': 0.5, 'theta': 0}
	# zumy = sys.argv[1]
	# 
	# s = rospy.Subscriber('zumy_position', ZumyCoord, move, [goal_state, zumy])
	# print s._connection_header
	# rate.sleep()
	#rospy.spin()


	# for zumy_name in sys.argv[1:]:
	# 	#print zumy_name
	# 	zumynode[zumy_name] =  MoveZumy(zumy_name)
	# 	zumynode[zumy_name].run()

	#Initiates our node
	
	
	# #Defines parameters that ultimately get fed to our move function
	# 
	# 

	# #Subscribes to the zumy_position topic created by Haoyu's code
	# rospy.Subscriber('zumy_position', ZumyCoord, move, [goal_state, zumy])
	# rospy.spin()