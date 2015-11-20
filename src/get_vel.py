import math
import config
import matplotlib.pyplot as plt
import time
from geometry_msgs.msg import Pose2D

def e_dist(del_x, del_y):
	return math.sqrt(del_x*del_x + del_y*del_y)

# def plotter(rbt_state, rbt_goal_state):
# 	plt.cla()

# 	plt.axis([0,2,0,2])

# 	endx = rbt_state['x'] + 0.25 * math.cos(rbt_state['theta'])
# 	endy = rbt_state['y'] + 0.25 * math.sin(rbt_state['theta'])
# 	heading_x_pts = [rbt_state['x'], endx]
# 	heading_y_pts = [rbt_state['y'], endy]

# 	tend_x = rbt_goal_state['x'] + 0.25 * math.cos(rbt_goal_state['theta'])
# 	tend_y = rbt_goal_state['y'] + 0.25 * math.sin(rbt_goal_state['theta'])
# 	theading_x_pts = [rbt_goal_state['x'], tend_x]
# 	theading_y_pts = [rbt_goal_state['y'], tend_y]

# 	rheading = plt.plot(heading_x_pts, heading_y_pts)
# 	theading = plt.plot(theading_x_pts, theading_y_pts)



# 	robot=plt.Circle((rbt_state['x'],rbt_state['y']),.05,color='r')
# 	target=plt.Circle((rbt_goal_state['x'],rbt_goal_state['y']),.05,color='g')

# 	fig = plt.gcf()
# 	fig.gca().add_artist(robot)
# 	fig.gca().add_artist(target)

# 	plt.setp(rheading, color ='c')
# 	plt.setp(theading, color='m')

# 	plt.draw()
# 	return


def getCmdVel(state, goal):
	del_x_world = goal.x - state.x
	del_y_world	= goal.y - state.y
	is_goal_reached = False

	print('distance_from_goal = %f' % e_dist(del_x_world, del_y_world))

	# Are we close enough to the goal that we can just fix orientation?
	if e_dist(del_x_world, del_y_world) < config.distThresh:
		desired_heading = goal.theta
		nearGoalPt = 1
	else:
		desired_heading = math.atan2(del_y_world, del_x_world)
		desired_heading = math.degrees(desired_heading)
		nearGoalPt = 0

	# Compute difference in desired and current heading.  Cast to [-PI,PI].
	del_heading = desired_heading - state.theta
	print('del_heading = %f' % del_heading)
	if del_heading > 180:
		del_heading = del_heading - 360
	elif del_heading < -180:
		del_heading = del_heading + 360

	# Motor commands issued here.  Block 1 = orientation fix/initial turning.  Block 2 = moving to goal.
	if abs(del_heading) > config.turnInPlaceThresh or nearGoalPt == 1:
		# Just turn since the delta theta is too high or we are near the goal and fixing orientation

		# Cases near the goal point.  Either turn slowly or stop.

		if nearGoalPt ==1 :
			if abs(del_heading) < config.finalHeadingThresh:
				lin_x = 0
				ang_z = 0
				print 'CASE 1'
				is_goal_reached = True
			elif del_heading > 25:
				lin_x = 0
				ang_z = config.maxTurnSpd
				print 'CASE 2'
			elif del_heading < -25:
				lin_x = 0;
				ang_z = -config.maxTurnSpd
				print 'CASE 3'
			elif del_heading > 0:
				lin_x = 0;
				ang_z = config.finalTurnSpd
				print 'CASE 4'
			elif del_heading < 0:
				lin_x = 0;
				ang_z = -config.finalTurnSpd
				print 'CASE 5'
		else:
			if del_heading > 0:
				lin_x = 0
				ang_z = config.maxTurnSpd
				print 'CASE 6'
			else:
				lin_x = 0
				ang_z = -config.maxTurnSpd
				print 'CASE 7'
	else:
		# Drive and turn at the same time.  Linear velocity scales with the amount of the distance along the robot's x-axis.
		# Angular velocity scales with the amount of the distance along the robot's y-axis (that we can't cover without turning).
		theta_rad = math.radians(state.theta)
		del_x_robot = del_x_world * math.cos(theta_rad) + del_y_world * math.sin(theta_rad)
		del_y_robot = -del_x_world * math.sin(theta_rad) + del_y_world * math.cos(theta_rad)
		lin_x = config.maxFwdSpd * del_x_robot/e_dist(del_x_robot, del_y_robot)
		ang_z = config.maxTurnSpd * del_y_robot/e_dist(del_x_robot, del_y_robot)
	
	# Final check if motor commands are within bounds.
	if abs(lin_x) > config.maxFwdSpd:
		if lin_x > 0:
			lin_x = config.maxFwdSpd
		else:
			lin_x = -config.maxFwdSpd

	if abs(ang_z) > config.maxTurnSpd:
		if ang_z > 0:
			ang_z = config.maxTurnSpd
		else:
			ang_z = -config.maxTurnSpd

	return ({'lin_x': lin_x, 'ang_z': ang_z}, is_goal_reached)
	
# def fake_robot_dynamics(state, cmd_vel):
# 	# this would just be a publish command for the actual Zumy
# 	new_x = state['x'] + config.sim_tim_scale * cmd_vel['lin_x'] * math.cos(state['theta'])
# 	new_y = state['y'] + config.sim_tim_scale * cmd_vel['lin_x'] * math.sin(state['theta'])
# 	new_theta = state['theta'] + config.sim_tim_scale*cmd_vel['ang_z']

# 	while new_theta > config.PI:
# 		new_theta = new_theta - 2.0*config.PI
# 	while new_theta < -config.PI:
# 		new_theta = new_theta + 2.0*config.PI

# 	return {'x':new_x, 'y':new_y, 'theta':new_theta }

if __name__=='__main__':
	rbt_state = {'x': 0.0, 'y': 0.0, 'theta': 0}
	print "x ", rbt_state['x']
	print "y ", rbt_state['y']
	print "theta ", rbt_state['theta']	
	cmd_vel = getCmdVel(rbt_state, rbt_goal_state)

	plt.ion()
	plt.show()

	while cmd_vel['lin_x'] != 0.0 or cmd_vel['ang_z'] != 0.0:
		#print "lin_x ", cmd_vel['lin_x']
		#print "ang_z ", cmd_vel['ang_z']
		rbt_state = fake_robot_dynamics(rbt_state, cmd_vel)
		cmd_vel = getCmdVel(rbt_state, rbt_goal_state)
		plotter(rbt_state, rbt_goal_state)
		time.sleep(0.01)

	print "x ", rbt_state['x']
	print "y ", rbt_state['y']
	print "theta ", rbt_state['theta']	