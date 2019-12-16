#!/usr/bin/env python

import rospy
# Brings in the SimpleActionClient
import actionlib
# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import geometry_msgs.msg
import move_base_msgs.msg
import tf2_ros
from actionlib_msgs.msg import GoalStatus

# to clear the map rosservice call /move_base/clear_costmaps "{}"
# to open rviz roslaunch turtlebot_rviz_launchers view_navigation.launch

def runToGoalClient():
	# Creates the SimpleActionClient, passing the type of the action
	client = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)
	# Waits until the action server has started up and started
	# listening for goals.
	client.wait_for_server()

	# Creates a goal to send to the action server.
	goal = move_base_msgs.msg.MoveBaseGoal()
	goal.target_pose.header.frame_id = "map"

	poses = [[-2.0, 4.0], [-1.5, 0.0], [-5.0, -1.0], [7.0, 0.0]]
	counter = 0
	currentState = 0
	rate = rospy.Rate(50)

	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)

	while not rospy.is_shutdown():
        if(currentState == 0):
			goal.target_pose.header.stamp = rospy.get_rostime()
			goal.target_pose.pose.position.x = poses[counter][0]
			goal.target_pose.pose.position.y = poses[counter][1]
			goal.target_pose.pose.orientation.w = 1
			# Sends the goal to the action server.
			client.send_goal(goal)
			currentState = 1
		# Waits for the server to finish performing the action.
		elif(currentState == 1):

			if (client.get_state() == GoalStatus.SUCCEEDED or client.get_state() == GoalStatus.LOST or client.get_state() == GoalStatus.REJECTED):
				currentState = 0
				counter = (counter + 1) % 4

		rate.sleep()


if __name__ == '__main__':
	poseIsNew = False

	rospy.init_node('run_to_goal_client_py')
	runToGoalClient()
