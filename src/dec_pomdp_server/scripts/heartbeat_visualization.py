#!/usr/bin/env python
import rospy
from dec_pomdp_msgs.msg import ExecutionState
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3

class Visualizer:
	marker = Marker(
			type= Marker.TEXT_VIEW_FACING,
			ns="heartbeat",
			id=0,
			lifetime=rospy.Duration(5),
			scale=Vector3(0, 0, 0.1),
		)
	robot_position = Marker(
		type= Marker.ARROW,
		ns="heartbeat_position",
		id=0,
		lifetime=rospy.Duration(5),
		scale=Vector3(0.2,0.2, 0.2)
	)
	robots = []
	# visualizes a single robot state in RVIZ
	def visualize(self, state):
		if(not(state.robot_name in self.robots)):
			self.robots.append(state.robot_name)
		self.marker.id = self.robots.index(state.robot_name)
		self.robot_position.id = self.robots.index(state.robot_name)
		self.marker.header.stamp = rospy.get_rostime()
		self.robot_position.header.stamp = rospy.get_rostime()
		self.marker.pose = state.pose.pose
		self.robot_position.pose = state.pose.pose
		self.marker.text = 'Name: ' + state.robot_name + ' State: ' + state.status
		self.robot_position.color.r = (1.0 * self.robots.index(state.robot_name)) / len(self.robots)
		self.robot_position.color.g = (1.0 * self.robots.index(state.robot_name)) / len(self.robots)
		self.markerPublisher.publish(self.marker)
		self.positionPublisher.publish(self.robot_position)

	def __init__(self, frame_id):
		self.marker.header.frame_id = frame_id
		self.robot_position.header.frame_id = frame_id
		self.marker.color.r = 0.0
		self.marker.color.g = 0.0
		self.marker.color.b = 0.0
		self.marker.color.a = 1.0
		self.robot_position.color.a = 1.0
		rospy.init_node('heartbeat_visualization')
		self.markerPublisher= rospy.Publisher('robotState', Marker, queue_size=4)
		self.positionPublisher= rospy.Publisher('robotPositon', Marker, queue_size=4)
		rospy.Subscriber('heartbeat', ExecutionState, self.visualize)


if __name__ == '__main__':
	x = Visualizer('map')
	rospy.spin()
