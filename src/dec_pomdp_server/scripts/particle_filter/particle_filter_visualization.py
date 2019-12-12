#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3
from dec_pomdp_msgs.msg import Measurements
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point

class ParticleFilterVisualizer:
	marker = Marker(
			type= Marker.POINTS,
			id=1,
			lifetime=rospy.Duration(10),
			scale=Vector3(0.1, 0.1, 0.06),
		)

	def visualize(self, particles, weights):
		self.marker.id = 1
		for i, particle in enumerate(particles):
			self.marker.header.stamp = rospy.get_rostime()
			position = Point(particle[0], particle[1], 0.0)
			self.marker.points.append(position)
			self.marker.colors.append(self.get_color(weights[i]))

		self.markerPublisher.publish(self.marker)

	def get_color(self, weight):
		color = ColorRGBA(0, 0, 0, 1)
		color.r = 1 - weight
		color.g = weight
		return color

	def __init__(self, frame_id):
		self.marker.header.frame_id = frame_id
		self.marker.color.r = 1.0
		self.marker.color.g = 1.0
		self.marker.color.b = 1.0
		self.marker.color.a = 1.0
		self.markerPublisher= rospy.Publisher('particles', Marker, queue_size=10)
