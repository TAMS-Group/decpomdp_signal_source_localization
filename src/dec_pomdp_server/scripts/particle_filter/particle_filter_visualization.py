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
			lifetime=rospy.Duration(0),
			scale=Vector3(0.1, 0.1, 0.06),
		)

	def visualize(self, particles, weights):
		self.marker.points = []
		self.marker.colors = []
		for i, particle in enumerate(particles):
			self.marker.header.stamp = rospy.get_rostime()
			position = Point(particle[0], particle[1], 0.0)
			self.marker.points.append(position)
			self.marker.colors.append(self.get_color(weights[i], weights))

		self.markerPublisher.publish(self.marker)

	def get_color(self, weight, weights):
		color = ColorRGBA(0, 0, 0, 1)
		if max(weights) != min(weights):
			normalized_weight = round((weight - min(weights)) / (max(weights) - min(weights)), 5)
			color.r = 1 - normalized_weight
			color.g = normalized_weight
		else: 
			color.g = 1.0
		#rospy.logwarn("normalized_weight is:{} - {} / {} - {} = {}".format(weight, min(weights), max(weights), min(weights), normalized_weight))
		return color

	def __init__(self, frame_id):
		self.marker.header.frame_id = frame_id
		self.marker.color.r = 1.0
		self.marker.color.g = 1.0
		self.marker.color.b = 1.0
		self.marker.color.a = 1.0
		self.markerPublisher= rospy.Publisher('particles', Marker, queue_size=10)
