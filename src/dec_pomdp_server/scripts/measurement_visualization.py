#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3
from dec_pomdp_msgs.msg import Measurements
from std_msgs.msg import ColorRGBA
import numpy as np

class MeasurementVisualizer:
	marker_id = 0
	marker = Marker(
			type= Marker.SPHERE_LIST,
			ns="MeasurmentVisualization",
			id=marker_id,
			lifetime=rospy.Duration(0),
			scale=Vector3(0.1, 0.1, 0.06),
		)
	average_marker = Marker(
			type= Marker.CYLINDER,
			ns="MeasurmentVisualization",
			id=marker_id,
			lifetime=rospy.Duration(0),
			scale=Vector3(0.1, 0.1, 0.06),
		)
	def visualize(self, measurements):
		self.marker_id += 1
		self.marker.id = self.marker_id
		self.marker_id += 1
		self.average_marker.id = self.marker_id
		locations_x = []
		locations_y = []
		signal_strength_values = []
		for measurement in measurements.measurements:
			self.marker.header.stamp = rospy.get_rostime()
			self.marker.points.append(measurement.pose.position)
			locations_x.append(measurement.pose.position.x)
			locations_y.append(measurement.pose.position.y)
			signal_strength_values.append(measurement.signal_strength)
			self.marker.colors.append(self.get_color(measurement.signal_strength))
		self.average_marker.color = self.get_color(np.mean(signal_strength_values))
		self.average_marker.pose.position.x = np.mean(locations_x)
		self.average_marker.pose.position.y = np.mean(locations_y)
		self.average_marker.pose.orientation.w = 1.0
		rospy.logwarn("Measurment value at x = %f and y = %f is %f", np.mean(locations_x), np.mean(locations_y), np.mean(signal_strength_values))
		self.markerPublisher.publish(self.average_marker)

	def get_color(self, signal_strength):
		color = ColorRGBA(0, 0, 0, 1)
		normalized_signal_strength = (signal_strength + 80.0)/(-40 + 80.0)
		color.r = 1 - normalized_signal_strength
		color.g = normalized_signal_strength
		return color

	def __init__(self, frame_id):
		self.marker.header.frame_id = frame_id
		self.marker.color.r = 1.0
		self.marker.color.g = 1.0
		self.marker.color.b = 1.0
		self.marker.color.a = 1.0
		self.average_marker.header.frame_id = frame_id
		rospy.init_node('measurements_visualization')
		self.markerPublisher= rospy.Publisher('measurement_locations', Marker, queue_size=10)
		rospy.Subscriber('measurements', Measurements, self.visualize)


if __name__ == '__main__':
	x = MeasurementVisualizer('map')
	rospy.spin()
