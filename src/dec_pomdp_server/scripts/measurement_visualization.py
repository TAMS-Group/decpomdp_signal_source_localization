#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3
from dec_pomdp_msgs.msg import Measurements
from std_msgs.msg import ColorRGBA

class MeasurementVisualizer:
	marker = Marker(
			type= Marker.SPHERE_LIST,
			ns="MeasurmentVisualization",
			id=1,
			lifetime=rospy.Duration(0),
			scale=Vector3(0.1, 0.1, 0.06),
		)

	def visualize(self, measurements):
		for measurement in measurements.measurements:
			self.marker.header.stamp = rospy.get_rostime()
			self.marker.points.append(measurement.position)
			self.marker.colors.append(self.get_color(measurement.signal_strength))

		self.markerPublisher.publish(self.marker)

	def get_color(self, signal_strength):
		color = ColorRGBA(0, 0, 0, 1)
		normalized_signal_strength = (signal_strength + 100.0)/(-30 + 100.0)
		color.r = 1 - normalized_signal_strength
		color.g = normalized_signal_strength
		return color

	def __init__(self, frame_id):
		self.marker.header.frame_id = frame_id
		self.marker.color.r = 1.0
		self.marker.color.g = 1.0
		self.marker.color.b = 1.0
		self.marker.color.a = 1.0
		rospy.init_node('measurements_visualization')
		self.markerPublisher= rospy.Publisher('measurement_locations', Marker, queue_size=10)
		rospy.Subscriber('measurements', Measurements, self.visualize)


if __name__ == '__main__':
	x = MeasurementVisualizer('map')
	rospy.spin()
