#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3
from dec_pomdp_msgs.msg import Measurements

class MeasurementVisualizer:
	marker = Marker(
			type= Marker.TEXT_VIEW_FACING,
			id=1,
			lifetime=rospy.Duration(10),
			scale=Vector3(0, 0, 0.06),
		)

	def visualize(self, measurements):
		self.marker.header.stamp = rospy.get_rostime()
		self.marker.id = 1
		for measurement in measurements.measurements:
			self.marker.pose.position = measurement.position
			self.marker.text = 'Signal strenght: ' + str(measurement.signal_strength)
			self.markerPublisher.publish(self.marker)
			self.marker.id += 1

	def __init__(self, frame_id):
		self.marker.header.frame_id = frame_id
		self.marker.color.r = 1.0
		self.marker.color.g = 1.0
		self.marker.color.b = 1.0
		self.marker.color.a = 1.0
		rospy.init_node('measurements_visualization')
		self.markerPublisher= rospy.Publisher('measurement_locations', Marker, queue_size=1)
		rospy.Subscriber('measurements', Measurements, self.visualize)


if __name__ == '__main__':
	x = MeasurementVisualizer('map')
	rospy.spin()
