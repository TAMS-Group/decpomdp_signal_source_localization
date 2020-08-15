#!/usr/bin/env python

#    measurement_visualization.py
#
#	 Copyright 2020 Tobias Krueger
#
#    Licensed under the Apache License, Version 2.0 (the "License");
#    you may not use this file except in compliance with the License.
#    You may obtain a copy of the License at
#
#        http://www.apache.org/licenses/LICENSE-2.0
#
#    Unless required by applicable law or agreed to in writing, software
#    distributed under the License is distributed on an "AS IS" BASIS,
#    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#    See the License for the specific language governing permissions and
#    limitations under the License.

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
		self.average_marker.color = self.get_color_discrete(np.mean(signal_strength_values))
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

	def get_color_discrete(self, signal_strength):
		color = ColorRGBA(0, 0, 0, 1)
		high_threshold = -55.0
  		low_threshold = -65.0
		if signal_strength > high_threshold:
			color.g = 1
		elif signal_strength < high_threshold and signal_strength > low_threshold:
			color.g = 0.5
			color.r = 0.5
		elif signal_strength < low_threshold:
			color.r = 1.0
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
