#!/usr/bin/env python
import rospy
import numpy as np
from dec_pomdp_msgs.msg import Measurements
from particle_filter import *
from wlan_localization import WLANLocalization

class MeasurementSubscriber:
	def evaluate(self, measurements):
		self.measurements.append(measurements)

	def get_measurements():
		rospy.logwarn(self.measurements)
		return self.measurements

	def __init__(self, frame_id):
		self.measurements = []
		rospy.init_node('measurements_evaluation')
		rospy.Subscriber('measurements', Measurements, self.evaluate)

def run_measurement_evaluation(subscriber, problem, num_particles, visualize):
    rate = rospy.Rate(0.1)
    x = -20.0 + 40.0 * np.random.uniform(size=(num_particles,2))
    weights = np.repeat(1.0/float(num_particles), num_particles)
    while not rospy.is_shutdown():
		observations = subscriber.get_measurements()
		latest_observations = observations[len(observations) - 1]
		for measurement in latest_observations.measurements:
			l = problem.likelihood(x, measurement)
			x, weights = SIR_step(x, weights, l)
		rate.sleep()


if __name__ == '__main__':
	sub = MeasurementSubscriber('map')
	problem = WLANLocalization()
	run_measurement_evaluation(sub, 1000, True)
	rospy.spin()
