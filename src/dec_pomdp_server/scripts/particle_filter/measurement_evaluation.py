#!/usr/bin/env python
import rospy
import numpy as np
from dec_pomdp_msgs.msg import Measurements
from particle_filter import *
from wlan_localization import WLANLocalization
from particle_filter_visualization import ParticleFilterVisualizer

class MeasurementSubscriber:
	def evaluate(self, measurements):
		self.measurements.append(measurements)

	def get_measurements(self):
		rospy.logwarn(self.measurements)
		return self.measurements

	def __init__(self, frame_id):
		self.measurements = []
		rospy.init_node('measurements_evaluation')
		rospy.Subscriber('measurements', Measurements, self.evaluate)



def run_measurement_evaluation(subscriber, problem, num_particles, visualize):
	visualizer = ParticleFilterVisualizer('map')
	rate = rospy.Rate(0.1)
	x = -8.0 + 32.0 * np.random.uniform(size=(num_particles,2))
	weights = np.repeat(1.0/float(num_particles), num_particles)
	while not rospy.is_shutdown():
		observations = subscriber.get_measurements()
		if (len(observations) > 0):
			latest_observations = observations[len(observations) - 1]
			rospy.logwarn(latest_observations)
			for measurement in latest_observations.measurements:
				position = [measurement.position.x, measurement.position.y]
				rospy.logwarn("position is: " + str(position) + " measurement is: " + str([measurement.signal_strength]))
				l = problem.likelihood(x, position, [measurement.signal_strength])
				rospy.logwarn("probablity is: " + str(l))
				x, weights = SIR_step(x, weights, l)
				rospy.logwarn("Particles are at: " +  str(x))
				rospy.logwarn("particle weights are: " + str(weights))
				visualizer.visualize(x, weights)
		rate.sleep()

if __name__ == '__main__':
	sub = MeasurementSubscriber('map')
	problem = WLANLocalization([],[])
	run_measurement_evaluation(sub, problem, 1000, True)
	rospy.spin()
