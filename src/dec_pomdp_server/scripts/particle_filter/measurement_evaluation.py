#!/usr/bin/env python
import rospy
import numpy as np
from dec_pomdp_msgs.msg import Measurements
from particle_filter import *
from wlan_localization import WLANLocalization
from particle_filter_visualization import ParticleFilterVisualizer
import logging


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
	x = -8.0 + 24.0 * np.random.uniform(size=(num_particles,2))
	weights = np.repeat(1.0/float(num_particles), num_particles)
	while not rospy.is_shutdown():
		logging.info("Next Iteration of Particle filter")
		observations = subscriber.get_measurements()
		if (len(observations) > 0):
			latest_observations = observations[len(observations) - 1].measurements
			logging.info(latest_observations)
			#Later it is supposed to work like this:
			#for measurement in latest_observations.measurements:
			if(len(latest_observations) > 0):
				measurement = latest_observations[len(latest_observations) -1]
				position = [measurement.position.x, measurement.position.y]
				logging.info("position is: " + str(position) + " measurement is: " + str([measurement.signal_strength]))
				l = problem.likelihood(x, position, [measurement.signal_strength])
				logging.info("probablity is: " + str(sum(l)))
				x, weights = SIR_step(x, weights, l)
				logging.info("Particles are at: " +  str(x))
				logging.info("particle weight 1 is: " + str(sum(weights)))
				visualizer.visualize(x, weights)
		rate.sleep()

if __name__ == '__main__':
	sub = MeasurementSubscriber('map')
	problem = WLANLocalization([],[])
	logging.basicConfig(filename='filter.log', filemode='w', format='%(name)s - %(levelname)s - %(message)s')
	run_measurement_evaluation(sub, problem, 2000, True)
