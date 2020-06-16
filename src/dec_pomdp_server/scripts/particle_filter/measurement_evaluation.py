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
		self.callback(self.measurements)

	def get_measurements(self):
		rospy.logwarn(self.measurements)
		return self.measurements

	def __init__(self, frame_id, callback):
		self.measurements = []
		self.callback = callback
		rospy.init_node('measurements_evaluation')
		rospy.Subscriber('measurements', Measurements, self.evaluate)


class MeasurementEvaluator:
	def __init__(self, frame_id, problem, num_particles, visualize=True):
		self.frame_id = frame_id
		self.problem = problem
		self.num_particles = num_particles
		self.visualize = visualize
		self.visualizer = ParticleFilterVisualizer(self.frame_id)
		self.particles = []
		self.weights = []
		self.initialize_measurement_evaluation()
		self.measurement_sub = MeasurementSubscriber(self.frame_id, self.run_measurement_evaluation)


	def initialize_measurement_evaluation(self):
		self.particles = -1.5 + 9.5 * np.random.uniform(size=(self.num_particles,2))
		self.weights = np.repeat(1.0/float(self.num_particles), self.num_particles)

	def run_measurement_evaluation(self, observations):
		rospy.logwarn("=======================Next Iteration of Particle filter===============================")
		if (len(observations) > 0):
			latest_observations = observations[len(observations) - 1].measurements
			#Later it is supposed to work like this:
			for measurement in latest_observations:
			#if(len(latest_observations) > 0):
				#measurement = latest_observations[len(latest_observations) -1]
				if (measurement.signal_strength == 0):
					rospy.logerr("Skipping invalid measurement")
					continue
				position = [measurement.pose.position.x, measurement.pose.position.y]

				#Testing this out
				# self.particles, self.weights = resample(self.particles, self.weights)
				# self.particles = particle_reinvigoration(self.particles)
				#rospy.logwarn("position is: " + str(position) + " measurement is: " + str([measurement.signal_strength]))
				l = problem.likelihood(self.particles, position, [measurement.signal_strength])
				#rospy.logwarn("probablity is: " + str(sum(l)))
				self.particles, self.weights = SIR_step(self.particles, self.weights, l)
				self.visualizer.visualize(self.particles, self.weights)
				#rospy.logwarn("Number of particles is %d" % len(self.particles))

if __name__ == '__main__':
	problem = WLANLocalization([],[])
	evaluator = MeasurementEvaluator('map', problem, 1000, True)
	logging.basicConfig(filename='filter.log', filemode='w', format='%(name)s - %(levelname)s - %(message)s')
	rospy.spin()
