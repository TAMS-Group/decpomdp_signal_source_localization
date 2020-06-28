#!/usr/bin/env python
import rospy
import numpy as np
from dec_pomdp_msgs.msg import Measurements
from particle_filter import *
from wlan_localization import WLANLocalization
from particle_filter_visualization import ParticleFilterVisualizer
from std_srvs.srv import Empty
import json
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
		self.real_location = rospy.get_param("access_point_location")
		self.num_of_measurements = 0
		self.logging_code = 0
		self.log_file_path = rospy.get_param('result_folder_path') + 'measurement_results.json'
		self.initialize_measurement_evaluation()
		self.measurement_sub = MeasurementSubscriber(self.frame_id, self.run_measurement_evaluation)
		self.start_service = rospy.Service('reset_measurement_evaluation', Empty, self.initialize_measurement_evaluation)


	def initialize_measurement_evaluation(self, request=Empty()):
		self.particles = -1.5 + 9.5 * np.random.uniform(size=(self.num_particles,2))
		self.weights = np.repeat(1.0/float(self.num_particles), self.num_particles)
		#self.particles, self.weights = initialize(self.num_particles, -1.5, -1.5, 7.0, 8.0)
		rospy.loginfo(len(self.particles))
		rospy.loginfo(len(self.weights))
		self.num_of_measurements = 0
		self.logging_code = str(rospy.get_time())
		data = None
		with open(self.log_file_path) as json_file:
			data = json.load(json_file)
			data[self.logging_code] = []
		if data is not None:
			self.write_json(data, self.log_file_path)
		self.visualizer.visualize(self.particles, self.weights)
		rospy.loginfo('Successfully initialized particle filter')
		return True
	
	def write_json(self, data, filename):
		with open(filename, 'w') as file:
			json.dump(data, file, indent=4)


	def run_measurement_evaluation(self, observations):
		rospy.loginfo("=======================Next Iteration of Particle filter===============================")
		json_data = None
		experiment_json_object = []
		with open(self.log_file_path) as json_file:
			json_data = json.load(json_file)
			experiment_json_object = json_data[self.logging_code]
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
				#self.particles, self.weights = resample(self.particles, self.weights)
				#self.particles = particle_reinvigoration(self.particles)
				#rospy.logwarn("position is: " + str(position) + " measurement is: " + str([measurement.signal_strength]))
				l = problem.likelihood(self.particles, position, [measurement.signal_strength])
				#rospy.logwarn("probablity is: " + str(sum(l)))
				self.particles, self.weights = SIR_step(self.particles, self.weights, l)
				self.visualizer.visualize(self.particles, self.weights)
				#rospy.logwarn("Number of particles is %d" % len(self.particles))
				weighted_mean_error = 0
				self.num_of_measurements += 1
				for index, particle in enumerate(self.particles):
					distance = np.sqrt((self.real_location['x'] - particle[0])**2 + (self.real_location['y'] - particle[1])**2 )
					weighted_distance = self.weights[index] * distance**2
					weighted_mean_error += weighted_distance
				weighted_mean_error = np.sqrt(weighted_mean_error / sum(self.weights))
				# rospy.loginfo("Error of current particle filter is %f after evaluation of %d measurements sum of weight is %f", weighted_mean_error, self.num_of_measurements, sum(self.weights))
				new_value = {"num_measurements": self.num_of_measurements, "error": weighted_mean_error}
				experiment_json_object.append(new_value)
				if json_data is not None:
					self.write_json(json_data, self.log_file_path)

if __name__ == '__main__':
	rospy.init_node('measurements_evaluation')
	problem = WLANLocalization([],[])
	evaluator = MeasurementEvaluator('map', problem, 1500, True)
	logging.basicConfig(filename='filter.log', filemode='w', format='%(name)s - %(levelname)s - %(message)s')
	rospy.spin()
