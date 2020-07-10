#!/usr/bin/env python
import rospy
from dec_pomdp_msgs.msg import Measurements
from dec_pomdp_msgs.msg import ExecutionState
from dec_pomdp_msgs.srv import StartExperiment
from dec_pomdp_msgs.srv import GeneratePolicies
from dec_pomdp_msgs.msg import Policy
from std_msgs.msg import Int64


class LocalizationManager:
	robot_publishers = {}
	robot_states = {}
	robot_subscribers = {}
	robot_random_publishers = {}

	def handle_heartbeat(self, state):
		if state.robot_name not in self.robot_publishers.keys():
			rospy.logwarn("Registered new Robot: " + state.robot_name)
			self.robot_publishers[state.robot_name] = rospy.Publisher(state.robot_name + '/policy', Policy, queue_size=1)
		if state.robot_name not in self.robot_random_publishers.keys():
			self.robot_random_publishers[state.robot_name] = rospy.Publisher(state.robot_name + '/randomMovement', Int64, queue_size=1)
		self.robot_states[state.robot_name] = state
		self.heartbeat_publisher.publish(state)


	def handle_start(self, start_msg):
		ex_param = rospy.get_param("/experiment_parameters")
		if start_msg.random_movement:
			for robot_publisher in self.robot_random_publishers.keys():
				self.robot_random_publishers[robot_publisher].publish(Int64(ex_param['horizon']))
		else:
			rospy.logwarn("Experiment has been started, waiting for generate Policy Service to come online")
			rospy.wait_for_service('generate_policies')
			try:
				generate_policies = rospy.ServiceProxy('generate_policies', GeneratePolicies)
				rospy.logwarn("Waiting for algorithm to generate policies")
				result = generate_policies(ex_param['seed'], ex_param['horizon'], ex_param['width'], ex_param['improvement_steps'], ex_param['num_particles'], ex_param['num_rollouts'], ex_param['num_particles_rollout'], ex_param['gaussian'], ex_param['mx'], ex_param['my'], ex_param['sx'], ex_param['sy'], self.robot_states.values())
				rospy.logwarn("Generation succcessful")
				for policy in result.policies:
					if policy.robot_name in self.robot_publishers.keys():
						self.robot_publishers[policy.robot_name].publish(policy)
					else:
						rospy.logwarn("The following robot is not online anymore %s", policy.robot_name)
			except rospy.ServiceException, e:
				rospy.logerr("Service call to generate Policies failed: %s", e)
				return False
		return True

	def handle_measurements(self, measurements):
		self.measurement_publisher.publish(measurements)


	def __init__(self):
		rospy.init_node('localization_manager')
		possible_agents = rospy.get_param('/possible_agents')
		for agent in possible_agents:
			self.robot_subscribers[agent + 'heartbeat'] = rospy.Subscriber(agent + '/heartbeat', ExecutionState, self.handle_heartbeat)
		for agent in possible_agents:
			self.robot_subscribers[agent + 'measurements'] = rospy.Subscriber(agent +'/measurements', Measurements, self.handle_measurements)
		self.measurement_publisher = rospy.Publisher('/measurements', Measurements, queue_size=10)
		self.heartbeat_publisher = rospy.Publisher('/heartbeat', ExecutionState, queue_size=10)
		self.start_service = rospy.Service('start_experiment', StartExperiment, self.handle_start)


if __name__ == '__main__':
	x = LocalizationManager()
	rospy.spin()
