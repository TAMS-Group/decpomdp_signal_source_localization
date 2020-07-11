#!/usr/bin/env python
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from dec_pomdp_msgs.msg import Measurements
from dec_pomdp_msgs.msg import ExecutionState
from dec_pomdp_msgs.srv import StartExperiment
from dec_pomdp_msgs.srv import GeneratePolicies
from dec_pomdp_msgs.msg import Policy, ExecutePolicyAction, ExecutePolicyFeedback, ExecutePolicyGoal, ExecutePolicyResult, ExecuteRandomMovementGoal, ExecuteRandomMovementAction, EvaluateMeasurementsAction, EvaluateMeasurementsGoal
from std_msgs.msg import Int64


class LocalizationManager:
	robot_action_clients = {}
	robot_states = {}
	robot_subscribers = {}
	robot_random_action_clients = {}

	def handle_heartbeat(self, state):
		if state.robot_name not in self.robot_action_clients.keys():
			rospy.logwarn("Registered new Robot: " + state.robot_name)
			self.robot_action_clients[state.robot_name] = actionlib.SimpleActionClient(state.robot_name + '/execute_policy', ExecutePolicyAction)
			self.robot_action_clients[state.robot_name].wait_for_server()
			rospy.logwarn("Registered Action Client for robot %s", state.robot_name)
		if state.robot_name not in self.robot_random_action_clients.keys():
			self.robot_random_action_clients[state.robot_name] = actionlib.SimpleActionClient(state.robot_name + '/execute_random_movement', ExecuteRandomMovementAction)
			self.robot_random_action_clients[state.robot_name].wait_for_server()
		self.robot_states[state.robot_name] = state
		self.heartbeat_publisher.publish(state)


	def handle_start(self, start_msg):
		ex_param = rospy.get_param("/experiment_parameters")
		resulting_measurements = []
		if start_msg.random_movement:
			for robot_action_client in self.robot_random_action_clients.values():
				random_movement_goal = ExecuteRandomMovementGoal()
				random_movement_goal.number_of_steps = ex_param['horizon']
				random_movement_goal.simulate_measurements = start_msg.simulate_measurements
				robot_action_client.send_goal(random_movement_goal)
			while not rospy.is_shutdown():
				rospy.sleep(rospy.Duration(5.0))
				rospy.logwarn("Wait for Random Movement Execution to finish")
				global_state_finished = True
				for action_client in self.robot_random_action_clients.values():
					global_state_finished = global_state_finished & (action_client.get_state() == GoalStatus.SUCCEEDED)
				if global_state_finished:
					for action_client in self.robot_random_action_clients.values():
						result = action_client.get_result().measurements
						resulting_measurements.append(result)
						rospy.logwarn("Got %d results", len(result.measurements))
					break
		else:
			rospy.logwarn("Experiment has been started, waiting for generate Policy Service to come online")
			rospy.wait_for_service('generate_policies')
			try:
				generate_policies = rospy.ServiceProxy('generate_policies', GeneratePolicies)
				rospy.logwarn("Waiting for algorithm to generate policies for %d robots", len(self.robot_states.values()))
				result = generate_policies(ex_param['seed'], ex_param['horizon'], ex_param['width'], ex_param['improvement_steps'], ex_param['num_particles'], ex_param['num_rollouts'], ex_param['num_particles_rollout'], ex_param['gaussian'], ex_param['mx'], ex_param['my'], ex_param['sx'], ex_param['sy'], self.robot_states.values())
				rospy.logwarn("Generation succcessful")
				for policy in result.policies:
					if policy.robot_name in self.robot_action_clients.keys():
						goal = ExecutePolicyGoal()
						goal.policy = policy
						goal.simulate_measurements = start_msg.simulate_measurements
						self.robot_action_clients[policy.robot_name].send_goal(goal)
					else:
						rospy.logwarn("The following robot is not online anymore %s", policy.robot_name)
				while not rospy.is_shutdown():
					rospy.sleep(rospy.Duration(5.0))
					rospy.logwarn("Wait for Policy Execution to finish")
					global_state_finished = True
					for action_client in self.robot_action_clients.values():
						global_state_finished = global_state_finished & (action_client.get_state() == GoalStatus.SUCCEEDED)
					if global_state_finished:
						for action_client in self.robot_action_clients.values():
							result = action_client.get_result().measurements
							resulting_measurements.append(result)
							rospy.logwarn("Got %d results", len(result.measurements))
						break
			except rospy.ServiceException, e:
				rospy.logerr("Service call to generate Policies failed: %s", e)
				return False
		evaluate_measurements_goal = EvaluateMeasurementsGoal()
		combined_measurements = []
		for measurements in resulting_measurements:
			combined_measurements += measurements.measurements
		for idx, measurements in enumerate(resulting_measurements):
			combined_measurements[idx::len(resulting_measurements)] = measurements.measurements
		evaluate_measurements_goal.measurements.measurements = combined_measurements
		rospy.logwarn("Got %d measurements", len(evaluate_measurements_goal.measurements.measurements))
		self.measurement_action_client.send_goal_and_wait(evaluate_measurements_goal)

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
		self.measurement_action_client = actionlib.SimpleActionClient('measurements_evaluation', EvaluateMeasurementsAction)
		self.measurement_action_client.wait_for_server()
		self.start_service = rospy.Service('start_experiment', StartExperiment, self.handle_start)


if __name__ == '__main__':
	x = LocalizationManager()
	rospy.spin()
