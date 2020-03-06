#!/usr/bin/env python
import rospy
from dec_pomdp_msgs.msg import Measurements
from dec_pomdp_msgs.msg import ExecutionState
from dec_pomdp_msgs.srv import StartExperiment
from dec_pomdp_msgs.srv import GeneratePolicies
from dec_pomdp_msgs.msg import Policy


class LocalizationManager:
	robots = {}

    def handle_heartbeat(self, state):
        if state.robot_name not in robots.keys():
            robots[state.robot_name] = rospy.Publisher(state.robot_name + '/policy', Policy)

    def handle_start(self, start_msg):
        rospy.wait_for_service('generate_policies')
        try:
            generate_policies = rospy.ServiceProxy('generate_policies', GeneratePolicies)
            policies = generate_policies(1234567890, 3, 3, 2, 1000, 100, 100, false, 0.1, 0.1, 0.1, 0.1, robots.keys())
        except rospy.ServiceException, e:
            rospy.logerror("Service call to generate Policies failed: ", e)

        for policy in policies:
            if policy.robot_name in robots.keys():
                robots[policy.robot_name].publish(policy)

    def __init__(self):
        rospy.init_node('localization_manager')
        rospy.Subscriber('heartbeat', ExecutionState, self.handle_heartbeat)

        self.start_service = rospy.Service('start_experiment', StartExperiment, self.handle_start)


if __name__ == '__main__':
    x = LocalizationManager()
    rospy.spin()
