#!/usr/bin/env python
import rospy
from dec_pomdp_msgs.msg import Policy
from policy_executioner import PolicyExecutioner

class PolicyListener:
    def handle_policy(self, policy):
        rospy.loginfo("Got policy: now executing!")
        self.policy_executioner.execute_policy(policy)

    def __init__(self, policy_topic, policy_executioner):
        self.policy_executioner = policy_executioner
        rospy.loginfo("subscribing to " + policy_topic)
        rospy.Subscriber(policy_topic, Policy, self.handle_policy)

if __name__ == "__main__":
    rospy.init_node('policy_execution')
    policy_executioner = PolicyExecutioner()
    robot_name = rospy.get_param('robot_name')
    rospy.loginfo(robot_name + '/policy')
    policy_listener = PolicyListener(robot_name + '/policy', policy_executioner)
    rospy.spin()