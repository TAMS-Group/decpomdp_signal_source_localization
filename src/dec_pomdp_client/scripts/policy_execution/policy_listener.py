#!/usr/bin/env python

#    policy_listener.py
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
    policy_listener = PolicyListener('policy', policy_executioner)
    rospy.spin()
