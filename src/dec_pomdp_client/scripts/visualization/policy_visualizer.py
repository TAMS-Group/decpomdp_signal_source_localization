#!/usr/bin/env python

#    policy_visualizer.py
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
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Vector3, Point
from std_msgs.msg import ColorRGBA
from dec_pomdp_msgs.msg import Policy

class PolicyVisualizer:
    """
    This class can be used to visualize the potential next Steps of a policy
    """
    NAMESPACE="PolicyVisualization"

    def find(self, l_function, array):
        for item in array:
            if l_function(item):
                return item

    def visualize_next_steps(self, policy, current_node):
        next_steps_marker = MarkerArray()
        node_action = self.find(lambda action: action.node_number == current_node, policy.actions)
        node_transitions = self.find(lambda transitions: transitions.node_number == current_node, policy.transitions)
        for edge in node_transitions.edges:
            arrow_marker = Marker(
                type=Marker.ARROW,
                ns=self.NAMESPACE,
                id=self.marker_id,
                lifetime=rospy.Duration(0),
                scale=Vector3(0.1, 0.1, 1.0),
                color=ColorRGBA(0.0,0.0,0.0, 1.0)
            )
            self.marker_id += 1
            arrow_marker.header.stamp =rospy.get_rostime()
            arrow_marker.header.frame_id = self.frame_id
            num_edges = float(len(node_transitions.edges) - 1)
            num_lower_intervals = sum(e.measurement_interval.lower_bound < edge.measurement_interval.lower_bound for e in node_transitions.edges)
            arrow_marker.color.r = 1.0 - float(num_lower_intervals)/num_edges
            num_higher_intervals = sum(e.measurement_interval.upper_bound > edge.measurement_interval.upper_bound for e in node_transitions.edges)
            arrow_marker.color.g = 1.0 - float(num_higher_intervals)/num_edges
            start = node_action.pose.position
            end = self.find(lambda action: action.node_number == edge.node_number, policy.actions).pose.position
            start.z = float(current_node)/len(policy.nodes)
            end.z = float(current_node)/len(policy.nodes)
            arrow_marker.points.append(start)
            arrow_marker.points.append(end)
            next_steps_marker.markers.append(arrow_marker)
        self.marker_publisher.publish(next_steps_marker)

    def visualize_policy(self, policy):
        self.marker_id = 0
        for node in policy.nodes:
            self.visualize_next_steps(policy, node)

    def __init__(self, frame_id, robot_name):
        self.marker_id = 0
        self.robot_name = robot_name
        self.marker_publisher = rospy.Publisher('possible_moves', MarkerArray, queue_size=100)
        self.frame_id = frame_id

if __name__ == "__main__":
    rospy.init_node('policy_visualization')
    robot_name = rospy.get_param('robot_name')
    visualizer = PolicyVisualizer('map', robot_name)
    policy_listener = rospy.Subscriber('policy',Policy, visualizer.visualize_policy)
    rospy.spin()
