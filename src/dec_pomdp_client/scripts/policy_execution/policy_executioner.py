#!/usr/bin/env python
import actionlib
import rospy
import numpy as np 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from dec_pomdp_msgs.msg import Policy
from actionlib_msgs.msg import GoalStatus
from dec_pomdp_msgs.msg import Measurements, TakeMeasurementsAction, TakeMeasurementsGoal
from dec_pomdp_msgs.srv import GetMeasurement



class PolicyExecutioner:
    """
    This class can be used to execute a predefined Policy
    It will make the robot move to the defined positions (NodeActions),
    take signal strength measurements in those places 
    and determine according to the result where to move next
    """
    
    def execute_policy(self, policy):
        """ Executes a given policy of type dec_pomdp_msgs.msg.Policy"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        current_node = policy.starting_node
        while not rospy.is_shutdown():
            node_action = self.find(lambda node_action: node_action.node_number == current_node, policy.actions)
            rospy.loginfo('Current node number is %d and node action is x = %f and y = %f' % (current_node, node_action.pose.position.x, node_action.pose.position.y))
            goal.target_pose.pose = node_action.pose
            goal.target_pose.header.stamp = rospy.get_rostime()
            self.move_to_goal(goal)
            rospy.loginfo('Taking measurement')
            measurement_value = self.take_measurement()
            rospy.loginfo('measurement taken %d' % measurement_value)
            node_transition = self.find(lambda node_transition: node_transition.node_number == current_node, policy.transitions)
            if (len(node_transition.edges) == 0):
                break
            for edge in node_transition.edges:
                if (measurement_value > edge.measurement_interval.lower_bound and measurement_value <= edge.measurement_interval.upper_bound):
                    current_node = edge.node_number
                    rospy.loginfo('next node has been found')
                    break
        rospy.loginfo("Finished policy execution")


    def take_measurement(self):
        goal = TakeMeasurementsGoal(number=self.measurements_per_step)
        self.measurement_client.send_goal(goal)
        while not rospy.is_shutdown():
            current_state = self.measurement_client.get_state()
            if (current_state == GoalStatus.SUCCEEDED):
                result = self.measurement_client.get_result()
                signal_strengths = []
                for measurement in result.measurements.measurements:
                    self.measurements.measurements.append(measurement)
                    signal_strengths.append(measurement.signal_strength)
                return np.mean(signal_strengths)
            elif(current_state == GoalStatus.LOST or current_state == GoalStatus.REJECTED):
                rospy.logerr("Action client failed to take measurements")
                break
            rospy.sleep(rospy.Duration(0.5))

    def move_to_goal(self, goal):
        rospy.logwarn('Moving to goal')
        self.move_base_client.send_goal(goal)
        while not rospy.is_shutdown():
            current_state = self.move_base_client.get_state()
            if (current_state == GoalStatus.SUCCEEDED):
                rospy.loginfo('Goal has been reached')
                break
            elif(current_state == GoalStatus.LOST or current_state == GoalStatus.REJECTED):
                rospy.logerr('Goal with coordinates x = %f and y = %f couldnt be reached' % (goal.target_pose.pose.position.x, goal.target_pose.pose.position.y))
            rospy.logwarn('Sleeping until arrival current state is %d' % current_state)
            rospy.sleep(rospy.Duration(0.5))

    def find(self, l_function, array):
        for item in array:
            if l_function(item):
                return item

    def __init__(self):
        self.measurements_per_step = rospy.get_param("/measurements_per_step")
        self.measurements = Measurements()
        self.measurement_client = actionlib.SimpleActionClient('measurements', TakeMeasurementsAction)
        self.measurement_client.wait_for_server()
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()
        
