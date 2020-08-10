#!/usr/bin/env python
import rospy
import actionlib
import tf
import sys
import numpy as np
import random
from geometry_msgs.msg import Twist, PoseStamped, Point
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from dec_pomdp_msgs.msg import Policy
from actionlib_msgs.msg import GoalStatus
from dec_pomdp_msgs.msg import Measurements, TakeMeasurementsAction, TakeMeasurementsGoal, ExecutePolicyAction, ExecutePolicyFeedback, ExecutePolicyGoal, ExecutePolicyResult, ExecutionState
from dec_pomdp_msgs.srv import GetMeasurement, GetExecutionStatus, GetExecutionStatusResponse
from std_srvs.srv import Empty


class PolicyExecutioner:
    """
    This class can be used to execute a predefined Policy
    It will make the robot move to the defined positions (NodeActions),
    take signal strength measurements in those places
    and determine according to the result where to move next
    """
    _feedback = ExecutePolicyFeedback()
    _result = ExecutePolicyResult()
    _current_status = ExecutionState.IDLE

    def execute_policy(self, goal):
        """ Executes a given policy of type dec_pomdp_msgs.msg.Policy"""
        policy = goal.policy
        step = 0
        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose.header.frame_id = "map"
        current_node = policy.starting_node
        while not rospy.is_shutdown():
            node_action = self.find(lambda node_action: node_action.node_number == current_node, policy.actions)
            node_transition = self.find(lambda node_transition: node_transition.node_number == current_node, policy.transitions)
            if (len(node_transition.edges) == 0):
                rospy.loginfo('Current Node number is %d and the action is end' % ( current_node ))
                break
            rospy.loginfo('Current node number is %d and node action is x = %f and y = %f' % (current_node, node_action.pose.position.x, node_action.pose.position.y))
            move_base_goal.target_pose.pose = node_action.pose
            move_base_goal.target_pose.header.stamp = rospy.get_rostime()
            self.move_to_goal(move_base_goal)
            rospy.loginfo('Taking measurement')
            measurement_value = self.take_measurement()
            rospy.loginfo('measurement taken %d' % measurement_value)
            for edge in node_transition.edges:
                if (measurement_value > edge.measurement_interval.lower_bound and measurement_value <= edge.measurement_interval.upper_bound):
                    current_node = edge.node_number
                    rospy.loginfo('next node has been found')
                    break
            self._feedback.step = step
            self._feedback.current_measurements = self.measurements
            self.policy_action_server.publish_feedback(self._feedback)
        rospy.loginfo("Finished policy execution")
        self._result.measurements = self.measurements
        self.measurements = Measurements()
        # Get ready for the next experiment by moving to a random location
        self.move_to_random_start_location()
        self._current_status = ExecutionState.IDLE
        self.policy_action_server.set_succeeded(self._result)


    def take_measurement(self):
        self._current_status = ExecutionState.MEASURING
        goal = TakeMeasurementsGoal(number=self.measurements_per_step)
        self.measurement_client.send_goal(goal)
        while not rospy.is_shutdown():
            current_state = self.measurement_client.get_state()
            if (current_state == GoalStatus.SUCCEEDED):
                self.rotation_msg.angular.z = 0.0
                self.rotation_publisher.publish(self.rotation_msg)
                result = self.measurement_client.get_result()
                signal_strengths = []
                for measurement in result.measurements.measurements:
                    self.measurements.measurements.append(measurement)
                    signal_strengths.append(measurement.signal_strength)
                return np.mean(signal_strengths)
            elif(current_state == GoalStatus.LOST or current_state == GoalStatus.REJECTED):
                rospy.logerr("Action client failed to take measurements")
                break
            self.rotation_msg.angular.z = self.rotation_speed
            self.rotation_publisher.publish(self.rotation_msg)
            rospy.sleep(rospy.Duration(0.5))

    def move_to_goal(self, goal):
        self._current_status = ExecutionState.MOVING
        while not rospy.is_shutdown():
            rospy.loginfo('Moving to goal')
            self.move_base_client.send_goal_and_wait(goal)
            current_state = self.move_base_client.get_state()
            if (current_state == GoalStatus.SUCCEEDED):
                rospy.loginfo('Goal has been reached')
                break
            elif(current_state == GoalStatus.RECALLED or current_state == GoalStatus.PREEMPTED):
                rospy.logerr('Goal has been Recalled or Preempted')
                break
            else:
                try:
                    rospy.loginfo('1')
                    now = rospy.Time.now()
                    self.transformer.waitForTransform('map', 'base_footprint', now, rospy.Duration(3.0))
                    rospy.loginfo('2')
                    self.base_footprint_msg.header.stamp = now
                    rospy.loginfo('3')
                    pose = self.transformer.transformPose('map', self.base_footprint_msg)
                    rospy.loginfo(goal.target_pose.pose.position.x )
                    rospy.loginfo(pose.pose.position.x)
                    distance = np.sqrt(
                        (goal.target_pose.pose.position.x - pose.pose.position.x)**2 +
                        (goal.target_pose.pose.position.y - pose.pose.position.y)**2
                    )
                    rospy.loginfo('5')
                    if(distance > self.MIN_CLEARING_DISTANCE):
                        rospy.loginfo('Clearing costmaps')
                        self.clear_costmaps()
                    rospy.logwarn('Goal with coordinates x = %f and y = %f couldnt be reached will try again' % (goal.target_pose.pose.position.x, goal.target_pose.pose.position.y))
                except: # (tf.LookupException, tf.ConnectivityException,tf.ExtrapolationException, tf.TransformException) as ex
                    e = sys.exc_info()[0]
                    rospy.logerr(str(e))

    def find(self, l_function, array):
        for item in array:
            if l_function(item):
                return item
    
    def move_to_random_start_location(self):
        random_location = random.choice(range(len(self.locations)))
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = self.locations[random_location].x
        goal.target_pose.pose.position.y = self.locations[random_location].y
        goal.target_pose.pose.orientation.w = 1
        goal.target_pose.header.stamp = rospy.get_rostime()
        self.move_to_goal(goal)

    def get_execution_status(self, req):
        return GetExecutionStatusResponse(self._current_status)

    def __init__(self):
        mgfp_param_name = rospy.search_param('movement_graph_file_path')
        movement_graph_file_path = rospy.get_param(mgfp_param_name)
        #initialize Execution status feedback service
        self.execution_status_service = rospy.Service('get_execution_status', GetExecutionStatus, self.get_execution_status)
        #Read nodes
        location_file = open(movement_graph_file_path + "/locations.txt", "r")
        self.locations = dict()
        for line in location_file.readlines():
            line_content = [float(x) for x in line.split(" ")]
            self.locations[int(line_content[0])] = Point(line_content[1],line_content[2], 0.0)
        location_file.close()
        mps_param_name = rospy.search_param('measurements_per_step')
        self.measurements_per_step = rospy.get_param(mps_param_name)
        rs_param_name = rospy.search_param('rotation_speed')
        self.rotation_speed = rospy.get_param(rs_param_name)
        mci_param_name = rospy.search_param('min_clearing_distance')
        self.MIN_CLEARING_DISTANCE = rospy.get_param(mci_param_name)
        self.rotation_publisher = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        self.rotation_msg = Twist()
        self.measurements = Measurements()
        self.measurement_client = actionlib.SimpleActionClient('measurements', TakeMeasurementsAction)
        self.measurement_client.wait_for_server()
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()
        self.transformer = tf.TransformListener()
        try:
            self.transformer.waitForTransform('map',
                                    'base_footprint',
                                    rospy.Time(0),
                                    rospy.Duration(3.0)
                                    )
        except:
            e = sys.exc_info()[0]
            rospy.logwarn(str(e))
        self.base_footprint_msg = PoseStamped()
        self.base_footprint_msg.header.frame_id = 'base_footprint'
        self.base_footprint_msg.pose.orientation.w = 1
        self.base_footprint_msg.header.stamp = rospy.Time(0)
        self.clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        self.policy_action_server = actionlib.SimpleActionServer('execute_policy', ExecutePolicyAction, execute_cb=self.execute_policy, auto_start=False)
        self.policy_action_server.start()

if __name__ == "__main__":
    rospy.init_node('policy_execution')
    policy_executioner = PolicyExecutioner()
    robot_name = rospy.get_param('robot_name')
    rospy.spin()
