#!/usr/bin/env python
import rospy
import actionlib
import tf
import sys
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped, Point
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from dec_pomdp_msgs.msg import Policy
from actionlib_msgs.msg import GoalStatus
from dec_pomdp_msgs.msg import Measurements, TakeMeasurementsAction, TakeMeasurementsGoal
from dec_pomdp_msgs.srv import GetMeasurement
from std_srvs.srv import Empty


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

    def __init__(self):
        self.measurements_per_step = rospy.get_param("/measurements_per_step")
        self.rotation_speed = rospy.get_param("/rotation_speed")
        self.MIN_CLEARING_DISTANCE = rospy.get_param("/min_clearing_distance")
        self.rotation_publisher = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
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
