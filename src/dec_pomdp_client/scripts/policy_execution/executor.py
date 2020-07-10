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

class Executor(object):
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
                pose = self.get_current_loc()
                distance = np.sqrt(
                    (goal.target_pose.pose.position.x - pose.pose.position.x)**2 +
                    (goal.target_pose.pose.position.y - pose.pose.position.y)**2
                )
            if(distance > self.MIN_CLEARING_DISTANCE):
                rospy.loginfo('Clearing costmaps')
                self.clear_costmaps()
            rospy.logwarn('Goal with coordinates x = %f and y = %f couldnt be reached will try again' % (goal.target_pose.pose.position.x, goal.target_pose.pose.position.y))

    def get_current_loc(self):
        try:
            now = rospy.Time.now()
            self.transformer.waitForTransform('map', 'base_footprint', now, rospy.Duration(3.0))
            self.base_footprint_msg.header.stamp = now
            pose = self.transformer.transformPose('map', self.base_footprint_msg)
            return pose
        except:
            e = sys.exc_info()[0]
            rospy.logerr(str(e))
            return PoseStamped()

    def __init__(self):
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
            self.transformer.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(3.0))
        except:
            e = sys.exc_info()[0]
            rospy.logwarn(str(e))
        self.base_footprint_msg = PoseStamped()
        self.base_footprint_msg.header.frame_id = 'base_footprint'
        self.base_footprint_msg.pose.orientation.w = 1
        self.base_footprint_msg.header.stamp = rospy.Time(0)
        self.clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        rospy.logwarn('Parent fully initialized')
