#!/usr/bin/env python
import rospy
import tf
import sys
import actionlib
import numpy as np
from dec_pomdp_msgs.msg import Measurement, Measurements
from geometry_msgs.msg import PoseStamped, Point
from signal_strength_measurer import SignalStrengthMeasurer
from dec_pomdp_msgs.msg import TakeMeasurementsAction, TakeMeasurementsFeedback, TakeMeasurementsGoal, TakeMeasurementsResult


class SignalStrengthServer:
    SIMULATE_MEASUREMENT = False
    _feedback = TakeMeasurementsFeedback()
    _result = TakeMeasurementsResult()
    def __init__(self, action_name, robot_name):
        self.publisher = rospy.Publisher(robot_name + "/measurements", Measurements, queue_size=1)
        self.previous_measurements = Measurements()
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
        self.measurer = SignalStrengthMeasurer()
        self.action_name = action_name
        self.action_server = actionlib.SimpleActionServer(self.action_name, TakeMeasurementsAction, execute_cb=self.handleGetMeasurement, auto_start=False)
        self.action_server.start()

    
    def takeMeasurement(self, simulation=False):
        measurement_msg = Measurement()
        signal_level = None
        now = rospy.Time.now()
        try:
            self.transformer.waitForTransform('map', 'base_footprint', now, rospy.Duration(3.0))
            self.base_footprint_msg.header.stamp = now
            pose = self.transformer.transformPose('map', self.base_footprint_msg)
            measurement_msg.header = pose.header
            measurement_msg.position = pose.pose.position
        except: # (tf.LookupException, tf.ConnectivityException,tf.ExtrapolationException, tf.TransformException) as ex
            e = sys.exc_info()[0]
            rospy.logwarn(str(e))
        if(simulation):
            router_location = rospy.get_param("/access_point_location")
            distance = np.sqrt(
                (router_location['x'] - measurement_msg.position.x)**2 +
                (router_location['y'] - measurement_msg.position.y)**2 +
                (router_location['z'] - measurement_msg.position.z)**2
            )
            rospy.loginfo('Distance is %f', distance)
            signal_level = self.measurer.takeSimulatedMeasurement('TurtleTest', distance)
            measurement_msg.signal_strength = int(signal_level)
        else:
            signal_level = self.measurer.takeMeasurement('TurtleTest')
            measurement_msg.signal_strength = int(signal_level)

        return measurement_msg

    def handleGetMeasurment(self, goal):
        rospy.loginfo('Getting %d measurements'% goal.number)
        starting_len = len(self.previous_measurements.measurements)
        rospy.loginfo('starting with %d measurements stored' % starting_len)
        while ((len(self.previous_measurements.measurements) < (starting_len + goal.number)) & (not rospy.is_shutdown())):
            self.previous_measurements.measurements.append(self.takeMeasurement(simulation=self.SIMULATE_MEASUREMENT))
            self._feedback.collected_measurments = len(self.previous_measurements.measurements) - starting_len
            self.action_server.publish_feedback(self._feedback)
            rospy.loginfo("currently got (%d / %d)" % (len(self.previous_measurements.measurements)- starting_len, goal.number))
        result = Measurements()
        result.measurements = self.previous_measurements.measurements[starting_len : starting_len + goal.number]
        self.publish_measurements(result)
        self._result.measurements = result
        self.action_server.set_succeeded(self._result)

    def publish_measurements(self, measurements):
        self.publisher.publish(measurements)
        rospy.logwarn('Published Measurements!')


if __name__ == '__main__':
    rospy.init_node('measurements')
    robot_name = rospy.get_param('robot_name')
    measurement_node = MeasurementPublisher(rospy.get_name(), robot_name)
    rospy.spin()
