#!/usr/bin/env python
import rospy
import tf
import sys
import numpy as np
from dec_pomdp_msgs.msg import Measurement, Measurements
from geometry_msgs.msg import PoseStamped, Point
from signal_strength_measurer import SignalStrengthMeasurer
from dec_pomdp_msgs.srv import GetMeasurement


class MeasurementPublisher:
    def __init__(self):
        self.publisher = rospy.Publisher("measurements", Measurements, queue_size=1)
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
        self.service = rospy.Service('collect_measurements', GetMeasurement, self.handleGetMeasurment)

    def takeMeasurement(self, position=None):
        measurement_msg = Measurement()
        signal_level = None
        now = rospy.Time.now()
        if(position is None):
            signal_level = self.measurer.takeMeasurement('TurtleTest')
            try:
                self.transformer.waitForTransform('map', 'base_footprint', now, rospy.Duration(3.0))
                self.base_footprint_msg.header.stamp = now
                pose = self.transformer.transformPose('map', self.base_footprint_msg)
                measurement_msg.header = pose.header
                measurement_msg.position = pose.pose.position
                measurement_msg.signal_strength = int(signal_level)
            except: # (tf.LookupException, tf.ConnectivityException,tf.ExtrapolationException, tf.TransformException) as ex
                e = sys.exc_info()[0]
                rospy.logwarn(str(e))
        else:
            router_location = rospy.get_param("/access_point_location")
            distance = np.sqrt(router_location['x']**2 + position.x**2)
            rospy.logwarn('Distance is %f', distance)
            signal_level = self.measurer.takeSimulatedMeasurement('TurtleTest', distance)
            measurement_msg.signal_strength = int(signal_level)
        return measurement_msg

    def collectMeasurement(self):
        position = Point()
        position.x = 0.0
        position.y = 0.0
        self.previous_measurements.measurements.append(self.takeMeasurement(position=position))

    def handleGetMeasurment(self, srv_msg):
        rospy.logwarn('GetsCalled')
        rospy.logwarn(srv_msg.number)
        starting_len = len(self.previous_measurements.measurements)
        rospy.logwarn('starting_len')
        rospy.logwarn(len(self.previous_measurements.measurements) < starting_len + srv_msg.number)
        while ((len(self.previous_measurements.measurements) < (starting_len + srv_msg.number)) & (not rospy.is_shutdown())):
            rospy.logwarn(len(self.previous_measurements.measurements))
            rospy.sleep(1.)
        result = Measurements()
        result.measurements = self.previous_measurements.measurements[starting_len : starting_len + srv_msg.number]
        self.publish_measurements(result)
        return result


    def publish_measurements(self, measurements):
        self.publisher.publish(measurements)
        rospy.logwarn('got measurements! ')


if __name__ == '__main__':
    rospy.init_node('measurements')
    rate = rospy.Rate(1) # to be removed later
    measurement_node = MeasurementPublisher()
    while not rospy.is_shutdown():
        measurement_node.collectMeasurement()
        rospy.logwarn('sleeping')
        rate.sleep()
