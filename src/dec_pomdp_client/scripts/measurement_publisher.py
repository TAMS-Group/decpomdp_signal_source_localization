#!/usr/bin/env python
import rospy
import tf
import sys
from dec_pomdp_msgs.msg import Measurement, Measurements
from geometry_msgs.msg import PoseStamped
from signal_strength_measurer import SignalStrengthMeasurer

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

    def getMeasurement(self):
        measurement_msg = Measurement()
        link_quality, max_link_quality, signal_level = self.measurer.takeMeasurement()
        try:
            now = rospy.Time.now()
            self.transformer.waitForTransform('map', 'base_footprint', now, rospy.Duration(3.0))
            self.base_footprint_msg.header.stamp = now
            pose = self.transformer.transformPose('map', self.base_footprint_msg)
            measurement_msg.header = pose.header
            measurement_msg.position = pose.pose.position
            measurement_msg.signal_strength = int(signal_level)
        except: # (tf.LookupException, tf.ConnectivityException,tf.ExtrapolationException, tf.TransformException) as ex
            e = sys.exc_info()[0]
            rospy.logwarn(str(e))
        return measurement_msg

    def publish_measurements(self):
        self.previous_measurements.measurements.append(self.getMeasurement())
        self.publisher.publish(self.previous_measurements)


if __name__ == '__main__':
    rospy.init_node('measurements')
    rate = rospy.Rate(0.1) # to be removed later
    measurement_node = MeasurementPublisher()
    while not rospy.is_shutdown():
        measurement_node.publish_measurements()
        rate.sleep()
