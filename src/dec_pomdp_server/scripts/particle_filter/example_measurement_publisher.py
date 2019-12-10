#!/usr/bin/env python
import rospy
import sys
from dec_pomdp_msgs.msg import Measurement, Measurements
from geometry_msgs.msg import PoseStamped

class ExampleMeasurment:
    def __init__(self, x, y, z, time, signal_strength):
        self.x = x
        self.y = y
        self.z = z
        self.time = time
        self.signal_strength = signal_strength

class MeasurementPublisher:
    def __init__(self):
        self.publisher = rospy.Publisher("measurements", Measurements, queue_size=1)
        self.previous_measurements = Measurements()
        self.nr = 0
        now = rospy.Time.now()
        self.example_measurments = [
            ExampleMeasurment(1, 1, 1, now, -40),
            ExampleMeasurment(2, 2, 1, now, -50),
            ExampleMeasurment(3, 3, 1, now, -60),
            ExampleMeasurment(4, 4, 1, now, -70),
            ExampleMeasurment(5, 5, 1, now, -80),
            ExampleMeasurment(6, 6, 1, now, -90),
            ExampleMeasurment(7, 7, 1, now, -100),
            ExampleMeasurment(8, 8, 1, now, -110)
        ]

    def getMeasurement(self):
        measurement_msg = Measurement()
        now = rospy.Time.now()
        measurement = self.example_measurments[self.nr]
        self.nr = (1 + self.nr) % len(self.example_measurments)
        measurement_msg.header.stamp = measurement.time
        measurement_msg.header.frame_id = 'map'
        measurement_msg.position.x = measurement.x
        measurement_msg.position.y = measurement.y
        measurement_msg.position.z = measurement.z
        measurement_msg.signal_strength = measurement.signal_strength
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
