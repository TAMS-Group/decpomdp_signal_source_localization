#!/usr/bin/env python
import rospy
import sys
import rosbag
from dec_pomdp_msgs.msg import Measurement, Measurements
from visualization_msgs.msg import Marker
from dec_pomdp_msgs.msg import ExecutionState
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid

def collect(measurements):
    numbers = []
    for measurement in measurements.measurements:
        numbers.append(measurement.signal_strength);
    rospy.logwarn(numbers)

if __name__ == '__main__':
    rospy.init_node('model_fitting_collector')
    rospy.Subscriber('measurements', Measurements, collect)
    rospy.spin()
