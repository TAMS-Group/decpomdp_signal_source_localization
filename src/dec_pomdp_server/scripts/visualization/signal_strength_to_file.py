#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
import json
import numpy as np
from dec_pomdp_msgs.msg import Measurement, Measurements

class SignalStrengthSubscriber:
    def handleMeasurements(self, measurements):
        f = open('/home/tobias/Documents/BachelorThesis Visualization/measurements2.json', 'w')
        data = {}
        data["measurements"] = []
        x_orientation = []
        y_signal_strengths = []
        for measurement in measurements.measurements:
            data["measurements"].append({'time_secs': measurement.header.stamp.secs, 'time_nsecs': measurement.header.stamp.nsecs, 'orientation': measurement.pose.orientation.z, 'signal_strength': measurement.signal_strength})
        json.dump(data, f)
        f.close()

    
    def __init__(self):
        self.measurement_subscriber = rospy.Subscriber('/donny/measurements', Measurements, self.handleMeasurements)

if __name__ == '__main__':
    rospy.init_node('signal_strength_visualization', log_level=rospy.INFO)
    visualizer = SignalStrengthSubscriber()
    rospy.spin()
    