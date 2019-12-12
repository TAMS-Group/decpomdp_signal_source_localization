#!/usr/bin/env python
import rosbag
import rospy
from dec_pomdp_msgs.msg import Measurement, Measurements

class MeasurementCollector:
    def collect(self, measurements):
        try:
            self.bag.write('Measurment List: ', measurements)
        except:
            e = sys.exc_info()[0]
            rospy.logwarn(str(e))

    def close(self):
        self.bag.close()

    def __init__(self):
        self.bag = rosbag.Bag('measurements.bag', 'w')
        rospy.Subscriber('measurements', Measurements, self.collect)


if __name__ == '__main__':
    rospy.init_node('measurements_collector')
    x = MeasurementCollector()
    try:
        rospy.spin()
    finally:
        x.close()
