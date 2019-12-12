#!/usr/bin/env python
import rospy
import sys
import rosbag
from dec_pomdp_msgs.msg import Measurement, Measurements
from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':
    rospy.init_node('measurements')
    rate = rospy.Rate(0.1) # to be removed later
    publisher = rospy.Publisher("measurements", Measurements, queue_size=1)

    while not rospy.is_shutdown():
        bag = rosbag.Bag('/informatik2/students/home/6tkruege/bachelor/2019-12-12-16-48-18.bag')
        for topic, msg, t in bag.read_messages(topics=['measurements']):
            publisher.publish(msg)
            rate.sleep()
        bag.close()
