#!/usr/bin/env python
import rospy
import sys
import rosbag
from dec_pomdp_msgs.msg import Measurement, Measurements, ExecutionState
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid

# Can be used to

if __name__ == '__main__':
    rospy.init_node('measurements')
    rate = rospy.Rate(1.0) # to be removed later
    topics = {
        "measurements": Measurements,
        "map": OccupancyGrid,
        "heartbeat": ExecutionState
    }
    publishers = {}
    #publisher = rospy.Publisher("measurements", Measurements, queue_size=1)
    for topic, msg_type in topics.items():
        publishers[topic] = rospy.Publisher(topic, msg_type, queue_size=1)

    bag = rosbag.Bag('/informatik2/students/home/6tkruege/bachelor/2020-01-20-16-19-49.bag')
    for topic, msg, t in bag.read_messages(topics=topics.keys()):
        publishers[topic].publish(msg)
        if rospy.is_shutdown():
            break;
        rate.sleep()
    bag.close()
