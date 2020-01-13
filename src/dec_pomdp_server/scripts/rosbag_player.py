#!/usr/bin/env python
import rospy
import sys
import rosbag
from dec_pomdp_msgs.msg import Measurement, Measurements
from visualization_msgs.msg import Marker
from dec_pomdp_msgs.msg import ExecutionState
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid

if __name__ == '__main__':
    rospy.init_node('rosbag_player')
    rate = rospy.Rate(0.1)
    rospy.spin()
    # bag = rosbag.Bag('/informatik2/students/home/6tkruege/bachelor/2020-01-13-11-45-27.bag')
    # topic_info = bag.get_type_and_topic_info().topics
    # publishers = dict()
    # for topic, info in topic_info.items():
    #     if(info.msg_type=='visualization_msgs/Marker'):
    #         publishers[topic] = rospy.Publisher(topic, Marker, queue_size=1)
    #     elif(info.msg_type=='dec_pomdp_msgs/ExecutionState'):
    #         publishers[topic] = rospy.Publisher(topic, ExecutionState, queue_size=1)
    #     elif(info.msg_type=='dec_pomdp_msgs/Measurements'):
    #         publishers[topic] = rospy.Publisher(topic, Measurements, queue_size=1)
    #     elif(info.msg_type=='nav_msgs/OccupancyGrid'):
    #         publishers[topic] = rospy.Publisher(topic, OccupancyGrid, queue_size=1)
    #     else:
    #         rospy.logwarn("There is no message type like " + info.msg_type +" defined" )
    # for topic, msg, t in bag.read_messages(topics=topic_info.keys()):
    #     if topic in publishers:
    #         publishers[topic].publish(msg)
    #         rate.sleep()
    # bag.close()
