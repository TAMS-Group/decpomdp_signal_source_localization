#!/usr/bin/env python

import rospy
from dec_pomdp_msgs.msg import ExecutionState

def main():
	rospy.init_node('heartbeat')
	pub = rospy.Publisher('heartbeat', ExecutionState, queue_size=1)
	rate = rospy.Rate(1)
	msg = ExecutionState()
	msg.robot_name = rospy.get_param('~robot_name')
	while not rospy.is_shutdown():
		msg.pose.header.stamp = rospy.get_rostime()
		msg.status = ExecutionState.IDLE
		msg.pose.header.frame_id = 'map'
		msg.pose.pose.orientation.w = 1
		pub.publish(msg)
		rate.sleep()

if __name__ == '__main__':
	main()