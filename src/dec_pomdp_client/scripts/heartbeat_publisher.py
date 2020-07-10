#!/usr/bin/env python

import rospy
import tf
import sys
from dec_pomdp_msgs.msg import ExecutionState
from geometry_msgs.msg import PoseStamped
from signal_strength.signal_strength_measurer import SignalStrengthMeasurer



def main():
	rospy.init_node('heartbeat')
	robot_name = rospy.get_param('robot_name')
	transformer = tf.TransformListener()
	base_footprint_msg = PoseStamped()
	base_footprint_msg.header.frame_id = 'base_footprint'
	base_footprint_msg.pose.orientation.w = 1
	base_footprint_msg.header.stamp = rospy.Time(0)
	transformer.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(3.0))
	pub = rospy.Publisher('heartbeat', ExecutionState, queue_size=1)
	rate = rospy.Rate(1)
	msg = ExecutionState()
	msg.robot_name = robot_name
	while not rospy.is_shutdown():
		# msg.pose.header.stamp = rospy.get_rostime()
		msg.status = ExecutionState.IDLE
		# msg.pose.header.frame_id = 'map'
		try:
			now = rospy.Time.now()
			transformer.waitForTransform('map', 'base_footprint', now, rospy.Duration(3.0))
			base_footprint_msg.header.stamp = now
			msg.pose = transformer.transformPose('map', base_footprint_msg)
			pub.publish(msg)
		except:
			e = sys.exc_info()[0]
			rospy.logwarn(str(e))
		rate.sleep()

if __name__ == '__main__':
	main()
