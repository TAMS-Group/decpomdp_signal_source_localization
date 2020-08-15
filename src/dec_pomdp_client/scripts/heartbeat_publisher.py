#!/usr/bin/env python
#    heartbeat_publisher.py
#
#	 Copyright 2020 Tobias Krueger
#
#    Licensed under the Apache License, Version 2.0 (the "License");
#    you may not use this file except in compliance with the License.
#    You may obtain a copy of the License at
#
#        http://www.apache.org/licenses/LICENSE-2.0
#
#    Unless required by applicable law or agreed to in writing, software
#    distributed under the License is distributed on an "AS IS" BASIS,
#    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#    See the License for the specific language governing permissions and
#    limitations under the License.


import rospy
import tf
import sys
from dec_pomdp_msgs.msg import ExecutionState
from geometry_msgs.msg import PoseStamped
from dec_pomdp_msgs.srv import GetExecutionStatus
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
	rospy.wait_for_service('get_execution_status')
	get_execution_status = rospy.ServiceProxy('get_execution_status', GetExecutionStatus)
	rospy.wait_for_service('get_random_move_execution_status')
	get_random_move_execution_status = rospy.ServiceProxy('get_random_move_execution_status', GetExecutionStatus)
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
			#Get status of policy execution node 
			try: 
				status_response = get_execution_status()
				msg.status = status_response.status
			except rospy.ServiceException as e:
				rospy.logerr('Policy Execution status unavailable')
				msg.status = ExecutionState.ERROR
			if msg.status == ExecutionState.IDLE:
				try: 
					random_status_response = get_random_move_execution_status()
					msg.status = random_status_response.status
				except rospy.ServiceException as e:
					rospy.logerr('Random Movement Execution status unavailable')
					msg.status = ExecutionState.ERROR

			pub.publish(msg)
		except:
			e = sys.exc_info()[0]
			rospy.logwarn(str(e))
		rate.sleep()

if __name__ == '__main__':
	main()
