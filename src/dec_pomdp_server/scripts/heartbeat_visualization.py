#!/usr/bin/env python
import rospy
from dec_pomdp_msgs.msg import ExecutionState
from visualization_msgs.msg import Marker

#marker = Marker(
 #               type=Marker.TEXT_VIEW_FACING,
  #              id=0,
   #             lifetime=rospy.Duration(1.5),
    #            scale=Vector3(0, 0, 0.06),
     #           header=Header(frame_id='map'),
      #          color=ColorRGBA(0.0, 1.0, 0.0, 0.8)
       #     )

def visualize(state):
	rospy.loginfo("%s has state %s and is currently at x = %d, y = %d, z=%d" % (state.robot_name, state.status, state.pose.pose.position.x,state.pose.pose.position.y,state.pose.pose.position.z))
	# marker.pose = state.pose.pose
	# marker.text = state.robot_name + 

def main():
	rospy.init_node('heartbeat_visualization')
	rospy.Subscriber('heartbeat', ExecutionState, visualize)

	rospy.spin()

if __name__ == '__main__':
	main()