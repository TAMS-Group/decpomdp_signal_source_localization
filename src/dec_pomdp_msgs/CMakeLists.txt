cmake_minimum_required(VERSION 2.8.3)
project(dec_pomdp_msgs)

find_package(catkin REQUIRED COMPONENTS
   	std_msgs
   	message_generation
   	geometry_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Measurement.msg
	ExecutionState.msg
  Measurements.msg
  Edge.msg
  NodeAction.msg
  NodeTransition.msg
  Policy.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  	DEPENDENCIES
  	std_msgs
  	geometry_msgs
)


catkin_package(
	CATKIN_DEPENDS
	# message_runtime
)
