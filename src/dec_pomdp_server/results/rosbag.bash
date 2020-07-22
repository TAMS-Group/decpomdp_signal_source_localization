#!/bin/bash
rosbag record \
-O RandomMovementAttempt3-V1.bag \
/dec_pomdp \
/donny/amcl_pose \
/donny/execute_policy/cancel \
/donny/execute_policy/feedback \
/donny/execute_policy/goal \
/donny/execute_policy/result \
/donny/execute_policy/status \
/donny/execute_random_movement/cancel \
/donny/execute_random_movement/feedback \
/donny/execute_random_movement/goal \
/donny/execute_random_movement/result \
/donny/execute_random_movement/status \
/donny/heartbeat \
/donny/map \
/donny/map_metadata \
/donny/map_navigation \
/donny/measurements \
/donny/measurements/cancel \
/donny/measurements/feedback \
/donny/measurements/goal \
/donny/measurements/result \
/donny/measurements/status \
/donny/move_base/cancel \
/donny/move_base/feedback \
/donny/move_base/goal \
/donny/move_base/result \
/donny/move_base/status \
/donny/move_base_simple/goal \
/donny/policy \
/donny/possible_moves \
/donny/tf \
/donny/tf_static \
/heartbeat \
/leo/amcl_pose \
/leo/execute_policy/cancel \
/leo/execute_policy/feedback \
/leo/execute_policy/goal \
/leo/execute_policy/result \
/leo/execute_policy/status \
/leo/execute_random_movement/cancel \
/leo/execute_random_movement/feedback \
/leo/execute_random_movement/goal \
/leo/execute_random_movement/result \
/leo/execute_random_movement/status \
/leo/heartbeat \
/leo/joint_states \
/leo/map \
/leo/map_metadata \
/leo/map_navigation \
/leo/measurements \
/leo/measurements/cancel \
/leo/measurements/feedback \
/leo/measurements/goal \
/leo/measurements/result \
/leo/measurements/status \
/leo/move_base/cancel \
/leo/move_base/feedback \
/leo/move_base/goal \
/leo/move_base/result \
/leo/move_base/status \
/leo/move_base_simple/goal \
/leo/policy \
/leo/possible_moves \
/leo/tf \
/leo/tf_static \
/map \
/map_metadata \
/map_navigation \
/measurement_locations \
/measurements \
/measurements_evaluation/cancel \
/measurements_evaluation/feedback \
/measurements_evaluation/goal \
/measurements_evaluation/result \
/measurements_evaluation/status \
/movement_graph_edges \
/movement_graph_labels \
/movement_graph_nodes \
/particles \
/robotPositon \
/robotState \
/rosout \
/rosout_agg \
/source_location 