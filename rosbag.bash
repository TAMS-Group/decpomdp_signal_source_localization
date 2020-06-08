#!/bin/bash
rosbag record \
  /dec_pomdp \
  /donny/heartbeat \
  /donny/measurements \
  /donny/policy \
  /heartbeat \
  /leo/heartbeat \
  /leo/measurements \
  /leo/policy \
  /map \
  /map_metadata \
  /map_navigation \
  /map_updates \
  /measurement_locations \
  /measurement_locations_array \
  /measurements \
  /mikey/heartbeat \
  /mikey/measurements \
  /movement_graph_edges \
  /movement_graph_edges_array \
  /movement_graph_nodes \
  /movement_graph_nodes_array \
  /raph/heartbeat \
  /raph/measurements \
  /robotState \
  /robotState_array \
  /rosout \
  /rosout_agg \
  /source_location \
  /source_location_array \
  /tf \
  /tf_static
