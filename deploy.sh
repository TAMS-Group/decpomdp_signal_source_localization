#!/bin/bash

robot_1=leo
robot_2=donny
robot_user=tkrueger

workspace=~/bachelor/src/

rsync -ar "$workspace" $robot_user@$robot_1:bachelor/src &&
ssh $robot_user@$robot_1 -t "bash -i -c 'cd ~/bachelor && catkin build dec_pomdp_client dec_pomdp_msgs'"

rsync -ar "$workspace" $robot_user@$robot_2:bachelor/src &&
ssh $robot_user@$robot_2 -t "bash -i -c 'cd ~/bachelor && catkin build dec_pomdp_client dec_pomdp_msgs'"
