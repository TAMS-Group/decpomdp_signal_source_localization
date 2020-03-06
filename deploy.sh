#!/bin/bash

robot=leo
robot_user=tkrueger

workspace=~/bachelor/src/

rsync -ar "$workspace" $robot_user@$robot:bachelor/src &&
ssh $robot_user@$robot -t "bash -i -c 'cd ~/bachelor && catkin build dec_pomdp_client'"
