/*ROS related Imports*/
#include "ros/ros.h"
#include "dec_pomdp_msgs/Policy.h"
#include "dec_pomdp_msgs/PublishPolicies.h"
/*Dec POMDP Algorithm imports*/
#include "BackwardPass.h"
#include "Belief.hpp"
#include "DecPOMDPDiscrete.h"
#include "HistoryCache.hpp"
#include "JointPolicy.h"
#include "JointPolicyUtilities.h"
#include "MADPWrapper.h"
#include "MADPWrapperUtils.h"
#include "PRNG.h"
#include "PolicyInitialization.h"
#include "ValueFunction.h"

#include <boost/program_options.hpp>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

/* Global Variables */
unsigned int RNG_SEED = 1234567890;
unsigned int HORIZON = 10;
unsigned int WIDTH = 3;
int IMPROVEMENT_STEPS = 9;

ros::Publisher decPomdpPub;

bool publishPolicies(dec_pomdp_msgs::PublishPolicies::Request &req,
                dec_pomdp_msgs::PublishPolicies::Response &res)
{
  dec_pomdp_msgs::Policy policy;
  decPomdpPub.publish(policy);
  res.successful = false;
  ROS_WARN("Request: number = %ld", (long int)req.number_of_agents);
  return true;
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "dec_pomdp_publisher");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;
  decPomdpPub = n.advertise<dec_pomdp_msgs::Policy>("dec_pomdp", 10);
  ros::ServiceServer policy_service = n.advertiseService("publish_policies", publishPolicies);
  ROS_WARN("Policy service ready to go");
  ros::spin();


  return 0;
}
