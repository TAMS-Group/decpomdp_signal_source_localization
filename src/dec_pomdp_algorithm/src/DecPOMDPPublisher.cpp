/**
* Disclaimer:
* This code is partially supplied/copied from https://github.com/laurimi/npgi
*/

/*ROS related Imports*/
#include "ros/ros.h"
#include <ros/console.h>
#include "dec_pomdp_msgs/Policy.h"
#include "dec_pomdp_msgs/GeneratePolicies.h"
/*Dec POMDP Algorithm imports*/
#include <boost/program_options.hpp>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include "BackwardPassParticle.hpp"
#include "GraphSensingProblem.h"
#include "JointPolicyUtilities.h"
#include "Particle.hpp"
#include "PolicyInitialization.h"

/* Global Variables */
unsigned int RNG_SEED = 1234567890;
unsigned int HORIZON = 10;
unsigned int WIDTH = 3;
int IMPROVEMENT_STEPS = 9;

ros::Publisher decPomdpPub;

std::vector<pgi::PolicyGraph> generatePolicies(unsigned int rng_seed,
  unsigned int horizon,
  unsigned int width,
  int improvement_steps,
  int num_particles_fwd,
  int num_rollouts,
  int num_particle_rollout,
  std::size_t blind_policy_initial_joint_action,
  pgi::PolicyInitialization policy_initialization,
  bool gaussian,
  double mx,
  double my,
  double sx,
  double sy
)
{
  ROS_INFO("Generating Policies...");
  pgi::PRNG rng(rng_seed);

  // Create the GraphSensing Dec-POMDP problem
  pgi::JointActionSpace jas = pgi::GraphSensing::joint_action_space;
  pgi::GraphSensing::StateTransitionModel t;
  pgi::GraphSensing::RewardModel r;

  // RSS
  pgi::JointObservationSpace jos =
      pgi::GraphSensing::rss_joint_observation_space;
  pgi::GraphSensing::RSSObservationModel o;

  // Initialize localy policy graphs
  std::vector<pgi::PolicyGraph> local_policy_graphs;
  for (std::size_t agent = 0; agent < jas.num_local_spaces(); ++agent) {
    local_policy_graphs.emplace_back(pgi::fixed_width(
        horizon, width, jas.num_local_indices(agent),
        jos.num_local_indices(agent)));
  }

  // Determine the way the policies are supposed to be initialized
  if (policy_initialization == pgi::PolicyInitialization::random) {
    ROS_INFO("Random policy Initialization");
    pgi::set_random(local_policy_graphs, rng);
  } else if (policy_initialization == pgi::PolicyInitialization::greedy) {
    throw std::runtime_error(
        "Greedy open loop initialization not implemented for continuous-state "
        "problems!");
  } else if (policy_initialization == pgi::PolicyInitialization::blind) {
    ROS_INFO("Blind policy Initialization");
    pgi::set_random(local_policy_graphs,
                    rng);  // to randomize the edges in the policy
    pgi::set_blind(local_policy_graphs, blind_policy_initial_joint_action, jas);
  }
  // Create JointPolicy using the local policy graphs
  pgi::JointPolicy jp(local_policy_graphs);

  // Create initial belief
  pgi::ParticleSet<pgi::GraphSensing::state_t> init_particles;
  if (gaussian) {
    pgi::GraphSensing::sample_initial_states_gaussian(
        init_particles.states_, num_particles_fwd,
        pgi::GraphSensing::location_t{mx, my}, sx, sy, rng);
    ROS_INFO("Particle Filter initialized with gaussian distributed particles");
  } else {
    // creates an initial state for two agents and certain bounds
    pgi::GraphSensing::sample_initial_states(init_particles.states_,
                                             num_particles_fwd, rng);
    ROS_INFO("Particle Filter initialized with random particle locations");
  }
  init_particles.nodes_ =
      std::vector<pgi::JointPolicy::joint_vertex_t>(num_particles_fwd, jp.root());
  init_particles.weights_ = std::vector<double>(
      num_particles_fwd, 1.0 / static_cast<double>(num_particles_fwd));

  // TODO Where to go with this output
  // std::string output_prefix = "~/Documents/Bachelor/tmp_test_results/";
  // std::ofstream fvalue(output_prefix +
  //                      "policy_values.txt");
  // std::ofstream ftime(output_prefix +
  //                     "duration_microseconds.txt");

  // Get value of initial policy and write it to a log Document (The logging part could potentially be replaced by ROS Logging)
  // Also keep it to compare later created policies to this.
  double best_value = estimate_value(num_rollouts, init_particles.states_,
                                     init_particles.weights_, jp, jp.root(), t,
                                     o, r, jas, jos, rng);

  ROS_INFO_STREAM("Policy value: " << best_value);


  // TODO Where should the initial best policy and value be written to
  //for (std::size_t agent = 0; agent < jas.num_local_spaces(); ++agent) {
  //  std::ofstream fs(output_prefix +
  //                   "best_policy_agent" + std::to_string(agent) + ".dot");
  //  print(fs, jp.local_policy(agent), pgi::element_names(jas.get(agent)),
  //        pgi::element_names(jos.get(agent)));
  //}

  //std::ofstream fs(output_prefix + "best_value.txt");
  //  fs << best_value << std::endl;

  // Backward pass setup
  pgi::backpass::BackPassProperties props;
  props.prob_random_history_in_heuristic_improvement = 0.05;

  ROS_INFO_STREAM("Improving the Policies over " << improvement_steps << " Improvement Steps");
  for (int i = 1; i <= improvement_steps; ++i) {
    // sample new set of particles for improvement
    ROS_INFO_STREAM("Step "<< i << ": Resampling particles");
    if (gaussian) {
      pgi::GraphSensing::sample_initial_states_gaussian(
          init_particles.states_, num_particles_fwd,
          pgi::GraphSensing::location_t{mx, my}, sx, sy, rng);
    } else {
      pgi::GraphSensing::sample_initial_states(init_particles.states_,
                                               num_particles_fwd, rng);
    }
    init_particles.nodes_ =
        std::vector<pgi::JointPolicy::joint_vertex_t>(num_particles_fwd, jp.root());
    init_particles.weights_ = std::vector<double>(
        num_particles_fwd, 1.0 / static_cast<double>(num_particles_fwd));

    std::chrono::high_resolution_clock::time_point t1 =
        std::chrono::high_resolution_clock::now();
    auto bp = pgi::backpass::improve_particle(num_rollouts, num_particle_rollout, init_particles, jp, t, o, r, jas,
                                              jos, rng, props);
    std::chrono::high_resolution_clock::time_point t3 =
        std::chrono::high_resolution_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::microseconds>(t3 - t1).count();
    ROS_INFO_STREAM("Step "<< i << ": Improved particles in " << duration << " microseconds");

    // check the new policies value
    double improved_policy_value = estimate_value(
        num_rollouts, init_particles.states_, init_particles.weights_, bp.improved_policy,
        bp.improved_policy.root(), t, o, r, jas, jos, rng);
    ROS_INFO_STREAM("Step "<< i << ": Value of improved policy= "<< improved_policy_value);

    if (improved_policy_value > best_value) {
      best_value = improved_policy_value;
      jp = bp.improved_policy;
      ROS_INFO_STREAM("Step "<< i << ": The new joint policy is better then the old one.");
      // TODO Again how do i log this information to a File
      // Update best value and policy!
      //for (std::size_t agent = 0; agent < jas.num_local_spaces(); ++agent) {
      //   std::ofstream fs(output_prefix +
      //                    "best_policy_agent" + std::to_string(agent) + ".dot");
      //   print(fs, jp.local_policy(agent), pgi::element_names(jas.get(agent)),
      //         pgi::element_names(jos.get(agent)));
      // }
      //
      // std::ofstream fs(output_prefix + "best_value.txt");
      // fs << best_value << std::endl;
    }

    // for (std::size_t agent = 0; agent < jas.num_local_spaces(); ++agent) {
    //   std::ofstream fs(ss.str() + "agent" + std::to_string(agent) + ".dot");
    //   print(fs, bp.improved_policy.local_policy(agent),
    //         pgi::element_names(jas.get(agent)),
    //         pgi::element_names(jos.get(agent)));
    // }
  }

  pgi::ForwardPassParticle<pgi::GraphSensing::state_t> fwd(init_particles, jp,
                                                           t, o, jas, jos, rng);
  std::cout << "Forward pass results: \n";
  for (unsigned int s = 1; s <= jp.max_steps(); ++s) {
    for (auto& q : jp.joint_vertices_with_steps_remaining(s))
    {
      auto ql = jp.to_local(q);
      for (auto& qq : ql)
        std::cout << qq << " ";

      const pgi::ParticleSet<pgi::GraphSensing::state_t>& p = fwd.particles_at(q);
      std::cout << ": " << p.weights_.size() << " particles\n";
    }
  }
  return jp.local_policies();
}

dec_pomdp_msgs::Policy policyToMsg(pgi::PolicyGraph policy, std::string robot_name, int agent_start){
  dec_pomdp_msgs::Policy result;
  result.robot_name = robot_name;
  pgi::vertex_t start = pgi::find_root(policy);
  ROS_INFO_STREAM("Starting node: " << start);
  result.starting_node = agent_start;
  // Log the action space
  pgi::JointActionSpace jas = pgi::GraphSensing::joint_action_space;
  std::vector<std::string> actions = pgi::element_names(jas.get(0));
  for (auto const& action : actions){
    ROS_INFO_STREAM("Action: "<< action);
  }
  // log the joint observation space
  pgi::JointObservationSpace jos = pgi::GraphSensing::rss_joint_observation_space;
  std::vector<std::string> observations = pgi::element_names(jos.get(0));
  for (auto const& observation : observations){
    ROS_INFO_STREAM("Observation: "<< observation);
  }

  // iterate over all vertices of the policy graphs
  for (pgi::vertex_t vert : boost::make_iterator_range(boost::vertices(policy))){
    ROS_INFO_STREAM("For Vert: "<< vert << " the following out edges exist:");
    result.nodes.push_back(vert);
    dec_pomdp_msgs::NodeTransition transition;
    transition.node_number = vert;
    for (pgi::edge_t out : boost::make_iterator_range(boost::out_edges(vert, policy))){
      ROS_INFO_STREAM("Vert: " << vert << " and Edge: " << out);
      dec_pomdp_msgs::Edge edge;
      edge.node_number = boost::target(out, policy);
      // TODO insert interval for edge
      transition.edges.push_back(edge);
    }
    // TODO Insert NodeAction for node !!!
    result.transitions.push_back(transition);
  }

  return result;
}

bool generatePolicies(dec_pomdp_msgs::GeneratePolicies::Request &req,
                dec_pomdp_msgs::GeneratePolicies::Response &res)
{
  /* Insert Code to generate policy and transform it to fit policy msg */
  std::vector<pgi::PolicyGraph> policyGraphs = generatePolicies(1234567890, 3, 3, 2, 1000, 100, 100, 0, pgi::PolicyInitialization::random, false, 0.1, 0.1, 0.1, 0.1);
  /* Finish publishing policies and give response to service request */
  std::vector<dec_pomdp_msgs::Policy> result;
  // for (std::size_t agent = 0; agent < jas.num_local_spaces(); ++agent)
  for (pgi::PolicyGraph policy: policyGraphs){
    dec_pomdp_msgs::Policy policyMsg = policyToMsg(policy, "Test Name", 0);
    result.push_back(policyMsg);
    decPomdpPub.publish(policyMsg);
    ROS_WARN_STREAM("Done: " << policyMsg.robot_name);
  }
  res.policies = result;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dec_pomdp_publisher");
  ros::NodeHandle n;
  decPomdpPub = n.advertise<dec_pomdp_msgs::Policy>("dec_pomdp", 10);
  ros::ServiceServer policy_service = n.advertiseService("generate_policies", generatePolicies);
  ROS_WARN("Policy service ready to go");
  ros::spin();


  return 0;
}
