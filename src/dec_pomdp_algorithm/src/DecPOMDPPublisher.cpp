/*ROS related Imports*/
#include "ros/ros.h"
#include "dec_pomdp_msgs/Policy.h"
#include "dec_pomdp_msgs/PublishPolicies.h"
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
  pgi::PRNG rng(rng_seed);

  // Create the GraphSensing Dec-POMDP problem
  pgi::JointActionSpace jas = pgi::GraphSensing::joint_action_space;
  pgi::GraphSensing::StateTransitionModel t;
  pgi::GraphSensing::RewardModel r;

  // RSS
  pgi::JointObservationSpace jos =
      pgi::GraphSensing::rss_joint_observation_space;
  pgi::GraphSensing::RSSObservationModel o;

  // Initialize policy
  std::vector<pgi::PolicyGraph> local_policy_graphs;
  for (std::size_t agent = 0; agent < jas.num_local_spaces(); ++agent) {
    local_policy_graphs.emplace_back(pgi::fixed_width(
        horizon, width, jas.num_local_indices(agent),
        jos.num_local_indices(agent)));
  }

  if (policy_initialization == pgi::PolicyInitialization::random) {
    pgi::set_random(local_policy_graphs, rng);
  } else if (policy_initialization == pgi::PolicyInitialization::greedy) {
    throw std::runtime_error(
        "Greedy open loop initialization not implemented for continuous-state "
        "problems!");
  } else if (policy_initialization == pgi::PolicyInitialization::blind) {
    pgi::set_random(local_policy_graphs,
                    rng);  // to randomize the edges in the policy
    pgi::set_blind(local_policy_graphs, blind_policy_initial_joint_action, jas);
  }
  pgi::JointPolicy jp(local_policy_graphs);

  // Create initial belief
  pgi::ParticleSet<pgi::GraphSensing::state_t> init_particles;
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

  // TODO Where to go with this output
  std::string output_prefix = "~/Documents/Bachelor/tmp_test_results/";
  std::ofstream fvalue(output_prefix +
                       "policy_values.txt");
  std::ofstream ftime(output_prefix +
                      "duration_microseconds.txt");

  // Get value of initial policy
  double best_value = estimate_value(num_rollouts, init_particles.states_,
                                     init_particles.weights_, jp, jp.root(), t,
                                     o, r, jas, jos, rng);

  fvalue << best_value << std::endl;
  std::cout << "Policy value: " << best_value << std::endl;

  // TODO Where should the initial best policy and value be written to
  for (std::size_t agent = 0; agent < jas.num_local_spaces(); ++agent) {
    std::ofstream fs(output_prefix +
                     "best_policy_agent" + std::to_string(agent) + ".dot");
    print(fs, jp.local_policy(agent), pgi::element_names(jas.get(agent)),
          pgi::element_names(jos.get(agent)));
  }

  std::ofstream fs(output_prefix + "best_value.txt");
  fs << best_value << std::endl;

  // Backward pass setup
  pgi::backpass::BackPassProperties props;
  props.prob_random_history_in_heuristic_improvement = 0.05;

  for (int i = 1; i <= improvement_steps; ++i) {

    // sample new set of particles for improvement
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
    // ftime << duration << std::endl;

    // check
    double improved_policy_value = estimate_value(
        num_rollouts, init_particles.states_, init_particles.weights_, bp.improved_policy,
        bp.improved_policy.root(), t, o, r, jas, jos, rng);

    // fvalue << improved_policy_value << std::endl;

    if (improved_policy_value > best_value) {
      best_value = improved_policy_value;
      jp = bp.improved_policy;

      // TODO Again where to put this information
      // Update best value and policy!
      for (std::size_t agent = 0; agent < jas.num_local_spaces(); ++agent) {
        std::ofstream fs(output_prefix +
                         "best_policy_agent" + std::to_string(agent) + ".dot");
        print(fs, jp.local_policy(agent), pgi::element_names(jas.get(agent)),
              pgi::element_names(jos.get(agent)));
      }

      std::ofstream fs(output_prefix + "best_value.txt");
      fs << best_value << std::endl;
    }
    // TODO where schould this be written to ???
    std::cout << "Step " << i << " of " << improvement_steps << ": "
              << improved_policy_value << " (best: " << best_value << ")"
              << ", " << duration << " microseconds" << std::endl;

    std::ostringstream ss;
    ss << output_prefix << "/step" << std::setw(3)
       << std::setfill('0') << i << "_";

    for (std::size_t agent = 0; agent < jas.num_local_spaces(); ++agent) {
      std::ofstream fs(ss.str() + "agent" + std::to_string(agent) + ".dot");
      print(fs, bp.improved_policy.local_policy(agent),
            pgi::element_names(jas.get(agent)),
            pgi::element_names(jos.get(agent)));
    }
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

  return local_policy_graphs;
}

bool publishPolicies(dec_pomdp_msgs::PublishPolicies::Request &req,
                dec_pomdp_msgs::PublishPolicies::Response &res)
{
  dec_pomdp_msgs::Policy policy;

  /* Insert Code to generate policy and transform it to fit policy msg */
  std::vector<pgi::PolicyGraph> policyGraphs = generatePolicies(1234567890, 3, 3, 9, 1000, 100, 100, 0, pgi::PolicyInitialization::random, false, 0.1, 0.1, 0.1, 0.1);
  /* Finish publishing policies and give response to service request */
  decPomdpPub.publish(policy);

  res.successful = false;
  for (auto i = policyGraphs.begin(); i != policyGraphs.end(); ++i){
    ROS_WARN("Done");
  }
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
