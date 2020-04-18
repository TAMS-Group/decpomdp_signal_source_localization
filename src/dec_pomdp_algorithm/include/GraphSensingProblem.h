// GraphSensingProblem.h
// Copyright 2019 Mikko Lauri
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef GRAPHSENSINGPROBLEM_H
#define GRAPHSENSINGPROBLEM_H
#include <boost/random/discrete_distribution.hpp>
#include <boost/random/normal_distribution.hpp>
#include <map>
#include <fstream>
#include <iostream>
#include <sstream>
#include "DiscreteAbstractions.hpp"
#include "ObservationModel.hpp"
#include "PRNG.h"
#include "RewardModel.hpp"
#include "StateTransitionModel.hpp"

#include <ros/console.h>
#include "dec_pomdp_msgs/Interval.h"


namespace pgi {
namespace GraphSensing {
struct location_t {
  double x;
  double y;
};

double distance(const location_t& a, const location_t& b);

struct state_t {
  location_t source_location_;
  int agent_0_location_;
  int agent_1_location_;

  void reinvigorate(PRNG& rng);
};

void sample_initial_states(std::vector<state_t>& states, int num, PRNG& rng);
state_t sample_initial_state(PRNG& rng);
void sample_initial_states_gaussian(std::vector<state_t>& states, int num,
                                    const location_t& source_mean_location,
                                    double stdev_x, double stdev_y, PRNG& rng);

//Default locations and allowed moves. Should be replaced during execution time
static std::map<int, location_t> index_to_loc = []() {
  std::map<int, location_t> loc_map;

  std::ifstream inputfile("/home/tobias/Documents/Bachelor_Thesis_Code/src/dec_pomdp_algorithm/config/locations.txt");
  std::string line;
  if(!inputfile.is_open()){
    ROS_ERROR_STREAM("locations.txt file could not be opened");
  }
  while (std::getline(inputfile, line)) {
    std::istringstream ss(line);

    int id;
    double x, y;
    ss >> id >> x >> y;
    ROS_INFO_STREAM("Got location id:"<< id << " x:"<< x << " y:"<< y);
    loc_map.emplace(id, location_t{x, y});
  }
  return loc_map;
}();
static std::map<int, std::vector<int>> allowed_moves= []() {
  std::map<int, std::vector<int>> allowed_moves_map;

  std::ifstream inputfile("/home/tobias/Documents/Bachelor_Thesis_Code/src/dec_pomdp_algorithm/config/allowed_moves.txt");
  std::string line;
  if(!inputfile.is_open()){
    ROS_ERROR_STREAM("allowed_moves.txt file could not be opened");
  }
  while (std::getline(inputfile, line)) {
    std::istringstream ss(line);
    int id;
    ss >> id;
    std::vector<int> possible_destinations;
    int number;
    while(ss >> number){
      possible_destinations.push_back(number);
      ROS_INFO_STREAM("Got possible move id:"<< id << " destination:"<< number);
    }
    allowed_moves_map.emplace(id, possible_destinations);
  }
  return allowed_moves_map;
}();

static void set_locations(std::map<int, location_t> locations){
  index_to_loc = locations;
}

static void set_allowed_moves(std::map<int, std::vector<int>> moves){
  allowed_moves = moves;
}

//initialize Joint action space as empty
static JointActionSpace joint_action_space = []() {
  std::vector<DiscreteAction> local_actions;
  for (std::size_t i = 0; i < index_to_loc.size(); ++i) {
    local_actions.emplace_back(DiscreteAction("goto" + std::to_string(i)));
  }
  std::vector<ActionSpace> a;
  a.emplace_back(ActionSpace(local_actions));
  a.emplace_back(ActionSpace(local_actions));
  return JointActionSpace(a);
}();

static void set_joint_action_space(std::map<int, location_t> index_to_loc_map){
  std::vector<DiscreteAction> local_actions;
  ROS_INFO_STREAM("Gets Called");
  for (std::size_t i = 0; i < index_to_loc_map.size(); ++i) {
    ROS_INFO_STREAM("Do shit for " << i );
    local_actions.emplace_back(DiscreteAction("goto" + std::to_string(i)));
  }
  ROS_INFO_STREAM("Get action space vector");
  std::vector<ActionSpace> a;
  a.emplace_back(ActionSpace(local_actions));
  a.emplace_back(ActionSpace(local_actions));
  ROS_INFO_STREAM("created Actionspace vector");
  joint_action_space = JointActionSpace(a);
}

static const JointObservationSpace rss_joint_observation_space = []() {
  std::vector<DiscreteObservation> local_observations{
      DiscreteObservation("high"), DiscreteObservation("mid"),
      DiscreteObservation("low")};
  return JointObservationSpace(
      std::vector<ObservationSpace>(2, local_observations));
}();


class MovementModel {
 public:
  MovementModel() : allowed_next_locations_(allowed_moves) {}
  bool action_allowed_in(int loc, int act) const;
  int next_location(int old_loc, int act) const;

 private:
  const std::map<int, std::vector<int>> allowed_next_locations_;
};

class StateTransitionModel : public pgi::StateTransitionModel<state_t> {
 public:
  double get(const state_t& next_state, const state_t& old_state,
             std::size_t j_act) const;
  state_t sample_next_state(const state_t& state, std::size_t j_act,
                            PRNG& rng) const;

 private:
  const MovementModel moves_;
};

class RSSObservationModel : public pgi::ObservationModel<state_t> {
 public:
  RSSObservationModel() : index_to_xy_loc_(index_to_loc) {}
  double get(std::size_t j_obs, const state_t& state, std::size_t j_act) const;
  std::size_t sample_observation(const state_t& new_state, std::size_t j_act,
                                 PRNG& rng) const;


  double rss_noise_free(const location_t& source, const location_t& agent) const;
  static double rice_cdf(double loss);
  std::array<double, 3> get_observation_prob(const location_t& source,
                                             const location_t& agent) const;
  static dec_pomdp_msgs::Interval getIntervalForObservation(std::string observation) {
    dec_pomdp_msgs::Interval result; 
    std::vector<std::string> observations = pgi::element_names(rss_joint_observation_space.get(0));
    if(observation == observations[0]){
      result.upper_bound = top_threshold;
      result.lower_bound = high_threshold;
    } else if(observation == observations[1]){
      result.upper_bound = high_threshold;
      result.lower_bound = low_threshold;
    } else if(observation == observations[2]){
      result.upper_bound = low_threshold;
      result.lower_bound = bottom_threshold;
    }
    return result;
  }
private:
  const std::map<int, location_t> index_to_xy_loc_;

  // Signal model parameters
  // const double Ptx = 18.0;
  // const double Gtx = 1.5;
  // const double Ltx = 0.0;
  // const double Grx = 1.5;
  // const double Lrx = 0.0;
  const double v = 2.4e9;
  // Partition model
  static constexpr double top_threshold = 0.0;
  static constexpr double high_threshold = -55.0;
  static constexpr double low_threshold = -65.0;
  static constexpr double bottom_threshold = -160.0;
};

class RewardModel : public pgi::RewardModel<state_t> {
 public:
  double get(const std::vector<state_t>& states,
             const std::vector<double>& weights, std::size_t j_act) const;
  double final_reward(const std::vector<state_t>& states,
                      const std::vector<double>& weights) const;

 private:
  const MovementModel moves_;
  static constexpr double INVALID_MOVE_PENALTY{100.0};
};
}  // namespace GraphSensing

}  // namespace pgi

#endif  // GRAPHSENSINGPROBLEM_H
