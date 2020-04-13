// GraphSensingProblem.cpp
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

#include "GraphSensingProblem.h"
#include <boost/random/normal_distribution.hpp>
#include <boost/random/uniform_01.hpp>
#include "EigenUtils.hpp"
#include "common.hpp"
/*ROS related Imports*/
#include "ros/ros.h"
#include <ros/console.h>

namespace pgi {
namespace GraphSensing {

double distance(const location_t& a, const location_t& b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

void state_t::reinvigorate(PRNG& rng) {
  // add tiny noise to the source location to reinvigorate particle
  static const double mean = 0.0;
  static const double noise_stdev = 0.25;
  boost::random::normal_distribution<> nd(mean, noise_stdev);
  source_location_.x += rng(nd);
  source_location_.y += rng(nd);
}

void sample_initial_states(std::vector<state_t>& states, int num, PRNG& rng) {
  states.resize(num);
  for (int i = 0; i < num; ++i)
    states[i] = sample_initial_state(rng);
}

// TODO For this sampling of the initial state i need more input parameters,
// to specifiy the size of the map and the given starting positions of the robots
state_t sample_initial_state(PRNG& rng)
{
  state_t s;
  s.agent_0_location_ = 0;
  s.agent_1_location_ = 0;
  boost::random::uniform_01<double> ud;
  s.source_location_.x = -20.0 + 40.0 * rng(ud);
  s.source_location_.y = -20.0 + 40.0 * rng(ud);
  return s;
}

void sample_initial_states_gaussian(std::vector<state_t>& states, int num,
                                    const location_t& source_mean_location,
                                    double stdev_x, double stdev_y, PRNG& rng) {
  states.resize(num);
  boost::random::normal_distribution<> xdist(source_mean_location.x, stdev_x);
  boost::random::normal_distribution<> ydist(source_mean_location.y, stdev_y);
  for (auto& s : states) {
    s.agent_0_location_ = 0;
    s.agent_1_location_ = 0;
    s.source_location_.x = rng(xdist);
    s.source_location_.y = rng(ydist);
  }
}

bool MovementModel::action_allowed_in(int loc, int act) const {
  const auto& allowed = allowed_next_locations_.at(loc);
  return (std::find(allowed.begin(), allowed.end(), act) != allowed.end());
}

int MovementModel::next_location(int old_loc, int act) const {
  return act;
}

double StateTransitionModel::get(const state_t& next_state,
                                 const state_t& old_state,
                                 std::size_t j_act) const {
  if ((next_state.source_location_.x != old_state.source_location_.x) ||
      (next_state.source_location_.y != old_state.source_location_.y))
    return 0.0;

  auto agent0_act = joint_action_space.local_index(j_act, 0);
  if (!moves_.action_allowed_in(old_state.agent_0_location_, agent0_act))
    return 0.0;

  if (next_state.agent_0_location_ !=
      moves_.next_location(old_state.agent_0_location_, agent0_act))
    return 0.0;

  auto agent1_act = joint_action_space.local_index(j_act, 1);
  if (!moves_.action_allowed_in(old_state.agent_1_location_, agent1_act))
    return 0.0;

  if (next_state.agent_1_location_ !=
      moves_.next_location(old_state.agent_1_location_, agent1_act))
    return 0.0;

  return 1.0;
}

state_t StateTransitionModel::sample_next_state(const state_t& state,
                                                std::size_t j_act,
                                                PRNG& rng) const {
  state_t new_state(state);

  auto agent0_act = joint_action_space.local_index(j_act, 0);
  new_state.agent_0_location_ =
      moves_.next_location(state.agent_0_location_, agent0_act);

  auto agent1_act = joint_action_space.local_index(j_act, 1);
  new_state.agent_1_location_ =
      moves_.next_location(state.agent_1_location_, agent1_act);

  return new_state;
}

double RSSObservationModel::get(std::size_t j_obs, const state_t& state,
                                std::size_t j_act) const {
  const location_t& agent0_location =
      index_to_xy_loc_.at(state.agent_0_location_);
  const auto p0 = get_observation_prob(state.source_location_, agent0_location);

  const location_t& agent1_location =
      index_to_xy_loc_.at(state.agent_1_location_);
  const auto p1 = get_observation_prob(state.source_location_, agent1_location);

  auto agent0_obs_idx = rss_joint_observation_space.local_index(j_obs, 0);
  auto agent1_obs_idx = rss_joint_observation_space.local_index(j_obs, 1);
  return p0[agent0_obs_idx] * p1[agent1_obs_idx];
}

std::size_t RSSObservationModel::sample_observation(const state_t& new_state,
                                                    std::size_t j_act,
                                                    PRNG& rng) const {
  const location_t& agent0_location =
      index_to_xy_loc_.at(new_state.agent_0_location_);
  const auto p0 = get_observation_prob(new_state.source_location_, agent0_location);

  const location_t& agent1_location =
      index_to_xy_loc_.at(new_state.agent_1_location_);
  const auto p1 = get_observation_prob(new_state.source_location_, agent1_location);

  const std::vector<double> p_obs = [&]() {
    std::vector<double> p(rss_joint_observation_space.num_joint_indices(),
                          0.0);
    for (std::size_t j_obs = 0;
         j_obs < rss_joint_observation_space.num_joint_indices();
         ++j_obs) {
      auto agent0_obs_idx =
          rss_joint_observation_space.local_index(j_obs, 0);
      auto agent1_obs_idx =
          rss_joint_observation_space.local_index(j_obs, 1);
      p[j_obs] = p0[agent0_obs_idx] * p1[agent1_obs_idx];
    }
    return p;
  }();
  boost::random::discrete_distribution<> dist(p_obs);
  return rng(dist);
}

double RSSObservationModel::rss_noise_free(const location_t& source, const location_t& agent) const
{
  const double d = distance(source, agent);
  const double free_space_loss = is_almost_zero(d) ? 0.0 :  20.0 * std::log10(d);
  // Use free space gain to model rss_noise_free
  const double free_space_gain = -(20.0 * std::log10(v) + free_space_loss -147.55);
  // Since there is no good way to determine the free variables, free_space_gain is used to model rss noise free
  // return Ptx + Gtx - Ltx + Grx - Lrx + 27.55 - 20.0 * std::log10(v) - free_space_loss;
  return free_space_gain;
}

double RSSObservationModel::rice_cdf(double loss)
{
  static std::vector<double> losses{0.0, 1.02732629683, 1.45407903973, 1.78237580677, 2.05984865643, 2.30493149027, 2.52707068423, 2.7318734613, 2.92299049674, 3.10295571708, 3.27361276256, 3.43635248753, 3.59225473965, 3.74217772375, 3.88681684812, 4.02674489739, 4.16244029938, 4.29430753316, 4.42269219375, 4.54789232942, 4.67016712021, 4.7897436219, 4.9068220776, 5.02158015199, 5.13417634371, 5.24475276284, 5.35343741198, 5.46034607526, 5.56558389459, 5.66924669414, 5.77142210055, 5.87219049601, 5.97162583379, 6.06979633964, 6.16676511802, 6.26259067848, 6.3573273947, 6.45102590628, 6.54373347202, 6.63549428143, 6.72634973043, 6.81633866621, 6.90549760524, 6.99386092807, 7.08146105373, 7.16832859644, 7.25449250664, 7.33998019826, 7.42481766389, 7.50902957917, 7.59263939756, 7.67566943671, 7.75814095718, 7.84007423432, 7.92148862419, 8.00240262388, 8.08283392699, 8.16279947466, 8.24231550254, 8.3213975842, 8.40006067117, 8.47831913008, 8.55618677698, 8.63367690926, 8.71080233526, 8.78757540185, 8.86400802005, 8.94011168898, 9.01589751823, 9.09137624866, 9.16655827198, 9.24145364901, 9.31607212683, 9.39042315485, 9.46451589996, 9.53835926067, 9.61196188051, 9.68533216065, 9.75847827172, 9.83140816506, 9.90412958327, 9.97665009518, 10.0489770057, 10.1211175139, 10.1930786224, 10.2648671699, 10.3364898392, 10.4079531641, 10.4792635367, 10.5504272135, 10.6214503223, 10.6923388675, 10.7630987363, 10.8337357039, 10.9042554386, 10.9746635072, 11.0449653794, 11.115166433, 11.1852719568, 11.2552871556, 11.3252171549, 11.3950670041, 11.4648416803, 11.5345460918, 11.604185082, 11.6737634322, 11.7432858649, 11.8127570474, 11.8821815942, 11.9515640701, 12.0209089933, 12.0902208378, 12.1595040361, 12.2287629821, 12.2980020332, 12.3672255129, 12.4364377134, 12.5056428975, 12.5748453013, 12.6440491362, 12.7132585912, 12.7824778348, 12.8517110175, 12.9209622734, 12.9902357226, 13.0595354732, 13.1288656228, 13.1982302611, 13.2676334714, 13.3370793326, 13.4065719211, 13.4761153128, 13.5457135847, 13.615370817, 13.6850910947, 13.7548785099, 13.824737163, 13.8946711649, 13.9646846391, 14.0347817229, 14.1049665696, 14.1752433507, 14.245616257, 14.316089501, 14.386667319, 14.4573539722, 14.5281537494, 14.5990709688, 14.6701099795, 14.7412751642, 14.8125709406, 14.884001764, 14.9555721291, 15.027286572, 15.0991496727, 15.1711660557, 15.243340396, 15.3156774191, 15.3881819024, 15.4608586788, 15.5337126391, 15.6067487341, 15.6799719778, 15.7533874496, 15.8270002971, 15.9008157789, 15.9748391084, 16.049075695, 16.1235309883, 16.1982105211, 16.2731199128, 16.3482648724, 16.423651202, 16.4992848008, 16.5751716683, 16.6513179081, 16.7277297325, 16.8044134658, 16.881375549, 16.958622544, 17.0361611384, 17.1139981501, 17.1921405318, 17.2705953769, 17.3493699242, 17.4284715635, 17.5079078416, 17.587686468, 17.6678153211, 17.748302455, 17.8291561059, 17.9103846991, 17.991996857, 18.0740014057, 18.156407384, 18.2392240512, 18.3224608963, 18.4061276465, 18.4902342775, 18.5747910232, 18.6598083862, 18.7452971489, 18.8312683853, 18.9177334727, 19.0047041049, 19.0921923055, 19.1802104418, 19.2687712399, 19.3578878001, 19.4475736138, 19.5378425947, 19.6287090164, 19.7201877189, 19.8122939023, 19.9050432978, 19.9984521468, 20.0925371875, 20.1873158168, 20.2828058957, 20.3790259744, 20.4759952323, 20.5737335274, 20.6722614304, 20.7716002612, 20.8717721281, 20.972799969, 21.0747075954, 21.17751974, 21.2812621064, 21.3859614233, 21.4916455022, 21.5983432985, 21.7060849784, 21.8149019891, 21.9248271359, 22.0358946635, 22.1481403446, 22.2616015753, 22.3763174779, 22.4923290115, 22.6096790929, 22.7284127264, 22.8485771449, 22.9702219641, 23.0933993495, 23.2181642004, 23.3445743388, 23.4726907377, 23.602577755, 23.7343033934, 23.8679395876, 24.0035625192, 24.141252964, 24.2810966766, 24.4231848159, 24.5676144168, 24.7144889775, 24.8639187983, 25.0160220056, 25.1709250399, 25.3287635409, 25.4896832771, 25.6538411959, 25.8214066143, 25.9925625739, 26.1675073877, 26.346456413, 26.5296440912, 26.7173263032, 26.9097831043, 27.1073219108, 27.310281235, 27.5190350833, 27.7339981673, 27.9556321123, 28.1844529212, 28.4210399341, 28.6660468561, 28.920215151, 29.1843907342, 29.4595448163, 29.7468002673, 30.0474653546, 30.3630774925, 30.6954608604, 31.0468032032, 31.4197610555, 31.817605618, 32.2444310614, 32.7054599736, 33.2075063968, 33.759705989, 34.3747238905, 35.0708753162, 35.8761430375, 36.8365999776, 38.0367738775, 39.6609922567, 42.2702648156, 53.8691298789};
  static std::vector<double> cdf{0.0, 0.00334414715778, 0.00668829432382, 0.0100324414691, 0.0133765886288, 0.016720735786, 0.0200648829431, 0.0234090301003, 0.0267531772575, 0.0300973244147, 0.0334414715719, 0.0367856187291, 0.0401297658863, 0.0434739130435, 0.0468180602007, 0.0501622073578, 0.053506354515, 0.0568505016722, 0.0601946488294, 0.0635387959865, 0.0668829431437, 0.0702270903009, 0.073571237458, 0.0769153846151, 0.0802595317722, 0.0836036789293, 0.0869478260864, 0.0902919732434, 0.0936361204004, 0.0969802675573, 0.100324414714, 0.103668561871, 0.107012709028, 0.110356856184, 0.113701003341, 0.117045150497, 0.120389297654, 0.12373344481, 0.127077591966, 0.130421739122, 0.133765886278, 0.137110033433, 0.140454180589, 0.143798327744, 0.147142474898, 0.150486622053, 0.153830769207, 0.157174916361, 0.160519063514, 0.163863210667, 0.16720735782, 0.170551504972, 0.173895652124, 0.177239799275, 0.180583946425, 0.183928093575, 0.187272240725, 0.190616387873, 0.193960535021, 0.197304682168, 0.200648829315, 0.20399297646, 0.207337123605, 0.210681270749, 0.214025417891, 0.217369565033, 0.220713712174, 0.224057859313, 0.227402006452, 0.230746153589, 0.234090300726, 0.23743444786, 0.240778594994, 0.244122742127, 0.247466889258, 0.250811036388, 0.254155183516, 0.257499330643, 0.260843477769, 0.264187624893, 0.267531772016, 0.270875920289, 0.27422006742, 0.27756421455, 0.280908361678, 0.284252508804, 0.28759665593, 0.290940803053, 0.294284950175, 0.297629097296, 0.300973244415, 0.304317391533, 0.30766153865, 0.311005685765, 0.314349832879, 0.317693979992, 0.321038127103, 0.324382274247, 0.327726421405, 0.331070568562, 0.334414715719, 0.337758862876, 0.341103010033, 0.344447157191, 0.347791304348, 0.351135451505, 0.354479598662, 0.357823745819, 0.361167892977, 0.364512040134, 0.367856187291, 0.371200334448, 0.374544481605, 0.377888628763, 0.38123277592, 0.384576923077, 0.387921070234, 0.391265217391, 0.394609364548, 0.397953511706, 0.401297658863, 0.40464180602, 0.407985953177, 0.411330100334, 0.414674247492, 0.418018394649, 0.421362541806, 0.424706688963, 0.42805083612, 0.431394983278, 0.434739130435, 0.438083277592, 0.441427424749, 0.444771571906, 0.448115719064, 0.451459866221, 0.454804013378, 0.458148160535, 0.461492307692, 0.464836454849, 0.468180602007, 0.471524749164, 0.474868896321, 0.478213043478, 0.481557190635, 0.484901337793, 0.48824548495, 0.491589632107, 0.494933779264, 0.498277926421, 0.501622073579, 0.504966220736, 0.508310367893, 0.51165451505, 0.514998662207, 0.518342809297, 0.521686956361, 0.525031103427, 0.528375250495, 0.531719397564, 0.535063544635, 0.538407691709, 0.541751838784, 0.545095985862, 0.548440132943, 0.551784281825, 0.555128428914, 0.558472576006, 0.561816723101, 0.565160870199, 0.5685050173, 0.571849164404, 0.575193311511, 0.578537458621, 0.581881605734, 0.58522575285, 0.588569899969, 0.591914047091, 0.595258194216, 0.598602341343, 0.601946488473, 0.605290635606, 0.608634782741, 0.611978929879, 0.615323077019, 0.61866722416, 0.622011371304, 0.62535551845, 0.628699665597, 0.632043812745, 0.635387959895, 0.638732107046, 0.642076254198, 0.645420401351, 0.648764548505, 0.65210869566, 0.655452842815, 0.65879698997, 0.662141137126, 0.665485284283, 0.668829431439, 0.672173578596, 0.675517725753, 0.67886187291, 0.682206020067, 0.685550167224, 0.688894314381, 0.692238461538, 0.695582608696, 0.698926755853, 0.702270903547, 0.70561504985, 0.708959197289, 0.712303344481, 0.715647491643, 0.718991638976, 0.722335785344, 0.72567993311, 0.729024080268, 0.732368227425, 0.735712374582, 0.739056521739, 0.742400668896, 0.745744816053, 0.749088963209, 0.752433110365, 0.75577725752, 0.759121404674, 0.762465551827, 0.765809698977, 0.769153846126, 0.772497993272, 0.775842140415, 0.779186287554, 0.782530434689, 0.78587458182, 0.789218728946, 0.792562876067, 0.795907023183, 0.799251170294, 0.8025953174, 0.805939464502, 0.809283611601, 0.812627758697, 0.81597190581, 0.819316053021, 0.82266020022, 0.826004347406, 0.829348494577, 0.832692641734, 0.836036788876, 0.839380936005, 0.84272508312, 0.846069230224, 0.849413377318, 0.852757524406, 0.856101672884, 0.859445819951, 0.862789967019, 0.866134114091, 0.869478261169, 0.872822408256, 0.876166555353, 0.87951070246, 0.882854849577, 0.886198996705, 0.889543143841, 0.892887290985, 0.896231438134, 0.899575585287, 0.902919732442, 0.906263879599, 0.909608026756, 0.912952173913, 0.916296321046, 0.919640468302, 0.922984615385, 0.926328762542, 0.929672909698, 0.93301705685, 0.936361203995, 0.939705351126, 0.94304949824, 0.94639364534, 0.949737792956, 0.953081940103, 0.956426087201, 0.959770234271, 0.963114381346, 0.966458528451, 0.969802675588, 0.973146822743, 0.9764909699, 0.979835117057, 0.983179264189, 0.986523411371, 0.989867558528, 0.993211705686, 0.996555852843, 0.999899999998};

  if (loss <= 0.0)
    return 0.0;
  if (loss >= losses.back())
    return 1.0;

  // linear interpolation
  auto l_above = std::upper_bound(losses.begin(), losses.end(), loss);
  auto l_below = std::prev(l_above);

  auto c_below = std::next(cdf.begin(), std::distance(losses.begin(), l_below));
  auto c_above = std::next(c_below);

  const double fraction = (loss - *l_below) / (*l_above - *l_below);
  return (*c_below * (1.0 - fraction)) + (*c_above * fraction);
}

std::array<double, 3> RSSObservationModel::get_observation_prob(
    const location_t& source, const location_t& agent) const {
  const double rss = rss_noise_free(source, agent);

  std::array<double, 3> probabilities;
  probabilities[0] = rice_cdf(rss - high_threshold); // high
  probabilities[2] = 1.0 - rice_cdf(rss - low_threshold); // low
  probabilities[1] = (1.0 - probabilities[2]) - probabilities[0]; // mid

  return probabilities;
}

double RewardModel::get(const std::vector<state_t>& states,
                        const std::vector<double>& weights,
                        std::size_t j_act) const {
  ROS_INFO_STREAM("Reward model.get gets called");
  ROS_INFO_STREAM("jas length is " << joint_action_space.num_local_spaces());
  const auto agent0_act = joint_action_space.local_index(j_act, 0);
  const auto agent1_act = joint_action_space.local_index(j_act, 1);

  double reward(0.0);
  for (std::size_t i = 0; i < states.size(); ++i) {
    const state_t& s = states[i];
    ROS_INFO_STREAM("Current state a0 loc = "<< s.agent_0_location_ << " a1 loc =" << s.agent_1_location_ << " source loc x: " << s.source_location_.x << " y: " << s.source_location_.y);
    if (!moves_.action_allowed_in(s.agent_0_location_, agent0_act))
      reward += weights[i] * (-INVALID_MOVE_PENALTY);
    if (!moves_.action_allowed_in(s.agent_1_location_, agent1_act))
      reward += weights[i] * (-INVALID_MOVE_PENALTY);
  }
  return reward;
}

double RewardModel::final_reward(const std::vector<state_t>& states,
                                 const std::vector<double>& weights) const {
  Eigen::Matrix<double, 2, Eigen::Dynamic> xy(2, states.size());
  for (std::size_t i = 0; i < states.size(); ++i) {
    xy(0, i) = states[i].source_location_.x;
    xy(1, i) = states[i].source_location_.y;
  }

  Eigen::Map<const Eigen::VectorXd> w(weights.data(), weights.size());
  Eigen::MatrixXd weighted_cov;
  pgi::detail::weighted_cov(xy, w, weighted_cov);
  double det = weighted_cov.determinant();

  if ((det <= 0.0) || (std::isnan(det)))
  {
    return 50.0;
  }

  if (std::isnan(det)) // probably because we had only one non-zero weight
    return 50.0;

  return -std::log(det);
}
}  // namespace GraphSensing

}  // namespace pgi
