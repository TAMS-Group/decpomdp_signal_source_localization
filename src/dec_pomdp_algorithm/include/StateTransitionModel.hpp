// StateTransitionModel.hpp
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

#ifndef STATETRANSITIONMODEL_HPP
#define STATETRANSITIONMODEL_HPP
#include <cstddef>
#include "NPGICRTP.hpp"
#include "PRNG.h"

namespace pgi {
template <typename State>
class StateTransitionModel : crtp<State> {
 public:
  double get(const State& next_state, const State& old_state,
             std::size_t j_act) const {
    return this->underlying().get(next_state, old_state, j_act);
  }

  State sample_next_state(const State& state, std::size_t j_act,
                          PRNG& rng) const {
    return this->underlying().sample_next_state(state, j_act, rng);
  }
};

}  // namespace pgi

#endif  // STATETRANSITIONMODEL_HPP
