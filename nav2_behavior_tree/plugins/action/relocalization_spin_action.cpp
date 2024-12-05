// Copyright (c) 2018 Intel Corporation
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

#include <string>
#include <memory>

#include "nav2_behavior_tree/plugins/action/relocalization_spin_action.hpp"

namespace nav2_behavior_tree
{

RelocalizationSpinAction::RelocalizationSpinAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<capella_ros_msg::action::RelocalizationSpin>(xml_tag_name, action_name, conf), localization_score_(1.0)
{

}
void RelocalizationSpinAction::on_tick()
{
  
  getInput("localization_score", localization_score_);
  goal_.target = localization_score_;
}
}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::RelocalizationSpinAction>(name, "relocalization_spin", config);
    };

  factory.registerBuilder<nav2_behavior_tree::RelocalizationSpinAction>("RelocalizationSpin", builder);
}
