// Copyright (c) 2026 Capella
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

#include <gtest/gtest.h>
#include <memory>
#include <string>

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"

#include "nav2_behavior_tree/plugins/action/clear_path_action.hpp"

class ClearPathTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("clear_path_test_fixture");
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();

    config_ = new BT::NodeConfiguration();
    config_->blackboard = BT::Blackboard::create();
    config_->blackboard->set<rclcpp::Node::SharedPtr>("node", node_);

    factory_->registerNodeType<nav2_behavior_tree::ClearPath>("ClearPath");
  }

  static void TearDownTestCase()
  {
    delete config_;
    config_ = nullptr;
    node_.reset();
    factory_.reset();
  }

  void TearDown() override
  {
    tree_.reset();
  }

protected:
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

rclcpp::Node::SharedPtr ClearPathTestFixture::node_ = nullptr;
BT::NodeConfiguration * ClearPathTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> ClearPathTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> ClearPathTestFixture::tree_ = nullptr;

TEST_F(ClearPathTestFixture, test_tick_clears_poses)
{
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
          <ClearPath input_path="{input_path}" output_path="{output_path}"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = 1.0;
  path.poses.push_back(pose);
  path.poses.push_back(pose);
  config_->blackboard->set("input_path", path);

  EXPECT_EQ(tree_->rootNode()->executeTick(), BT::NodeStatus::SUCCESS);

  nav_msgs::msg::Path cleared;
  config_->blackboard->get("path", cleared);
  EXPECT_TRUE(cleared.poses.empty());
  EXPECT_EQ(cleared.header.frame_id, "map");
}

TEST_F(ClearPathTestFixture, test_tick_missing_path)
{
  std::string xml_txt =
    R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
          <ClearPath input_path="{input_path}" output_path="{output_path}"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  EXPECT_EQ(tree_->rootNode()->executeTick(), BT::NodeStatus::SUCCESS);

  nav_msgs::msg::Path cleared;
  config_->blackboard->get("missing_path", cleared);
  EXPECT_TRUE(cleared.poses.empty());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int all_successful = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return all_successful;
}
