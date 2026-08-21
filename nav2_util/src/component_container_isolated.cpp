// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "nav2_util/component_manager_isolated.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  bool use_multi_threaded_executor{false};
  std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);
  for (auto & arg : args) {
    if (arg == std::string("--use_multi_threaded_executor")) {
      use_multi_threaded_executor = true;
    }
  }
  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  rclcpp::Node::SharedPtr node;
  if (use_multi_threaded_executor) {
    using ComponentManagerIsolated =
      nav2_util::ComponentManagerIsolated<rclcpp::executors::MultiThreadedExecutor>;
    node = std::make_shared<ComponentManagerIsolated>(exec);
  } else {
    using ComponentManagerIsolated =
      nav2_util::ComponentManagerIsolated<rclcpp::executors::SingleThreadedExecutor>;
    node = std::make_shared<ComponentManagerIsolated>(exec);
  }
  exec->add_node(node);
  exec->spin();
  RCLCPP_INFO(node->get_logger(), "Main executor stopped, exiting");
  // Let Fast-DDS send participant GOODBYE before the process vanishes.
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  std::fflush(stdout);
  std::fflush(stderr);
  // Humble: after rcl_shutdown, ComponentManager / composed-node destructors
  // hang or SIGSEGV (Magick abort). Skip them; the process is exiting.
  _Exit(0);
}
