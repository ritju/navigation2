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

#ifndef NAV2_UTIL__COMPONENT_MANAGER_ISOLATED_HPP_
#define NAV2_UTIL__COMPONENT_MANAGER_ISOLATED_HPP_

#include <cstdlib>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>

#include "rclcpp_components/component_manager.hpp"

namespace nav2_util
{
/// Humble overlay of ComponentManagerIsolated.
///
/// Do not cancel/join/destroy composed nodes from Context::shutdown()
/// pre_shutdown: Humble holds init_mutex_ and pre_shutdown_callbacks_mutex_
/// for the whole callback list. executor->cancel() from that callback deadlocks.
/// After spin() returns, skip ROS/DDS destructors (_Exit) — they hang or SIGSEGV.
template<typename ExecutorT = rclcpp::executors::SingleThreadedExecutor>
class ComponentManagerIsolated : public rclcpp_components::ComponentManager
{
  struct DedicatedExecutorWrapper
  {
    std::shared_ptr<rclcpp::Executor> executor;
    std::thread thread;
  };

public:
  explicit ComponentManagerIsolated(
    std::weak_ptr<rclcpp::Executor> executor =
    std::weak_ptr<rclcpp::executors::MultiThreadedExecutor>(),
    std::string node_name = "ComponentManager",
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions()
    .start_parameter_services(false)
    .start_parameter_event_publisher(false))
  : rclcpp_components::ComponentManager(executor, node_name, node_options)
  {
  }

  ~ComponentManagerIsolated() override
  {
    _Exit(0);
  }

protected:
  void
  add_node_to_executor(uint64_t node_id) override
  {
    DedicatedExecutorWrapper executor_wrapper;
    auto exec = std::make_shared<ExecutorT>();
    exec->add_node(node_wrappers_[node_id].get_node_base_interface());
    executor_wrapper.executor = exec;
    executor_wrapper.thread = std::thread(
      [exec]() {
        exec->spin();
      });
    std::lock_guard<std::mutex> lock(wrappers_mutex_);
    dedicated_executor_wrappers_[node_id] = std::move(executor_wrapper);
  }

  void
  remove_node_from_executor(uint64_t node_id) override
  {
    std::lock_guard<std::mutex> lock(wrappers_mutex_);
    auto executor_wrapper = dedicated_executor_wrappers_.find(node_id);
    if (executor_wrapper == dedicated_executor_wrappers_.end()) {
      return;
    }
    if (executor_wrapper->second.executor) {
      auto context = this->get_node_base_interface()->get_context();
      while (!executor_wrapper->second.executor->is_spinning() && rclcpp::ok(context)) {
        rclcpp::sleep_for(std::chrono::milliseconds(1));
      }
      executor_wrapper->second.executor->cancel();
    }
    if (executor_wrapper->second.thread.joinable()) {
      executor_wrapper->second.thread.join();
    }
    dedicated_executor_wrappers_.erase(executor_wrapper);
  }

private:
  std::mutex wrappers_mutex_;
  std::unordered_map<uint64_t, DedicatedExecutorWrapper> dedicated_executor_wrappers_;
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__COMPONENT_MANAGER_ISOLATED_HPP_
