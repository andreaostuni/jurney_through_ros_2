// Copyright 2023 andrea_ostuni. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

#ifndef LIFECYCLE_EXAMPLE__MANAGER_NODE_HPP_
#define LIFECYCLE_EXAMPLE__MANAGER_NODE_HPP_

#include "lifecycle_example/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_example/managed_node.hpp"

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace lifecycle_example
{
using ChangeState = lifecycle_msgs::srv::ChangeState;
using GetState = lifecycle_msgs::srv::GetState;
using Trigger = std_srvs::srv::Trigger;
using ChangeStateClient = rclcpp::Client<ChangeState>;

class ManagerNode : public rclcpp::Node 
{
public:
  explicit ManagerNode(const rclcpp::NodeOptions & options);

  protected:

  // Trigger service 

  rclcpp::Service<Trigger>::SharedPtr trigger_activation_;
  rclcpp::Service<Trigger>::SharedPtr trigger_deactivation_;

  // Callbacks
  void activate_callback(const Trigger::Request::SharedPtr request,
    Trigger::Response::SharedPtr response);
  void deactivate_callback(const Trigger::Request::SharedPtr request,
    Trigger::Response::SharedPtr response);

  /// Callback group for services
  rclcpp::CallbackGroup::SharedPtr callback_group_services_;
  
  // Nodes managed by this manager
  std::vector<std::string> managed_nodes_;

  // Lifecycle clients to change state of managed nodes
  std::map<std::string, std::shared_ptr<ChangeStateClient>> change_state_clients_;
  
};

}  // namespace lifecycle_example

#endif  // LIFECYCLE_EXAMPLE__MANAGER_NODE_HPP_
