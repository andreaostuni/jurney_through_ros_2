// Copyright 2023 andrea_ostuni. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.


#include "lifecycle_example/manager_node.hpp"
#include <chrono>

namespace lifecycle_example
{

ManagerNode::ManagerNode(const rclcpp::NodeOptions & options):
  rclcpp::Node("manager_node", options)
{
    RCLCPP_INFO(this->get_logger(), "Creating ManagerNode");

    declare_parameter("managed_nodes", std::vector<std::string>{"managed_node_1", "managed_node_2"});
    managed_nodes_ = this->get_parameter("managed_nodes").as_string_array();

    for (auto & managed_node : managed_nodes_)
    {
      change_state_clients_[managed_node] = this->create_client<ChangeState>(managed_node + "/change_state");
    }
    callback_group_services_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    trigger_activation_ = this->create_service<Trigger>("activate",
        std::bind(&ManagerNode::activate_callback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_services_);
    trigger_deactivation_ = this->create_service<Trigger>("deactivate",
        std::bind(&ManagerNode::deactivate_callback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_services_);
    
}

void ManagerNode::activate_callback(const Trigger::Request::SharedPtr request,
  Trigger::Response::SharedPtr response)
{
  RCLCPP_INFO(this->get_logger(), "Activating ManagerNode");
  // Waiting services availability
  response->success = false;
  for (auto & managed_node : managed_nodes_)
  {
    while (!change_state_clients_[managed_node]->wait_for_service(std::chrono::seconds(1)) && rclcpp::ok())
    {
    RCLCPP_INFO(this->get_logger(), "Waiting for service for activating %s", managed_node.c_str());
    }  
  }

  // lambda to handle the waiting for result
  auto wait_for_result = [this](auto & result_future, const std::string & node_name) {
      while (result_future.wait_for(std::chrono::seconds(1))!= std::future_status::ready && rclcpp::ok())
      {
      RCLCPP_INFO(this->get_logger(), "Waiting for transition result for %s", node_name.c_str());
      }
  };

  auto change_state_request = std::make_shared<ChangeState::Request>();
  change_state_request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;
  
  for (auto & managed_node : managed_nodes_)
  {
    auto result_future = change_state_clients_[managed_node]->async_send_request(change_state_request);
    wait_for_result(result_future, managed_node);
    if(result_future.get()->success)
    {
      RCLCPP_INFO(this->get_logger(), "Node %s configured", managed_node.c_str());
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Node %s failed to configure", managed_node.c_str());
      return;
    }
  }

    // Activate
  change_state_request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
  
  for (auto & managed_node : managed_nodes_)
  {
    auto result_future = change_state_clients_[managed_node]->async_send_request(change_state_request);
    wait_for_result(result_future, managed_node);
    if(result_future.get()->success)
    {
      RCLCPP_INFO(this->get_logger(), "Node %s configured", managed_node.c_str());
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Node %s failed to configure", managed_node.c_str());
      return;
    }
  }
  

  response->success = true;
}

void ManagerNode::deactivate_callback(const Trigger::Request::SharedPtr request,
  Trigger::Response::SharedPtr response)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating ManagerNode");
  // Waiting services availability
  response->success = false;
  for (auto & managed_node : managed_nodes_)
  {
    while (!change_state_clients_[managed_node]->wait_for_service(std::chrono::seconds(1)) && rclcpp::ok())
    {
    RCLCPP_INFO(this->get_logger(), "Waiting for service for deactivating %s", managed_node.c_str());
    }  
  }

  // lambda to handle the waiting for result
  auto wait_for_result = [this](auto & result_future, const std::string & node_name) {
      while (result_future.wait_for(std::chrono::seconds(1))!= std::future_status::ready && rclcpp::ok())
      {
      RCLCPP_INFO(this->get_logger(), "Waiting for transition result for %s", node_name.c_str());
      }
  };

  auto change_state_request = std::make_shared<ChangeState::Request>();
  change_state_request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;
  
  for (auto & managed_node : managed_nodes_)
  {
    auto result_future = change_state_clients_[managed_node]->async_send_request(change_state_request);
    wait_for_result(result_future, managed_node);
    if(result_future.get()->success)
    {
      RCLCPP_INFO(this->get_logger(), "Node %s deactivated", managed_node.c_str());
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Node %s failed to deactivate", managed_node.c_str());
      return;
    }
  }

  response->success = true;
}

}  // namespace lifecycle_example

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(lifecycle_example::ManagerNode)