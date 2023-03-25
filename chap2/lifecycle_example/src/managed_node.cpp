#include "lifecycle_example/managed_node.hpp"

#include <memory>

namespace lifecycle_example
{

ManagedNode::ManagedNode(const rclcpp::NodeOptions & options):
  rclcpp_lifecycle::LifecycleNode("managed_node", options)
{
  RCLCPP_INFO(this->get_logger(), "Creating ManagedNode");
}   

CallbackReturn ManagedNode::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(this->get_logger(), "Configuring ManagedNode");
  return CallbackReturn::SUCCESS;
}

CallbackReturn ManagedNode::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(this->get_logger(), "Activating ManagedNode");
  return CallbackReturn::SUCCESS;
}

CallbackReturn ManagedNode::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating ManagedNode");
  return CallbackReturn::SUCCESS;
}

}  // namespace lifecycle_example

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(lifecycle_example::ManagedNode)