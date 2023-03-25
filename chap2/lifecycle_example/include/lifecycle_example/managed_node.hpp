#ifndef LIFECYCLE_EXAMPLE_MANAGED_NODE_HPP_
#define LIFECYCLE_EXAMPLE_MANAGED_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace lifecycle_example
{
using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ManagedNode : public rclcpp_lifecycle::LifecycleNode
{
public: 

/*
        * The constructor of the class ManagedNode is the same as the constructor of the class Node.
        * The only difference is that the constructor of the class ManagedNode takes an additional
        * argument of type rclcpp::NodeOptions.
*/
explicit ManagedNode(const rclcpp::NodeOptions & options);

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state);

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state);

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state);
};

}  // namespace lifecycle_example

#endif /* LIFECYCLE_EXAMPLE_MANAGED_NODE_HPP_ */
