#include "simbiosys_final_approach_controller/resettable_progress_checker.hpp"

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace simbiosys_final_approach_controller
{

void ResettableProgressChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name)
{
  nav2_controller::SimpleProgressChecker::initialize(parent, plugin_name);
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Unable to lock lifecycle node");
  }

  const auto parameter_name = plugin_name + ".reset_topic";
  nav2_util::declare_parameter_if_not_declared(
    node, parameter_name, rclcpp::ParameterValue("final_approach_reset"));
  const auto reset_topic = node->get_parameter(parameter_name).as_string();
  reset_subscription_ = node->create_subscription<std_msgs::msg::Empty>(
    reset_topic,
    rclcpp::SystemDefaultsQoS(),
    [this](std_msgs::msg::Empty::ConstSharedPtr) {reset();});
}

}  // namespace simbiosys_final_approach_controller

PLUGINLIB_EXPORT_CLASS(
  simbiosys_final_approach_controller::ResettableProgressChecker,
  nav2_core::ProgressChecker)
