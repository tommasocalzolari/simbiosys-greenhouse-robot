#ifndef SIMBIOSYS_FINAL_APPROACH_CONTROLLER__RESETTABLE_PROGRESS_CHECKER_HPP_
#define SIMBIOSYS_FINAL_APPROACH_CONTROLLER__RESETTABLE_PROGRESS_CHECKER_HPP_

#include <memory>
#include <string>

#include "nav2_controller/plugins/simple_progress_checker.hpp"
#include "rclcpp/subscription.hpp"
#include "std_msgs/msg/empty.hpp"

namespace simbiosys_final_approach_controller
{

class ResettableProgressChecker : public nav2_controller::SimpleProgressChecker
{
public:
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name) override;

private:
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_subscription_;
};

}  // namespace simbiosys_final_approach_controller

#endif  // SIMBIOSYS_FINAL_APPROACH_CONTROLLER__RESETTABLE_PROGRESS_CHECKER_HPP_
