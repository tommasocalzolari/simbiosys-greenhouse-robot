#ifndef SIMBIOSYS_FINAL_APPROACH_CONTROLLER__FINAL_APPROACH_CONTROLLER_HPP_
#define SIMBIOSYS_FINAL_APPROACH_CONTROLLER__FINAL_APPROACH_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/empty.hpp"

namespace simbiosys_final_approach_controller
{

class FinalApproachController : public nav2_core::Controller
{
public:
  FinalApproachController();
  ~FinalApproachController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  template<typename T>
  T parameter(const std::string & suffix, const T & default_value);

  double remainingPathLength(const geometry_msgs::msg::PoseStamped & pose) const;
  bool transformGoalToBase(geometry_msgs::msg::PoseStamped & goal_in_base) const;
  geometry_msgs::msg::TwistStamped finalCommand(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::PoseStamped & goal_in_base) const;
  bool commandIsCollisionFree(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & command) const;
  void switchMode(bool final_approach, const char * reason);
  static double commandAxis(
    double error, double gain, double tolerance, double min_speed, double max_speed);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>
  collision_checker_;
  pluginlib::ClassLoader<nav2_core::Controller> controller_loader_;
  nav2_core::Controller::Ptr primary_controller_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Empty>::SharedPtr reset_publisher_;
  rclcpp::Logger logger_{rclcpp::get_logger("FinalApproachController")};
  rclcpp::Clock::SharedPtr clock_;

  std::string plugin_name_;
  std::string base_frame_;
  nav_msgs::msg::Path path_;
  bool final_approach_{false};
  bool final_approach_armed_{true};

  double entry_distance_{0.8};
  double return_distance_{1.0};
  double position_tolerance_{0.03};
  double yaw_tolerance_{0.0872665};
  double kx_{1.2};
  double ky_{1.2};
  double kyaw_{2.0};
  double min_vx_{0.20};
  double min_vy_{0.10};
  double min_wz_{0.45};
  double max_vx_{0.5};
  double max_vy_{0.5};
  double max_wz_{1.0};
  double collision_horizon_{0.5};
  double collision_step_{0.05};
  double transform_tolerance_{0.2};
  double speed_limit_scale_{1.0};
};

}  // namespace simbiosys_final_approach_controller

#endif  // SIMBIOSYS_FINAL_APPROACH_CONTROLLER__FINAL_APPROACH_CONTROLLER_HPP_
