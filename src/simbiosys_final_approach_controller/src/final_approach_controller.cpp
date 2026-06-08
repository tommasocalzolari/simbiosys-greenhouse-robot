#include "simbiosys_final_approach_controller/final_approach_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <utility>

#include "angles/angles.h"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"

namespace simbiosys_final_approach_controller
{

FinalApproachController::FinalApproachController()
: controller_loader_("nav2_core", "nav2_core::Controller")
{
}

template<typename T>
T FinalApproachController::parameter(const std::string & suffix, const T & default_value)
{
  auto node = node_.lock();
  const auto name = plugin_name_ + "." + suffix;
  nav2_util::declare_parameter_if_not_declared(
    node, name, rclcpp::ParameterValue(default_value));
  return node->get_parameter(name).get_value<T>();
}

void FinalApproachController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  plugin_name_ = std::move(name);
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Unable to lock lifecycle node");
  }
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  base_frame_ = costmap_ros_->getBaseFrameID();

  entry_distance_ = parameter("entry_distance", entry_distance_);
  return_distance_ = parameter("return_distance", return_distance_);
  position_tolerance_ = parameter("position_tolerance", position_tolerance_);
  yaw_tolerance_ = parameter("yaw_tolerance", yaw_tolerance_);
  kx_ = parameter("kx", kx_);
  ky_ = parameter("ky", ky_);
  kyaw_ = parameter("kyaw", kyaw_);
  min_vx_ = parameter("min_vx", min_vx_);
  min_vy_ = parameter("min_vy", min_vy_);
  min_wz_ = parameter("min_wz", min_wz_);
  max_vx_ = parameter("max_vx", max_vx_);
  max_vy_ = parameter("max_vy", max_vy_);
  max_wz_ = parameter("max_wz", max_wz_);
  collision_horizon_ = parameter("collision_horizon", collision_horizon_);
  collision_step_ = parameter("collision_step", collision_step_);
  transform_tolerance_ = parameter("transform_tolerance", transform_tolerance_);
  const auto reset_topic = parameter<std::string>("reset_topic", "final_approach_reset");
  const auto primary_type = parameter<std::string>(
    "primary_controller", "nav2_rotation_shim_controller::RotationShimController");

  if (entry_distance_ <= 0.0 || return_distance_ <= entry_distance_) {
    throw std::runtime_error("return_distance must be greater than entry_distance");
  }
  if (position_tolerance_ <= 0.0 || yaw_tolerance_ <= 0.0 ||
    collision_horizon_ <= 0.0 || collision_step_ <= 0.0)
  {
    throw std::runtime_error("Tolerances and collision prediction settings must be positive");
  }

  primary_controller_ = controller_loader_.createSharedInstance(primary_type);
  primary_controller_->configure(parent, plugin_name_ + ".primary", tf_, costmap_ros_);
  collision_checker_ = std::make_unique<
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(
    costmap_ros_->getCostmap());
  reset_publisher_ = node->create_publisher<std_msgs::msg::Empty>(
    reset_topic, rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(
    logger_,
    "Configured final approach: enter %.2f m, return %.2f m, tolerances %.3f m / %.1f deg",
    entry_distance_, return_distance_, position_tolerance_, yaw_tolerance_ * 180.0 / M_PI);
}

void FinalApproachController::cleanup()
{
  primary_controller_->cleanup();
  primary_controller_.reset();
  collision_checker_.reset();
  reset_publisher_.reset();
  path_.poses.clear();
}

void FinalApproachController::activate()
{
  primary_controller_->activate();
  reset_publisher_->on_activate();
}

void FinalApproachController::deactivate()
{
  reset_publisher_->on_deactivate();
  primary_controller_->deactivate();
}

void FinalApproachController::setPlan(const nav_msgs::msg::Path & path)
{
  bool new_goal = path_.poses.empty() || path.poses.empty();
  if (!new_goal) {
    const auto & old_goal = path_.poses.back().pose;
    const auto & new_goal_pose = path.poses.back().pose;
    const double position_change = std::hypot(
      old_goal.position.x - new_goal_pose.position.x,
      old_goal.position.y - new_goal_pose.position.y);
    const double yaw_change = std::abs(
      angles::shortest_angular_distance(
        tf2::getYaw(old_goal.orientation), tf2::getYaw(new_goal_pose.orientation)));
    new_goal = position_change > 0.02 || yaw_change > 0.02 ||
      path.header.frame_id != path_.header.frame_id;
  }

  path_ = path;
  primary_controller_->setPlan(path);
  if (new_goal) {
    final_approach_armed_ = true;
    if (final_approach_) {
      switchMode(false, "new goal");
    }
  }
}

double FinalApproachController::remainingPathLength(
  const geometry_msgs::msg::PoseStamped & pose) const
{
  if (path_.poses.empty()) {
    return std::numeric_limits<double>::infinity();
  }

  geometry_msgs::msg::PoseStamped pose_in_path;
  if (!nav2_util::transformPoseInTargetFrame(
      pose, pose_in_path, *tf_, path_.header.frame_id, transform_tolerance_))
  {
    return std::numeric_limits<double>::infinity();
  }

  std::size_t nearest = 0;
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < path_.poses.size(); ++i) {
    const double distance = std::hypot(
      path_.poses[i].pose.position.x - pose_in_path.pose.position.x,
      path_.poses[i].pose.position.y - pose_in_path.pose.position.y);
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest = i;
    }
  }

  double length = nearest_distance;
  for (std::size_t i = nearest + 1; i < path_.poses.size(); ++i) {
    length += std::hypot(
      path_.poses[i].pose.position.x - path_.poses[i - 1].pose.position.x,
      path_.poses[i].pose.position.y - path_.poses[i - 1].pose.position.y);
  }
  return length;
}

bool FinalApproachController::transformGoalToBase(
  geometry_msgs::msg::PoseStamped & goal_in_base) const
{
  if (path_.poses.empty()) {
    return false;
  }
  return nav2_util::transformPoseInTargetFrame(
    path_.poses.back(), goal_in_base, *tf_, base_frame_, transform_tolerance_);
}

double FinalApproachController::commandAxis(
  const double error,
  const double gain,
  const double tolerance,
  const double min_speed,
  const double max_speed)
{
  const double magnitude = std::abs(error);
  if (magnitude <= tolerance || max_speed <= 0.0) {
    return 0.0;
  }

  // Taper the usable floor to zero over the final three tolerance widths.
  const double floor_scale = std::clamp(
    (magnitude - tolerance) / (3.0 * tolerance), 0.0, 1.0);
  const double usable_floor = min_speed * floor_scale;
  const double commanded = std::clamp(gain * magnitude, usable_floor, max_speed);
  return std::copysign(commanded, error);
}

geometry_msgs::msg::TwistStamped FinalApproachController::finalCommand(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::PoseStamped & goal_in_base) const
{
  geometry_msgs::msg::TwistStamped command;
  command.header.stamp = clock_->now();
  command.header.frame_id = base_frame_;

  const double yaw_error = angles::shortest_angular_distance(
    0.0, tf2::getYaw(goal_in_base.pose.orientation));
  // Per-axis tolerance is tightened so both stopped axes imply the configured
  // radial position tolerance used by Nav2's goal checker.
  const double axis_tolerance = position_tolerance_ / std::sqrt(2.0);
  command.twist.linear.x = commandAxis(
    goal_in_base.pose.position.x, kx_, axis_tolerance, min_vx_ * speed_limit_scale_,
    max_vx_ * speed_limit_scale_);
  command.twist.linear.y = commandAxis(
    goal_in_base.pose.position.y, ky_, axis_tolerance, min_vy_ * speed_limit_scale_,
    max_vy_ * speed_limit_scale_);
  command.twist.angular.z = commandAxis(
    yaw_error, kyaw_, yaw_tolerance_, min_wz_ * speed_limit_scale_,
    max_wz_ * speed_limit_scale_);
  (void)pose;
  return command;
}

bool FinalApproachController::commandIsCollisionFree(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & command) const
{
  geometry_msgs::msg::PoseStamped costmap_pose;
  if (!nav2_util::transformPoseInTargetFrame(
      pose, costmap_pose, *tf_, costmap_ros_->getGlobalFrameID(), transform_tolerance_))
  {
    return false;
  }

  double x = costmap_pose.pose.position.x;
  double y = costmap_pose.pose.position.y;
  double yaw = tf2::getYaw(costmap_pose.pose.orientation);
  const auto footprint = costmap_ros_->getRobotFootprint();
  const int samples = std::max(
    1, static_cast<int>(std::ceil(collision_horizon_ / collision_step_)));
  const double dt = collision_horizon_ / samples;

  auto * costmap = costmap_ros_->getCostmap();
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap->getMutex());
  for (int i = 0; i < samples; ++i) {
    x += (command.linear.x * std::cos(yaw) - command.linear.y * std::sin(yaw)) * dt;
    y += (command.linear.x * std::sin(yaw) + command.linear.y * std::cos(yaw)) * dt;
    yaw += command.angular.z * dt;
    const double cost = collision_checker_->footprintCostAtPose(x, y, yaw, footprint);
    if (cost < 0.0 || cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
      return false;
    }
  }
  return true;
}

void FinalApproachController::switchMode(const bool final_approach, const char * reason)
{
  if (final_approach_ == final_approach) {
    return;
  }
  final_approach_ = final_approach;
  reset_publisher_->publish(std_msgs::msg::Empty());
  RCLCPP_INFO(
    logger_, "%s final-approach mode (%s)",
    final_approach ? "Entered" : "Left", reason);
}

geometry_msgs::msg::TwistStamped FinalApproachController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  const double remaining = remainingPathLength(pose);
  if (remaining > return_distance_) {
    final_approach_armed_ = true;
  }

  if (!final_approach_ && final_approach_armed_ && remaining <= entry_distance_) {
    switchMode(true, "entry distance reached");
  } else if (final_approach_ && remaining > return_distance_) {
    switchMode(false, "remaining path grew");
  }

  if (!final_approach_) {
    return primary_controller_->computeVelocityCommands(pose, velocity, goal_checker);
  }

  geometry_msgs::msg::PoseStamped goal_in_base;
  if (!transformGoalToBase(goal_in_base)) {
    final_approach_armed_ = false;
    switchMode(false, "goal transform unavailable");
    return primary_controller_->computeVelocityCommands(pose, velocity, goal_checker);
  }

  auto command = finalCommand(pose, goal_in_base);
  if (!commandIsCollisionFree(pose, command.twist)) {
    final_approach_armed_ = false;
    switchMode(false, "direct motion obstructed");
    return primary_controller_->computeVelocityCommands(pose, velocity, goal_checker);
  }
  return command;
}

void FinalApproachController::setSpeedLimit(
  const double & speed_limit, const bool & percentage)
{
  primary_controller_->setSpeedLimit(speed_limit, percentage);
  if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
    speed_limit_scale_ = 1.0;
  } else if (percentage) {
    speed_limit_scale_ = std::clamp(speed_limit / 100.0, 0.0, 1.0);
  } else {
    speed_limit_scale_ = std::clamp(speed_limit / std::max(max_vx_, max_vy_), 0.0, 1.0);
  }
}

}  // namespace simbiosys_final_approach_controller

PLUGINLIB_EXPORT_CLASS(
  simbiosys_final_approach_controller::FinalApproachController,
  nav2_core::Controller)
