#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;

namespace
{
double clamp(double value, double lower, double upper)
{
  return std::min(std::max(value, lower), upper);
}
}  // namespace

class Nav2StraightPathClient : public rclcpp::Node
{
public:
  using FollowPath = nav2_msgs::action::FollowPath;
  using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;
  using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
  using GoalHandleComputePath = rclcpp_action::ClientGoalHandle<ComputePathToPose>;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  Nav2StraightPathClient()
  : Node("nav2_straight_path_client"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    distance_m_ = declare_parameter<double>("distance_m", 1.0);
    goal_lateral_offset_m_ = declare_parameter<double>("goal_lateral_offset_m", 0.0);
    auto_lateral_offsets_m_ = declare_parameter<std::vector<double>>(
      "auto_lateral_offsets_m", std::vector<double>{});
    path_resolution_m_ = declare_parameter<double>("path_resolution_m", 0.10);
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    controller_id_ = declare_parameter<std::string>("controller_id", "FollowPath");
    goal_checker_id_ = declare_parameter<std::string>("goal_checker_id", "general_goal_checker");
    action_name_ = declare_parameter<std::string>("action_name", "follow_path");
    use_navigate_to_pose_ = declare_parameter<bool>("use_navigate_to_pose", false);
    navigate_to_pose_action_name_ =
      declare_parameter<std::string>("navigate_to_pose_action_name", "navigate_to_pose");
    use_planner_ = declare_parameter<bool>("use_planner", false);
    planner_action_name_ = declare_parameter<std::string>("planner_action_name", "compute_path_to_pose");
    planner_id_ = declare_parameter<std::string>("planner_id", "GridBased");
    server_timeout_sec_ = declare_parameter<double>("server_timeout_sec", 20.0);
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 10.0);
    finish_on_path_s_ = declare_parameter<bool>("finish_on_path_s", false);
    path_s_topic_ = declare_parameter<std::string>("path_s_topic", "/go2/path_s");
    plan_only_ = declare_parameter<bool>("plan_only", false);

    action_client_ = rclcpp_action::create_client<FollowPath>(this, action_name_);
    planner_client_ = rclcpp_action::create_client<ComputePathToPose>(this, planner_action_name_);
    navigate_to_pose_client_ =
      rclcpp_action::create_client<NavigateToPose>(this, navigate_to_pose_action_name_);
    path_s_sub_ = create_subscription<std_msgs::msg::Float32>(
      path_s_topic_, 10,
      [this](const std_msgs::msg::Float32::SharedPtr msg)
      {
        latest_path_s_m_ = static_cast<double>(msg->data);
        have_path_s_ = true;
      });
  }

  int run()
  {
    geometry_msgs::msg::TransformStamped start_tf;
    const auto tf_deadline = now() + rclcpp::Duration::from_seconds(tf_timeout_sec_);
    while (rclcpp::ok()) {
      try {
        start_tf = tf_buffer_.lookupTransform(odom_frame_, base_frame_, tf2::TimePointZero);
        break;
      } catch (const tf2::TransformException & error) {
        if (now() > tf_deadline) {
          RCLCPP_ERROR(get_logger(), "Timed out waiting for %s -> %s TF: %s",
            odom_frame_.c_str(), base_frame_.c_str(), error.what());
          return 1;
        }
        rclcpp::sleep_for(100ms);
      }
    }

    if (use_navigate_to_pose_) {
      return runNavigateToPose(start_tf);
    }

    if (!action_client_->wait_for_action_server(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(server_timeout_sec_))))
    {
      RCLCPP_ERROR(get_logger(), "FollowPath action server '%s' is not available.", action_name_.c_str());
      return 1;
    }

    nav_msgs::msg::Path path;
    if (use_planner_) {
      if (!computePlannedPath(start_tf, path)) {
        return 1;
      }
    } else {
      path = makeStraightPath(start_tf);
    }
    logPathSummary(path, start_tf);

    if (plan_only_) {
      RCLCPP_INFO(get_logger(), "plan_only=true; not sending FollowPath goal.");
      return 0;
    }

    FollowPath::Goal goal;
    goal.path = path;
    goal.controller_id = controller_id_;
    goal.goal_checker_id = goal_checker_id_;

    rclcpp_action::Client<FollowPath>::SendGoalOptions options;
    options.feedback_callback =
      [this](GoalHandleFollowPath::SharedPtr, const std::shared_ptr<const FollowPath::Feedback> feedback)
      {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "tracking: distance_to_goal=%.3f speed=%.3f",
          feedback->distance_to_goal,
          feedback->speed);
      };

    auto goal_future = action_client_->async_send_goal(goal, options);
    if (rclcpp::spin_until_future_complete(shared_from_this(), goal_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Failed to send FollowPath goal.");
      return 1;
    }

    auto goal_handle = goal_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "FollowPath goal was rejected.");
      return 1;
    }

    RCLCPP_INFO(get_logger(), "FollowPath goal accepted: distance=%.2f m, points=%zu",
      distance_m_, path.poses.size());

    bool have_start_path_s = false;
    double start_path_s_m = 0.0;
    if (finish_on_path_s_ && have_path_s_) {
      start_path_s_m = latest_path_s_m_;
      have_start_path_s = true;
    }

    auto result_future = action_client_->async_get_result(goal_handle);
    while (rclcpp::ok()) {
      const auto status = rclcpp::spin_until_future_complete(shared_from_this(), result_future, 100ms);
      if (status == rclcpp::FutureReturnCode::SUCCESS) {
        break;
      }
      if (status != rclcpp::FutureReturnCode::TIMEOUT) {
        RCLCPP_ERROR(get_logger(), "Failed while waiting for FollowPath result.");
        return 1;
      }

      if (finish_on_path_s_) {
        if (!have_start_path_s && have_path_s_) {
          start_path_s_m = latest_path_s_m_;
          have_start_path_s = true;
          RCLCPP_INFO(
            get_logger(), "Path-s finish monitor started at %.3f m on %s.",
            start_path_s_m, path_s_topic_.c_str());
        }
        if (!have_start_path_s) {
          continue;
        }
        const double travelled_m = latest_path_s_m_ - start_path_s_m;
        if (std::isfinite(travelled_m) && travelled_m >= distance_m_) {
          RCLCPP_INFO(
            get_logger(),
            "Path-s target reached: travelled=%.3f m target=%.3f m; canceling FollowPath.",
            travelled_m, distance_m_);
          const auto cancel_future = action_client_->async_cancel_goal(goal_handle);
          (void)rclcpp::spin_until_future_complete(shared_from_this(), cancel_future, 1s);
          return 0;
        }
      }
    }

    const auto result = result_future.get();
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_ERROR(get_logger(), "FollowPath failed: action_code=%d", static_cast<int>(result.code));
      return 1;
    }

    RCLCPP_INFO(get_logger(), "FollowPath completed.");
    return 0;
  }

private:
  int runNavigateToPose(const geometry_msgs::msg::TransformStamped & start_tf)
  {
    if (!navigate_to_pose_client_->wait_for_action_server(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(server_timeout_sec_))))
    {
      RCLCPP_ERROR(
        get_logger(), "NavigateToPose action server '%s' is not available.",
        navigate_to_pose_action_name_.c_str());
      return 1;
    }

    NavigateToPose::Goal goal;
    goal.pose = makePoseAtDistance(
      start_tf, clamp(distance_m_, 0.1, 250.0), clamp(goal_lateral_offset_m_, -3.0, 3.0));

    const auto & p = goal.pose.pose.position;
    RCLCPP_INFO(
      get_logger(), "NavigateToPose goal: distance=%.2f m lateral_offset=%.2f m target=(%.3f, %.3f)",
      distance_m_, goal_lateral_offset_m_, p.x, p.y);

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.feedback_callback =
      [this](
        GoalHandleNavigateToPose::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback)
      {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "navigating: distance_remaining=%.3f recoveries=%d",
          feedback->distance_remaining,
          static_cast<int>(feedback->number_of_recoveries));
      };

    auto goal_future = navigate_to_pose_client_->async_send_goal(goal, options);
    if (rclcpp::spin_until_future_complete(shared_from_this(), goal_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Failed to send NavigateToPose goal.");
      return 1;
    }

    auto goal_handle = goal_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "NavigateToPose goal was rejected.");
      return 1;
    }

    bool have_start_path_s = false;
    double start_path_s_m = 0.0;
    if (finish_on_path_s_ && have_path_s_) {
      start_path_s_m = latest_path_s_m_;
      have_start_path_s = true;
    }

    auto result_future = navigate_to_pose_client_->async_get_result(goal_handle);
    while (rclcpp::ok()) {
      const auto status = rclcpp::spin_until_future_complete(shared_from_this(), result_future, 100ms);
      if (status == rclcpp::FutureReturnCode::SUCCESS) {
        break;
      }
      if (status != rclcpp::FutureReturnCode::TIMEOUT) {
        RCLCPP_ERROR(get_logger(), "Failed while waiting for NavigateToPose result.");
        return 1;
      }

      if (finish_on_path_s_) {
        if (!have_start_path_s && have_path_s_) {
          start_path_s_m = latest_path_s_m_;
          have_start_path_s = true;
          RCLCPP_INFO(
            get_logger(), "Path-s finish monitor started at %.3f m on %s.",
            start_path_s_m, path_s_topic_.c_str());
        }
        if (!have_start_path_s) {
          continue;
        }
        const double travelled_m = latest_path_s_m_ - start_path_s_m;
        if (std::isfinite(travelled_m) && travelled_m >= distance_m_) {
          RCLCPP_INFO(
            get_logger(),
            "Path-s target reached: travelled=%.3f m target=%.3f m; canceling NavigateToPose.",
            travelled_m, distance_m_);
          const auto cancel_future = navigate_to_pose_client_->async_cancel_goal(goal_handle);
          (void)rclcpp::spin_until_future_complete(shared_from_this(), cancel_future, 1s);
          return 0;
        }
      }
    }

    const auto result = result_future.get();
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_ERROR(get_logger(), "NavigateToPose failed: action_code=%d", static_cast<int>(result.code));
      return 1;
    }

    RCLCPP_INFO(get_logger(), "NavigateToPose completed.");
    return 0;
  }

  nav_msgs::msg::Path makeStraightPath(const geometry_msgs::msg::TransformStamped & start_tf)
  {
    tf2::Quaternion q(
      start_tf.transform.rotation.x,
      start_tf.transform.rotation.y,
      start_tf.transform.rotation.z,
      start_tf.transform.rotation.w);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    const double distance = clamp(distance_m_, 0.1, 250.0);
    const double resolution = clamp(path_resolution_m_, 0.02, 1.0);
    const int steps = std::max(1, static_cast<int>(std::ceil(distance / resolution)));
    const double start_x = start_tf.transform.translation.x;
    const double start_y = start_tf.transform.translation.y;

    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = odom_frame_;
    path.poses.reserve(static_cast<std::size_t>(steps + 1));
    for (int i = 0; i <= steps; ++i) {
      const double s = distance * static_cast<double>(i) / static_cast<double>(steps);
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      const double lateral = goal_lateral_offset_m_ * s / distance;
      pose.pose.position.x = start_x + std::cos(yaw) * s - std::sin(yaw) * lateral;
      pose.pose.position.y = start_y + std::sin(yaw) * s + std::cos(yaw) * lateral;
      pose.pose.position.z = 0.0;
      pose.pose.orientation = start_tf.transform.rotation;
      path.poses.push_back(pose);
    }
    return path;
  }

  geometry_msgs::msg::PoseStamped makePoseAtDistance(
    const geometry_msgs::msg::TransformStamped & start_tf, double distance_m, double lateral_offset_m = 0.0)
  {
    tf2::Quaternion q(
      start_tf.transform.rotation.x,
      start_tf.transform.rotation.y,
      start_tf.transform.rotation.z,
      start_tf.transform.rotation.w);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now();
    pose.header.frame_id = odom_frame_;
    pose.pose.position.x =
      start_tf.transform.translation.x + std::cos(yaw) * distance_m - std::sin(yaw) * lateral_offset_m;
    pose.pose.position.y =
      start_tf.transform.translation.y + std::sin(yaw) * distance_m + std::cos(yaw) * lateral_offset_m;
    pose.pose.position.z = 0.0;
    pose.pose.orientation = start_tf.transform.rotation;
    return pose;
  }

  bool computePlannedPath(
    const geometry_msgs::msg::TransformStamped & start_tf,
    nav_msgs::msg::Path & path)
  {
    if (!planner_client_->wait_for_action_server(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(server_timeout_sec_))))
    {
      RCLCPP_ERROR(
        get_logger(), "ComputePathToPose action server '%s' is not available.",
        planner_action_name_.c_str());
      return false;
    }

    std::vector<double> candidate_offsets = auto_lateral_offsets_m_;
    if (candidate_offsets.empty()) {
      candidate_offsets.push_back(goal_lateral_offset_m_);
    }
    RCLCPP_INFO(
      get_logger(), "Trying %zu Nav2 planner candidate lateral offset(s).",
      candidate_offsets.size());
    for (const double candidate_offset : candidate_offsets) {
      if (requestPlannedPath(start_tf, candidate_offset, path)) {
        selected_lateral_offset_m_ = candidate_offset;
        return true;
      }
    }
    RCLCPP_ERROR(get_logger(), "All Nav2 planner lateral-offset candidates failed.");
    return false;
  }

  bool requestPlannedPath(
    const geometry_msgs::msg::TransformStamped & start_tf,
    double lateral_offset_m,
    nav_msgs::msg::Path & path)
  {
    ComputePathToPose::Goal goal;
    goal.start = makePoseAtDistance(start_tf, 0.0);
    goal.goal = makePoseAtDistance(
      start_tf, clamp(distance_m_, 0.1, 250.0), clamp(lateral_offset_m, -3.0, 3.0));
    goal.planner_id = planner_id_;
    goal.use_start = true;

    RCLCPP_INFO(
      get_logger(), "ComputePathToPose candidate: distance=%.2f m lateral_offset=%.2f m",
      distance_m_, lateral_offset_m);

    auto goal_future = planner_client_->async_send_goal(goal);
    if (rclcpp::spin_until_future_complete(shared_from_this(), goal_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_WARN(get_logger(), "Failed to send ComputePathToPose goal for lateral_offset=%.2f.",
        lateral_offset_m);
      return false;
    }

    auto goal_handle = goal_future.get();
    if (!goal_handle) {
      RCLCPP_WARN(get_logger(), "ComputePathToPose goal was rejected for lateral_offset=%.2f.",
        lateral_offset_m);
      return false;
    }

    auto result_future = planner_client_->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_WARN(get_logger(), "Failed while waiting for ComputePathToPose result for lateral_offset=%.2f.",
        lateral_offset_m);
      return false;
    }

    const auto result = result_future.get();
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_WARN(get_logger(), "ComputePathToPose failed for lateral_offset=%.2f: action_code=%d",
        lateral_offset_m, static_cast<int>(result.code));
      return false;
    }

    path = result.result->path;
    if (path.poses.empty()) {
      RCLCPP_WARN(get_logger(), "ComputePathToPose returned an empty path for lateral_offset=%.2f.",
        lateral_offset_m);
      return false;
    }

    RCLCPP_INFO(
      get_logger(), "ComputePathToPose selected: distance=%.2f m lateral_offset=%.2f m planned_points=%zu",
      distance_m_, lateral_offset_m, path.poses.size());
    return true;
  }

  void logPathSummary(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::TransformStamped & start_tf)
  {
    if (path.poses.empty()) {
      return;
    }

    double min_x = path.poses.front().pose.position.x;
    double max_x = min_x;
    double min_y = path.poses.front().pose.position.y;
    double max_y = min_y;
    for (const auto & pose : path.poses) {
      min_x = std::min(min_x, pose.pose.position.x);
      max_x = std::max(max_x, pose.pose.position.x);
      min_y = std::min(min_y, pose.pose.position.y);
      max_y = std::max(max_y, pose.pose.position.y);
    }

    const double start_x = start_tf.transform.translation.x;
    const double start_y = start_tf.transform.translation.y;
    const auto & first = path.poses.front().pose.position;
    const auto & last = path.poses.back().pose.position;
    RCLCPP_INFO(
      get_logger(),
      "planned_path_summary: points=%zu first=(%.3f, %.3f) last=(%.3f, %.3f) "
      "rel_last=(%.3f, %.3f) rel_y_range=(%.3f, %.3f) rel_x_range=(%.3f, %.3f)",
      path.poses.size(),
      first.x, first.y,
      last.x, last.y,
      last.x - start_x, last.y - start_y,
      min_y - start_y, max_y - start_y,
      min_x - start_x, max_x - start_x);
  }

  double distance_m_{1.0};
  double goal_lateral_offset_m_{0.0};
  double selected_lateral_offset_m_{0.0};
  std::vector<double> auto_lateral_offsets_m_;
  double path_resolution_m_{0.10};
  std::string odom_frame_;
  std::string base_frame_;
  std::string controller_id_;
  std::string goal_checker_id_;
  std::string action_name_;
  bool use_navigate_to_pose_{false};
  std::string navigate_to_pose_action_name_;
  bool use_planner_{false};
  std::string planner_action_name_;
  std::string planner_id_;
  double server_timeout_sec_{20.0};
  double tf_timeout_sec_{10.0};
  bool finish_on_path_s_{false};
  std::string path_s_topic_;
  bool plan_only_{false};
  bool have_path_s_{false};
  double latest_path_s_m_{0.0};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp_action::Client<FollowPath>::SharedPtr action_client_;
  rclcpp_action::Client<ComputePathToPose>::SharedPtr planner_client_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_to_pose_client_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr path_s_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Nav2StraightPathClient>();
  const int ret = node->run();
  rclcpp::shutdown();
  return ret;
}
