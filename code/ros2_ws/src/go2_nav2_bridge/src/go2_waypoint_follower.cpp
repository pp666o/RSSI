#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;

namespace
{
double clamp(double value, double lower, double upper)
{
  return std::min(std::max(value, lower), upper);
}

double wrapAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double yawFromOdom(const nav_msgs::msg::Odometry & odom)
{
  const auto & q_msg = odom.pose.pose.orientation;
  tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

struct Waypoint
{
  double x;
  double y;
  double yaw;
};
}  // namespace

class Go2WaypointFollower : public rclcpp::Node
{
public:
  Go2WaypointFollower()
  : Node("go2_waypoint_follower")
  {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    path_s_topic_ = declare_parameter<std::string>("path_s_topic", "/go2/path_s");
    relative_waypoints_text_ = declare_parameter<std::string>("relative_waypoints_text", "");
    if (!relative_waypoints_text_.empty()) {
      waypoint_values_ = parseWaypointValuesText(relative_waypoints_text_);
    } else {
      waypoint_values_ = declare_parameter<std::vector<double>>(
        "relative_waypoints", std::vector<double>{3.0, 0.0, 0.0});
    }
    controller_frequency_hz_ = declare_parameter<double>("controller_frequency_hz", 20.0);
    max_vx_mps_ = declare_parameter<double>("max_vx_mps", 0.30);
    max_vy_mps_ = declare_parameter<double>("max_vy_mps", 0.08);
    max_wz_radps_ = declare_parameter<double>("max_wz_radps", 0.45);
    xy_tolerance_m_ = declare_parameter<double>("xy_tolerance_m", 0.20);
    yaw_tolerance_rad_ = declare_parameter<double>("yaw_tolerance_rad", 0.30);
    min_vx_mps_ = declare_parameter<double>("min_vx_mps", 0.10);
    k_xy_ = declare_parameter<double>("k_xy", 0.45);
    k_yaw_ = declare_parameter<double>("k_yaw", 0.90);
    stop_on_finish_ = declare_parameter<bool>("stop_on_finish", true);
    shutdown_on_finish_ = declare_parameter<bool>("shutdown_on_finish", true);
    use_path_s_stop_ = declare_parameter<bool>("use_path_s_stop", true);
    path_s_stop_tolerance_m_ = declare_parameter<double>("path_s_stop_tolerance_m", 0.03);
    stop_publish_repeats_ = static_cast<int>(
      clamp(declare_parameter<int>("stop_publish_repeats", 20), 1, 100));
    stop_publish_period_ms_ = static_cast<int>(
      clamp(declare_parameter<int>("stop_publish_period_ms", 50), 10, 500));
    autostart_ = declare_parameter<bool>("autostart", true);

    waypoints_ = parseWaypoints(waypoint_values_);
    if (waypoints_.empty()) {
      throw std::runtime_error("relative_waypoints must contain at least one x,y,yaw triple.");
    }
    target_route_length_m_ = computeRouteLength(waypoints_);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&Go2WaypointFollower::onOdom, this, std::placeholders::_1));
    path_s_sub_ = create_subscription<std_msgs::msg::Float32>(
      path_s_topic_, rclcpp::SensorDataQoS(),
      std::bind(&Go2WaypointFollower::onPathS, this, std::placeholders::_1));
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

    const double hz = clamp(controller_frequency_hz_, 2.0, 100.0);
    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / hz)),
      std::bind(&Go2WaypointFollower::controlLoop, this));

    RCLCPP_INFO(
      get_logger(),
      "Go2 waypoint follower started: odom=%s path_s=%s cmd_vel=%s waypoints=%zu autostart=%s route_length_stop=%s target_route_length=%.3f",
      odom_topic_.c_str(), path_s_topic_.c_str(), cmd_vel_topic_.c_str(), waypoints_.size(),
      autostart_ ? "true" : "false", use_path_s_stop_ ? "true" : "false",
      target_route_length_m_);
  }

  ~Go2WaypointFollower() override
  {
    publishStop();
  }

private:
  std::vector<double> parseWaypointValuesText(const std::string & text)
  {
    std::string normalized = text;
    for (char & ch : normalized) {
      const bool numeric_char =
        (ch >= '0' && ch <= '9') || ch == '-' || ch == '+' || ch == '.' || ch == 'e' || ch == 'E';
      if (!numeric_char) {
        ch = ' ';
      }
    }

    std::stringstream stream(normalized);
    std::vector<double> values;
    double value = 0.0;
    while (stream >> value) {
      values.push_back(value);
    }
    if (values.empty()) {
      throw std::runtime_error("relative_waypoints_text did not contain any numeric values.");
    }
    return values;
  }

  std::vector<Waypoint> parseWaypoints(const std::vector<double> & values)
  {
    if (values.size() % 3 != 0) {
      throw std::runtime_error("relative_waypoints must be [x1, y1, yaw1, x2, y2, yaw2, ...].");
    }
    std::vector<Waypoint> waypoints;
    waypoints.reserve(values.size() / 3);
    for (std::size_t i = 0; i < values.size(); i += 3) {
      waypoints.push_back(Waypoint{values[i], values[i + 1], values[i + 2]});
    }
    return waypoints;
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    latest_odom_ = *msg;
    have_odom_ = true;
    if (!have_origin_) {
      origin_x_ = msg->pose.pose.position.x;
      origin_y_ = msg->pose.pose.position.y;
      origin_yaw_ = yawFromOdom(*msg);
      have_origin_ = true;
      RCLCPP_INFO(
        get_logger(), "Waypoint origin set: x=%.3f y=%.3f yaw=%.3f",
        origin_x_, origin_y_, origin_yaw_);
    }
  }

  void onPathS(const std_msgs::msg::Float32::SharedPtr msg)
  {
    latest_path_s_m_ = msg->data;
    have_path_s_ = true;
  }

  Waypoint targetInOdomFrame(const Waypoint & relative) const
  {
    const double c = std::cos(origin_yaw_);
    const double s = std::sin(origin_yaw_);
    return Waypoint{
      origin_x_ + c * relative.x - s * relative.y,
      origin_y_ + s * relative.x + c * relative.y,
      wrapAngle(origin_yaw_ + relative.yaw)};
  }

  void controlLoop()
  {
    if (!autostart_ || finished_) {
      return;
    }
    if (!have_odom_ || !have_origin_) {
      return;
    }
    if (pathSReached()) {
      RCLCPP_INFO(
        get_logger(), "Reached target path_s: current=%.3f target=%.3f tolerance=%.3f",
        latest_path_s_m_, target_route_length_m_, path_s_stop_tolerance_m_);
      finish();
      return;
    }
    if (current_waypoint_index_ >= waypoints_.size()) {
      finish();
      return;
    }

    const Waypoint target = targetInOdomFrame(waypoints_[current_waypoint_index_]);
    const double x = latest_odom_.pose.pose.position.x;
    const double y = latest_odom_.pose.pose.position.y;
    const double yaw = yawFromOdom(latest_odom_);
    const double dx = target.x - x;
    const double dy = target.y - y;
    const double distance = std::hypot(dx, dy);
    const double target_heading = std::atan2(dy, dx);
    const double heading_error = wrapAngle(target_heading - yaw);
    const double final_yaw_error = wrapAngle(target.yaw - yaw);

    if (distance <= xy_tolerance_m_) {
      if (std::fabs(final_yaw_error) <= yaw_tolerance_rad_) {
        RCLCPP_INFO(
          get_logger(), "Reached waypoint %zu/%zu: target=(%.3f, %.3f, %.3f)",
          current_waypoint_index_ + 1, waypoints_.size(), target.x, target.y, target.yaw);
        ++current_waypoint_index_;
        if (current_waypoint_index_ >= waypoints_.size()) {
          finish();
        }
        return;
      }
      publishCommand(0.0, 0.0, clamp(k_yaw_ * final_yaw_error, -max_wz_radps_, max_wz_radps_));
      return;
    }

    const double forward_scale = clamp(std::cos(heading_error), 0.0, 1.0);
    double vx = clamp(k_xy_ * distance * forward_scale, 0.0, max_vx_mps_);
    if (vx > 1e-3) {
      vx = std::max(vx, min_vx_mps_);
    }
    const double vy = clamp(k_xy_ * distance * std::sin(heading_error), -max_vy_mps_, max_vy_mps_);
    const double wz = clamp(k_yaw_ * heading_error, -max_wz_radps_, max_wz_radps_);
    publishCommand(vx, vy, wz);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "waypoint %zu/%zu dist=%.3f heading_err=%.3f cmd=(%.3f, %.3f, %.3f)",
      current_waypoint_index_ + 1, waypoints_.size(), distance, heading_error, vx, vy, wz);
  }

  void finish()
  {
    if (finished_) {
      return;
    }
    finished_ = true;
    RCLCPP_INFO(get_logger(), "All waypoints reached.");
    if (stop_on_finish_) {
      publishStopBurst();
    }
    if (shutdown_on_finish_) {
      RCLCPP_INFO(get_logger(), "Shutting down waypoint follower after finish.");
      rclcpp::shutdown();
    }
  }

  void publishCommand(double vx, double vy, double wz)
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = vx;
    cmd.linear.y = vy;
    cmd.angular.z = wz;
    cmd_pub_->publish(cmd);
  }

  void publishStop()
  {
    if (!cmd_pub_) {
      return;
    }
    publishCommand(0.0, 0.0, 0.0);
  }

  void publishStopBurst()
  {
    for (int i = 0; i < stop_publish_repeats_ && rclcpp::ok(); ++i) {
      publishStop();
      rclcpp::sleep_for(std::chrono::milliseconds(stop_publish_period_ms_));
    }
  }

  bool pathSReached() const
  {
    return use_path_s_stop_ && have_path_s_ &&
           latest_path_s_m_ >= target_route_length_m_ - path_s_stop_tolerance_m_;
  }

  double computeRouteLength(const std::vector<Waypoint> & waypoints) const
  {
    double length = 0.0;
    double prev_x = 0.0;
    double prev_y = 0.0;
    for (const auto & waypoint : waypoints) {
      length += std::hypot(waypoint.x - prev_x, waypoint.y - prev_y);
      prev_x = waypoint.x;
      prev_y = waypoint.y;
    }
    return length;
  }

  std::string odom_topic_;
  std::string cmd_vel_topic_;
  std::string path_s_topic_;
  std::string relative_waypoints_text_;
  std::vector<double> waypoint_values_;
  std::vector<Waypoint> waypoints_;
  double target_route_length_m_{0.0};
  double controller_frequency_hz_{20.0};
  double max_vx_mps_{0.30};
  double max_vy_mps_{0.08};
  double max_wz_radps_{0.45};
  double xy_tolerance_m_{0.20};
  double yaw_tolerance_rad_{0.30};
  double min_vx_mps_{0.10};
  double k_xy_{0.45};
  double k_yaw_{0.90};
  bool stop_on_finish_{true};
  bool shutdown_on_finish_{true};
  bool use_path_s_stop_{true};
  double path_s_stop_tolerance_m_{0.03};
  int stop_publish_repeats_{20};
  int stop_publish_period_ms_{50};
  bool autostart_{true};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr path_s_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  nav_msgs::msg::Odometry latest_odom_;
  bool have_odom_{false};
  bool have_path_s_{false};
  bool have_origin_{false};
  bool finished_{false};
  std::size_t current_waypoint_index_{0};
  double origin_x_{0.0};
  double origin_y_{0.0};
  double origin_yaw_{0.0};
  double latest_path_s_m_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<Go2WaypointFollower>();
    rclcpp::spin(node);
  } catch (const std::exception & ex) {
    std::cerr << "go2_waypoint_follower fatal: " << ex.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
