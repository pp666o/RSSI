#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/float32.hpp>

using namespace std::chrono_literals;

namespace
{
double clamp(double value, double lower, double upper)
{
  return std::min(std::max(value, lower), upper);
}

double normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

struct XyzFieldOffsets
{
  int x{-1};
  int y{-1};
  int z{-1};
};

XyzFieldOffsets findXyzFields(const sensor_msgs::msg::PointCloud2 & cloud)
{
  XyzFieldOffsets offsets;
  for (const auto & field : cloud.fields) {
    if (field.datatype != sensor_msgs::msg::PointField::FLOAT32 || field.count != 1) {
      continue;
    }
    if (field.name == "x") {
      offsets.x = static_cast<int>(field.offset);
    } else if (field.name == "y") {
      offsets.y = static_cast<int>(field.offset);
    } else if (field.name == "z") {
      offsets.z = static_cast<int>(field.offset);
    }
  }
  return offsets;
}

bool hasXyzFields(const XyzFieldOffsets & offsets)
{
  return offsets.x >= 0 && offsets.y >= 0 && offsets.z >= 0;
}
}  // namespace

class Go2DirectStraightController : public rclcpp::Node
{
public:
  Go2DirectStraightController()
  : Node("go2_direct_straight_controller")
  {
    distance_m_ = declare_parameter<double>("distance_m", 1.0);
    speed_mps_ = declare_parameter<double>("speed_mps", 0.20);
    max_runtime_sec_ = declare_parameter<double>("max_runtime_sec", 30.0);
    control_hz_ = declare_parameter<double>("control_hz", 10.0);
    start_delay_sec_ = declare_parameter<double>("start_delay_sec", 1.0);
    distance_source_ = declare_parameter<std::string>("distance_source", "path_s");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    path_s_topic_ = declare_parameter<std::string>("path_s_topic", "/go2/path_s");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    pointcloud_topic_ = declare_parameter<std::string>("pointcloud_topic", "/go2/pointcloud");
    use_pointcloud_avoidance_ = declare_parameter<bool>("use_pointcloud_avoidance", true);
    require_fresh_pointcloud_ = declare_parameter<bool>("require_fresh_pointcloud", true);
    pointcloud_timeout_sec_ = declare_parameter<double>("pointcloud_timeout_sec", 0.8);
    pointcloud_front_stop_m_ = declare_parameter<double>("pointcloud_front_stop_m", 0.42);
    pointcloud_front_turn_m_ = declare_parameter<double>("pointcloud_front_turn_m", 0.75);
    pointcloud_side_clearance_m_ = declare_parameter<double>("pointcloud_side_clearance_m", 0.24);
    pointcloud_front_angle_rad_ = declare_parameter<double>("pointcloud_front_angle_rad", 0.35);
    pointcloud_side_min_angle_rad_ = declare_parameter<double>("pointcloud_side_min_angle_rad", 0.45);
    pointcloud_side_max_angle_rad_ = declare_parameter<double>("pointcloud_side_max_angle_rad", 1.57);
    pointcloud_min_height_m_ = declare_parameter<double>("pointcloud_min_height_m", -0.20);
    pointcloud_max_height_m_ = declare_parameter<double>("pointcloud_max_height_m", 0.90);
    max_turn_radps_ = declare_parameter<double>("max_turn_radps", 0.40);
    heading_hold_kp_ = declare_parameter<double>("heading_hold_kp", 0.80);

    distance_m_ = clamp(distance_m_, 0.05, 100.0);
    speed_mps_ = clamp(speed_mps_, 0.03, 0.35);
    max_runtime_sec_ = clamp(max_runtime_sec_, 1.0, 3600.0);
    control_hz_ = clamp(control_hz_, 2.0, 50.0);
    start_delay_sec_ = clamp(start_delay_sec_, 0.0, 10.0);
    pointcloud_timeout_sec_ = clamp(pointcloud_timeout_sec_, 0.2, 5.0);
    pointcloud_front_stop_m_ = clamp(pointcloud_front_stop_m_, 0.20, 2.0);
    pointcloud_front_turn_m_ = clamp(pointcloud_front_turn_m_, pointcloud_front_stop_m_, 3.0);
    pointcloud_side_clearance_m_ = clamp(pointcloud_side_clearance_m_, 0.10, 1.0);
    pointcloud_front_angle_rad_ = clamp(pointcloud_front_angle_rad_, 0.05, 1.20);
    pointcloud_side_min_angle_rad_ = clamp(
      pointcloud_side_min_angle_rad_, pointcloud_front_angle_rad_, 2.50);
    pointcloud_side_max_angle_rad_ = clamp(
      pointcloud_side_max_angle_rad_, pointcloud_side_min_angle_rad_, M_PI);
    pointcloud_min_height_m_ = clamp(pointcloud_min_height_m_, -1.0, 2.0);
    pointcloud_max_height_m_ = clamp(pointcloud_max_height_m_, pointcloud_min_height_m_, 2.5);
    max_turn_radps_ = clamp(max_turn_radps_, 0.0, 1.0);
    heading_hold_kp_ = clamp(heading_hold_kp_, 0.0, 5.0);

    if (distance_source_ != "path_s" && distance_source_ != "odom" && distance_source_ != "time") {
      throw std::runtime_error("distance_source must be path_s, odom, or time.");
    }

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    path_s_sub_ = create_subscription<std_msgs::msg::Float32>(
      path_s_topic_, rclcpp::SensorDataQoS(),
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        current_path_s_ = msg->data;
        have_path_s_ = true;
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_odom_x_ = msg->pose.pose.position.x;
        current_odom_y_ = msg->pose.pose.position.y;
        current_yaw_ = yawFromQuaternion(msg->pose.pose.orientation);
        have_odom_ = true;
      });
    pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&Go2DirectStraightController::onPointCloud, this, std::placeholders::_1));

    start_time_ = now();
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / control_hz_)),
      std::bind(&Go2DirectStraightController::controlStep, this));

    RCLCPP_INFO(
      get_logger(),
      "Direct straight controller started: distance=%.3f m speed=%.3f m/s source=%s timeout=%.1f s pointcloud_avoidance=%s",
      distance_m_, speed_mps_, distance_source_.c_str(), max_runtime_sec_,
      use_pointcloud_avoidance_ ? "true" : "false");
  }

  bool succeeded() const
  {
    return succeeded_;
  }

  bool finished() const
  {
    return finished_;
  }

private:
  void controlStep()
  {
    if (finished_) {
      return;
    }

    const auto stamp = now();
    const double elapsed = (stamp - start_time_).seconds();
    if (elapsed < start_delay_sec_) {
      publishStop();
      return;
    }

    if (!started_) {
      if (!initializeDistanceReference()) {
        publishStop();
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "waiting for distance source '%s'", distance_source_.c_str());
        if (elapsed > max_runtime_sec_) {
          finish(false, "timed out before distance source became available");
        }
        return;
      }
      motion_start_time_ = stamp;
      started_ = true;
      if (have_odom_) {
        start_yaw_ = current_yaw_;
        have_start_yaw_ = true;
      }
      RCLCPP_INFO(get_logger(), "motion reference initialized.");
    }
    if (!have_start_yaw_ && have_odom_) {
      start_yaw_ = current_yaw_;
      have_start_yaw_ = true;
    }

    const double runtime = (stamp - motion_start_time_).seconds();
    const double traveled = estimateTraveled(runtime);
    if (traveled >= distance_m_) {
      finish(true, "target distance reached");
      return;
    }
    if (elapsed >= max_runtime_sec_) {
      finish(false, "max runtime reached before target distance");
      return;
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = speed_mps_;
    cmd.angular.z = computeObstacleCorrection(stamp, cmd.linear.x);
    cmd_pub_->publish(cmd);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 500,
      "direct tracking: traveled=%.3f/%.3f m vx=%.3f wz=%.3f front=%.2f left=%.2f right=%.2f points=%d",
      traveled, distance_m_, cmd.linear.x, cmd.angular.z,
      rangeOrZero(pointcloud_front_min_m_), rangeOrZero(pointcloud_left_min_m_),
      rangeOrZero(pointcloud_right_min_m_), pointcloud_valid_points_);
  }

  void onPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const auto offsets = findXyzFields(*msg);
    if (!hasXyzFields(offsets)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "PointCloud2 has no float32 x/y/z fields; cannot use pointcloud avoidance.");
      return;
    }

    const auto point_step = static_cast<std::size_t>(msg->point_step);
    const auto row_step = static_cast<std::size_t>(msg->row_step);
    const auto width = static_cast<std::size_t>(msg->width);
    const auto height = static_cast<std::size_t>(msg->height);
    const auto max_offset = static_cast<std::size_t>(
      std::max({offsets.x, offsets.y, offsets.z}) + static_cast<int>(sizeof(float)));
    if (point_step < max_offset) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "PointCloud2 point_step is too small for x/y/z fields; cannot use pointcloud avoidance.");
      return;
    }

    double front = std::numeric_limits<double>::infinity();
    double left = std::numeric_limits<double>::infinity();
    double right = std::numeric_limits<double>::infinity();
    int valid = 0;

    for (std::size_t row = 0; row < height; ++row) {
      for (std::size_t col = 0; col < width; ++col) {
        const std::size_t index = row * row_step + col * point_step;
        if (index + max_offset > msg->data.size()) {
          continue;
        }

        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        std::memcpy(&x, &msg->data[index + static_cast<std::size_t>(offsets.x)], sizeof(float));
        std::memcpy(&y, &msg->data[index + static_cast<std::size_t>(offsets.y)], sizeof(float));
        std::memcpy(&z, &msg->data[index + static_cast<std::size_t>(offsets.z)], sizeof(float));
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
          continue;
        }
        if (z < pointcloud_min_height_m_ || z > pointcloud_max_height_m_) {
          continue;
        }

        ++valid;
        const double range = std::hypot(static_cast<double>(x), static_cast<double>(y));
        const double angle = std::atan2(static_cast<double>(y), static_cast<double>(x));
        if (std::abs(angle) <= pointcloud_front_angle_rad_) {
          front = std::min(front, range);
        } else if (angle >= pointcloud_side_min_angle_rad_ && angle <= pointcloud_side_max_angle_rad_) {
          left = std::min(left, range);
        } else if (
          angle <= -pointcloud_side_min_angle_rad_ &&
          angle >= -pointcloud_side_max_angle_rad_)
        {
          right = std::min(right, range);
        }
      }
    }

    pointcloud_front_min_m_ = front;
    pointcloud_left_min_m_ = left;
    pointcloud_right_min_m_ = right;
    pointcloud_valid_points_ = valid;
    last_pointcloud_time_ = now();
    have_pointcloud_ = true;
  }

  bool initializeDistanceReference()
  {
    if (distance_source_ == "time") {
      return true;
    }
    if (distance_source_ == "path_s") {
      if (!have_path_s_) {
        return false;
      }
      start_path_s_ = current_path_s_;
      return true;
    }
    if (!have_odom_) {
      return false;
    }
    start_odom_x_ = current_odom_x_;
    start_odom_y_ = current_odom_y_;
    return true;
  }

  double estimateTraveled(double runtime) const
  {
    if (distance_source_ == "time") {
      return runtime * speed_mps_;
    }
    if (distance_source_ == "path_s") {
      return std::max(0.0, current_path_s_ - start_path_s_);
    }
    return std::hypot(current_odom_x_ - start_odom_x_, current_odom_y_ - start_odom_y_);
  }

  bool pointcloudFresh(const rclcpp::Time & stamp) const
  {
    return have_pointcloud_ && (stamp - last_pointcloud_time_).seconds() <= pointcloud_timeout_sec_;
  }

  double computeObstacleCorrection(const rclcpp::Time & stamp, double & vx)
  {
    if (!use_pointcloud_avoidance_) {
      return headingCorrection();
    }
    if (require_fresh_pointcloud_ && !pointcloudFresh(stamp)) {
      vx = 0.0;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "stopping because /go2/pointcloud is stale or unavailable");
      return 0.0;
    }

    const bool front_blocked =
      std::isfinite(pointcloud_front_min_m_) && pointcloud_front_min_m_ <= pointcloud_front_stop_m_;
    const bool front_close =
      std::isfinite(pointcloud_front_min_m_) && pointcloud_front_min_m_ <= pointcloud_front_turn_m_;
    const bool left_close =
      std::isfinite(pointcloud_left_min_m_) && pointcloud_left_min_m_ <= pointcloud_side_clearance_m_;
    const bool right_close =
      std::isfinite(pointcloud_right_min_m_) && pointcloud_right_min_m_ <= pointcloud_side_clearance_m_;

    if (front_blocked) {
      vx = 0.0;
      return chooseTurnDirection() * max_turn_radps_;
    }
    if (front_close) {
      vx = std::min(vx, std::max(0.08, 0.45 * speed_mps_));
      return chooseTurnDirection() * std::min(max_turn_radps_, 0.32);
    }
    if (left_close && !right_close) {
      return -std::min(max_turn_radps_, 0.25);
    }
    if (right_close && !left_close) {
      return std::min(max_turn_radps_, 0.25);
    }
    return headingCorrection();
  }

  double chooseTurnDirection() const
  {
    const double left =
      std::isfinite(pointcloud_left_min_m_) ? pointcloud_left_min_m_ : pointcloud_front_turn_m_;
    const double right =
      std::isfinite(pointcloud_right_min_m_) ? pointcloud_right_min_m_ : pointcloud_front_turn_m_;
    return left >= right ? 1.0 : -1.0;
  }

  double headingCorrection() const
  {
    if (!have_start_yaw_ || !have_odom_ || max_turn_radps_ <= 0.0) {
      return 0.0;
    }
    const double error = normalizeAngle(current_yaw_ - start_yaw_);
    return clamp(-heading_hold_kp_ * error, -max_turn_radps_, max_turn_radps_);
  }

  double rangeOrZero(double range) const
  {
    return std::isfinite(range) ? range : 0.0;
  }

  void finish(bool ok, const std::string & reason)
  {
    succeeded_ = ok;
    finished_ = true;
    publishStop();
    if (ok) {
      RCLCPP_INFO(get_logger(), "Direct straight controller completed: %s", reason.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "Direct straight controller failed: %s", reason.c_str());
    }
  }

  void publishStop()
  {
    geometry_msgs::msg::Twist cmd;
    cmd_pub_->publish(cmd);
  }

  double distance_m_{1.0};
  double speed_mps_{0.20};
  double max_runtime_sec_{30.0};
  double control_hz_{10.0};
  double start_delay_sec_{1.0};
  std::string distance_source_{"path_s"};
  std::string cmd_vel_topic_{"/cmd_vel"};
  std::string path_s_topic_{"/go2/path_s"};
  std::string odom_topic_{"/odom"};
  std::string pointcloud_topic_{"/go2/pointcloud"};
  bool use_pointcloud_avoidance_{true};
  bool require_fresh_pointcloud_{true};
  double pointcloud_timeout_sec_{0.8};
  double pointcloud_front_stop_m_{0.42};
  double pointcloud_front_turn_m_{0.75};
  double pointcloud_side_clearance_m_{0.24};
  double pointcloud_front_angle_rad_{0.35};
  double pointcloud_side_min_angle_rad_{0.45};
  double pointcloud_side_max_angle_rad_{1.57};
  double pointcloud_min_height_m_{-0.20};
  double pointcloud_max_height_m_{0.90};
  double max_turn_radps_{0.40};
  double heading_hold_kp_{0.80};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr path_s_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time start_time_;
  rclcpp::Time motion_start_time_;
  bool started_{false};
  bool finished_{false};
  bool succeeded_{false};

  bool have_path_s_{false};
  double current_path_s_{0.0};
  double start_path_s_{0.0};
  bool have_odom_{false};
  double current_odom_x_{0.0};
  double current_odom_y_{0.0};
  double current_yaw_{0.0};
  double start_odom_x_{0.0};
  double start_odom_y_{0.0};
  double start_yaw_{0.0};
  bool have_start_yaw_{false};
  bool have_pointcloud_{false};
  rclcpp::Time last_pointcloud_time_{0, 0, RCL_ROS_TIME};
  double pointcloud_front_min_m_{std::numeric_limits<double>::infinity()};
  double pointcloud_left_min_m_{std::numeric_limits<double>::infinity()};
  double pointcloud_right_min_m_{std::numeric_limits<double>::infinity()};
  int pointcloud_valid_points_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Go2DirectStraightController>();
  rclcpp::Rate rate(20.0);
  while (rclcpp::ok() && !node->finished()) {
    rclcpp::spin_some(node);
    rate.sleep();
  }
  const int ret = node->succeeded() ? 0 : 1;
  rclcpp::shutdown();
  return ret;
}
