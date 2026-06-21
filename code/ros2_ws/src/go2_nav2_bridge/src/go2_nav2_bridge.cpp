#include <array>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <cstdlib>
#include <csignal>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/idl/ros2/PointCloud2_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/go2/obstacles_avoid/obstacles_avoid_client.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

using namespace std::chrono_literals;

namespace
{
std::atomic_bool g_unitree_channel_factory_initialized{false};
std::atomic_bool g_shutdown_requested{false};

double clamp(double value, double lower, double upper)
{
  return std::min(std::max(value, lower), upper);
}

double finite_or_zero(double value)
{
  return std::isfinite(value) ? value : 0.0;
}

std::array<double, 9> makeRotationMatrix(double roll, double pitch, double yaw)
{
  const double cr = std::cos(roll);
  const double sr = std::sin(roll);
  const double cp = std::cos(pitch);
  const double sp = std::sin(pitch);
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);

  // R = Rz(yaw) * Ry(pitch) * Rx(roll)
  return {
    cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
    sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
    -sp, cp * sr, cp * cr};
}

std::array<double, 3> vector3Param(
  const std::vector<double> & values, const std::string & name)
{
  if (values.size() != 3) {
    throw std::runtime_error(name + " must have exactly three values.");
  }
  return {values[0], values[1], values[2]};
}

struct XyzFieldOffsets
{
  int x{-1};
  int y{-1};
  int z{-1};
};

struct SelfFilterBox
{
  double min_x;
  double max_x;
  double min_y;
  double max_y;
  double min_z;
  double max_z;
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

std::vector<SelfFilterBox> parseSelfFilterBoxes(const std::vector<double> & values)
{
  if (values.size() % 6 != 0) {
    throw std::runtime_error("self_filter_boxes must contain 6 values per box.");
  }

  std::vector<SelfFilterBox> boxes;
  boxes.reserve(values.size() / 6);
  for (std::size_t i = 0; i < values.size(); i += 6) {
    SelfFilterBox box{
      values[i + 0], values[i + 1],
      values[i + 2], values[i + 3],
      values[i + 4], values[i + 5]};
    if (box.min_x > box.max_x || box.min_y > box.max_y || box.min_z > box.max_z) {
      throw std::runtime_error("self_filter_boxes entries must be ordered as min <= max.");
    }
    boxes.push_back(box);
  }
  return boxes;
}

void initUnitreeChannelFactoryOnce(const std::string & network_interface)
{
  if (network_interface.empty()) {
    throw std::runtime_error("go2_nav2_bridge requires parameter network_interface.");
  }

  bool expected = false;
  if (g_unitree_channel_factory_initialized.compare_exchange_strong(expected, true)) {
    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);
  }
}

void requestShutdown(int)
{
  g_shutdown_requested.store(true);
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
}
}  // namespace

class Go2Nav2Bridge : public rclcpp::Node
{
public:
  Go2Nav2Bridge()
  : Node("go2_nav2_bridge")
  {
    network_interface_ = declare_parameter<std::string>("network_interface", "");
    sport_state_topic_ = declare_parameter<std::string>("sport_state_topic", "rt/sportmodestate");
    unitree_point_cloud_topic_ = declare_parameter<std::string>("unitree_point_cloud_topic", "rt/utlidar/cloud");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    cmd_vel_stamped_ = declare_parameter<bool>("cmd_vel_stamped", false);
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/go2/imu");
    ros_point_cloud_topic_ = declare_parameter<std::string>("ros_point_cloud_topic", "/go2/pointcloud");
    path_s_topic_ = declare_parameter<std::string>("path_s_topic", "/go2/path_s");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    point_cloud_frame_ = declare_parameter<std::string>("point_cloud_frame", "base_link");
    point_cloud_transform_enabled_ = declare_parameter<bool>("point_cloud_transform_enabled", true);
    point_cloud_xyz_offset_ = vector3Param(
      declare_parameter<std::vector<double>>(
        "point_cloud_xyz_offset", std::vector<double>{0.0, 0.0, 0.0}),
      "point_cloud_xyz_offset");
    point_cloud_rpy_offset_ = vector3Param(
      declare_parameter<std::vector<double>>(
        "point_cloud_rpy_offset", std::vector<double>{0.0, 0.0, 0.0}),
      "point_cloud_rpy_offset");
    point_cloud_rotation_ = makeRotationMatrix(
      point_cloud_rpy_offset_[0], point_cloud_rpy_offset_[1], point_cloud_rpy_offset_[2]);
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    publish_point_cloud_ = declare_parameter<bool>("publish_point_cloud", true);
    stand_on_start_ = declare_parameter<bool>("stand_on_start", true);
    classic_walk_on_start_ = declare_parameter<bool>("classic_walk_on_start", false);
    speed_level_on_start_ = declare_parameter<int>("speed_level_on_start", -1);
    cmd_timeout_sec_ = declare_parameter<double>("cmd_timeout_sec", 0.5);
    control_watchdog_hz_ = declare_parameter<double>("control_watchdog_hz", 20.0);
    motion_client_ = declare_parameter<std::string>("motion_client", "sport");
    obstacle_avoid_switch_on_start_ = declare_parameter<bool>("obstacle_avoid_switch_on_start", false);
    obstacle_avoid_remote_api_on_start_ =
      declare_parameter<bool>("obstacle_avoid_remote_api_on_start", false);
    stop_on_shutdown_ = declare_parameter<bool>("stop_on_shutdown", true);
    disable_obstacle_avoid_on_shutdown_ =
      declare_parameter<bool>("disable_obstacle_avoid_on_shutdown", false);
    disable_remote_api_on_shutdown_ =
      declare_parameter<bool>("disable_remote_api_on_shutdown", true);
    shutdown_stop_repeats_ =
      static_cast<int>(clamp(declare_parameter<int>("shutdown_stop_repeats", 8), 1, 30));
    shutdown_stop_period_ms_ =
      static_cast<int>(clamp(declare_parameter<int>("shutdown_stop_period_ms", 100), 20, 1000));
    log_cmd_vel_ = declare_parameter<bool>("log_cmd_vel", false);
    log_odom_state_ = declare_parameter<bool>("log_odom_state", false);
    log_point_cloud_roi_ = declare_parameter<bool>("log_point_cloud_roi", false);
    point_cloud_roi_log_period_ms_ = static_cast<int>(
      1000.0 * clamp(declare_parameter<double>("point_cloud_roi_log_period_sec", 1.0), 0.2, 10.0));
    require_fresh_point_cloud_for_motion_ =
      declare_parameter<bool>("require_fresh_point_cloud_for_motion", false);
    point_cloud_motion_timeout_sec_ =
      clamp(declare_parameter<double>("point_cloud_motion_timeout_sec", 1.0), 0.2, 10.0);
    self_filter_enabled_ = declare_parameter<bool>("self_filter_enabled", true);
    self_filter_boxes_ = parseSelfFilterBoxes(
      declare_parameter<std::vector<double>>(
        "self_filter_boxes",
        std::vector<double>{
          -0.45, 0.45, -0.65, 0.65, -0.30, 0.90,
          0.45, 0.65, -0.22, 0.22, -0.30, 0.90}));
    max_vx_mps_ = declare_parameter<double>("max_vx_mps", 0.35);
    max_vy_mps_ = declare_parameter<double>("max_vy_mps", 0.20);
    max_wz_radps_ = declare_parameter<double>("max_wz_radps", 0.80);

    if (network_interface_.empty()) {
      throw std::runtime_error("go2_nav2_bridge requires parameter network_interface.");
    }

    initUnitreeChannelFactoryOnce(network_interface_);

    sport_client_ = std::make_unique<unitree::robot::go2::SportClient>();
    sport_client_->SetTimeout(2.0f);
    sport_client_->Init();

    if (motion_client_ == "obstacles_avoid") {
      obstacle_client_ = std::make_unique<unitree::robot::go2::ObstaclesAvoidClient>();
      obstacle_client_->SetTimeout(2.0f);
      obstacle_client_->Init();
      if (obstacle_avoid_remote_api_on_start_) {
        const int32_t ret = obstacle_client_->UseRemoteCommandFromApi(true);
        RCLCPP_INFO(get_logger(), "ObstaclesAvoid UseRemoteCommandFromApi(true) ret=%d", ret);
      }
      if (obstacle_avoid_switch_on_start_) {
        const int32_t ret = obstacle_client_->SwitchSet(true);
        RCLCPP_INFO(get_logger(), "ObstaclesAvoid SwitchSet(true) ret=%d", ret);
      }
    } else if (motion_client_ != "sport") {
      throw std::runtime_error("motion_client must be 'sport' or 'obstacles_avoid'.");
    }

    if (stand_on_start_) {
      const int32_t ret = sport_client_->BalanceStand();
      RCLCPP_INFO(get_logger(), "BalanceStand ret=%d", ret);
    }
    if (speed_level_on_start_ >= 0) {
      const int32_t ret = sport_client_->SpeedLevel(speed_level_on_start_);
      RCLCPP_INFO(get_logger(), "SpeedLevel(%d) ret=%d", speed_level_on_start_, ret);
    }
    if (classic_walk_on_start_) {
      const int32_t ret = sport_client_->ClassicWalk(true);
      RCLCPP_INFO(get_logger(), "ClassicWalk(true) ret=%d", ret);
    }

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::SensorDataQoS());
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, rclcpp::SensorDataQoS());
    path_s_pub_ = create_publisher<std_msgs::msg::Float32>(path_s_topic_, 10);
    point_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      ros_point_cloud_topic_, rclcpp::SensorDataQoS());
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    if (cmd_vel_stamped_) {
      cmd_vel_stamped_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        cmd_vel_topic_, 10,
        std::bind(&Go2Nav2Bridge::onTwistStamped, this, std::placeholders::_1));
    } else {
      cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic_, 10,
        std::bind(&Go2Nav2Bridge::onTwist, this, std::placeholders::_1));
    }

    sport_state_sub_.reset(
      new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(
        sport_state_topic_));
    sport_state_sub_->InitChannel(
      std::bind(&Go2Nav2Bridge::onSportState, this, std::placeholders::_1), 10);

    if (publish_point_cloud_) {
      point_cloud_sub_.reset(
        new unitree::robot::ChannelSubscriber<::sensor_msgs::msg::dds_::PointCloud2_>(
          unitree_point_cloud_topic_));
      point_cloud_sub_->InitChannel(
        std::bind(&Go2Nav2Bridge::onPointCloud, this, std::placeholders::_1), 1);
    }

    const double watchdog_hz = clamp(control_watchdog_hz_, 5.0, 100.0);
    watchdog_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / watchdog_hz)),
      std::bind(&Go2Nav2Bridge::watchdog, this));

    RCLCPP_INFO(
      get_logger(),
      "Go2 Nav2 bridge started. DDS interface=%s, cmd_vel=%s, odom=%s, pointcloud=%s, motion_client=%s",
      network_interface_.c_str(), cmd_vel_topic_.c_str(), odom_topic_.c_str(),
      ros_point_cloud_topic_.c_str(), motion_client_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "Point cloud: publish=%s frame=%s transform=%s xyz=(%.3f, %.3f, %.3f) rpy=(%.3f, %.3f, %.3f)",
      publish_point_cloud_ ? "true" : "false",
      point_cloud_frame_.c_str(),
      point_cloud_transform_enabled_ ? "true" : "false",
      point_cloud_xyz_offset_[0], point_cloud_xyz_offset_[1], point_cloud_xyz_offset_[2],
      point_cloud_rpy_offset_[0], point_cloud_rpy_offset_[1], point_cloud_rpy_offset_[2]);
  }

  ~Go2Nav2Bridge() override
  {
    shutdownMotion();
  }

private:
  void onTwist(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    sendVelocity(msg->linear.x, msg->linear.y, msg->angular.z);
  }

  void onTwistStamped(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    sendVelocity(msg->twist.linear.x, msg->twist.linear.y, msg->twist.angular.z);
  }

  void sendVelocity(double vx, double vy, double wz)
  {
    double safe_vx = clamp(finite_or_zero(vx), -max_vx_mps_, max_vx_mps_);
    double safe_vy = clamp(finite_or_zero(vy), -max_vy_mps_, max_vy_mps_);
    double safe_wz = clamp(finite_or_zero(wz), -max_wz_radps_, max_wz_radps_);
    const bool command_nonzero =
      std::fabs(safe_vx) > 1e-4 || std::fabs(safe_vy) > 1e-4 || std::fabs(safe_wz) > 1e-4;
    if (require_fresh_point_cloud_for_motion_ && command_nonzero && !pointCloudFresh()) {
      safe_vx = 0.0;
      safe_vy = 0.0;
      safe_wz = 0.0;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Blocking nonzero cmd_vel because /go2/pointcloud is stale or unavailable.");
    }

    std::lock_guard<std::mutex> lock(command_mutex_);
    if (motion_client_ == "obstacles_avoid" && obstacle_client_) {
      last_command_ret_ = obstacle_client_->Move(
        static_cast<float>(safe_vx),
        static_cast<float>(safe_vy),
        static_cast<float>(safe_wz));
    } else if (sport_client_) {
      last_command_ret_ = sport_client_->Move(
        static_cast<float>(safe_vx),
        static_cast<float>(safe_vy),
        static_cast<float>(safe_wz));
    }
    if (log_cmd_vel_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 500,
        "cmd_vel -> %s Move vx=%.3f vy=%.3f wz=%.3f ret=%d",
        motion_client_.c_str(), safe_vx, safe_vy, safe_wz, last_command_ret_);
    }
    if (last_command_ret_ != 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "%s Move returned %d", motion_client_.c_str(), last_command_ret_);
    }
    last_cmd_time_ = now();
    has_cmd_ = true;
    last_cmd_nonzero_ =
      std::fabs(safe_vx) > 1e-4 || std::fabs(safe_vy) > 1e-4 || std::fabs(safe_wz) > 1e-4;
  }

  bool pointCloudFresh() const
  {
    if (!publish_point_cloud_) {
      return true;
    }
    if (!have_point_cloud_) {
      return false;
    }
    return (now() - last_point_cloud_time_).seconds() <= point_cloud_motion_timeout_sec_;
  }

  void sendStop()
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    sendStopLocked();
  }

  void sendStopLocked()
  {
    if (!sport_client_ && !obstacle_client_) {
      return;
    }
    if (obstacle_client_) {
      obstacle_client_->Move(0.0f, 0.0f, 0.0f);
    }
    if (sport_client_) {
      sport_client_->Move(0.0f, 0.0f, 0.0f);
      sport_client_->StopMove();
    }
    last_cmd_nonzero_ = false;
  }

  void shutdownMotion()
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (!stop_on_shutdown_) {
      return;
    }
    for (int i = 0; i < shutdown_stop_repeats_; ++i) {
      sendStopLocked();
      std::this_thread::sleep_for(std::chrono::milliseconds(shutdown_stop_period_ms_));
    }
    if (obstacle_client_ && disable_remote_api_on_shutdown_) {
      const int32_t ret = obstacle_client_->UseRemoteCommandFromApi(false);
      RCLCPP_INFO(get_logger(), "ObstaclesAvoid UseRemoteCommandFromApi(false) ret=%d", ret);
    }
    if (obstacle_client_ && disable_obstacle_avoid_on_shutdown_) {
      const int32_t ret = obstacle_client_->SwitchSet(false);
      RCLCPP_INFO(get_logger(), "ObstaclesAvoid SwitchSet(false) ret=%d", ret);
    }
  }

  void watchdog()
  {
    if (!has_cmd_ || !last_cmd_nonzero_) {
      return;
    }
    if (require_fresh_point_cloud_for_motion_ && !pointCloudFresh()) {
      sendStop();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Pointcloud watchdog stop: /go2/pointcloud is stale or unavailable.");
      return;
    }
    const double age = (now() - last_cmd_time_).seconds();
    if (age > cmd_timeout_sec_) {
      sendStop();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "No cmd_vel for %.2f sec; sent StopMove. last_ret=%d", age, last_command_ret_);
    }
  }

  void onSportState(const void * message)
  {
    const auto state = *static_cast<const unitree_go::msg::dds_::SportModeState_ *>(message);
    const auto stamp = now();
    const auto & position = state.position();
    const auto & velocity = state.velocity();
    const auto & rpy = state.imu_state().rpy();
    const auto & gyro = state.imu_state().gyroscope();
    const auto & acc = state.imu_state().accelerometer();

    tf2::Quaternion quat;
    quat.setRPY(rpy[0], rpy[1], rpy[2]);
    quat.normalize();

    if (!have_path_origin_) {
      origin_x_ = position[0];
      origin_y_ = position[1];
      last_x_ = position[0];
      last_y_ = position[1];
      have_path_origin_ = true;
    } else {
      const double dx = position[0] - last_x_;
      const double dy = position[1] - last_y_;
      const double ds = std::hypot(dx, dy);
      if (std::isfinite(ds) && ds >= 0.001) {
        path_s_m_ += ds;
        last_x_ = position[0];
        last_y_ = position[1];
      }
    }

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = position[0];
    odom.pose.pose.position.y = position[1];
    odom.pose.pose.position.z = position[2];
    odom.pose.pose.orientation.x = quat.x();
    odom.pose.pose.orientation.y = quat.y();
    odom.pose.pose.orientation.z = quat.z();
    odom.pose.pose.orientation.w = quat.w();
    odom.twist.twist.linear.x = velocity[0];
    odom.twist.twist.linear.y = velocity[1];
    odom.twist.twist.linear.z = velocity[2];
    odom.twist.twist.angular.z = state.yaw_speed();
    odom_pub_->publish(odom);

    sensor_msgs::msg::Imu imu;
    imu.header.stamp = stamp;
    imu.header.frame_id = base_frame_;
    imu.orientation = odom.pose.pose.orientation;
    imu.angular_velocity.x = gyro[0];
    imu.angular_velocity.y = gyro[1];
    imu.angular_velocity.z = gyro[2];
    imu.linear_acceleration.x = acc[0];
    imu.linear_acceleration.y = acc[1];
    imu.linear_acceleration.z = acc[2];
    imu_pub_->publish(imu);

    std_msgs::msg::Float32 path_s;
    path_s.data = static_cast<float>(path_s_m_);
    path_s_pub_->publish(path_s);

    if (log_odom_state_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "sport_state path_s=%.3f pos=(%.3f, %.3f, %.3f) vel=(%.3f, %.3f, %.3f) yaw=%.3f mode=%d gait=%d error=%u",
        path_s_m_, position[0], position[1], position[2],
        velocity[0], velocity[1], velocity[2], rpy[2],
        static_cast<int>(state.mode()), static_cast<int>(state.gait_type()),
        state.error_code());
    }

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header.stamp = stamp;
      tf_msg.header.frame_id = odom_frame_;
      tf_msg.child_frame_id = base_frame_;
      tf_msg.transform.translation.x = odom.pose.pose.position.x;
      tf_msg.transform.translation.y = odom.pose.pose.position.y;
      tf_msg.transform.translation.z = odom.pose.pose.position.z;
      tf_msg.transform.rotation = odom.pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf_msg);
    }
  }

  void onPointCloud(const void * message)
  {
    const auto & cloud = *static_cast<const ::sensor_msgs::msg::dds_::PointCloud2_ *>(message);

    sensor_msgs::msg::PointCloud2 out;
    out.header.stamp = now();
    last_point_cloud_time_ = out.header.stamp;
    have_point_cloud_ = true;
    out.header.frame_id = point_cloud_frame_;
    out.height = cloud.height();
    out.width = cloud.width();
    out.is_bigendian = cloud.is_bigendian();
    out.point_step = cloud.point_step();
    out.row_step = cloud.row_step();
    out.is_dense = cloud.is_dense();
    out.data = cloud.data();
    out.fields.reserve(cloud.fields().size());
    for (const auto & field : cloud.fields()) {
      sensor_msgs::msg::PointField ros_field;
      ros_field.name = field.name();
      ros_field.offset = field.offset();
      ros_field.datatype = field.datatype();
      ros_field.count = field.count();
      out.fields.push_back(ros_field);
    }
    transformPointCloudInPlace(out);
    filterSelfPointsInPlace(out);
    logPointCloudRoi(out);
    point_cloud_pub_->publish(out);
  }

  void transformPointCloudInPlace(sensor_msgs::msg::PointCloud2 & cloud)
  {
    if (!point_cloud_transform_enabled_) {
      return;
    }

    const auto offsets = findXyzFields(cloud);
    if (!hasXyzFields(offsets)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "PointCloud2 has no float32 x/y/z fields; publishing without transform.");
      return;
    }

    const auto point_step = static_cast<std::size_t>(cloud.point_step);
    const auto row_step = static_cast<std::size_t>(cloud.row_step);
    const auto width = static_cast<std::size_t>(cloud.width);
    const auto height = static_cast<std::size_t>(cloud.height);
    const auto max_offset = static_cast<std::size_t>(
      std::max({offsets.x, offsets.y, offsets.z}) + static_cast<int>(sizeof(float)));
    if (point_step < max_offset) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "PointCloud2 point_step is too small for x/y/z fields; publishing without transform.");
      return;
    }

    for (std::size_t row = 0; row < height; ++row) {
      for (std::size_t col = 0; col < width; ++col) {
        const std::size_t index = row * row_step + col * point_step;
        if (index + max_offset > cloud.data.size()) {
          continue;
        }

        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        std::memcpy(&x, &cloud.data[index + static_cast<std::size_t>(offsets.x)], sizeof(float));
        std::memcpy(&y, &cloud.data[index + static_cast<std::size_t>(offsets.y)], sizeof(float));
        std::memcpy(&z, &cloud.data[index + static_cast<std::size_t>(offsets.z)], sizeof(float));
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
          continue;
        }

        const double tx =
          point_cloud_rotation_[0] * x + point_cloud_rotation_[1] * y +
          point_cloud_rotation_[2] * z + point_cloud_xyz_offset_[0];
        const double ty =
          point_cloud_rotation_[3] * x + point_cloud_rotation_[4] * y +
          point_cloud_rotation_[5] * z + point_cloud_xyz_offset_[1];
        const double tz =
          point_cloud_rotation_[6] * x + point_cloud_rotation_[7] * y +
          point_cloud_rotation_[8] * z + point_cloud_xyz_offset_[2];

        x = static_cast<float>(tx);
        y = static_cast<float>(ty);
        z = static_cast<float>(tz);
        std::memcpy(&cloud.data[index + static_cast<std::size_t>(offsets.x)], &x, sizeof(float));
        std::memcpy(&cloud.data[index + static_cast<std::size_t>(offsets.y)], &y, sizeof(float));
        std::memcpy(&cloud.data[index + static_cast<std::size_t>(offsets.z)], &z, sizeof(float));
      }
    }
  }

  void filterSelfPointsInPlace(sensor_msgs::msg::PointCloud2 & cloud)
  {
    if (!self_filter_enabled_ || self_filter_boxes_.empty()) {
      return;
    }

    const auto offsets = findXyzFields(cloud);
    if (!hasXyzFields(offsets)) {
      return;
    }

    const auto point_step = static_cast<std::size_t>(cloud.point_step);
    const auto row_step = static_cast<std::size_t>(cloud.row_step);
    const auto width = static_cast<std::size_t>(cloud.width);
    const auto height = static_cast<std::size_t>(cloud.height);
    const auto max_offset = static_cast<std::size_t>(
      std::max({offsets.x, offsets.y, offsets.z}) + static_cast<int>(sizeof(float)));
    if (point_step < max_offset) {
      return;
    }

    const float nan = std::numeric_limits<float>::quiet_NaN();
    int filtered_points = 0;
    for (std::size_t row = 0; row < height; ++row) {
      for (std::size_t col = 0; col < width; ++col) {
        const std::size_t index = row * row_step + col * point_step;
        if (index + max_offset > cloud.data.size()) {
          continue;
        }

        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        std::memcpy(&x, &cloud.data[index + static_cast<std::size_t>(offsets.x)], sizeof(float));
        std::memcpy(&y, &cloud.data[index + static_cast<std::size_t>(offsets.y)], sizeof(float));
        std::memcpy(&z, &cloud.data[index + static_cast<std::size_t>(offsets.z)], sizeof(float));
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
          continue;
        }

        bool inside_self = false;
        for (const auto & box : self_filter_boxes_) {
          if (x >= box.min_x && x <= box.max_x &&
            y >= box.min_y && y <= box.max_y &&
            z >= box.min_z && z <= box.max_z)
          {
            inside_self = true;
            break;
          }
        }
        if (!inside_self) {
          continue;
        }

        std::memcpy(&cloud.data[index + static_cast<std::size_t>(offsets.x)], &nan, sizeof(float));
        std::memcpy(&cloud.data[index + static_cast<std::size_t>(offsets.y)], &nan, sizeof(float));
        std::memcpy(&cloud.data[index + static_cast<std::size_t>(offsets.z)], &nan, sizeof(float));
        ++filtered_points;
      }
    }

    if (filtered_points > 0) {
      cloud.is_dense = false;
    }
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), point_cloud_roi_log_period_ms_,
      "pointcloud_self_filter removed=%d boxes=%zu",
      filtered_points, self_filter_boxes_.size());
  }

  void logPointCloudRoi(const sensor_msgs::msg::PointCloud2 & cloud)
  {
    if (!log_point_cloud_roi_) {
      return;
    }

    const auto offsets = findXyzFields(cloud);
    if (!hasXyzFields(offsets)) {
      return;
    }

    const auto point_step = static_cast<std::size_t>(cloud.point_step);
    const auto row_step = static_cast<std::size_t>(cloud.row_step);
    const auto width = static_cast<std::size_t>(cloud.width);
    const auto height = static_cast<std::size_t>(cloud.height);
    const auto max_offset = static_cast<std::size_t>(
      std::max({offsets.x, offsets.y, offsets.z}) + static_cast<int>(sizeof(float)));
    if (point_step < max_offset) {
      return;
    }

    struct Roi
    {
      const char * name;
      double min_x;
      double max_x;
      double min_y;
      double max_y;
      int count{0};
      double nearest{std::numeric_limits<double>::infinity()};
    };

    std::array<Roi, 4> rois{{
      {"front_center", 0.15, 0.90, -0.20, 0.20},
      {"front_left", 0.10, 0.90, 0.20, 0.65},
      {"front_right", 0.10, 0.90, -0.65, -0.20},
      {"near_body", -0.30, 0.30, -0.35, 0.35},
    }};

    int valid_points = 0;
    for (std::size_t row = 0; row < height; ++row) {
      for (std::size_t col = 0; col < width; ++col) {
        const std::size_t index = row * row_step + col * point_step;
        if (index + max_offset > cloud.data.size()) {
          continue;
        }

        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        std::memcpy(&x, &cloud.data[index + static_cast<std::size_t>(offsets.x)], sizeof(float));
        std::memcpy(&y, &cloud.data[index + static_cast<std::size_t>(offsets.y)], sizeof(float));
        std::memcpy(&z, &cloud.data[index + static_cast<std::size_t>(offsets.z)], sizeof(float));
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
          continue;
        }
        if (z < -0.20f || z > 0.90f) {
          continue;
        }

        ++valid_points;
        const double range = std::hypot(static_cast<double>(x), static_cast<double>(y));
        for (auto & roi : rois) {
          if (x >= roi.min_x && x <= roi.max_x && y >= roi.min_y && y <= roi.max_y) {
            ++roi.count;
            roi.nearest = std::min(roi.nearest, range);
          }
        }
      }
    }

    auto nearestOrZero = [](double nearest) {
      return std::isfinite(nearest) ? nearest : 0.0;
    };
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), point_cloud_roi_log_period_ms_,
      "pointcloud_roi valid=%d %s=%d/%.2fm %s=%d/%.2fm %s=%d/%.2fm %s=%d/%.2fm",
      valid_points,
      rois[0].name, rois[0].count, nearestOrZero(rois[0].nearest),
      rois[1].name, rois[1].count, nearestOrZero(rois[1].nearest),
      rois[2].name, rois[2].count, nearestOrZero(rois[2].nearest),
      rois[3].name, rois[3].count, nearestOrZero(rois[3].nearest));
  }

  std::string network_interface_;
  std::string sport_state_topic_;
  std::string unitree_point_cloud_topic_;
  std::string cmd_vel_topic_;
  bool cmd_vel_stamped_{false};
  std::string odom_topic_;
  std::string imu_topic_;
  std::string ros_point_cloud_topic_;
  std::string path_s_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string point_cloud_frame_;
  bool point_cloud_transform_enabled_{true};
  std::array<double, 3> point_cloud_xyz_offset_{0.0, 0.0, 0.0};
  std::array<double, 3> point_cloud_rpy_offset_{0.0, 0.0, 0.0};
  std::array<double, 9> point_cloud_rotation_{
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0};
  bool publish_tf_{true};
  bool publish_point_cloud_{true};
  bool stand_on_start_{true};
  bool classic_walk_on_start_{false};
  int speed_level_on_start_{-1};
  std::string motion_client_{"sport"};
  bool obstacle_avoid_switch_on_start_{false};
  bool obstacle_avoid_remote_api_on_start_{false};
  bool stop_on_shutdown_{true};
  bool disable_obstacle_avoid_on_shutdown_{false};
  bool disable_remote_api_on_shutdown_{true};
  int shutdown_stop_repeats_{8};
  int shutdown_stop_period_ms_{100};
  bool log_cmd_vel_{false};
  bool log_odom_state_{false};
  bool log_point_cloud_roi_{false};
  int point_cloud_roi_log_period_ms_{1000};
  bool require_fresh_point_cloud_for_motion_{false};
  double point_cloud_motion_timeout_sec_{1.0};
  bool self_filter_enabled_{true};
  std::vector<SelfFilterBox> self_filter_boxes_;
  double cmd_timeout_sec_{0.5};
  double control_watchdog_hz_{20.0};
  double max_vx_mps_{0.35};
  double max_vy_mps_{0.20};
  double max_wz_radps_{0.80};

  std::unique_ptr<unitree::robot::go2::SportClient> sport_client_;
  std::unique_ptr<unitree::robot::go2::ObstaclesAvoidClient> obstacle_client_;
  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> sport_state_sub_;
  unitree::robot::ChannelSubscriberPtr<::sensor_msgs::msg::dds_::PointCloud2_> point_cloud_sub_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr path_s_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_stamped_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  std::mutex command_mutex_;
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  bool has_cmd_{false};
  bool last_cmd_nonzero_{false};
  int32_t last_command_ret_{0};
  bool have_point_cloud_{false};
  rclcpp::Time last_point_cloud_time_{0, 0, RCL_ROS_TIME};

  bool have_path_origin_{false};
  double origin_x_{0.0};
  double origin_y_{0.0};
  double last_x_{0.0};
  double last_y_{0.0};
  double path_s_m_{0.0};
};

int main(int argc, char ** argv)
{
  try {
    std::signal(SIGINT, requestShutdown);
    std::signal(SIGTERM, requestShutdown);
    const char * preinit_network_interface =
      std::getenv("GO2_NAV2_BRIDGE_NETWORK_INTERFACE");
    if (preinit_network_interface != nullptr && std::strlen(preinit_network_interface) > 0) {
      initUnitreeChannelFactoryOnce(preinit_network_interface);
    }

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Go2Nav2Bridge>());
  } catch (const std::exception & error) {
    std::cerr << error.what() << std::endl;
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
