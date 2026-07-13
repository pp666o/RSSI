#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

class Go2PosePathVisualizer : public rclcpp::Node
{
public:
  Go2PosePathVisualizer()
  : Node("go2_pose_path_visualizer")
  {
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/utlidar/robot_pose");
    path_topic_ = declare_parameter<std::string>("path_topic", "/go2/mapping_path");
    marker_topic_ = declare_parameter<std::string>("marker_topic", "/go2/mapping_pose_marker");
    output_frame_ = declare_parameter<std::string>("output_frame", "odom");
    min_distance_m_ = declare_parameter<double>("min_distance_m", 0.05);
    max_path_points_ = declare_parameter<int>("max_path_points", 20000);

    path_.header.frame_id = output_frame_;
    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, rclcpp::QoS(1).transient_local());
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(marker_topic_, 10);
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_, rclcpp::SensorDataQoS(),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        onPose(*msg);
      });

    RCLCPP_INFO(
      get_logger(),
      "Go2 pose path visualizer started: pose=%s path=%s marker=%s frame=%s min_distance=%.3f",
      pose_topic_.c_str(), path_topic_.c_str(), marker_topic_.c_str(),
      output_frame_.c_str(), min_distance_m_);
  }

private:
  void onPose(const geometry_msgs::msg::PoseStamped & msg)
  {
    geometry_msgs::msg::PoseStamped pose = msg;
    if (!output_frame_.empty()) {
      pose.header.frame_id = output_frame_;
    }

    if (!have_last_pose_ || distanceFromLast(pose) >= min_distance_m_) {
      appendPose(pose);
      have_last_pose_ = true;
      last_pose_ = pose;
    }

    path_.header.stamp = pose.header.stamp;
    path_pub_->publish(path_);
    publishPoseMarker(pose);
  }

  double distanceFromLast(const geometry_msgs::msg::PoseStamped & pose) const
  {
    const double dx = pose.pose.position.x - last_pose_.pose.position.x;
    const double dy = pose.pose.position.y - last_pose_.pose.position.y;
    const double dz = pose.pose.position.z - last_pose_.pose.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  void appendPose(const geometry_msgs::msg::PoseStamped & pose)
  {
    path_.poses.push_back(pose);
    if (max_path_points_ > 0 &&
      path_.poses.size() > static_cast<std::size_t>(max_path_points_))
    {
      path_.poses.erase(path_.poses.begin());
    }
  }

  void publishPoseMarker(const geometry_msgs::msg::PoseStamped & pose)
  {
    visualization_msgs::msg::Marker marker;
    marker.header = pose.header;
    marker.ns = "go2_mapping_pose";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = pose.pose;
    marker.scale.x = 0.55;
    marker.scale.y = 0.10;
    marker.scale.z = 0.10;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.15;
    marker.color.b = 0.05;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker_pub_->publish(marker);
  }

  std::string pose_topic_;
  std::string path_topic_;
  std::string marker_topic_;
  std::string output_frame_;
  double min_distance_m_{0.05};
  int max_path_points_{20000};

  bool have_last_pose_{false};
  geometry_msgs::msg::PoseStamped last_pose_;
  nav_msgs::msg::Path path_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go2PosePathVisualizer>());
  rclcpp::shutdown();
  return 0;
}
