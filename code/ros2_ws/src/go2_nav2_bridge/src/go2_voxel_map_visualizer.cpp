#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <unitree/idl/go2/VoxelMapCompressed_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

namespace
{
std::atomic_bool g_unitree_channel_factory_initialized{false};

void initUnitreeChannelFactoryOnce(const std::string & network_interface)
{
  if (network_interface.empty()) {
    throw std::runtime_error("go2_voxel_map_visualizer requires parameter network_interface.");
  }

  bool expected = false;
  if (g_unitree_channel_factory_initialized.compare_exchange_strong(expected, true)) {
    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);
  }
}

double clamp(double value, double lower, double upper)
{
  return std::min(std::max(value, lower), upper);
}

std::vector<uint8_t> decompressLz4Block(
  const std::vector<uint8_t> & input, std::size_t expected_size)
{
  std::size_t ip = 0;
  std::vector<uint8_t> output;
  output.reserve(expected_size);

  auto readExtendedLength = [&input, &ip](std::size_t base) {
      std::size_t length = base;
      if (base == 15) {
        while (true) {
          if (ip >= input.size()) {
            throw std::runtime_error("truncated LZ4 extended length");
          }
          const auto value = static_cast<std::size_t>(input[ip++]);
          length += value;
          if (value != 255) {
            break;
          }
        }
      }
      return length;
    };

  while (ip < input.size()) {
    const uint8_t token = input[ip++];

    const std::size_t literal_length = readExtendedLength(token >> 4);
    if (ip + literal_length > input.size()) {
      throw std::runtime_error("LZ4 literal section exceeds payload");
    }
    output.insert(output.end(), input.begin() + static_cast<std::ptrdiff_t>(ip),
      input.begin() + static_cast<std::ptrdiff_t>(ip + literal_length));
    ip += literal_length;

    if (ip >= input.size()) {
      break;
    }
    if (ip + 2 > input.size()) {
      throw std::runtime_error("LZ4 match offset is missing");
    }

    const std::size_t offset =
      static_cast<std::size_t>(input[ip]) |
      (static_cast<std::size_t>(input[ip + 1]) << 8);
    ip += 2;
    if (offset == 0 || offset > output.size()) {
      throw std::runtime_error("invalid LZ4 match offset");
    }

    std::size_t match_length = readExtendedLength(token & 0x0F) + 4;
    while (match_length-- > 0) {
      output.push_back(output[output.size() - offset]);
      if (output.size() > expected_size) {
        throw std::runtime_error("LZ4 decoded output exceeds expected size");
      }
    }
  }

  if (output.size() != expected_size) {
    throw std::runtime_error(
            "LZ4 decoded size " + std::to_string(output.size()) +
            " != expected " + std::to_string(expected_size));
  }
  return output;
}

std_msgs::msg::ColorRGBA heightColor(double t, double alpha)
{
  t = clamp(t, 0.0, 1.0);

  std_msgs::msg::ColorRGBA color;
  color.a = static_cast<float>(alpha);

  auto lerp = [](double a, double b, double u) {
      return static_cast<float>(a + (b - a) * u);
    };

  if (t < 0.35) {
    const double u = t / 0.35;
    color.r = 0.0f;
    color.g = 1.0f;
    color.b = lerp(1.0, 0.0, u);
  } else if (t < 0.60) {
    const double u = (t - 0.35) / 0.25;
    color.r = lerp(0.0, 1.0, u);
    color.g = 1.0f;
    color.b = 0.0f;
  } else if (t < 0.80) {
    const double u = (t - 0.60) / 0.20;
    color.r = 1.0f;
    color.g = lerp(1.0, 0.45, u);
    color.b = 0.0f;
  } else {
    const double u = (t - 0.80) / 0.20;
    color.r = 1.0f;
    color.g = lerp(0.45, 0.0, u);
    color.b = lerp(0.0, 1.0, u);
  }
  return color;
}

sensor_msgs::msg::PointField makeFloatField(const std::string & name, uint32_t offset)
{
  sensor_msgs::msg::PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field.count = 1;
  return field;
}
}  // namespace

class Go2VoxelMapVisualizer : public rclcpp::Node
{
public:
  Go2VoxelMapVisualizer()
  : Node("go2_voxel_map_visualizer")
  {
    network_interface_ = declare_parameter<std::string>("network_interface", "enp5s0");
    voxel_topic_ = declare_parameter<std::string>(
      "voxel_topic", "rt/utlidar/voxel_map_compressed");
    points_topic_ = declare_parameter<std::string>("points_topic", "/go2/voxel_map_points");
    marker_topic_ = declare_parameter<std::string>("marker_topic", "/go2/voxel_map_marker");
    publish_points_ = declare_parameter<bool>("publish_points", true);
    publish_marker_ = declare_parameter<bool>("publish_marker", true);
    bit_order_ = declare_parameter<std::string>("bit_order", "lsb");
    downsample_stride_ = std::max<int>(1, declare_parameter<int>("downsample_stride", 1));
    marker_alpha_ = clamp(declare_parameter<double>("marker_alpha", 0.85), 0.05, 1.0);
    marker_scale_multiplier_ =
      clamp(declare_parameter<double>("marker_scale_multiplier", 1.0), 0.1, 3.0);
    marker_lifetime_sec_ =
      clamp(declare_parameter<double>("marker_lifetime_sec", 0.25), 0.0, 5.0);
    min_z_ = declare_parameter<double>("min_z", -1000.0);
    max_z_ = declare_parameter<double>("max_z", 1000.0);

    if (bit_order_ != "lsb" && bit_order_ != "msb") {
      throw std::runtime_error("bit_order must be 'lsb' or 'msb'.");
    }
    if (min_z_ > max_z_) {
      throw std::runtime_error("min_z must be <= max_z.");
    }

    initUnitreeChannelFactoryOnce(network_interface_);

    if (publish_points_) {
      points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        points_topic_, rclcpp::QoS(5).reliable());
    }
    if (publish_marker_) {
      marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
        marker_topic_, rclcpp::QoS(5).reliable());
    }

    voxel_sub_.reset(
      new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::VoxelMapCompressed_>(
        voxel_topic_));
    voxel_sub_->InitChannel(
      std::bind(&Go2VoxelMapVisualizer::onVoxelMap, this, std::placeholders::_1), 5);

    RCLCPP_INFO(
      get_logger(),
      "Go2 voxel visualizer started. DDS interface=%s voxel_topic=%s points=%s marker=%s bit_order=%s",
      network_interface_.c_str(), voxel_topic_.c_str(), points_topic_.c_str(),
      marker_topic_.c_str(), bit_order_.c_str());
  }

private:
  struct VoxelPoint
  {
    float x;
    float y;
    float z;
    float intensity;
  };

  void onVoxelMap(const void * message)
  {
    const auto & msg =
      *static_cast<const unitree_go::msg::dds_::VoxelMapCompressed_ *>(message);

    const auto width = msg.width();
    const std::size_t wx = width[0];
    const std::size_t wy = width[1];
    const std::size_t wz = width[2];
    const std::size_t total_voxels = wx * wy * wz;
    const std::size_t expected_bitpacked_size = (total_voxels + 7) / 8;
    const std::size_t expected_size = static_cast<std::size_t>(msg.src_size());

    if (expected_size != expected_bitpacked_size) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "VoxelMapCompressed src_size=%zu but width product expects %zu bytes.",
        expected_size, expected_bitpacked_size);
    }

    std::vector<uint8_t> decoded;
    try {
      decoded = decompressLz4Block(msg.data(), expected_size);
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Failed to decode VoxelMapCompressed as raw LZ4 block: %s", error.what());
      return;
    }

    std::vector<VoxelPoint> points;
    points.reserve(std::min<std::size_t>(total_voxels, 20000));

    const auto & origin = msg.origin();
    const double resolution = msg.resolution();
    int occupied_count = 0;
    for (std::size_t linear = 0; linear < total_voxels; linear += downsample_stride_) {
      const uint8_t byte = decoded[linear / 8];
      const int bit_index = static_cast<int>(linear % 8);
      const bool occupied = bit_order_ == "msb" ?
        ((byte >> (7 - bit_index)) & 0x01) != 0 :
        ((byte >> bit_index) & 0x01) != 0;
      if (!occupied) {
        continue;
      }

      const std::size_t ix = linear % wx;
      const std::size_t iy = (linear / wx) % wy;
      const std::size_t iz = linear / (wx * wy);
      const double z = origin[2] + (static_cast<double>(iz) + 0.5) * resolution;
      if (z < min_z_ || z > max_z_) {
        continue;
      }

      VoxelPoint point;
      point.x = static_cast<float>(origin[0] + (static_cast<double>(ix) + 0.5) * resolution);
      point.y = static_cast<float>(origin[1] + (static_cast<double>(iy) + 0.5) * resolution);
      point.z = static_cast<float>(z);
      point.intensity = static_cast<float>(iz) / static_cast<float>(std::max<std::size_t>(1, wz - 1));
      points.push_back(point);
      ++occupied_count;
    }

    const auto stamp = now();
    if (points_pub_) {
      publishPointCloud(stamp, msg.frame_id(), points);
    }
    if (marker_pub_) {
      publishMarker(stamp, msg.frame_id(), points, resolution);
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "voxel_map stamp=%.3f frame=%s resolution=%.3f width=%ux%ux%u compressed=%zu decoded=%zu occupied=%d published=%zu",
      msg.stamp(), msg.frame_id().c_str(), resolution,
      width[0], width[1], width[2], msg.data().size(), decoded.size(),
      occupied_count, points.size());
  }

  void publishPointCloud(
    const rclcpp::Time & stamp, const std::string & frame_id,
    const std::vector<VoxelPoint> & points)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = frame_id;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(points.size());
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = 16;
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields = {
      makeFloatField("x", 0),
      makeFloatField("y", 4),
      makeFloatField("z", 8),
      makeFloatField("intensity", 12)};
    cloud.data.resize(static_cast<std::size_t>(cloud.row_step));

    for (std::size_t i = 0; i < points.size(); ++i) {
      const std::size_t base = i * cloud.point_step;
      std::memcpy(&cloud.data[base + 0], &points[i].x, sizeof(float));
      std::memcpy(&cloud.data[base + 4], &points[i].y, sizeof(float));
      std::memcpy(&cloud.data[base + 8], &points[i].z, sizeof(float));
      std::memcpy(&cloud.data[base + 12], &points[i].intensity, sizeof(float));
    }
    points_pub_->publish(cloud);
  }

  void publishMarker(
    const rclcpp::Time & stamp, const std::string & frame_id,
    const std::vector<VoxelPoint> & points, double resolution)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = frame_id;
    marker.ns = "go2_voxel_map";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = resolution * marker_scale_multiplier_;
    marker.scale.y = resolution * marker_scale_multiplier_;
    marker.scale.z = resolution * marker_scale_multiplier_;
    marker.lifetime.sec = static_cast<int32_t>(std::floor(marker_lifetime_sec_));
    marker.lifetime.nanosec = static_cast<uint32_t>(
      (marker_lifetime_sec_ - static_cast<double>(marker.lifetime.sec)) * 1e9);
    marker.points.reserve(points.size());
    marker.colors.reserve(points.size());

    for (const auto & voxel : points) {
      geometry_msgs::msg::Point point;
      point.x = voxel.x;
      point.y = voxel.y;
      point.z = voxel.z;
      marker.points.push_back(point);
      marker.colors.push_back(heightColor(voxel.intensity, marker_alpha_));
    }

    marker_pub_->publish(marker);
  }

  std::string network_interface_;
  std::string voxel_topic_;
  std::string points_topic_;
  std::string marker_topic_;
  bool publish_points_{true};
  bool publish_marker_{true};
  std::string bit_order_;
  int downsample_stride_{1};
  double marker_alpha_{0.85};
  double marker_scale_multiplier_{1.0};
  double marker_lifetime_sec_{0.25};
  double min_z_{-1000.0};
  double max_z_{1000.0};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::VoxelMapCompressed_> voxel_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go2VoxelMapVisualizer>());
  rclcpp::shutdown();
  return 0;
}
