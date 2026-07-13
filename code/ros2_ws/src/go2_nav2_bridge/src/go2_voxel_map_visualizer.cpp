#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <unitree/idl/go2/VoxelMapCompressed_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

using namespace std::chrono_literals;

namespace
{
constexpr int kExpectedX = 128;
constexpr int kExpectedY = 128;
constexpr int kExpectedZ = 38;
constexpr std::size_t kExpectedDecodedBytes =
  static_cast<std::size_t>(kExpectedX) * kExpectedY * kExpectedZ / 8;

std::atomic_bool g_unitree_channel_factory_initialized{false};

void initUnitreeChannelFactoryOnce(const std::string & network_interface)
{
  if (network_interface.empty()) {
    throw std::runtime_error("network_interface is required, for example enp5s0");
  }

  bool expected = false;
  if (g_unitree_channel_factory_initialized.compare_exchange_strong(expected, true)) {
    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);
  }
}

std::vector<uint8_t> decompressLz4Block(
  const std::vector<uint8_t> & payload,
  std::size_t expected_size)
{
  std::size_t ip = 0;
  std::vector<uint8_t> out;
  out.reserve(expected_size);

  while (ip < payload.size()) {
    const uint8_t token = payload[ip++];

    std::size_t literal_len = token >> 4;
    if (literal_len == 15) {
      while (true) {
        if (ip >= payload.size()) {
          throw std::runtime_error("truncated LZ4 literal length");
        }
        const uint8_t value = payload[ip++];
        literal_len += value;
        if (value != 255) {
          break;
        }
      }
    }

    if (ip + literal_len > payload.size()) {
      throw std::runtime_error("LZ4 literal section exceeds payload");
    }
    out.insert(out.end(), payload.begin() + static_cast<std::ptrdiff_t>(ip),
      payload.begin() + static_cast<std::ptrdiff_t>(ip + literal_len));
    ip += literal_len;

    if (ip >= payload.size()) {
      break;
    }

    if (ip + 2 > payload.size()) {
      throw std::runtime_error("missing LZ4 match offset");
    }
    const std::size_t offset = static_cast<std::size_t>(payload[ip]) |
      (static_cast<std::size_t>(payload[ip + 1]) << 8);
    ip += 2;
    if (offset == 0 || offset > out.size()) {
      throw std::runtime_error("invalid LZ4 match offset");
    }

    std::size_t match_len = token & 0x0F;
    if (match_len == 15) {
      while (true) {
        if (ip >= payload.size()) {
          throw std::runtime_error("truncated LZ4 match length");
        }
        const uint8_t value = payload[ip++];
        match_len += value;
        if (value != 255) {
          break;
        }
      }
    }
    match_len += 4;

    for (std::size_t i = 0; i < match_len; ++i) {
      out.push_back(out[out.size() - offset]);
      if (out.size() > expected_size) {
        throw std::runtime_error("LZ4 output exceeds expected voxel bitmap size");
      }
    }
  }

  if (out.size() != expected_size) {
    throw std::runtime_error(
      "LZ4 output size " + std::to_string(out.size()) +
      " != expected " + std::to_string(expected_size));
  }

  return out;
}

void addPointField(
  sensor_msgs::msg::PointCloud2 & cloud,
  const std::string & name,
  uint32_t offset)
{
  sensor_msgs::msg::PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field.count = 1;
  cloud.fields.push_back(field);
}

void writeFloat32(std::vector<uint8_t> & data, std::size_t offset, float value)
{
  std::memcpy(data.data() + offset, &value, sizeof(float));
}
}  // namespace

class Go2VoxelMapVisualizer : public rclcpp::Node
{
public:
  Go2VoxelMapVisualizer()
  : Node("go2_voxel_map_visualizer")
  {
    network_interface_ = declare_parameter<std::string>("network_interface", "");
    voxel_topic_ = declare_parameter<std::string>(
      "voxel_topic", "rt/utlidar/voxel_map_compressed");
    points_topic_ = declare_parameter<std::string>(
      "points_topic", "/go2/voxel_map_points");
    marker_topic_ = declare_parameter<std::string>(
      "marker_topic", "/go2/voxel_map_marker");
    output_frame_ = declare_parameter<std::string>("output_frame", "odom");
    bit_order_ = declare_parameter<std::string>("bit_order", "lsb");
    publish_marker_ = declare_parameter<bool>("publish_marker", true);
    marker_max_voxels_ = declare_parameter<int>("marker_max_voxels", 60000);
    log_every_n_frames_ = declare_parameter<int>("log_every_n_frames", 20);

    if (bit_order_ != "lsb" && bit_order_ != "msb") {
      throw std::runtime_error("bit_order must be lsb or msb");
    }

    initUnitreeChannelFactoryOnce(network_interface_);

    points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      points_topic_, rclcpp::SensorDataQoS().reliable());
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      marker_topic_, rclcpp::QoS(1).reliable());

    voxel_sub_ =
      std::make_shared<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::VoxelMapCompressed_>>(
        voxel_topic_);
    voxel_sub_->InitChannel(
      [this](const void * message) { onVoxel(message); }, 20);

    RCLCPP_INFO(
      get_logger(),
      "Go2 voxel map visualizer started: iface=%s dds=%s points=%s marker=%s frame=%s bit_order=%s",
      network_interface_.c_str(), voxel_topic_.c_str(), points_topic_.c_str(),
      marker_topic_.c_str(), output_frame_.c_str(), bit_order_.c_str());
  }

private:
  void onVoxel(const void * message)
  {
    const auto & msg = *static_cast<const unitree_go::msg::dds_::VoxelMapCompressed_ *>(message);
    ++frame_count_;

    const int width_x = static_cast<int>(msg.width()[0]);
    const int width_y = static_cast<int>(msg.width()[1]);
    const int width_z = static_cast<int>(msg.width()[2]);
    const std::size_t src_size = static_cast<std::size_t>(msg.src_size());

    if (width_x != kExpectedX || width_y != kExpectedY || width_z != kExpectedZ ||
      src_size != kExpectedDecodedBytes)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Unexpected voxel metadata: width=[%d,%d,%d] src_size=%zu expected=[128,128,38]/77824",
        width_x, width_y, width_z, src_size);
    }

    std::vector<uint8_t> decoded;
    try {
      decoded = decompressLz4Block(msg.data(), kExpectedDecodedBytes);
    } catch (const std::exception & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Failed to decode raw LZ4 voxel block: %s data_size=%zu src_size=%zu",
        ex.what(), msg.data().size(), src_size);
      return;
    }

    const auto stamp = now();
    const std::string frame = output_frame_.empty() ? msg.frame_id() : output_frame_;
    const float resolution = static_cast<float>(msg.resolution());
    const float origin_x = static_cast<float>(msg.origin()[0]);
    const float origin_y = static_cast<float>(msg.origin()[1]);
    const float origin_z = static_cast<float>(msg.origin()[2]);

    std::vector<std::array<float, 4>> points;
    points.reserve(32000);

    const int total = kExpectedX * kExpectedY * kExpectedZ;
    for (int linear = 0; linear < total; ++linear) {
      const uint8_t byte = decoded[static_cast<std::size_t>(linear / 8)];
      const int bit_index = linear % 8;
      const int bit = bit_order_ == "msb" ?
        ((byte >> (7 - bit_index)) & 1) :
        ((byte >> bit_index) & 1);
      if (bit == 0) {
        continue;
      }

      const int ix = linear % kExpectedX;
      const int iy = (linear / kExpectedX) % kExpectedY;
      const int iz = linear / (kExpectedX * kExpectedY);
      const float x = origin_x + (static_cast<float>(ix) + 0.5f) * resolution;
      const float y = origin_y + (static_cast<float>(iy) + 0.5f) * resolution;
      const float z = origin_z + (static_cast<float>(iz) + 0.5f) * resolution;
      const float intensity = static_cast<float>(iz);
      points.push_back({x, y, z, intensity});
    }

    publishPointCloud(points, stamp, frame);
    if (publish_marker_) {
      publishMarker(points, stamp, frame, resolution);
    }

    if (log_every_n_frames_ > 0 && frame_count_ % log_every_n_frames_ == 0) {
      RCLCPP_INFO(
        get_logger(),
        "voxel frame=%zu compressed=%zu decoded=%zu occupied=%zu resolution=%.3f origin=(%.2f, %.2f, %.2f)",
        frame_count_, msg.data().size(), decoded.size(), points.size(),
        resolution, origin_x, origin_y, origin_z);
    }
  }

  void publishPointCloud(
    const std::vector<std::array<float, 4>> & points,
    const rclcpp::Time & stamp,
    const std::string & frame)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = frame;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(points.size());
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = 16;
    cloud.row_step = cloud.point_step * cloud.width;
    addPointField(cloud, "x", 0);
    addPointField(cloud, "y", 4);
    addPointField(cloud, "z", 8);
    addPointField(cloud, "intensity", 12);
    cloud.data.resize(static_cast<std::size_t>(cloud.row_step));

    for (std::size_t i = 0; i < points.size(); ++i) {
      const std::size_t base = i * cloud.point_step;
      writeFloat32(cloud.data, base + 0, points[i][0]);
      writeFloat32(cloud.data, base + 4, points[i][1]);
      writeFloat32(cloud.data, base + 8, points[i][2]);
      writeFloat32(cloud.data, base + 12, points[i][3]);
    }

    points_pub_->publish(cloud);
  }

  void publishMarker(
    const std::vector<std::array<float, 4>> & points,
    const rclcpp::Time & stamp,
    const std::string & frame,
    float resolution)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = frame;
    marker.ns = "go2_voxel_map";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = resolution;
    marker.scale.y = resolution;
    marker.scale.z = resolution;
    marker.color.a = 0.65f;
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.lifetime = rclcpp::Duration::from_seconds(0.3);

    const std::size_t limit = marker_max_voxels_ > 0 ?
      std::min<std::size_t>(points.size(), static_cast<std::size_t>(marker_max_voxels_)) :
      points.size();
    marker.points.reserve(limit);
    marker.colors.reserve(limit);
    for (std::size_t i = 0; i < limit; ++i) {
      geometry_msgs::msg::Point p;
      p.x = points[i][0];
      p.y = points[i][1];
      p.z = points[i][2];
      marker.points.push_back(p);

      std_msgs::msg::ColorRGBA color;
      const float h = points[i][3] / static_cast<float>(std::max(1, kExpectedZ - 1));
      color.a = 0.65f;
      color.r = std::min(1.0f, std::max(0.0f, h * 2.0f));
      color.g = std::min(1.0f, std::max(0.0f, 1.0f - std::fabs(h - 0.5f) * 2.0f));
      color.b = std::min(1.0f, std::max(0.0f, 1.0f - h * 2.0f));
      marker.colors.push_back(color);
    }

    marker_pub_->publish(marker);
  }

  std::string network_interface_;
  std::string voxel_topic_;
  std::string points_topic_;
  std::string marker_topic_;
  std::string output_frame_;
  std::string bit_order_{"lsb"};
  bool publish_marker_{true};
  int marker_max_voxels_{60000};
  int log_every_n_frames_{20};
  std::size_t frame_count_{0};

  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::VoxelMapCompressed_> voxel_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<Go2VoxelMapVisualizer>());
  } catch (const std::exception & ex) {
    std::cerr << "go2_voxel_map_visualizer failed: " << ex.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
