#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/go2/obstacles_avoid/obstacles_avoid_client.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using namespace unitree::robot;

namespace
{
std::atomic<bool> running{true};

void HandleSignal(int)
{
    running.store(false);
}

double Clamp(double value, double lower, double upper)
{
    return std::min(std::max(value, lower), upper);
}

double WrapAngle(double angle)
{
    while (angle > M_PI)
    {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI)
    {
        angle += 2.0 * M_PI;
    }
    return angle;
}

double ParseDouble(const char * text, const std::string & name)
{
    try
    {
        std::size_t used = 0;
        const double value = std::stod(text, &used);
        if (used != std::string(text).size() || !std::isfinite(value))
        {
            throw std::invalid_argument("not finite");
        }
        return value;
    }
    catch (const std::exception &)
    {
        throw std::runtime_error("invalid " + name + ": " + text);
    }
}

std::string NowForLog()
{
    const auto now = std::chrono::system_clock::now();
    const auto t = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    localtime_r(&t, &tm);
    std::ostringstream out;
    out << std::put_time(&tm, "%F %T");
    return out.str();
}

struct Waypoint
{
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
};

struct RobotState
{
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double yaw{0.0};
    double vx{0.0};
    double vy{0.0};
    double wz{0.0};
    uint32_t error_code{0};
};

std::vector<double> ParseNumbersFromFile(const std::string & path)
{
    std::ifstream file(path);
    if (!file)
    {
        throw std::runtime_error("failed to open route file: " + path);
    }

    std::ostringstream buffer;
    buffer << file.rdbuf();
    std::string text = buffer.str();
    for (char & ch : text)
    {
        const bool numeric =
            (ch >= '0' && ch <= '9') || ch == '-' || ch == '+' || ch == '.' ||
            ch == 'e' || ch == 'E';
        if (!numeric)
        {
            ch = ' ';
        }
    }

    std::vector<double> values;
    std::stringstream stream(text);
    double value = 0.0;
    while (stream >> value)
    {
        values.push_back(value);
    }
    if (values.empty() || values.size() % 3 != 0)
    {
        throw std::runtime_error(
            "route file must contain x,y,yaw triples, e.g. 0.5,0,0,1.0,0,0");
    }
    return values;
}

std::vector<Waypoint> ParseRouteFile(const std::string & path)
{
    const std::vector<double> values = ParseNumbersFromFile(path);
    std::vector<Waypoint> route;
    route.reserve(values.size() / 3);
    for (std::size_t i = 0; i < values.size(); i += 3)
    {
        route.push_back(Waypoint{values[i], values[i + 1], values[i + 2]});
    }
    return route;
}

double RouteLength(const std::vector<Waypoint> & route)
{
    double length = 0.0;
    double prev_x = 0.0;
    double prev_y = 0.0;
    for (const auto & point : route)
    {
        length += std::hypot(point.x - prev_x, point.y - prev_y);
        prev_x = point.x;
        prev_y = point.y;
    }
    return length;
}

struct SegmentMetrics
{
    double distance_to_target{0.0};
    double segment_length{0.0};
    double projection_ratio{0.0};
    double cross_track{0.0};
};

SegmentMetrics ComputeSegmentMetrics(
    const RobotState & state, const Waypoint & prev, const Waypoint & target)
{
    SegmentMetrics metrics;
    const double sx = target.x - prev.x;
    const double sy = target.y - prev.y;
    const double px = state.x - prev.x;
    const double py = state.y - prev.y;
    metrics.segment_length = std::hypot(sx, sy);
    metrics.distance_to_target = std::hypot(target.x - state.x, target.y - state.y);
    if (metrics.segment_length > 1e-6)
    {
        metrics.projection_ratio = (px * sx + py * sy) / (metrics.segment_length * metrics.segment_length);
        metrics.cross_track = (px * sy - py * sx) / metrics.segment_length;
    }
    return metrics;
}
} // namespace

class RouteReplay
{
public:
    RouteReplay(
        std::vector<Waypoint> relative_route,
        std::string output_csv,
        double xy_tolerance_m,
        double max_vx_mps,
        double max_vy_mps,
        double max_wz_radps,
        double yaw_tolerance_rad,
        double timeout_sec,
        std::string motion_client,
        double control_hz)
        : relative_route_(std::move(relative_route)),
          output_csv_(std::move(output_csv)),
          xy_tolerance_m_(xy_tolerance_m),
          max_vx_mps_(max_vx_mps),
          max_vy_mps_(max_vy_mps),
          max_wz_radps_(max_wz_radps),
          yaw_tolerance_rad_(yaw_tolerance_rad),
          timeout_sec_(timeout_sec),
          motion_client_(std::move(motion_client)),
          control_period_(std::chrono::duration<double>(1.0 / control_hz))
    {
        if (relative_route_.empty())
        {
            throw std::runtime_error("route is empty");
        }
        if (motion_client_ != "sport" && motion_client_ != "obstacles_avoid")
        {
            throw std::runtime_error("motion_client must be sport or obstacles_avoid");
        }
    }

    void Init()
    {
        sport_client_.SetTimeout(2.0f);
        sport_client_.Init();

        if (motion_client_ == "obstacles_avoid")
        {
            obstacle_client_.SetTimeout(2.0f);
            obstacle_client_.Init();
            const int32_t remote_ret = obstacle_client_.UseRemoteCommandFromApi(true);
            const int32_t switch_ret = obstacle_client_.SwitchSet(true);
            std::cout << "ObstaclesAvoid UseRemoteCommandFromApi(true) ret=" << remote_ret
                      << " SwitchSet(true) ret=" << switch_ret << std::endl;
        }

        state_sub_.reset(new ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>("rt/sportmodestate"));
        state_sub_->InitChannel(
            std::bind(&RouteReplay::OnSportState, this, std::placeholders::_1), 10);

        if (!output_csv_.empty() && output_csv_ != "none")
        {
            log_.open(output_csv_, std::ios::out | std::ios::trunc);
            if (!log_)
            {
                throw std::runtime_error("failed to open output csv: " + output_csv_);
            }
            log_ << "wall_time,elapsed_sec,phase,target_index,target_count,x,y,z,yaw,"
                    "target_x,target_y,target_yaw,distance_m,projection_ratio,cross_track_m,"
                    "cmd_vx,cmd_vy,cmd_wz,move_ret,error_code\n";
        }
    }

    int Run()
    {
        std::cout << "Waiting for rt/sportmodestate..." << std::endl;
        const auto wait_start = std::chrono::steady_clock::now();
        while (running.load() && !HaveState())
        {
            if ((std::chrono::steady_clock::now() - wait_start) > std::chrono::seconds(5))
            {
                std::cerr << "Timed out waiting for rt/sportmodestate." << std::endl;
                return 2;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        sport_client_.BalanceStand();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        const RobotState origin = GetState();
        BuildWorldRoute(origin);
        const double route_length = RouteLength(relative_route_);
        std::cout << "Route replay started." << std::endl;
        std::cout << "origin x=" << origin.x << " y=" << origin.y << " yaw=" << origin.yaw << std::endl;
        std::cout << "waypoints=" << world_route_.size()
                  << " route_length_m=" << route_length
                  << " xy_tolerance_m=" << xy_tolerance_m_
                  << " yaw_tolerance_rad=" << yaw_tolerance_rad_
                  << " motion_client=" << motion_client_ << std::endl;

        const auto start = std::chrono::steady_clock::now();
        std::size_t target_index = 0;
        int last_ret = 0;
        bool finished = false;
        while (running.load())
        {
            const double elapsed =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
            if (elapsed > timeout_sec_)
            {
                std::cerr << "Route replay timeout after " << elapsed << " sec." << std::endl;
                break;
            }

            RobotState state = GetState();
            while (target_index < world_route_.size())
            {
                const Waypoint prev =
                    target_index == 0 ? Waypoint{origin.x, origin.y, origin.yaw}
                                      : world_route_[target_index - 1];
                const Waypoint target = world_route_[target_index];
                const SegmentMetrics metrics = ComputeSegmentMetrics(state, prev, target);
                const bool final_target = target_index + 1 >= world_route_.size();
                const double target_yaw_error = WrapAngle(target.yaw - state.yaw);
                const bool near_target = metrics.distance_to_target <= xy_tolerance_m_;
                const bool passed_target =
                    metrics.segment_length > 1e-6 &&
                    metrics.projection_ratio >= 1.0 &&
                    std::fabs(metrics.cross_track) <= xy_tolerance_m_;
                const bool yaw_ok = !final_target || std::fabs(target_yaw_error) <= yaw_tolerance_rad_;
                if ((!near_target && !passed_target) || !yaw_ok)
                {
                    break;
                }

                Log(elapsed, "reached", target_index, state, target, metrics, 0.0, 0.0, 0.0, last_ret);
                std::cout << "Reached waypoint " << (target_index + 1) << "/"
                          << world_route_.size()
                          << " dist=" << metrics.distance_to_target
                          << " cross_track=" << metrics.cross_track
                          << " projection=" << metrics.projection_ratio << std::endl;
                ++target_index;
            }

            if (target_index >= world_route_.size())
            {
                finished = true;
                break;
            }

            const Waypoint target = world_route_[target_index];
            const Waypoint prev =
                target_index == 0 ? Waypoint{origin.x, origin.y, origin.yaw}
                                  : world_route_[target_index - 1];
            const SegmentMetrics metrics = ComputeSegmentMetrics(state, prev, target);
            double vx = 0.0;
            double vy = 0.0;
            double wz = 0.0;
            ComputeCommand(state, target_index, target, metrics, vx, vy, wz);
            last_ret = SendMove(vx, vy, wz);
            Log(elapsed, "track", target_index, state, target, metrics, vx, vy, wz, last_ret);

            const auto sleep_until = std::chrono::steady_clock::now() + control_period_;
            std::this_thread::sleep_until(sleep_until);
        }

        Stop();
        if (finished)
        {
            std::cout << "All waypoints reached." << std::endl;
            return 0;
        }
        std::cout << "Route replay stopped before all waypoints were reached." << std::endl;
        return 1;
    }

private:
    void OnSportState(const void * message)
    {
        const auto state = *static_cast<const unitree_go::msg::dds_::SportModeState_ *>(message);
        RobotState next;
        next.x = state.position()[0];
        next.y = state.position()[1];
        next.z = state.position()[2];
        next.yaw = state.imu_state().rpy()[2];
        next.vx = state.velocity()[0];
        next.vy = state.velocity()[1];
        next.wz = state.yaw_speed();
        next.error_code = state.error_code();
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            latest_state_ = next;
            have_state_ = true;
        }
    }

    bool HaveState() const
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return have_state_;
    }

    RobotState GetState() const
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return latest_state_;
    }

    void BuildWorldRoute(const RobotState & origin)
    {
        world_route_.clear();
        world_route_.reserve(relative_route_.size());
        const double c = std::cos(origin.yaw);
        const double s = std::sin(origin.yaw);
        for (const auto & relative : relative_route_)
        {
            world_route_.push_back(Waypoint{
                origin.x + c * relative.x - s * relative.y,
                origin.y + s * relative.x + c * relative.y,
                WrapAngle(origin.yaw + relative.yaw)});
        }
    }

    void ComputeCommand(
        const RobotState & state,
        std::size_t target_index,
        const Waypoint & target,
        const SegmentMetrics & metrics,
        double & vx,
        double & vy,
        double & wz) const
    {
        const bool final_target = target_index + 1 >= world_route_.size();
        const bool passed_final_target =
            metrics.segment_length > 1e-6 &&
            metrics.projection_ratio >= 1.0 &&
            std::fabs(metrics.cross_track) <= xy_tolerance_m_;
        if (final_target && (metrics.distance_to_target <= xy_tolerance_m_ || passed_final_target))
        {
            const double target_yaw_error = WrapAngle(target.yaw - state.yaw);
            vx = 0.0;
            vy = 0.0;
            wz = Clamp(0.85 * target_yaw_error, -max_wz_radps_, max_wz_radps_);
            return;
        }

        const double dx = target.x - state.x;
        const double dy = target.y - state.y;
        const double c = std::cos(state.yaw);
        const double s = std::sin(state.yaw);
        const double x_body = c * dx + s * dy;
        const double y_body = -s * dx + c * dy;
        const double target_heading = std::atan2(dy, dx);
        const double heading_error = WrapAngle(target_heading - state.yaw);

        vx = Clamp(0.55 * x_body, 0.0, max_vx_mps_);
        if (vx > 1e-3)
        {
            vx = std::max(vx, 0.08);
        }
        vy = Clamp(0.55 * y_body, -max_vy_mps_, max_vy_mps_);
        wz = Clamp(0.85 * heading_error, -max_wz_radps_, max_wz_radps_);
    }

    int SendMove(double vx, double vy, double wz)
    {
        if (motion_client_ == "obstacles_avoid")
        {
            return obstacle_client_.Move(static_cast<float>(vx), static_cast<float>(vy), static_cast<float>(wz));
        }
        return sport_client_.Move(static_cast<float>(vx), static_cast<float>(vy), static_cast<float>(wz));
    }

    void Stop()
    {
        for (int i = 0; i < 12; ++i)
        {
            if (motion_client_ == "obstacles_avoid")
            {
                obstacle_client_.Move(0.0f, 0.0f, 0.0f);
            }
            sport_client_.Move(0.0f, 0.0f, 0.0f);
            std::this_thread::sleep_for(std::chrono::milliseconds(80));
        }
        sport_client_.StopMove();
    }

    void Log(
        double elapsed,
        const char * phase,
        std::size_t target_index,
        const RobotState & state,
        const Waypoint & target,
        const SegmentMetrics & metrics,
        double vx,
        double vy,
        double wz,
        int move_ret)
    {
        if (!log_)
        {
            return;
        }
        log_ << NowForLog() << ","
             << std::fixed << std::setprecision(6)
             << elapsed << ","
             << phase << ","
             << (target_index + 1) << ","
             << world_route_.size() << ","
             << state.x << ","
             << state.y << ","
             << state.z << ","
             << state.yaw << ","
             << target.x << ","
             << target.y << ","
             << target.yaw << ","
             << metrics.distance_to_target << ","
             << metrics.projection_ratio << ","
             << metrics.cross_track << ","
             << vx << ","
             << vy << ","
             << wz << ","
             << move_ret << ","
             << state.error_code << "\n";
    }

    std::vector<Waypoint> relative_route_;
    std::vector<Waypoint> world_route_;
    std::string output_csv_;
    double xy_tolerance_m_{0.25};
    double max_vx_mps_{0.30};
    double max_vy_mps_{0.08};
    double max_wz_radps_{0.45};
    double yaw_tolerance_rad_{0.35};
    double timeout_sec_{90.0};
    std::string motion_client_{"obstacles_avoid"};
    std::chrono::duration<double> control_period_;

    unitree::robot::go2::SportClient sport_client_;
    unitree::robot::go2::ObstaclesAvoidClient obstacle_client_;
    ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> state_sub_;

    mutable std::mutex state_mutex_;
    RobotState latest_state_;
    bool have_state_{false};
    std::ofstream log_;
};

void PrintUsage(const char * program)
{
    std::cout
        << "Usage:\n"
        << "  " << program << " <networkInterface> <route_file> [output_csv] [xy_tolerance_m]\n"
        << "      [max_vx_mps] [max_vy_mps] [max_wz_radps] [timeout_sec]\n"
        << "      [motion_client] [control_hz] [yaw_tolerance_rad]\n\n"
        << "route_file contains relative x,y,yaw triples, such as the\n"
        << "offline_map/waypoints_relative_text.txt generated from a capture.\n"
        << "motion_client: obstacles_avoid or sport. Default: obstacles_avoid.\n";
}

int main(int argc, const char ** argv)
{
    if (argc < 3 || argc > 12)
    {
        PrintUsage(argv[0]);
        return 1;
    }

    std::signal(SIGINT, HandleSignal);
    std::signal(SIGTERM, HandleSignal);

    try
    {
        const std::string network_interface = argv[1];
        const std::string route_file = argv[2];
        const std::string output_csv = argc > 3 ? argv[3] : "route_replay_log.csv";
        const double xy_tolerance_m = Clamp(argc > 4 ? ParseDouble(argv[4], "xy_tolerance_m") : 0.25, 0.05, 1.0);
        const double max_vx_mps = Clamp(argc > 5 ? ParseDouble(argv[5], "max_vx_mps") : 0.30, 0.02, 0.35);
        const double max_vy_mps = Clamp(argc > 6 ? ParseDouble(argv[6], "max_vy_mps") : 0.08, 0.0, 0.20);
        const double max_wz_radps = Clamp(argc > 7 ? ParseDouble(argv[7], "max_wz_radps") : 0.45, 0.05, 0.80);
        const double timeout_sec = Clamp(argc > 8 ? ParseDouble(argv[8], "timeout_sec") : 90.0, 1.0, 3600.0);
        const std::string motion_client = argc > 9 ? argv[9] : "obstacles_avoid";
        const double control_hz = Clamp(argc > 10 ? ParseDouble(argv[10], "control_hz") : 20.0, 2.0, 50.0);
        const double yaw_tolerance_rad = Clamp(argc > 11 ? ParseDouble(argv[11], "yaw_tolerance_rad") : 0.35, 0.02, M_PI);

        std::vector<Waypoint> route = ParseRouteFile(route_file);
        unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);
        RouteReplay replay(
            std::move(route), output_csv, xy_tolerance_m, max_vx_mps, max_vy_mps,
            max_wz_radps, yaw_tolerance_rad, timeout_sec, motion_client, control_hz);
        replay.Init();
        return replay.Run();
    }
    catch (const std::exception & ex)
    {
        std::cerr << "go2_route_replay fatal: " << ex.what() << std::endl;
        return 2;
    }
}
