#include <atomic>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/go2/obstacles_avoid/obstacles_avoid_client.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

using namespace unitree::robot;

namespace
{
constexpr double kRadToDeg = 57.29577951308232;
constexpr double kMinOdomDeltaM = 0.001;
constexpr double kIncrementCommandTimeoutSec = 5.0;
constexpr double kProgressPrintIntervalM = 1.0;
constexpr double kTargetToleranceM = 0.02;
constexpr int kMotionVelocity = 0;
constexpr int kMotionIncrementPosition = 1;

std::atomic<bool> running{true};

void Trace(const char *message)
{
    std::cerr << "[go2_straight_line_runner] " << message << std::endl;
}

void HandleSignal(int)
{
    running.store(false);
}

std::string IsoTimestamp(std::chrono::system_clock::time_point now)
{
    const auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::tm tm_now{};
    localtime_r(&time_t_now, &tm_now);

    std::ostringstream out;
    out << std::put_time(&tm_now, "%Y-%m-%d %H:%M:%S");
    return out.str();
}

double UnixSeconds(std::chrono::system_clock::time_point now)
{
    return std::chrono::duration<double>(now.time_since_epoch()).count();
}

double ElapsedSeconds(std::chrono::steady_clock::time_point start)
{
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

double ParseDouble(const char *value, const std::string &name)
{
    char *end = nullptr;
    const double parsed = std::strtod(value, &end);
    if (end == value || *end != '\0')
    {
        throw std::invalid_argument(name + " must be numeric.");
    }
    return parsed;
}

double Clamp(double value, double lower, double upper)
{
    return std::min(std::max(value, lower), upper);
}

double MinValidObstacleRange(const std::array<float, 4> &ranges)
{
    double min_range = std::numeric_limits<double>::infinity();
    for (float value : ranges)
    {
        const double range = static_cast<double>(value);
        if (std::isfinite(range) && range > 0.01 && range < min_range)
        {
            min_range = range;
        }
    }
    return min_range;
}

class StraightLineRunner
{
public:
    StraightLineRunner(
        std::string output_csv,
        double distance_m,
        double speed_mps,
        double warmup_sec,
        double control_hz,
        bool log_sport_state,
        double max_runtime_sec,
        bool use_obstacle_avoid,
        int motion_mode,
        double increment_step_m)
        : output_csv_(std::move(output_csv)),
          distance_m_(distance_m),
          speed_mps_(speed_mps),
          warmup_sec_(warmup_sec),
          control_hz_(control_hz),
          log_sport_state_(log_sport_state),
          max_runtime_sec_(max_runtime_sec),
          use_obstacle_avoid_(use_obstacle_avoid),
          motion_mode_(motion_mode),
          increment_step_m_(increment_step_m)
    {
    }

    void Init()
    {
        output_.open(output_csv_);
        if (!output_.is_open())
        {
            throw std::runtime_error("Failed to open output CSV: " + output_csv_);
        }

        output_ << "timestamp_iso,timestamp_unix,elapsed_sec,phase,"
                << "cmd_vx_mps,cmd_vy_mps,cmd_vyaw_radps,cmd_s_m,"
                << "odom_s_m,odom_x_m,odom_y_m,odom_z_m,"
                << "vx_mps,vy_mps,vz_mps,yaw_rad,yaw_deg,progress,"
                << "mode,gait_type,error_code,has_state,command_ret,motion_client,"
                << "range_obstacle_min_m,range_obstacle_0_m,range_obstacle_1_m,"
                << "range_obstacle_2_m,range_obstacle_3_m,motion_mode\n";

        Trace("Constructing SportClient...");
        sport_client_ = std::make_unique<unitree::robot::go2::SportClient>();
        Trace("SportClient constructed.");
        Trace("Initializing SportClient...");
        sport_client_->SetTimeout(10.0f);
        sport_client_->Init();
        Trace("SportClient initialized.");

        if (motion_mode_ == kMotionIncrementPosition && !use_obstacle_avoid_)
        {
            throw std::runtime_error("motion_mode=increment_position requires use_obstacle_avoid=1.");
        }

        if (use_obstacle_avoid_)
        {
            Trace("Constructing ObstaclesAvoidClient...");
            obstacle_client_ = std::make_unique<unitree::robot::go2::ObstaclesAvoidClient>();
            Trace("ObstaclesAvoidClient constructed.");
            Trace("Initializing ObstaclesAvoidClient...");
            obstacle_client_->SetTimeout(10.0f);
            obstacle_client_->Init();
            Trace("ObstaclesAvoidClient initialized.");
            ConfigureObstacleAvoid("init");
        }

        if (log_sport_state_)
        {
            Trace("Initializing rt/sportmodestate subscriber...");
            subscriber_.reset(new ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>("rt/sportmodestate"));
            subscriber_->InitChannel(std::bind(&StraightLineRunner::SportStateHandler, this, std::placeholders::_1), 10);
            Trace("Sport state subscriber initialized.");
        }
        else
        {
            Trace("Sport state subscriber disabled; using cmd_s_m as the Phase A label.");
        }
    }

    void Run()
    {
        std::cout << "Straight-line runner starting." << std::endl;
        std::cout << "Output CSV: " << output_csv_ << std::endl;
        std::cout << "Distance: " << distance_m_ << " m, speed: " << speed_mps_ << " m/s" << std::endl;
        std::cout << "Warmup: " << warmup_sec_ << " sec" << std::endl;
        std::cout << "Stop source: " << (log_sport_state_ ? "odom_s_m from rt/sportmodestate" : "cmd_s_m timeout fallback") << std::endl;
        std::cout << "Max move runtime: " << max_runtime_sec_ << " sec" << std::endl;
        std::cout << "Motion client: " << MotionClientName() << std::endl;
        std::cout << "Motion mode: " << MotionModeName() << std::endl;
        std::cout << "Increment step: "
                  << (increment_step_m_ > 0.0 ? std::to_string(increment_step_m_) + " m" : "full remaining distance")
                  << std::endl;

        std::cout << "Sending BalanceStand..." << std::endl;
        if (!sport_client_)
        {
            throw std::runtime_error("SportClient is not initialized.");
        }
        sport_client_->BalanceStand();
        if (use_obstacle_avoid_)
        {
            // BalanceStand can change the active sport service state, so
            // configure obstacle avoidance again immediately before movement.
            ConfigureObstacleAvoid("after_balance_stand");
        }

        if (log_sport_state_)
        {
            const auto init_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
            while (running.load() && !CaptureInitialState())
            {
                if (std::chrono::steady_clock::now() > init_deadline)
                {
                    throw std::runtime_error("No rt/sportmodestate received before movement; refusing odom-controlled run.");
                }
                LogRow("warmup", 0.0, 0.0, 0.0, 0.0);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }

        const auto warmup_start = std::chrono::steady_clock::now();
        while (running.load() && ElapsedSeconds(warmup_start) < warmup_sec_)
        {
            LogRow("warmup", 0.0, 0.0, 0.0, 0.0);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        ResetPathAccumulator();
        move_start_ = std::chrono::steady_clock::now();
        next_increment_goal_s_ = 0.0;
        next_progress_print_s_ = 0.0;
        const auto period = std::chrono::duration<double>(1.0 / control_hz_);
        bool reached_target = false;

        while (running.load())
        {
            const double elapsed = ElapsedSeconds(move_start_);
            if (elapsed >= max_runtime_sec_)
            {
                std::cerr << "Warning: max move runtime reached before target distance." << std::endl;
                break;
            }

            const double cmd_s = std::min(distance_m_, speed_mps_ * elapsed);
            const double odom_s = CurrentOdomS();
            PrintProgress(odom_s);
            if (log_sport_state_ && IsTargetReached(odom_s))
            {
                reached_target = true;
                LogRow("target_reached", 0.0, 0.0, 0.0, cmd_s);
                break;
            }
            if (!log_sport_state_ && cmd_s >= distance_m_)
            {
                reached_target = true;
                break;
            }

            if (motion_mode_ == kMotionIncrementPosition)
            {
                SendIncrementIfNeeded(odom_s);
                LogRow("move_increment", 0.0, 0.0, 0.0, cmd_s);
            }
            else
            {
                SendMove(speed_mps_, 0.0, 0.0);
                LogRow("move_velocity", speed_mps_, 0.0, 0.0, cmd_s);
            }
            std::this_thread::sleep_for(period);
        }

        StopAndLog();
        if (reached_target)
        {
            std::cout << "Target distance reached." << std::endl;
        }
        std::cout << "Straight-line runner finished." << std::endl;
    }

    void StopAndLog()
    {
        if (!sport_client_)
        {
            return;
        }
        for (int i = 0; i < 5; ++i)
        {
            SendStop();
            LogRow("stop", 0.0, 0.0, 0.0, distance_m_);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

private:
    void SportStateHandler(const void *message)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state_ = *(const unitree_go::msg::dds_::SportModeState_ *)message;
        has_state_ = true;

        if (accumulate_odom_ && has_initial_state_)
        {
            const auto &position = state_.position();
            const double x = position[0];
            const double y = position[1];

            if (!has_last_path_position_)
            {
                last_path_x_ = x;
                last_path_y_ = y;
                has_last_path_position_ = true;
                return;
            }

            const double dx = x - last_path_x_;
            const double dy = y - last_path_y_;
            const double ds = std::sqrt(dx * dx + dy * dy);
            if (std::isfinite(ds) && ds >= kMinOdomDeltaM)
            {
                odom_path_s_ += ds;
                last_path_x_ = x;
                last_path_y_ = y;
            }
        }
    }

    bool CaptureInitialState()
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!has_state_)
        {
            return false;
        }

        const auto &position = state_.position();
        const auto &rpy = state_.imu_state().rpy();
        initial_x_ = position[0];
        initial_y_ = position[1];
        initial_yaw_ = rpy[2];
        odom_path_s_ = 0.0;
        last_path_x_ = initial_x_;
        last_path_y_ = initial_y_;
        has_last_path_position_ = true;
        has_initial_state_ = true;
        return true;
    }

    void LogRow(
        const std::string &phase,
        double cmd_vx,
        double cmd_vy,
        double cmd_vyaw,
        double cmd_s)
    {
        unitree_go::msg::dds_::SportModeState_ state_snapshot;
        bool has_state_snapshot = false;
        double odom_s_snapshot = std::numeric_limits<double>::quiet_NaN();
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            state_snapshot = state_;
            has_state_snapshot = has_state_;
            if (has_initial_state_)
            {
                odom_s_snapshot = odom_path_s_;
            }
        }

        const auto now = std::chrono::system_clock::now();
        const auto elapsed_start = has_move_started()
                                       ? move_start_
                                       : std::chrono::steady_clock::now();
        const double elapsed = has_move_started() ? ElapsedSeconds(elapsed_start) : 0.0;

        double odom_s = std::numeric_limits<double>::quiet_NaN();
        double x = std::numeric_limits<double>::quiet_NaN();
        double y = std::numeric_limits<double>::quiet_NaN();
        double z = std::numeric_limits<double>::quiet_NaN();
        double vx = std::numeric_limits<double>::quiet_NaN();
        double vy = std::numeric_limits<double>::quiet_NaN();
        double vz = std::numeric_limits<double>::quiet_NaN();
        double yaw = std::numeric_limits<double>::quiet_NaN();
        double progress = std::numeric_limits<double>::quiet_NaN();
        double range_obstacle_min = std::numeric_limits<double>::infinity();
        std::array<double, 4> range_obstacle{};
        int mode = -1;
        int gait_type = -1;
        uint32_t error_code = 0;

        if (has_state_snapshot)
        {
            const auto &position = state_snapshot.position();
            const auto &velocity = state_snapshot.velocity();
            const auto &rpy = state_snapshot.imu_state().rpy();
            const auto &ranges = state_snapshot.range_obstacle();
            x = position[0];
            y = position[1];
            z = position[2];
            vx = velocity[0];
            vy = velocity[1];
            vz = velocity[2];
            yaw = rpy[2];
            progress = state_snapshot.progress();
            mode = static_cast<int>(state_snapshot.mode());
            gait_type = static_cast<int>(state_snapshot.gait_type());
            error_code = state_snapshot.error_code();
            range_obstacle_min = MinValidObstacleRange(ranges);
            for (std::size_t i = 0; i < ranges.size(); ++i)
            {
                range_obstacle[i] = ranges[i];
            }

            odom_s = odom_s_snapshot;
        }

        output_ << IsoTimestamp(now) << ','
                << std::fixed << std::setprecision(6) << UnixSeconds(now) << ','
                << elapsed << ','
                << phase << ','
                << cmd_vx << ','
                << cmd_vy << ','
                << cmd_vyaw << ','
                << cmd_s << ','
                << odom_s << ','
                << x << ','
                << y << ','
                << z << ','
                << vx << ','
                << vy << ','
                << vz << ','
                << yaw << ','
                << yaw * kRadToDeg << ','
                << progress << ','
                << mode << ','
                << gait_type << ','
                << error_code << ','
                << (has_state_snapshot ? 1 : 0) << ','
                << last_command_ret_ << ','
                << MotionClientName() << ','
                << range_obstacle_min << ','
                << range_obstacle[0] << ','
                << range_obstacle[1] << ','
                << range_obstacle[2] << ','
                << range_obstacle[3] << ','
                << MotionModeName()
                << '\n';
        output_.flush();
    }

    int32_t SendMove(double vx, double vy, double vyaw)
    {
        int32_t ret = 0;
        if (use_obstacle_avoid_ && obstacle_client_)
        {
            ret = obstacle_client_->Move(static_cast<float>(vx), static_cast<float>(vy), static_cast<float>(vyaw));
        }
        else if (sport_client_)
        {
            ret = sport_client_->Move(static_cast<float>(vx), static_cast<float>(vy), static_cast<float>(vyaw));
        }
        last_command_ret_ = ret;
        return ret;
    }

    int32_t SendIncrementIfNeeded(double odom_s)
    {
        if (!obstacle_client_)
        {
            last_command_ret_ = -1;
            return last_command_ret_;
        }
        if (!std::isfinite(odom_s))
        {
            return last_command_ret_;
        }

        const bool has_increment_goal = next_increment_goal_s_ > 0.0;
        const bool reached_increment_goal = odom_s + 0.05 >= next_increment_goal_s_;
        const bool command_timed_out = has_last_increment_command_ &&
                                       ElapsedSeconds(last_increment_command_time_) >= kIncrementCommandTimeoutSec;
        if (has_increment_goal && !reached_increment_goal && !command_timed_out)
        {
            return last_command_ret_;
        }

        if (IsTargetReached(odom_s))
        {
            return last_command_ret_;
        }
        const double remaining = std::max(0.0, distance_m_ - odom_s);

        const double step = increment_step_m_ > 0.0 ? std::min(increment_step_m_, remaining) : remaining;
        const int32_t ret = obstacle_client_->MoveToIncrementPosition(
            static_cast<float>(step),
            0.0f,
            0.0f);
        next_increment_goal_s_ = odom_s + step;
        last_increment_command_time_ = std::chrono::steady_clock::now();
        has_last_increment_command_ = true;
        std::cout << "Sent official increment command: step=" << step
                  << " m, odom_s_m=" << odom_s
                  << " m, next_goal_s_m=" << next_increment_goal_s_
                  << " ret=" << ret << std::endl;
        last_command_ret_ = ret;
        return ret;
    }

    void PrintProgress(double odom_s)
    {
        if (!std::isfinite(odom_s) || odom_s + 1e-6 < next_progress_print_s_)
        {
            return;
        }

        std::cout << "Progress: odom_s_m=" << odom_s
                  << " / target=" << distance_m_
                  << " motion_mode=" << MotionModeName()
                  << " command_ret=" << last_command_ret_ << std::endl;
        while (next_progress_print_s_ <= odom_s + 1e-6)
        {
            next_progress_print_s_ += kProgressPrintIntervalM;
        }
    }

    bool IsTargetReached(double odom_s) const
    {
        return std::isfinite(odom_s) && odom_s >= distance_m_ - kTargetToleranceM;
    }

    void ConfigureObstacleAvoid(const std::string &phase)
    {
        if (!obstacle_client_)
        {
            return;
        }

        int32_t ret = obstacle_client_->UseRemoteCommandFromApi(true);
        std::cout << "ObstaclesAvoid[" << phase << "] UseRemoteCommandFromApi(true) ret=" << ret << std::endl;
        last_command_ret_ = ret;

        ret = obstacle_client_->SwitchSet(true);
        std::cout << "ObstaclesAvoid[" << phase << "] SwitchSet(true) ret=" << ret << std::endl;
        last_command_ret_ = ret;

        bool enabled = false;
        ret = obstacle_client_->SwitchGet(enabled);
        std::cout << "ObstaclesAvoid[" << phase << "] SwitchGet ret=" << ret << " enabled=" << (enabled ? 1 : 0) << std::endl;
        last_command_ret_ = ret;
    }

    int32_t SendStop()
    {
        int32_t ret = 0;
        if (use_obstacle_avoid_ && obstacle_client_)
        {
            ret = obstacle_client_->Move(0.0f, 0.0f, 0.0f);
        }
        if (sport_client_)
        {
            const int32_t sport_ret = sport_client_->StopMove();
            if (ret == 0)
            {
                ret = sport_ret;
            }
        }
        last_command_ret_ = ret;
        return ret;
    }

    const char *MotionClientName() const
    {
        return use_obstacle_avoid_ ? "obstacles_avoid" : "sport";
    }

    const char *MotionModeName() const
    {
        return motion_mode_ == kMotionIncrementPosition ? "increment_position" : "velocity";
    }

    double CurrentOdomS()
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!has_state_ || !has_initial_state_)
        {
            return std::numeric_limits<double>::quiet_NaN();
        }

        return odom_path_s_;
    }

    void ResetPathAccumulator()
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!has_state_)
        {
            return;
        }

        const auto &position = state_.position();
        odom_path_s_ = 0.0;
        last_path_x_ = position[0];
        last_path_y_ = position[1];
        has_last_path_position_ = true;
        accumulate_odom_ = true;
    }

    bool has_move_started() const
    {
        return move_start_ != std::chrono::steady_clock::time_point{};
    }

    std::string output_csv_;
    double distance_m_;
    double speed_mps_;
    double warmup_sec_;
    double control_hz_;
    bool log_sport_state_;
    double max_runtime_sec_;
    bool use_obstacle_avoid_;
    int motion_mode_;
    double increment_step_m_;
    double next_increment_goal_s_{0.0};
    bool has_last_increment_command_{false};
    std::chrono::steady_clock::time_point last_increment_command_time_{};
    double next_progress_print_s_{0.0};
    int32_t last_command_ret_{0};

    std::unique_ptr<unitree::robot::go2::SportClient> sport_client_;
    std::unique_ptr<unitree::robot::go2::ObstaclesAvoidClient> obstacle_client_;
    ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> subscriber_;
    std::ofstream output_;

    std::mutex state_mutex_;
    unitree_go::msg::dds_::SportModeState_ state_;
    bool has_state_{false};
    bool has_initial_state_{false};
    bool accumulate_odom_{false};
    bool has_last_path_position_{false};
    double initial_x_{0.0};
    double initial_y_{0.0};
    double initial_yaw_{0.0};
    double last_path_x_{0.0};
    double last_path_y_{0.0};
    double odom_path_s_{0.0};
    std::chrono::steady_clock::time_point move_start_{};
};

void PrintUsage(const char *program)
{
    std::cout << "Usage:\n"
              << "  " << program << " <networkInterface> [output_csv] [distance_m] [speed_mps] [warmup_sec] [control_hz] [log_sport_state] [max_runtime_sec] [use_obstacle_avoid] [motion_mode] [increment_step_m]\n\n"
              << "Defaults:\n"
              << "  output_csv=sport_state_straight.csv\n"
              << "  distance_m=70.0\n"
              << "  speed_mps=0.20\n"
              << "  warmup_sec=2.0\n"
              << "  control_hz=20\n"
              << "  log_sport_state=1; required for odom-controlled distance stopping\n"
              << "  max_runtime_sec=max(10, 4 * distance_m / speed_mps)\n\n"
              << "  use_obstacle_avoid=1; use Unitree official obstacles_avoid client for motion\n\n"
              << "  motion_mode=1; 0=official velocity Move, 1=official MoveToIncrementPosition\n"
              << "  increment_step_m=0.0; 0 means one official increment command for full remaining distance\n\n"
              << "Safety clamps:\n"
              << "  speed_mps is clamped to [0.05, 0.35]\n"
              << "  distance_m is clamped to [0.1, 100.0]\n";
}
}

int main(int argc, const char **argv)
{
    Trace("main entered.");
    if (argc < 2 || argc > 12)
    {
        PrintUsage(argv[0]);
        return 1;
    }

    StraightLineRunner *runner = nullptr;
    int exit_code = 0;
    try
    {
        const std::string network_interface = argv[1];
        const std::string output_csv = argc > 2 ? argv[2] : "sport_state_straight.csv";
        const double distance_m = Clamp(argc > 3 ? ParseDouble(argv[3], "distance_m") : 70.0, 0.1, 100.0);
        const double speed_mps = Clamp(argc > 4 ? ParseDouble(argv[4], "speed_mps") : 0.20, 0.05, 0.35);
        const double warmup_sec = Clamp(argc > 5 ? ParseDouble(argv[5], "warmup_sec") : 2.0, 0.0, 10.0);
        const double control_hz = Clamp(argc > 6 ? ParseDouble(argv[6], "control_hz") : 20.0, 5.0, 50.0);
        const bool log_sport_state = argc > 7 ? (ParseDouble(argv[7], "log_sport_state") != 0.0) : true;
        const double default_max_runtime_sec = std::max(10.0, 4.0 * distance_m / speed_mps);
        const double max_runtime_sec = Clamp(argc > 8 ? ParseDouble(argv[8], "max_runtime_sec") : default_max_runtime_sec, 1.0, 3600.0);
        const bool use_obstacle_avoid = argc > 9 ? (ParseDouble(argv[9], "use_obstacle_avoid") != 0.0) : true;
        const int motion_mode = static_cast<int>(Clamp(argc > 10 ? ParseDouble(argv[10], "motion_mode") : kMotionIncrementPosition, 0.0, 1.0));
        const double increment_step_m = Clamp(argc > 11 ? ParseDouble(argv[11], "increment_step_m") : 0.0, 0.0, 100.0);

        std::signal(SIGINT, HandleSignal);
        std::signal(SIGTERM, HandleSignal);

        Trace("Calling ChannelFactory::Init...");
        ChannelFactory::Instance()->Init(0, network_interface);
        Trace("ChannelFactory::Init returned.");

        Trace("Constructing runner...");
        runner = new StraightLineRunner(
            output_csv, distance_m, speed_mps, warmup_sec, control_hz, log_sport_state,
            max_runtime_sec, use_obstacle_avoid, motion_mode, increment_step_m);
        Trace("Runner constructed.");
        runner->Init();
        runner->Run();
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << std::endl;
        if (runner != nullptr)
        {
            try
            {
                runner->StopAndLog();
            }
            catch (...)
            {
            }
        }
        exit_code = 1;
    }

    std::cout.flush();
    std::cerr.flush();
    // Some Unitree SDK builds can abort during client/subscriber teardown on exit.
    // This command-line runner has already sent StopMove and flushed its CSV.
    std::_Exit(exit_code);
}
