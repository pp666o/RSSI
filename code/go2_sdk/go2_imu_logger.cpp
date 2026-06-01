#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

using namespace unitree::robot;

namespace
{
constexpr double kGravity = 9.80665;
constexpr double kRadToDeg = 57.29577951308232;

std::atomic<bool> running{true};

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

class Go2ImuLogger
{
public:
    Go2ImuLogger(
        std::string output_csv,
        double acc_to_g_scale,
        double gyro_to_dps_scale,
        double angle_to_deg_scale)
        : output_csv_(std::move(output_csv)),
          acc_to_g_scale_(acc_to_g_scale),
          gyro_to_dps_scale_(gyro_to_dps_scale),
          angle_to_deg_scale_(angle_to_deg_scale)
    {
    }

    void Init()
    {
        output_.open(output_csv_);
        if (!output_.is_open())
        {
            throw std::runtime_error("Failed to open output CSV: " + output_csv_);
        }

        output_ << "timestamp_iso,timestamp_unix,elapsed_sec,"
                << "frame_type,has_acc,has_gyro,has_angle,"
                << "acc_x_g,acc_y_g,acc_z_g,"
                << "gyro_x_dps,gyro_y_dps,gyro_z_dps,"
                << "roll_deg,pitch_deg,yaw_deg\n";

        start_steady_ = std::chrono::steady_clock::now();
        subscriber_.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>("rt/lowstate"));
        subscriber_->InitChannel(std::bind(&Go2ImuLogger::LowStateHandler, this, std::placeholders::_1), 10);
    }

    uint64_t FrameCount() const
    {
        return frame_count_.load();
    }

private:
    void LowStateHandler(const void *message)
    {
        const auto low_state = *(const unitree_go::msg::dds_::LowState_ *)message;
        const auto &acc = low_state.imu_state().accelerometer();
        const auto &gyro = low_state.imu_state().gyroscope();
        const auto &rpy = low_state.imu_state().rpy();

        const auto now = std::chrono::system_clock::now();
        const double elapsed = ElapsedSeconds(start_steady_);

        std::lock_guard<std::mutex> lock(output_mutex_);
        output_ << IsoTimestamp(now) << ','
                << std::fixed << std::setprecision(6) << UnixSeconds(now) << ','
                << elapsed << ','
                << "go2_lowstate,1,1,1,"
                << acc[0] * acc_to_g_scale_ << ','
                << acc[1] * acc_to_g_scale_ << ','
                << acc[2] * acc_to_g_scale_ << ','
                << gyro[0] * gyro_to_dps_scale_ << ','
                << gyro[1] * gyro_to_dps_scale_ << ','
                << gyro[2] * gyro_to_dps_scale_ << ','
                << rpy[0] * angle_to_deg_scale_ << ','
                << rpy[1] * angle_to_deg_scale_ << ','
                << rpy[2] * angle_to_deg_scale_ << '\n';

        frame_count_.fetch_add(1);
    }

    std::string output_csv_;
    double acc_to_g_scale_;
    double gyro_to_dps_scale_;
    double angle_to_deg_scale_;
    std::ofstream output_;
    std::mutex output_mutex_;
    std::chrono::steady_clock::time_point start_steady_;
    std::atomic<uint64_t> frame_count_{0};
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> subscriber_;
};

void PrintUsage(const char *program)
{
    std::cout << "Usage:\n"
              << "  " << program << " <networkInterface> [output_csv] [duration_sec] "
              << "[acc_to_g_scale] [gyro_to_dps_scale] [angle_to_deg_scale]\n\n"
              << "Defaults:\n"
              << "  output_csv=imu_stream.csv\n"
              << "  duration_sec=0 means run until Ctrl+C\n"
              << "  acc_to_g_scale=1/9.80665, gyro_to_dps_scale=180/pi, angle_to_deg_scale=180/pi\n\n"
              << "If your Go2 firmware already reports IMU in g, dps, and degrees, pass: 1 1 1\n";
}
}

int main(int argc, const char **argv)
{
    if (argc < 2)
    {
        PrintUsage(argv[0]);
        return 1;
    }

    try
    {
        const std::string network_interface = argv[1];
        const std::string output_csv = argc > 2 ? argv[2] : "imu_stream.csv";
        const double duration_sec = argc > 3 ? ParseDouble(argv[3], "duration_sec") : 0.0;
        const double acc_to_g_scale = argc > 4 ? ParseDouble(argv[4], "acc_to_g_scale") : (1.0 / kGravity);
        const double gyro_to_dps_scale = argc > 5 ? ParseDouble(argv[5], "gyro_to_dps_scale") : kRadToDeg;
        const double angle_to_deg_scale = argc > 6 ? ParseDouble(argv[6], "angle_to_deg_scale") : kRadToDeg;

        std::signal(SIGINT, HandleSignal);
        std::signal(SIGTERM, HandleSignal);

        ChannelFactory::Instance()->Init(0, network_interface);

        Go2ImuLogger logger(output_csv, acc_to_g_scale, gyro_to_dps_scale, angle_to_deg_scale);
        logger.Init();

        std::cout << "Go2 IMU logging started: " << output_csv << std::endl;
        const auto start = std::chrono::steady_clock::now();
        while (running.load())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            if (duration_sec > 0.0)
            {
                const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
                if (elapsed >= duration_sec)
                {
                    break;
                }
            }
        }

        std::cout << "Go2 IMU logging finished. frames=" << logger.FrameCount() << std::endl;
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << std::endl;
        return 1;
    }

    return 0;
}
