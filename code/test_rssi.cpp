#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cctype>
#include <ctime>
#include <deque>
#include <errno.h>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <regex>
#include <sstream>
#include <string>
#include <sys/wait.h>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {

constexpr double kMissingRssi = -999.0;

const std::vector<std::string> kTargetMacList = {
    "C0:C9:38:AE:59:96",  // beacon_1
    "C3:72:5C:88:86:A2",  // beacon_2
    "C4:2C:1C:61:C3:12",  // beacon_3
    "D8:40:11:05:63:52",  // beacon_4
    "DE:BC:AC:80:47:92",  // beacon_5
    "F3:56:75:0B:0D:BD",  // beacon_6
    "F7:50:F8:6B:B9:9A",  // beacon_7
    "F7:B8:0C:DB:3A:EB",  // beacon_8
    "F9:9C:A7:A5:68:54",  // beacon_9
    "FB:AE:18:39:C1:3B"   // beacon_10
};

volatile sig_atomic_t g_stop_signal = 0;
std::atomic<bool> g_user_stop{false};

struct RssiSample {
    double elapsed_sec;
    int rssi;
};

struct ImuData {
    double acc_x_g = 0.0;
    double acc_y_g = 0.0;
    double acc_z_g = 0.0;
    double gyro_x_dps = 0.0;
    double gyro_y_dps = 0.0;
    double gyro_z_dps = 0.0;
    double roll_deg = 0.0;
    double pitch_deg = 0.0;
    double yaw_deg = 0.0;
    bool has_acc = false;
    bool has_gyro = false;
    bool has_angle = false;
};

struct ProgramOptions {
    int duration_sec = 0;
    std::string window_csv = "rssi_realtime_windows.csv";
    double window_sec = 1.0;
    double step_sec = 0.5;
    double marker_step_m = 0.5;
    double start_s_m = 0.0;
    std::string imu_port = "/dev/ttyUSB1";
    int imu_baud = 921600;
    std::string raw_csv = "rssi_raw_stream.csv";
    std::string marker_csv = "rssi_markers.csv";
    std::string imu_csv = "imu_stream.csv";
};

void handleSignal(int) {
    g_stop_signal = 1;
}

bool isNumber(const std::string& text) {
    if (text.empty()) {
        return false;
    }

    char* endptr = nullptr;
    std::strtod(text.c_str(), &endptr);
    return endptr == text.c_str() + text.size();
}

std::string trim(const std::string& text) {
    const char* whitespace = " \t\r\n";
    const size_t start = text.find_first_not_of(whitespace);
    if (start == std::string::npos) {
        return "";
    }
    const size_t end = text.find_last_not_of(whitespace);
    return text.substr(start, end - start + 1);
}

std::string stripAnsi(const std::string& text) {
    std::string result;
    size_t i = 0;
    while (i < text.size()) {
        if (text[i] == '\033' && i + 1 < text.size() && text[i + 1] == '[') {
            while (i < text.size() && text[i] != 'm') {
                ++i;
            }
            if (i < text.size()) {
                ++i;
            }
        } else {
            result += text[i++];
        }
    }
    return result;
}

std::string toUpper(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        return static_cast<char>(std::toupper(ch));
    });
    return value;
}

std::string localTimeString() {
    char time_str[64];
    const time_t now = time(nullptr);
    struct tm tm_buf;
    localtime_r(&now, &tm_buf);
    strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", &tm_buf);
    return std::string(time_str);
}

double unixTimeSeconds() {
    using namespace std::chrono;
    const auto now = system_clock::now();
    return duration_cast<duration<double>>(now.time_since_epoch()).count();
}

double elapsedSeconds(std::chrono::steady_clock::time_point start_time) {
    using namespace std::chrono;
    return duration_cast<duration<double>>(steady_clock::now() - start_time).count();
}

bool parseMacAddress(const std::string& line, std::string* mac_out) {
    static const std::regex mac_regex("([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}");
    std::smatch match;
    if (std::regex_search(line, match, mac_regex)) {
        *mac_out = toUpper(match.str());
        return true;
    }
    return false;
}

bool parseRssi(const std::string& line, int* rssi_out) {
    static const std::regex rssi_regex("RSSI:\\s*(-?\\d+)");
    std::smatch match;
    if (std::regex_search(line, match, rssi_regex)) {
        *rssi_out = std::stoi(match[1].str());
        return true;
    }
    return false;
}

double mean(const std::deque<RssiSample>& samples) {
    if (samples.empty()) {
        return kMissingRssi;
    }

    double sum = 0.0;
    for (const auto& sample : samples) {
        sum += sample.rssi;
    }
    return sum / static_cast<double>(samples.size());
}

double stddev(const std::deque<RssiSample>& samples) {
    if (samples.empty()) {
        return kMissingRssi;
    }
    if (samples.size() == 1) {
        return 0.0;
    }

    const double avg = mean(samples);
    double sum_sq = 0.0;
    for (const auto& sample : samples) {
        const double diff = sample.rssi - avg;
        sum_sq += diff * diff;
    }
    return std::sqrt(sum_sq / static_cast<double>(samples.size() - 1));
}

void pruneOldSamples(std::vector<std::deque<RssiSample>>& windows, double now_sec, double window_sec) {
    const double cutoff = now_sec - window_sec;
    for (auto& samples : windows) {
        while (!samples.empty() && samples.front().elapsed_sec < cutoff) {
            samples.pop_front();
        }
    }
}

void writeWindowHeader(std::ofstream& out) {
    out << "timestamp_iso,timestamp_unix,elapsed_sec";
    for (size_t i = 0; i < kTargetMacList.size(); ++i) {
        const std::string beacon = "beacon_" + std::to_string(i + 1);
        out << "," << beacon << "_mean"
            << "," << beacon << "_std"
            << "," << beacon << "_count";
    }
    out << "\n";
}

void writeRawHeader(std::ofstream& out) {
    out << "timestamp_iso,timestamp_unix,elapsed_sec,beacon_id,mac,rssi\n";
}

void writeMarkerHeader(std::ofstream& out) {
    out << "timestamp_iso,timestamp_unix,elapsed_sec,s\n";
}

void writeImuHeader(std::ofstream& out) {
    out << "timestamp_iso,timestamp_unix,elapsed_sec,"
        << "frame_type,has_acc,has_gyro,has_angle,"
        << "acc_x_g,acc_y_g,acc_z_g,"
        << "gyro_x_dps,gyro_y_dps,gyro_z_dps,"
        << "roll_deg,pitch_deg,yaw_deg\n";
}

void emitWindowRow(
    std::ofstream& out,
    std::vector<std::deque<RssiSample>>& windows,
    double now_sec,
    double window_sec) {
    pruneOldSamples(windows, now_sec, window_sec);

    out << std::fixed << std::setprecision(6)
        << localTimeString() << ","
        << unixTimeSeconds() << ","
        << now_sec;

    for (const auto& samples : windows) {
        out << "," << mean(samples)
            << "," << stddev(samples)
            << "," << samples.size();
    }
    out << "\n";
    out.flush();
}

void markerInputLoop(
    std::ofstream* marker_out,
    std::chrono::steady_clock::time_point start_time,
    double marker_step_m,
    double start_s_m) {
    double current_s = start_s_m;
    std::string line;
    while (!g_user_stop.load() && g_stop_signal == 0 && std::getline(std::cin, line)) {
        line = trim(line);
        if (line == "q" || line == "Q") {
            g_user_stop.store(true);
            return;
        }
        if (line.empty()) {
            current_s += marker_step_m;
        } else if (isNumber(line)) {
            current_s = std::strtod(line.c_str(), nullptr);
        } else {
            std::cerr << "Marker ignored. Press Enter to add "
                      << marker_step_m
                      << " m, type a numeric s value to correct, or q to stop.\n";
            continue;
        }

        const double elapsed = elapsedSeconds(start_time);
        *marker_out << std::fixed << std::setprecision(6)
                    << localTimeString() << ","
                    << unixTimeSeconds() << ","
                    << elapsed << ","
                    << current_s
                    << "\n";
        marker_out->flush();
        std::cout << "Marker saved: s=" << current_s << " m at elapsed=" << elapsed << " sec\n";
    }
}

speed_t baudToSpeed(int baud) {
    switch (baud) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default: return B921600;
    }
}

double parseAxis(unsigned char low, unsigned char high, double scale) {
    const int16_t value = static_cast<int16_t>((static_cast<unsigned short>(high) << 8) | low);
    return static_cast<double>(value) / 32768.0 * scale;
}

class ImuProcessor {
public:
    ImuProcessor(const std::string& port, int baud) {
        fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ == -1) {
            std::cerr << "Failed to open IMU serial port " << port << ": " << strerror(errno) << "\n";
            return;
        }

        termios options;
        if (tcgetattr(fd_, &options) != 0) {
            std::cerr << "Failed to read IMU serial attributes: " << strerror(errno) << "\n";
            close(fd_);
            fd_ = -1;
            return;
        }

        cfmakeraw(&options);
        const speed_t speed = baudToSpeed(baud);
        cfsetispeed(&options, speed);
        cfsetospeed(&options, speed);
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        options.c_cflag &= ~CRTSCTS;
        options.c_cflag |= CREAD | CLOCAL;
        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 0;

        if (tcsetattr(fd_, TCSANOW, &options) != 0) {
            std::cerr << "Failed to configure IMU serial port: " << strerror(errno) << "\n";
            close(fd_);
            fd_ = -1;
            return;
        }
        tcflush(fd_, TCIOFLUSH);
    }

    ~ImuProcessor() {
        if (fd_ != -1) {
            close(fd_);
        }
    }

    bool isValid() const {
        return fd_ != -1;
    }

    bool poll(ImuData* out, std::string* frame_type, int* bytes_read_total) {
        if (fd_ == -1) {
            return false;
        }

        bool produced = false;
        unsigned char buffer[256];
        while (true) {
            const ssize_t n = read(fd_, buffer, sizeof(buffer));
            if (n > 0) {
                *bytes_read_total += static_cast<int>(n);
                for (ssize_t i = 0; i < n; ++i) {
                    if (handleByte(buffer[i], frame_type)) {
                        *out = data_;
                        produced = true;
                    }
                }
                continue;
            }
            if (n == -1 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
                std::cerr << "Failed to read IMU serial data: " << strerror(errno) << "\n";
            }
            break;
        }
        return produced;
    }

private:
    bool handleByte(unsigned char byte, std::string* frame_type) {
        if (frame_.empty()) {
            if (byte == 0x55) {
                frame_.push_back(byte);
            }
            return false;
        }

        frame_.push_back(byte);
        if (frame_.size() < 11) {
            return false;
        }

        const bool complete = parseFrame(frame_, frame_type);
        frame_.clear();
        return complete;
    }

    bool parseFrame(const std::vector<unsigned char>& frame, std::string* frame_type) {
        unsigned char checksum = 0;
        for (size_t i = 0; i < 10; ++i) {
            checksum = static_cast<unsigned char>(checksum + frame[i]);
        }
        if (checksum != frame[10]) {
            return false;
        }

        const unsigned char type = frame[1];
        if (type == 0x51) {
            data_.acc_x_g = parseAxis(frame[2], frame[3], 16.0);
            data_.acc_y_g = parseAxis(frame[4], frame[5], 16.0);
            data_.acc_z_g = parseAxis(frame[6], frame[7], 16.0);
            data_.has_acc = true;
            *frame_type = "acc";
            return true;
        }
        if (type == 0x52) {
            data_.gyro_x_dps = parseAxis(frame[2], frame[3], 2000.0);
            data_.gyro_y_dps = parseAxis(frame[4], frame[5], 2000.0);
            data_.gyro_z_dps = parseAxis(frame[6], frame[7], 2000.0);
            data_.has_gyro = true;
            *frame_type = "gyro";
            return true;
        }
        if (type == 0x53) {
            data_.roll_deg = parseAxis(frame[2], frame[3], 180.0);
            data_.pitch_deg = parseAxis(frame[4], frame[5], 180.0);
            data_.yaw_deg = parseAxis(frame[6], frame[7], 180.0);
            data_.has_angle = true;
            *frame_type = "angle";
            return true;
        }
        return false;
    }

    int fd_ = -1;
    std::vector<unsigned char> frame_;
    ImuData data_;
};

void imuLoop(
    const std::string& port,
    int baud,
    std::ofstream* imu_out,
    std::chrono::steady_clock::time_point start_time) {
    ImuProcessor imu(port, baud);
    if (!imu.isValid()) {
        std::cerr << "IMU disabled because serial initialization failed.\n";
        return;
    }

    std::cout << "IMU collection started: " << port << " @ " << baud << "\n";
    int bytes_read_total = 0;
    int frames_written = 0;
    double next_status_sec = 1.0;
    while (!g_user_stop.load() && g_stop_signal == 0) {
        ImuData data;
        std::string frame_type;
        if (imu.poll(&data, &frame_type, &bytes_read_total)) {
            const double elapsed = elapsedSeconds(start_time);
            *imu_out << std::fixed << std::setprecision(6)
                     << localTimeString() << ","
                     << unixTimeSeconds() << ","
                     << elapsed << ","
                     << frame_type << ","
                     << (data.has_acc ? 1 : 0) << ","
                     << (data.has_gyro ? 1 : 0) << ","
                     << (data.has_angle ? 1 : 0) << ","
                     << data.acc_x_g << ","
                     << data.acc_y_g << ","
                     << data.acc_z_g << ","
                     << data.gyro_x_dps << ","
                     << data.gyro_y_dps << ","
                     << data.gyro_z_dps << ","
                     << data.roll_deg << ","
                     << data.pitch_deg << ","
                     << data.yaw_deg << "\n";
            imu_out->flush();
            ++frames_written;
        } else {
            usleep(2000);
        }

        const double elapsed = elapsedSeconds(start_time);
        if (elapsed >= next_status_sec) {
            std::cout << "IMU status: bytes=" << bytes_read_total
                      << " valid_frames=" << frames_written
                      << " elapsed=" << elapsed << " sec\n";
            next_status_sec += 1.0;
        }
    }
    std::cout << "IMU collection stopped. bytes=" << bytes_read_total
              << " valid_frames=" << frames_written << "\n";
}

void printUsage(const char* argv0) {
    std::cerr
        << "Usage:\n"
        << "  sudo " << argv0 << " <duration_sec> [window_csv] [window_sec] [step_sec] [marker_step_m] [start_s_m] [imu_port] [imu_baud]\n"
        << "\n"
        << "Examples:\n"
        << "  sudo " << argv0 << " 120\n"
        << "  sudo " << argv0 << " 0 rssi_realtime_windows.csv 1.0 0.5 0.5 0.0 /dev/ttyUSB1 921600\n"
        << "  sudo " << argv0 << " 10 rssi_realtime_windows.csv 1.0 0.5 0.5 0.0 none\n"
        << "\n"
        << "Notes:\n"
        << "  duration_sec=0 means run until Ctrl+C or q.\n"
        << "  Press Enter when crossing each ground marker; s increases by marker_step_m.\n"
        << "  Type a numeric path progress s and press Enter only when you need correction.\n"
        << "  IMU defaults to /dev/ttyUSB1. Use imu_port=none to disable IMU collection.\n";
}

bool parseOptions(int argc, char* argv[], ProgramOptions* options) {
    if (argc < 2 || argc > 9) {
        printUsage(argv[0]);
        return false;
    }
    if (!isNumber(argv[1])) {
        std::cerr << "duration_sec must be numeric.\n";
        return false;
    }

    options->duration_sec = std::atoi(argv[1]);
    if (argc >= 3) {
        options->window_csv = argv[2];
    }
    if (argc >= 4) {
        if (!isNumber(argv[3])) {
            std::cerr << "window_sec must be numeric.\n";
            return false;
        }
        options->window_sec = std::strtod(argv[3], nullptr);
    }
    if (argc >= 5) {
        if (!isNumber(argv[4])) {
            std::cerr << "step_sec must be numeric.\n";
            return false;
        }
        options->step_sec = std::strtod(argv[4], nullptr);
    }
    if (argc >= 6) {
        if (!isNumber(argv[5])) {
            std::cerr << "marker_step_m must be numeric.\n";
            return false;
        }
        options->marker_step_m = std::strtod(argv[5], nullptr);
    }
    if (argc >= 7) {
        if (!isNumber(argv[6])) {
            std::cerr << "start_s_m must be numeric.\n";
            return false;
        }
        options->start_s_m = std::strtod(argv[6], nullptr);
    }
    if (argc >= 8) {
        options->imu_port = argv[7];
        if (options->imu_port == "none" || options->imu_port == "off" || options->imu_port == "disable") {
            options->imu_port.clear();
        }
    }
    if (argc >= 9) {
        if (!isNumber(argv[8])) {
            std::cerr << "imu_baud must be numeric.\n";
            return false;
        }
        options->imu_baud = std::atoi(argv[8]);
    }

    if (options->duration_sec < 0 || options->window_sec <= 0.0 || options->step_sec <= 0.0 ||
        options->marker_step_m <= 0.0) {
        std::cerr << "duration_sec must be >= 0; window_sec, step_sec, and marker_step_m must be > 0.\n";
        return false;
    }
    return true;
}

pid_t startBtmon(int* read_fd) {
    int pipe_fd[2];
    if (pipe(pipe_fd) == -1) {
        std::cerr << "Failed to create pipe: " << strerror(errno) << "\n";
        return -1;
    }

    const pid_t pid = fork();
    if (pid == -1) {
        std::cerr << "Failed to fork: " << strerror(errno) << "\n";
        close(pipe_fd[0]);
        close(pipe_fd[1]);
        return -1;
    }

    if (pid == 0) {
        close(pipe_fd[0]);
        dup2(pipe_fd[1], STDOUT_FILENO);
        dup2(pipe_fd[1], STDERR_FILENO);
        close(pipe_fd[1]);
        execlp("btmon", "btmon", nullptr);
        std::cerr << "Failed to start btmon. Please install bluez.\n";
        _exit(EXIT_FAILURE);
    }

    close(pipe_fd[1]);
    const int flags = fcntl(pipe_fd[0], F_GETFL, 0);
    fcntl(pipe_fd[0], F_SETFL, flags | O_NONBLOCK);
    *read_fd = pipe_fd[0];
    return pid;
}

void stopBtmon(pid_t pid, int read_fd) {
    if (pid > 0) {
        kill(pid, SIGTERM);
        waitpid(pid, nullptr, 0);
    }
    if (read_fd >= 0) {
        close(read_fd);
    }
}

}  // namespace

int main(int argc, char* argv[]) {
    ProgramOptions options;
    if (!parseOptions(argc, argv, &options)) {
        return EXIT_FAILURE;
    }

    signal(SIGINT, handleSignal);
    signal(SIGTERM, handleSignal);

    std::map<std::string, size_t> mac_to_index;
    for (size_t i = 0; i < kTargetMacList.size(); ++i) {
        mac_to_index[kTargetMacList[i]] = i;
    }

    std::ofstream window_out(options.window_csv, std::ios::out);
    std::ofstream raw_out(options.raw_csv, std::ios::out);
    std::ofstream marker_out(options.marker_csv, std::ios::out);
    std::ofstream imu_out;
    if (!options.imu_port.empty()) {
        imu_out.open(options.imu_csv, std::ios::out);
    }
    if (!window_out.is_open() || !raw_out.is_open() || !marker_out.is_open() ||
        (!options.imu_port.empty() && !imu_out.is_open())) {
        std::cerr << "Failed to open output CSV files.\n";
        return EXIT_FAILURE;
    }

    writeWindowHeader(window_out);
    writeRawHeader(raw_out);
    writeMarkerHeader(marker_out);
    if (!options.imu_port.empty()) {
        writeImuHeader(imu_out);
    }

    int btmon_fd = -1;
    const pid_t btmon_pid = startBtmon(&btmon_fd);
    if (btmon_pid < 0) {
        return EXIT_FAILURE;
    }

    std::vector<std::deque<RssiSample>> windows(kTargetMacList.size());
    const auto start_time = std::chrono::steady_clock::now();

    marker_out << std::fixed << std::setprecision(6)
               << localTimeString() << ","
               << unixTimeSeconds() << ","
               << 0.0 << ","
               << options.start_s_m
               << "\n";
    marker_out.flush();

    std::thread marker_thread(
        markerInputLoop,
        &marker_out,
        start_time,
        options.marker_step_m,
        options.start_s_m);
    marker_thread.detach();

    std::thread imu_thread;
    if (!options.imu_port.empty()) {
        imu_thread = std::thread(imuLoop, options.imu_port, options.imu_baud, &imu_out, start_time);
    }

    std::cout << "Realtime RSSI + IMU collection started.\n"
              << "Window CSV: " << options.window_csv << "\n"
              << "Raw RSSI CSV: " << options.raw_csv << "\n"
              << "Marker CSV: " << options.marker_csv << "\n";
    if (!options.imu_port.empty()) {
        std::cout << "IMU CSV: " << options.imu_csv << "\n"
                  << "IMU port: " << options.imu_port << " @ " << options.imu_baud << "\n";
    } else {
        std::cout << "IMU: disabled. Pass imu_port to enable it.\n";
    }
    std::cout << "Start s: " << options.start_s_m << " m\n"
              << "Marker step: " << options.marker_step_m << " m\n"
              << "Press Enter at each ground marker, type numeric s to correct, q + Enter to stop.\n";

    std::string current_mac;
    std::string pending;
    double next_emit_sec = 0.0;

    while (g_stop_signal == 0 && !g_user_stop.load()) {
        const double now_sec = elapsedSeconds(start_time);
        if (options.duration_sec > 0 && now_sec >= options.duration_sec) {
            break;
        }

        char buffer[4096];
        const ssize_t bytes_read = read(btmon_fd, buffer, sizeof(buffer));
        if (bytes_read > 0) {
            pending.append(buffer, bytes_read);

            size_t newline_pos = std::string::npos;
            while ((newline_pos = pending.find('\n')) != std::string::npos) {
                std::string line = stripAnsi(pending.substr(0, newline_pos));
                pending.erase(0, newline_pos + 1);

                std::string parsed_mac;
                if (parseMacAddress(line, &parsed_mac)) {
                    current_mac = parsed_mac;
                    continue;
                }

                int rssi = 0;
                if (!current_mac.empty() && parseRssi(line, &rssi)) {
                    const auto it = mac_to_index.find(current_mac);
                    if (it == mac_to_index.end()) {
                        continue;
                    }

                    const size_t beacon_index = it->second;
                    const double sample_elapsed = elapsedSeconds(start_time);
                    windows[beacon_index].push_back({sample_elapsed, rssi});

                    raw_out << std::fixed << std::setprecision(6)
                            << localTimeString() << ","
                            << unixTimeSeconds() << ","
                            << sample_elapsed << ","
                            << "beacon_" << (beacon_index + 1) << ","
                            << current_mac << ","
                            << rssi << "\n";
                    raw_out.flush();
                }
            }
        } else if (bytes_read == -1 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
            std::cerr << "Failed to read btmon output: " << strerror(errno) << "\n";
            break;
        }

        if (now_sec >= next_emit_sec) {
            emitWindowRow(window_out, windows, now_sec, options.window_sec);
            next_emit_sec += options.step_sec;
        }

        usleep(50000);
    }

    stopBtmon(btmon_pid, btmon_fd);
    g_user_stop.store(true);

    if (imu_thread.joinable()) {
        imu_thread.join();
    }

    const double total_elapsed = elapsedSeconds(start_time);
    emitWindowRow(window_out, windows, total_elapsed, options.window_sec);

    std::cout << "Realtime RSSI + IMU collection finished. elapsed=" << total_elapsed << " sec\n";
    return EXIT_SUCCESS;
}
