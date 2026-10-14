/*
LiDARラズパイがUARTを受信できるかテストするコード

トピック通信のメッセージ定義は次の通り
float32 theta_rad
float32 distance_m
bool detected
uint8 sequence
bool stale
*/

#include <ros/ros.h>
#include <innurobot_uart/BallTarget.h>

#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <mutex>
#include <string>
#include <thread>

#include <termios.h>
#include <unistd.h>

static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559,
              "UART protocol requires IEEE-754 float32");

struct UARTConfig {
    std::string port = "/dev/serial0";
    speed_t baudrate = B115200;
    int timeout_ms = 500;
};

class UARTReceiver {
public:
    struct BallTargetData {
        float theta_rad = 0.0f;
        float distance_m = 0.0f;
        bool detected = false;
        uint8_t sequence = 0;
    };

    explicit UARTReceiver(const UARTConfig& config) : cfg_(config) {}

    bool start() {
        if (crc16(reinterpret_cast<const uint8_t*>("123456789"), 9) != 0x29B1) {
            ROS_ERROR("CRC-16/CCITT-FALSE self-check failed");
            return false;
        }

        const int opened_fd = ::open(cfg_.port.c_str(), O_RDONLY | O_NOCTTY);
        if (opened_fd < 0) {
            ROS_ERROR("UART port open failed: %s (errno=%d)", cfg_.port.c_str(), errno);
            return false;
        }

        termios tty{};
        if (tcgetattr(opened_fd, &tty) != 0) {
            ROS_ERROR("tcgetattr failed (errno=%d)", errno);
            ::close(opened_fd);
            return false;
        }

        cfmakeraw(&tty);
        cfsetispeed(&tty, cfg_.baudrate);
        cfsetospeed(&tty, cfg_.baudrate);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
        tty.c_cflag |= CS8;
#ifdef CRTSCTS
        tty.c_cflag &= ~CRTSCTS;
#endif
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 2;

        if (tcsetattr(opened_fd, TCSANOW, &tty) != 0) {
            ROS_ERROR("tcsetattr failed (errno=%d)", errno);
            ::close(opened_fd);
            return false;
        }

        fd_.store(opened_fd, std::memory_order_release);
        running_.store(true, std::memory_order_release);
        worker_ = std::thread(&UARTReceiver::readLoop, this);
        return true;
    }

    BallTargetData getLatest(bool& stale) const {
        std::lock_guard<std::mutex> lock(state_mutex_);
        stale = !running_.load(std::memory_order_acquire) ||
                fd_.load(std::memory_order_acquire) < 0 ||
                !has_valid_frame_ ||
                (nowMs() - last_update_ms_) >= cfg_.timeout_ms;
        return latest_;
    }

    void stop() {
        running_.store(false, std::memory_order_release);
        if (worker_.joinable()) {
            worker_.join();
        }

        const int old_fd = fd_.exchange(-1, std::memory_order_acq_rel);
        if (old_fd >= 0) {
            ::close(old_fd);
        }
    }

private:
    static constexpr uint8_t SOF0 = 0xAA;
    static constexpr uint8_t SOF1 = 0x55;
    static constexpr uint8_t VERSION = 0x01;
    static constexpr size_t FRAME_SIZE = 15;
    static constexpr size_t CRC_INPUT_OFFSET = 2;
    static constexpr size_t CRC_INPUT_SIZE = 11;

    void readLoop() {
        uint8_t byte = 0;
        while (running_.load(std::memory_order_acquire) && ros::ok()) {
            const int descriptor = fd_.load(std::memory_order_acquire);
            if (descriptor < 0) break;

            const ssize_t count = ::read(descriptor, &byte, 1);
            if (count == 0) continue;
            if (count < 0) {
                if (errno == EINTR) continue;
                handleReadError();
                return;
            }
            consumeByte(byte);
        }
    }

    void consumeByte(uint8_t byte) {
        if (frame_size_ == 0) {
            if (byte == SOF0) frame_buffer_[frame_size_++] = byte;
            return;
        }

        if (frame_size_ == 1) {
            if (byte == SOF1) {
                frame_buffer_[frame_size_++] = byte;
            } else if (byte == SOF0) {
                frame_buffer_[0] = byte;
            } else {
                frame_size_ = 0;
            }
            return;
        }

        frame_buffer_[frame_size_++] = byte;
        if (frame_size_ == FRAME_SIZE) {
            if (processFrame()) {
                frame_size_ = 0;
            } else {
                resynchronize();
            }
        }
    }

    bool processFrame() {
        if (frame_buffer_[0] != SOF0 || frame_buffer_[1] != SOF1) return false;

        if (frame_buffer_[2] != VERSION) {
            ROS_WARN("Unsupported UART version: 0x%02X", frame_buffer_[2]);
            return false;
        }

        const uint16_t received_crc =
            static_cast<uint16_t>(frame_buffer_[13]) |
            (static_cast<uint16_t>(frame_buffer_[14]) << 8);
        const uint16_t calculated_crc =
            crc16(frame_buffer_.data() + CRC_INPUT_OFFSET, CRC_INPUT_SIZE);
        if (received_crc != calculated_crc) {
            ROS_WARN("CRC error detected");
            return false;
        }

        const uint8_t flags = frame_buffer_[12];
        if ((flags & 0xFE) != 0) {
            ROS_WARN("UART frame has reserved flag bits set: 0x%02X", flags);
            return false;
        }

        const float theta_rad = decodeFloat32LE(frame_buffer_.data() + 4);
        const float distance_m = decodeFloat32LE(frame_buffer_.data() + 8);
        const bool detected = (flags & 0x01) != 0;
        const float pi_f32 = static_cast<float>(3.14159265358979323846);

        if (!std::isfinite(theta_rad) || theta_rad <= -pi_f32 ||
            theta_rad > pi_f32 || !std::isfinite(distance_m) ||
            distance_m < 0.0f ||
            (!detected && (theta_rad != 0.0f || distance_m != 0.0f))) {
            ROS_WARN("Invalid UART values: theta=%g distance=%g detected=%d",
                     theta_rad, distance_m, detected ? 1 : 0);
            return false;
        }

        const uint8_t sequence = frame_buffer_[3];

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            latest_ = {theta_rad, distance_m, detected, sequence};
            last_update_ms_ = nowMs();
            has_valid_frame_ = true;
        }
        return true;
    }

    void resynchronize() {
        for (size_t i = 1; i + 1 < FRAME_SIZE; ++i) {
            if (frame_buffer_[i] == SOF0 && frame_buffer_[i + 1] == SOF1) {
                const size_t remaining = FRAME_SIZE - i;
                std::memmove(frame_buffer_.data(), frame_buffer_.data() + i, remaining);
                frame_size_ = remaining;
                return;
            }
        }

        if (frame_buffer_[FRAME_SIZE - 1] == SOF0) {
            frame_buffer_[0] = SOF0;
            frame_size_ = 1;
        } else {
            frame_size_ = 0;
        }
    }

    void handleReadError() {
        ROS_ERROR("UART read error: %s", std::strerror(errno));
        running_.store(false, std::memory_order_release);
    }

    static float decodeFloat32LE(const uint8_t* bytes) {
        const uint32_t bits =
            static_cast<uint32_t>(bytes[0]) |
            (static_cast<uint32_t>(bytes[1]) << 8) |
            (static_cast<uint32_t>(bytes[2]) << 16) |
            (static_cast<uint32_t>(bytes[3]) << 24);
        float value = 0.0f;
        std::memcpy(&value, &bits, sizeof(value));
        return value;
    }

    static uint16_t crc16(const uint8_t* data, size_t length) {
        uint16_t crc = 0xFFFF;
        for (size_t i = 0; i < length; ++i) {
            crc ^= static_cast<uint16_t>(data[i]) << 8;
            for (int bit = 0; bit < 8; ++bit) {
                crc = (crc & 0x8000)
                          ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                          : static_cast<uint16_t>(crc << 1);
            }
        }
        return crc;
    }

    static int64_t nowMs() {
        using namespace std::chrono;
        return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
    }

    UARTConfig cfg_;
    std::atomic<int> fd_{-1};
    std::atomic<bool> running_{false};
    std::thread worker_;

    mutable std::mutex state_mutex_;
    BallTargetData latest_;
    int64_t last_update_ms_ = 0;
    bool has_valid_frame_ = false;

    std::array<uint8_t, FRAME_SIZE> frame_buffer_{};
    size_t frame_size_ = 0;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "uart_receiver_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    UARTConfig cfg;
    pnh.param<std::string>("port", cfg.port, "/dev/serial0");
    pnh.param<int>("timeout_ms", cfg.timeout_ms, 500);

    ros::Publisher pub = nh.advertise<innurobot_uart::BallTarget>("/ball_target", 10);

    UARTReceiver receiver(cfg);
    if (!receiver.start()) {
        ROS_ERROR("Failed to start UART Receiver");
        return 1;
    }

    ROS_INFO("UART Receiver Node started: port=%s", cfg.port.c_str());

    // 40Hz でトピックをパブリッシュするループ
    ros::Rate rate(40);
    while (ros::ok()) {
        bool stale = false;
        UARTReceiver::BallTargetData target = receiver.getLatest(stale);

        innurobot_uart::BallTarget msg;
        msg.theta_rad = target.theta_rad;
        msg.distance_m = target.distance_m;
        msg.detected = target.detected;
        msg.sequence = target.sequence;
        msg.stale = stale;

        pub.publish(msg);

        ros::spinOnce();
        rate.sleep();
    }

    receiver.stop();
    return 0;
}
